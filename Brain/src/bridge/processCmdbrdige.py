"""ROS2 bridge that forwards /cmd_vel commands to the BFMC serial pipeline.

When AUTO mode is activated on the dashboard, this node reads geometry_msgs/Twist
messages from `/cmd_vel` and converts them to the SpeedMotor/SteerMotor queue
updates consumed by `threadWrite`.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Mapping, MutableMapping, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Enable imports of BFMC frameworks.
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # .../ros2_ws/src/Brain
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.templates.threadwithstop import ThreadWithStop  # type: ignore
from src.utils.messages.allMessages import DrivingMode, SpeedMotor, SteerMotor  # type: ignore
from src.utils.messages.messageHandlerSender import messageHandlerSender  # type: ignore
from src.utils.messages.messageHandlerSubscriber import (  # type: ignore
    messageHandlerSubscriber,
)


class CmdVelBridgeNode(Node):
    """Forward `/cmd_vel` to the serial handler when AUTO mode is active."""

    def __init__(self, queues_list: Mapping[str, object]) -> None:
        super().__init__("cmd_vel_bridge")
        self._queues_list = queues_list
        self._auto_active = False

        # BFMC message handlers
        self._driving_mode_subscriber = messageHandlerSubscriber(
            self._queues_list, DrivingMode, "lastOnly", True
        )

        self._speed_sender = messageHandlerSender(self._queues_list, SpeedMotor)
        self._steer_sender = messageHandlerSender(self._queues_list, SteerMotor)

        # Allow quick tuning from launch files / CLI.
        self.declare_parameter("speed_scale", 10.0)
        self.declare_parameter("steer_scale", 250.0)
        self.declare_parameter("steer_limit", 250)
        self.declare_parameter("steer_invert", False)  # 향후 사용 안 함
        self.declare_parameter("steer_use_degrees", False)

        self._speed_scale = float(self.get_parameter("speed_scale").value)
        self._steer_scale = float(self.get_parameter("steer_scale").value)
        self._steer_limit = int(self.get_parameter("steer_limit").value)
        self._steer_invert = bool(self.get_parameter("steer_invert").value)
        self._steer_use_degrees = bool(self.get_parameter("steer_use_degrees").value)

        # ---------------- QoS 설정 ----------------
        # 항상 "가장 최신" /cmd_vel 하나만 유지하도록 KEEP_LAST(1) 사용.
        # RELIABLE로 두어서 ControlNode의 기본 퍼블리셔(RELIABLE)와 호환.
        cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,  # 최신 명령 1개만 유지 (밀린 조향값 순차 처리 방지)
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self._cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._handle_cmd_vel,
            cmd_vel_qos,
        )

        # Driving mode polling
        # 50 Hz로 모드 상태를 확인해 첫 명령 전송 지연을 줄임 (기존 0.1 s)
        self._mode_poll_timer = self.create_timer(0.02, self._poll_driving_mode)

    # ------------------------------------------------------------------ callbacks --
    def _poll_driving_mode(self) -> None:
        mode = self._driving_mode_subscriber.receive()
        if mode is None:
            return

        mode_lower = mode.lower()
        if mode_lower == "auto" and not self._auto_active:
            self.get_logger().info("AUTO mode activated – /cmd_vel bridge enabled.")
            self._auto_active = True
        elif mode_lower != "auto" and self._auto_active:
            self.get_logger().info(f"AUTO mode exit ({mode_lower}); bridge paused.")
            self._auto_active = False

    def _handle_cmd_vel(self, msg: Twist) -> None:
        if not self._auto_active:
            return

        speed_cmd = self._scale_speed(msg.linear.x)
        steer_cmd = self._scale_steer(msg.angular.z)

        if speed_cmd is not None:
            self._speed_sender.send(speed_cmd)
        if steer_cmd is not None:
            self._steer_sender.send(steer_cmd)

        self.get_logger().debug(f"/cmd_vel -> speed={speed_cmd} steer={steer_cmd}")

    def _scale_speed(self, linear_x: float) -> str:
        scaled = int(linear_x * self._speed_scale)
        return str(scaled)

    def _scale_steer(self, angular_z: float) -> str | None:
        # ROS 표준: +z = 좌회전
        # 하드웨어: +가 우회전이므로 여기서 부호 한 번 뒤집어줌
        steer_value = -angular_z

        if self._steer_use_degrees:
            steer_value = math.degrees(steer_value)

        steer_value *= self._steer_scale

        # ⚠️ 이걸 쓰면 두 번 뒤집힐 수 있으니,
        #     앞으로 steer_invert 파라미터는 항상 False로 두는 걸 전제로.
        if self._steer_invert:
            steer_value = -steer_value

        if self._steer_limit > 0:
            steer_value = max(-self._steer_limit, min(self._steer_limit, steer_value))

        return str(int(round(steer_value)))


class _CmdVelBridgeThread(ThreadWithStop):
    """Spin the ROS2 node in a BFMC-friendly thread (pause/resume/stop)."""

    def __init__(self, queues_list: Mapping[str, object]) -> None:
        super().__init__(pause=0.005)  # match spin timeout
        self._queues_list = queues_list
        self._executor: Optional[SingleThreadedExecutor] = None
        self._node: Optional[CmdVelBridgeNode] = None

    def thread_work(self) -> None:
        # Lazily initialize ROS context and node.
        if self._node is None or self._executor is None:
            self._maybe_init_ros()
            time.sleep(0.05)
            return

        try:
            self._executor.spin_once(timeout_sec=0.005)
        except Exception as exc:  # keep bridge alive after transient failures
            print(f"[CmdVelBridge] spin_once failed: {exc}")
            self._reset_ros()
            time.sleep(0.1)

    def stop(self) -> None:
        self._reset_ros()
        super().stop()

    def _maybe_init_ros(self) -> None:
        if self._node is not None:
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = CmdVelBridgeNode(self._queues_list)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            print(f"[CmdVelBridge] init failed: {exc}")
            self._reset_ros()

    def _reset_ros(self) -> None:
        if self._executor and self._node:
            try:
                self._executor.remove_node(self._node)
            except Exception:
                pass
            try:
                self._executor.shutdown()
            except Exception:
                pass
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        self._executor = None
        self._node = None


def create_cmd_vel_bridge_process(
    queue_list: MutableMapping[str, object], ready_event=None
):
    """Factory compatible with WorkerProcess usage in main.py."""
    from src.templates.workerprocess import WorkerProcess  # type: ignore

    class CmdVelBridgeProcess(WorkerProcess):
        def _init_threads(self):
            self.threads.append(_CmdVelBridgeThread(self.queuesList))

    return CmdVelBridgeProcess(queue_list, ready_event=ready_event, daemon=True)


if __name__ == "__main__":
    from multiprocessing import Queue
    import time

    queue_list: MutableMapping[str, object] = {
        "General": Queue(),
        "Config": Queue(),
    }

    rclpy.init(args=None)
    node = CmdVelBridgeNode(queue_list)

    try:
        # 테스트용 메인: 여기서도 spin 주기를 0.02로 맞춰줌
        for _ in range(200):
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
