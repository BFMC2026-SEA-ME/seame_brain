# this code is for ackermann process
# have to install ackermann_msgs package first
# ex) sudo apt install ros-humble-ackermann-msgs
"""ROS2 bridge that forwards Ackermann commands to the BFMC serial pipeline.

When AUTO mode is activated on the dashboard, this node reads
ackermann_msgs/AckermannDriveStamped messages (default: `/ackermann_cmd`)
and converts them to the SpeedMotor/SteerMotor queue updates consumed by `threadWrite`.
"""
# speed : 값 그대로
# steer : -0.436 ~ 0.436(rad)(25도정도 ) + 우회전, -좌회전

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Mapping, MutableMapping, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# Ackermann message
from ackermann_msgs.msg import AckermannDriveStamped

# Enable imports of BFMC frameworks.
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # .../ros2_ws/src/Brain
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.templates.threadwithstop import ThreadWithStop  # type: ignore
from src.utils.messages.allMessages import DrivingMode, SpeedMotor, SteerMotor  # type: ignore
from src.utils.messages.messageHandlerSender import messageHandlerSender  # type: ignore
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber  # type: ignore


class AckermannBridgeNode(Node):
    """Forward Ackermann commands to the serial handler when AUTO mode is active."""

    def __init__(self, queues_list: Mapping[str, object]) -> None:
        # ROS2 노드 초기화, 파라미터/QoS/구독 설정.
        super().__init__("ackermann_bridge")
        self._queues_list = queues_list
        self._auto_active = False
        self._pending_cmd: Optional[tuple[str, str]] = None

        # BFMC message handlers
        self._driving_mode_subscriber = messageHandlerSubscriber(
            self._queues_list, DrivingMode, "lastOnly", True
        )

        self._speed_sender = messageHandlerSender(self._queues_list, SpeedMotor)
        self._steer_sender = messageHandlerSender(self._queues_list, SteerMotor)

        # ---------------- Parameters ----------------
        # Topic
        self.declare_parameter("ackermann_topic", "/ackermann_cmd")

        # Conversion / limits
        self.declare_parameter("speed_scale", 10.0)      # m/s -> motor cmd
        self.declare_parameter("steer_scale", 10.0)     # rad(or deg) -> motor cmd
        self.declare_parameter("steer_limit", 250)
        self.declare_parameter("steer_invert", False)    # 되도록 False 유지
        self.declare_parameter("steer_use_degrees", True)  # True면 rad->deg 후 scale ; rad 값을 deg로 변환후 스케일

        self._ackermann_topic = str(self.get_parameter("ackermann_topic").value)
        self._speed_scale = float(self.get_parameter("speed_scale").value)
        self._steer_scale = float(self.get_parameter("steer_scale").value)
        self._steer_limit = int(self.get_parameter("steer_limit").value)
        self._steer_invert = bool(self.get_parameter("steer_invert").value)
        self._steer_use_degrees = bool(self.get_parameter("steer_use_degrees").value)

        # ---------------- QoS ----------------
        # 최신 명령 1개만 유지
        ack_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )


        self._ack_sub = self.create_subscription(
            AckermannDriveStamped,
            self._ackermann_topic,
            self._handle_ackermann,
            ack_qos,
        )

        # Driving mode polling (50 Hz)
        self._mode_poll_timer = self.create_timer(0.02, self._poll_driving_mode)

        # Ackermann 출력은 30 Hz로 제한
        self._send_period = 1.0 / 30.0
        self._send_timer = self.create_timer(self._send_period, self._publish_pending_cmd)

        self.get_logger().info(
            f"Ackermann bridge subscribed: {self._ackermann_topic} (AUTO only)"
        )

    # ------------------------------------------------------------------ callbacks --
    def _poll_driving_mode(self) -> None:
        # DrivingMode를 폴링해서 AUTO 게이트를 켜고 끕니다.
        mode = self._driving_mode_subscriber.receive()
        if mode is None:
            return

        mode_lower = mode.lower()
        was_auto = self._auto_active
        if mode_lower == "auto":
            if not was_auto:
                self.get_logger().info("AUTO mode activated – Ackermann bridge enabled.")
            self._auto_active = True
            return

        if was_auto:
            self.get_logger().info(f"AUTO mode exit ({mode_lower}); bridge paused.")
        self._auto_active = False

        if was_auto or mode_lower == "stop":
            self._send_stop_command()

    def _handle_ackermann(self, msg: AckermannDriveStamped) -> None:
        # AUTO 상태에서 AckermannDriveStamped를 Speed/Steer로 변환합니다.
        if not self._auto_active:
            return

        # AckermannDriveStamped:
        #   msg.drive.speed           [cm/s]
        #   msg.drive.steering_angle  [rad]  
        speed_cmd = self._scale_speed(msg.drive.speed)
        steer_cmd = self._scale_steer(msg.drive.steering_angle)

        # 30 Hz 제한을 위해 마지막 명령만 저장
        self._pending_cmd = (speed_cmd, steer_cmd)

        # self.get_logger().debug(
        #     f"{self._ackermann_topic} -> speed={msg.drive.speed:.3f} "
        #     f"steer(rad)={msg.drive.steering_angle:.3f} => "
        #     f"motor speed={speed_cmd} steer={steer_cmd}"
        # )

    # ------------------------------------------------------------------ scaling --
    def _scale_speed(self, speed_mps: float) -> str:
        # 선속도(m/s)를 모터 명령 문자열로 스케일링합니다.
        scaled = int(speed_mps * self._speed_scale) # *10
        return str(scaled)

    def _scale_steer(self, steering_angle: float) -> str:
        # 조향각(rad/deg)을 모터 명령 문자열로 스케일링합니다.
        # ROS Ackermann convention: + = left
        # Hardware: + = right  => flip sign once here
        steer_value = steering_angle # +일때 오른쪽, -일때 왼쪽으로 가도록 (하드웨어에 맞게)

        # Optional: convert to degrees before scaling
        if self._steer_use_degrees:
            steer_value = math.degrees(steer_value) # rad to deg

        steer_value *= self._steer_scale # deg * 10.0

        # Avoid double inversion: keep this False if you already flipped above
        if self._steer_invert:
            steer_value = -steer_value

        if self._steer_limit > 0:
            steer_value = max(-self._steer_limit, min(self._steer_limit, steer_value))

        return str(int(round(steer_value)))

    def _publish_pending_cmd(self) -> None:
        """Send latest command at 30 Hz max."""
        if not self._auto_active:
            self._pending_cmd = None
            return
        if self._pending_cmd is None:
            return
        speed_cmd, steer_cmd = self._pending_cmd
        self._speed_sender.send(speed_cmd)
        self._steer_sender.send(steer_cmd)
        self._pending_cmd = None

    def _send_stop_command(self) -> None:
        """Send a single stop command and clear pending."""
        self._speed_sender.send("0")
        self._steer_sender.send("0")
        self._pending_cmd = None


class _AckermannBridgeThread(ThreadWithStop):
    """Spin the ROS2 node in a BFMC-friendly thread (pause/resume/stop)."""

    def __init__(self, queues_list: Mapping[str, object]) -> None:
        # ROS executor/node를 위한 스레드 상태를 준비합니다.
        super().__init__(pause=0.005)
        self._queues_list = queues_list
        self._executor: Optional[SingleThreadedExecutor] = None
        self._node: Optional[AckermannBridgeNode] = None

    def thread_work(self) -> None:
        # 주기마다 executor를 한 번씩 spin; 실패 시 재초기화합니다.
        if self._node is None or self._executor is None:
            self._maybe_init_ros()
            time.sleep(0.05)
            return

        try:
            self._executor.spin_once(timeout_sec=0.005)
        except Exception as exc:
            print(f"[AckermannBridge] spin_once failed: {exc}")
            self._reset_ros()
            time.sleep(0.1)

    def stop(self) -> None:
        # 스레드를 중지하고 ROS 리소스를 정리합니다.
        self._reset_ros()
        super().stop()

    def _maybe_init_ros(self) -> None:
        # 필요 시 rclpy/node/executor를 초기화합니다.
        if self._node is not None:
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self._node = AckermannBridgeNode(self._queues_list)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            print(f"[AckermannBridge] init failed: {exc}")
            self._reset_ros()

    def _reset_ros(self) -> None:
        # executor/node/rclpy 컨텍스트를 안전하게 종료합니다.
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


def create_ackermann_bridge_process(
    queue_list: MutableMapping[str, object], ready_event=None
):
    """Factory compatible with WorkerProcess usage in main.py."""
    from src.templates.workerprocess import WorkerProcess  # type: ignore

    class AckermannBridgeProcess(WorkerProcess):
        def _init_threads(self):
            # ROS2 브릿지 스레드를 등록합니다.
            self.threads.append(_AckermannBridgeThread(self.queuesList))

    return AckermannBridgeProcess(queue_list, ready_event=ready_event, daemon=True)


if __name__ == "__main__":
    from multiprocessing import Queue

    queue_list: MutableMapping[str, object] = {
        "General": Queue(),
        "Config": Queue(),
    }

    rclpy.init(args=None)
    node = AckermannBridgeNode(queue_list)

    try:
        for _ in range(200):
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()
