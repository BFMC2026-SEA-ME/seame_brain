# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
# (BSD-3 Clause)

import base64
import time
from typing import Optional

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import serialCamera, StateChange
from src.statemachine.systemMode import SystemMode

from rclpy.qos import qos_profile_sensor_data


class RosCameraThread(ThreadWithStop):
    """ROS2 CompressedImage 구독 스레드 (구독 전용)

    - realsense2_camera 노드는 외부에서 따로 실행한다고 가정
      예) ros2 run realsense2_camera realsense2_camera_node
    - /.../compressed 토픽을 받아 base64로 인코딩하여 serialCamera로 전송
    - keepalive로 마지막 프레임을 주기적으로 재전송 (프론트 끊김 방지)
    """

    def __init__(
        self,
        queuesList,
        logger,
        debugging: bool = False,
        topic_name: str = "/camera/camera/color/image_raw/compressed",
        keepalive_sec: float = 0.5,
        min_frame_interval: float = 0.1,  # 10fps (필요시 0.05=20fps)
        init_retry_sec: float = 1.0,      # 토픽/ROS 준비 안됐을 때 재시도 간격
        node_name: str = "ros_camera_bridge",
    ):
        super(RosCameraThread, self).__init__(pause=0.01)

        self.queuesList = queuesList
        self.logger = logger
        self.debugging = debugging

        self.topic_name = topic_name
        self.keepalive_sec = keepalive_sec
        self.min_frame_interval = min_frame_interval
        self.init_retry_sec = init_retry_sec
        self.node_name = node_name

        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, "lastOnly", True
        )

        self._ros_import_error = False
        self._rclpy = None
        self._executor = None
        self._node = None

        self._last_payload: Optional[str] = None
        self._last_emit_ts: float = 0.0
        self._last_send_ts: float = 0.0
        self._last_init_try_ts: float = 0.0

    # ================================ STATE CHANGE ====================================
    def state_change_handler(self):
        """대시보드 모드 변경 시 pause/resume 연동."""
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value.get("camera", {}).get("thread", {})
            if modeDict.get("enabled", True):
                self.resume()
            else:
                self.pause()

    # ================================ RUN ============================================
    def thread_work(self):
        # 1) ROS Node 없으면 초기화 시도 (realsense 노드 외부 실행)
        if self._node is None:
            now = time.time()
            if now - self._last_init_try_ts >= self.init_retry_sec:
                self._last_init_try_ts = now
                self._maybe_init_ros()
            time.sleep(0.05)
            return

        # 2) rclpy 컨텍스트 죽었으면 리셋 후 재시도
        if self._rclpy is not None and not self._rclpy.ok():
            self._reset_ros_context()
            time.sleep(0.1)
            return

        # 3) executor spin
        try:
            if self._executor is not None:
                self._executor.spin_once(timeout_sec=0.02)
        except Exception as exc:
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - spin_once failed: {exc}")
            self._reset_ros_context()
            time.sleep(0.1)
            return

        # 4) keepalive: 마지막 프레임 재전송
        now = time.time()
        if self._last_payload is not None and now - self._last_emit_ts >= self.keepalive_sec:
            self.serialCameraSender.send(self._last_payload)
            self._last_emit_ts = now

    # ================================ STOP ============================================
    def stop(self):
        self._reset_ros_context()
        super(RosCameraThread, self).stop()

    # ================================ INTERNALS =======================================
    def _maybe_init_ros(self):
        """ROS2 초기화 및 구독 설정."""
        if self._node is not None or self._ros_import_error:
            return

        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from sensor_msgs.msg import CompressedImage
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        except ImportError as exc:
            if not self._ros_import_error:
                print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - ROS2 imports failed: {exc}")
            self._ros_import_error = True
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            qos_profile_sensor_data = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )

            outer_self = self
            serial_sender = self.serialCameraSender

            class RosImageBridgeNode(Node):
                def __init__(self, topic: str):
                    super().__init__(outer_self.node_name)
                    self.subscription = self.create_subscription(
                        CompressedImage,
                        topic,
                        self.listener_callback,
                        qos_profile_sensor_data,
                    )
                    self.get_logger().info(f"Subscribed to {topic} (qos_profile_sensor_data)")

                def listener_callback(self, msg):
                    try:
                        now_ts = time.time()

                        # FPS 제한: 너무 많이 보내면 대시보드/큐가 밀릴 수 있음
                        if now_ts - outer_self._last_send_ts < outer_self.min_frame_interval:
                            return

                        # msg.data: JPEG/PNG 바이트 → 그대로 전달 (base64 불필요)
                        payload = bytes(msg.data)

                        serial_sender.send(payload)

                        outer_self._last_payload = payload
                        outer_self._last_emit_ts = now_ts
                        outer_self._last_send_ts = now_ts

                    except Exception as exc2:
                        self.get_logger().error(f"Image callback failed: {exc2}")

            self._rclpy = rclpy
            self._node = RosImageBridgeNode(self.topic_name)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)

            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;92mINFO\033[0m - Ready. Listening: {self.topic_name}")

        except Exception as exc:
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - Failed to init ROS2: {exc}")
            self._reset_ros_context()

    def _reset_ros_context(self):
        """Clean up ROS2 executor/node/context so we can re-init on next loop."""
        try:
            if self._executor and self._node:
                try:
                    self._executor.remove_node(self._node)
                except Exception:
                    pass
                try:
                    self._executor.shutdown()
                except Exception:
                    pass
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
        except Exception:
            pass

        try:
            if self._rclpy is not None and self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

        self._executor = None
        self._node = None
        self._rclpy = None
        self._last_payload = None
        self._last_emit_ts = 0.0
        self._last_send_ts = 0.0
