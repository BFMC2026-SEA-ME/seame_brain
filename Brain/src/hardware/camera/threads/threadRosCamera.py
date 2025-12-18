# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import base64
import os
import shlex
import subprocess
import time
from typing import Optional

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import serialCamera, StateChange
from src.statemachine.systemMode import SystemMode


class RosCameraThread(ThreadWithStop):
    """ROS2 RealSense 이미지 구독 스레드.

    - 필요 시 `realsense2_camera` 노드를 서브프로세스로 실행
    - ROS2 압축 이미지 토픽(CompressedImage)을 받아 base64 후 `serialCamera` 채널로 전송
    - 최근 프레임을 주기적으로 재전송해 프런트 로딩 끊김을 줄임
    """

    def __init__(
        self,
        queuesList,
        logger,
        debugging: bool = False,
        topic_name: str = "/camera/camera/color/image_raw/compressed",
        realsense_cmd: Optional[str] = None,
        keepalive_sec: float = 0.3,
        min_frame_interval: float = 0.1,  # fps cap to prevent backlog
    ):
        super(RosCameraThread, self).__init__(pause=0.01)
        self.queuesList = queuesList
        self.logger = logger
        self.debugging = debugging
        self.topic_name = topic_name
        self.keepalive_sec = keepalive_sec
        # 최소 프레임 간격(초). 너무 많이 보내면 큐가 밀려 지연이 발생하므로 속도 제한.
        self.min_frame_interval = min_frame_interval

        self.serialCameraSender = messageHandlerSender(self.queuesList, serialCamera)
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, "lastOnly", True
        )

        self.realsense_cmd = (
            realsense_cmd
            or os.environ.get("REAL_SENSE_CMD", "ros2 run realsense2_camera realsense2_camera_node")
        )
        self.realsense_proc: Optional[subprocess.Popen] = None

        self._ros_import_error = False
        self._rclpy = None
        self._executor = None
        self._node = None

        self._last_payload: Optional[str] = None
        self._last_emit_ts: float = 0.0
        self._last_send_ts: float = 0.0

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
        if self._node is None:
            self._maybe_start_realsense_node()
            self._maybe_init_ros()
            time.sleep(0.1)
            return

        # rclpy 컨텍스트가 내려갔으면 재초기화 시도
        if self._rclpy is not None and not self._rclpy.ok():
            self._reset_ros_context()
            time.sleep(0.05)
            return

        try:
            if self._executor is not None:
                self._executor.spin_once(timeout_sec=0.05)
        except Exception as exc:
            # 컨텍스트가 이미 shutdown된 경우 반복 에러를 막기 위해 이후 spin을 건너뜀
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - spin_once failed: {exc}")
            self._reset_ros_context()
            time.sleep(0.1)
            return

        now = time.time()
        if self._last_payload is not None and now - self._last_emit_ts >= self.keepalive_sec:
            self.serialCameraSender.send(self._last_payload)
            self._last_emit_ts = now

    # ================================ STOP ============================================
    def stop(self):
        self._reset_ros_context()

        if self.realsense_proc is not None:
            self.realsense_proc.terminate()
            try:
                self.realsense_proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.realsense_proc.kill()
        super(RosCameraThread, self).stop()

    # ================================ INTERNALS =======================================
    def _maybe_start_realsense_node(self):
        """필요 시 RealSense ROS 노드를 띄움."""
        if self.realsense_proc is not None or not self.realsense_cmd:
            return
        try:
            args = shlex.split(self.realsense_cmd)
            self.realsense_proc = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;92mINFO\033[0m - Started RealSense node: {self.realsense_cmd}")
        except FileNotFoundError:
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - Command not found: {self.realsense_cmd}")
        except Exception as exc:
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - Failed to start RealSense node: {exc}")

    def _maybe_init_ros(self):
        """ROS2 초기화 및 구독 설정."""
        if self._node is not None or self._ros_import_error:
            return

        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from sensor_msgs.msg import CompressedImage
        except ImportError as exc:
            if not self._ros_import_error:
                print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - ROS2 imports failed: {exc}")
            self._ros_import_error = True
            return

        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            serial_sender = self.serialCameraSender
            last_payload_ref = self
            outer_self = self  # alias for inner class access

            class RosImageBridgeNode(Node):
                def __init__(self, topic: str):
                    super().__init__("ros_camera_bridge")
                    self.subscription = self.create_subscription(
                        CompressedImage, topic, self.listener_callback, 10
                    )

                def listener_callback(self, msg):
                    try:
                        now_ts = time.time()
                        # FPS 제한: min_frame_interval보다 빠르면 드롭 (과도한 적재 방지)
                        if now_ts - outer_self._last_send_ts < outer_self.min_frame_interval:
                            return

                        payload = base64.b64encode(bytes(msg.data)).decode("utf-8")
                        serial_sender.send(payload)
                        last_payload_ref._last_payload = payload
                        last_payload_ref._last_emit_ts = time.time()
                        last_payload_ref._last_send_ts = now_ts
                    except Exception as exc2:
                        self.get_logger().error(f"Image callback failed: {exc2}")

            self._rclpy = rclpy
            self._node = RosImageBridgeNode(self.topic_name)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;92mINFO\033[0m - Subscribed to {self.topic_name}")
        except Exception as exc:
            print(f"\033[1;97m[ RosCamera ] :\033[0m \033[1;91mERROR\033[0m - Failed to init ROS2: {exc}")

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
