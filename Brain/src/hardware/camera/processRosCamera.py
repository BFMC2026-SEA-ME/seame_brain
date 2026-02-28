# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.
# (BSD-3 Clause)

"""ROS2 RealSense 카메라 프로세스 (구독 전용)

- realsense2_camera 노드는 외부에서 따로 실행한다고 가정
- 이 프로세스는 CompressedImage 토픽만 구독해서 dashboard로 전달
"""

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import time
import os

from src.templates.workerprocess import WorkerProcess
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode
from src.hardware.camera.threads.threadRosCamera import RosCameraThread

# ============================ CAMERA CONFIG ============================
# Choose one of the available topics.
# Default: compressed stream for low CPU usage.
ROS_CAMERA_TOPIC = os.getenv(
    "ROS_CAMERA_TOPIC",
    "/d455f/d455f/color/image_raw/compressed",
)

# Max FPS to push into the system. Lower = less load / less queue pressure.
ROS_CAMERA_MAX_FPS = float(os.getenv("ROS_CAMERA_MAX_FPS", "10"))

# Keepalive resend interval (only used when no new frames are coming).
ROS_CAMERA_KEEPALIVE_SEC = float(os.getenv("ROS_CAMERA_KEEPALIVE_SEC", "1.0"))

# If True and using /compressed topics, forward bytes as-is (no decode/resize).
# This minimizes CPU and prevents queue buildup from expensive re-encoding.
# Set to 0 when you want to downscale here for lower bandwidth.
ROS_CAMERA_PASSTHROUGH = os.getenv("ROS_CAMERA_PASSTHROUGH", "0") == "1"

# Downscale size when passthrough is off. Format: "WIDTHxHEIGHT".
# Example: 320x180
_downscale_env = os.getenv("ROS_CAMERA_DOWNSCALE", "320x180")
_downscale_size = None
try:
    _w, _h = _downscale_env.lower().split("x", 1)
    _downscale_size = (int(_w), int(_h))
except Exception:
    _downscale_size = None


class processRosCamera(WorkerProcess):
    """RealSense ROS 카메라 프로세스 (구독 전용)."""

    def __init__(self, queueList, logging, ready_event=None, debugging: bool = False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        self.stateChangeSubscriber = messageHandlerSubscriber(
            self.queuesList, StateChange, "lastOnly", True
        )
        super(processRosCamera, self).__init__(self.queuesList, ready_event)

    def _init_threads(self):
        min_frame_interval = 0.0
        if ROS_CAMERA_MAX_FPS > 0:
            min_frame_interval = 1.0 / ROS_CAMERA_MAX_FPS

        cam_thread = RosCameraThread(
            self.queuesList,
            self.logging,
            debugging=self.debugging,
            topic_name=ROS_CAMERA_TOPIC,
            keepalive_sec=ROS_CAMERA_KEEPALIVE_SEC,
            min_frame_interval=min_frame_interval,
            init_retry_sec=1.0,
            passthrough_compressed=ROS_CAMERA_PASSTHROUGH,
            downscale_size=None if ROS_CAMERA_PASSTHROUGH else _downscale_size,
            jpeg_quality=60,
        )
        self.threads.append(cam_thread)

    def state_change_handler(self):
        message = self.stateChangeSubscriber.receive()
        if message is not None:
            modeDict = SystemMode[message].value["camera"]["process"]
            if modeDict.get("enabled", True):
                self.resume_threads()
            else:
                self.pause_threads()


if __name__ == "__main__":
    from multiprocessing import Queue
    import logging

    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
        "Image": Queue(maxsize=1),
    }

    logger = logging.getLogger()
    logging.basicConfig(level=logging.INFO)

    process = processRosCamera(queueList, logger, debugging=True)
    process.daemon = True
    process.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        process.stop()
