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

from src.templates.workerprocess import WorkerProcess
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import StateChange
from src.statemachine.systemMode import SystemMode
from src.hardware.camera.threads.threadRosCamera import RosCameraThread


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
        cam_thread = RosCameraThread(
            self.queuesList,
            self.logging,
            debugging=self.debugging,
            topic_name="/camera/camera/color/image_raw/compressed",
            keepalive_sec=0.5,
            min_frame_interval=0.1,  # 10fps, 필요하면 0.05로
            init_retry_sec=1.0,
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
        "Image": Queue(),
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
