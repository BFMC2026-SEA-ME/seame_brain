# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
# All rights reserved.
# (license text omitted for brevity)

from __future__ import annotations

# ✅ IMPORTANT: fix sys.path BEFORE importing "src.*"
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import time
import json
import glob
import inspect
import argparse
from enum import Enum
from typing import Any, Optional

import psutil
import eventlet

# ✅ eventlet 기반 SocketIO면 monkey_patch가 안정적
eventlet.monkey_patch()

from flask import Flask, request
from flask_socketio import SocketIO
from flask_cors import CORS

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.templates.workerprocess import WorkerProcess
from src.utils.messages.allMessages import Semaphores
from src.dashboard.components.calibration import Calibration
from src.dashboard.components.ip_manger import IpManager
import src.utils.messages.allMessages as allMessages

# StateMachine은 dev 모드에서 없어도 돌아가게 처리
try:
    from src.statemachine.stateMachine import StateMachine
except Exception:
    StateMachine = None  # type: ignore


# ---------------- Jetson temperature helpers ----------------

def _read(p: str) -> Optional[str]:
    try:
        with open(p, "r") as f:
            return f.read().strip()
    except Exception:
        return None


def get_jetson_temps_c() -> dict[str, float]:
    temps: dict[str, float] = {}
    for z in sorted(glob.glob("/sys/devices/virtual/thermal/thermal_zone*")):
        name = _read(z + "/type")
        raw = _read(z + "/temp")
        if not name or not raw:
            continue
        try:
            temps[name] = float(raw) / 1000.0  # milli°C -> °C
        except ValueError:
            continue
    return temps


def get_jetson_cpu_temp_c() -> Optional[float]:
    temps = get_jetson_temps_c()
    return (temps.get("cpu-thermal")
            or temps.get("tj-thermal")
            or temps.get("soc0-thermal"))


def get_cpu_temp_generic() -> Optional[float]:
    """
    - Jetson/Linux: /sys thermal_zone 사용
    - 그 외: psutil.sensors_temperatures() 시도 (macOS는 대부분 None)
    """
    t = get_jetson_cpu_temp_c()
    if t is not None:
        return t

    try:
        temps = psutil.sensors_temperatures()  # type: ignore[attr-defined]
        if not temps:
            return None
        for _, entries in temps.items():
            if entries:
                return float(entries[0].current)
    except Exception:
        pass
    return None


# ---------------- Dev-mode stubs ----------------

class _DummyStateMachine:
    def request_mode(self, *_args: Any, **_kwargs: Any) -> None:
        return

    @staticmethod
    def get_instance() -> "_DummyStateMachine":
        return _DummyStateMachine()


class processDashboard(WorkerProcess):
    """
    Dashboard backend (Flask-SocketIO).
    - Brain(main.py)에서 multiprocessing 프로세스로 실행 가능
    - Mac에서 단독 실행(dev_mode) 가능
    """

    def __init__(
        self,
        queueList: Optional[dict[str, Any]],
        logging_obj: Any,
        ready_event=None,
        debugging: bool = False,
        dev_mode: bool = False,
        host: str = "0.0.0.0",
        port: int = 5005,
    ):
        self.running = True
        self.queueList = queueList
        self.logger = logging_obj
        self.debugging = debugging
        self.dev_mode = dev_mode
        self.host = host
        self.port = port

        # state machine
        if self.dev_mode or StateMachine is None:
            self.stateMachine = _DummyStateMachine.get_instance()
        else:
            self.stateMachine = StateMachine.get_instance()  # type: ignore[union-attr]

        # message handling
        self.messages: dict[str, dict[str, Any]] = {}
        self.sendMessages: dict[str, dict[str, Any]] = {}
        self.messagesAndVals: dict[str, dict[str, Any]] = {}

        # hardware monitoring
        self.memoryUsage: float = 0.0
        self.cpuCoreUsage: float = 0.0
        self.cpuTemperature: Optional[int] = None

        # heartbeat
        self.heartbeat_last_sent = time.time()
        self.heartbeat_retries = 0
        self.heartbeat_max_retries = 3
        self.heartbeat_time_between_heartbeats = 20
        self.heartbeat_time_between_retries = 5
        self.heartbeat_received = False

        # session management
        self.sessionActive = False
        self.activeUser = None

        # serial connection state
        self.serialConnected = False

        # configuration
        self.table_state_file = self._get_table_state_path()

        # ✅ Flask/SocketIO는 run()에서 생성
        self.app: Optional[Flask] = None
        self.socketio: Optional[SocketIO] = None
        self.calibration: Optional[Calibration] = None

        super(processDashboard, self).__init__(self.queueList, ready_event)

    # ---------------- internal init helpers ----------------

    def _get_table_state_path(self) -> str:
        base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        return os.path.join(base_path, "src", "utils", "table_state.json")

    def _safe_replace_ip_in_file(self) -> None:
        """
        IpManager가 Linux 명령(hostname -I)에 의존하면 macOS에서 실패함.
        dev_mode에서는 실패해도 진행.
        """
        try:
            IpManager.replace_ip_in_file()
        except Exception as e:
            if self.dev_mode:
                print(f"[ Dashboard ] : WARNING - IpManager failed in dev mode: {e}")
            else:
                raise

    def _create_web_app(self) -> None:
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app, cors_allowed_origins="*", async_mode="eventlet")
        CORS(self.app, supports_credentials=True)

        # ✅ Calibration은 queueList["Config"] 등을 요구하므로
        # dev_mode라도 반드시 올바른 queueList dict를 넘겨야 함.
        assert self.socketio is not None
        assert self.queueList is not None, "queueList must not be None (even in dev_mode)."
        self.calibration = Calibration(self.queueList, self.socketio)

    def _initialize_messages(self) -> None:
        if self.dev_mode:
            # dev 모드에서는 brain 메시징(구독/송신) 필요 없음
            self.messages = {}
            self.sendMessages = {}
            self.messagesAndVals = {}
            return

        assert self.queueList is not None
        self.get_name_and_vals()
        self.messagesAndVals.pop("mainCamera", None)
        self.messagesAndVals.pop("Semaphores", None)
        self.subscribe()

    def _setup_websocket_handlers(self) -> None:
        assert self.socketio is not None
        self.socketio.on_event("message", self.handle_message)
        self.socketio.on_event("save", self.handle_save_table_state)
        self.socketio.on_event("load", self.handle_load_table_state)

    def _start_background_tasks(self) -> None:
        psutil.cpu_percent(interval=1, percpu=False)
        eventlet.spawn(self.update_hardware_data)
        eventlet.spawn(self.send_hardware_data_to_frontend)
        eventlet.spawn(self.send_heartbeat)

        if not self.dev_mode:
            eventlet.spawn(self.send_continuous_messages)

    # ---------------- lifecycle ----------------

    def stop(self) -> None:
        super(processDashboard, self).stop()
        self.running = False

    def run(self) -> None:
        self._safe_replace_ip_in_file()
        self._create_web_app()
        self._initialize_messages()
        self._setup_websocket_handlers()
        self._start_background_tasks()

        if self.ready_event:
            self.ready_event.set()

        assert self.socketio is not None
        assert self.app is not None
        self.socketio.run(self.app, host=self.host, port=self.port)

    # ---------------- messaging wiring (brain mode) ----------------

    def subscribe(self) -> None:
        assert self.queueList is not None
        for name, enum in self.messagesAndVals.items():
            if enum["owner"] != "Dashboard":
                subscriber = messageHandlerSubscriber(self.queueList, enum["enum"], "lastOnly", True)
                self.messages[name] = {"obj": subscriber}
            else:
                sender = messageHandlerSender(self.queueList, enum["enum"])
                self.sendMessages[str(name)] = {"obj": sender}

        subscriber = messageHandlerSubscriber(self.queueList, Semaphores, "fifo", True)
        self.messages["Semaphores"] = {"obj": subscriber}

    def get_name_and_vals(self) -> None:
        classes = inspect.getmembers(allMessages, inspect.isclass)
        for name, cls in classes:
            if name != "Enum" and issubclass(cls, Enum):
                self.messagesAndVals[name] = {"enum": cls, "owner": cls.Owner.value}  # type: ignore

    def send_message_to_brain(self, dataName: str, dataDict: dict[str, Any]) -> None:
        if self.dev_mode:
            return
        if dataName in self.sendMessages:
            self.sendMessages[dataName]["obj"].send(dataDict.get("Value"))

    # ---------------- websocket handlers ----------------

    def handle_message(self, data: str) -> None:
        assert self.socketio is not None

        try:
            dataDict = json.loads(data)
            dataName = dataDict.get("Name")
            socketId = request.sid

            if dataName == "SessionAccess":
                self.handle_single_user_session(socketId)
            elif self.sessionActive and self.activeUser != socketId:
                print(f"\033[1;97m[ Dashboard ] :\033[0m \033[1;93mWARNING\033[0m - Message received from unauthorized user \033[94m{socketId}\033[0m")
                return

            if dataName == "Heartbeat":
                self.handle_heartbeat()
            elif dataName == "SessionEnd":
                self.handle_session_end(socketId)
            elif dataName == "DrivingMode":
                self.handle_driving_mode(dataDict)
            elif dataName == "Calibration":
                self.handle_calibration(dataDict, socketId)
            elif dataName == "GetCurrentSerialConnectionState":
                self.handle_get_current_serial_connection_state(socketId)
            else:
                self.send_message_to_brain(str(dataName), dataDict)

            self.socketio.emit("response", {"data": "Message received: " + str(data)}, room=socketId)  # type: ignore
        except json.JSONDecodeError:
            self.socketio.emit("response", {"error": "Invalid JSON format"}, room=request.sid)  # type: ignore

    def handle_heartbeat(self) -> None:
        self.heartbeat_retries = 0
        self.heartbeat_last_sent = time.time()
        self.heartbeat_received = True

    def handle_driving_mode(self, dataDict: dict[str, Any]) -> None:
        try:
            self.stateMachine.request_mode(f"dashboard_{dataDict['Value']}_button")
        except Exception:
            pass

    def handle_calibration(self, dataDict: dict[str, Any], socketId: str) -> None:
        if self.calibration is None:
            return
        self.calibration.handle_calibration_signal(dataDict, socketId)

    def handle_get_current_serial_connection_state(self, socketId: str) -> None:
        assert self.socketio is not None
        self.socketio.emit("current_serial_connection_state", {"data": self.serialConnected}, room=socketId)

    def handle_single_user_session(self, socketId: str) -> None:
        assert self.socketio is not None
        if not self.sessionActive:
            self.sessionActive = True
            self.activeUser = socketId
            self.socketio.emit("session_access", {"data": True}, room=socketId)
            self.send_message_to_brain("RequestSteerLimits", {"Value": True})
        elif self.activeUser == socketId:
            self.socketio.emit("session_access", {"data": True}, room=socketId)
            self.send_message_to_brain("RequestSteerLimits", {"Value": True})
        else:
            self.socketio.emit("session_access", {"data": False}, room=socketId)

    def handle_session_end(self, socketId: str) -> None:
        if self.sessionActive and self.activeUser == socketId:
            self.sessionActive = False
            self.activeUser = None

    def handle_save_table_state(self, data: str) -> None:
        assert self.socketio is not None
        try:
            dataDict = json.loads(data)
            os.makedirs(os.path.dirname(self.table_state_file), exist_ok=True)
            with open(self.table_state_file, "w") as json_file:
                json.dump(dataDict, json_file, indent=4)
            self.socketio.emit("response", {"data": "Table state saved successfully"})
        except Exception:
            self.socketio.emit("response", {"error": "Failed to save table state"})

    def handle_load_table_state(self, _data: str) -> None:
        assert self.socketio is not None
        try:
            with open(self.table_state_file, "r") as json_file:
                dataDict = json.load(json_file)
            self.socketio.emit("loadBack", {"data": dataDict})
        except Exception:
            self.socketio.emit("response", {"error": "Failed to load table state"})

    # ---------------- periodic tasks ----------------

    def update_hardware_data(self) -> None:
        self.cpuCoreUsage = psutil.cpu_percent(interval=None, percpu=False)
        self.memoryUsage = psutil.virtual_memory().percent
        t = get_cpu_temp_generic()
        self.cpuTemperature = round(t) if t is not None else None
        eventlet.spawn_after(1, self.update_hardware_data)

    def send_heartbeat(self) -> None:
        if not self.running or self.socketio is None:
            return

        if not self.heartbeat_received and self.sessionActive:
            self.heartbeat_retries += 1
            if self.heartbeat_retries < self.heartbeat_max_retries:
                self.socketio.emit("heartbeat", {"data": "Heartbeat"})
            else:
                self.socketio.emit("heartbeat_disconnect", {"data": "Heartbeat timeout"})
                self.sessionActive = False
                self.activeUser = None
                self.heartbeat_retries = 0

            eventlet.spawn_after(self.heartbeat_time_between_retries, self.send_heartbeat)
        else:
            self.heartbeat_received = False
            eventlet.spawn_after(self.heartbeat_time_between_heartbeats, self.send_heartbeat)

    def send_continuous_messages(self) -> None:
        if not self.running or self.socketio is None:
            return

        for msg, subscriber in self.messages.items():
            try:
                resp = subscriber["obj"].receive()
            except Exception:
                resp = None

            if resp is not None:
                if msg == "SerialConnectionState":
                    self.serialConnected = resp
                self.socketio.emit(msg, {"value": resp})

        eventlet.spawn_after(0.1, self.send_continuous_messages)

    def send_hardware_data_to_frontend(self) -> None:
        if not self.running or self.socketio is None:
            return

        self.socketio.emit("memory_channel", {"data": self.memoryUsage})
        self.socketio.emit("cpu_channel", {"data": {"usage": self.cpuCoreUsage, "temp": self.cpuTemperature}})
        eventlet.spawn_after(1.0, self.send_hardware_data_to_frontend)


# ---------------- standalone runner (Mac dev) ----------------

def _make_logger():
    import logging
    logging.basicConfig(level=logging.INFO)
    return logging.getLogger("dashboard-dev")


def run_dev_server(host: str = "0.0.0.0", port: int = 5005) -> None:
    """
    Mac에서 main.py 없이 dashboard backend만 띄우기 위한 실행 함수.
    ✅ Calibration이 요구하는 queueList 키들이 존재하도록 더미 큐를 만들어준다.
    """
    import queue

    logger = _make_logger()

    dummy_queue_list = {
        "Critical": queue.Queue(),
        "Warning": queue.Queue(),
        "General": queue.Queue(),
        "Config": queue.Queue(),
    }

    proc = processDashboard(
        queueList=dummy_queue_list,   # ✅ None 금지 (Calibration 때문에)
        logging_obj=logger,
        ready_event=None,
        debugging=True,
        dev_mode=True,
        host=host,
        port=port,
    )
    proc.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=5005, type=int)
    args = parser.parse_args()
    run_dev_server(host=args.host, port=args.port)
