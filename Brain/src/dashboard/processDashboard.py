# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
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

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

import psutil
import json
import inspect
import eventlet
import os
import time
import glob
from queue import Empty


from flask import Flask, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS
from enum import Enum

from src.bridge.processGlobalPlanningBridge import _find_graphml_path, _parse_graphml_nodes
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.templates.workerprocess import WorkerProcess
from src.utils.messages.allMessages import Semaphores
from src.statemachine.stateMachine import StateMachine
from src.dashboard.components.calibration import Calibration
from src.dashboard.components.ip_manger import IpManager

import src.utils.messages.allMessages as allMessages


# new component for jetson 
def _read(p: str):
    try:
        with open(p, "r") as f:
            return f.read().strip()
    except Exception:
        return None

_thermal_zone_paths = None
_map_nodes_payload_cache = None
_map_nodes_payload_graph_path = None
_map_nodes_payload_graph_mtime_ns = None

def _get_thermal_zone_paths():
    global _thermal_zone_paths
    if _thermal_zone_paths is None:
        _thermal_zone_paths = sorted(glob.glob("/sys/devices/virtual/thermal/thermal_zone*"))
    return _thermal_zone_paths

def get_jetson_temps_c() -> dict[str, float]:
    temps = {}
    for z in _get_thermal_zone_paths():
        name = _read(z + "/type")
        raw  = _read(z + "/temp")
        if not name or not raw:
            continue
        try:
            temps[name] = float(raw) / 1000.0  # milli°C -> °C
        except ValueError:
            continue
    return temps

def get_jetson_cpu_temp_c() -> float | None:
    temps = get_jetson_temps_c()
    # Jetson마다 이름이 다를 수 있어서 fallback을 둠
    return (temps.get("cpu-thermal")
            or temps.get("tj-thermal")   # 너 출력에도 있음
            or temps.get("soc0-thermal"))


def get_dashboard_map_nodes_payload():
    global _map_nodes_payload_cache
    global _map_nodes_payload_graph_path
    global _map_nodes_payload_graph_mtime_ns

    graph_path = _find_graphml_path()
    if graph_path is None:
        return {"path": None, "bounds": None, "nodes": []}

    try:
        mtime_ns = graph_path.stat().st_mtime_ns
    except Exception:
        mtime_ns = None

    if (
        _map_nodes_payload_cache is not None
        and _map_nodes_payload_graph_path == graph_path
        and _map_nodes_payload_graph_mtime_ns == mtime_ns
    ):
        return _map_nodes_payload_cache

    try:
        nodes, bounds = _parse_graphml_nodes(graph_path)
    except Exception as exc:
        print(
            f"\033[1;97m[ Dashboard ] :\033[0m "
            f"\033[1;91mERROR\033[0m - Failed to load map nodes from {graph_path}: {exc}"
        )
        return {"path": str(graph_path), "bounds": None, "nodes": []}

    payload = {
        "path": str(graph_path),
        "bounds": bounds,
        "nodes": nodes,
    }
    _map_nodes_payload_cache = payload
    _map_nodes_payload_graph_path = graph_path
    _map_nodes_payload_graph_mtime_ns = mtime_ns
    return payload



class processDashboard(WorkerProcess):
    """This process handles the dashboard interactions, updating the UI based on the system's state.
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Made for debugging.
        debugging (bool): Enable debugging mode.
    """
    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, ready_event=None, debugging = False):

        self.running = True
        self.queueList = queueList
        self.logger = logging
        self.debugging = debugging

        # state machine
        self.stateMachine = StateMachine.get_instance()

        # message handling
        self.messages = {}
        self.sendMessages = {}
        self.messagesAndVals = {}

        # hardware monitoring
        self.memoryUsage = 0
        self.cpuCoreUsage = 0
        self.cpuTemperature = 0


        # heartbeat
        self.heartbeat_last_sent = time.time()
        self.heartbeat_retries = 0
        # Heartbeat is used to detect stale sessions. Keep these values forgiving
        # to tolerate background tabs / CPU spikes.
        self.heartbeat_max_retries = 12
        self.heartbeat_time_between_heartbeats = 60 # seconds
        self.heartbeat_time_between_retries = 15 # seconds
        self.heartbeat_received = False

        # session management
        self.sessionActive = False
        self.activeUser = None
        self.connectedClients = set()

        # serial connection state
        self.serialConnected = False

        # Semaphores stream can include high-rate car updates.
        # Keep only latest semaphore states and emit in small batches.
        self._latest_semaphores = {}
        self._pending_semaphore_ids = set()
        self._last_semaphore_emit = 0.0
        self._semaphore_emit_period_s = float(os.getenv("DASHBOARD_SEMAPHORE_EMIT_PERIOD", "0.25"))
        self._semaphore_drain_limit = int(os.getenv("DASHBOARD_SEMAPHORE_DRAIN_LIMIT", "64"))
        self._last_camera_emit = 0.0
        self._camera_emit_period_s = float(os.getenv("DASHBOARD_CAMERA_EMIT_PERIOD", "0.12"))
        self._camera_loop_period_s = max(
            0.02,
            float(os.getenv("DASHBOARD_CAMERA_LOOP_PERIOD", str(self._camera_emit_period_s))),
        )
        self._camera_idle_loop_period_s = max(
            self._camera_loop_period_s,
            float(os.getenv("DASHBOARD_CAMERA_IDLE_LOOP_PERIOD", "0.25")),
        )
        self._latest_camera_frame = None
        self._camera_frame_dirty = False
        self._camera_stream_enabled = False
        self._no_ack_message_names = {"SteerMotor", "SpeedMotor", "Brake", "Control"}

        # configuration
        self.table_state_file = self._get_table_state_path()

        # Runtime-only objects must be created in the child process.
        # Creating Flask/SocketIO/greenlets in the parent and then forking
        # causes unstable websocket behavior under load.
        self.app = None
        self.socketio = None
        self.calibration = None

        super(processDashboard, self).__init__(self.queueList, ready_event)
    

    def _get_table_state_path(self):
        """Get the path for table state file."""
        base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        return os.path.join(base_path, 'src', 'utils', 'table_state.json')
    

    def _initialize_messages(self):
        """Initialize message handling systems."""
        self.get_name_and_vals()
        self.messagesAndVals.pop("mainCamera", None)
        self.messagesAndVals.pop("serialCamera", None)
        self.messagesAndVals.pop("Semaphores", None)
        self.messagesAndVals.pop("Cars", None)
        # These channels are not rendered in the current dashboard UI.
        # Keep their internal queue flows available for other components, but
        # avoid dashboard subscribe/emit overhead for them.
        self.messagesAndVals.pop("ImuData", None)
        self.messagesAndVals.pop("ImuAck", None)
        self.messagesAndVals.pop("AliveSignal", None)
        self.messagesAndVals.pop("CalibPWMData", None)
        self.messagesAndVals.pop("CalibRunDone", None)
        self.messagesAndVals.pop("GlobalPath", None)
        self.messagesAndVals.pop("MapNodes", None)
        self.subscribe()
    

    def _setup_websocket_handlers(self):
        """Setup WebSocket event handlers."""
        if self.socketio is None:
            return
        self.socketio.on_event('connect', self.handle_connect)
        self.socketio.on_event('message', self.handle_message)
        self.socketio.on_event('save', self.handle_save_table_state)
        self.socketio.on_event('load', self.handle_load_table_state)
        self.socketio.on_event('disconnect', self.handle_disconnect)

    def _setup_http_routes(self):
        """Setup lightweight HTTP routes for static dashboard data."""
        if self.app is None:
            return

        self.app.add_url_rule(
            "/api/map_nodes",
            view_func=self.handle_get_map_nodes,
            methods=["GET"],
        )
    
    
    def _start_background_tasks(self):
        """Start background monitoring tasks."""
        psutil.cpu_percent(interval=1, percpu=False) # warm up

        eventlet.spawn(self.update_hardware_data)
        eventlet.spawn(self.send_continuous_messages)
        eventlet.spawn(self.send_camera_messages)
        eventlet.spawn(self.send_hardware_data_to_frontend)
        eventlet.spawn(self.send_heartbeat)


    # ===================================== STOP ==========================================
    def stop(self):
        """Stop the dashboard process."""
        super(processDashboard, self).stop()
        self.running = False


    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing method."""
        try:
            # ip replacement (opt-in to avoid dev-server rebuilds and disconnects)
            if os.environ.get("DASHBOARD_AUTO_IP") == "1":
                IpManager.replace_ip_in_file()

            self.app = Flask(__name__)
            self.socketio = SocketIO(
                self.app,
                cors_allowed_origins="*",
                async_mode='eventlet',
                ping_interval=25,
                ping_timeout=120,
            )
            CORS(self.app, supports_credentials=True)
            self.calibration = Calibration(self.queueList, self.socketio)
            self._initialize_messages()
            self._setup_http_routes()
            self._setup_websocket_handlers()
            self._start_background_tasks()

            if self.ready_event:
                self.ready_event.set()

            self.socketio.run(self.app, host='0.0.0.0', port=5005)
        except Exception as exc:
            print(
                f"\033[1;97m[ Dashboard ] :\033[0m "
                f"\033[1;91mERROR\033[0m - Dashboard process failed to start: {exc}"
            )
            raise


    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway."""
        for name, enum in self.messagesAndVals.items():
            if enum["owner"] != "Dashboard":
                subscriber = messageHandlerSubscriber(self.queueList, enum["enum"], "lastOnly", True)
                self.messages[name] = {"obj": subscriber}
            else:
                sender = messageHandlerSender(self.queueList, enum["enum"])
                self.sendMessages[str(name)] = {"obj": sender}

        subscriber = messageHandlerSubscriber(self.queueList, Semaphores, "lastOnly", True)
        self.messages["Semaphores"] = {"obj": subscriber}


    def get_name_and_vals(self):
        """Extract all message names and values for processing."""
        classes = inspect.getmembers(allMessages, inspect.isclass)
        for name, cls in classes:
            if name != "Enum" and issubclass(cls, Enum):
                self.messagesAndVals[name] = {"enum": cls, "owner": cls.Owner.value} # type: ignore


    def send_message_to_brain(self, dataName, dataDict):
        """Send messages to the backend."""
        if dataName in self.sendMessages:
            self.sendMessages[dataName]["obj"].send(dataDict.get("Value"))

    def _sync_camera_stream_state(self):
        should_stream = bool(self.connectedClients)
        if should_stream == self._camera_stream_enabled:
            return
        self._camera_stream_enabled = should_stream
        self.send_message_to_brain("CameraStreamState", {"Value": should_stream})


    def handle_message(self, data):
        """Handle incoming WebSocket messages."""
        if self.debugging:
            self.logger.info("Received message: " + str(data))

        try:
            dataDict = json.loads(data)
            dataName = dataDict["Name"]
            socketId = request.sid

            if dataName == "SessionAccess":
                self.handle_single_user_session(socketId)
            elif self.sessionActive and self.activeUser != socketId:
                print(f"\033[1;97m[ Dashboard ] :\033[0m \033[1;93mWARNING\033[0m - Message received from unauthorized user \033[94m{socketId}\033[0m")
                return
            elif self.sessionActive and self.activeUser == socketId:
                # Any valid message from active user implies the connection is alive.
                self.heartbeat_retries = 0
                self.heartbeat_last_sent = time.time()
                self.heartbeat_received = True

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
                self.send_message_to_brain(dataName, dataDict)

            if dataName not in self._no_ack_message_names:
                try:
                    self.socketio.emit('response', {'data': 'Message received: ' + str(data)}, room=socketId) # type: ignore
                except Exception as exc:
                    self.logger.error(f"Failed to emit response: {exc}")
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON message: {e}")
            self.socketio.emit('response', {'error': 'Invalid JSON format'}, room=socketId) # type: ignore

    def handle_connect(self):
        """Track connected clients so camera frames are not broadcast into the void."""
        self.connectedClients.add(request.sid)
        self._sync_camera_stream_state()


    def handle_heartbeat(self):
        """Handle heartbeat message."""
        self.heartbeat_retries = 0
        self.heartbeat_last_sent = time.time()
        self.heartbeat_received = True


    def handle_driving_mode(self, dataDict):
        """Handle driving mode change."""
        # 상태머신 모드 전환
        self.stateMachine.request_mode(f"dashboard_{dataDict['Value']}_button")
        # 다른 컴포넌트(예: cmd_vel 브릿지)가 DrivingMode를 구독할 수 있도록 큐로도 전파
        self.send_message_to_brain("DrivingMode", dataDict)

        # STOP 모드 진입 시 즉시 정지 명령 전송 (KL 상태는 건드리지 않음)
        mode_value = str(dataDict.get("Value", "")).lower()
        if mode_value == "stop":
            self.send_message_to_brain("EmergencyStop", {"Value": True})
            self.send_message_to_brain("SpeedMotor", {"Value": "0"})
            self.send_message_to_brain("SteerMotor", {"Value": "0"})
            self.send_message_to_brain("Brake", {"Value": "0"})


    def handle_calibration(self, dataDict, socketId):
        """Handle calibration signals from frontend."""
        if self.calibration is not None:
            self.calibration.handle_calibration_signal(dataDict, socketId)


    def handle_get_current_serial_connection_state(self, socketId):
        """Handle getting the current serial connection state."""
        self.socketio.emit('current_serial_connection_state', {'data': self.serialConnected}, room=socketId)

    def _trigger_safety_stop(self, reason: str = "disconnect"):
        """Force a safe stop on the vehicle when control link is lost."""
        try:
            self.send_message_to_brain("EmergencyStop", {"Value": True})
            self.send_message_to_brain("SpeedMotor", {"Value": "0"})
            self.send_message_to_brain("SteerMotor", {"Value": "0"})
            self.send_message_to_brain("Brake", {"Value": "0"})
            print(
                f"\033[1;97m[ Dashboard ] :\033[0m "
                f"\033[1;93mWARNING\033[0m - Safety stop triggered due to {reason}"
            )
        except Exception as exc:
            print(
                f"\033[1;97m[ Dashboard ] :\033[0m "
                f"\033[1;91mERROR\033[0m - Safety stop failed: {exc}"
            )

    def handle_single_user_session(self, socketId):
        """Handle session access for a single user."""
        if not self.sessionActive:
            self.sessionActive = True
            self.activeUser = socketId
            print(f"\033[1;97m[ Dashboard ] :\033[0m \033[1;92mINFO\033[0m - Session access granted to \033[94m{socketId}\033[0m")
            self.socketio.emit('session_access', {'data': True}, room=socketId)
            self.send_message_to_brain("RequestSteerLimits", {"Value": True})
        elif self.activeUser == socketId:
            self.socketio.emit('session_access', {'data': True}, room=socketId)
            self.send_message_to_brain("RequestSteerLimits", {"Value": True})
        else:
            print(f"\033[1;97m[ Dashboard ] :\033[0m \033[1;92mINFO\033[0m - Session access denied to \033[94m{socketId}\033[0m")
            self.socketio.emit('session_access', {'data': False}, room=socketId)


    def handle_session_end(self, socketId):
        """Handle session end for the single user."""
        if self.sessionActive and self.activeUser == socketId:
            self.sessionActive = False
            self.activeUser = None


    def handle_disconnect(self):
        """Handle client disconnect to release session ownership."""
        socketId = request.sid
        self.connectedClients.discard(socketId)
        self._sync_camera_stream_state()
        if self.sessionActive and self.activeUser == socketId:
            self._trigger_safety_stop("socket disconnect")
            self.sessionActive = False
            self.activeUser = None


    def handle_save_table_state(self, data):
        """Handle saving the table state to a JSON file."""
        if self.debugging:
            self.logger.info("Received save message: " + data)

        try:
            dataDict = json.loads(data)
            os.makedirs(os.path.dirname(self.table_state_file), exist_ok=True)
            
            with open(self.table_state_file, 'w') as json_file:
                json.dump(dataDict, json_file, indent=4)
                
            self.socketio.emit('response', {'data': 'Table state saved successfully'})
        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse JSON for save: {e}")
            self.socketio.emit('response', {'error': 'Invalid JSON format'})
        except OSError as e:
            self.logger.error(f"Failed to save table state: {e}")
            self.socketio.emit('response', {'error': 'Failed to save table state'})


    def handle_load_table_state(self, data):
        """Handle loading the table state from a JSON file."""
        try:
            with open(self.table_state_file, 'r') as json_file:
                dataDict = json.load(json_file)
            self.socketio.emit('loadBack', {'data': dataDict})
        except FileNotFoundError:
            self.socketio.emit('response', {'error': 'File not found. Please save the table state first.'})
        except json.JSONDecodeError:
            self.socketio.emit('response', {'error': 'Failed to parse JSON data from the file.'})
        except OSError as e:
            self.logger.error(f"Failed to load table state: {e}")
            self.socketio.emit('response', {'error': 'Failed to load table state'})

    def handle_get_map_nodes(self):
        """Serve static map nodes over HTTP so the dashboard does not keep a live pipe."""
        return jsonify(get_dashboard_map_nodes_payload())


    def update_hardware_data(self):
        """Monitor and update hardware metrics periodically."""
        self.cpuCoreUsage = psutil.cpu_percent(interval=None, percpu=False)
        self.memoryUsage = psutil.virtual_memory().percent
        try:
            t = get_jetson_cpu_temp_c()
            self.cpuTemperature = round(t) if t is not None else None   # 또는 -1 같은 기본값
        except : 
            print("Can not use psutil.sensor_temparatures() in jetson ")
        eventlet.spawn_after(1, self.update_hardware_data) # 1초마다 프론트엔드로 업데이트 


    def send_heartbeat(self):
        """Send a heartbeat message to the frontend."""
        if not self.running:
            return

        if not self.heartbeat_received and self.sessionActive:
            self.heartbeat_retries += 1
            try:
                if self.heartbeat_retries < self.heartbeat_max_retries:
                    self.socketio.emit('heartbeat', {'data': 'Heartbeat'})
                else:
                    print(f"\033[1;97m[ Dashboard ] :\033[0m \033[1;93mWARNING\033[0m - Connection lost with peer \033[94m{self.activeUser}\033[0m")
                    self._trigger_safety_stop("heartbeat timeout")
                    self.socketio.emit('heartbeat_disconnect', {'data': 'Heartbeat timeout'})
                    self.sessionActive = False
                    self.activeUser = None
                    self.heartbeat_retries = 0
            except Exception as exc:
                self.logger.error(f"Heartbeat emit failed: {exc}")

            eventlet.spawn_after(self.heartbeat_time_between_retries, self.send_heartbeat)
        else:
            self.heartbeat_received = False
            eventlet.spawn_after(self.heartbeat_time_between_heartbeats, self.send_heartbeat)


    def send_continuous_messages(self):
        """Process and send subscriber messages to the frontend."""
        if not self.running:
            return

        try:
            for msg, subscriber in self.messages.items():
                if msg == "Semaphores":
                    self._drain_and_emit_semaphores(subscriber["obj"])
                    continue
                resp = subscriber["obj"].receive()
                if resp is not None:
                    if msg == "SerialConnectionState":
                        self.serialConnected = resp
                    self.socketio.emit(msg, {"value": resp})
                    if self.debugging:
                        self.logger.info(f"{msg}: {resp}")
        except Exception as exc:
            self.logger.error(f"send_continuous_messages failed: {exc}")

        eventlet.spawn_after(0.1, self.send_continuous_messages)

    def send_camera_messages(self):
        """Send camera frames independently so telemetry can continue under video backpressure."""
        if not self.running:
            return

        try:
            self._drain_camera_queue()

            if not self.connectedClients:
                return

            now = time.monotonic()
            if (
                self._latest_camera_frame is not None
                and self._camera_frame_dirty
                and now - self._last_camera_emit >= self._camera_emit_period_s
            ):
                payload = self._latest_camera_frame
                emit_kwargs = {}
                if self.sessionActive and self.activeUser in self.connectedClients:
                    emit_kwargs["room"] = self.activeUser
                try:
                    self.socketio.emit("serialCamera", payload, binary=True, **emit_kwargs)
                except Exception:
                    self.socketio.emit("serialCamera", {"value": payload}, **emit_kwargs)
                self._camera_frame_dirty = False
                self._last_camera_emit = now
        except Exception as exc:
            self.logger.error(f"send_camera_messages failed: {exc}")
        finally:
            next_run_s = (
                self._camera_loop_period_s
                if self.connectedClients
                else self._camera_idle_loop_period_s
            )
            eventlet.spawn_after(next_run_s, self.send_camera_messages)

    def _drain_camera_queue(self):
        """Read the latest camera payload directly from the Image queue."""
        image_queue = self.queueList.get("Image")
        if image_queue is None:
            return

        latest_payload = None
        while True:
            try:
                message = image_queue.get_nowait()
            except Empty:
                break
            except Exception:
                break

            if not isinstance(message, dict):
                continue
            payload = message.get("msgValue")
            if payload is not None:
                latest_payload = payload

        if latest_payload is not None:
            self._latest_camera_frame = latest_payload
            self._camera_frame_dirty = True

    def _drain_and_emit_semaphores(self, subscriber_obj):
        drained = 0
        while drained < self._semaphore_drain_limit:
            resp = subscriber_obj.receive()
            if resp is None:
                break
            drained += 1

            if not isinstance(resp, dict):
                continue
            state = resp.get("state")
            if not isinstance(state, str) or not state:
                # Ignore non-semaphore packets (e.g. car payloads).
                continue

            try:
                sem_id = int(resp.get("id"))
                x = float(resp.get("x"))
                y = float(resp.get("y"))
            except Exception:
                continue

            normalized = {"id": sem_id, "state": state, "x": x, "y": y}
            if self._latest_semaphores.get(sem_id) != normalized:
                self._latest_semaphores[sem_id] = normalized
                self._pending_semaphore_ids.add(sem_id)

        if not self._pending_semaphore_ids:
            return

        now = time.monotonic()
        if now - self._last_semaphore_emit < self._semaphore_emit_period_s:
            return

        for sem_id in sorted(self._pending_semaphore_ids):
            payload = self._latest_semaphores.get(sem_id)
            if payload is None:
                continue
            self.socketio.emit("Semaphores", {"value": payload})
        self._pending_semaphore_ids.clear()
        self._last_semaphore_emit = now


    def send_hardware_data_to_frontend(self):
        """Send hardware monitoring data to the frontend."""
        if not self.running:
            return
        try:
            self.socketio.emit('memory_channel', {'data': self.memoryUsage})
            self.socketio.emit('cpu_channel', {
                'data': {
                    'usage': self.cpuCoreUsage,
                    'temp': self.cpuTemperature
                }
            })
        except Exception as exc:
            self.logger.error(f"send_hardware_data_to_frontend failed: {exc}")

        eventlet.spawn_after(3.0, self.send_hardware_data_to_frontend)
