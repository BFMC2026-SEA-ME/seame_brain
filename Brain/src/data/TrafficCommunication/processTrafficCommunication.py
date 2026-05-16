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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

if __name__ == "__main__":
    import sys
    sys.path.insert(0, "../../..")

# Import necessary modules
import json
import math
import os
import queue
import select
import socket
import threading
import time
from multiprocessing import Pipe
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.templates.workerprocess import WorkerProcess
from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.allMessages import Cars, Location, Semaphores
try:
    from src.data.TrafficCommunication.threads.threadTrafficCommunication import threadTrafficCommunication
except Exception:
    threadTrafficCommunication = None

TRAFFIC_LEGACY_ENABLE_DEFAULT = "1"

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
except Exception:
    rclpy = None
    Node = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None

try:
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped, TwistWithCovarianceStamped
except Exception:
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Vector3Stamped = None
    TwistWithCovarianceStamped = None

try:
    from std_msgs.msg import String as StringMsg
except Exception:
    StringMsg = None


class threadTrafficDataCollector(ThreadWithStop):
    """Collect vehicle state from ROS topics and store it in shared memory."""

    # [ADDED] TODO: topic names will be finalized later.
    POS_TOPIC = "/global_pose"             # expected type: geometry_msgs/PoseStamped
    SPEED_TOPIC = "/wheel_encoder"         # expected type: geometry_msgs/Vector3Stamped (y in m/s)
    SPEED_TWIST_TOPIC = "/wheel_twist"     # expected type: geometry_msgs/TwistWithCovarianceStamped (linear.x in m/s)
    HISTORY_TOPIC = "/obstacle_roi/event_xy"   # expected type: std_msgs/String ("class_name,x,y")

    def __init__(self, shared_memory, queues_list=None, logger=None, debugging=False, uwb_queue=None, device_id=None):
        super(threadTrafficDataCollector, self).__init__(pause=0.05) # 20Hz, 10Hz GPS 패킷 손실 방지
        self._uwb_queue = uwb_queue  # threadUWBSerial이 넣어주는 (x, y, quality, rx_time) 큐
        self._device_id = device_id
        self.shared_memory = shared_memory
        self.queues_list = queues_list
        self.logger = logger
        self.debugging = debugging

        self.latest_pos = None
        self.latest_rot = None
        self.latest_speed = None
        self._last_pos_for_heading: tuple | None = None
        self._heading_min_dist_m = float(os.getenv("TRAFFIC_HEADING_MIN_DIST_M", "0.05"))
        # [ADDED][historyData] last parsed event payload as (event_id, x, y)
        self.latest_history = None
        self._history_update_seq = 0
        self._last_shared_history_seq = 0
        self._last_tcp_history_seq = 0
        self._last_speed_update = 0.0
        self._speed_source = None
        self._last_pose_for_speed = None  # (x, y, monotonic_s)
        self._use_pose_speed_fallback = os.getenv("TRAFFIC_SPEED_FALLBACK_POSE", "0").lower() in ("1", "true", "yes", "y")
        self._use_twist_speed_source = os.getenv("TRAFFIC_USE_WHEEL_TWIST_SPEED", "0").lower() in ("1", "true", "yes", "y")
        # /wheel_encoder vector.y is m/s; always convert to cm/s in code.
        self._speed_scale = 100.0
        self._verbose_log = os.getenv("TRAFFIC_VERBOSE_LOG", "0").lower() in ("1", "true", "yes", "y")
        # [ADDED][historyData] allow topic override while keeping requested default.
        self._history_topic = os.getenv("TRAFFIC_HISTORY_TOPIC", self.HISTORY_TOPIC)
        # [ADDED][historyData] deterministic class->event_id mapping (requested table).
        self._history_class_to_id = {
            "STOPSIGN": 1,
            "PRIORITY": 2,
            "PARK": 3,
            "CROSSWALK": 4,
            "HIGHWAYENTRANCE": 5,
            "HIGHWAYEXIT": 6,
            "ROUNDABOUT": 7,
            "ONEWAY": 8,
            "NOENTRY": 9,
            "CAR": 10,
            "PEDESTRIAN_ON_CROSSWALK": 11,
            "PEDESTRIAN_ON_ROAD": 12,
            "BLOCK": 13,
            "LIGHTS": 14,
            "FOG": 15,
            "TUNNEL": 16,
            "RAMP": 17,
        }
        self._history_label_to_id = {}
        self._history_next_label_id = int(max(self._history_class_to_id.values(), default=0) + 1)

        # [ADDED] Server upload payload is refreshed at 1 Hz.
        self._min_publish_period = 1.0  # seconds
        self._last_insert = {"devicePos": 0.0, "deviceRot": 0.0, "deviceSpeed": 0.0, "historyData": 0.0}
        self._publish_heartbeat_period = max(
            self._min_publish_period,
            float(os.getenv("TRAFFIC_PUBLISH_HEARTBEAT_PERIOD", str(self._min_publish_period))),
        )
        self._position_change_epsilon = float(os.getenv("TRAFFIC_POSITION_CHANGE_EPSILON_M", "0.05"))
        self._rotation_change_epsilon = float(os.getenv("TRAFFIC_ROTATION_CHANGE_EPSILON_DEG", "1.0"))
        self._speed_change_epsilon = float(os.getenv("TRAFFIC_SPEED_CHANGE_EPSILON_CMS", "2.0"))
        self._last_shared_payload = {"devicePos": None, "deviceRot": None, "deviceSpeed": None}
        self._last_tcp_send = {"devicePos": 0.0, "deviceRot": 0.0, "deviceSpeed": 0.0, "historyData": 0.0}
        self._last_tcp_payload = {"devicePos": None, "deviceRot": None, "deviceSpeed": None}

        # TCP socket lock: shared between main thread (send) and RX thread (recv)
        self._sock_lock = threading.Lock()
        # GPS coordinates received from server, passed from RX thread to main thread for ROS publish
        # items: (x, y, covariance_xy_or_None, rx_time)
        self._gps_rx_queue: queue.SimpleQueue = queue.SimpleQueue()
        self._quality_threshold = int(os.getenv("UWB_SERIAL_QUALITY_THRESHOLD", "30"))
        # RX thread handle
        self._tcp_rx_thread: threading.Thread | None = None

        # direct TCP sender (based on proven test script)
        # legacy 모드에서는 threadTrafficCommunication이 서버 자동 탐색+전송을 담당하므로
        # 직접 TCP 전송은 기본 비활성화
        legacy_enabled = os.getenv("TRAFFIC_LEGACY_ENABLE", TRAFFIC_LEGACY_ENABLE_DEFAULT).lower() in ("1", "true", "yes", "y")
        tcp_default = "0" if legacy_enabled else "1"
        self._tcp_enabled = os.getenv("TRAFFIC_SIMPLE_TCP_ENABLE", tcp_default).lower() in ("1", "true", "yes", "y")
        # self._tcp_host = os.getenv("TRAFFIC_TCP_HOST", "192.168.86.35") # 기훈이형 pc ip 
        # self._tcp_host = os.getenv("TRAFFIC_TCP_HOST", "192.168.86.20") # 내 pc ip 
        self._tcp_host = os.getenv("TRAFFIC_TCP_HOST", "192.168.86.39") # 주헌 pc ip 

        
        self._tcp_port = int(os.getenv("TRAFFIC_TCP_PORT", "5000"))
        self._tcp_bind_ip = os.getenv("TRAFFIC_TCP_BIND_IP", "").strip()
        self._tcp_timeout = float(os.getenv("TRAFFIC_TCP_CONNECT_TIMEOUT", "3.0"))
        self._tcp_send_speed = os.getenv("TRAFFIC_TCP_SEND_SPEED", "1").lower() in ("1", "true", "yes", "y")
        self._sock = None
        self._tcp_rx_buffer = ""
        self._next_tcp_retry = 0.0
        self._last_tcp_diag_log = 0.0
        self._traffic_color_topic = os.getenv("TRAFFIC_COLOR_TOPIC", "/traffic_color")
        self._traffic_color_pub = None
        # Always mirror to the canonical topic for quick `ros2 topic echo /traffic_color` checks.
        self._traffic_color_pub_fixed = None
        self._gps_topic = os.getenv("TRAFFIC_GPS_TOPIC", "/gps")
        self._gps_frame_id = os.getenv("TRAFFIC_GPS_FRAME_ID", "map")
        self._gps_pub = None
        self._gps_min_publish_period = float(os.getenv("TRAFFIC_GPS_MIN_PUBLISH_PERIOD", "0.1"))
        # 데이터 지연 측정후 수정하기 . 기본 1초 
        self._uwb_measurement_delay = float(os.getenv("UWB_MEASUREMENT_DELAY_S", "1.0"))
        self._last_gps_publish = 0.0
        self._gps_car_id_filter = self._device_id
        # UDP direct listen is optional; prefer queue feed from processSemaphores to avoid port conflicts.
        self._udp_enabled = os.getenv("TRAFFIC_UDP_SEMAPHORE_ENABLE", "0").lower() in ("1", "true", "yes", "y")
        self._udp_port = int(os.getenv("TRAFFIC_UDP_SEMAPHORE_PORT", "5007"))
        self._udp_bind_ip = os.getenv("TRAFFIC_UDP_SEMAPHORE_BIND_IP", "").strip()
        self._udp_sock = None
        self._next_udp_retry = 0.0
        sem_id_filter = os.getenv("TRAFFIC_UDP_SEMAPHORE_ID", "*").strip()
        if sem_id_filter in ("", "*"):
            self._udp_semaphore_id_filter = None
        else:
            try:
                self._udp_semaphore_id_filter = int(sem_id_filter)
            except ValueError:
                self._udp_semaphore_id_filter = None
        self._semaphore_subscriber = None
        self._cars_subscriber = None
        self._location_subscriber = None
        if self.queues_list is not None:
            try:
                self._semaphore_subscriber = messageHandlerSubscriber(
                    self.queues_list, Semaphores, "lastOnly", True
                )
            except Exception:
                self._semaphore_subscriber = None
            try:
                self._cars_subscriber = messageHandlerSubscriber(
                    self.queues_list, Cars, "fifo", True
                )
            except Exception:
                self._cars_subscriber = None
            try:
                self._location_subscriber = messageHandlerSubscriber(
                    self.queues_list, Location, "fifo", True
                )
            except Exception:
                self._location_subscriber = None

        self._ros_enabled = (
            rclpy is not None
            and Node is not None
            and QoSProfile is not None
            and QoSHistoryPolicy is not None
            and QoSReliabilityPolicy is not None
        )
        self._ros_node = None
        self._ros_initialized_here = False
        self._next_ros_retry = 0.0
        self._last_no_pose_log = 0.0
        self._last_no_speed_log = 0.0
        self._last_speed_send_log = 0.0
        self._last_speed_input_log = 0.0
        self._last_ros_match_log = 0.0

        if self._verbose_log and not self._ros_enabled:
            # WARNING log intentionally suppressed.
            pass
        if self._verbose_log and self._tcp_enabled:
            bind_info = self._tcp_bind_ip if self._tcp_bind_ip else "auto"
            print(
                f"\033[1;97m[ Traffic Communication ] :\033[0m "
                f"\033[1;92mINFO\033[0m - Simple TCP target "
                f"\033[94m{self._tcp_host}:{self._tcp_port}\033[0m "
                f"(bind_ip={bind_info}, timeout={self._tcp_timeout}s, send_speed={self._tcp_send_speed})"
            )
            if self._tcp_send_speed:
                print(
                    f"\033[1;97m[ Traffic Communication ] :\033[0m "
                    f"\033[1;92mINFO\033[0m - Speed source policy "
                    f"(wheel_y primary, pose_fallback={self._use_pose_speed_fallback}, "
                    f"twist_enabled={self._use_twist_speed_source}, scale={self._speed_scale})"
                )

    def thread_work(self):
        self._ensure_tcp_rx_thread()
        self._connect_tcp_if_needed()  # ROS 데이터 유무와 무관하게 항상 연결 유지
        self._spin_ros_once()
        self._flush_to_shared_memory()
        self._flush_to_tcp()
        self._drain_gps_rx_queue()   # RX 스레드에서 받은 GPS 좌표를 ROS publish
        self._drain_uwb_queue()      # UWB serial 스레드에서 받은 좌표를 ROS publish
        self._poll_cars_queue()
        self._poll_semaphore_queue()
        self._poll_location_queue()
        self._poll_udp_rx()
        if self._verbose_log:
            self._log_waiting_pose()
            self._log_waiting_speed()
            self._log_ros_match_status()

    def stop(self):
        super(threadTrafficDataCollector, self).stop()  # _blocker.set() → RX 스레드 루프 종료
        self._close_tcp()
        if self._tcp_rx_thread is not None and self._tcp_rx_thread.is_alive():
            self._tcp_rx_thread.join(timeout=1.0)
            self._tcp_rx_thread = None
        self._close_udp()
        self._close_ros()

    # 정기적으로 ROS TOPIC 구독
    def _spin_ros_once(self):
        if not self._ros_enabled:
            return

        now = time.monotonic()
        if self._ros_node is None:
            if now < self._next_ros_retry:
                return
            self._init_ros()
            return

        try:
            rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        except Exception:
            self._close_ros()
            self._next_ros_retry = time.monotonic() + 3.0

    # [ADDED] Subscribe ROS topics that provide position/rotation/speed.
    def _init_ros(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                self._ros_initialized_here = True
            self._ros_node = Node("traffic_com_data_listener")
            sensor_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
            if PoseStamped is not None:
                # global pose 구독해서 위치/회전 정보 가져오기
                self._ros_node.create_subscription(PoseStamped, self.POS_TOPIC, self._on_pos, sensor_qos)
            if Vector3Stamped is not None:
                self._ros_node.create_subscription(Vector3Stamped, self.SPEED_TOPIC, self._on_speed, sensor_qos)
            if self._use_twist_speed_source and TwistWithCovarianceStamped is not None:
                self._ros_node.create_subscription(
                    TwistWithCovarianceStamped, self.SPEED_TWIST_TOPIC, self._on_speed_twist, sensor_qos
                )
            if StringMsg is not None:
                # [ADDED][historyData] subscribe obstacle event topic for historyData payloads.
                self._ros_node.create_subscription(
                    StringMsg, self._history_topic, self._on_history_event_xy, sensor_qos
                )
                self._traffic_color_pub = self._ros_node.create_publisher(
                    StringMsg, self._traffic_color_topic, 10
                )
                if self._traffic_color_topic != "/traffic_color":
                    self._traffic_color_pub_fixed = self._ros_node.create_publisher(
                        StringMsg, "/traffic_color", 10
                    )
            if PoseWithCovarianceStamped is not None:
                self._gps_pub = self._ros_node.create_publisher(
                    PoseWithCovarianceStamped, self._gps_topic, 10
                )
            subs = [self.POS_TOPIC, self.SPEED_TOPIC]
            if self._use_twist_speed_source:
                subs.append(self.SPEED_TWIST_TOPIC)
            if StringMsg is not None:
                subs.append(self._history_topic)
            if self._verbose_log:
                print(
                    f"\033[1;97m[ Traffic Communication ] :\033[0m "
                    f"\033[1;92mINFO\033[0m - ROS subscribers active: "
                    + ", ".join(f"\033[94m{s}\033[0m" for s in subs)
                )
        except Exception:
            self._close_ros()
            self._next_ros_retry = time.monotonic() + 3.0

    def _close_ros(self):
        if self._ros_node is not None:
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
            self._ros_node = None
            self._traffic_color_pub = None
            self._traffic_color_pub_fixed = None
            self._gps_pub = None

        if self._ros_initialized_here and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._ros_initialized_here = False

    # [ADDED] Topic callbacks -> local cache.
    # global pose 콜백함수 
    def _on_pos(self, msg):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        self.latest_pos = (x, y)

        # UWB 위치 차이로 절대 헤딩 계산: 맵 +x(오른쪽) = 0°, 시계방향 양수
        if self._last_pos_for_heading is not None:
            px, py = self._last_pos_for_heading
            dx = x - px
            dy = y - py
            if math.hypot(dx, dy) >= self._heading_min_dist_m:
                heading_ccw = math.degrees(math.atan2(dy, dx))
                self.latest_rot = (-heading_ccw) % 360.0
                self._last_pos_for_heading = (x, y)
        else:
            self._last_pos_for_heading = (x, y)

        # Optional fallback speed from pose delta when wheel speed topics are missing/stale.
        now = time.monotonic()
        if self._use_pose_speed_fallback:
            if self._last_pose_for_speed is not None:
                px, py, pt = self._last_pose_for_speed
                dt = now - pt
                if dt > 0.05:
                    dist_m = math.hypot(x - px, y - py)
                    pose_speed_cms = (dist_m / dt) * 100.0
                    if now - self._last_speed_update > 2.0:
                        self.latest_speed = pose_speed_cms
                        self._last_speed_update = now
                        self._speed_source = "pose_fallback"
            self._last_pose_for_speed = (x, y, now)

    def _on_speed(self, msg):
        # Strict policy: use /wheel_encoder vector.y as the single source.
        raw_vel = float(msg.vector.y)
        self.latest_speed = raw_vel * self._speed_scale
        self._last_speed_update = time.monotonic()
        self._speed_source = "wheel_encoder_y"
        if self._verbose_log:
            self._log_speed_input(msg.vector.x, msg.vector.y, msg.vector.z, self.latest_speed, self._speed_source)

    def _on_speed_twist(self, msg):
        if not self._use_twist_speed_source:
            return
        # Optional source (disabled by default).
        self.latest_speed = float(msg.twist.twist.linear.x) * self._speed_scale
        self._last_speed_update = time.monotonic()
        self._speed_source = "wheel_twist"

    # [ADDED][historyData] parse /obstacle/event_xy payload and cache one-shot history tuple.
    def _on_history_event_xy(self, msg):
        parsed = self._parse_history_event_xy(msg.data)
        if parsed is None:
            return
        self.latest_history = parsed
        self._history_update_seq += 1

    # [ADDED][historyData] supported inputs:
    # - CSV text: "class_name,x,y" (e.g. "PEDESTRIAN,1.390,0.814")
    # - JSON dict: {"x":..., "y":..., "event_id"/"id"/"class_id"/...}
    # - JSON list: [x, y, event]
    # - plain text: "x,y,event", "x y event", "class x y"
    # - return tuple order: (event_id:int, x:float, y:float)
    def _parse_history_event_xy(self, raw):
        text = str(raw or "").strip()
        if not text:
            return None

        payload = text
        try:
            payload = json.loads(text)
        except Exception:
            payload = text

        if isinstance(payload, dict):
            x = self._pick_history_float(payload, ("x", "value1", "cx", "center_x"))
            y = self._pick_history_float(payload, ("y", "value2", "cy", "center_y"))
            event = self._pick_history_event(
                payload,
                ("event_id", "id", "class_id", "value3", "event", "class_name", "class", "type", "label", "name"),
            )
            if x is None or y is None:
                return None
            if event is None:
                event = 0
            return (event, x, y)

        if isinstance(payload, (list, tuple)):
            if len(payload) < 2:
                return None
            first_num = self._coerce_history_float(payload[0])
            if len(payload) >= 3 and first_num is None:
                cls_event = self._coerce_history_event(payload[0])
                x2 = self._coerce_history_float(payload[1])
                y2 = self._coerce_history_float(payload[2])
                if cls_event is not None and x2 is not None and y2 is not None:
                    return (cls_event, x2, y2)
            x = self._coerce_history_float(payload[0])
            y = self._coerce_history_float(payload[1])
            if x is None or y is None:
                return None
            event = self._coerce_history_event(payload[2]) if len(payload) > 2 else 0
            if event is None:
                event = 0
            return (event, x, y)

        tokens = [t.strip() for t in text.replace(";", ",").replace("|", ",").split(",") if t.strip()]
        if len(tokens) < 2:
            tokens = [t for t in text.split() if t]
        if len(tokens) < 2:
            return None

        # [ADDED][historyData] Ignore header line printed/forwarded as "class_name,x,y".
        header_tokens = [self._normalize_history_label(tok) for tok in tokens[:3]]
        if len(header_tokens) >= 3 and header_tokens[0] in ("CLASSNAME", "CLASS") and header_tokens[1] == "X" and header_tokens[2] == "Y":
            return None

        # Preferred runtime format: "CLASS_NAME,x,y"
        token0_num = self._coerce_history_float(tokens[0])
        if len(tokens) >= 3 and token0_num is None:
            cls_event = self._coerce_history_event(tokens[0])
            x2 = self._coerce_history_float(tokens[1])
            y2 = self._coerce_history_float(tokens[2])
            if cls_event is not None and x2 is not None and y2 is not None:
                return (cls_event, x2, y2)

        # Backward-compat format: "x,y,event"
        x = self._coerce_history_float(tokens[0])
        y = self._coerce_history_float(tokens[1])
        if x is not None and y is not None:
            event = self._coerce_history_event(tokens[2]) if len(tokens) > 2 else 0
            if event is None:
                event = 0
            return (event, x, y)

        # Fallback: first label + first two numeric tokens (e.g. "PEDESTRIAN score x y").
        label = None
        numeric_vals = []
        for token in tokens:
            v = self._coerce_history_float(token)
            if v is not None:
                numeric_vals.append(v)
                continue
            if label is None:
                label = token
        if label is not None and len(numeric_vals) >= 2:
            event = self._coerce_history_event(label)
            if event is None:
                return None
            return (event, numeric_vals[0], numeric_vals[1])
        return None

    # [ADDED][historyData] utility: choose first numeric value from a dict key list.
    def _pick_history_float(self, payload, keys):
        for key in keys:
            if key in payload:
                value = self._coerce_history_float(payload.get(key))
                if value is not None:
                    return value
        return None

    # [ADDED][historyData] utility: choose first valid event code from a dict key list.
    def _pick_history_event(self, payload, keys):
        for key in keys:
            if key in payload:
                value = self._coerce_history_event(payload.get(key))
                if value is not None:
                    return value
        return None

    # [ADDED][historyData] coerce numeric x/y values.
    def _coerce_history_float(self, raw):
        try:
            return float(raw)
        except Exception:
            return None

    # [ADDED][historyData] event value can be numeric or label text.
    # Label text is mapped to stable runtime IDs (1, 2, 3, ...).
    def _coerce_history_event(self, raw):
        if raw is None:
            return None
        if isinstance(raw, bool):
            return int(raw)
        if isinstance(raw, (int, float)):
            return int(raw)

        text = str(raw).strip()
        if not text:
            return None
        try:
            return int(float(text))
        except ValueError:
            normalized = self._normalize_history_label(text)
            if not normalized or normalized in ("CLASSNAME", "CLASS"):
                return None

            # Keep sign class mapping aligned with global-planning aliases.
            canonical = self._canonical_history_class(normalized)
            if canonical is not None and canonical in self._history_class_to_id:
                return self._history_class_to_id[canonical]

            if normalized not in self._history_label_to_id:
                self._history_label_to_id[normalized] = int(self._history_next_label_id)
                self._history_next_label_id += 1
            return self._history_label_to_id[normalized]

    # [ADDED][historyData] Normalize labels: "pedestrian", "PEDESTRIAN", "pedestrian-1" -> comparable keys.
    def _normalize_history_label(self, raw):
        return "".join(ch for ch in str(raw).strip().upper() if ch.isalnum())

    # [ADDED][historyData] Canonicalize common aliases before class->event_id lookup.
    def _canonical_history_class(self, normalized):
        if "HIGHWAY" in normalized:
            if any(token in normalized for token in ("ENTRANCE", "ENTRY", "IN")):
                return "HIGHWAYENTRANCE"
            if any(token in normalized for token in ("EXIT", "OUT")):
                return "HIGHWAYEXIT"
        alias_map = {
            "STOP": "STOPSIGN",
            "STOPSIGN": "STOPSIGN",
            "PRIORITY": "PRIORITY",
            "PARKING": "PARK",
            "PARK": "PARK",
            "CROSSWALK": "CROSSWALK",
            "HIGHWAYENTRANCE": "HIGHWAYENTRANCE",
            "HIGHWAYEXIT": "HIGHWAYEXIT",
            "ROUNDABOUT": "ROUNDABOUT",
            "ONEWAYROAD": "ONEWAY",
            "ONEWAY": "ONEWAY",
            "NOENTRY": "NOENTRY",
            "DONOTENTER": "NOENTRY",
            "STATICCARONPARKING": "CAR",
            "STATICCAR": "CAR",
            "PARKEDCAR": "CAR",
            "CAR": "CAR",
            "PEDESTRIANONCROSSWALK": "PEDESTRIAN_ON_CROSSWALK",
            "PEDESTRIANONROAD": "PEDESTRIAN_ON_ROAD",
            "ROADBLOCK": "BLOCK",
            "BLOCK": "BLOCK",
            "TRAFFICLIGHT": "LIGHTS",
            "LIGHTS": "LIGHTS",
            "FOG": "FOG",
            "TUNNEL": "TUNNEL",
            "RAMP": "RAMP",
        }
        return alias_map.get(normalized)

    def _angle_diff_deg(self, current, previous):
        return abs((float(current) - float(previous) + 180.0) % 360.0 - 180.0)

    def _payload_changed(self, key, current, previous):
        if previous is None:
            return True

        if key == "devicePos":
            return math.hypot(
                float(current[0]) - float(previous[0]),
                float(current[1]) - float(previous[1]),
            ) >= self._position_change_epsilon

        if key == "deviceRot":
            return self._angle_diff_deg(current, previous) >= self._rotation_change_epsilon

        if key == "deviceSpeed":
            return abs(float(current) - float(previous)) >= self._speed_change_epsilon

        return current != previous

    def _should_publish_cached(self, key, current, now, last_payloads, last_sent):
        previous = last_payloads.get(key)
        if previous is None:
            return True

        if self._payload_changed(key, current, previous):
            return (now - last_sent.get(key, 0.0)) >= self._min_publish_period

        return (now - last_sent.get(key, 0.0)) >= self._publish_heartbeat_period

    #  shared memory에 업데이트 
    def _flush_to_shared_memory(self):
        now = time.monotonic()

        if self.latest_pos is not None and self._should_publish_cached(
            "devicePos",
            self.latest_pos,
            now,
            self._last_shared_payload,
            self._last_insert,
        ):
            self.shared_memory.insert("devicePos", [self.latest_pos[0], self.latest_pos[1]])
            self._last_shared_payload["devicePos"] = tuple(self.latest_pos)
            self._last_insert["devicePos"] = now

        if self.latest_rot is not None and self._should_publish_cached(
            "deviceRot",
            self.latest_rot,
            now,
            self._last_shared_payload,
            self._last_insert,
        ):
            self.shared_memory.insert("deviceRot", [self.latest_rot])
            self._last_shared_payload["deviceRot"] = float(self.latest_rot)
            self._last_insert["deviceRot"] = now

        if self.latest_speed is not None and self._should_publish_cached(
            "deviceSpeed",
            self.latest_speed,
            now,
            self._last_shared_payload,
            self._last_insert,
        ):
            self.shared_memory.insert("deviceSpeed", [self.latest_speed])
            self._last_shared_payload["deviceSpeed"] = float(self.latest_speed)
            self._last_insert["deviceSpeed"] = now

        # [ADDED][historyData] push new /obstacle/event_xy event as historyData(value1=id, value2=x, value3=y).
        has_new_history = (
            self.latest_history is not None
            and self._history_update_seq != self._last_shared_history_seq
        )
        if has_new_history and (now - self._last_insert["historyData"]) >= self._min_publish_period:
            self.shared_memory.insert(
                "historyData",
                [
                    int(self.latest_history[0]),
                    float(self.latest_history[1]),
                    float(self.latest_history[2]),
                ],
            )
            self._last_insert["historyData"] = now
            self._last_shared_history_seq = self._history_update_seq

    def _close_tcp(self):
        with self._sock_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None
        self._tcp_rx_buffer = ""

    def _close_udp(self):
        if self._udp_sock is not None:
            try:
                self._udp_sock.close()
            except Exception:
                pass
            self._udp_sock = None

    def _bind_udp_if_needed(self):
        if not self._udp_enabled:
            return False
        if self._udp_sock is not None:
            return True

        now = time.monotonic()
        if now < self._next_udp_retry:
            return False

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                except OSError:
                    pass
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.bind((self._udp_bind_ip if self._udp_bind_ip else "", self._udp_port))
            sock.setblocking(False)
            self._udp_sock = sock
            if self._verbose_log:
                bind_ip = self._udp_bind_ip if self._udp_bind_ip else "0.0.0.0"
                print(
                    f"\033[1;97m[ Traffic Communication ] :\033[0m "
                    f"\033[1;92mINFO\033[0m - UDP semaphore listen "
                    f"\033[94m{bind_ip}:{self._udp_port}\033[0m"
                )
            return True
        except Exception:
            self._close_udp()
            self._next_udp_retry = now + 3.0
            return False

    def _connect_tcp_if_needed(self):
        if not self._tcp_enabled:
            return False
        with self._sock_lock:
            if self._sock is not None:
                return True

        now = time.monotonic()
        if now < self._next_tcp_retry:
            return False

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self._tcp_bind_ip:
                sock.bind((self._tcp_bind_ip, 0))
            sock.settimeout(self._tcp_timeout)
            sock.connect((self._tcp_host, self._tcp_port))
            sock.settimeout(None)
            with self._sock_lock:
                self._sock = sock
            print(
                f"\033[1;97m[ Traffic Communication ] :\033[0m "
                f"\033[1;92mINFO\033[0m - Simple TCP connected to "
                f"\033[94m{self._tcp_host}:{self._tcp_port}\033[0m"
            )
            return True
        except Exception:
            self._close_tcp()
            self._next_tcp_retry = now + 3.0
            return False

    
    def _send_tcp_json(self, payload):
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        with self._sock_lock:
            if self._sock is None:
                return False
            try:
                # tcp 통해서 json 형태로 데이터 전송
                self._sock.sendall(raw.encode("utf-8"))
                return True
            except Exception:
                pass
        self._close_tcp()
        self._next_tcp_retry = time.monotonic() + 3.0
        return False

    def _flush_to_tcp(self):
        if not self._tcp_enabled:
            return
        has_pos_payload = self.latest_pos is not None
        has_rot_payload = self.latest_rot is not None
        has_pose_payload = has_pos_payload or has_rot_payload
        has_speed_payload = self._tcp_send_speed and self.latest_speed is not None
        # [ADDED][historyData] send each new history event once over TCP.
        has_history_payload = (
            self.latest_history is not None
            and self._history_update_seq != self._last_tcp_history_seq
        )
        if not has_pos_payload and not has_rot_payload and not has_speed_payload and not has_history_payload:
            return

        now = time.monotonic()
        if not self._connect_tcp_if_needed():
            return

        pos_due = False
        if has_pos_payload:
            pos_due = self._should_publish_cached(
                "devicePos",
                self.latest_pos,
                now,
                self._last_tcp_payload,
                self._last_tcp_send,
            )

        rot_due = False
        if has_rot_payload:
            rot_due = self._should_publish_cached(
                "deviceRot",
                self.latest_rot,
                now,
                self._last_tcp_payload,
                self._last_tcp_send,
            )

        speed_due = False
        if has_speed_payload:
            speed_due = self._should_publish_cached(
                "deviceSpeed",
                self.latest_speed,
                now,
                self._last_tcp_payload,
                self._last_tcp_send,
            )

        history_due = False
        if has_history_payload:
            history_due = (now - self._last_tcp_send["historyData"]) >= self._min_publish_period

        if not pos_due and not rot_due and not speed_due and not history_due:
            return

        # 실제 데이터 payload 전송 부분
        if pos_due:
            ok = self._send_tcp_json(
                {
                    "reqORinfo": "info",
                    "type": "devicePos",
                    "value1": float(self.latest_pos[0]),
                    "value2": float(self.latest_pos[1]),
                }
            )
            if not ok:
                return
            self._last_tcp_payload["devicePos"] = tuple(self.latest_pos)
            self._last_tcp_send["devicePos"] = now

        if rot_due:
            ok = self._send_tcp_json(
                {
                    "reqORinfo": "info",
                    "type": "deviceRot",
                    "value1": round(float(self.latest_rot), 3),
                }
            )
            if not ok:
                return
            self._last_tcp_payload["deviceRot"] = float(self.latest_rot)
            self._last_tcp_send["deviceRot"] = now

        if speed_due:
            speed_value = float(self.latest_speed)
            speed_source = self._speed_source if self._speed_source is not None else "unknown"
            ok = self._send_tcp_json(
                {
                    "reqORinfo": "info",
                    "type": "deviceSpeed",
                    "value1": speed_value,
                }
            )
            if not ok:
                return
            self._last_tcp_payload["deviceSpeed"] = speed_value
            self._last_tcp_send["deviceSpeed"] = now
            if self._verbose_log:
                self._log_speed_sent(speed_value, speed_source)

        if history_due:
            ok = self._send_tcp_json(
                {
                    "reqORinfo": "info",
                    "type": "historyData",
                    "value1": int(self.latest_history[0]),
                    "value2": float(self.latest_history[1]),
                    "value3": float(self.latest_history[2]),
                }
            )
            if not ok:
                return
            self._last_tcp_history_seq = self._history_update_seq
            self._last_tcp_send["historyData"] = now

    def _ensure_tcp_rx_thread(self):
        """RX 전용 스레드가 없거나 죽었으면 재시작."""
        if not self._tcp_enabled:
            return
        if self._tcp_rx_thread is not None and self._tcp_rx_thread.is_alive():
            return
        t = threading.Thread(target=self._tcp_rx_loop, daemon=True, name="tcp_rx_loop")
        t.start()
        self._tcp_rx_thread = t

    def _tcp_rx_loop(self):
        """별도 스레드: TCP 수신 전용 루프. select 블로킹으로 GPS 지연 최소화."""
        rx_buffer = ""
        while not self._blocker.is_set():
            # 소켓 연결 대기
            with self._sock_lock:
                sock = self._sock
            if sock is None:
                time.sleep(0.1)
                continue

            try:
                readable, _, _ = select.select([sock], [], [], 0.05)
            except Exception:
                self._close_tcp()
                self._next_tcp_retry = time.monotonic() + 3.0
                time.sleep(0.1)
                continue

            if not readable:
                continue

            try:
                chunk = sock.recv(4096)
            except BlockingIOError:
                continue
            except Exception:
                self._close_tcp()
                self._next_tcp_retry = time.monotonic() + 3.0
                time.sleep(0.1)
                continue

            if not chunk:
                self._close_tcp()
                self._next_tcp_retry = time.monotonic() + 3.0
                time.sleep(0.1)
                continue

            rx_buffer += chunk.decode("utf-8", errors="ignore")
            rx_buffer = self._consume_rx_buffer_in_thread(rx_buffer)

    def _consume_rx_buffer_in_thread(self, rx_buffer: str) -> str:
        """RX 스레드 전용 버퍼 파싱. GPS(x,y)만 큐에 넣고 나머지는 무시."""
        decoder = json.JSONDecoder()
        while True:
            rx_buffer = rx_buffer.lstrip()
            if not rx_buffer:
                return ""

            if rx_buffer[0] not in "{[":
                next_start = min(
                    [idx for idx in (rx_buffer.find("{"), rx_buffer.find("[")) if idx >= 0],
                    default=-1,
                )
                if next_start == -1:
                    return ""
                rx_buffer = rx_buffer[next_start:]
                continue

            try:
                payload, end_idx = decoder.raw_decode(rx_buffer)
            except ValueError:
                return rx_buffer  # 불완전한 프레임, 다음 recv까지 보관

            rx_buffer = rx_buffer[end_idx:]

            gps_xy = self._extract_gps_xy(payload)
            if gps_xy is not None:
                if not isinstance(payload, dict) or "quality" not in payload:
                    continue  # quality 없는 패킷 폐기
                try:
                    q = int(payload["quality"])
                    if q < self._quality_threshold:
                        continue  # 품질 기준 미달 패킷 폐기
                    sigma = 0.05 + (1.0 - q / 100.0) * 0.35
                    covariance_xy = sigma ** 2
                except Exception:
                    continue
                self._gps_rx_queue.put((gps_xy[0], gps_xy[1], covariance_xy, time.time()))
            # 계속 루프 → 버퍼에 남은 프레임 처리
        return rx_buffer  # unreachable, 타입 힌트 만족용

    def _drain_gps_rx_queue(self):
        """메인 스레드: RX 스레드가 쌓아 둔 GPS 좌표를 ROS publish."""
        while not self._gps_rx_queue.empty():
            try:
                x, y, covariance_xy, rx_time = self._gps_rx_queue.get_nowait()
            except queue.Empty:
                break
            self._publish_gps(x, y, rx_time, covariance_xy)

    def _drain_uwb_queue(self):
        """메인 스레드: UWB serial 스레드가 쌓아 둔 좌표를 ROS publish."""
        if self._uwb_queue is None:
            return
        while not self._uwb_queue.empty():
            try:
                x, y, covariance_xy, rx_time = self._uwb_queue.get_nowait()
            except queue.Empty:
                break
            self._publish_gps(x, y, rx_time, covariance_xy)

    def _poll_udp_rx(self):
        if not self._udp_enabled:
            return
        if not self._bind_udp_if_needed():
            return

        # Cap packets per cycle to avoid starving other tasks under burst traffic.
        for _ in range(32):
            try:
                readable, _, _ = select.select([self._udp_sock], [], [], 0.0)
            except Exception:
                self._close_udp()
                self._next_udp_retry = time.monotonic() + 3.0
                return

            if not readable:
                return

            try:
                data, _addr = self._udp_sock.recvfrom(8192)  # type: ignore[arg-type]
            except BlockingIOError:
                return
            except Exception:
                self._close_udp()
                self._next_udp_retry = time.monotonic() + 3.0
                return

            try:
                payload = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            self._handle_tcp_payload(payload)

    def _poll_semaphore_queue(self):
        if self._semaphore_subscriber is None:
            return
        # Semaphores queue carries semaphore payloads.
        for _ in range(64):
            try:
                payload = self._semaphore_subscriber.receive()
            except Exception:
                return
            if payload is None:
                return
            self._handle_tcp_payload(payload)

    def _poll_cars_queue(self):
        if self._cars_subscriber is None:
            return
        for _ in range(64):
            try:
                payload = self._cars_subscriber.receive()
            except Exception:
                return
            if payload is None:
                return
            self._handle_tcp_payload(payload)

    def _poll_location_queue(self):
        """Legacy tcpClient가 서버에서 받은 Location 데이터를 ROS /gps로 publish."""
        if self._location_subscriber is None:
            return
        for _ in range(64):
            try:
                payload = self._location_subscriber.receive()
            except Exception:
                return
            if payload is None:
                return
            if not isinstance(payload, dict):
                continue
            # BFMC 서버 location 응답: {"type":"location", "posA":x, "posB":y} 또는 {"x":..., "y":...}
            x = None
            y = None
            for xkey in ("posA", "x", "value1"):
                if xkey in payload:
                    try:
                        x = float(payload[xkey])
                    except (TypeError, ValueError):
                        pass
                    break
            for ykey in ("posB", "y", "value2"):
                if ykey in payload:
                    try:
                        y = float(payload[ykey])
                    except (TypeError, ValueError):
                        pass
                    break
            if x is None or y is None:
                continue
            # TRAFFIC_GPS_CAR_ID 필터 적용
            if self._gps_car_id_filter is not None:
                raw_id = payload.get("id")
                if raw_id is not None:
                    try:
                        if int(raw_id) != self._gps_car_id_filter:
                            continue
                    except (TypeError, ValueError):
                        continue
            covariance_xy = None
            raw_quality = payload.get("quality")
            if raw_quality is not None:
                try:
                    q = int(raw_quality)
                    if q >= self._quality_threshold:
                        sigma = 0.05 + (1.0 - q / 100.0) * 0.35
                        covariance_xy = sigma ** 2
                    else:
                        continue  # quality 기준 미달 → 폐기
                except (TypeError, ValueError):
                    pass
            rx_time = payload.get("_rx_time")
            self._publish_gps(x, y, rx_time=rx_time, covariance_xy=covariance_xy)

    def _consume_tcp_rx_buffer(self):
        if not self._tcp_rx_buffer:
            return

        decoder = json.JSONDecoder()
        while True:
            self._tcp_rx_buffer = self._tcp_rx_buffer.lstrip()
            if not self._tcp_rx_buffer:
                return

            if self._tcp_rx_buffer[0] not in "{[":
                next_start = min(
                    [idx for idx in (self._tcp_rx_buffer.find("{"), self._tcp_rx_buffer.find("[")) if idx >= 0],
                    default=-1,
                )
                if next_start == -1:
                    self._tcp_rx_buffer = ""
                    return
                self._tcp_rx_buffer = self._tcp_rx_buffer[next_start:]
                continue

            try:
                payload, end_idx = decoder.raw_decode(self._tcp_rx_buffer)
            except ValueError:
                # Partial JSON frame: keep buffer and wait next recv.
                return

            self._tcp_rx_buffer = self._tcp_rx_buffer[end_idx:]
            self._handle_tcp_payload(payload)

    def _handle_tcp_payload(self, payload):
        semaphore_payload = self._extract_semaphore_payload(payload)
        if semaphore_payload is not None:
            self._publish_traffic_color(semaphore_payload)

        gps_xy = self._extract_gps_xy(payload)
        if gps_xy is not None:
            self._publish_gps(gps_xy[0], gps_xy[1])

    def _extract_semaphore_payload(self, payload):
        # Publish as-is. Filter only non-traffic payloads.
        if not isinstance(payload, dict):
            return payload

        device = str(payload.get("device", "")).strip().lower()
        if device == "semaphore" or ("state" in payload and device in ("", "semaphore")):
            if self._udp_semaphore_id_filter is not None:
                try:
                    sem_id = int(payload.get("id"))
                except Exception:
                    return None
                if sem_id != self._udp_semaphore_id_filter:
                    return None
            return payload

        msg_type = str(payload.get("type", "")).strip().lower()
        if msg_type in ("traffic_color", "trafficcolor", "traffic_light", "trafficlight"):
            return payload
        if "traffic_color" in payload or "trafficColor" in payload:
            return payload
        return None

    def _build_semaphore_payload(self, payload, color_value):
        state_raw = payload.get("state")
        if isinstance(state_raw, str) and state_raw.strip():
            state_text = state_raw.strip().lower()
        else:
            state_text = self._color_code_to_state(int(color_value))

        out = {
            "device": "semaphore",
            "state": state_text,
            "color": int(color_value),
        }

        if "id" in payload:
            try:
                out["id"] = int(payload.get("id"))
            except Exception:
                pass
        if "x" in payload:
            try:
                out["x"] = float(payload.get("x"))
            except Exception:
                pass
        if "y" in payload:
            try:
                out["y"] = float(payload.get("y"))
            except Exception:
                pass
        return out

    def _extract_traffic_color(self, payload):
        if not isinstance(payload, dict):
            return None

        # UDP stream payload example:
        # {"device":"semaphore","id":0,"state":"red","x":1,"y":1}
        device = str(payload.get("device", "")).strip().lower()
        if device == "semaphore" or ("state" in payload and device in ("", "semaphore")):
            if self._udp_semaphore_id_filter is not None:
                try:
                    sem_id = int(payload.get("id"))
                except Exception:
                    return None
                if sem_id != self._udp_semaphore_id_filter:
                    return None
            return self._coerce_traffic_color(payload.get("state"))

        msg_type = str(payload.get("type", "")).strip().lower()
        raw_value = None

        for key in ("traffic_color", "trafficColor"):
            if key in payload:
                raw_value = payload[key]
                break

        if raw_value is None and msg_type in ("traffic_color", "trafficcolor", "traffic_light", "trafficlight"):
            for key in ("value", "value1", "color", "state"):
                if key in payload:
                    raw_value = payload[key]
                    break

        if raw_value is None:
            return None

        return self._coerce_traffic_color(raw_value)

    def _coerce_traffic_color(self, raw_value):
        if isinstance(raw_value, bool):
            return int(raw_value)
        if isinstance(raw_value, (int, float)):
            return int(raw_value)
        if isinstance(raw_value, str):
            value = raw_value.strip().lower()
            text_map = {
                "red": 0,
                "yellow": 1,
                "amber": 1,
                "green": 2,
                "off": 3,
            }
            if value in text_map:
                return text_map[value]
            try:
                return int(float(value))
            except ValueError:
                return None
        return None

    def _color_code_to_state(self, color_value):
        return {
            0: "red",
            1: "yellow",
            2: "green",
            3: "off",
        }.get(int(color_value), "unknown")

    def _publish_traffic_color(self, payload):
        if self._ros_node is None or StringMsg is None:
            return
        if self._traffic_color_pub is None and self._traffic_color_pub_fixed is None:
            return
        msg = StringMsg()
        if isinstance(payload, str):
            msg.data = payload
        else:
            try:
                msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            except Exception:
                msg.data = str(payload)
        if self._traffic_color_pub is not None:
            self._traffic_color_pub.publish(msg)
        if self._traffic_color_pub_fixed is not None:
            self._traffic_color_pub_fixed.publish(msg)

    def _extract_gps_xy(self, payload):
        if not isinstance(payload, dict):
            return None

        device = str(payload.get("device", "")).strip().lower()
        msg_type = str(payload.get("type", "")).strip().lower()

        is_car = False
        if device == "car" or msg_type in ("car", "location", "gps"):
            is_car = True
        elif device in ("", "car") and "x" in payload and "y" in payload and "state" not in payload:
            # Semaphores queue car payload has no `device`/`type`.
            is_car = True

        if not is_car:
            return None

        if self._gps_car_id_filter is not None:
            raw_id = payload.get("id")
            if raw_id is not None:  # id 필드 없으면 필터 적용 안 함 (우리 차 위치 포맷)
                try:
                    if int(raw_id) != self._gps_car_id_filter:
                        return None
                except Exception:
                    return None

        try:
            x = float(payload.get("x"))
            y = float(payload.get("y"))
        except Exception:
            return None
        return (x, y)

    def _publish_gps(self, x, y, rx_time: float | None = None, covariance_xy: float | None = None):
        if self._ros_node is None or self._gps_pub is None or PoseWithCovarianceStamped is None:
            return
        # period 체크는 수신 시각 기준으로 → drain 한 번에 여러 패킷이 쌓여도 모두 publish 가능
        check_time = rx_time if rx_time is not None else time.time()
        if self._gps_min_publish_period > 0.0 and (check_time - self._last_gps_publish) < self._gps_min_publish_period:
            return
        msg = PoseWithCovarianceStamped()
        # rx_time이 있으면 UWB 측정 지연을 보정한 시각을 타임스탬프로 사용, 없으면 현재 ROS 시간
        if rx_time is not None:
            adjusted_time = rx_time - self._uwb_measurement_delay
            sec = int(adjusted_time)
            nanosec = int((adjusted_time - sec) * 1e9)
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
        else:
            msg.header.stamp = self._ros_node.get_clock().now().to_msg()
        msg.header.frame_id = self._gps_frame_id
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        # covariance 없으면 서버 location 기본값 사용 (sigma=0.15m)
        var_xy = float(covariance_xy) if covariance_xy is not None else 0.0225
        cov[0]  = var_xy   # x
        cov[7]  = var_xy   # y
        cov[14] = 999.0    # z (UWB 미측정)
        cov[21] = 999.0    # roll (UWB 미측정)
        cov[28] = 999.0    # pitch (UWB 미측정)
        cov[35] = 999.0    # yaw (UWB 미측정)
        msg.pose.covariance = cov
        self._gps_pub.publish(msg)
        self._last_gps_publish = check_time

    def _log_waiting_pose(self):
        if not self._tcp_enabled:
            return
        if self.latest_pos is not None and self.latest_rot is not None:
            return
        now = time.monotonic()
        if now - self._last_no_pose_log < 5.0:
            return
        self._last_no_pose_log = now
        # WARNING log intentionally suppressed.

    def _log_waiting_speed(self):
        if not self._tcp_enabled or not self._tcp_send_speed:
            return
        if self.latest_speed is not None:
            return
        now = time.monotonic()
        if now - self._last_no_speed_log < 5.0:
            return
        self._last_no_speed_log = now
        # WARNING log intentionally suppressed.

    def _log_speed_sent(self, value, source):
        now = time.monotonic()
        if now - self._last_speed_send_log < 3.0:
            return
        self._last_speed_send_log = now
        print(
            f"\033[1;97m[ Traffic Communication ] :\033[0m "
            f"\033[1;92mINFO\033[0m - deviceSpeed sent "
            f"\033[94m{value:.6f}\033[0m (source={source}, scale={self._speed_scale})"
        )

    def _log_speed_input(self, rpm, vel_raw, dist_raw, speed_value, source):
        now = time.monotonic()
        if now - self._last_speed_input_log < 3.0:
            return
        self._last_speed_input_log = now
        print(
            f"\033[1;97m[ Traffic Communication ] :\033[0m "
            f"\033[1;92mINFO\033[0m - wheel_encoder rx "
            f"(rpm={float(rpm):.3f}, vel_y={float(vel_raw):.6f}, dist_z={float(dist_raw):.6f}) "
            f"-> speed_out={float(speed_value):.6f} (source={source}, scale={self._speed_scale})"
        )

    def _log_ros_match_status(self):
        if self._ros_node is None:
            return
        now = time.monotonic()
        if now - self._last_ros_match_log < 5.0:
            return
        self._last_ros_match_log = now
        try:
            enc_pub = self._ros_node.count_publishers(self.SPEED_TOPIC)
            twist_pub = self._ros_node.count_publishers(self.SPEED_TWIST_TOPIC)
            print(
                f"\033[1;97m[ Traffic Communication ] :\033[0m "
                f"\033[1;92mINFO\033[0m - ROS match speed pubs "
                f"({self.SPEED_TOPIC}={enc_pub}, {self.SPEED_TWIST_TOPIC}={twist_pub})"
            )
        except Exception:
            pass

    def _compute_tcp_route_diag(self):
        """Best-effort route diagnostics for timeout troubleshooting."""
        now = time.monotonic()
        if now - self._last_tcp_diag_log < 3.0:
            return ""
        self._last_tcp_diag_log = now
        try:
            test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if self._tcp_bind_ip:
                test_sock.bind((self._tcp_bind_ip, 0))
            test_sock.connect((self._tcp_host, self._tcp_port))
            local_ip, local_port = test_sock.getsockname()
            test_sock.close()
            return f" [route local={local_ip}:{local_port}]"
        except Exception as diag_exc:
            return f" [route unknown: {diag_exc}]"

##########################################################

class threadUWBSerial(ThreadWithStop):
    """UWB 로컬라이제이션 장치에서 USB serial로 위치 데이터를 읽어 uwb_queue에 넣는 스레드.

    장치 출력 포맷: {"x":float, "y":float, "z":float, "quality":int}
    quality → covariance 변환: sigma = 0.05 + (1 - quality/100) * 0.35
    """

    def __init__(self, uwb_queue):
        super(threadUWBSerial, self).__init__(pause=0)
        self._uwb_queue = uwb_queue
        self._port = os.getenv("UWB_SERIAL_PORT", "/dev/ttyUSB0")
        self._baud = int(os.getenv("UWB_SERIAL_BAUD", "115200"))
        self._quality_threshold = int(os.getenv("UWB_SERIAL_QUALITY_THRESHOLD", "30"))
        self._ser = None
        self._next_retry = 0.0

    def thread_work(self):
        if self._ser is None:
            now = time.monotonic()
            if now < self._next_retry:
                self._blocker.wait(0.5)
                return
            self._try_connect()
            return

        try:
            raw = self._ser.readline()
            rx_time = time.time()
            if raw:
                self._parse_and_enqueue(raw, rx_time)
        except Exception:
            self._close_serial()

    def _try_connect(self):
        try:
            import serial as _serial
            self._ser = _serial.Serial(self._port, self._baud, timeout=1.0)
            print(
                f"\033[1;97m[ UWB Serial ] :\033[0m "
                f"\033[1;92mINFO\033[0m - Opened {self._port} at {self._baud} baud"
            )
        except Exception as e:
            print(
                f"\033[1;97m[ UWB Serial ] :\033[0m "
                f"\033[1;91mERROR\033[0m - Failed to open {self._port}: {e}"
            )
            self._ser = None
            self._next_retry = time.monotonic() + 3.0

    def _close_serial(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._next_retry = time.monotonic() + 3.0

    def _parse_and_enqueue(self, raw, rx_time):
        try:
            data = json.loads(raw.decode("utf-8", errors="ignore").strip())
        except (json.JSONDecodeError, ValueError):
            return

        x = data.get("x")
        y = data.get("y")
        quality = int(data.get("quality", 0))

        if x is None or y is None:
            return
        if quality < self._quality_threshold:
            return

        # quality 100 → sigma=0.05m, quality 0 → sigma=0.40m
        sigma = 0.05 + (1.0 - quality / 100.0) * 0.35
        covariance_xy = sigma ** 2

        self._uwb_queue.put((float(x), float(y), covariance_xy, rx_time))

    def stop(self):
        self._close_serial()
        super(threadUWBSerial, self).stop()


##########################################################

class processTrafficCommunication(WorkerProcess):
    """This process receives the location of the car and sends it to the processGateway.
    
    Args:
        queueList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        logging (logging object): Used for debugging.
        deviceID (int): The ID of the device.
        frequency (float): The frequency of communication.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, deviceID, ready_event=None, debugging=False, frequency=1):
        self.queuesList = queueList
        self.logging = logging
        self.shared_memory = sharedMem()
        self.filename = "src/data/TrafficCommunication/useful/publickey_server.pem"
        self.deviceID = deviceID
        self.frequency = frequency
        self.debugging = debugging
        super(processTrafficCommunication, self).__init__(self.queuesList, ready_event)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Traffic Communication thread and add it to the list of threads."""

        uwb_queue = queue.SimpleQueue()

        TrafficDataCollectorTh = threadTrafficDataCollector(
            self.shared_memory, self.queuesList, self.logging, self.debugging,
            uwb_queue=uwb_queue, device_id=self.deviceID,
        )
        self.threads.append(TrafficDataCollectorTh)

        uwb_enabled = os.getenv("UWB_SERIAL_ENABLE", "0").lower() in ("1", "true", "yes", "y")
        if uwb_enabled:
            UWBSerialTh = threadUWBSerial(uwb_queue)
            self.threads.append(UWBSerialTh)

        # Legacy BFMC traffic-com stack (UDP discovery + Twisted TCP) can be enabled explicitly.
        legacy_enabled = os.getenv("TRAFFIC_LEGACY_ENABLE", TRAFFIC_LEGACY_ENABLE_DEFAULT).lower() in ("1", "true", "yes", "y")
        if legacy_enabled and threadTrafficCommunication is not None:
            TrafficComTh = threadTrafficCommunication(
                self.shared_memory, self.queuesList, self.deviceID, self.frequency, self.filename
            )
            self.threads.append(TrafficComTh)
        elif legacy_enabled and threadTrafficCommunication is None:
            # WARNING log intentionally suppressed.
            pass


# =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processTrafficCommunication.py

if __name__ == "__main__":
    from multiprocessing import Queue
    import sys
    import time

    shared_memory = sharedMem()
    locsysReceivePipe, locsysSendPipe = Pipe(duplex=False)
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    # filename = "useful/publickey_server.pem"
    filename = "useful/publickey_server_test.pem"
    deviceID = 3
    frequency = 0.4
    if threadTrafficCommunication is None:
        print("[ Traffic Communication ] : Legacy example unavailable (missing Twisted).")
        sys.exit(0)
    traffic_communication = threadTrafficCommunication(
        shared_memory, queueList, deviceID, frequency, filename
    )
    traffic_communication.start()    

    start_time = time.time()
    duration = 10  # specify the duration in seconds
    
    shared_memory.insert("devicePos", [1.2, 2.3]) # send a position x, y to the server
    shared_memory.insert("deviceRot", [3.4]) # send a rotation to the server
    shared_memory.insert("deviceSpeed", [4.5]) # send a speed to the server
    shared_memory.insert("historyData", [5.6, 6.7, 8]) # send a history data point to the server

    while time.time() - start_time < duration:
        try:
            print(queueList["General"].get(timeout=1))
        except:pass
    traffic_communication.stop()
