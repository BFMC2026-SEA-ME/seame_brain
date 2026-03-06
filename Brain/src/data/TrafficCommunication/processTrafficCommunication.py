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
import select
import socket
import time
from multiprocessing import Pipe
from src.data.TrafficCommunication.useful.sharedMem import sharedMem
from src.templates.workerprocess import WorkerProcess
from src.templates.threadwithstop import ThreadWithStop
try:
    from src.data.TrafficCommunication.threads.threadTrafficCommunication import threadTrafficCommunication
except Exception:
    threadTrafficCommunication = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from geometry_msgs.msg import PoseStamped, Vector3Stamped, TwistWithCovarianceStamped
    from std_msgs.msg import Int32
except Exception:
    rclpy = None
    Node = None
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    PoseStamped = None
    Vector3Stamped = None
    TwistWithCovarianceStamped = None
    Int32 = None


class threadTrafficDataCollector(ThreadWithStop):
    """Collect vehicle state from ROS topics and store it in shared memory."""

    # [ADDED] TODO: topic names will be finalized later.
    POS_TOPIC = "/global_pose"             # expected type: geometry_msgs/PoseStamped
    SPEED_TOPIC = "/wheel_encoder"         # expected type: geometry_msgs/Vector3Stamped (y in m/s)
    SPEED_TWIST_TOPIC = "/wheel_twist"     # expected type: geometry_msgs/TwistWithCovarianceStamped (linear.x in m/s)

    def __init__(self, shared_memory, logger=None, debugging=False):
        super(threadTrafficDataCollector, self).__init__(pause=0.05)
        self.shared_memory = shared_memory
        self.logger = logger
        self.debugging = debugging

        self.latest_pos = None
        self.latest_rot = None
        self.latest_speed = None
        self._last_speed_update = 0.0
        self._speed_source = None
        self._last_pose_for_speed = None  # (x, y, monotonic_s)
        self._use_pose_speed_fallback = os.getenv("TRAFFIC_SPEED_FALLBACK_POSE", "0").lower() in ("1", "true", "yes", "y")
        self._use_twist_speed_source = os.getenv("TRAFFIC_USE_WHEEL_TWIST_SPEED", "0").lower() in ("1", "true", "yes", "y")
        # /wheel_encoder vector.y is m/s; always convert to cm/s in code.
        self._speed_scale = 100.0
        self._verbose_log = os.getenv("TRAFFIC_VERBOSE_LOG", "0").lower() in ("1", "true", "yes", "y")

        # [ADDED] Server upload payload is refreshed at 1 Hz.
        self._min_publish_period = 1.0  # seconds
        self._last_insert = {"devicePos": 0.0, "deviceRot": 0.0, "deviceSpeed": 0.0}
        self._last_tcp_send = 0.0

        # direct TCP sender (based on proven test script)
        self._tcp_enabled = os.getenv("TRAFFIC_SIMPLE_TCP_ENABLE", "1").lower() in ("1", "true", "yes", "y")
        self._tcp_host = os.getenv("TRAFFIC_TCP_HOST", "192.168.86.60") # 기훈이형 pc ip 
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
        self._udp_enabled = os.getenv("TRAFFIC_UDP_SEMAPHORE_ENABLE", "1").lower() in ("1", "true", "yes", "y")
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

        self._ros_enabled = (
            rclpy is not None
            and Node is not None
            and PoseStamped is not None
            and Vector3Stamped is not None
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
        self._spin_ros_once()
        self._flush_to_shared_memory()
        self._flush_to_tcp()
        self._poll_tcp_rx()
        self._poll_udp_rx()
        if self._verbose_log:
            self._log_waiting_pose()
            self._log_waiting_speed()
            self._log_ros_match_status()

    def stop(self):
        self._close_tcp()
        self._close_udp()
        self._close_ros()
        super(threadTrafficDataCollector, self).stop()

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
            self._ros_node.create_subscription(PoseStamped, self.POS_TOPIC, self._on_pos, sensor_qos)
            self._ros_node.create_subscription(Vector3Stamped, self.SPEED_TOPIC, self._on_speed, sensor_qos)
            if self._use_twist_speed_source and TwistWithCovarianceStamped is not None:
                self._ros_node.create_subscription(
                    TwistWithCovarianceStamped, self.SPEED_TWIST_TOPIC, self._on_speed_twist, sensor_qos
                )
            if Int32 is not None:
                self._traffic_color_pub = self._ros_node.create_publisher(
                    Int32, self._traffic_color_topic, 10
                )
            subs = [self.POS_TOPIC, self.SPEED_TOPIC]
            if self._use_twist_speed_source:
                subs.append(self.SPEED_TWIST_TOPIC)
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

        if self._ros_initialized_here and rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._ros_initialized_here = False

    # [ADDED] Topic callbacks -> local cache.
    def _on_pos(self, msg):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        self.latest_pos = (x, y)
        # Use clockwise-positive yaw in [0, 360) to match external TCP test format.
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg_ccw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self.latest_rot = (-yaw_deg_ccw) % 360.0

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

    def _flush_to_shared_memory(self):
        now = time.monotonic()

        if self.latest_pos is not None and (now - self._last_insert["devicePos"]) >= self._min_publish_period:
            self.shared_memory.insert("devicePos", [self.latest_pos[0], self.latest_pos[1]])
            self._last_insert["devicePos"] = now

        if self.latest_rot is not None and (now - self._last_insert["deviceRot"]) >= self._min_publish_period:
            self.shared_memory.insert("deviceRot", [self.latest_rot])
            self._last_insert["deviceRot"] = now

        if self.latest_speed is not None and (now - self._last_insert["deviceSpeed"]) >= self._min_publish_period:
            self.shared_memory.insert("deviceSpeed", [self.latest_speed])
            self._last_insert["deviceSpeed"] = now

    def _close_tcp(self):
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
        if self._sock is None:
            return False
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        try:
            self._sock.sendall(raw.encode("utf-8"))
            return True
        except Exception:
            self._close_tcp()
            self._next_tcp_retry = time.monotonic() + 3.0
            return False

    def _flush_to_tcp(self):
        if not self._tcp_enabled:
            return
        has_pose_payload = self.latest_pos is not None and self.latest_rot is not None
        has_speed_payload = self._tcp_send_speed and self.latest_speed is not None
        if not has_pose_payload and not has_speed_payload:
            return

        now = time.monotonic()
        if (now - self._last_tcp_send) < self._min_publish_period:
            return
        if not self._connect_tcp_if_needed():
            return

        if has_pose_payload:
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

            ok = self._send_tcp_json(
                {
                    "reqORinfo": "info",
                    "type": "deviceRot",
                    "value1": round(float(self.latest_rot), 3),
                }
            )
            if not ok:
                return

        if has_speed_payload:
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
            if self._verbose_log:
                self._log_speed_sent(speed_value, speed_source)

        self._last_tcp_send = now

    def _poll_tcp_rx(self):
        if not self._tcp_enabled:
            return
        if self._sock is None:
            self._connect_tcp_if_needed()
            return

        while True:
            try:
                readable, _, _ = select.select([self._sock], [], [], 0.0)
            except Exception:
                self._close_tcp()
                self._next_tcp_retry = time.monotonic() + 3.0
                return

            if not readable:
                break

            try:
                chunk = self._sock.recv(4096)
            except BlockingIOError:
                break
            except Exception:
                self._close_tcp()
                self._next_tcp_retry = time.monotonic() + 3.0
                return

            if not chunk:
                self._close_tcp()
                self._next_tcp_retry = time.monotonic() + 3.0
                return

            self._tcp_rx_buffer += chunk.decode("utf-8", errors="ignore")

        self._consume_tcp_rx_buffer()

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
        color_value = self._extract_traffic_color(payload)
        if color_value is None:
            return
        self._publish_traffic_color(color_value)

    def _extract_traffic_color(self, payload):
        if not isinstance(payload, dict):
            return None

        # UDP stream payload example:
        # {"device":"semaphore","id":0,"state":"red","x":1,"y":1}
        device = str(payload.get("device", "")).strip().lower()
        if device == "semaphore":
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

    def _publish_traffic_color(self, color_value):
        if self._ros_node is None or self._traffic_color_pub is None or Int32 is None:
            return
        msg = Int32()
        msg.data = int(color_value)
        self._traffic_color_pub.publish(msg)

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
        self.filename = "src/data/TrafficCommunication/useful/publickey_server_test.pem"
        self.deviceID = deviceID
        self.frequency = frequency
        self.debugging = debugging
        super(processTrafficCommunication, self).__init__(self.queuesList, ready_event)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        """Create the Traffic Communication thread and add it to the list of threads."""

        TrafficDataCollectorTh = threadTrafficDataCollector(
            self.shared_memory, self.logging, self.debugging
        )
        self.threads.append(TrafficDataCollectorTh)

        # Legacy BFMC traffic-com stack (UDP discovery + Twisted TCP) can be enabled explicitly.
        legacy_enabled = os.getenv("TRAFFIC_LEGACY_ENABLE", "0").lower() in ("1", "true", "yes", "y")
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
    
    shared_memory.insert("devicePos", [1.2, 2.3]) # send a position to the server
    shared_memory.insert("deviceRot", [3.4]) # send a rotation to the server
    shared_memory.insert("deviceSpeed", [4.5]) # send a speed to the server
    shared_memory.insert("historyData", [5.6, 6.7, 8]) # send a history data point to the server

    while time.time() - start_time < duration:
        try:
            print(queueList["General"].get(timeout=1))
        except:pass
    traffic_communication.stop()
