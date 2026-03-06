"""ROS2 bridge for global_planning goal + global path visualization.

- Receives goal node id from dashboard queue and publishes to
  `/global_planning/goal_node_id` (std_msgs/String).
- Subscribes to `/global_path` (nav_msgs/Path) and forwards it to the dashboard.
- Subscribes to `/global_pose` and forwards current vehicle pose to dashboard.
- Optionally loads GraphML nodes and forwards node list to the dashboard so
  the UI can render selectable nodes.
"""

from __future__ import annotations

import json
import os
import re
import sys
import time
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Mapping, MutableMapping, Optional, Tuple

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import Path as NavPath
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
except Exception:  # allow running without ROS2 deps
    rclpy = None
    SingleThreadedExecutor = None
    Node = object
    QoSHistoryPolicy = None
    QoSProfile = None
    QoSReliabilityPolicy = None
    String = None
    NavPath = None
    PoseStamped = None
    PoseWithCovarianceStamped = None

# Enable imports of BFMC frameworks.
PROJECT_ROOT = Path(__file__).resolve().parents[2]  # .../ros2_ws/src/Brain
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.templates.threadwithstop import ThreadWithStop  # type: ignore
from src.utils.messages.allMessages import (  # type: ignore
    GlobalPlanningGoalNodeId,
    GlobalPath,
    GlobalPose,
    MapNodes,
    RoadSign,
    RequestMapNodes,
)
from src.utils.messages.messageHandlerSender import messageHandlerSender  # type: ignore
from src.utils.messages.messageHandlerSubscriber import (  # type: ignore
    messageHandlerSubscriber,
)


_GRAPHML_RETRY_SEC = 1.0


def _find_graphml_path() -> Optional[Path]:
    env_path = os.environ.get("GLOBAL_PLANNING_GRAPHML")
    if env_path:
        candidate = Path(env_path)
        if candidate.exists():
            return candidate

    # Try to locate relative to ROS2 workspace layout.
    # Brain is expected at <ros2_ws>/src/Brain, so parents[3] => <ros2_ws>/src
    ws_src = Path(__file__).resolve().parents[3]
    candidates = [
        # Prefer GraphML stored inside this Brain repo.
        PROJECT_ROOT / "src" / "map" / "lab" / "lab_track_v1.graphml",
        ws_src / "map" / "lab" / "lab_track_v1.graphml",
        ws_src / "perception" / "maps" / "lab_track_v1.graphml",
    ]

    # Try ament_index_python if available.
    try:
        from ament_index_python.packages import get_package_share_directory

        try:
            share = Path(get_package_share_directory("perception"))
            candidates.append(share / "maps" / "lab_track_v1.graphml")
        except Exception:
            pass
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return None


def _parse_graphml_nodes(path: Path) -> Tuple[List[Dict[str, object]], Dict[str, float]]:
    tree = ET.parse(path)
    root = tree.getroot()

    ns = ""
    if root.tag.startswith("{"):
        ns = root.tag.split("}", 1)[0].strip("{")
    ns_map = {"g": ns} if ns else {}

    key_map: Dict[str, str] = {}
    for key in root.findall("g:key" if ns else "key", ns_map):
        if key.attrib.get("for") != "node":
            continue
        key_id = key.attrib.get("id")
        attr_name = key.attrib.get("attr.name") or key.attrib.get("attr.name")
        if key_id and attr_name:
            key_map[key_id] = attr_name

    nodes: List[Dict[str, object]] = []
    min_x = float("inf")
    max_x = float("-inf")
    min_y = float("inf")
    max_y = float("-inf")

    for node in root.findall(".//g:node" if ns else ".//node", ns_map):
        node_id = node.attrib.get("id")
        if not node_id:
            continue

        data_map: Dict[str, str] = {}
        for data in node.findall("g:data" if ns else "data", ns_map):
            key = data.attrib.get("key")
            if not key:
                continue
            name = key_map.get(key, key)
            if data.text is not None:
                data_map[name] = data.text

        # Common attribute names for coordinates.
        x_val = None
        y_val = None
        for key in ("x", "X", "pos_x", "posX", "lon", "longitude"):
            if key in data_map:
                x_val = data_map[key]
                break
        for key in ("y", "Y", "pos_y", "posY", "lat", "latitude"):
            if key in data_map:
                y_val = data_map[key]
                break

        if x_val is None or y_val is None:
            continue

        try:
            x = float(x_val)
            y = float(y_val)
        except ValueError:
            continue

        nodes.append({"id": str(node_id), "x": x, "y": y})
        min_x = min(min_x, x)
        max_x = max(max_x, x)
        min_y = min(min_y, y)
        max_y = max(max_y, y)

    bounds = {
        "min_x": min_x if nodes else 0.0,
        "max_x": max_x if nodes else 0.0,
        "min_y": min_y if nodes else 0.0,
        "max_y": max_y if nodes else 0.0,
    }
    return nodes, bounds


class GlobalPlanningBridgeNode(Node):
    def __init__(
        self,
        queues_list: Mapping[str, object],
        path_sender: messageHandlerSender,
        pose_sender: messageHandlerSender,
        road_sign_sender: messageHandlerSender,
    ):
        super().__init__("global_planning_bridge")
        self._queues_list = queues_list
        self._path_sender = path_sender
        self._pose_sender = pose_sender
        self._road_sign_sender = road_sign_sender
        self._last_path_send = 0.0
        self._path_send_period = 0.5  # 2 Hz
        self._last_pose_send = 0.0
        self._pose_send_period = 0.1  # 10 Hz
        self._last_sign_send = 0.0
        self._sign_send_period = float(os.environ.get("ROAD_SIGN_SEND_PERIOD", "0.2"))
        self._enable_path_stream = str(os.environ.get("DASHBOARD_ENABLE_GLOBAL_PATH", "0")).lower() in (
            "1",
            "true",
            "yes",
            "y",
        )
        self._allowed_classes = {
            "ONEWAY",
            "HIGHWAYENTRANCE",
            "STOPSIGN",
            "ROUNDABOUT",
            "PARK",
            "CROSSWALK",
            "NOENTRY",
            "HIGHWAYEXIT",
            "PRIORITY",
            "LIGHTS",
            "BLOCK",
            "PEDESTRIAN",
            "CAR",
        }

        goal_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._goal_pub = self.create_publisher(String, "/global_planning/goal_node_id", goal_qos)
        self._path_sub = None
        if self._enable_path_stream:
            if NavPath is not None:
                self._path_sub = self.create_subscription(
                    NavPath,
                    "/global_path",
                    self._on_path,
                    path_qos,
                )
            else:
                self.get_logger().warning("NavPath not available, /global_path subscription disabled.")
        else:
            self.get_logger().info("GlobalPath stream to dashboard is disabled (DASHBOARD_ENABLE_GLOBAL_PATH=0).")
        self._pose_sub = None
        self._global_pose_topic = str(os.environ.get("GLOBAL_POSE_TOPIC", "/global_pose"))
        self._global_pose_type = str(os.environ.get("GLOBAL_POSE_TYPE", "pose_stamped")).lower()

        try:
            if self._global_pose_type in (
                "pose_with_covariance_stamped",
                "posewithcovariancestamped",
                "cov",
                "covariance",
            ):
                if PoseWithCovarianceStamped is not None:
                    self._pose_sub = self.create_subscription(
                        PoseWithCovarianceStamped,
                        self._global_pose_topic,
                        self._on_pose_with_covariance,
                        path_qos,
                    )
                else:
                    self.get_logger().warning(
                        "PoseWithCovarianceStamped not available, /global_pose subscription disabled."
                    )
            else:
                if PoseStamped is not None:
                    self._pose_sub = self.create_subscription(
                        PoseStamped,
                        self._global_pose_topic,
                        self._on_pose_stamped,
                        path_qos,
                    )
                else:
                    self.get_logger().warning("PoseStamped not available, /global_pose subscription disabled.")
        except Exception as exc:
            self.get_logger().warning(f"Failed to subscribe {self._global_pose_topic}: {exc}")

        self._event_xy_topic = str(os.environ.get("ROAD_SIGN_EVENT_XY_TOPIC", "/obstacle_roi/event_xy"))
        self._obstacle_roi_topic = str(os.environ.get("ROAD_SIGN_OBSTACLE_ROI_TOPIC", "/obstacle_roi"))
        self._event_xy_sub = None
        self._obstacle_roi_sub = None
        try:
            self._event_xy_sub = self.create_subscription(
                String, self._event_xy_topic, self._on_event_xy, path_qos
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to subscribe {self._event_xy_topic}: {exc}")
        try:
            self._obstacle_roi_sub = self.create_subscription(
                String, self._obstacle_roi_topic, self._on_obstacle_roi, path_qos
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to subscribe {self._obstacle_roi_topic}: {exc}")

    def publish_goal(self, node_id: str) -> None:
        msg = String()
        msg.data = str(node_id)
        self._goal_pub.publish(msg)
        self.get_logger().info(f"Goal node published: {msg.data}")

    def _on_path(self, msg: NavPath) -> None:
        now = time.time()
        if now - self._last_path_send < self._path_send_period:
            return

        points = [
            {"x": pose.pose.position.x, "y": pose.pose.position.y}
            for pose in msg.poses
        ]
        payload = {
            "frame": msg.header.frame_id or "map",
            "points": points,
        }
        self._path_sender.send(payload)
        self._last_path_send = now

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _send_pose(self, x: float, y: float, frame: str, qx: float, qy: float, qz: float, qw: float) -> None:
        now = time.time()
        if now - self._last_pose_send < self._pose_send_period:
            return
        payload = {
            "frame": frame or "map",
            "x": float(x),
            "y": float(y),
            "yaw": self._quat_to_yaw(float(qx), float(qy), float(qz), float(qw)),
        }
        self._pose_sender.send(payload)
        self._last_pose_send = now

    def _on_pose_stamped(self, msg: PoseStamped) -> None:
        pose = msg.pose
        self._send_pose(
            pose.position.x,
            pose.position.y,
            msg.header.frame_id or "map",
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def _on_pose_with_covariance(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        self._send_pose(
            pose.position.x,
            pose.position.y,
            msg.header.frame_id or "map",
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )

    def _on_event_xy(self, msg: String) -> None:
        self._handle_road_sign_payload(msg.data, self._event_xy_topic)

    def _on_obstacle_roi(self, msg: String) -> None:
        self._handle_road_sign_payload(msg.data, self._obstacle_roi_topic)

    def _handle_road_sign_payload(self, raw: str, source_topic: str) -> None:
        class_name = self._extract_class_name(raw)
        if class_name is None:
            return

        now = time.time()
        if now - self._last_sign_send < self._sign_send_period:
            return

        payload = {
            "class_name": class_name,
            "source_topic": source_topic,
        }
        self._road_sign_sender.send(payload)
        self._last_sign_send = now

    def _extract_class_name(self, raw: str) -> Optional[str]:
        text = str(raw or "").strip()
        if not text:
            return None

        candidates = []
        try:
            parsed = json.loads(text)
            if isinstance(parsed, str):
                candidates.append(parsed)
            elif isinstance(parsed, dict):
                for key in (
                    "class_name",
                    "className",
                    "class",
                    "label",
                    "name",
                    "event",
                    "type",
                    "value",
                ):
                    value = parsed.get(key)
                    if isinstance(value, str) and value.strip():
                        candidates.append(value)
                classes = parsed.get("classes")
                if isinstance(classes, list):
                    for item in classes:
                        if isinstance(item, str):
                            candidates.append(item)
                        elif isinstance(item, dict):
                            value = item.get("class_name") or item.get("class") or item.get("label")
                            if isinstance(value, str):
                                candidates.append(value)
            elif isinstance(parsed, list):
                for item in parsed:
                    if isinstance(item, str):
                        candidates.append(item)
                    elif isinstance(item, dict):
                        value = item.get("class_name") or item.get("class") or item.get("label")
                        if isinstance(value, str):
                            candidates.append(value)
        except Exception:
            pass

        if not candidates:
            candidates.append(text)
        # Common detector payload format: "CLASS_NAME,score,x" (CSV-like)
        for token in re.split(r"[,\s;|]+", text):
            token = token.strip()
            if token:
                candidates.append(token)

        for candidate in candidates:
            normalized = "".join(ch for ch in str(candidate).strip().upper() if ch.isalnum())
            canonical = self._to_canonical_class_name(normalized)
            if canonical is not None:
                return canonical

        return None

    def _to_canonical_class_name(self, normalized: str) -> Optional[str]:
        alias_map = {
            "ONEWAY": "ONEWAY",
            "HIGHWAYENTRANCE": "HIGHWAYENTRANCE",
            "STOPSIGN": "STOPSIGN",
            "ROUNDABOUT": "ROUNDABOUT",
            "PARK": "PARK",
            "PARKING": "PARK",
            "CROSSWALK": "CROSSWALK",
            "NOENTRY": "NOENTRY",
            "HIGHWAYEXIT": "HIGHWAYEXIT",
            "PRIORITY": "PRIORITY",
            "LIGHTS": "LIGHTS",
            "TRAFFICLIGHT": "LIGHTS",
            "BLOCK": "BLOCK",
            "ROADBLOCK": "BLOCK",
            "PEDESTRIAN": "PEDESTRIAN",
            "CAR": "CAR",
        }
        canonical = alias_map.get(normalized)
        if canonical in self._allowed_classes:
            return canonical
        return None


class _GlobalPlanningBridgeThread(ThreadWithStop):
    def __init__(self, queues_list: Mapping[str, object]) -> None:
        super().__init__(pause=0.01)
        self._queues_list = queues_list
        self._executor: Optional[SingleThreadedExecutor] = None
        self._node: Optional[GlobalPlanningBridgeNode] = None

        self._goal_subscriber = messageHandlerSubscriber(
            self._queues_list, GlobalPlanningGoalNodeId, "lastOnly", True
        )
        self._map_request_subscriber = messageHandlerSubscriber(
            self._queues_list, RequestMapNodes, "lastOnly", True
        )
        self._path_sender = messageHandlerSender(self._queues_list, GlobalPath, drop_old=True)
        self._pose_sender = messageHandlerSender(self._queues_list, GlobalPose, drop_old=True)
        self._map_nodes_sender = messageHandlerSender(self._queues_list, MapNodes, drop_old=True)
        self._road_sign_sender = messageHandlerSender(self._queues_list, RoadSign, drop_old=True)

        self._graph_sent = False
        self._last_graph_try = 0.0
        self._graph_path: Optional[Path] = None

    def thread_work(self) -> None:
        # Handle GraphML loading (non-ROS).
        self._maybe_send_graph_nodes()
        request = self._map_request_subscriber.receive()
        if request is not None:
            self._maybe_send_graph_nodes(force=True)

        # Initialize ROS if needed.
        if rclpy is None:
            time.sleep(0.1)
            return

        if self._node is None or self._executor is None:
            self._maybe_init_ros()
            time.sleep(0.05)
            return

        # Send goal updates if any.
        goal = self._goal_subscriber.receive()
        if goal is not None:
            try:
                self._node.publish_goal(str(goal))
            except Exception as exc:
                print(f"[GlobalPlanningBridge] publish_goal failed: {exc}")

        try:
            self._executor.spin_once(timeout_sec=0.01)
        except Exception as exc:
            print(f"[GlobalPlanningBridge] spin_once failed: {exc}")
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

            self._node = GlobalPlanningBridgeNode(
                self._queues_list, self._path_sender, self._pose_sender, self._road_sign_sender
            )
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
        except Exception as exc:
            print(f"[GlobalPlanningBridge] init failed: {exc}")
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
        if rclpy is not None and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        self._executor = None
        self._node = None

    def _maybe_send_graph_nodes(self, force: bool = False) -> None:
        if self._graph_sent and not force:
            return

        now = time.time()
        if not force:
            if now - self._last_graph_try < _GRAPHML_RETRY_SEC:
                return
        self._last_graph_try = now

        if self._graph_path is None:
            self._graph_path = _find_graphml_path()
            if self._graph_path is None:
                return

        try:
            nodes, bounds = _parse_graphml_nodes(self._graph_path)
            if not nodes:
                return

            payload = {
                "path": str(self._graph_path),
                "bounds": bounds,
                "nodes": nodes,
            }
            self._map_nodes_sender.send(payload)
            self._graph_sent = True
            print(f"[GlobalPlanningBridge] Loaded {len(nodes)} nodes from {self._graph_path}")
        except Exception as exc:
            print(f"[GlobalPlanningBridge] GraphML load failed: {exc}")


def create_global_planning_bridge_process(
    queue_list: MutableMapping[str, object], ready_event=None
):
    """Factory compatible with WorkerProcess usage in main.py."""
    from src.templates.workerprocess import WorkerProcess  # type: ignore

    class GlobalPlanningBridgeProcess(WorkerProcess):
        def _init_threads(self):
            self.threads.append(_GlobalPlanningBridgeThread(self.queuesList))

    return GlobalPlanningBridgeProcess(queue_list, ready_event=ready_event, daemon=True)


if __name__ == "__main__":
    from multiprocessing import Queue

    queue_list: MutableMapping[str, object] = {
        "General": Queue(),
        "Config": Queue(),
    }

    process = create_global_planning_bridge_process(queue_list)
    process.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        process.stop()
