#!/usr/bin/env python3
"""
SEAME Brain — Odometry Calibration Tool
오도메트리 보정 도구

실제 주행 경로와 /odom 출력을 비교하여
v_scale / WHEEL_VEL_SCALE / WHEEL_DIST_SCALE 보정값을 계산합니다.

사용법:
    # ROS 2 + seame_ros (odom_generator) 가 실행 중인 상태에서:
    source /opt/ros/humble/setup.bash
    cd ~/seame_brain/Brain
    python3 tools/odom_calibrator.py

    # Brain 대시보드의 방향키로 차량을 조종하면서 각 테스트를 수행합니다.

테스트 코스:
    [1] 직선 2m   — 2m 직진 후 정지
    [2] 좌회전 원 — 좌회전으로 원을 그려 원점 복귀
    [3] 우회전 원 — 우회전으로 원을 그려 원점 복귀

보정 대상 파라미터:
    - WHEEL_VEL_SCALE  (env, threadRead.py → /wheel_twist)
    - WHEEL_DIST_SCALE (env, threadRead.py → /wheel_encoder distance)
    - v_scale          (ROS param, odom_generator.py → /odom)
      * 총 스케일 = WHEEL_VEL_SCALE × v_scale
"""

import os
import sys
import math
import time
import json
import threading
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass
from typing import List, Optional, Dict, Tuple

# ── ROS 2 (optional) ──────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from nav_msgs.msg import Odometry
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    Node = object  # type: ignore

# ── matplotlib (optional, for trajectory plots) ───────────────────────────────
try:
    import matplotlib
    matplotlib.use("Agg")          # non-interactive: save to file
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ══════════════════════════════════════════════════════════════════════════════
# Configuration
# ══════════════════════════════════════════════════════════════════════════════
STRAIGHT_DISTANCE_M = 2.0          # known ground-truth distance for straight test

# Read current scales from environment (same as threadRead.py defaults)
CUR_WHEEL_VEL_SCALE  = float(os.environ.get("WHEEL_VEL_SCALE",  "1.042"))
CUR_WHEEL_DIST_SCALE = float(os.environ.get("WHEEL_DIST_SCALE", "1.042"))

RESULTS_DIR = Path(__file__).parent.parent / "calibration_results"

# ══════════════════════════════════════════════════════════════════════════════
# Data
# ══════════════════════════════════════════════════════════════════════════════
@dataclass
class Pose2D:
    x:   float   # m
    y:   float   # m
    yaw: float   # rad
    t:   float   # wall-clock seconds

# ══════════════════════════════════════════════════════════════════════════════
# Math helpers
# ══════════════════════════════════════════════════════════════════════════════
def _wrap_pi(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def path_length(poses: List[Pose2D]) -> float:
    """Arc length of the recorded trajectory."""
    total = 0.0
    for i in range(1, len(poses)):
        total += math.hypot(poses[i].x - poses[i-1].x,
                            poses[i].y - poses[i-1].y)
    return total

def displacement(poses: List[Pose2D]) -> Tuple[float, float, float]:
    """(dx, dy, Euclidean distance) from first to last pose."""
    dx = poses[-1].x - poses[0].x
    dy = poses[-1].y - poses[0].y
    return dx, dy, math.hypot(dx, dy)

def total_yaw_change(poses: List[Pose2D]) -> float:
    """Accumulated yaw change in radians (each step wrapped to [-π, π])."""
    total = 0.0
    for i in range(1, len(poses)):
        total += _wrap_pi(poses[i].yaw - poses[i-1].yaw)
    return total

# ══════════════════════════════════════════════════════════════════════════════
# Analysis
# ══════════════════════════════════════════════════════════════════════════════
def analyze_straight(poses: List[Pose2D]) -> Dict:
    """Compare recorded odom displacement against the known 2 m ground truth.

    Uses straight-line displacement (disp) rather than arc path length (plen)
    as the correction basis, making it robust against measurement noise that
    causes the odom trajectory to zigzag and overestimate distance.
    """
    plen = path_length(poses)
    dx, dy, disp = displacement(poses)
    yaw_drift_deg = math.degrees(total_yaw_change(poses))
    dur = poses[-1].t - poses[0].t if len(poses) >= 2 else 0.0

    # Scale correction uses displacement (start→end straight line), not arc length.
    # Noise causes plen ≥ disp; disp is a more stable ground-truth proxy.
    #   new_scale = current_scale × (actual / odom_measured)
    correction = STRAIGHT_DISTANCE_M / disp if disp > 1e-4 else 1.0
    sug_wv  = CUR_WHEEL_VEL_SCALE  * correction
    sug_wd  = CUR_WHEEL_DIST_SCALE * correction
    # v_scale in odom_generator is applied ON TOP of wheel_vel_scale
    # Total effective scale = WHEEL_VEL_SCALE × v_scale
    # To correct total scale without touching env var:
    sug_v_scale = 1.0 * correction   # assuming current v_scale = 1.0

    err_m   = disp - STRAIGHT_DISTANCE_M
    err_pct = (err_m / STRAIGHT_DISTANCE_M) * 100.0

    # Lateral deviation: perpendicular component relative to main travel axis.
    # If travel is mainly along X → lateral = abs(dy), and vice versa.
    if math.hypot(dx, dy) > 1e-4:
        lateral_m = abs(dy) if abs(dx) >= abs(dy) else abs(dx)
    else:
        lateral_m = 0.0

    return dict(
        test_type            = "straight",
        expected_m           = STRAIGHT_DISTANCE_M,
        odom_path_m          = round(plen, 5),
        odom_disp_m          = round(disp, 5),
        lateral_dev_m        = round(lateral_m, 5),
        path_error_m         = round(err_m, 5),
        path_error_pct       = round(err_pct, 3),
        yaw_drift_during_deg = round(yaw_drift_deg, 3),
        duration_s           = round(dur, 3),
        samples              = len(poses),
        avg_speed_m_s        = round(plen / dur, 4) if dur > 0 else 0.0,
        cur_wheel_vel_scale  = CUR_WHEEL_VEL_SCALE,
        cur_wheel_dist_scale = CUR_WHEEL_DIST_SCALE,
        correction_factor    = round(correction, 6),
        sug_wheel_vel_scale  = round(sug_wv, 6),
        sug_wheel_dist_scale = round(sug_wd, 6),
        sug_v_scale_odom     = round(sug_v_scale, 6),
    )


def analyze_circle(poses: List[Pose2D], direction: str) -> Dict:
    """
    Check whether the robot's odom returns to origin after one full circle.

    Position closure error and yaw accumulation error reveal whether
    distance scale and yaw estimation are consistent.
    """
    plen = path_length(poses)
    dx, dy, closure = displacement(poses)
    yaw_tot = total_yaw_change(poses)
    dur = poses[-1].t - poses[0].t if len(poses) >= 2 else 0.0

    # Expected: left turn +2π, right turn -2π
    exp_yaw = 2 * math.pi if direction == "left" else -2 * math.pi
    yaw_err_deg = math.degrees(yaw_tot - exp_yaw)

    # Estimate circle radius: if one full circle, circumference ≈ path_length
    est_r = plen / (2 * math.pi) if plen > 1e-4 else 0.0

    # Diagnosis
    diag: List[str] = []
    if closure > 0.10:
        diag.append(
            f"위치 폐합 오차 큼 ({closure:.3f} m) "
            "→ 거리 스케일 or yaw 오차 의심"
        )
    if abs(yaw_err_deg) > 5.0:
        diag.append(
            f"yaw 누적 오차 큼 ({yaw_err_deg:+.1f}°) "
            "→ IMU / yaw 추정 점검 필요"
        )
    if closure <= 0.05 and abs(yaw_err_deg) <= 3.0:
        diag.append("원형 폐합 양호 ✅  현재 파라미터 적절")
    if closure <= 0.05 and abs(yaw_err_deg) > 3.0:
        diag.append("위치는 닫히지만 yaw 누적 오차 → IMU yaw_offset_rad 조정 고려")
    if closure > 0.05 and abs(yaw_err_deg) <= 3.0:
        diag.append("yaw는 OK지만 위치 오차 → v_scale / WHEEL_VEL_SCALE 미세 조정 필요")

    return dict(
        test_type        = f"circle_{direction}",
        direction        = direction,
        odom_path_m      = round(plen,    5),
        est_radius_m     = round(est_r,   5),
        closure_error_m  = round(closure, 5),
        closure_dx_m     = round(dx,      5),
        closure_dy_m     = round(dy,      5),
        total_yaw_deg    = round(math.degrees(yaw_tot), 3),
        expected_yaw_deg = round(math.degrees(exp_yaw), 3),
        yaw_error_deg    = round(yaw_err_deg, 3),
        duration_s       = round(dur, 3),
        samples          = len(poses),
        diagnosis        = diag,
    )

# ══════════════════════════════════════════════════════════════════════════════
# ROS 2 subscriber node
# ══════════════════════════════════════════════════════════════════════════════
if HAS_ROS:
    class _OdomNode(Node):  # type: ignore
        def __init__(self):
            super().__init__("odom_calibrator")
            self.recording = False
            self._poses: List[Pose2D] = []
            self._lock   = threading.Lock()
            self.create_subscription(Odometry, "/odom", self._cb, 10)
            self.get_logger().info("odom_calibrator: /odom 구독 중")

        def _cb(self, msg: "Odometry"):
            with self._lock:
                if not self.recording:
                    return
            x   = msg.pose.pose.position.x
            y   = msg.pose.pose.position.y
            q   = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            with self._lock:
                self._poses.append(Pose2D(x, y, yaw, time.time()))

        def start_recording(self):
            with self._lock:
                self._poses.clear()
                self.recording = True

        def stop_recording(self) -> List[Pose2D]:
            with self._lock:
                self.recording = False
                return list(self._poses)

        def latest(self) -> Optional[Pose2D]:
            with self._lock:
                return self._poses[-1] if self._poses else None

        def sample_count(self) -> int:
            with self._lock:
                return len(self._poses)

# ══════════════════════════════════════════════════════════════════════════════
# Trajectory plot
# ══════════════════════════════════════════════════════════════════════════════
def _save_plot(poses: List[Pose2D], title: str, path: Path):
    if not HAS_MPL or not poses:
        return
    xs = [p.x for p in poses]
    ys = [p.y for p in poses]

    fig, axes = plt.subplots(1, 2, figsize=(13, 6))

    # ── left: trajectory ──
    ax = axes[0]
    ax.plot(xs, ys, "b-", lw=1.5, label="Odometry")
    ax.plot(xs[0],  ys[0],  "go", ms=10, label="Start")
    ax.plot(xs[-1], ys[-1], "r^", ms=10, label="End")
    # draw closure line for circle tests
    if len(xs) > 1:
        ax.plot([xs[0], xs[-1]], [ys[0], ys[-1]], "r--", lw=1, alpha=0.6, label="Closure")
    step = max(1, len(poses) // 25)
    for i in range(0, len(poses) - 1, step):
        ddx = xs[i + 1] - xs[i]
        ddy = ys[i + 1] - ys[i]
        if math.hypot(ddx, ddy) > 1e-4:
            ax.annotate(
                "", xy=(xs[i] + ddx * 0.6, ys[i] + ddy * 0.6),
                xytext=(xs[i], ys[i]),
                arrowprops=dict(arrowstyle="->", color="cornflowerblue", lw=0.8),
            )
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.35)
    ax.legend(fontsize=9)
    ax.set_title(f"Trajectory: {title}", fontsize=11)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")

    # ── right: yaw over time ──
    ax2 = axes[1]
    yaws = [math.degrees(p.yaw) for p in poses]
    ts   = [p.t - poses[0].t for p in poses]
    ax2.plot(ts, yaws, "m-", lw=1.5)
    ax2.set_title("Yaw over time", fontsize=11)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Yaw [°]")
    ax2.grid(True, alpha=0.35)

    plt.suptitle(title, fontsize=12, fontweight="bold")
    plt.tight_layout()
    plt.savefig(str(path), dpi=130)
    plt.close(fig)

# ══════════════════════════════════════════════════════════════════════════════
# Terminal UI helpers
# ══════════════════════════════════════════════════════════════════════════════
_W = 60

def _hr(ch="─"): return ch * _W

def print_header():
    print("\n" + _hr("═"))
    print("  SEAME Brain  —  Odometry Calibration Tool")
    print(_hr("═"))
    print(f"  WHEEL_VEL_SCALE  (현재) = {CUR_WHEEL_VEL_SCALE}")
    print(f"  WHEEL_DIST_SCALE (현재) = {CUR_WHEEL_DIST_SCALE}")
    if not HAS_MPL:
        print("  ℹ  matplotlib 없음 — 그래프 저장 비활성")
    print(_hr())

def print_menu():
    print()
    print("  ─── 테스트 선택 ───────────────────────")
    print("  [1]  직선 2m 테스트")
    print("  [2]  좌회전 원형 테스트  (원점 복귀)")
    print("  [3]  우회전 원형 테스트  (원점 복귀)")
    print("  [r]  전체 결과 요약 보기")
    print("  [s]  결과 저장  (JSON + 궤적 그래프)")
    print("  [q]  종료")

def _print_straight_result(a: Dict):
    print()
    print(_hr())
    print("  [ 직선 테스트 결과 ]")
    print(_hr("-"))
    print(f"  예상 거리            : {a['expected_m']:.3f} m")
    print(f"  오도메트리 경로 길이  : {a['odom_path_m']:.4f} m  (참고용)")
    print(f"  오도메트리 변위      : {a['odom_disp_m']:.4f} m  (보정 기준)")
    print(f"  변위 오차           : {a['path_error_m']:+.4f} m  ({a['path_error_pct']:+.2f} %)")
    print(f"  측방 편차           : {a['lateral_dev_m']:.4f} m")
    print(f"  직진 중 yaw 변화    : {a['yaw_drift_during_deg']:+.2f}°")
    print(f"  평균 속도           : {a['avg_speed_m_s']:.4f} m/s")
    print(f"  샘플 / 시간         : {a['samples']} ea  /  {a['duration_s']:.1f} s")
    print()
    print("  [ 보정 권장값 ]")
    print(f"  보정 계수           : {a['correction_factor']:.6f}")
    print("  ※ 아래 중 하나만 선택 적용하세요 (중복 적용 시 이중 보정 오류)")
    print(f"  ▶  [방법A] export WHEEL_VEL_SCALE={a['sug_wheel_vel_scale']:.6f}")
    print(f"             export WHEEL_DIST_SCALE={a['sug_wheel_dist_scale']:.6f}")
    print(f"  ▶  [방법B] ros2 param set /wheel_v_imu_odom v_scale {a['sug_v_scale_odom']:.6f}")
    _quality_badge(a["path_error_pct"])
    print(_hr())

def _print_circle_result(a: Dict):
    d = "좌회전" if a["direction"] == "left" else "우회전"
    print()
    print(_hr())
    print(f"  [ {d} 원형 테스트 결과 ]")
    print(_hr("-"))
    print(f"  오도메트리 경로 길이 : {a['odom_path_m']:.4f} m")
    print(f"  추정 원 반지름       : {a['est_radius_m']:.4f} m")
    print(f"  위치 폐합 오차       : {a['closure_error_m']:.4f} m")
    print(f"    X 방향 오차        : {a['closure_dx_m']:+.4f} m")
    print(f"    Y 방향 오차        : {a['closure_dy_m']:+.4f} m")
    print(f"  누적 yaw             : {a['total_yaw_deg']:+.2f}°  (예상 {a['expected_yaw_deg']:+.1f}°)")
    print(f"  yaw 오차             : {a['yaw_error_deg']:+.2f}°")
    print(f"  샘플 / 시간          : {a['samples']} ea  /  {a['duration_s']:.1f} s")
    print()
    print("  [ 진단 ]")
    for msg in a["diagnosis"]:
        print(f"    → {msg}")
    print(_hr())

def _quality_badge(err_pct: float):
    print()
    if   abs(err_pct) < 1.0:  print("  ✅  거리 오차 < 1 %  —  현재 스케일 양호")
    elif abs(err_pct) < 5.0:  print("  ⚠️   거리 오차 1~5 %  —  보정 권장")
    else:                      print("  ❌  거리 오차 > 5 %  —  즉시 보정 필요")

def print_summary(results: List[Dict]):
    print()
    print(_hr("═"))
    print("  [ 전체 테스트 요약 ]")
    print(_hr("═"))

    straights = [r for r in results if r["test_type"] == "straight"]
    circles   = [r for r in results if "circle" in r["test_type"]]

    if straights:
        avg_err   = sum(r["path_error_pct"]      for r in straights) / len(straights)
        avg_scale = sum(r["sug_wheel_vel_scale"]  for r in straights) / len(straights)
        avg_v     = sum(r["sug_v_scale_odom"]     for r in straights) / len(straights)
        print(f"\n  직선 테스트  ({len(straights)} 회 평균)")
        print(f"    평균 변위 오차 (보정 기준) : {avg_err:+.2f} %")
        print(f"    권장 WHEEL_VEL_SCALE      : {avg_scale:.6f}")
        print(f"    권장 WHEEL_DIST_SCALE     : {avg_scale:.6f}")
        print(f"    권장 v_scale (odom_gen)   : {avg_v:.6f}")

    if circles:
        print(f"\n  원형 테스트  ({len(circles)} 회)")
        for r in circles:
            tag = "좌" if r["direction"] == "left" else "우"
            print(f"    {tag}회전  폐합 오차 {r['closure_error_m']:.4f} m"
                  f"  /  yaw 오차 {r['yaw_error_deg']:+.2f}°")

    if straights:
        avg_scale = sum(r["sug_wheel_vel_scale"] for r in straights) / len(straights)
        avg_v     = sum(r["sug_v_scale_odom"]    for r in straights) / len(straights)
        print()
        print("  ─── 적용 방법 (하나만 선택!) ─────────────────────────")
        print("  ⚠️  방법 A 와 B 를 동시에 적용하면 이중 보정됩니다.")
        print()
        print("  # 방법 A: 환경 변수 조정  (threadRead.py 스케일)")
        print(f"  export WHEEL_VEL_SCALE={avg_scale:.6f}")
        print(f"  export WHEEL_DIST_SCALE={avg_scale:.6f}")
        print(f"  # odom_generator 의 v_scale 은 1.0 으로 유지")
        print()
        print("  # 방법 B: odom_generator ROS 파라미터만 조정")
        print(f"  ros2 param set /wheel_v_imu_odom v_scale {avg_v:.6f}")
        print(f"  # WHEEL_VEL_SCALE / WHEEL_DIST_SCALE 는 현재값 유지")
        print()
        print("  ⚠️  환경 변수를 변경한 후에는 반드시 스크립트를 재시작해야")
        print("     새로운 스케일 값이 반영됩니다.")

    print(_hr("═"))

# ══════════════════════════════════════════════════════════════════════════════
# Live display thread (shows odom pose while recording)
# ══════════════════════════════════════════════════════════════════════════════
def _live_display(node: "_OdomNode", stop_ev: threading.Event):
    while not stop_ev.is_set():
        p = node.latest()
        n = node.sample_count()
        if p:
            print(
                f"\r  📍 x={p.x:+7.4f} m  y={p.y:+7.4f} m"
                f"  yaw={math.degrees(p.yaw):+7.2f}°  [{n} samples]   ",
                end="",
                flush=True,
            )
        time.sleep(0.1)

# ══════════════════════════════════════════════════════════════════════════════
# Test runners
# ══════════════════════════════════════════════════════════════════════════════
def _record_session(node: "_OdomNode", label: str) -> List[Pose2D]:
    """Start recording, show live pose, stop on Enter, return poses."""
    node.start_recording()
    print(f"\n  🔴  기록 시작 — {label}")
    print("  주행 완료 후 Enter 를 누르세요 ...")

    stop_ev = threading.Event()
    t = threading.Thread(target=_live_display, args=(node, stop_ev), daemon=True)
    t.start()

    try:
        input()
    except EOFError:
        pass  # stdin closed (e.g. piped input) — treat as Enter

    stop_ev.set()
    t.join()
    poses = node.stop_recording()
    print(f"\r  ⏹  기록 완료  —  {len(poses)} samples                          ")
    return poses


def run_straight_test(node: "_OdomNode") -> Optional[Dict]:
    print()
    print(_hr())
    print("  [ 직선 2m 테스트 ]")
    print(_hr("-"))
    print("  준비:")
    print("    1) 차량을 직선 코스 시작점에 위치")
    print("    2) 전방 정확히 2.0 m 지점에 마커 설치")
    print("    3) 대시보드 방향키로 2m 직진 후 정지")
    print("    4) 정지 후 Enter → 기록 종료")
    print()
    print("  Enter 를 누르면 기록을 시작합니다 ...")
    input()

    poses = _record_session(node, "2m 직진 후 정지 → Enter")

    if len(poses) < 10:
        print(f"  ⚠️  데이터 부족 ({len(poses)} samples). /odom 토픽이 퍼블리시되는지 확인하세요.")
        return None

    a = analyze_straight(poses)
    _print_straight_result(a)
    a["_poses"] = [(p.x, p.y, p.yaw, p.t) for p in poses]
    return a


def run_circle_test(node: "_OdomNode", direction: str) -> Optional[Dict]:
    dstr = "좌회전" if direction == "left" else "우회전"
    print()
    print(_hr())
    print(f"  [ {dstr} 원형 테스트 ]")
    print(_hr("-"))
    print("  준비:")
    print(f"    1) 차량을 출발점에 위치 (방향 기억)")
    print(f"    2) 대시보드 방향키로 {dstr}하며 원을 그려 출발점으로 복귀")
    print( "    3) 원점 복귀 완료 후 Enter → 기록 종료")
    print()
    print("  Enter 를 누르면 기록을 시작합니다 ...")
    input()

    poses = _record_session(node, f"{dstr} 원 주행 → 원점 복귀 후 → Enter")

    if len(poses) < 10:
        print(f"  ⚠️  데이터 부족 ({len(poses)} samples). /odom 토픽이 퍼블리시되는지 확인하세요.")
        return None

    a = analyze_circle(poses, direction)
    _print_circle_result(a)
    a["_poses"] = [(p.x, p.y, p.yaw, p.t) for p in poses]
    return a

# ══════════════════════════════════════════════════════════════════════════════
# Save results
# ══════════════════════════════════════════════════════════════════════════════
def save_results(results: List[Dict]):
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Build serialisable summaries without _poses (shallow copy per dict)
    clean_results = [{k: v for k, v in r.items() if k != "_poses"} for r in results]
    poses_by_idx:  Dict[int, list] = {i: r.get("_poses", []) for i, r in enumerate(results)}

    payload = dict(
        timestamp        = ts,
        wheel_vel_scale  = CUR_WHEEL_VEL_SCALE,
        wheel_dist_scale = CUR_WHEEL_DIST_SCALE,
        results          = clean_results,
    )
    json_path = RESULTS_DIR / f"calib_{ts}.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"\n  ✅  JSON 저장: {json_path}")

    # trajectory plots (original results dicts are never modified)
    if HAS_MPL:
        for i, raw in poses_by_idx.items():
            if not raw:
                continue
            poses = [Pose2D(*p) for p in raw]
            test_label = clean_results[i]["test_type"]
            img_path = RESULTS_DIR / f"traj_{ts}_{test_label}_{i}.png"
            _save_plot(poses, test_label.replace("_", " "), img_path)
            print(f"  ✅  궤적 이미지: {img_path}")
    else:
        print("  ℹ  matplotlib 없음 — 이미지 저장 생략")
        print("     pip install matplotlib  으로 설치 가능")

# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    print_header()

    if not HAS_ROS:
        print("\n  ❌  ROS 2 Python 패키지를 찾을 수 없습니다.")
        print("     source /opt/ros/humble/setup.bash  후 다시 실행하세요.")
        sys.exit(1)

    rclpy.init()
    node = _OdomNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n  ✅  ROS 2 초기화 완료. /odom 토픽 대기 중 ...")
    time.sleep(1.0)

    results: List[Dict] = []

    try:
        while True:
            print_menu()
            choice = input("\n  선택 > ").strip().lower()

            if choice == "1":
                r = run_straight_test(node)
                if r:
                    results.append(r)

            elif choice == "2":
                r = run_circle_test(node, "left")
                if r:
                    results.append(r)

            elif choice == "3":
                r = run_circle_test(node, "right")
                if r:
                    results.append(r)

            elif choice == "r":
                if results:
                    print_summary(results)
                else:
                    print("\n  아직 테스트 결과가 없습니다.")

            elif choice == "s":
                if results:
                    print_summary(results)
                    save_results(results)
                else:
                    print("\n  저장할 결과가 없습니다.")

            elif choice == "q":
                print("\n  종료합니다.\n")
                break

            else:
                print("  잘못된 입력입니다.")

    except KeyboardInterrupt:
        print("\n\n  Ctrl+C — 종료합니다.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
