#!/usr/bin/env python3
"""
SEAME Brain — Odometry Calibration GUI (Standalone Web App)

사용법:
    source /opt/ros/humble/setup.bash
    cd ~/seame_brain/Brain
    pip install flask flask-socketio   # 최초 1회
    python3 tools/calib_gui.py
    # 브라우저: http://localhost:5050

기능:
    - 직선 / 좌원 / 우원 테스트 코스 선택
    - 실시간 궤적 시각화
    - WASD 주행 제어 (ackermann_msgs 있을 때)
    - 원점 초기화 버튼 (기록 초기화 + /initialpose 발행)
    - 보정값 즉시 적용 (ros2 param set)
    - 영구 적용 안내 및 odom_generator 재시작
"""

import os
import sys
import math
import time
import threading
import subprocess
from pathlib import Path

# ── odom_calibrator 에서 분석 함수 재사용 ─────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
try:
    from odom_calibrator import (
        Pose2D, analyze_straight, analyze_circle,
        CUR_WHEEL_VEL_SCALE, CUR_WHEEL_DIST_SCALE,
    )
except ImportError as e:
    print(f"[오류] odom_calibrator.py 임포트 실패: {e}")
    sys.exit(1)

try:
    from flask import Flask, render_template_string
    from flask_socketio import SocketIO, emit
except ImportError:
    print("[오류] pip install flask flask-socketio 후 재실행하세요.")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

try:
    from ackermann_msgs.msg import AckermannDriveStamped
    HAS_ACKERMANN = True
except ImportError:
    HAS_ACKERMANN = False

# ══════════════════════════════════════════════════════════════════════════════
# Config
# ══════════════════════════════════════════════════════════════════════════════
PORT            = 5050
ODOM_NODE       = "/wheel_v_imu_odom"
ODOM_GEN_PATH   = Path("/home/seame/seame_ros/src/localization/src/scripts/odom_generator.py")
RESULTS_DIR     = Path(__file__).parent.parent / "calibration_results"

# ══════════════════════════════════════════════════════════════════════════════
# Shared state
# ══════════════════════════════════════════════════════════════════════════════
_lock      = threading.Lock()
_poses: list[Pose2D] = []
_recording = False
_ros_node  = None  # set after rclpy.init()

app      = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ══════════════════════════════════════════════════════════════════════════════
# ROS Node
# ══════════════════════════════════════════════════════════════════════════════
if HAS_ROS:
    class _CalibNode(Node):
        def __init__(self):
            super().__init__("calib_gui")
            self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
            self._init_pub = self.create_publisher(
                PoseWithCovarianceStamped, "/initialpose", 1
            )
            if HAS_ACKERMANN:
                self._cmd_pub = self.create_publisher(
                    AckermannDriveStamped, "/ackermann_cmd", 1
                )
            else:
                self._cmd_pub = None

        def _odom_cb(self, msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            p = Pose2D(x, y, yaw, time.time())
            with _lock:
                if _recording:
                    _poses.append(p)
            socketio.emit("pose", {"x": x, "y": y, "yaw": math.degrees(yaw)})

        def publish_initialpose(self):
            """odom 원점을 현재 위치로 리셋 (/initialpose 발행)."""
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.pose.pose.orientation.w = 1.0
            self._init_pub.publish(msg)

        def publish_cmd(self, speed: float, steer: float):
            if self._cmd_pub is None:
                return
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steer)
            self._cmd_pub.publish(msg)

# ══════════════════════════════════════════════════════════════════════════════
# Socket.IO events
# ══════════════════════════════════════════════════════════════════════════════
@socketio.on("connect")
def on_connect():
    emit("server_info", {
        "has_ros":         HAS_ROS,
        "has_ackermann":   HAS_ACKERMANN,
        "wheel_vel_scale": CUR_WHEEL_VEL_SCALE,
        "wheel_dist_scale": CUR_WHEEL_DIST_SCALE,
        "odom_gen_path":   str(ODOM_GEN_PATH),
    })


@socketio.on("start_recording")
def on_start():
    global _recording
    with _lock:
        _poses.clear()
        _recording = True
    emit("status", {"recording": True})


@socketio.on("stop_recording")
def on_stop(data):
    global _recording
    test_type = data.get("test_type", "straight")
    with _lock:
        _recording = False
        poses = list(_poses)

    if len(poses) < 10:
        emit("error", {"msg": f"데이터 부족 ({len(poses)} samples). /odom 토픽 확인."})
        return

    if test_type == "straight":
        result = analyze_straight(poses)
    elif test_type == "circle_left":
        result = analyze_circle(poses, "left")
    else:
        result = analyze_circle(poses, "right")

    result["_poses_xy"] = [[p.x, p.y] for p in poses]
    emit("result", result)


@socketio.on("reset_origin")
def on_reset_origin():
    global _recording
    with _lock:
        _poses.clear()
        _recording = False

    # /initialpose 발행 → odom_generator 가 다음 콜백부터 새 위치 기준으로 적분
    if _ros_node is not None:
        try:
            _ros_node.publish_initialpose()
        except Exception:
            pass

    emit("origin_reset", {"msg": "원점 초기화 완료. 차량 위치를 원점으로 설정했습니다."})


@socketio.on("apply_param")
def on_apply_param(data):
    param = data.get("param")
    value = data.get("value")
    try:
        r = subprocess.run(
            ["ros2", "param", "set", ODOM_NODE, param, str(value)],
            capture_output=True, text=True, timeout=5,
        )
        success = r.returncode == 0
        msg = r.stdout.strip() or r.stderr.strip()
    except Exception as e:
        success, msg = False, str(e)
    emit("apply_result", {"success": success, "msg": msg, "param": param, "value": value})


@socketio.on("patch_odom_gen")
def on_patch_odom_gen(data):
    """
    odom_generator.py 의 declare_parameter 기본값을 새 값으로 교체.
    param: 'v_scale' | 'yaw_scale'
    value: float
    """
    param = data.get("param")
    value = float(data.get("value"))

    if not ODOM_GEN_PATH.exists():
        emit("patch_result", {"success": False, "msg": f"파일 없음: {ODOM_GEN_PATH}"})
        return

    src = ODOM_GEN_PATH.read_text(encoding="utf-8")
    import re

    # 기존 선언이 있으면 값만 교체
    pattern = rf'(self\.declare_parameter\("{param}",\s*)[\d.]+(\))'
    if re.search(pattern, src):
        new_src = re.sub(pattern, rf'\g<1>{value}\2', src)
        action = "기본값 수정"
    else:
        # 없으면 v_scale 선언 바로 아래에 추가 (yaw_scale 인 경우)
        insert_after = 'self.declare_parameter("v_scale"'
        if insert_after in src:
            indent = "        "
            new_line = f'\n{indent}self.declare_parameter("{param}", {value})'
            new_src = src.replace(
                insert_after,
                f'{insert_after}{new_line.replace(new_line, "")}',  # placeholder
            )
            # 실제 삽입: v_scale 줄 끝 다음에 추가
            new_src = re.sub(
                r'(self\.declare_parameter\("v_scale"[^\n]+\n)',
                rf'\1        self.declare_parameter("{param}", {value})\n',
                src,
            )
            action = "파라미터 추가"
        else:
            emit("patch_result", {"success": False, "msg": "삽입 위치를 찾지 못했습니다."})
            return

    ODOM_GEN_PATH.write_text(new_src, encoding="utf-8")
    emit("patch_result", {
        "success": True,
        "msg": f"odom_generator.py {action}: {param} = {value}. 노드를 재시작하세요.",
    })


@socketio.on("restart_node")
def on_restart_node():
    try:
        subprocess.run(["pkill", "-f", "odom_generator.py"], capture_output=True, timeout=3)
        time.sleep(0.5)
        emit("restart_result", {
            "success": True,
            "msg": "odom_generator 종료 완료. 런치 파일이 자동 재시작합니다.",
        })
    except Exception as e:
        emit("restart_result", {"success": False, "msg": str(e)})


@socketio.on("drive_cmd")
def on_drive_cmd(data):
    if _ros_node is not None and HAS_ACKERMANN:
        _ros_node.publish_cmd(data.get("speed", 0.0), data.get("steer", 0.0))


# ══════════════════════════════════════════════════════════════════════════════
# HTML
# ══════════════════════════════════════════════════════════════════════════════
HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SEAME Calibration GUI</title>
<link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.2/dist/css/bootstrap.min.css" rel="stylesheet">
<script src="https://cdn.socket.io/4.7.2/socket.io.min.js"></script>
<style>
  /* ── Design tokens ──────────────────────────────────────────── */
  :root {
    --bg-page   : #0c0f1a;
    --bg-card   : #141824;
    --bg-header : #0e1120;
    --bg-inset  : #090c16;
    --border    : #252a3d;

    --text-base : #e8eaf0;   /* primary text — near white */
    --text-sub  : #b0b8cc;   /* secondary labels */
    --text-hint : #7a85a0;   /* hints / descriptions */

    --blue  : #38bdf8;
    --green : #4ade80;
    --amber : #fbbf24;
    --red   : #f87171;
    --teal  : #2dd4bf;

    --font-mono : 'Consolas','Menlo',monospace;
  }

  /* ── Base ───────────────────────────────────────────────────── */
  *, *::before, *::after { box-sizing: border-box; }
  body {
    background: var(--bg-page);
    color: var(--text-base);
    font-family: 'Segoe UI', system-ui, sans-serif;
    font-size: 0.9rem;
    line-height: 1.5;
  }

  /* ── Cards ──────────────────────────────────────────────────── */
  .card {
    background: var(--bg-card);
    border: 1px solid var(--border);
    border-radius: 10px;
    overflow: hidden;
  }
  .card-header {
    background: var(--bg-header);
    border-bottom: 1px solid var(--border);
    padding: 9px 14px;
    font-weight: 600;
    font-size: 0.85rem;
    letter-spacing: 0.3px;
    color: var(--text-base);
  }
  .card-header .icon { margin-right: 6px; }
  .card-body { padding: 12px 14px; }

  /* ── Section label (was #555 — now readable) ────────────────── */
  .sec-lbl {
    font-size: 0.72rem;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 1.2px;
    color: var(--text-hint);      /* #7a85a0 — clearly visible */
    margin: 10px 0 5px;
  }

  /* ── Test buttons ───────────────────────────────────────────── */
  .btn-test {
    width: 100%;
    margin-bottom: 7px;
    border-radius: 8px;
    padding: 10px 12px;
    text-align: left;
    font-size: 0.85rem;
    line-height: 1.4;
    color: var(--text-base);
    background: var(--bg-inset);
    border: 1px solid var(--border);
    transition: border-color .15s, background .15s;
  }
  .btn-test:hover { border-color: #3a4060; background: #10131e; color: var(--text-base); }
  .btn-test.active {
    border: 2px solid var(--blue) !important;
    background: #0b1e2e !important;
    color: var(--text-base) !important;
  }
  .btn-test .sub { font-size: 0.75rem; color: var(--text-hint); display: block; margin-top: 2px; }

  /* ── Control buttons ────────────────────────────────────────── */
  .btn-start { background:#155724; border-color:#198754; color:#d1f5db; font-weight:600; }
  .btn-start:hover:not(:disabled) { background:#1a6b2d; color:#fff; }
  .btn-stop  { background:#5c1a1a; border-color:#dc3545; color:#ffd5d5; font-weight:600; }
  .btn-stop:hover:not(:disabled)  { background:#7a2020; color:#fff; }
  .btn-origin { background:var(--bg-inset); border-color:#3a4060; color:var(--text-sub); font-weight:500; }
  .btn-origin:hover { border-color:#5a6090; color:var(--text-base); }

  /* ── Status indicator ───────────────────────────────────────── */
  .rec-dot { width:10px; height:10px; border-radius:50%; display:inline-block; flex-shrink:0; }
  .rec-dot.idle { background:#3a4060; }
  .rec-dot.live { background:#f87171; box-shadow:0 0 6px #f87171; animation:pulse 1s infinite; }
  @keyframes pulse { 0%,100%{opacity:1}50%{opacity:.3} }

  /* ── Live pose display ──────────────────────────────────────── */
  .pose-grid {
    display: grid;
    grid-template-columns: auto 1fr;
    gap: 3px 10px;
    font-family: var(--font-mono);
    font-size: 0.85rem;
  }
  .pose-lbl { color: var(--text-hint); }
  .pose-val { color: var(--blue); font-weight: 600; }

  /* ── Canvas ─────────────────────────────────────────────────── */
  #traj-canvas { display: block; width: 100%; border-radius: 6px; background: var(--bg-inset); }

  /* ── Results table ──────────────────────────────────────────── */
  .result-table {
    width: 100%;
    border-collapse: collapse;
    font-family: var(--font-mono);
    font-size: 0.83rem;
  }
  .result-table td { padding: 3px 0; vertical-align: top; }
  .result-table td:first-child {
    color: var(--text-sub);        /* #b0b8cc — clearly readable */
    padding-right: 12px;
    white-space: nowrap;
    width: 55%;
  }
  .result-table td:last-child {
    color: var(--text-base);
    font-weight: 600;
  }
  .result-table tr.divider td { padding: 6px 0 2px; border-top: 1px solid var(--border); }
  .val-suggest { color: var(--amber) !important; }
  .val-ok      { color: var(--green) !important; }
  .val-warn    { color: var(--amber) !important; }
  .val-bad     { color: var(--red)   !important; }

  /* ── Code box ───────────────────────────────────────────────── */
  .cmd-box {
    background: var(--bg-inset);
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 8px 12px;
    font-family: var(--font-mono);
    font-size: 0.8rem;
    color: #c8d0e8;              /* was #aaa — much more readable */
    white-space: pre-wrap;
    word-break: break-all;
    line-height: 1.6;
  }

  /* ── Warning notice ─────────────────────────────────────────── */
  .notice {
    background: #1e1508;
    border: 1px solid #4a3a10;
    border-radius: 6px;
    padding: 7px 10px;
    font-size: 0.8rem;
    color: #f5cc6a;              /* amber, clearly visible */
    line-height: 1.5;
  }

  /* ── Apply param row ────────────────────────────────────────── */
  .apply-row {
    display: flex;
    align-items: center;
    gap: 8px;
    background: var(--bg-inset);
    border: 1px solid var(--border);
    border-radius: 7px;
    padding: 7px 10px;
    margin-bottom: 6px;
  }
  .apply-row .p-name { color: var(--text-sub); font-family: var(--font-mono); font-size: 0.83rem; flex: 1; }
  .apply-row .p-val  { color: var(--amber); font-family: var(--font-mono); font-size: 0.83rem; font-weight: 700; }

  /* ── Drive pad ──────────────────────────────────────────────── */
  #drive-pad { display: grid; grid-template-columns: repeat(3,50px); gap: 5px; }
  #drive-pad button {
    width: 50px; height: 50px; border-radius: 8px;
    border: 1px solid var(--border);
    background: var(--bg-inset);
    color: var(--text-sub);
    font-size: 1.1rem; cursor: pointer; user-select: none;
    transition: background .1s, color .1s;
  }
  #drive-pad button:hover  { background: #1a1f30; }
  #drive-pad button.pressed { background: var(--blue); color: #000; border-color: var(--blue); }

  /* ── Log ────────────────────────────────────────────────────── */
  .log-entry {
    display: flex; align-items: flex-start; gap: 8px;
    padding: 7px 10px; border-radius: 7px;
    margin-bottom: 5px; font-size: 0.82rem; line-height: 1.4;
  }
  .log-ok   { background: #0e2018; border: 1px solid #1d4030; color: #86efac; }
  .log-err  { background: #200e0e; border: 1px solid #401d1d; color: #fca5a5; }
  .log-info { background: #0c1525; border: 1px solid #1a2f4a; color: #93c5fd; }
  .log-entry .log-x { margin-left: auto; cursor: pointer; opacity: .6; flex-shrink: 0; }
  .log-entry .log-x:hover { opacity: 1; }

  /* ── Separator ──────────────────────────────────────────────── */
  .sep { border: none; border-top: 1px solid var(--border); margin: 10px 0; }

  /* ── Connection badge ───────────────────────────────────────── */
  #conn-badge { font-size: 0.8rem; }

  /* ── Scrollable right column ────────────────────────────────── */
  .right-col { max-height: calc(100vh - 90px); overflow-y: auto; }
</style>
</head>
<body>
<div class="container-fluid py-2 px-3">

  <!-- ── Header ─────────────────────────────────────────────── -->
  <div class="d-flex align-items-center justify-content-between mb-2"
       style="border-bottom:1px solid var(--border); padding-bottom:8px;">
    <span style="font-size:1rem; font-weight:700; color:var(--text-base);">
      ⚙&nbsp; SEAME Brain &mdash; Odometry Calibration
    </span>
    <div class="d-flex align-items-center gap-3">
      <span id="conn-badge" style="color:var(--text-hint);">연결 중…</span>
      <div class="d-flex align-items-center gap-2">
        <span class="rec-dot idle" id="rec-dot"></span>
        <span id="rec-lbl" style="color:var(--text-hint); font-size:0.83rem;">대기</span>
      </div>
      <span id="sample-cnt" style="color:var(--text-hint); font-family:var(--font-mono); font-size:0.8rem;">0 samples</span>
    </div>
  </div>

  <div class="row g-2">

    <!-- ── Left ──────────────────────────────────────────────── -->
    <div class="col-md-3 d-flex flex-column gap-2">

      <!-- Test course -->
      <div class="card">
        <div class="card-header"><span class="icon">🗺</span>테스트 코스</div>
        <div class="card-body">
          <button class="btn-test active" id="btn-straight" onclick="selectTest('straight')">
            📏 직선 2m 테스트
            <span class="sub">전진 거리로 v_scale 보정</span>
          </button>
          <button class="btn-test" id="btn-circle_left" onclick="selectTest('circle_left')">
            ↺ 좌회전 원형 테스트
            <span class="sub">원점 복귀로 yaw_scale 보정</span>
          </button>
          <button class="btn-test" id="btn-circle_right" onclick="selectTest('circle_right')">
            ↻ 우회전 원형 테스트
            <span class="sub">원점 복귀로 yaw_scale 보정</span>
          </button>
        </div>
      </div>

      <!-- Controls -->
      <div class="card">
        <div class="card-header"><span class="icon">🎮</span>컨트롤</div>
        <div class="card-body d-flex flex-column gap-2">
          <button class="btn btn-start w-100" id="btn-start" onclick="startRec()">▶ 기록 시작</button>
          <button class="btn btn-stop  w-100" id="btn-stop"  onclick="stopRec()" disabled>■ 중지 &amp; 분석</button>
          <hr class="sep">
          <button class="btn btn-origin w-100" onclick="resetOrigin()">⊙ 원점 초기화</button>
          <p style="font-size:0.75rem; color:var(--text-hint); margin:0;">
            현재 차량 위치를 원점으로 재설정하고<br>기록을 초기화합니다.
          </p>
        </div>
      </div>

      <!-- Live pose -->
      <div class="card">
        <div class="card-header"><span class="icon">📍</span>현재 위치</div>
        <div class="card-body">
          <div class="pose-grid">
            <span class="pose-lbl">X</span>     <span class="pose-val" id="p-x">—</span>
            <span class="pose-lbl">Y</span>     <span class="pose-val" id="p-y">—</span>
            <span class="pose-lbl">Yaw</span>   <span class="pose-val" id="p-yaw">—</span>
            <span class="pose-lbl">거리</span>  <span class="pose-val" id="p-dist">—</span>
          </div>
        </div>
      </div>

      <!-- Drive pad -->
      <div class="card" id="drive-card" style="display:none">
        <div class="card-header"><span class="icon">🕹</span>주행 제어</div>
        <div class="card-body">
          <div id="drive-pad">
            <div></div>
            <button id="key-w" onmousedown="dk('w',1)" onmouseup="dk('w',0)"
              ontouchstart="dk('w',1)" ontouchend="dk('w',0)">▲</button>
            <div></div>
            <button id="key-a" onmousedown="dk('a',1)" onmouseup="dk('a',0)"
              ontouchstart="dk('a',1)" ontouchend="dk('a',0)">◀</button>
            <button id="key-s" onmousedown="dk('s',1)" onmouseup="dk('s',0)"
              ontouchstart="dk('s',1)" ontouchend="dk('s',0)">■</button>
            <button id="key-d" onmousedown="dk('d',1)" onmouseup="dk('d',0)"
              ontouchstart="dk('d',1)" ontouchend="dk('d',0)">▶</button>
            <div></div><div></div><div></div>
          </div>
          <p style="font-size:0.75rem; color:var(--text-hint); margin:8px 0 0;">
            키보드 WASD 또는 위 버튼 / Space 정지
          </p>
        </div>
      </div>

    </div><!-- /left -->

    <!-- ── Center: canvas ─────────────────────────────────────── -->
    <div class="col-md-5">
      <div class="card h-100">
        <div class="card-header d-flex justify-content-between align-items-center">
          <span><span class="icon">🗺</span>궤적 미리보기</span>
          <button class="btn btn-sm" style="color:var(--text-hint);border:1px solid var(--border);background:var(--bg-inset);"
                  onclick="clearCanvas()">지우기</button>
        </div>
        <div class="card-body p-2">
          <canvas id="traj-canvas" height="430"></canvas>
        </div>
      </div>
    </div>

    <!-- ── Right: results + apply ─────────────────────────────── -->
    <div class="col-md-4 right-col d-flex flex-column gap-2">

      <div class="card" id="result-card" style="display:none">
        <div class="card-header"><span class="icon">📊</span>분석 결과</div>
        <div class="card-body" id="result-body"></div>
      </div>

      <div class="card" id="apply-card" style="display:none">
        <div class="card-header"><span class="icon">🔧</span>보정값 적용</div>
        <div class="card-body" id="apply-body"></div>
      </div>

      <div id="log-area"></div>

    </div>

  </div>
</div>

<script>
const socket = io();
const SPEED = 0.30, STEER = 0.20;
let testType = 'straight', recording = false;
let poses = [], originX = null, originY = null;
const keys = {};

// ── Socket ────────────────────────────────────────────────────────────────
socket.on('connect',    () => badge('연결됨 ✓', 'var(--green)'));
socket.on('disconnect', () => badge('연결 끊김', 'var(--red)'));

socket.on('server_info', d => {
  if (d.has_ackermann) document.getElementById('drive-card').style.display = '';
  if (!d.has_ros) log('err', 'ROS 2 미연결 — /odom 데이터 없음');
  log('info', `WHEEL_VEL_SCALE=${d.wheel_vel_scale}  |  WHEEL_DIST_SCALE=${d.wheel_dist_scale}`);
});

socket.on('pose', p => {
  document.getElementById('p-x').textContent   = p.x.toFixed(4) + ' m';
  document.getElementById('p-y').textContent   = p.y.toFixed(4) + ' m';
  document.getElementById('p-yaw').textContent = p.yaw.toFixed(1) + '°';
  if (originX !== null)
    document.getElementById('p-dist').textContent =
      Math.hypot(p.x - originX, p.y - originY).toFixed(4) + ' m';
  if (!recording) return;
  if (originX === null) { originX = p.x; originY = p.y; }
  poses.push({x: p.x, y: p.y});
  document.getElementById('sample-cnt').textContent = poses.length + ' samples';
  drawCanvas();
});

socket.on('result', d => {
  if (d._poses_xy) { poses = d._poses_xy.map(([x,y]) => ({x,y})); drawCanvas(); }
  renderResult(d);
  renderApply(d);
});

socket.on('error',        d => log('err',  d.msg));
socket.on('apply_result', d => log(d.success ? 'ok' : 'err', `${d.param} = ${d.value} → ${d.msg}`));
socket.on('patch_result', d => log(d.success ? 'ok' : 'err', d.msg));
socket.on('restart_result', d => log(d.success ? 'ok' : 'err', d.msg));
socket.on('origin_reset', d => {
  poses = []; originX = originY = null;
  document.getElementById('p-dist').textContent = '—';
  document.getElementById('sample-cnt').textContent = '0 samples';
  drawCanvas(); log('info', d.msg);
});

// ── Controls ──────────────────────────────────────────────────────────────
function selectTest(t) {
  testType = t;
  document.querySelectorAll('.btn-test').forEach(b => b.classList.remove('active'));
  document.getElementById('btn-' + t).classList.add('active');
}
function startRec() {
  poses = []; originX = originY = null;
  document.getElementById('sample-cnt').textContent = '0 samples';
  drawCanvas();
  socket.emit('start_recording');
  setRec(true);
}
function stopRec()     { socket.emit('stop_recording', {test_type: testType}); setRec(false); }
function resetOrigin() { socket.emit('reset_origin'); setRec(false); }
function setRec(r) {
  recording = r;
  document.getElementById('btn-start').disabled = r;
  document.getElementById('btn-stop').disabled  = !r;
  document.getElementById('rec-dot').className  = 'rec-dot ' + (r ? 'live' : 'idle');
  document.getElementById('rec-lbl').textContent = r ? '기록 중' : '대기';
  document.getElementById('rec-lbl').style.color = r ? 'var(--red)' : 'var(--text-hint)';
}

// ── Drive pad ─────────────────────────────────────────────────────────────
function sendDrive() {
  const speed = keys['w'] ? SPEED : keys['s'] ? -SPEED : 0;
  const steer = keys['a'] ? -STEER : keys['d'] ? STEER : 0;
  socket.emit('drive_cmd', {speed, steer});
}
function dk(k, down) {
  keys[k] = !!down;
  const b = document.getElementById('key-' + k);
  if (b) b.classList.toggle('pressed', !!down);
  sendDrive();
}
document.addEventListener('keydown', e => {
  const k = e.key === ' ' ? ' ' : e.key.toLowerCase();
  if (k === ' ') { Object.keys(keys).forEach(kk => keys[kk] = false); sendDrive(); return; }
  if ('wasd'.includes(k) && !keys[k]) { keys[k] = true; sendDrive(); }
});
document.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if ('wasd'.includes(k)) { keys[k] = false; sendDrive(); }
});

// ── Canvas ────────────────────────────────────────────────────────────────
function clearCanvas() {
  poses = []; originX = originY = null;
  document.getElementById('p-dist').textContent = '—';
  drawCanvas();
}
function drawCanvas() {
  const canvas = document.getElementById('traj-canvas');
  canvas.width  = canvas.parentElement.clientWidth - 16;
  canvas.height = 430;
  const W = canvas.width, H = canvas.height;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#090c16'; ctx.fillRect(0,0,W,H);

  if (poses.length < 2) {
    ctx.fillStyle = '#3a4060'; ctx.font = '14px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('주행 후 경로가 여기에 표시됩니다', W/2, H/2);
    return;
  }
  const xs = poses.map(p=>p.x), ys = poses.map(p=>p.y);
  const minX=Math.min(...xs), maxX=Math.max(...xs);
  const minY=Math.min(...ys), maxY=Math.max(...ys);
  const pad=48;
  const scale = Math.min((W-pad*2)/(maxX-minX||1), (H-pad*2)/(maxY-minY||1));
  const ox = (W-(maxX-minX)*scale)/2 - minX*scale;
  const oy = (H-(maxY-minY)*scale)/2 - minY*scale;
  const tx = x =>  x*scale+ox;
  const ty = y => H-(y*scale+oy);

  // Grid
  const step = bestStep(scale);
  ctx.strokeStyle = '#181d2e'; ctx.lineWidth = 1;
  for (let gx=Math.floor(minX/step)*step; gx<=maxX+step; gx+=step) {
    ctx.beginPath(); ctx.moveTo(tx(gx),0); ctx.lineTo(tx(gx),H); ctx.stroke();
  }
  for (let gy=Math.floor(minY/step)*step; gy<=maxY+step; gy+=step) {
    ctx.beginPath(); ctx.moveTo(0,ty(gy)); ctx.lineTo(W,ty(gy)); ctx.stroke();
  }
  // Grid labels
  ctx.fillStyle='#3a4060'; ctx.font='10px monospace';
  ctx.textAlign='center';
  for (let gx=Math.floor(minX/step)*step; gx<=maxX+step; gx+=step)
    ctx.fillText(gx.toFixed(1)+'m', tx(gx), H-5);
  ctx.textAlign='right';
  for (let gy=Math.floor(minY/step)*step; gy<=maxY+step; gy+=step)
    ctx.fillText(gy.toFixed(1)+'m', W-5, ty(gy)+4);

  // Path
  ctx.beginPath(); ctx.strokeStyle='#38bdf8'; ctx.lineWidth=2.5;
  poses.forEach((p,i) => i===0 ? ctx.moveTo(tx(p.x),ty(p.y)) : ctx.lineTo(tx(p.x),ty(p.y)));
  ctx.stroke();

  // Closure
  if (poses.length>1) {
    ctx.beginPath(); ctx.strokeStyle='#f87171'; ctx.lineWidth=1.5; ctx.setLineDash([5,4]);
    ctx.moveTo(tx(poses[0].x),ty(poses[0].y));
    ctx.lineTo(tx(poses[poses.length-1].x),ty(poses[poses.length-1].y));
    ctx.stroke(); ctx.setLineDash([]);
  }
  // Markers
  dot(ctx, tx(poses[0].x), ty(poses[0].y), '#4ade80', 9);
  dot(ctx, tx(poses[poses.length-1].x), ty(poses[poses.length-1].y), '#f87171', 9);
  ctx.font='12px sans-serif'; ctx.textAlign='left';
  ctx.fillStyle='#4ade80'; ctx.fillText('S', tx(poses[0].x)+12, ty(poses[0].y)+5);
  ctx.fillStyle='#f87171'; ctx.fillText('E', tx(poses[poses.length-1].x)+12, ty(poses[poses.length-1].y)+5);
}
function dot(ctx,x,y,c,r){ ctx.beginPath(); ctx.fillStyle=c; ctx.arc(x,y,r,0,Math.PI*2); ctx.fill(); }
function bestStep(s){ for(const c of[0.1,.25,.5,1,2,5,10]) if(c*s>55) return c; return 10; }

// ── Result rendering ──────────────────────────────────────────────────────
function renderResult(d) {
  document.getElementById('result-card').style.display = '';
  let h = '<table class="result-table">';
  const tr = (lbl,val,cls='') => `<tr><td>${lbl}</td><td class="${cls}">${val}</td></tr>`;

  if (d.test_type === 'straight') {
    const ep = Math.abs(d.path_error_pct);
    const ec = ep<1?'val-ok':ep<5?'val-warn':'val-bad';
    h += tr('예상 거리',       fmt(d.expected_m,3)+' m');
    h += tr('오도메트리 변위', fmt(d.odom_disp_m,4)+' m');
    h += tr('변위 오차',       sg(d.path_error_m)+fmt(d.path_error_m,4)+' m&nbsp;('+sg(d.path_error_pct)+fmt(d.path_error_pct,2)+'%)', ec);
    h += tr('측방 편차',       fmt(d.lateral_dev_m,4)+' m');
    h += tr('yaw 드리프트',    sg(d.yaw_drift_during_deg)+fmt(Math.abs(d.yaw_drift_during_deg),2)+'°');
    h += tr('보정 계수',       fmt(d.correction_factor,6));
    h += `<tr class="divider"><td>현재 v_scale</td><td>${fmt(d.cur_v_scale,6)}</td></tr>`;
    h += tr('→ 권장 v_scale',  fmt(d.sug_v_scale_odom,6), 'val-suggest');
  } else {
    const ye = Math.abs(d.yaw_error_deg);
    const yc = ye<3?'val-ok':ye<10?'val-warn':'val-bad';
    h += tr('경로 길이',       fmt(d.odom_path_m,4)+' m');
    h += tr('추정 반지름',     fmt(d.est_radius_m,4)+' m');
    h += tr('위치 폐합 오차',  fmt(d.closure_error_m,4)+' m');
    h += tr('누적 yaw',        sg(d.total_yaw_deg)+fmt(Math.abs(d.total_yaw_deg),2)+'°');
    h += tr('예상 yaw',        sg(d.expected_yaw_deg)+fmt(Math.abs(d.expected_yaw_deg),1)+'°');
    h += tr('yaw 오차',        sg(d.yaw_error_deg)+fmt(Math.abs(d.yaw_error_deg),2)+'°', yc);
    h += `<tr class="divider"><td>현재 yaw_scale</td><td>${fmt(d.cur_yaw_scale,6)}</td></tr>`;
    h += tr('보정 계수',       fmt(d.yaw_correction_factor,6));
    h += tr('→ 권장 yaw_scale',fmt(d.sug_yaw_scale,6), 'val-suggest');
    if (d.diagnosis?.length) {
      h += `<tr class="divider"><td colspan="2" style="color:var(--text-hint);font-size:0.72rem;font-weight:600;text-transform:uppercase;letter-spacing:1px;">진단</td></tr>`;
      d.diagnosis.forEach(m => {
        h += `<tr><td colspan="2" style="color:#5eead4;padding-top:2px;">→ ${m}</td></tr>`;
      });
    }
  }
  h += '</table>';
  document.getElementById('result-body').innerHTML = h;
}

// ── Apply rendering ───────────────────────────────────────────────────────
function renderApply(d) {
  document.getElementById('apply-card').style.display = '';
  let h = '';

  if (d.test_type === 'straight') {
    const v = fmt(d.sug_v_scale_odom, 6);
    h += secLbl('즉시 적용 (런타임 — 재시작 시 초기화)');
    h += applyRow('v_scale', v, `applyParam('v_scale',${d.sug_v_scale_odom})`);
    h += cmdBox(`ros2 param set /wheel_v_imu_odom v_scale ${v}`);

    h += `<hr class="sep">` + secLbl('영구 적용 — odom_generator.py 수정');
    h += `<div class="d-flex gap-2 mb-2">
      <button class="btn btn-sm flex-fill" style="background:#1c1200;border:1px solid #4a3010;color:#fbbf24;"
              onclick="patchGen('v_scale',${d.sug_v_scale_odom})">파일 수정</button>
      <button class="btn btn-sm flex-fill" style="background:#200c0c;border:1px solid #5a1515;color:#fca5a5;"
              onclick="restartNode()">노드 재시작</button>
    </div>`;

    h += `<hr class="sep">` + secLbl('방법 A — 환경 변수 (Brain 재시작 필요)');
    h += cmdBox(`export WHEEL_VEL_SCALE=${fmt(d.sug_wheel_vel_scale,6)}\nexport WHEEL_DIST_SCALE=${fmt(d.sug_wheel_dist_scale,6)}\nros2 param set /wheel_v_imu_odom v_scale 1.0`);
    h += copyBtn(`export WHEEL_VEL_SCALE=${fmt(d.sug_wheel_vel_scale,6)}\nexport WHEEL_DIST_SCALE=${fmt(d.sug_wheel_dist_scale,6)}\nros2 param set /wheel_v_imu_odom v_scale 1.0`);

  } else {
    const yv = fmt(d.sug_yaw_scale, 6);
    h += secLbl('즉시 적용');
    h += `<div class="notice mb-2">⚠ odom_generator.py에 yaw_scale 파라미터가 없으면 실패합니다.<br>먼저 아래 "파일 수정"으로 파라미터를 추가하세요.</div>`;
    h += applyRow('yaw_scale', yv, `applyParam('yaw_scale',${d.sug_yaw_scale})`);
    h += cmdBox(`ros2 param set /wheel_v_imu_odom yaw_scale ${yv}`);

    h += `<hr class="sep">` + secLbl('영구 적용 — odom_generator.py에 yaw_scale 추가');
    h += `<div class="d-flex gap-2 mb-2">
      <button class="btn btn-sm flex-fill" style="background:#1c1200;border:1px solid #4a3010;color:#fbbf24;"
              onclick="patchGen('yaw_scale',${d.sug_yaw_scale})">파라미터 선언 추가</button>
      <button class="btn btn-sm flex-fill" style="background:#200c0c;border:1px solid #5a1515;color:#fca5a5;"
              onclick="restartNode()">노드 재시작</button>
    </div>`;
    h += secLbl('wz에 적용할 코드 (수동 추가 필요)');
    h += cmdBox(`yaw_scale = float(self.get_parameter("yaw_scale").value)\nwz = float(imu_msg.angular_velocity.z) * yaw_scale`);
    h += copyBtn(`yaw_scale = float(self.get_parameter("yaw_scale").value)\nwz = float(imu_msg.angular_velocity.z) * yaw_scale`);
  }

  document.getElementById('apply-body').innerHTML = h;
}

function secLbl(t) {
  return `<div class="sec-lbl">${t}</div>`;
}
function applyRow(name, val, fn) {
  return `<div class="apply-row mb-2">
    <span class="p-name">${name}</span>
    <span class="p-val">${val}</span>
    <button class="btn btn-sm ms-auto" style="background:#0b1e2e;border:1px solid #38bdf8;color:#38bdf8;font-size:0.8rem;"
            onclick="${fn}">즉시 적용</button>
  </div>`;
}
function cmdBox(text) {
  const esc = text.replace(/</g,'&lt;').replace(/>/g,'&gt;');
  return `<div class="cmd-box mb-1">${esc}</div>`;
}
function copyBtn(text) {
  return `<button class="btn btn-sm w-100 mb-2"
    style="background:var(--bg-inset);border:1px solid var(--border);color:var(--text-sub);font-size:0.8rem;"
    onclick="copyText(${JSON.stringify(text)})">📋 복사</button>`;
}

function applyParam(p, v) { socket.emit('apply_param', {param:p, value:v}); }
function patchGen(p, v)   { socket.emit('patch_odom_gen', {param:p, value:v}); }
function restartNode() {
  if (!confirm('odom_generator 노드를 재시작하시겠습니까?')) return;
  socket.emit('restart_node');
}
function copyText(t) {
  navigator.clipboard.writeText(t).then(() => log('info', '클립보드에 복사됨'));
}

// ── Helpers ───────────────────────────────────────────────────────────────
function fmt(v,d)  { return Number(v).toFixed(d); }
function sg(v)     { return v >= 0 ? '+' : ''; }
function badge(txt, color) {
  const el = document.getElementById('conn-badge');
  el.textContent = txt; el.style.color = color;
}
function log(type, msg) {
  const area = document.getElementById('log-area');
  const d = document.createElement('div');
  d.className = 'log-entry log-' + type;
  d.innerHTML = `<span>${msg}</span><span class="log-x" onclick="this.parentElement.remove()">✕</span>`;
  area.prepend(d);
  while (area.children.length > 6) area.lastChild.remove();
}

window.addEventListener('resize', drawCanvas);
drawCanvas();
</script>
</body>
</html>
"""


# ══════════════════════════════════════════════════════════════════════════════
# Routes
# ══════════════════════════════════════════════════════════════════════════════
@app.route("/")
def index():
    return render_template_string(HTML)


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════
def main():
    global _ros_node

    if not HAS_ROS:
        print("⚠  ROS 2 없이 실행 중 — UI 표시만 가능 (/odom 데이터 없음)")
    else:
        rclpy.init()
        _ros_node = _CalibNode()
        executor  = MultiThreadedExecutor()
        executor.add_node(_ros_node)
        threading.Thread(target=executor.spin, daemon=True).start()
        print("✅  ROS 2 초기화 완료")

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"🌐  브라우저에서 열기: http://localhost:{PORT}")
    socketio.run(app, host="0.0.0.0", port=PORT, debug=False,
                 allow_unsafe_werkzeug=True)


if __name__ == "__main__":
    main()
