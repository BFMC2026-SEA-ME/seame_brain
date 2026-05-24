"""
TrafficCommunicationServer TCP 수신 테스트.

서버에서 실제로 어떤 포맷의 데이터가 오는지 raw로 출력합니다.
GPS 관련 패킷은 파싱 결과도 함께 출력합니다.

사용법:
    python3 tools/test_tcp_localization.py
    python3 tools/test_tcp_localization.py --host 192.168.86.39 --port 5000
    python3 tools/test_tcp_localization.py --host 192.168.86.39 --duration 30
"""

import argparse
import json
import select
import socket
import time


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="192.168.86.39")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--duration", type=float, default=20.0, help="수신 시간(초)")
    p.add_argument("--raw", action="store_true", help="파싱 없이 raw 바이트만 출력")
    return p.parse_args()


def classify_payload(payload):
    """패킷 종류 분류 및 GPS 추출."""
    if not isinstance(payload, dict):
        return "unknown", None

    device = str(payload.get("device", "")).strip().lower()
    msg_type = str(payload.get("type", "")).strip().lower()

    # 신호등
    if device == "semaphore" or "state" in payload:
        return "semaphore", None

    # 차량 GPS (공식 문서 포맷: {"device":"car","id":N,"x":F,"y":F})
    is_car = (
        device == "car"
        or msg_type in ("car", "location", "gps")
        or (device == "" and "x" in payload and "y" in payload and "state" not in payload)
    )
    if is_car:
        try:
            x = float(payload["x"])
            y = float(payload["y"])
            car_id = payload.get("id", "?")
            return "gps_car", {"id": car_id, "x": x, "y": y}
        except Exception:
            return "gps_car(parse_fail)", None

    # 서버 → 차량 송신 확인 응답 등 기타
    return f"other(device={device!r},type={msg_type!r})", None


def run(host, port, duration, raw_mode):
    print(f"[TEST] Connecting to {host}:{port} ...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((host, port))
        sock.settimeout(None)
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        return

    print(f"[TEST] Connected. Listening for {duration}s  (Ctrl+C to stop)\n")

    buf = ""
    decoder = json.JSONDecoder()
    stats = {"total": 0, "gps": 0, "semaphore": 0, "other": 0, "parse_fail": 0}
    deadline = time.monotonic() + duration

    try:
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            readable, _, _ = select.select([sock], [], [], min(remaining, 1.0))
            if not readable:
                continue

            chunk = sock.recv(4096)
            if not chunk:
                print("[TEST] Server closed connection.")
                break

            if raw_mode:
                print(f"[RAW] {chunk!r}")
                continue

            buf += chunk.decode("utf-8", errors="ignore")

            # JSON 프레임 파싱
            while buf:
                buf = buf.lstrip()
                if not buf:
                    break
                if buf[0] not in "{[":
                    idx = min((i for i in (buf.find("{"), buf.find("[")) if i >= 0), default=-1)
                    if idx == -1:
                        buf = ""
                        break
                    buf = buf[idx:]
                    continue

                try:
                    payload, end = decoder.raw_decode(buf)
                except ValueError:
                    break  # 불완전 프레임, 다음 recv 대기

                buf = buf[end:]
                stats["total"] += 1

                kind, gps = classify_payload(payload)

                if kind.startswith("gps_car"):
                    stats["gps"] += 1
                    if gps:
                        ts = time.strftime("%H:%M:%S")
                        print(f"[{ts}] GPS  id={gps['id']}  x={gps['x']:.3f}  y={gps['y']:.3f}  raw={json.dumps(payload)}")
                    else:
                        stats["parse_fail"] += 1
                        print(f"[GPS-FAIL] raw={json.dumps(payload)}")
                elif kind.startswith("semaphore"):
                    stats["semaphore"] += 1
                    print(f"[SEM ] raw={json.dumps(payload)}")
                else:
                    stats["other"] += 1
                    print(f"[OTHER] {kind}  raw={json.dumps(payload)}")

    except KeyboardInterrupt:
        print("\n[TEST] Interrupted.")
    finally:
        sock.close()

    print(f"\n[STATS] total={stats['total']}  gps={stats['gps']}  "
          f"semaphore={stats['semaphore']}  other={stats['other']}  parse_fail={stats['parse_fail']}")


if __name__ == "__main__":
    args = parse_args()
    run(args.host, args.port, args.duration, args.raw)
