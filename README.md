# SEA:ME BFMC Dashboard

SEA:ME 팀의 BFMC 자율주행 차량용 대시보드입니다. 이 프로젝트는 BFMC 공식 Brain 프로젝트를 기반으로 SEA:ME 차량 구조, ROS2 파이프라인, Jetson 실행 환경에 맞게 커스터마이징했습니다.

- 원본 BFMC Brain 저장소: https://github.com/ECC-BFMC/Brain
- 원본 프로젝트는 Raspberry Pi 기반 차량 제어, Nucleo 통신, 센서 데이터 처리, 환경 서버 API, 시뮬레이션 서버 코드를 포함합니다.
- 본 저장소는 위 구조를 유지하면서 SEA:ME 팀의 대시보드와 ROS2 연동 기능을 추가했습니다.

## 주요 변경 사항

- `Brain/main.py` 기준으로 대시보드, 카메라, 신호등/Traffic 서버 통신, Serial Handler, Ackermann bridge, Global Planning bridge를 함께 실행하도록 구성했습니다.
- ROS2 RealSense compressed image 토픽을 구독해 대시보드의 live camera 화면으로 전달합니다.
- `/ackermann_cmd`를 BFMC Serial Handler가 사용하는 speed/steer 명령으로 변환하는 Ackermann bridge를 추가했습니다.
- Global Planning goal node, global path, global pose, ordered checkpoints를 ROS2와 대시보드 사이에서 주고받도록 bridge를 추가했습니다.
- 지도 GraphML node, traffic light/semaphore 상태, 차량 pose를 대시보드 map UI에 표시할 수 있도록 API와 WebSocket 흐름을 확장했습니다.
- Jetson CPU temperature, CPU/memory/network 상태 등 차량 컴퓨터 모니터링 정보를 대시보드에서 확인할 수 있도록 보강했습니다.

## 실행 준비

기본 실행 위치는 `Brain` 디렉터리입니다.

```bash
cd Brain
```

설치 스크립트를 사용할 경우:

```bash
chmod +x setup.sh
./setup.sh
```

수동으로 설치할 경우:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt

cd src/dashboard/frontend
npm install
cd ../../..
```

ROS2 bridge 기능을 켜서 실행하려면 ROS2 환경과 메시지 패키지가 먼저 준비되어 있어야 합니다. 특히 `main.py`의 기본 설정은 Ackermann bridge를 켜기 때문에 `rclpy`, `ackermann_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `sensor_msgs` 등을 사용할 수 있는 ROS2 환경에서 실행해야 합니다.

예시:

```bash
source /opt/ros/humble/setup.bash
source <your_ros2_ws>/install/setup.bash
```

## 실행 방법

`main.py`를 기준으로 백엔드 프로세스를 실행합니다.

```bash
cd Brain
source .venv/bin/activate
python3 main.py
```

대시보드 프론트엔드는 별도 터미널에서 실행합니다.

```bash
cd Brain/src/dashboard/frontend
npm start
```

브라우저에서 다음 주소로 접속합니다.

```text
http://localhost:4200
```

대시보드 백엔드는 `main.py`의 `processDashboard`가 `0.0.0.0:5005`에서 실행합니다. 프론트엔드는 이 백엔드와 WebSocket/API로 통신합니다.

## 실행 옵션

`Brain/main.py` 상단의 enable flag로 필요한 프로세스를 켜고 끌 수 있습니다.

```python
ENABLE_GATEWAY = True
ENABLE_DASHBOARD = True
ENABLE_CAMERA = True
ENABLE_SEMAPHORES = True
ENABLE_TRAFFIC_COM = True
ENABLE_SERIAL_HANDLER = True
ENABLE_CMDVELBRIDGE = False
ENABLE_ACKERMAN = True
ENABLE_GLOBAL_PLANNING_BRIDGE = True
```

자주 사용하는 환경 변수:

```bash
export TRAFFIC_GPS_CAR_ID=5
export ROS_CAMERA_TOPIC=/d455f/d455f/color/image_raw/compressed
export ROS_CAMERA_MAX_FPS=1
export GLOBAL_PLANNING_GRAPHML=/path/to/track2.graphml
```

ROS2 또는 차량 하드웨어 없이 UI만 확인하려면 `main.py`에서 필요한 bridge, camera, serial 관련 flag를 `False`로 바꾼 뒤 실행합니다.

## 종료

백엔드와 프론트엔드 터미널에서 각각 `Ctrl + C`로 종료합니다. `main.py`는 `KeyboardInterrupt`를 받으면 실행 중인 child process를 순서대로 정리합니다.
