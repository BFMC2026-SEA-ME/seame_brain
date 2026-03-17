# SEAME Brain — 프로젝트 전체 문서

> BFMC(Bosch Future Mobility Challenge) 자율주행 로봇의 중앙 제어 시스템
> Raspberry Pi / Jetson Nano 위에서 동작하는 멀티프로세스 Python 애플리케이션

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [전체 파일 트리](#2-전체-파일-트리)
3. [진입점 및 설정](#3-진입점-및-설정)
4. [메시지 버스 시스템](#4-메시지-버스-시스템)
5. [상태 머신](#5-상태-머신)
6. [템플릿 기반 클래스](#6-템플릿-기반-클래스)
7. [게이트웨이 (메시지 라우터)](#7-게이트웨이-메시지-라우터)
8. [하드웨어 시리얼 핸들러](#8-하드웨어-시리얼-핸들러)
9. [ROS 2 브리지](#9-ros-2-브리지)
10. [대시보드](#10-대시보드)
11. [유틸리티](#11-유틸리티)
12. [외부 서브모듈](#12-외부-서브모듈)
13. [서비스 설정](#13-서비스-설정)
14. [전체 데이터 흐름](#14-전체-데이터-흐름)
15. [환경 변수 설정](#15-환경-변수-설정)
16. [자율주행 시스템 (seame_ros)](#16-자율주행-시스템-seame_ros)
17. [자율주행 데이터 파이프라인 상세](#17-자율주행-데이터-파이프라인-상세)
18. [위치 추정 시스템](#18-위치-추정-시스템)
19. [EKF 개념](#19-ekf-개념)
20. [Brain 자율주행 성능 개선 포인트](#20-brain-자율주행-성능-개선-포인트)

---

## 1. 시스템 개요

```
┌─────────────────────────────────────────────────────────┐
│                    SEAME Brain                          │
│                  (Raspberry Pi / Jetson Nano)           │
│                                                         │
│  ┌──────────┐   ┌──────────┐   ┌─────────────────────┐ │
│  │Dashboard │   │  ROS 2   │   │   Serial (NUCLEO)   │ │
│  │Flask+SIO │   │ Bridges  │   │   STM32 via USB     │ │
│  └──────────┘   └──────────┘   └─────────────────────┘ │
│        │               │                │               │
│        └───────────────┴────────────────┘               │
│                        │                                │
│              ┌─────────▼──────────┐                     │
│              │   processGateway   │                     │
│              │  (Message Router)  │                     │
│              └────────────────────┘                     │
│         5개 multiprocessing.Queue 기반 메시지 버스          │
└─────────────────────────────────────────────────────────┘
```

### 연동 대상
| 대상 | 프로토콜 | 방향 |
|------|---------|------|
| NUCLEO STM32 | USB 시리얼 | 양방향 |
| ROS 2 | Topic Pub/Sub | 양방향 |
| 웹 대시보드 | Flask + Socket.IO | 양방향 |
| Bosch 환경 API | TCP/UDP | 양방향 |
| GPS | UDP | 수신 |

---

## 2. 전체 파일 트리

```
Brain/
├── main.py                          ← 시스템 진입점
├── newComponent.py                  ← 컴포넌트 자동 생성 도구
├── test.py                          ← Jetson 온도 테스트
├── setup.sh                         ← 초기 환경 설정 스크립트
├── requirements.txt                 ← Python 의존성
├── README.md
├── LICENSE
│
├── src/
│   ├── statemachine/
│   │   ├── stateMachine.py          ← 멀티프로세스 안전 싱글톤 상태 머신
│   │   ├── systemMode.py            ← 시스템 모드 Enum 정의
│   │   └── transitionTable.py      ← 모드 전환 규칙 테이블
│   │
│   ├── templates/
│   │   ├── workerprocess.py         ← 프로세스 기반 클래스
│   │   └── threadwithstop.py       ← 스레드 기반 클래스
│   │
│   ├── gateway/
│   │   ├── processGateway.py        ← 메시지 라우터 프로세스
│   │   └── threads/
│   │       └── threadGateway.py    ← 실제 메시지 분배 스레드
│   │
│   ├── hardware/
│   │   └── serialhandler/
│   │       ├── processSerialHandler.py  ← 시리얼 연결 관리
│   │       └── threads/
│   │           ├── threadRead.py        ← NUCLEO 수신 파서
│   │           ├── threadWrite.py       ← NUCLEO 송신
│   │           ├── filehandler.py       ← 시리얼 로그 파일 기록
│   │           └── messageconverter.py  ← 메시지 포맷 변환
│   │
│   ├── bridge/
│   │   ├── processCmdbrdige.py          ← /cmd_vel → 모터 명령 브리지
│   │   ├── processAckermannBridge.py    ← /ackermann_cmd → 모터 명령 브리지
│   │   └── processGlobalPlanningBridge.py ← 전역 경로계획 브리지
│   │
│   ├── camera/
│   │   ├── processCamera.py         ← 로컬 카메라 프로세스
│   │   ├── processRosCamera.py      ← ROS 카메라 프로세스
│   │   ├── threads/threadCamera.py
│   │   └── threads/threadRosCamera.py
│   │
│   ├── dashboard/
│   │   ├── processDashboard.py      ← Flask + Socket.IO 서버
│   │   ├── Dashboard_mac.py         ← macOS 개발용 대시보드
│   │   ├── components/
│   │   │   ├── calibration.py       ← 모터 캘리브레이션
│   │   │   └── ip_manger.py         ← IP 주소 관리
│   │   └── frontend/                ← Angular 18 SPA
│   │       └── src/app/
│   │           ├── app.component.*
│   │           ├── services/
│   │           │   ├── web-socket.service.ts   ← Socket.IO 클라이언트
│   │           │   └── cluster.service.ts      ← 대시보드 데이터 상태 관리
│   │           └── components/
│   │               ├── cluster/         ← 전체 클러스터 레이아웃
│   │               ├── speedometer/     ← 속도계 표시
│   │               ├── battery-level/   ← 배터리 잔량 표시
│   │               ├── kl-switch/       ← 점화 스위치 UI
│   │               ├── car/             ← 차량 시각화
│   │               ├── map/             ← 지도 표시
│   │               │   ├── map-semaphore/   ← 신호등 오버레이
│   │               │   └── map-cursor/      ← 현재 위치 커서
│   │               ├── live-camera/     ← 실시간 카메라 스트림
│   │               ├── instant-consumption/ ← 전력 소비 표시
│   │               ├── hardware-data/   ← 하드웨어 상태 표시
│   │               ├── side-marker/     ← 차선/위치 마커
│   │               ├── record/          ← 녹화 제어
│   │               ├── time-speed-steer/ ← 시간/속도/조향 표시
│   │               ├── state-switch/    ← 시스템 모드 전환 버튼
│   │               └── warning-light/   ← 경고등 표시
│   │
│   ├── utils/
│   │   ├── messages/
│   │   │   ├── allMessages.py           ← 전체 메시지 타입 정의
│   │   │   ├── messageHandlerSender.py  ← 메시지 전송 핸들러
│   │   │   └── messageHandlerSubscriber.py ← 메시지 구독 핸들러
│   │   └── bigPrintMessages.py          ← 콘솔 ASCII 아트 출력
│   │
│   └── data/                            ← Git 서브모듈 (Bosch Shared)
│       ├── processSemaphores.py         ← 신호등 감지 프로세스
│       ├── threads/threadSemaphores.py
│       ├── processTrafficCommunication.py ← GPS/환경 API 프로세스
│       └── threads/
│           ├── threadTrafficCommunication.py
│           ├── tcpClient.py             ← TCP 통신 클라이언트
│           ├── udpListener.py           ← UDP 수신 (GPS)
│           ├── keyDealer.py             ← 인증 키 처리
│           ├── sharedMem.py             ← 공유 메모리
│           └── periodicTask.py          ← 주기적 작업 스케줄러
│
└── services/
    ├── brain-autostart/             ← Brain 부팅 자동시작 systemd 서비스
    ├── angular-autostart/           ← Angular 부팅 자동시작 서비스
    └── rpi-wifi-fallback/           ← RPi WiFi 폴백 서비스
```

---

## 3. 진입점 및 설정

### `Brain/main.py`

시스템의 시작점. 모든 프로세스를 생성·관리한다.

#### 프로세스 활성화 플래그 (최상단)
```python
ENABLE_GATEWAY              = True   # 메시지 라우터 (필수)
ENABLE_DASHBOARD            = True   # 웹 대시보드
ENABLE_CAMERA               = True   # ROS 카메라
ENABLE_SEMAPHORES           = True   # 신호등 감지
ENABLE_TRAFFIC_COM          = True   # GPS/환경 API
ENABLE_SERIAL_HANDLER       = True   # NUCLEO 시리얼 (필수)
ENABLE_CMDVELBRIDGE         = False  # /cmd_vel ROS 브리지
ENABLE_ACKERMAN             = True   # /ackermann_cmd ROS 브리지
ENABLE_GLOBAL_PLANNING_BRIDGE = True # 전역 경로계획 브리지
```

#### 큐 생성
```python
queueList = {
    "Critical": Queue(),   # 긴급 (StateChange, EmergencyStop)
    "Warning":  Queue(),   # 중요 상태
    "General":  Queue(),   # 일반 데이터 (속도, 조향, IMU 등)
    "Config":   Queue(),   # 구독/설정 관리
    "Image":    Queue(maxsize=1),  # 카메라 프레임 (최신만 유지)
}
```

#### 메인 루프 동작
1. 모든 활성화된 프로세스 spawn
2. 각 프로세스의 `ready_event` 대기 (동기화)
3. `Critical` 큐에서 `StateChange` 메시지 수신 감지
4. 모드에 따라 `processSemaphores` / `processTrafficCom` 동적 시작·종료
5. `KeyboardInterrupt` 시 모든 프로세스 graceful shutdown

#### 하트비트 타임아웃
- 재시도: 12회 × 15초 = 최대 3분 대기 후 강제 종료

---

### `Brain/newComponent.py`

새 컴포넌트를 자동 생성하는 스캐폴딩 도구.

**동작 순서:**
1. 패키지 이름 및 카테고리 입력 받기
2. `src/<category>/<name>/process<Name>.py` 생성
3. `src/<category>/<name>/threads/thread<Name>.py` 생성
4. `main.py`의 마커 사이에 import와 초기화 코드 자동 삽입

---

## 4. 메시지 버스 시스템

### 큐 구조

```
┌─────────────┐    메시지 전송     ┌──────────────────┐
│  Sender     │ ─────────────────▶ │   Queue (5종)    │
│ (Process/   │                    │  Critical        │
│  Thread)    │                    │  Warning         │
└─────────────┘                    │  General         │
                                   │  Config          │
                                   │  Image           │
                                   └────────┬─────────┘
                                            │ Gateway 라우팅
                                   ┌────────▼─────────┐
                                   │  Subscriber Pipe │
                                   │  (각 프로세스)     │
                                   └──────────────────┘
```

---

### `src/utils/messages/allMessages.py`

**전체 메시지 타입 정의 파일.** 모든 메시지는 Enum 클래스로 정의된다.

#### 메시지 구조
```python
class someMessage(Enum):
    Queue   = "General"          # 사용할 큐 이름
    Owner   = "senderName"       # 발신자 식별자
    msgID   = 0                  # 발신자 내 고유 ID
    msgType = list               # 데이터 타입
```

#### 메시지 카테고리별 정리

**카메라 관련**
| 메시지 | Queue | Owner | 데이터 타입 | 설명 |
|--------|-------|-------|------------|------|
| `mainCamera` | Image | Camera | bytes | 카메라 프레임 |
| `serialCamera` | General | Camera | str | 카메라 직렬화 데이터 |
| `Recording` | General | Camera | bool | 녹화 상태 |
| `Signal` | General | Camera | bool | 카메라 신호 |
| `LaneKeeping` | General | Camera | bool | 차선 유지 상태 |

**센서 데이터**
| 메시지 | Queue | Owner | 데이터 타입 | 설명 |
|--------|-------|-------|------------|------|
| `BatteryLvl` | General | SerialHandler | float | 배터리 전압 (mV) |
| `ImuData` | General | SerialHandler | list | IMU 데이터 [roll, pitch, yaw] |
| `InstantConsumption` | General | SerialHandler | float | 순간 전력 소비 |
| `ResourceMonitor` | General | SerialHandler | list | 힙/스택 메모리 |
| `CurrentSpeed` | General | SerialHandler | float | 현재 속도 |
| `CurrentSteer` | General | SerialHandler | float | 현재 조향각 |

**대시보드 → 하드웨어 제어**
| 메시지 | Queue | Owner | 데이터 타입 | 설명 |
|--------|-------|-------|------------|------|
| `SpeedMotor` | General | Dashboard | float | 목표 속도 명령 |
| `SteerMotor` | General | Dashboard | float | 목표 조향 명령 |
| `Control` | General | Dashboard | bool | 제어 활성화 |
| `Brake` | General | Dashboard | bool | 브레이크 |
| `Record` | General | Dashboard | bool | 녹화 시작/정지 |
| `DrivingMode` | General | Dashboard | int | 주행 모드 |

**시스템 제어**
| 메시지 | Queue | Owner | 데이터 타입 | 설명 |
|--------|-------|-------|------------|------|
| `StateChange` | Critical | StateMachine | str | 모드 전환 이벤트 |
| `EmergencyStop` | Critical | — | bool | 긴급 정지 |

**위치/GPS**
| 메시지 | Queue | Owner | 데이터 타입 | 설명 |
|--------|-------|-------|------------|------|
| `Location` | General | TrafficCommunication | list | GPS 좌표 |

---

### `src/utils/messages/messageHandlerSender.py`

메시지를 큐에 전송하는 유틸리티.

```python
# 사용 예시
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import allMessages

sender = messageHandlerSender(queueList)
sender.sendToQueue(allMessages.SpeedMotor, 15.0)

# 최신 값만 유지 (고주파 데이터용)
sender.sendToQueue(allMessages.ImuData, [0.1, 0.2, 0.3], drop_old=True)
```

**동작:**
- 메시지 Enum에서 `Queue`, `Owner`, `msgID`, `msgType` 읽기
- 값을 `(Owner, msgID, msgType, value)` 튜플로 래핑
- 해당 큐에 put

---

### `src/utils/messages/messageHandlerSubscriber.py`

메시지를 구독하는 유틸리티. **Pipe** 기반 IPC 사용.

```python
# 사용 예시 (Thread 내부)
subscriber = messageHandlerSubscriber(queueList)
subscriber.subscribe(allMessages.SpeedMotor)

# 수신 (블로킹 또는 non-blocking)
data = subscriber.receive()

# 최신값만 (lastOnly 모드)
subscriber.subscribe(allMessages.ImuData, lastOnly=True)
```

**전달 모드:**
| 모드 | 설명 |
|------|------|
| FIFO | 수신 순서대로 전달 |
| lastOnly | 항상 최신 메시지만 유지 (고주파 데이터에 적합) |

**내부 동작:**
1. `Config` 큐에 구독 요청 메시지 전송
2. Gateway가 `(Owner, msgID) → Pipe` 매핑 등록
3. 이후 해당 메시지가 오면 Pipe로 전달
4. Pipe 장애 시 자동 복구 메커니즘 포함

---

## 5. 상태 머신

### `src/statemachine/stateMachine.py`

**멀티프로세스 안전 싱글톤.** 모든 프로세스에서 공유하는 시스템 모드를 관리한다.

```python
# main.py에서 초기화 (프로세스 생성 전에 반드시 호출)
StateMachine.initialize_shared_state(manager)

# 모드 전환 요청
StateMachine.request_mode("AUTO")

# 현재 모드 조회 (어느 프로세스에서든 호출 가능)
mode = StateMachine.get_mode()
```

**내부 구현:**
- `multiprocessing.Manager().dict()`로 공유 상태 저장
- `multiprocessing.Lock()`으로 동시 접근 보호
- 전환 성공 시 `Critical` 큐에 `StateChange` 메시지 전송

---

### `src/statemachine/systemMode.py`

```python
class SystemMode(Enum):
    DEFAULT = "DEFAULT"  # 기본 대기 상태
    AUTO    = "AUTO"     # 자율주행 모드
    MANUAL  = "MANUAL"   # 수동 제어 모드
    LEGACY  = "LEGACY"   # 레거시 (신호등+GPS 활성)
    STOP    = "STOP"     # 정지 상태
```

**모드별 서브시스템 활성화:**
| 모드 | 카메라 | 해상도 | 시리얼 | Semaphore | TrafficCom |
|------|--------|--------|--------|-----------|------------|
| DEFAULT | ✗ | — | ✓ | ✗ | ✗ |
| AUTO | ✓ | 480p | ✓ | ✗ | ✗ |
| MANUAL | ✓ | 1080p | ✓ | ✗ | ✗ |
| LEGACY | ✓ | 1080p | ✓ | ✓ | ✓ |
| STOP | ✗ | — | ✓ | ✗ | ✗ |

---

### `src/statemachine/transitionTable.py`

허용된 상태 전환 규칙 테이블.

```
현재 모드 → 전환 가능한 다음 모드
─────────────────────────────────
DEFAULT  → AUTO, MANUAL, LEGACY, STOP
AUTO     → DEFAULT, MANUAL, LEGACY, STOP
MANUAL   → DEFAULT, AUTO, LEGACY, STOP
LEGACY   → DEFAULT, AUTO, MANUAL, STOP
STOP     → DEFAULT
```

**사용:**
```python
result = TransitionTable.check_transition("AUTO", "MANUAL")
# result = {"transition_valid": True, "next_mode": "MANUAL"}
```

---

## 6. 템플릿 기반 클래스

### `src/templates/workerprocess.py`

**모든 `Process`의 기반 클래스.**

```python
class WorkerProcess(Process):
    def __init__(self, queueList, debugging=False):
        ...

    def run(self):
        # 스레드 초기화 → ready_event set → work 루프
        self._init_threads()
        self.ready_event.set()
        while self._running:
            self.work()

    def work(self):
        # 서브클래스에서 오버라이드
        pass

    def _state_change_handler(self, new_mode):
        # 모드 변경 시 호출됨
        pass

    def pause(self):   # 모든 내부 스레드 일시정지
    def resume(self):  # 재개
    def stop(self):    # graceful shutdown
```

**주요 기능:**
- `ready_event`: 초기화 완료 신호 (main.py가 대기)
- `_threads` 리스트로 내부 스레드 관리
- `_running` 플래그로 루프 제어
- 데몬 프로세스 지원

---

### `src/templates/threadwithstop.py`

**모든 `Thread`의 기반 클래스.**

```python
class ThreadWithStop(Thread):
    def __init__(self):
        self._blocker = Event()    # stop 시 set
        self._pause_event = Event()  # pause 시 clear

    def run(self):
        while not self._blocker.is_set():
            if self._pause_event.is_set():
                self.work()
            self._blocker.wait(self._pause_duration)

    def work(self):
        # 서브클래스에서 오버라이드
        pass

    def stop(self):   # _blocker.set()
    def pause(self):  # _pause_event.clear()
    def resume(self): # _pause_event.set()
```

---

## 7. 게이트웨이 (메시지 라우터)

### `src/gateway/processGateway.py`

**모든 IPC 메시지를 라우팅하는 중앙 허브.**

- `threadGateway` 하나만 내부에 생성
- 구독/해지 요청을 처리하고 메시지를 분배
- 프로세스 장애 시 해당 구독 자동 정리

---

### `src/gateway/threads/threadGateway.py`

실제 라우팅 로직을 담당하는 스레드.

**구독 테이블 구조:**
```python
subscriptions = {
    "SerialHandler": {
        0: [pipe1, pipe2],   # msgID 0번을 구독하는 Pipe 목록
        1: [pipe3],
    },
    "Dashboard": {
        0: [pipe4],
    }
}
```

**메시지 우선순위 처리:**
```
1순위: Critical 큐  (StateChange, EmergencyStop)
2순위: Warning 큐
3순위: General 큐
4순위: Image 큐     (maxsize=1, 최신 프레임만)
```

**처리 루프:**
1. 4개 큐를 순서대로 polling
2. 메시지의 `(Owner, msgID)` 로 구독 테이블 조회
3. 해당 Pipe에 메시지 데이터 전송
4. `Config` 큐에서 subscribe/unsubscribe 처리

---

## 8. 하드웨어 시리얼 핸들러

### `src/hardware/serialhandler/processSerialHandler.py`

**NUCLEO STM32와의 USB 시리얼 연결 관리.**

```
/dev/ttyACM* 자동 감지
      │
      ▼
연결 성공 → threadRead + threadWrite 시작
      │
연결 실패 → 1초 후 재시도 (무한 반복)
```

**주요 기능:**
- `/dev/ttyACM*` 패턴으로 시리얼 포트 자동 탐색
- 연결 상태를 `General` 큐로 대시보드에 알림
- 연결 끊김 시 스레드 pause → 재연결 후 resume
- 모드 변경 시 적절한 pause/resume 처리

---

### `src/hardware/serialhandler/threads/threadRead.py`

**NUCLEO에서 오는 시리얼 데이터를 파싱하는 핵심 스레드.** (약 876줄)

#### 파싱하는 메시지 타입 (16종)

| 메시지 타입 | 데이터 | 큐 전송 | ROS 퍼블리시 |
|------------|--------|---------|-------------|
| `imu` | roll, pitch, yaw | ImuData → General | `/Imu` |
| `imuenc` | IMU + 엔코더 + timestamp | ImuData, CurrentSpeed | `/Imu`, `/wheel_encoder` |
| `encoder` / `enc` | RPM, velocity, distance | CurrentSpeed | `/wheel_encoder`, `/wheel_twist` |
| `battery` | 전압 (7000~21000 mV) | BatteryLvl | — |
| `instant` | 전력 소비 (W) | InstantConsumption | — |
| `speed` | 현재 속도 명령 | CurrentSpeed | — |
| `steer` | 현재 조향 명령 | CurrentSteer | — |
| `resourceMonitor` | 힙/스택 메모리 | ResourceMonitor | — |
| `alive` | 생존 신호 | — | — |
| `steerLimits` | 조향 각도 한계 | Config | — |
| `vcdCalib` | 캘리브레이션 PWM | Config | — |

#### ROS 2 퍼블리시 토픽
```
/Imu  (sensor_msgs/Imu)
  - orientation: 쿼터니언 변환 (roll/pitch/yaw → quaternion)
  - angular_velocity: 3축 자이로
  - linear_acceleration: 3축 가속도
  - covariance 행렬 포함

/wheel_encoder  (geometry_msgs/Vector3Stamped)
  - x: RPM
  - y: velocity (m/s)
  - z: distance (m)

/wheel_twist  (geometry_msgs/TwistWithCovarianceStamped)
  - 휠 속도 기반 twist (위치 추정용)
  - covariance 포함
```

#### 환경 변수 설정
```bash
IMU_ANGLE_UNIT=deg          # deg 또는 rad (기본: deg)
IMU_APPLY_VEHICLE_FRAME=1   # 차량 프레임 좌표 변환 (기본: 1)
WHEEL_SIGMA_V=0.05          # 휠 속도 공분산
WHEEL_DIST_SCALE=1.042      # 거리 스케일 보정
WHEEL_VEL_SCALE=1.042       # 속도 스케일 보정
```

---

### `src/hardware/serialhandler/threads/threadWrite.py`

**NUCLEO로 제어 명령을 전송하는 스레드.**

- `SpeedMotor` 메시지 구독 → 시리얼로 속도 명령 전송
- `SteerMotor` 메시지 구독 → 시리얼로 조향 명령 전송
- 브레이크/정지 명령 처리
- 캘리브레이션 명령 처리

**시리얼 프로토콜 형식:**
```
#<type>:<value>;;\r\n
예: #speed:15.0;;\r\n
    #steer:-25.0;;\r\n
```

---

### `src/hardware/serialhandler/threads/filehandler.py`

시리얼 수신 데이터를 타임스탬프와 함께 파일에 기록한다.

---

### `src/hardware/serialhandler/threads/messageconverter.py`

시리얼 raw 문자열을 파싱 가능한 Python 딕셔너리로 변환하는 유틸리티.

---

## 9. ROS 2 브리지

### `src/bridge/processCmdbrdige.py` — cmd_vel 브리지

**ROS 2 `/cmd_vel` (Twist) 메시지를 모터 명령으로 변환.**

```
ROS /cmd_vel (Twist)
  ├── linear.x  × scale(10.0)  → SpeedMotor
  └── angular.z × scale(250.0) → SteerMotor (부호 반전)
```

**설정값:**
```python
SPEED_SCALE = 10.0    # m/s → motor 단위
STEER_SCALE = 250.0   # rad/s → motor 단위
STEER_LIMIT = 250     # 최대 조향각 (±250)
```

**동작 조건:**
- `AUTO` 모드에서만 활성화
- QoS: RELIABLE, depth=1 (최신 명령만)
- 50 Hz 주기로 모드 폴링

---

### `src/bridge/processAckermannBridge.py` — Ackermann 브리지

**ROS 2 `/ackermann_cmd` (AckermannDriveStamped)를 모터 명령으로 변환.**

```
ROS /ackermann_cmd (AckermannDriveStamped)
  ├── drive.speed         × scale(10.0)  → SpeedMotor
  └── drive.steering_angle × scale(10.0) → SteerMotor
```

**설정값:**
```python
SPEED_SCALE = 10.0    # m/s → motor 단위
STEER_SCALE = 10.0    # rad 또는 deg → motor 단위
steer_use_degrees = False  # True 시 deg 입력
MAX_RATE = 30         # Hz, 출력 최대 주파수
```

**추가 기능:**
- 모드 종료 시 자동 정지 명령 전송
- `DrivingMode` 메시지도 모니터링
- QoS: BEST_EFFORT (실시간 제어에 적합)

---

### `src/bridge/processGlobalPlanningBridge.py` — 전역 경로계획 브리지

**경로 계획 시스템과 Brain 간의 양방향 브리지.** (약 633줄)

#### 발행 (Brain → ROS)
```
목표 노드 ID → /global_planning/goal_node_id (std_msgs/Int32)
```

#### 구독 (ROS → Brain)
```
/global_path                          → 대시보드 지도 표시
/global_pose                          → 현재 위치 대시보드 표시
/global_pose_with_covariance_stamped  → 공분산 포함 위치
/obstacle_roi                         → 장애물/도로표지판 감지
```

#### 도로 표지판 처리
```python
# 지원하는 표지판 클래스
ONEWAY, STOPSIGN, ROUNDABOUT, PARK,
CROSSWALK, NOENTRY, PRIORITY, HIGHWAY_ENTRY,
HIGHWAY_EXIT, PARKING_EXIT, TRAFFIC_LIGHT
```

#### 지도 파일 처리
- GraphML 형식의 지도 파일 파싱
- 노드 ID ↔ 좌표 매핑 구축
- 경로 표시를 위한 좌표 변환

#### 레이트 제한
```bash
# 환경 변수로 설정
ROAD_SIGN_SEND_PERIOD=0.2   # 도로 표지판 업데이트 주기 (초)
```

---

## 10. 대시보드

### `src/dashboard/processDashboard.py`

**Flask + Socket.IO 기반 웹 대시보드 서버.** (약 552줄)

#### Socket.IO 이벤트 (클라이언트 → 서버)
| 이벤트 | 데이터 | 동작 |
|--------|--------|------|
| `connect` | — | 세션 등록, 하트비트 시작 |
| `disconnect` | — | 세션 정리 |
| `heartbeat` | — | 연결 유지 확인 |
| `keyDown` | `{key, action}` | SpeedMotor/SteerMotor 명령 전송 |
| `setMode` | `{mode}` | StateMachine 모드 전환 요청 |
| `setRecord` | `{value}` | 녹화 시작/정지 |
| `setCalibration` | `{...}` | 모터 캘리브레이션 |
| `setGoalNode` | `{node_id}` | 목표 노드 설정 |

#### Socket.IO 이벤트 (서버 → 클라이언트)
| 이벤트 | 데이터 | 트리거 |
|--------|--------|--------|
| `battery` | float | BatteryLvl 수신 시 |
| `imu` | list | ImuData 수신 시 |
| `speed` | float | CurrentSpeed 수신 시 |
| `steer` | float | CurrentSteer 수신 시 |
| `consumption` | float | InstantConsumption 수신 시 |
| `camera` | bytes | 카메라 프레임 (throttled) |
| `semaphore` | dict | 신호등 상태 (throttled) |
| `serialState` | bool | 시리얼 연결 상태 변경 시 |
| `systemMode` | str | 모드 변경 시 |
| `hardwareData` | dict | CPU/메모리/온도 |
| `globalPath` | list | 경로 데이터 |
| `globalPose` | dict | 현재 위치 |
| `roadSign` | dict | 도로 표지판 감지 |

#### 하드웨어 모니터링
```python
# Jetson Nano 온도 경로
/sys/devices/virtual/thermal/thermal_zone*/temp

# 수집 데이터
- CPU 사용률 (%)
- 메모리 사용률 (%)
- 온도 (°C)
```

#### 이미지 전송 최적화
```bash
DASHBOARD_CAMERA_EMIT_PERIOD=0.12    # 카메라 프레임 전송 주기 (기본 8.3 fps)
DASHBOARD_SEMAPHORE_EMIT_PERIOD=0.25 # 신호등 업데이트 주기
DASHBOARD_AUTO_IP=0                  # 프론트엔드 IP 자동 교체 여부
```

---

### `src/dashboard/components/calibration.py`

모터 PWM 캘리브레이션 유틸리티.
- 최소/최대 PWM 값 저장 및 로드
- 속도/조향 캘리브레이션 값 계산

---

### `src/dashboard/components/ip_manger.py`

대시보드 접속용 IP 주소 관리.
- 현재 네트워크 인터페이스 IP 자동 감지
- 프론트엔드 환경파일에 IP 자동 주입 (DASHBOARD_AUTO_IP=1 시)

---

### Angular 18 프론트엔드 (`src/dashboard/frontend/`)

#### 핵심 서비스

**`web-socket.service.ts`**
- Socket.IO 클라이언트 연결 관리
- 이벤트 emit/on 래퍼 제공
- 연결 끊김 시 자동 재연결

**`cluster.service.ts`**
- 대시보드 전체 상태 관리 (BehaviorSubject 패턴)
- 센서 데이터, 모드 상태, 카메라 프레임 상태 중앙 관리

#### UI 컴포넌트

| 컴포넌트 | 기능 |
|----------|------|
| `cluster` | 전체 대시보드 레이아웃 조합 |
| `speedometer` | 속도계 (현재 속도 표시) |
| `battery-level` | 배터리 잔량 게이지 |
| `kl-switch` | 점화 스위치 (KL15/KL30) |
| `car` | 차량 상태 시각화 |
| `map` | 지도 표시 (경로, 위치, 신호등) |
| `map-semaphore` | 신호등 오버레이 |
| `map-cursor` | 현재 위치 커서 |
| `live-camera` | 실시간 카메라 영상 |
| `instant-consumption` | 순간 전력 소비 표시 |
| `hardware-data` | CPU/메모리/온도 상태 |
| `side-marker` | 차선/사이드 마커 |
| `record` | 녹화 시작/정지 버튼 |
| `time-speed-steer` | 시간, 속도, 조향값 수치 표시 |
| `state-switch` | 시스템 모드 전환 버튼 (AUTO/MANUAL 등) |
| `warning-light` | 경고 상태 표시등 |

---

## 11. 유틸리티

### `src/utils/bigPrintMessages.py`

콘솔에 ASCII 아트를 출력하는 유틸리티.

```
C4_BOMB:     폭탄 카운트다운 아트 (타임아웃 경고용)
PLEASE_WAIT: 대기 메시지
PRESS_CTRL_C: 종료 안내
```

---

## 12. 외부 서브모듈

`src/data/` → `https://github.com/ECC-BFMC/Shared.git` (branch: `data`)

### `processSemaphores.py` + `threads/threadSemaphores.py`
- 카메라 이미지에서 신호등 색상 감지 (딥러닝 또는 색상 필터)
- 감지 결과를 대시보드에 전송
- `LEGACY` 모드에서만 활성화

### `processTrafficCommunication.py` + threads
| 파일 | 기능 |
|------|------|
| `threadTrafficCommunication.py` | 메인 통신 스레드 |
| `tcpClient.py` | Bosch 서버 TCP 연결 |
| `udpListener.py` | GPS 데이터 UDP 수신 |
| `keyDealer.py` | 인증 키 교환 처리 |
| `sharedMem.py` | 프로세스간 공유 메모리 |
| `periodicTask.py` | 주기적 하트비트/업데이트 스케줄러 |

---

## 13. 서비스 설정

### `services/brain-autostart/`
- systemd 서비스 파일: 부팅 시 `python3 main.py` 자동 실행
- 모니터링 스크립트: 프로세스 크래시 시 자동 재시작
- start/kill 스크립트

### `services/angular-autostart/`
- Angular 빌드 결과물 정적 서빙 자동 시작

### `services/rpi-wifi-fallback/`
- 주 WiFi 연결 실패 시 AP 모드 폴백

---

## 14. 전체 데이터 흐름

### 수동 제어 흐름 (Dashboard → Motor)
```
[대시보드 UI]
    ↓ keyDown 이벤트 (Socket.IO)
[processDashboard]
    ↓ SpeedMotor / SteerMotor 메시지 → General 큐
[processGateway]
    ↓ Pipe → threadWrite 구독자
[threadWrite]
    ↓ #speed:15.0;;\r\n (USB Serial)
[NUCLEO STM32]
    ↓ PWM 신호
[모터 / 서보]
```

### 센서 데이터 흐름 (NUCLEO → Dashboard)
```
[NUCLEO STM32]
    ↓ #imu:0.1,0.2,0.3;;\r\n (USB Serial)
[threadRead]
    ↓ ImuData 메시지 → General 큐
[processGateway]
    ↓ Pipe → processDashboard 구독자
[processDashboard]
    ↓ imu 이벤트 (Socket.IO)
[대시보드 UI]
    ↓ 클러스터 표시 업데이트
```

### ROS 2 자율주행 흐름 (AUTO 모드)
```
[Navigation Stack]
    ↓ /ackermann_cmd (ROS 2 Topic)
[processAckermannBridge]
    ↓ SpeedMotor / SteerMotor → General 큐
[processGateway]
    ↓ Pipe → threadWrite
[threadWrite]
    ↓ Serial → NUCLEO → 모터
```

### 모드 전환 흐름
```
[대시보드 setMode 이벤트]
    ↓
[processDashboard]
    ↓ StateMachine.request_mode("AUTO")
[StateMachine]
    ↓ TransitionTable 검증
    ↓ StateChange 메시지 → Critical 큐
[main.py 메인 루프]
    ↓ 모드에 따라 processSemaphores / processTrafficCom 동적 시작·종료
    ↓ 모든 프로세스에 StateChange 전파
[각 Process / Thread]
    ↓ _state_change_handler() 호출
    ↓ 해당 모드에 맞게 동작 변경
```

---

## 15. 환경 변수 설정

| 변수명 | 기본값 | 설명 |
|--------|--------|------|
| `IMU_ANGLE_UNIT` | `deg` | IMU 각도 단위 (`deg` 또는 `rad`) |
| `IMU_APPLY_VEHICLE_FRAME` | `1` | 차량 프레임 좌표 변환 적용 여부 |
| `WHEEL_SIGMA_V` | `0.05` | 휠 속도 공분산 (센서 퓨전용) |
| `WHEEL_DIST_SCALE` | `1.042` | 거리 스케일 보정 계수 |
| `WHEEL_VEL_SCALE` | `1.042` | 속도 스케일 보정 계수 |
| `GLOBAL_PLANNING_GRAPHML` | — | GraphML 지도 파일 경로 |
| `ROAD_SIGN_SEND_PERIOD` | `0.2` | 도로 표지판 업데이트 주기 (초) |
| `DASHBOARD_SEMAPHORE_EMIT_PERIOD` | `0.25` | 신호등 UI 업데이트 주기 (초) |
| `DASHBOARD_CAMERA_EMIT_PERIOD` | `0.12` | 카메라 프레임 전송 주기 (초) |
| `DASHBOARD_AUTO_IP` | `0` | 프론트엔드 IP 자동 교체 여부 |

---

## 빠른 참조

### 새 컴포넌트 추가 절차
1. `python3 newComponent.py` 실행 (자동 스캐폴딩)
2. `allMessages.py`에 새 메시지 타입 추가
3. thread의 `subscribe()` 메서드에 구독 등록
4. `main.py` 상단 플래그 확인 (자동 패치됨)

### 자주 보는 파일 경로 요약
| 목적 | 파일 |
|------|------|
| 프로세스 켜고 끄기 | `Brain/main.py` 상단 플래그 |
| 메시지 타입 추가 | `src/utils/messages/allMessages.py` |
| 모드 정의/수정 | `src/statemachine/systemMode.py` |
| 전환 규칙 수정 | `src/statemachine/transitionTable.py` |
| 시리얼 파싱 추가 | `src/hardware/serialhandler/threads/threadRead.py` |
| 시리얼 명령 추가 | `src/hardware/serialhandler/threads/threadWrite.py` |
| UI 이벤트 추가 | `src/dashboard/processDashboard.py` |
| ROS 토픽 추가 | 해당 브리지 파일 |

---

## 16. 자율주행 시스템 (seame_ros)

Brain은 자율주행의 **허브/번역기** 역할만 담당한다. 실제 경로 계획과 경로 추종(제어)은 별도 저장소인 `~/seame_ros`에서 ROS 2 노드로 동작한다.

### 역할 분리 요약

```
seame_brain (Python)              seame_ros (C++ ROS 2)
──────────────────────            ──────────────────────
대시보드 UI → 목표 노드 ID 전송    경로 계획 (A* + B-spline)
/ackermann_cmd 수신 → 시리얼 변환  경로 추종 (Pure Pursuit)
센서 데이터 → ROS 토픽 퍼블리시    위치 추정 (EKF + Particle Filter)
```

### seame_ros 파일 구조

```
~/seame_ros/src/
├── planning/
│   └── src/global_planning.cpp      ← A* 경로 계획 노드
│
├── control/
│   ├── src/global_control_node.cpp  ← Pure Pursuit 제어 노드
│   └── include/control/global_control_node.hpp
│
├── localization/
│   ├── laneLocalizerNode.cpp        ← 파티클 필터 위치 추정
│   └── odom_generator.py           ← 휠 + IMU → /odom
│
├── robot_localization/
│   └── params/ekf.yaml             ← EKF 설정 (robot_localization 패키지)
│
└── Params/
    └── config.yaml                 ← 전체 파라미터 설정 파일
```

### 경로 계획 모드 (global_planning.cpp)

| 모드 | 설명 | 트리거 |
|------|------|--------|
| `default` | CSV 파일에서 미리 정의된 경로 로드 | 시작 시 |
| `click_node` | 대시보드에서 목표 노드 클릭 → A* 즉시 계산 | `/global_planning/goal_node_id` 수신 |
| `local` | 하이브리드 (A* + 로컬 수정) | — |

### A* 경로 계획 상세

```
입력: GraphML 지도 (130+ 노드, 노드 간 엣지+거리)
        ↓
A* 탐색: state = (prev_node, curr_node) — 방향성 고려
        ↓
최적 경로: 노드 시퀀스
        ↓
B-spline 스무딩: 꺾인 경로 → 부드러운 곡선
        ↓
3cm 간격 리샘플링
        ↓
퍼블리시: /global_path (nav_msgs/Path)
```

**State를 (prev, curr) 쌍으로 정의하는 이유:**
일방통행 도로, 회전교차로처럼 방향이 중요한 구간에서 동일 노드를 어느 방향으로 진입했는지에 따라 다음 경로가 달라지기 때문.

### ROS 토픽 요약

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/global_planning/goal_node_id` | Brain→ROS | std_msgs/Int32 | 목표 노드 ID |
| `/global_path` | ROS→Brain | nav_msgs/Path | 계획된 경로 |
| `/global_pose` | ROS→Brain | geometry_msgs/PoseStamped | 현재 위치 (map 프레임) |
| `/ackermann_cmd` | ROS→Brain | AckermannDriveStamped | 조향+속도 명령 |
| `/center_line_points` | ROS내부 | sensor_msgs/PointCloud2 | 차선 중심선 |
| `/odometry/filtered` | ROS내부 | nav_msgs/Odometry | EKF 필터링된 오도메트리 |
| `/wheel_twist` | Brain→ROS | TwistWithCovarianceStamped | 휠 인코더 속도 |
| `/Imu` | Brain→ROS | sensor_msgs/Imu | BNO055 IMU |

---

## 17. 자율주행 데이터 파이프라인 상세

### 파이프라인 1: 제어 명령 흐름 (경로 추종 → 모터)

```
[seame_ros: global_control_node]
  Pure Pursuit 알고리즘 계산
  조향각 + 속도 → AckermannDriveStamped
        ↓ /ackermann_cmd (ROS 2, BEST_EFFORT QoS)
[seame_brain: processAckermannBridge]
  speed_cmd = int(speed × 10.0)
  steer_deg = steering_angle × (180/π) × 10  (deg×10)
  steer_cmd = clamp(steer_deg, -250, 250)
  30Hz 타이머 → _pending_cmd flush
        ↓ SpeedMotor / SteerMotor → General 큐
[processGateway → threadWrite]
  #speed:25.0;;\r\n
  #steer:-87.5;;\r\n  (별도 전송)
        ↓ USB Serial
[NUCLEO STM32]
  PWM → DC 모터 (속도)
  PWM → 서보 모터 (조향)
```

**변환 예시:**
- 조향각 +0.175 rad → +10.0° → steer_cmd = +100
- 속도 2.5 m/s → speed_cmd = 25

### 파이프라인 2: 센서 피드백 흐름 (NUCLEO → ROS EKF)

```
[NUCLEO STM32]
  imuenc 패킷: ts_us, roll, pitch, yaw, gyro×3, accel×3, RPM, vel, dist
        ↓ USB Serial #imuenc:...\r\n
[threadRead._parse_imuenc()]
  MCU μs → ROS nanosecond 타임스탬프 동기화
  IMU + 엔코더가 동일 timestamp 공유 (원자적)
        ↓ 세 개 ROS 토픽 동시 퍼블리시
  /Imu (sensor_msgs/Imu)
    - orientation: BNO055 절대 방위 (quaternion)
    - angular_velocity: 자이로 (rad/s)
    - linear_acceleration: 가속도 (m/s²)
  /wheel_encoder (Vector3Stamped)
    - x: RPM, y: velocity, z: distance
  /wheel_twist (TwistWithCovarianceStamped)
    - twist.linear.x: WHEEL_VEL_SCALE(1.042) × vel
    - covariance: vx만 신뢰 (σ²=0.05²), 나머지=1e3
        ↓
[robot_localization EKF]
  /wheel_twist → vx만 사용 (선속도)
  /Imu (BNO055) → yaw만 사용 (절대 방위)
  /d455f/imu → vyaw만 사용 (자이로 보정)
  30Hz 필터링
        ↓ /odometry/filtered
[Particle Filter (laneLocalizerNode)]
  오도메트리 + 차선 정보 → 500 파티클 map 좌표 추정
        ↓ /global_pose (map 프레임)
[global_control_node]
  현재 위치로 경로 추종 계산
```

### 파이프라인 3: 경로 계획 흐름 (대시보드 → 경로)

```
[대시보드 UI]
  지도에서 목표 노드 클릭
        ↓ setGoalNode Socket.IO
[processDashboard]
  GoalNodeId 메시지 → General 큐
        ↓
[processGlobalPlanningBridge]
  /global_planning/goal_node_id 퍼블리시 (std_msgs/Int32)
        ↓ ROS 2
[global_planning.cpp]
  A* 계산 (GraphML 130+ 노드)
  B-spline 스무딩 + 3cm 리샘플링
        ↓ /global_path (nav_msgs/Path)
[processGlobalPlanningBridge]
  2Hz → 대시보드 globalPath 이벤트
        ↓
[global_control_node]
  경로 수신 → Pure Pursuit 시작
```

---

## 18. 위치 추정 시스템

차량의 현재 위치(x, y, yaw)를 추정하는 3단계 파이프라인:

```
[센서들]
휠 인코더 + BNO055 IMU + RealSense D455 IMU
        ↓
[Stage 1: odom_generator.py]
  휠 속도 + IMU yaw → dead reckoning
  퍼블리시: /odom (nav_msgs/Odometry, odom 프레임)
        ↓
[Stage 2: robot_localization EKF]
  /wheel_twist (vx)
  /Imu BNO055 (yaw 절대값)
  /d455f/imu (vyaw)
  → 센서 퓨전 → /odometry/filtered
  world_frame: odom (상대 위치, drift 누적)
        ↓
[Stage 3: laneLocalizerNode (Particle Filter)]
  입력: /odometry/filtered + /lane_points_base
  500 파티클 → 차선 지도 매칭
  출력: /global_pose (map 프레임, 절대 위치)
```

### Stage 1: odom_generator.py

```python
# 휠 속도 + IMU yaw → dead reckoning
v = wheel_velocity  # m/s
yaw = imu_yaw       # BNO055 절대 방위
dt = 현재_time - 이전_time

x += v * cos(yaw) * dt
y += v * sin(yaw) * dt
```

- 입력: `/wheel_encoder`, `/Imu`
- 출력: `/odom` (odom → base_link TF 포함)
- 문제점: yaw drift 없음(절대 IMU), 하지만 속도 적분 오차로 장거리 위치 drift 발생

### Stage 2: EKF (robot_localization)

`~/seame_ros/src/robot_localization/params/ekf.yaml` 설정:

```yaml
frequency: 30.0
two_d_mode: true       # z, roll, pitch 무시
world_frame: odom      # 절대 위치 기준 없음 (상대 odom)

twist0: /wheel_twist   # vx만 활성화
  [false, false, false, false, false, false,
   true,  false, false, false, false, false,
   false, false, false]

imu0: /Imu             # BNO055 yaw만 활성화 (절대값)
imu1: /d455f/d455f/imu # RealSense yaw rate만
```

- 출력: `/odometry/filtered` (odom 프레임)
- **현재 한계**: `world_frame: odom` 이므로 절대 위치(x,y) 기준 없음. 장거리 운행 시 odom drift 누적.

### Stage 3: Particle Filter (laneLocalizerNode.cpp)

- 파티클 수: 500개
- 입력: `/odometry/filtered` (모션 모델) + `/lane_points_base` (관측 모델)
- 차선 중심선 데이터로 지도의 어느 위치인지 매칭
- 출력: `/global_pose` (map 프레임, 절대 위치)
- 이 `/global_pose`가 Pure Pursuit 제어 노드의 입력이 됨

### 위치 정확도 향상 방법

| 방법 | 효과 | 비고 |
|------|------|------|
| WHEEL_VEL_SCALE 정밀 캘리브레이션 | 속도/거리 오차 감소 | 현재 1.042 |
| initial_estimate_covariance 조정 | EKF 수렴 속도/안정성 개선 | 현재 1e-9 (너무 작음) |
| `world_frame: map` + GPS 절대위치 | 장거리 drift 제거 | GPS 신호 필요 |
| 파티클 수 증가 (500→1000+) | Particle Filter 정확도 향상 | CPU 부담 증가 |
| 차선 인식 품질 개선 | PF 관측 모델 향상 | 카메라 조명 의존 |

---

## 19. EKF 개념

**EKF(Extended Kalman Filter)** — 비선형 시스템에서 상태를 추정하는 재귀적 베이즈 필터.

### 기본 Kalman Filter 사이클

```
┌─────────────────────────────┐
│         PREDICT             │
│                             │
│  x̂⁻ = f(x̂, u)             │  ← 이전 상태 + 제어 입력으로 예측
│  P⁻  = F·P·Fᵀ + Q          │  ← 예측 불확실성 계산 (Q: 프로세스 노이즈)
└─────────────┬───────────────┘
              ↓ 센서 데이터 도착
┌─────────────▼───────────────┐
│          UPDATE             │
│                             │
│  K = P⁻·Hᵀ·(H·P⁻·Hᵀ+R)⁻¹ │  ← 칼만 게인 계산 (R: 측정 노이즈)
│  x̂ = x̂⁻ + K·(z - h(x̂⁻))  │  ← 측정값으로 보정
│  P  = (I - K·H)·P⁻          │  ← 불확실성 감소
└─────────────────────────────┘
```

### 핵심 개념

**상태 벡터 x** (2D 모드):
```
x = [x 위치, y 위치, yaw, vx, vy, vyaw]
```

**프로세스 노이즈 Q:**
- 모델이 얼마나 부정확한지를 표현
- 값 ↑ → 센서를 더 믿음 (빠른 적응, 노이즈 민감)
- 값 ↓ → 모델을 더 믿음 (부드럽지만 느린 적응)

**측정 노이즈 R:**
- 센서가 얼마나 부정확한지를 표현
- 값 ↑ → 해당 센서 신뢰도 낮음
- 값 ↓ → 해당 센서 신뢰도 높음

**칼만 게인 K:**
- K = 0 이면 예측만 사용 (센서 무시)
- K = 1 이면 측정만 사용 (모델 무시)
- 실제로는 P⁻와 R의 비율로 자동 결정

### "Extended"인 이유

일반 KF는 선형 시스템만 다룬다. 로봇의 운동 모델은:
```
x_new = x + vx·cos(yaw)·dt   ← 비선형!
y_new = y + vx·sin(yaw)·dt
```
EKF는 이를 **야코비안(Jacobian) 행렬**으로 선형화하여 근사 처리한다.

### SEAME에서의 EKF 설정 요점

```yaml
# ekf.yaml 핵심 설정
twist0: /wheel_twist   → vx만 신뢰 (전진 속도)
imu0: /Imu             → yaw만 신뢰 (BNO055 절대 방위, 드리프트 없음)
imu1: /d455f/imu       → vyaw만 신뢰 (RealSense 자이로, 고주파 보정)

initial_estimate_covariance: 1e-9  # 초기 불확실성 (현재 너무 작음)
process_noise_covariance:          # 모델 노이즈 (튜닝 포인트)
```

---

## 20. Brain 자율주행 성능 개선 포인트

### 개선 사항 우선순위

| 우선순위 | 항목 | 현재 상태 | 개선 방향 |
|----------|------|-----------|-----------|
| 🔴 높음 | 속도+조향 원자적 전송 | 별도 전송 (1ms 데싱크) | `vcd` 명령 사용 |
| 🔴 높음 | 휠 스케일 캘리브레이션 | WHEEL_VEL_SCALE=1.042 고정 | 실측 기반 정밀 조정 |
| 🟡 중간 | threadWrite 루프 속도 | pause=0.001 (1000Hz) | 30~50Hz로 낮춰 CPU 절약 |
| 🟡 중간 | EKF 초기 공분산 | 1e-9 (너무 작음) | 1e-3 ~ 1e-1 수준으로 증가 |
| 🟢 낮음 | IMU 공분산 | spec 기반 기본값 | 실측 Allan Variance로 조정 |

### 1. `vcd` 원자적 명령 (threadWrite.py)

**문제:** 속도와 조향을 별도 메시지로 전송 → 최대 1ms 데싱크
```python
# 현재 (문제)
send("#speed:25.0;;\r\n")   # t=0
send("#steer:-87.5;;\r\n")  # t=0.001s
```

**개선:** `vcd` 명령으로 한 번에 전송
```python
# 개선
send('{"action":"vcd","time":0,"speed":25,"steer":-88}\r\n')
```

비상 정지 로직에서는 이미 `vcd(0,0)` 사용 중 — 일반 주행에도 적용 권장.

### 2. 휠 인코더 스케일 캘리브레이션

**문제:** `WHEEL_VEL_SCALE=1.042`는 추정값. 부정확하면 EKF vx 오차 → odom drift 누적.

**개선 방법:**
```bash
# 1. 알려진 거리를 직선 주행
# 2. /wheel_encoder의 누적 distance 읽기
# 3. 실제거리 / 측정거리 = 새 스케일
export WHEEL_DIST_SCALE=<실측값>
export WHEEL_VEL_SCALE=<실측값>
```

### 3. threadWrite 루프 속도 최적화

**문제:** `pause=0.001` (1000Hz) 는 30Hz 입력 대비 과도하게 빠름 → CPU 낭비.

**개선:**
```python
# threadWrite.py
self.pause = 1.0 / 40  # 40Hz (30Hz 입력보다 약간 빠르게)
```

### 4. EKF 초기 공분산 조정

**문제:** `initial_estimate_covariance: 1e-9` — 초기 불확실성이 거의 0으로 설정됨.
처음 실행 시 EKF가 센서 데이터를 잘 받아들이지 못하고 느리게 수렴.

**개선 (`ekf.yaml`):**
```yaml
initial_estimate_covariance: [1e-3, 0, 0, 0, 0, 0,
                               0, 1e-3, 0, 0, 0, 0,
                               0, 0, 1e-3, 0, 0, 0,
                               ...]
```

### 5. AckermannBridge 변환 검증

**현재 변환:**
```python
speed_cmd = int(speed * 10.0)         # 예: 2.5 m/s → 25
steer_deg = math.degrees(steer_rad)   # rad → degrees
steer_cmd = int(steer_deg * 10)       # 예: 10° → 100
```

**주의 사항:**
- `steer_sign_`이 -1이면 조향 방향이 반전됨 — config.yaml의 `steer_sign` 확인 필수
- `max_angular_z: 0.436332 rad` (25°) 에 맞게 steer clamp ±250 설정 확인
- NUCLEO 측 PWM 범위와 일치하는지 실차 테스트 필요

### 자율주행 전체 점검 체크리스트

```
[ ] WHEEL_VEL_SCALE / WHEEL_DIST_SCALE 실측 캘리브레이션 완료
[ ] ekf.yaml initial_estimate_covariance 적정값 조정
[ ] /odometry/filtered → /global_pose Particle Filter 정상 동작 확인
[ ] /global_pose가 /ackermann_cmd 생성에 사용되는지 확인
[ ] steer_sign_ 방향 일치 확인 (실차 테스트)
[ ] vcd 원자적 명령 적용 (선택)
[ ] threadWrite loop 속도 최적화 (선택)
[ ] config.yaml lookahead / lane_kp / g_steergain 주행 환경에 맞게 튜닝
```
