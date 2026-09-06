# ak60_driver

Jetson Orin Nano + AK60-6 4륜 로봇용 ROS 2 패키지.
SocketCAN 모터 구동, ArUco 마커 추종, IMU 기반 자세 복원을 포함한다.

> 같은 로봇을 OpenCR + DYNAMIXEL로 제어하는 별도 경로(ROS2 아님, Jetson 미경유,
> Linux PC에서 USB로 OpenCR에 직접 연결)는 [`opencr_dxl_control/`](opencr_dxl_control/README.md) 참고.

## Quick Start

**1. 준비** — 매번 터미널을 열 때마다

```bash
~/can_check.sh can1 1000000          # CAN 버스 기동 (이게 없으면 모터 노드가 죽는다)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

`ip -br link show type can` 으로 `can1` 이 `UP` 인지 확인할 수 있다.

**2. 실행** — 마커를 추종하다가 바디가 기울면 자세를 복원하고 다시 추종

```bash
# 판정만 확인 (모터가 움직이지 않는다 - 먼저 이걸로 확인할 것)
ros2 launch ak60_driver aruco_imu.launch.py

# 실제 구동
ros2 launch ak60_driver aruco_imu.launch.py dry_run:=false
```

`Ctrl+C` 로 종료하면 모터에 정지 명령이 나간다.

> ⚠️ **로봇을 평평한 바닥에 네 바퀴로 세운 상태에서 실행할 것.** 시작 시점의
> 자세를 자동으로 0점으로 잡고, 이후 그 기준에서 ±5도를 벗어나면 자세 복원이
> 작동한다. 기울어진 상태로 시작하면 그 자세를 수평으로 학습해 버린다.

### 개별 실행

```bash
ros2 launch ak60_driver aruco_imu.launch.py dry_run:=false     # 추종 + 자세 복원
ros2 launch ak60_driver aruco_follow.launch.py dry_run:=false  # 마커 추종만
ros2 launch ak60_driver imu_level.launch.py dry_run:=false     # 자세 복원만
ros2 run ak60_driver keyboard_teleop                           # 키보드 수동 조작
```

### 자주 쓰는 인자

```bash
# 추종할 마커 번호와 실제 인쇄 크기(m)
ros2 launch ak60_driver aruco_imu.launch.py dry_run:=false \
    target_marker_id:=1 marker_size:=0.15

# 정지 거리 / 자세 복원 데드존
ros2 launch ak60_driver aruco_imu.launch.py dry_run:=false \
    stop_distance:=0.60 level_threshold:=2.5

# 카메라 화면 보기
ros2 launch ak60_driver aruco_imu.launch.py dry_run:=false headless:=false
```

### 빌드와 테스트

```bash
cd ~/ros2_ws
colcon build --packages-select ak60_driver && source install/setup.bash

python3 -m pytest src/ak60_driver/test/ -q \
    --ignore=src/ak60_driver/test/test_flake8.py \
    --ignore=src/ak60_driver/test/test_pep257.py \
    --ignore=src/ak60_driver/test/test_copyright.py
```

### 문제가 생기면

| 증상 | 원인 |
|---|---|
| `Network is down` 으로 노드가 죽음 | CAN 미기동 → `~/can_check.sh can1 1000000` |
| 모터가 전혀 안 움직임 | `dry_run` 이 `true` (기본값) → `dry_run:=false` |
| 명령을 보내도 모터 무반응 | `ip -d link show can1` 이 `ERROR-PASSIVE` 면 모터 전원·배선·종단저항 확인 |
| CAN 프레임은 나가는데 바퀴가 안 돎 | MIT 진입 프레임 미전송. `four_wheel_drive_node` 가 시작할 때 보내므로 `MIT enable sent to motors 1-4` 로그를 확인할 것 |
| 바퀴가 힘없이 기어가듯 움직임 | 토크 부족. `drive.yaml` 의 `kd` 를 올린다 ([아래](#토크와-kd)) |

### 토크와 `kd`

MIT 모드에서 `kp=0` 으로 쓰므로 토크는 **속도 오차에만 비례**한다:

```
토크 = kd × (목표속도 − 현재속도)
```

즉 정지 상태에서 낼 수 있는 토크는 `kd × 목표속도`다. 자세 복원은 최대
0.6 rad/s 라서 `kd=1.0` 에서는 겨우 0.6 N·m 밖에 안 나와 바퀴가 움직이지 않았다.

후진 1.0 rad/s 명령으로 3초간 바퀴 위치 변화를 실측한 값:

| `kd` | 위치 변화 |
|---|---|
| 1.0 | +106 (거의 안 움직임) |
| 2.0 | +541 |
| **3.5** | **+1160** ← 기본값 |
| 5.0 | +1306 (프로토콜 상한) |

3.5 부터는 모터가 명령 속도를 따라잡아 속도 오차가 줄어들므로 증가폭이 완만해진다.
더 힘이 필요하면 5.0 까지 올릴 수 있다:

```bash
ros2 launch ak60_driver imu_level.launch.py dry_run:=false --ros-args -p kd:=5.0
```
| 엉뚱한 바퀴가 후진 | IMU 장착 방향 → [`mount_yaw_deg`](#imu-장착-방향-mount_yaw_deg) |

---

## 바퀴 배치

```
        앞
   ID2 ●────● ID1
       │    │
   ID4 ●────● ID3
        뒤
```

| ID | 위치 | 모터 방향 부호 |
|---|---|---|
| 1 | 앞-오른쪽 | +1 |
| 2 | 앞-왼쪽 | −1 |
| 3 | 뒤-오른쪽 | +1 |
| 4 | 뒤-왼쪽 | −1 |

왼쪽 모터는 물리적으로 반대를 향하므로 부호가 뒤집힌다. 이 보정은
`four_wheel_drive_node` 가 하므로, 토픽으로 보내는 값은 **항상 전진이 양수**다.

## 노드

| 실행 이름 | 역할 | 입력 | 출력 |
|---|---|---|---|
| `four_wheel_drive_node` | CAN 모터 구동 | `/cmd_vel`, `/wheel_velocities` | CAN |
| `aruco_follower` | 마커 추종 | CSI 카메라 | `/cmd_vel` |
| `imu_leveler` | 자세 복원 | IMU (USB 시리얼) | `/wheel_velocities` |
| `keyboard_teleop` | 수동 조작 | 키보드 | `/cmd_vel` |

`/cmd_vel` (Twist) 은 차동구동이라 좌/우 쌍으로만 움직인다.
**바퀴를 개별로 돌리려면** `/wheel_velocities` (Float64MultiArray, `data[0..3]`
= ID 1..4, 전진 양수 rad/s) 를 쓴다.

> 두 토픽이 동시에 들어오면 `/wheel_velocities` 가 이긴다 (자세 복원 우선).
> 자세한 중재 방식은 [주도권 중재](#주도권-중재) 참조.

---

## IMU 자세 복원 (`imu_leveler`)

바퀴 하나가 장애물 위에 올라가 바디가 기울면, **들린 바퀴를 후진시켜** 장애물에서
내려오게 해 바디를 다시 지면과 평행하게 만든다.

### 원리

roll 과 pitch 를 조합하면 어느 **모서리**가 들렸는지 알 수 있다. 앞바퀴는 pitch 에
`+`, 뒷바퀴는 `−`, 왼쪽은 roll 에 `+`, 오른쪽은 `−` 로 기여한다:

```
elevation(i) = pitch_up × front_sign(i) + roll_up × left_sign(i)
```

| ID | front_sign | left_sign |
|---|---|---|
| 1 앞-오른쪽 | +1 | −1 |
| 2 앞-왼쪽 | +1 | +1 |
| 3 뒤-오른쪽 | −1 | −1 |
| 4 뒤-왼쪽 | −1 | +1 |

ID2(앞-왼쪽)가 장애물에 올라가면 앞이 들리고(pitch+) 왼쪽이 들려서(roll+)
두 항이 모두 양수가 되므로, ID2 의 `elevation` 만 크게 나온다.

각 바퀴 속도는 들린 정도에 비례한다 (들린 바퀴는 후진):

```
velocity(i) = −gain × max(elevation(i), 0)
```

기울기가 `level_threshold` 아래로 내려오면 수평으로 보고 전부 정지한다.

### IMU 장착 방향 (`mount_yaw_deg`)

**이 로봇의 IMU 는 바디에 수직축으로 90도 돌아간 채 장착되어 있다.** 그래서 센서의
roll 축이 바디의 pitch 축이 된다. `mount_yaw_deg` 기본값 `90.0` 이 이를 보정한다.

네 모서리를 하나씩 손으로 들어올려 확인한 실측값:

| 실제로 든 바퀴 | 보정 전 판정 | 측정값 (roll, pitch) |
|---|---|---|
| ID2 앞-왼쪽 | front-right ✗ | (−5.8, +6.7) |
| ID1 앞-오른쪽 | rear-right ✗ | (−4.7, −7.7) |
| ID4 뒤-왼쪽 | front-left ✗ | (+5.5, +7.2) |
| ID3 뒤-오른쪽 | rear-left ✗ | (+2.1, −7.9) |

단순 부호 뒤집기가 아니라 **90도 회전**이라 `roll_sign` / `pitch_sign` 으로는
고칠 수 없다. 기울기를 2차원 벡터로 보고 장착 회전각만큼 되돌린다:

```
roll_body  =  roll·cos ψ + pitch·sin ψ
pitch_body = −roll·sin ψ + pitch·cos ψ
```

ψ=90° 이면 `roll_body = pitch`, `pitch_body = −roll` 이 되어 네 모서리가 모두
올바르게 판정된다. 이 실측값은 회귀 테스트로 고정해 두었다
(`test_mount_yaw_matches_the_measured_corners`).

### ⚠️ IMU 를 다시 장착했다면 반드시 재확인

장착 방향이 바뀌면 판정이 어긋나고, 그러면 로봇이 장애물에서 내려오는 게 아니라
**더 올라가려 한다.** `dry_run` 이 기본값 `true` 라 명령을 발행하지 않고 판정만
출력하므로, 모터를 붙이기 전에 반드시 확인할 것:

```bash
ros2 run ak60_driver imu_leveler
```

**네 모서리를 하나씩 들어올리며** 로그가 실제 모서리와 맞는지 본다:

```
[INFO] front-left raised +8.3 deg (roll +5.9, pitch +5.8)
```

어긋난다면 `mount_yaw_deg` 를 0 / 90 / 180 / 270 중에서 바꿔가며 맞춘다:

```bash
ros2 run ak60_driver imu_leveler --ros-args -p mount_yaw_deg:=270.0
```

거울처럼 좌우만 뒤집혔다면 (회전으로 설명되지 않는 경우) `roll_sign` 또는
`pitch_sign` 을 `-1.0` 으로 준다.

### 수평 기준과 데드존 (시작 시 자동 0점)

IMU 는 중력 기준 절대각을 출력하므로, 장착이 조금이라도 기울면 평지에서도 각도가
0 으로 읽히지 않는다. 그래서 **노드가 시작할 때 그 시점의 자세를 자동으로 0점으로
잡는다** (`tare_on_start` 기본값 `true`). 껐다 켤 때마다 맞춰지므로 offset 을
손으로 넣을 필요가 없다.

```
[INFO] 시작 자세를 0점으로 잡는다 - 로봇이 평평한 바닥에 서 있어야 한다
[INFO] tare 완료 (50 샘플): roll_offset=-6.28, pitch_offset=-3.88
[INFO] IMU leveler ready: zero=(roll -6.28, pitch -3.88), deadzone=+-5.0 deg
[INFO] [FOLLOW] level (tilt 0.0 deg)
```

이후 기울기는 **모두 이 0점 대비**로 판정되고, `level_threshold`(기본 2.5)에 따라
**기준에서 ±5도 안쪽의 변화에는 반응하지 않는다.**

> ⚠️ **반드시 평평한 바닥에 네 바퀴로 선 상태에서 실행할 것.** 바퀴가 장애물에
> 올라간 상태로 시작하면 그 기울어진 자세를 수평으로 학습해 버려서, 평지에
> 내려왔을 때 로봇이 오히려 다시 올라가려 한다.
>
> 기준을 잡는 0.5초 동안 자세가 1도 넘게 흔들리면 경고를 띄운다. 그 경고가
> 보이면 로봇을 세워두고 다시 시작할 것.

기준을 고정하고 싶으면 (예: 매번 같은 값을 쓰고 싶을 때) tare 를 끄고 직접 준다:

```bash
ros2 launch ak60_driver aruco_imu.launch.py tare_on_start:=false \
    roll_offset:=-6.28 pitch_offset:=-3.88
```

### 실행

```bash
# 1) CAN 올리기
~/can_check.sh can1 1000000

# 2) 판정만 확인 (모터 안 움직임)
ros2 launch ak60_driver imu_level.launch.py

# 3) 부호를 확인한 뒤 실제 구동
ros2 launch ak60_driver imu_level.launch.py dry_run:=false

# 부호가 반대였다면
ros2 launch ak60_driver imu_level.launch.py dry_run:=false pitch_sign:=-1.0
```

### 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `dry_run` | `true` | 명령을 발행하지 않고 판정만 출력 |
| `port` | 자동 탐색 | IMU 시리얼 포트 (비우면 `/dev/ttyUSB*` 자동) |
| `baud` | 115200 | IMU 통신속도 |
| `level_threshold` | 2.5 | 기준에서 이 각도(도) 안쪽이면 수평으로 보고 정지 (데드존) |
| `gain` | 0.08 | 들린 각도 1도당 rad/s |
| `max_velocity` | 0.6 | 바퀴 속도 상한 (rad/s) |
| `min_velocity` | 0.05 | 이보다 작은 명령은 무시 (모터 떨림 방지) |
| `reverse_only` | `true` | 들린 바퀴만 후진. `false` 면 내려간 바퀴는 전진 |
| `max_tilt` | 35.0 | 이보다 크게 기울면 비정상으로 보고 정지 |
| `roll_sign` / `pitch_sign` | 1.0 | 부호 규약 보정 (거울 반전인 경우) |
| `mount_yaw_deg` | 90.0 | IMU 장착 회전각 (이 로봇 실측) |
| `roll_offset` / `pitch_offset` | −2.31 / 1.83 | 수평 기준값. `tare_on_start` 가 `true` 면 시작 시 덮어쓴다 |
| `tare_on_start` | `true` | 시작 시점의 자세를 0점으로 잡기 (평지에서 실행) |
| `publish_rate` | 20.0 | 제어 주기 (Hz) |
| `yield_when_level` | `true` | 수평이면 발행하지 않아 마커 추종에 양보 |
| `level_hold_sec` | 0.3 | 수평이 이만큼 유지되어야 주도권 반환 |

### 안전장치

- `dry_run` 기본 `true` — 부호를 확인하기 전에는 움직이지 않는다
- `max_velocity` 기본 0.6 rad/s 로 낮게 제한
- `max_tilt` 초과 시 정지 (센서 이상이나 전복 상황에서 폭주 방지)
- `four_wheel_drive_node` 의 `command_timeout` (0.5초) — 노드가 죽거나 IMU 가
  끊기면 모터가 자동 정지한다
- 노드 종료 시 정지 명령 발행

---

## 마커 추종 + 자세 복원 통합 (`aruco_imu.launch.py`)

평소에는 마커를 추종하다가, 바퀴가 장애물에 올라가 바디가 기울면 **추종을 멈추고
자세를 복원한 뒤 다시 추종**한다.

```bash
~/can_check.sh can1 1000000

# 판정만 확인 (모터 안 움직임)
ros2 launch ak60_driver aruco_imu.launch.py

# 실제 구동
ros2 launch ak60_driver aruco_imu.launch.py dry_run:=false
```

### 주도권 중재

```
aruco_follower  --/cmd_vel----------+
                                    +--> four_wheel_drive_node --> CAN
imu_leveler  --/wheel_velocities----+
```

두 노드가 동시에 명령을 내면 서로 싸우므로 `four_wheel_drive_node` 가 중재한다.
`/wheel_velocities` 가 `wheel_priority_timeout`(0.3초) 안에 들어오면 `/cmd_vel` 을
무시한다. `imu_leveler` 는 **수평일 때 아무것도 발행하지 않으므로** 우선권이
자연히 풀리고 마커 추종이 로봇을 몬다.

`aruco_follower` 는 계속 `/cmd_vel` 을 발행해도 되며 수정할 필요가 없다.

### 상태 전이

| 상태 | 동작 |
|---|---|
| `FOLLOW` | 발행 없음. 마커 추종이 로봇을 몬다 |
| `STOP` | 기울기 감지 → **전체 정지** (한 주기) |
| `LEVELING` | 들린 바퀴를 후진. 수평이 될 때까지 |
| `RELEASE` | 수평이 `level_hold_sec` 유지됨 → 정지 명령 한 번 내고 주도권 반환 |

`level_hold_sec`(기본 0.3초)는 임계값 근처에서 추종과 복원이 번갈아 튀는 것을
막는다. 유지 중에 다시 기울면 타이머가 초기화되고 복원을 계속한다.

> `imu_leveler` 를 단독으로 쓸 때는 `yield_when_level:=false` 로 두면 수평일 때도
> 매 주기 정지 명령을 내보낸다.

---

## ArUco 마커 추종 (`aruco_follower`)

```bash
ros2 launch ak60_driver aruco_follow.launch.py                    # dry_run 기본 false
ros2 launch ak60_driver aruco_follow.launch.py dry_run:=true headless:=false
```

마커 생성·인쇄와 카메라 설정은 별도 저장소 참조:
https://github.com/2Wonwoo/aruco_tracking

---

## 개발 메모

제어 로직은 ROS·하드웨어 없이 테스트할 수 있도록 순수 모듈로 분리해 두었다.
카메라나 CAN 버스 없이도 정책을 검증할 수 있다.

| 순수 모듈 | 대응 노드 |
|---|---|
| `aruco_follow_control.py` | `aruco_follower.py` |
| `imu_level_control.py` | `imu_leveler.py` |

빌드와 테스트 명령은 [Quick Start](#빌드와-테스트) 참조.
