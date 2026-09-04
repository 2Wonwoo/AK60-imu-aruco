# opencr_dxl_control

**OpenCR + DYNAMIXEL(X-series, Protocol 2.0) 유선 제어.**
`ak60_driver`(Jetson + AK60-6 CAN 모터)와는 별개로, **Jetson을 거치지 않고
Linux PC(터미널)에서 USB 케이블로 OpenCR에 직접 연결**해 다이나믹셀을 인식·제어하는
코드다. ROS2 노드가 아니라 Arduino 펌웨어 + 순수 Python 스크립트로만 동작한다.

바퀴 배치와 방향 부호 규칙은 `ak60_driver`와 동일하다 (ID1,3 같은 부호 / ID2,4
같은 부호, 두 그룹은 서로 반대).

## 연결 구성

```
Linux PC (USB) ---- OpenCR ---- DXL 버스 ---- DYNAMIXEL ID 1~4
```

Jetson은 이 경로에 들어가지 않는다. PC의 USB 포트에 OpenCR을 직접 연결한다.

## Quick Start

**1. 준비물 설치 (최초 1회)**

```bash
# arduino-cli
mkdir -p ~/.local/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH="$HOME/.local/bin:$PATH"

# OpenCR 보드 코어
arduino-cli config init
arduino-cli config set board_manager.additional_urls \
    https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
arduino-cli core update-index
arduino-cli core install OpenCR:OpenCR

# DYNAMIXEL 제어 라이브러리
arduino-cli lib install Dynamixel2Arduino

# Python 시리얼
pip install pyserial
```

**2. udev 규칙 설치 (Linux, 최초 1회, sudo 필요)**

ModemManager가 OpenCR의 USB 시리얼을 가로채 업로드가 실패하는 걸 막고, 일반
사용자도 포트에 접근할 수 있게 해준다.

```bash
wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
sudo cp 99-opencr-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**3. 연결 확인**

```bash
ls /dev/ttyACM*
lsusb | grep 0483   # "STMicroelectronics Virtual COM Port" 가 보여야 OpenCR
```

아무것도 안 보이면 케이블(데이터 지원 케이블인지)과 포트를 바꿔가며 확인한다.
같은 컴퓨터에 다른 ttyACM 장치(예: Jetson 콘솔)가 있으면 vendor ID로 반드시
구분할 것 — OpenCR은 `0483`.

**4. 업로드**

```bash
cd opencr_dxl_control
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR keyboard_drive
arduino-cli upload -p /dev/ttyACM1 --fqbn OpenCR:OpenCR:OpenCR keyboard_drive
```

업로드가 `cmd_read_board_name fail : 0xF020` 로 실패하면 리커버리(부트로더) 모드로
강제 진입 후 재시도한다: **PUSH SW2를 누른 채로 Reset을 눌렀다 떼고, SW2는
1~2초 더 누르고 있다가 놓는다.** 한 번 정상 업로드된 뒤로는 보통 자동 리셋으로
업로드된다.

**5. 실행**

```bash
python3 keyboard_drive/keyboard_drive.py /dev/ttyACM1
```

`w` 전진 한 스텝 / `s` 후진 한 스텝 / `q` 종료.

## 방향 규칙

| ID | 그룹 | 전진 시 목표각 |
|---|---|---|
| 1 | A | + |
| 3 | A | + |
| 2 | B | − |
| 4 | B | − |

두 그룹(A, B)은 항상 반대 부호로 움직여야 실제로는 같은 방향(전진/후진)이 된다
(좌우 대칭 장착 가정). 회전이 필요하면 두 그룹에 같은 부호를 주면 된다.

## 스텝 제한

기준(목표각 변경 0회) 상태에서 `w`는 최대 8번까지만 눌리고, 그 이상은 무시된다.
`w`로 올라간 만큼만 `s`로 되돌릴 수 있고, 기준 상태에서 `s`를 누르면 무시된다.
(각도/스텝 수는 `keyboard_drive.ino`의 `ANGLE_STEP`, `MAX_STEPS`로 조절.)

## 문제가 생기면

| 증상 | 원인 |
|---|---|
| 업로드가 `cmd_read_board_name fail : 0xF020` | OpenCR이 실제로 이 PC의 USB에 안 잡혀 있거나(케이블/포트 문제), 부트로더 진입이 안 된 상태. `lsusb`에 `0483`이 보이는지부터 확인, 안 보이면 케이블/포트 교체. 보이는데도 실패하면 SW2+Reset으로 리커버리 모드 진입 후 재시도 |
| 업로드 중 ModemManager 간섭 | `99-opencr-cdc.rules` 설치 (Quick Start 2단계) |
| 특정 ID만 응답 없음 | 배선 접촉 불량 — 해당 모터의 전원선/데이터선 커넥터, 데이지체인 순서 확인 (`scan_id_1_4`로 반복 확인 가능) |
| 다른 컴퓨터에서 `/dev/ttyACM0`이 OpenCR이 아님 | 이 PC에 Jetson 등 다른 ttyACM 장치가 같이 붙어있을 수 있음. `lsusb`로 vendor ID(`0483`=OpenCR) 확인 후 맞는 포트 사용 |

## 파일 구성

```
opencr_dxl_control/
├── firmware/firmware.ino          # 범용 브릿지 (SCAN/PING/TORQUE/MOVE/READ 텍스트 명령)
├── pc_control.py                  # 위 브릿지용 PC 스크립트 (모터 스캔 + 기본 제어 예시)
├── scan_id_1_4/scan_id_1_4.ino    # ID 1~4 연결 상태를 2초마다 반복 확인 (단독 실행)
└── keyboard_drive/
    ├── keyboard_drive.ino         # W/S 각도 제어 펌웨어 (Position Control, 8스텝 제한)
    └── keyboard_drive.py          # W/S 키 입력을 시리얼로 전달하는 PC 스크립트
```

## 전체 코드

### firmware/firmware.ino

```cpp
/*
  OpenCR + DYNAMIXEL(X-series, Protocol 2.0) 제어 펌웨어
  - PC에서 USB Serial로 텍스트 명령을 보내면 DXL 버스를 제어하고 결과를 회신한다.
  - 필요한 라이브러리: Arduino IDE 라이브러리 매니저에서 "Dynamixel2Arduino" (ROBOTIS) 설치

  명령 목록 (한 줄씩, 개행으로 종료):
    SCAN                    -> 57600 / 1000000 두 보드레이트에서 ID 0~20 스캔, 발견된 ID와 모델번호 출력
    BAUD <bps>              -> DXL 버스 통신 보드레이트 변경 (기본 57600)
    PING <id>
    TORQUE <id> <0|1>
    MOVE <id> <goal_pos>    -> Position Control Mode 기준, goal_pos: 0~4095
    READ <id>               -> 현재 위치 출력
    HELP
*/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR 보드의 DXL 방향 제어 핀

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

const uint8_t SCAN_ID_MIN = 0;
const uint8_t SCAN_ID_MAX = 20;
const long SCAN_BAUDRATES[] = {57600, 1000000};

long currentBaud = 57600;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {}

  dxl.begin(currentBaud);
  dxl.setPortProtocolVersion(2.0);

  DEBUG_SERIAL.println("READY OpenCR DXL Controller");
  DEBUG_SERIAL.println("HELP 입력 시 명령 목록 출력");
}

void loop() {
  if (DEBUG_SERIAL.available()) {
    String line = DEBUG_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleCommand(line);
    }
  }
}

void printHelp() {
  DEBUG_SERIAL.println("SCAN");
  DEBUG_SERIAL.println("BAUD <bps>");
  DEBUG_SERIAL.println("PING <id>");
  DEBUG_SERIAL.println("TORQUE <id> <0|1>");
  DEBUG_SERIAL.println("MOVE <id> <goal_pos>");
  DEBUG_SERIAL.println("READ <id>");
}

void handleCommand(String line) {
  int sp1 = line.indexOf(' ');
  String cmd = (sp1 == -1) ? line : line.substring(0, sp1);
  cmd.toUpperCase();

  if (cmd == "HELP") {
    printHelp();
  } else if (cmd == "SCAN") {
    doScan();
  } else if (cmd == "BAUD") {
    long bps = line.substring(sp1 + 1).toInt();
    if (bps > 0) {
      currentBaud = bps;
      dxl.begin(currentBaud);
      DEBUG_SERIAL.print("OK BAUD ");
      DEBUG_SERIAL.println(currentBaud);
    } else {
      DEBUG_SERIAL.println("ERR BAUD");
    }
  } else if (cmd == "PING") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print("OK PING ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print(" MODEL ");
      DEBUG_SERIAL.println(dxl.getModelNumber(id));
    } else {
      DEBUG_SERIAL.print("ERR PING ");
      DEBUG_SERIAL.println(id);
    }
  } else if (cmd == "TORQUE") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int on = line.substring(sp2 + 1).toInt();
    bool ok = on ? dxl.torqueOn(id) : dxl.torqueOff(id);
    DEBUG_SERIAL.println(ok ? ("OK TORQUE " + String(id) + " " + String(on))
                             : ("ERR TORQUE " + String(id)));
  } else if (cmd == "MOVE") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int32_t goal = line.substring(sp2 + 1).toInt();
    bool ok = dxl.setGoalPosition(id, goal);
    DEBUG_SERIAL.println(ok ? ("OK MOVE " + String(id) + " " + String(goal))
                             : ("ERR MOVE " + String(id)));
  } else if (cmd == "READ") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    int32_t pos = dxl.getPresentPosition(id);
    DEBUG_SERIAL.print("OK READ ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(pos);
  } else {
    DEBUG_SERIAL.println("ERR UNKNOWN_CMD");
  }
}

void doScan() {
  DEBUG_SERIAL.println("SCAN_START");
  for (long baud : SCAN_BAUDRATES) {
    dxl.begin(baud);
    for (uint8_t id = SCAN_ID_MIN; id <= SCAN_ID_MAX; id++) {
      if (dxl.ping(id)) {
        DEBUG_SERIAL.print("FOUND ID ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(" BAUD ");
        DEBUG_SERIAL.print(baud);
        DEBUG_SERIAL.print(" MODEL ");
        DEBUG_SERIAL.println(dxl.getModelNumber(id));
      }
    }
  }
  dxl.begin(currentBaud);
  DEBUG_SERIAL.println("SCAN_END");
}
```

### pc_control.py

```python
"""
OpenCR(DXL 컨트롤러 펌웨어 업로드된 상태)와 USB 시리얼로 통신하여
다이나믹셀 모터를 인식/제어하는 PC용 스크립트.

사용 전:
  pip install pyserial
  firmware/firmware.ino 를 Arduino IDE로 OpenCR에 업로드해 둘 것.

포트 확인:
  Linux : ls /dev/ttyACM*
  Windows: 장치관리자에서 COM 포트 번호 확인
"""

import sys
import time
import serial

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


class OpenCRDxl:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # OpenCR 리셋 및 부팅 대기
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _send(self, line: str):
        self.ser.write((line.strip() + "\n").encode())

    def _read_until_idle(self, wait=0.3):
        lines = []
        deadline = time.time() + wait
        while time.time() < deadline:
            if self.ser.in_waiting:
                raw = self.ser.readline().decode(errors="ignore").strip()
                if raw:
                    lines.append(raw)
                    deadline = time.time() + wait
        return lines

    def scan(self):
        """연결된 모든 모터를 인식 (57600 / 1000000 두 보드레이트 모두 탐색)"""
        self._send("SCAN")
        lines = self._read_until_idle(wait=1.5)
        motors = []
        for l in lines:
            if l.startswith("FOUND"):
                # FOUND ID <id> BAUD <baud> MODEL <model>
                parts = l.split()
                motors.append({
                    "id": int(parts[2]),
                    "baud": int(parts[4]),
                    "model": int(parts[6]),
                })
        return motors, lines

    def ping(self, motor_id: int):
        self._send(f"PING {motor_id}")
        return self._read_until_idle()

    def set_baud(self, bps: int):
        self._send(f"BAUD {bps}")
        return self._read_until_idle()

    def torque(self, motor_id: int, on: bool):
        self._send(f"TORQUE {motor_id} {1 if on else 0}")
        return self._read_until_idle()

    def move(self, motor_id: int, goal_position: int):
        self._send(f"MOVE {motor_id} {goal_position}")
        return self._read_until_idle()

    def read_position(self, motor_id: int):
        self._send(f"READ {motor_id}")
        lines = self._read_until_idle()
        for l in lines:
            if l.startswith("OK READ"):
                return int(l.split()[-1])
        return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    dxl = OpenCRDxl(port=port)

    print(f"[연결됨] {port}")
    print("모터 스캔 중...")
    motors, raw = dxl.scan()

    if not motors:
        print("모터를 찾지 못했습니다. 배선/전원/보드레이트를 확인하세요.")
        print("펌웨어 응답:", raw)
        dxl.close()
        return

    print("발견된 모터:")
    for m in motors:
        print(f"  ID={m['id']}  BAUD={m['baud']}  MODEL={m['model']}")

    # 실제 사용 보드레이트로 고정 (스캔 시 마지막에 남는 값 대신 발견된 값으로 맞춰줌)
    used_baud = motors[0]["baud"]
    dxl.set_baud(used_baud)

    target_id = motors[0]["id"]
    print(f"\n[테스트 제어] ID {target_id} 토크 ON -> 위치 이동 -> 현재 위치 읽기")

    print(dxl.torque(target_id, True))
    print(dxl.move(target_id, 2048))
    time.sleep(1.0)
    pos = dxl.read_position(target_id)
    print(f"현재 위치: {pos}")

    print(dxl.torque(target_id, False))
    dxl.close()


if __name__ == "__main__":
    main()
```

### scan_id_1_4/scan_id_1_4.ino

```cpp
/*
  OpenCR DXL 포트에 연결된 다이나믹셀 ID 1~4 연결 상태를 계속 반복 확인하는 스케치
  - Arduino IDE에서 보드: OpenCR Board 선택 후 업로드
  - 업로드 후 시리얼 모니터(115200 baud)로 결과 확인 (2초마다 갱신)
  - 필요한 라이브러리: Dynamixel2Arduino (ROBOTIS)
  - DXL 버스 통신 속도는 57600bps 기준 (모터 스캔으로 확인된 값)
*/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR 보드의 DXL 방향 제어 핀

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const uint8_t TARGET_IDS[] = {1, 2, 3, 4};
const long DXL_BAUD = 57600;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {}
  delay(1000);

  dxl.setPortProtocolVersion(2.0);
  dxl.begin(DXL_BAUD);

  DEBUG_SERIAL.println("=== DYNAMIXEL ID 1~4 연결 상태 반복 확인 시작 ===");
}

void loop() {
  DEBUG_SERIAL.println("----------------------------");
  for (int i = 0; i < 4; i++) {
    uint8_t id = TARGET_IDS[i];
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" : ");
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print("연결됨 (Model Number: ");
      DEBUG_SERIAL.print(dxl.getModelNumber(id));
      DEBUG_SERIAL.println(")");
    } else {
      DEBUG_SERIAL.println("연결 안 됨");
    }
  }
  delay(2000);
}
```

### keyboard_drive/keyboard_drive.ino

```cpp
/*
  OpenCR: 키보드 명령(W/S)을 시리얼로 받아 다이나믹셀 4개(ID 1~4)를
  Position Control Mode(각도 제어)로, 아주 느린 속도(Profile Velocity = 1)로 구동.

  방향 규칙:
    - ID 2, 4 : 서로 같은 목표각(같은 그룹)
    - ID 1, 3 : 서로 같은 목표각(같은 그룹)
    - (1,3) 그룹과 (2,4) 그룹은 전진/후진 시 서로 반대 부호로 목표각을 바꿔야
      실제로는 같은 방향으로 움직인다 (좌우 대칭 장착 가정).

  키 명령(1바이트씩 수신, 누를 때마다 목표각이 ANGLE_STEP만큼 이동):
    w : 전진 방향으로 한 스텝   (1,3 += STEP, 2,4 -= STEP)
    s : 후진 방향으로 한 스텝   (1,3 -= STEP, 2,4 += STEP)

  스텝 카운트 제한:
    - 기준(0) 상태에서 w는 최대 MAX_STEPS(8)번까지만 눌림 (그 이상은 무시)
    - w를 눌러 올라간 만큼만 s로 되돌릴 수 있음 (0 밑으로는 못 내려감, 9번째 s는 무시)
*/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

const long DXL_BAUD = 57600;
const uint8_t IDS_13[] = {1, 3};
const uint8_t IDS_24[] = {2, 4};

const int32_t PROFILE_VEL_VALUE = 10; // 이동 속도 (값을 올리면 더 빨라짐, 1이 최저)
const int32_t ANGLE_STEP = 80;        // 키 한 번에 이동하는 각도 스텝 (0~4095 기준, 약 7도)
const int32_t POS_MIN = 0;
const int32_t POS_MAX = 4095;
const int MAX_STEPS = 8;              // 기준 상태에서 w로 올라갈 수 있는 최대 스텝 수

int32_t pos13, pos24;
int stepCount = 0; // 0 = 기준 상태, 최대 MAX_STEPS

int32_t clampPos(int32_t p) {
  if (p < POS_MIN) return POS_MIN;
  if (p > POS_MAX) return POS_MAX;
  return p;
}

void applyPositions() {
  for (uint8_t id : IDS_13) dxl.setGoalPosition(id, pos13);
  for (uint8_t id : IDS_24) dxl.setGoalPosition(id, pos24);
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {}
  delay(1000);

  dxl.setPortProtocolVersion(2.0);
  dxl.begin(DXL_BAUD);

  for (uint8_t id : IDS_13) {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, PROFILE_VEL_VALUE);
    dxl.torqueOn(id);
  }
  for (uint8_t id : IDS_24) {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.writeControlTableItem(PROFILE_VELOCITY, id, PROFILE_VEL_VALUE);
    dxl.torqueOn(id);
  }

  pos13 = dxl.getPresentPosition(IDS_13[0]);
  pos24 = dxl.getPresentPosition(IDS_24[0]);
  applyPositions();

  DEBUG_SERIAL.println("=== 키보드 각도 제어 준비 완료 (w/s, 기준 0 ~ 최대 8스텝) ===");
}

void loop() {
  if (DEBUG_SERIAL.available()) {
    char c = tolower(DEBUG_SERIAL.read());

    if (c == 'w') {
      if (stepCount < MAX_STEPS) {
        stepCount++;
        pos13 = clampPos(pos13 + ANGLE_STEP);
        pos24 = clampPos(pos24 - ANGLE_STEP);
        applyPositions();
        DEBUG_SERIAL.print("전진 스텝 (");
        DEBUG_SERIAL.print(stepCount);
        DEBUG_SERIAL.println("/8)");
      } else {
        DEBUG_SERIAL.println("최대 스텝 도달, w 무시됨");
      }
    } else if (c == 's') {
      if (stepCount > 0) {
        stepCount--;
        pos13 = clampPos(pos13 - ANGLE_STEP);
        pos24 = clampPos(pos24 + ANGLE_STEP);
        applyPositions();
        DEBUG_SERIAL.print("후진 스텝 (");
        DEBUG_SERIAL.print(stepCount);
        DEBUG_SERIAL.println("/8)");
      } else {
        DEBUG_SERIAL.println("기준 상태(0), s 무시됨");
      }
    }
    // w, s 외 키는 무시
  }
}
```

### keyboard_drive/keyboard_drive.py

```python
"""
OpenCR(keyboard_drive.ino 펌웨어 업로드된 상태)로 키보드 입력을 보내
다이나믹셀 4개(ID 1~4)를 W/S로 조종하는 PC용 스크립트.
기준(0) 상태에서 w 최대 8스텝, s로는 그만큼만 되돌릴 수 있음 (펌웨어에서 제한).

사용법:
  python3 keyboard_drive.py [포트]   (기본값 /dev/ttyACM1)

조작:
  w : 전진 한 스텝   s : 후진 한 스텝
  q : 종료 (정지 후 프로그램 종료)
"""

import sys
import termios
import tty
import select
import serial

DEFAULT_PORT = "/dev/ttyACM1"
DEFAULT_BAUD = 115200


def read_key(timeout=0.1):
    """터미널을 raw 모드로 두고, timeout 안에 입력이 있으면 한 글자 반환, 없으면 None"""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.1)

    print(f"[연결됨] {port}")
    print("w:전진 한 스텝  s:후진 한 스텝  q:종료")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            key = read_key(0.1)
            if key is None:
                continue
            key = key.lower()
            if key == "q":
                break
            if key in ("w", "s"):
                ser.write(key.encode())
                print(f"-> {key!r} 전송")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        ser.close()
        print("\n종료했습니다.")


if __name__ == "__main__":
    main()
```
