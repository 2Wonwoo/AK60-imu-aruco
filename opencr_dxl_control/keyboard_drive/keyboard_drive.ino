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
