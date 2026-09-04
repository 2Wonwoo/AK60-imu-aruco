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
