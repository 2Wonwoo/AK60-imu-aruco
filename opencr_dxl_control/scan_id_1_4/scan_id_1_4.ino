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
