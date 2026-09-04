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
