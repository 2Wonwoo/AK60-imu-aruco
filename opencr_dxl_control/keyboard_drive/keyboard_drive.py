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
