import socket
import struct
import threading
import time
import subprocess
import atexit
import sys

# =========================
# Network configuration
# =========================
STM32_IP = "10.177.21.4"
STM32_PORT = 5000

PC_ETHERNET_IP = "10.177.21.1"
PC_ETHERNET_MASK = "255.255.255.0"

# ipconfig에 표시된 어댑터 이름과 일치해야 함
ETH_IF_NAME = "이더넷"

# =========================
# STM32 packet configuration
# =========================
# ethernet.h의 SteerMode_t enum 값과 반드시 일치해야 함
STEER_MODE_AUTO = 1
STEER_MODE_MANUAL = 2
STEER_MODE_ESTOP = 3

TX_PERIOD_S = 0.02  # 50 Hz

# signed int16 ASMS range
ASMS_MIN = -2048
ASMS_CENTER = 0
ASMS_MAX = 2047

# =========================
# Global state
# =========================
sock = None

active_mode = "idle"   # "pc", "asms", "idle"
target_deg = 0
asms_value = 0          # signed int16: -2048 ~ +2047
running = True

temp_ip_added_by_this_process = False


# =========================
# Windows IP helper
# =========================
def run_cmd(cmd: str):
    result = subprocess.run(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    stdout = result.stdout.decode("utf-8", errors="ignore")
    stderr = result.stderr.decode("utf-8", errors="ignore")
    output = stdout + stderr

    if result.returncode != 0:
        print("[CMD FAILED]")
        print(cmd)

        if stdout:
            print("[stdout]")
            print(stdout)

        if stderr:
            print("[stderr]")
            print(stderr)

        return False, output

    if stdout:
        print("[stdout]")
        print(stdout)

    if stderr:
        print("[stderr]")
        print(stderr)

    return True, output


def add_temp_ip() -> bool:
    """
    이더넷 어댑터에 10.177.21.1/24 보조 IP를 추가합니다.
    관리자 권한 필요.

    이미 같은 IP가 존재하는 경우에는 실패로 보지 않고 계속 진행합니다.
    """
    global temp_ip_added_by_this_process

    cmd = (
        f'netsh interface ip add address '
        f'name="{ETH_IF_NAME}" '
        f'addr={PC_ETHERNET_IP} '
        f'mask={PC_ETHERNET_MASK} '
        f'store=active'
    )

    print(f"[NETSH] add temp IP: {PC_ETHERNET_IP}/{PC_ETHERNET_MASK}")
    ok, output = run_cmd(cmd)

    if ok:
        temp_ip_added_by_this_process = True
        return True

    # 이전 실행에서 정상 종료되지 않아 IP가 남아 있는 경우
    if ("개체가 이미 있습니다" in output) or ("already exists" in output.lower()):
        print()
        print("[NETSH] temp IP already exists. Continue.")
        temp_ip_added_by_this_process = False
        return True

    print()
    print("임시 IP 추가 실패.")
    print("관리자 권한 CMD/PowerShell에서 실행했는지 확인하십시오.")
    print(f'어댑터 이름이 "{ETH_IF_NAME}"인지도 확인하십시오.')
    print("현재 어댑터 이름은 ipconfig에서 확인 가능합니다.")
    return False


def delete_temp_ip():
    """
    프로그램 종료 시 보조 IP를 삭제합니다.

    이미 존재하던 IP까지 지우기 싫다면 temp_ip_added_by_this_process가 True일 때만 삭제합니다.
    현재는 비정상 종료 후 남은 IP까지 정리하기 위해 항상 삭제합니다.
    """
    cmd = (
        f'netsh interface ip delete address '
        f'name="{ETH_IF_NAME}" '
        f'addr={PC_ETHERNET_IP}'
    )

    print(f"[NETSH] delete temp IP: {PC_ETHERNET_IP}")
    run_cmd(cmd)


# =========================
# Clamp helpers
# =========================
def clamp_int(value: int, min_value: int, max_value: int) -> int:
    value = int(value)

    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


# =========================
# UDP packet functions
# =========================
def send_asms(mode: int, value: int = 0):
    """
    ASMS packet: 5 bytes

    STM32 ethernet.c 기준:
    byte[0]   : mode
    byte[1]   : unused
    byte[2]   : unused
    byte[3:4] : joystick value, little-endian int16_t

    value 범위:
    -2048 ~ +2047

    의미:
    value < 0  : 좌회전 방향 명령
    value = 0  : 중립
    value > 0  : 우회전 방향 명령
    """
    global sock

    value = clamp_int(value, ASMS_MIN, ASMS_MAX)

    # < : little-endian
    # B : uint8_t mode
    # B : unused
    # B : unused
    # h : int16_t signed ASMS value
    packet = struct.pack("<BBBh", int(mode), 0, 0, value)
    sock.sendto(packet, (STM32_IP, STM32_PORT))


def send_pc(deg: int, speed_raw: int = 0, estop: bool = False):
    """
    PC packet: 9 bytes

    byte[0:3] : steering raw, little-endian int32_t
    byte[4:7] : speed raw, little-endian uint32_t
    byte[8]   : misc, bit7 = emergency stop
    """
    global sock

    deg = clamp_int(deg, -60, 60)
    speed_raw = clamp_int(speed_raw, 0, 0xFFFFFFFF)
    misc = 0x80 if estop else 0x00

    packet = struct.pack("<iIB", deg, speed_raw, misc)
    sock.sendto(packet, (STM32_IP, STM32_PORT))


# =========================
# TX thread
# =========================
def tx_loop():
    global running, active_mode, target_deg, asms_value

    tx_count = 0

    while running:
        try:
            if active_mode == "pc":
                # STM32는 PC packet을 AUTO 모드에서만 인정하므로
                # AUTO ASMS packet도 주기적으로 같이 보냅니다.
                send_asms(STEER_MODE_AUTO, 0)
                send_pc(target_deg)

                tx_count += 1
                if tx_count % 50 == 0:
                    print(f"[TX] PC AUTO, target={target_deg} deg")

            elif active_mode == "asms":
                send_asms(STEER_MODE_MANUAL, asms_value)

                tx_count += 1
                if tx_count % 50 == 0:
                    print(f"[TX] ASMS MANUAL, value={asms_value}")

        except OSError as e:
            print(f"[UDP ERROR] {e}")

        time.sleep(TX_PERIOD_S)


# =========================
# CLI modes
# =========================
def pc_mode():
    global active_mode, target_deg

    active_mode = "pc"

    # 즉시 AUTO 모드 진입 packet 1회 송신
    send_asms(STEER_MODE_AUTO, 0)

    print()
    print("[PC MODE]")
    print("각도값[deg] 입력: 예) 0, 5, -5, 10")
    print("범위는 -60 ~ +60 deg로 제한됩니다.")
    print("q: 모드 선택으로 복귀")
    print("e: PC ESTOP 전송")

    while True:
        s = input("pc target deg> ").strip().lower()

        if s == "q":
            active_mode = "idle"
            print("[PC MODE] stop TX")
            return

        if s == "e":
            send_pc(target_deg, estop=True)
            active_mode = "idle"
            print("[PC MODE] ESTOP sent")
            return

        try:
            value = int(s)
        except ValueError:
            print("invalid input")
            continue

        target_deg = clamp_int(value, -60, 60)
        print(f"PC target updated: {target_deg} deg")


def asms_mode():
    global active_mode, asms_value

    active_mode = "asms"

    print()
    print("[ASMS MODE]")
    print("ASMS signed value 입력: -2048 ~ +2047")
    print("현재 final steering test 기준:")
    print("  +2047 ≈ -60 deg")
    print("  +1000 ≈ -29 deg")
    print("  0     = 0 deg")
    print("  -1000 ≈ +29 deg")
    print("  -2048 ≈ +60 deg")
    print("0 근처는 deadband 때문에 0 deg로 처리될 수 있습니다.")
    print("테스트 예: -1000, -500, 0, 500, 1000")
    print("q: 모드 선택으로 복귀")
    print("e: ASMS ESTOP 전송")

    while True:
        s = input("asms signed value> ").strip().lower()

        if s == "q":
            active_mode = "idle"
            print("[ASMS MODE] stop TX")
            return

        if s == "e":
            send_asms(STEER_MODE_ESTOP, 0)
            active_mode = "idle"
            print("[ASMS MODE] ESTOP sent")
            return

        try:
            value = int(s)
        except ValueError:
            print("invalid input")
            continue

        asms_value = clamp_int(value, ASMS_MIN, ASMS_MAX)
        print(f"ASMS value updated: {asms_value}")


# =========================
# Main
# =========================
def main():
    global sock, running, active_mode

    print("STM32 UDP steering debugger")
    print(f"STM32 target = {STM32_IP}:{STM32_PORT}")
    print(f"PC temp IP   = {PC_ETHERNET_IP}/{PC_ETHERNET_MASK}")
    print(f"Adapter name = {ETH_IF_NAME}")
    print()

    # 1. 임시 IP 추가
    if not add_temp_ip():
        print()
        print("임시 IP 추가에 실패했으므로 종료합니다.")
        print("관리자 권한으로 다시 실행하십시오.")
        sys.exit(1)

    time.sleep(1.0)

    # 종료 시 임시 IP 삭제
    atexit.register(delete_temp_ip)

    # 2. UDP socket 생성
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except OSError as e:
        print(f"[SOCKET CREATE FAILED] {e}")
        sys.exit(1)

    print("[SOCKET] created")
    print()

    # 3. 송신 thread 시작
    thread = threading.Thread(target=tx_loop, daemon=True)
    thread.start()

    print("commands:")
    print("  pc   : PC angle command mode")
    print("  asms : ASMS signed command mode")
    print("  e    : ASMS ESTOP")
    print("  q    : quit")

    try:
        while True:
            cmd = input("\nmode(pc/asms/e/q)> ").strip().lower()

            if cmd == "q":
                running = False
                active_mode = "idle"
                break

            if cmd == "e":
                send_asms(STEER_MODE_ESTOP, 0)
                active_mode = "idle"
                print("ASMS ESTOP sent")
                continue

            if cmd == "pc":
                pc_mode()
                continue

            if cmd == "asms":
                asms_mode()
                continue

            print("invalid command")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")
        running = False
        active_mode = "idle"

    finally:
        running = False
        active_mode = "idle"
        time.sleep(0.05)

        if sock is not None:
            sock.close()

        # 정상 종료 시 바로 삭제
        delete_temp_ip()

        print("closed")


if __name__ == "__main__":
    main()