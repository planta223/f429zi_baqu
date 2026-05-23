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

# =========================
# Global state
# =========================
sock = None

active_mode = "idle"   # "pc", "asms", "idle"
target_deg = 0
adc_raw = 2048
running = True


# =========================
# Windows IP helper
# =========================
def run_cmd(cmd: str) -> bool:
    result = subprocess.run(
        cmd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    stdout = result.stdout.decode("utf-8", errors="ignore")
    stderr = result.stderr.decode("utf-8", errors="ignore")

    if result.returncode != 0:
        print("[CMD FAILED]")
        print(cmd)

        if stdout:
            print("[stdout]")
            print(stdout)

        if stderr:
            print("[stderr]")
            print(stderr)

        return False

    if stdout:
        print("[stdout]")
        print(stdout)

    if stderr:
        print("[stderr]")
        print(stderr)

    return True


def add_temp_ip() -> bool:
    """
    이더넷 어댑터에 10.177.21.1/24 보조 IP를 추가합니다.
    관리자 권한 필요.
    """
    cmd = (
        f'netsh interface ip add address '
        f'name="{ETH_IF_NAME}" '
        f'addr={PC_ETHERNET_IP} '
        f'mask={PC_ETHERNET_MASK} '
        f'store=active'
    )

    print(f"[NETSH] add temp IP: {PC_ETHERNET_IP}/{PC_ETHERNET_MASK}")
    ok = run_cmd(cmd)

    if not ok:
        print()
        print("임시 IP 추가 실패.")
        print("관리자 권한 CMD/PowerShell에서 실행했는지 확인하십시오.")
        print(f'어댑터 이름이 "{ETH_IF_NAME}"인지도 확인하십시오.')
        print("현재 어댑터 이름은 ipconfig에서 확인 가능합니다.")
        return False

    return True


def delete_temp_ip():
    """
    프로그램 종료 시 추가했던 보조 IP를 삭제합니다.
    이미 삭제되어 있거나 추가 실패 상태여도 큰 문제는 없습니다.
    """
    cmd = (
        f'netsh interface ip delete address '
        f'name="{ETH_IF_NAME}" '
        f'addr={PC_ETHERNET_IP}'
    )

    print(f"[NETSH] delete temp IP: {PC_ETHERNET_IP}")
    run_cmd(cmd)


# =========================
# UDP packet functions
# =========================
def send_asms(mode: int, adc: int = 2048):
    """
    ASMS packet: 5 bytes

    byte[0]   : mode
    byte[1]   : unused
    byte[2]   : unused
    byte[3:4] : ADC raw, little-endian uint16_t
    """
    global sock

    adc = max(0, min(4095, int(adc)))
    packet = struct.pack("<BBBH", int(mode), 0, 0, adc)
    sock.sendto(packet, (STM32_IP, STM32_PORT))


def send_pc(deg: int, speed_raw: int = 0, estop: bool = False):
    """
    PC packet: 9 bytes

    byte[0:3] : steering raw, little-endian int32_t
    byte[4:7] : speed raw, little-endian uint32_t
    byte[8]   : misc, bit7 = emergency stop
    """
    global sock

    deg = max(-60, min(60, int(deg)))
    speed_raw = max(0, min(0xFFFFFFFF, int(speed_raw)))
    misc = 0x80 if estop else 0x00

    packet = struct.pack("<iIB", deg, speed_raw, misc)
    sock.sendto(packet, (STM32_IP, STM32_PORT))


# =========================
# TX thread
# =========================
def tx_loop():
    global running, active_mode, target_deg, adc_raw

    tx_count = 0

    while running:
        try:
            if active_mode == "pc":
                # STM32는 PC packet을 AUTO 모드에서만 인정하므로
                # AUTO ASMS packet도 주기적으로 같이 보냅니다.
                send_asms(STEER_MODE_AUTO, 2048)
                send_pc(target_deg)

                tx_count += 1
                if tx_count % 50 == 0:
                    print(f"[TX] PC AUTO, target={target_deg} deg")

            elif active_mode == "asms":
                send_asms(STEER_MODE_MANUAL, adc_raw)

                tx_count += 1
                if tx_count % 50 == 0:
                    print(f"[TX] ASMS MANUAL, adc={adc_raw}")

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
    send_asms(STEER_MODE_AUTO, 2048)

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

        target_deg = max(-60, min(60, value))
        print(f"PC target updated: {target_deg} deg")


def asms_mode():
    global active_mode, adc_raw

    active_mode = "asms"

    print()
    print("[ASMS MODE]")
    print("ADC raw 입력: 0 ~ 4095")
    print("중립값: 2048")
    print("2048 근처는 deadband 때문에 0 deg로 처리될 수 있습니다.")
    print("테스트 예: 1000, 1500, 2500, 3000")
    print("q: 모드 선택으로 복귀")
    print("e: ASMS ESTOP 전송")

    while True:
        s = input("asms adc raw> ").strip().lower()

        if s == "q":
            active_mode = "idle"
            print("[ASMS MODE] stop TX")
            return

        if s == "e":
            send_asms(STEER_MODE_ESTOP, 2048)
            active_mode = "idle"
            print("[ASMS MODE] ESTOP sent")
            return

        try:
            value = int(s)
        except ValueError:
            print("invalid input")
            continue

        adc_raw = max(0, min(4095, value))
        print(f"ASMS adc updated: {adc_raw}")


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

    # 비정상 종료가 아니면 종료 시 임시 IP 삭제
    atexit.register(delete_temp_ip)

    # 2. UDP socket 생성 및 PC 이더넷 IP에 bind
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
    print("  asms : ASMS adc command mode")
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
                send_asms(STEER_MODE_ESTOP, 2048)
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

        # atexit에도 등록했지만, 정상 종료 시 바로 삭제
        delete_temp_ip()

        print("closed")


if __name__ == "__main__":
    main()