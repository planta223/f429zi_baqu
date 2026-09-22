import socket
import struct
import threading
import time
import subprocess
import atexit
import sys
import locale

# ============================================================
# Network configuration
# ============================================================
STM32_IP = "10.177.21.4"
STM32_PORT = 5000

# Firmware IP filter와 맞춰 각 packet source IP를 분리합니다.
PC_SOURCE_IP = "10.177.21.1"
ASMS_SOURCE_IP = "10.177.21.5"
ETHERNET_MASK = "255.255.255.0"

# ipconfig에 표시되는 Windows Ethernet adapter 이름과 일치해야 합니다.
ETH_IF_NAME = "이더넷"

# ============================================================
# Protocol configuration
# ============================================================
STEER_MODE_AUTO = 1
STEER_MODE_MANUAL = 2
STEER_MODE_ESTOP = 3

TX_PERIOD_S = 0.02  # 50 Hz
IP_READY_TIMEOUT_S = 10.0
IP_READY_POLL_S = 0.1

# PC command: 현재 firmware 기준 raw 1 LSB = 1 deg
PC_STEER_MIN_DEG = -30
PC_STEER_MAX_DEG = 30

# ASMS joystick ADC raw range
ASMS_MIN = -2048
ASMS_CENTER = 0
ASMS_MAX = 2047

# ============================================================
# Global state
# ============================================================
pc_sock = None
asms_sock = None

active_mode = "idle"  # "pc", "asms", "idle"
target_deg = 0
asms_value = 0
running = True

# 이 실행에서 실제로 추가한 IP만 종료 시 삭제합니다.
added_temp_ips = set()


# ============================================================
# Windows IP helper
# ============================================================
def run_cmd(cmd):
    """Windows command를 현재 시스템 문자 인코딩으로 실행합니다."""
    result = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        encoding=locale.getpreferredencoding(False),
        errors="replace",
    )

    stdout = result.stdout
    stderr = result.stderr
    output = stdout + stderr

    if result.returncode != 0:
        print("[CMD FAILED]")
        print(subprocess.list2cmdline(cmd))

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


def can_bind_source_ip(ip_address: str) -> bool:
    """해당 IPv4 주소가 현재 유효한 로컬 주소인지 직접 확인합니다."""
    test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        test_sock.bind((ip_address, 0))
        return True
    except OSError:
        return False
    finally:
        test_sock.close()


def wait_for_source_ip(ip_address: str) -> bool:
    """DAD 완료 후 주소가 실제 bind 가능해질 때까지 기다립니다."""
    deadline = time.monotonic() + IP_READY_TIMEOUT_S

    while time.monotonic() < deadline:
        if can_bind_source_ip(ip_address):
            print(f"[IP READY] {ip_address}")
            return True

        time.sleep(IP_READY_POLL_S)

    print(f"[IP READY TIMEOUT] {ip_address}")
    print("주소가 Tentative/Duplicate 상태인지 확인하십시오:")
    print(
        f'  Get-NetIPAddress -InterfaceAlias "{ETH_IF_NAME}" '
        f'-AddressFamily IPv4'
    )
    return False


def ensure_source_ip(ip_address: str) -> bool:
    """
    Ethernet adapter에 테스트용 IPv4 address를 추가합니다.

    이미 존재하는 IP이면 그대로 사용하고,
    이 프로그램이 새로 추가한 IP만 종료 시 삭제합니다.
    """
    if can_bind_source_ip(ip_address):
        print(f"[IP READY] {ip_address} already configured; reusing it")
        return True

    cmd = [
        "netsh",
        "interface",
        "ipv4",
        "add",
        "address",
        f"name={ETH_IF_NAME}",
        f"address={ip_address}",
        f"mask={ETHERNET_MASK}",
        "store=active",
    ]

    print(f"[NETSH] add IP: {ip_address}/{ETHERNET_MASK}")
    ok, output = run_cmd(cmd)

    if ok:
        added_temp_ips.add(ip_address)
        return wait_for_source_ip(ip_address)

    lower_output = output.lower()

    # 이전 설정 또는 수동 설정으로 이미 존재하는 경우
    if (
        "개체가 이미 있습니다" in output
        or "already exists" in lower_output
        or "object already exists" in lower_output
    ):
        print(f"[NETSH] {ip_address} already exists; waiting until ready")
        return wait_for_source_ip(ip_address)

    print()
    print(f"IP 추가 실패: {ip_address}")
    print("관리자 권한 CMD/PowerShell에서 실행했는지 확인하십시오.")
    print(f'어댑터 이름이 "{ETH_IF_NAME}"인지 확인하십시오.')
    return False


def delete_temp_ip(ip_address: str):
    """이 프로그램이 추가한 보조 IP를 삭제합니다."""
    if ip_address not in added_temp_ips:
        return

    cmd = [
        "netsh",
        "interface",
        "ipv4",
        "delete",
        "address",
        f"name={ETH_IF_NAME}",
        f"address={ip_address}",
    ]

    print(f"[NETSH] delete IP: {ip_address}")
    run_cmd(cmd)
    added_temp_ips.discard(ip_address)


def cleanup_temp_ips():
    # set을 순회 중 수정하지 않도록 복사본 사용
    for ip_address in list(added_temp_ips):
        delete_temp_ip(ip_address)


# ============================================================
# Utility
# ============================================================
def clamp_int(value: int, min_value: int, max_value: int) -> int:
    value = int(value)

    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


def create_bound_udp_socket(source_ip: str):
    """
    source IP를 명시적으로 고정한 UDP socket을 생성합니다.

    Firmware가 송신 IP를 검사하므로:
      PC   packet -> 10.177.21.1
      ASMS packet -> 10.177.21.5
    에서 송신되도록 각각 별도 socket을 사용합니다.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((source_ip, 0))
    return sock


# ============================================================
# UDP packet functions
# ============================================================
def send_pc(deg: int, speed_raw: int = 0, estop: bool = False):
    """
    PC UDP packet: 9 bytes

    Byte 0~3 : steer raw, little-endian int32
    Byte 4~7 : speed raw, little-endian uint32 (현재 미사용)
    Byte 8   : misc, bit7 = E-Stop

    현재 firmware:
      PC_STEER_SCALE = 1.0
      PC_STEER_POLARITY = +1
    따라서 raw 값은 degree 정수와 동일하게 사용합니다.
    """
    global pc_sock

    if pc_sock is None:
        raise RuntimeError("PC UDP socket is not initialized")

    deg = clamp_int(deg, PC_STEER_MIN_DEG, PC_STEER_MAX_DEG)
    speed_raw = clamp_int(speed_raw, 0, 0xFFFFFFFF)
    misc = 0x80 if estop else 0x00

    packet = struct.pack("<iIB", deg, speed_raw, misc)
    pc_sock.sendto(packet, (STM32_IP, STM32_PORT))


def send_asms(mode: int, value: int = 0):
    """
    ASMS UDP packet: 5 bytes

    Byte 0   : mode
    Byte 1~2 : speed/reserved, 현재 미사용
    Byte 3~4 : steer ADC raw, little-endian int16
    """
    global asms_sock

    if asms_sock is None:
        raise RuntimeError("ASMS UDP socket is not initialized")

    value = clamp_int(value, ASMS_MIN, ASMS_MAX)

    packet = struct.pack("<BBBh", int(mode), 0, 0, value)
    asms_sock.sendto(packet, (STM32_IP, STM32_PORT))


# ============================================================
# TX thread
# ============================================================
def tx_loop():
    global running, active_mode, target_deg, asms_value

    tx_count = 0

    while running:
        try:
            if active_mode == "pc":
                # PC_ALLOW_AUTO_ENTRY = 1 이므로
                # ASMS AUTO packet 없이 PC packet만 송신합니다.
                send_pc(target_deg)

                tx_count += 1
                if tx_count % 50 == 0:
                    print(f"[TX][PC] target={target_deg} deg")

            elif active_mode == "asms":
                send_asms(STEER_MODE_MANUAL, asms_value)

                tx_count += 1
                if tx_count % 50 == 0:
                    print(f"[TX][ASMS] MANUAL raw={asms_value}")

        except OSError as e:
            print(f"[UDP ERROR] {e}")
        except RuntimeError as e:
            print(f"[STATE ERROR] {e}")

        time.sleep(TX_PERIOD_S)


# ============================================================
# CLI modes
# ============================================================
def pc_mode():
    global active_mode, target_deg

    active_mode = "pc"

    print()
    print("[PC MODE]")
    print(f"각도값[deg] 입력: {PC_STEER_MIN_DEG} ~ {PC_STEER_MAX_DEG}")
    print("예: 0, 5, -5, 10, -30, 30")
    print("q: 모드 선택으로 복귀")
    print("e: PC E-Stop 1회 전송 후 idle")
    print("주의: MANUAL/ESTOP 상태였다면 먼저 auto 명령을 보내야 합니다.")

    while True:
        s = input("pc target deg> ").strip().lower()

        if s == "q":
            active_mode = "idle"
            print("[PC MODE] stop TX")
            return

        if s == "e":
            # periodic normal TX를 먼저 정지한 뒤 E-Stop 송신
            active_mode = "idle"
            send_pc(target_deg, estop=True)
            print("[PC MODE] E-Stop sent")
            return

        try:
            value = int(s)
        except ValueError:
            print("invalid input")
            continue

        target_deg = clamp_int(
            value,
            PC_STEER_MIN_DEG,
            PC_STEER_MAX_DEG,
        )
        print(f"PC target updated: {target_deg} deg")


def asms_mode():
    global active_mode, asms_value

    active_mode = "asms"

    print()
    print("[ASMS MODE]")
    print(f"ASMS signed raw 입력: {ASMS_MIN} ~ {ASMS_MAX}")
    print(f"중립값: {ASMS_CENTER}")
    print("0 근처는 firmware deadband에 의해 0 deg로 처리될 수 있습니다.")
    print("예: -1000, -500, 0, 500, 1000")
    print("q: 모드 선택으로 복귀")
    print("e: ASMS E-Stop 1회 전송 후 idle")

    while True:
        s = input("asms signed raw> ").strip().lower()

        if s == "q":
            active_mode = "idle"
            print("[ASMS MODE] stop TX")
            return

        if s == "e":
            active_mode = "idle"
            send_asms(STEER_MODE_ESTOP, 0)
            print("[ASMS MODE] E-Stop sent")
            return

        try:
            value = int(s)
        except ValueError:
            print("invalid input")
            continue

        asms_value = clamp_int(value, ASMS_MIN, ASMS_MAX)
        print(f"ASMS value updated: {asms_value}")


# ============================================================
# One-shot commands
# ============================================================
def send_pc_estop():
    global active_mode

    active_mode = "idle"
    send_pc(target_deg, estop=True)
    print("[PC] E-Stop sent")


def send_asms_estop():
    global active_mode

    active_mode = "idle"
    send_asms(STEER_MODE_ESTOP, 0)
    print("[ASMS] E-Stop sent")


def send_asms_auto_once():
    """
    실제 ASMS AUTO mode transition 자체를 확인할 때만 사용합니다.
    PC mode에서는 이 packet을 자동 송신하지 않습니다.
    """
    global active_mode

    active_mode = "idle"
    send_asms(STEER_MODE_AUTO, 0)
    print("[ASMS] AUTO mode packet sent once")


def start_zero_degree_test():
    """검증된 ASMS MANUAL 0도 패킷을 50 Hz로 계속 송신합니다."""
    global active_mode, asms_value

    asms_value = 0
    active_mode = "asms"

    print("[ZERO TEST] ASMS MANUAL target=0, 50 Hz")
    print("[ZERO TEST] 정지하려면 idle을 입력하십시오.")
    print("[EXPECT] RED OFF, GREEN ON while packets are accepted")
    print(
        "[NOTE] GREEN ON인데 SVON OFF이면 UDP는 정상이며 "
        "firmware의 Encoder_IsInitialized() 조건을 확인하십시오."
    )


def print_runtime_status():
    print(f"[STATUS] TX mode     = {active_mode}")
    print(f"[STATUS] PC socket   = {pc_sock.getsockname() if pc_sock else None}")
    print(f"[STATUS] ASMS socket = {asms_sock.getsockname() if asms_sock else None}")
    print(f"[STATUS] target      = {STM32_IP}:{STM32_PORT}")
    print("[STATUS] UDP에는 STM32 ACK가 없으므로 LED/디버거로 수신을 판정합니다.")


# ============================================================
# Main
# ============================================================
def main():
    global pc_sock, asms_sock, running, active_mode

    print("STM32 UDP steering debugger v2")
    print(f"STM32 target = {STM32_IP}:{STM32_PORT}")
    print(f"PC source    = {PC_SOURCE_IP}")
    print(f"ASMS source  = {ASMS_SOURCE_IP}")
    print(f"Netmask      = {ETHERNET_MASK}")
    print(f"Adapter      = {ETH_IF_NAME}")
    print()

    # 준비 도중 종료되더라도 이 실행에서 추가한 주소는 정리합니다.
    atexit.register(cleanup_temp_ips)

    # 1. Firmware IP filter와 맞는 source IP 두 개 준비
    if not ensure_source_ip(PC_SOURCE_IP):
        print("PC source IP 준비 실패. 종료합니다.")
        sys.exit(1)

    if not ensure_source_ip(ASMS_SOURCE_IP):
        print("ASMS source IP 준비 실패. 종료합니다.")
        cleanup_temp_ips()
        sys.exit(1)

    # 2. source IP별 UDP socket 생성
    try:
        pc_sock = create_bound_udp_socket(PC_SOURCE_IP)
    except OSError as e:
        print(f"[PC SOCKET BIND FAILED] {PC_SOURCE_IP}: {e}")
        cleanup_temp_ips()
        sys.exit(1)

    try:
        asms_sock = create_bound_udp_socket(ASMS_SOURCE_IP)
    except OSError as e:
        print(f"[ASMS SOCKET BIND FAILED] {ASMS_SOURCE_IP}: {e}")

        if pc_sock is not None:
            pc_sock.close()
            pc_sock = None

        cleanup_temp_ips()
        sys.exit(1)

    print(f"[SOCKET] PC   bound to {pc_sock.getsockname()[0]}")
    print(f"[SOCKET] ASMS bound to {asms_sock.getsockname()[0]}")
    print()

    # 3. 50 Hz periodic TX thread
    thread = threading.Thread(target=tx_loop, daemon=True)
    thread.start()

    print("commands:")
    print("  pc     : PC steering command mode (50 Hz)")
    print("  asms   : ASMS MANUAL command mode (50 Hz)")
    print("  epc    : PC E-Stop 1회 전송")
    print("  easms  : ASMS E-Stop 1회 전송")
    print("  auto   : ASMS AUTO packet 1회 전송")
    print("  zero   : ASMS MANUAL 0도 연속 송신 (50 Hz)")
    print("  status : 현재 socket/TX 상태 표시")
    print("  idle   : periodic TX 정지")
    print("  q      : quit")

    try:
        while True:
            cmd = input("\nmode(pc/asms/epc/easms/auto/idle/q)> ").strip().lower()

            if cmd == "q":
                running = False
                active_mode = "idle"
                break

            if cmd == "idle":
                active_mode = "idle"
                print("[TX] idle")
                continue

            if cmd == "epc":
                send_pc_estop()
                continue

            if cmd == "easms":
                send_asms_estop()
                continue

            if cmd == "auto":
                send_asms_auto_once()
                continue

            if cmd == "zero":
                start_zero_degree_test()
                continue

            if cmd == "status":
                print_runtime_status()
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

        if pc_sock is not None:
            pc_sock.close()

        if asms_sock is not None:
            asms_sock.close()

        cleanup_temp_ips()
        print("closed")


if __name__ == "__main__":
    main()
