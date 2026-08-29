import argparse
import struct
import threading
import time
import sys

try:
    import can
except ImportError:
    print("python-can 패키지가 필요합니다.")
    print("설치:")
    print("  pip install python-can")
    sys.exit(1)


# ============================================================
# Default CANable configuration
# ============================================================
# 필요하면 실행 인자로 덮어쓸 수 있습니다.
#
# SLCAN 예:
#   python CANable_test.py --interface slcan --channel COM5 --bitrate 500000
#
# candleLight / gs_usb 예:
#   python CANable_test.py --interface gs_usb --channel 0 --bitrate 500000
#
DEFAULT_INTERFACE = "slcan"
DEFAULT_CHANNEL = "COM5"
DEFAULT_BITRATE = 500_000


# ============================================================
# Protocol configuration
# ============================================================
CAN_ID_STEER_REQUEST = 0x100
CAN_ID_STEER_STATUS = 0x180

CAN_REQUEST_DLC = 3
CAN_STATUS_DLC = 5

CAN_REQUEST_STEER_SCALE = 100.0   # raw = deg * 100
CAN_STATUS_STEER_SCALE = 100.0    # raw = deg * 100

CAN_REQUEST_ESTOP_MASK = 0x80

CAN_STATUS_FLAG_REACHED = 1 << 0
CAN_STATUS_FLAG_CONTROL_ENABLED = 1 << 1
CAN_STATUS_FLAG_SVON_ENABLED = 1 << 2
CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE = 1 << 3
CAN_STATUS_FLAG_ENCODER_INITIALIZED = 1 << 4

STEER_MIN_DEG = -30.0
STEER_MAX_DEG = 30.0

TX_PERIOD_S = 0.02            # 50 Hz
STATUS_PRINT_PERIOD_S = 0.20  # 자동 상태 출력은 최대 5 Hz


# ============================================================
# Global state
# ============================================================
bus = None
running = True

tx_enabled = False
target_deg = 0.0

auto_print_status = True

latest_status = None
latest_status_lock = threading.Lock()

tx_state_lock = threading.Lock()


# ============================================================
# Utility
# ============================================================
def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def deg_to_request_raw(deg):
    deg = clamp(float(deg), STEER_MIN_DEG, STEER_MAX_DEG)

    raw = int(round(deg * CAN_REQUEST_STEER_SCALE))
    raw = clamp(raw, -32768, 32767)

    return raw


def status_raw_to_deg(raw):
    return float(raw) / CAN_STATUS_STEER_SCALE


def flags_to_text(flags):
    names = []

    if flags & CAN_STATUS_FLAG_REACHED:
        names.append("REACHED")

    if flags & CAN_STATUS_FLAG_CONTROL_ENABLED:
        names.append("CONTROL")

    if flags & CAN_STATUS_FLAG_SVON_ENABLED:
        names.append("SVON")

    if flags & CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE:
        names.append("MOTOR")

    if flags & CAN_STATUS_FLAG_ENCODER_INITIALIZED:
        names.append("ENCODER")

    if not names:
        return "-"

    return "|".join(names)


# ============================================================
# CAN bus open
# ============================================================
def open_bus(interface, channel, bitrate):
    """
    python-can backend를 엽니다.

    slcan:
      channel = COM5 같은 문자열

    gs_usb:
      channel = 0 같은 숫자
    """

    if interface == "gs_usb":
        try:
            channel_value = int(channel)
        except ValueError:
            channel_value = channel
    else:
        channel_value = channel

    return can.Bus(
        interface=interface,
        channel=channel_value,
        bitrate=bitrate,
    )


# ============================================================
# TX packet functions
# ============================================================
def send_steer_once(deg):
    """
    Steering Request:
      ID       = 0x100
      Standard = 11-bit
      DLC      = 3

      Byte 0~1 = steer raw, little-endian int16
      Byte 2   = flags
    """
    global bus

    raw = deg_to_request_raw(deg)

    data = struct.pack(
        "<hB",
        raw,
        0x00,
    )

    msg = can.Message(
        arbitration_id=CAN_ID_STEER_REQUEST,
        is_extended_id=False,
        is_remote_frame=False,
        data=data,
    )

    bus.send(msg)

    return raw


def send_estop_once():
    """
    CAN E-Stop:
      ID       = 0x100
      steer    = 0
      flags    = 0x80

    현재 firmware 정책:
      E-Stop 수신 시 기존 normal pending request 폐기
      이후 새 normal request가 들어오면 ESTOP 해제 / AUTO 복귀
    """
    global bus

    data = struct.pack(
        "<hB",
        0,
        CAN_REQUEST_ESTOP_MASK,
    )

    msg = can.Message(
        arbitration_id=CAN_ID_STEER_REQUEST,
        is_extended_id=False,
        is_remote_frame=False,
        data=data,
    )

    bus.send(msg)


# ============================================================
# Periodic TX thread
# ============================================================
def tx_loop():
    global running

    tx_count = 0

    while running:
        with tx_state_lock:
            enabled = tx_enabled
            deg = target_deg

        if enabled:
            try:
                raw = send_steer_once(deg)

                tx_count += 1

                if tx_count % 50 == 0:
                    print(
                        f"[TX] ID=0x{CAN_ID_STEER_REQUEST:03X} "
                        f"target={deg:+.2f} deg raw={raw}"
                    )

            except can.CanError as e:
                print(f"[CAN TX ERROR] {e}")

        time.sleep(TX_PERIOD_S)


# ============================================================
# RX Status
# ============================================================
def decode_status(msg):
    if msg.is_extended_id:
        return None

    if msg.is_remote_frame:
        return None

    if msg.arbitration_id != CAN_ID_STEER_STATUS:
        return None

    if len(msg.data) != CAN_STATUS_DLC:
        return None

    actual_raw, target_raw, flags = struct.unpack(
        "<hhB",
        bytes(msg.data),
    )

    return {
        "timestamp": time.time(),
        "actual_raw": actual_raw,
        "target_raw": target_raw,
        "actual_deg": status_raw_to_deg(actual_raw),
        "target_deg": status_raw_to_deg(target_raw),
        "flags": flags,

        "reached": bool(flags & CAN_STATUS_FLAG_REACHED),
        "control_enabled": bool(
            flags & CAN_STATUS_FLAG_CONTROL_ENABLED
        ),
        "svon_enabled": bool(
            flags & CAN_STATUS_FLAG_SVON_ENABLED
        ),
        "motor_output_active": bool(
            flags & CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE
        ),
        "encoder_initialized": bool(
            flags & CAN_STATUS_FLAG_ENCODER_INITIALIZED
        ),
    }


def print_status(status, prefix="[RX]"):
    if status is None:
        print(f"{prefix} no 0x{CAN_ID_STEER_STATUS:03X} status received yet")
        return

    flags_text = flags_to_text(status["flags"])

    print(
        f"{prefix} ID=0x{CAN_ID_STEER_STATUS:03X} "
        f"actual={status['actual_deg']:+.2f} deg "
        f"target={status['target_deg']:+.2f} deg "
        f"flags=0x{status['flags']:02X} [{flags_text}]"
    )


def rx_loop():
    global running
    global latest_status

    last_print_time = 0.0

    while running:
        try:
            msg = bus.recv(timeout=0.1)

        except can.CanError as e:
            print(f"[CAN RX ERROR] {e}")
            time.sleep(0.1)
            continue

        if msg is None:
            continue

        status = decode_status(msg)

        if status is None:
            continue

        with latest_status_lock:
            latest_status = status

        now = time.monotonic()

        if (
            auto_print_status
            and (now - last_print_time) >= STATUS_PRINT_PERIOD_S
        ):
            print_status(status)
            last_print_time = now


# ============================================================
# CLI commands
# ============================================================
def set_periodic_target(deg):
    global tx_enabled
    global target_deg

    deg = clamp(
        float(deg),
        STEER_MIN_DEG,
        STEER_MAX_DEG,
    )

    with tx_state_lock:
        target_deg = deg
        tx_enabled = True

    print(
        f"[TX] periodic steering enabled: "
        f"{target_deg:+.2f} deg @ {1.0 / TX_PERIOD_S:.0f} Hz"
    )


def stop_periodic_tx():
    global tx_enabled

    with tx_state_lock:
        tx_enabled = False

    print("[TX] periodic steering stopped")


def command_estop():
    global tx_enabled

    # 중요:
    # periodic normal frame이 ESTOP 직후 다시 나가면
    # 현재 firmware 정책상 ESTOP이 즉시 해제될 수 있습니다.
    # 따라서 먼저 periodic TX를 끄고 E-Stop을 1회 송신합니다.
    with tx_state_lock:
        tx_enabled = False

    send_estop_once()

    print(
        "[TX] CAN E-Stop sent. "
        "Periodic steering is stopped."
    )
    print(
        "[INFO] 새 's <deg>' 명령을 입력하면 "
        "normal CAN request가 전송되어 ESTOP이 해제됩니다."
    )


def show_latest_status():
    with latest_status_lock:
        status = None if latest_status is None else dict(latest_status)

    print_status(status, prefix="[STATUS]")


def set_auto_print(value):
    global auto_print_status

    auto_print_status = bool(value)

    print(
        "[RX] automatic status print "
        + ("ON" if auto_print_status else "OFF")
    )


def print_help():
    print()
    print("commands:")
    print("  s <deg>       : steering target을 50 Hz로 지속 송신")
    print("                  예) s 10, s -5.5, s 0")
    print("  <deg>         : 숫자만 입력해도 s <deg>와 동일")
    print("  once <deg>    : steering request 1회만 송신")
    print("  e             : CAN E-Stop 1회 송신 + periodic TX 정지")
    print("  idle          : periodic TX 정지 (300 ms timeout 시험용)")
    print("  status        : 가장 최근 0x180 Status 1회 출력")
    print("  rx on         : Status 자동 출력 ON")
    print("  rx off        : Status 자동 출력 OFF")
    print("  help          : 도움말")
    print("  q             : 종료")
    print()


# ============================================================
# Arguments
# ============================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="STM32 steering CANable debugger"
    )

    parser.add_argument(
        "--interface",
        default=DEFAULT_INTERFACE,
        choices=["slcan", "gs_usb"],
        help="python-can backend (default: slcan)",
    )

    parser.add_argument(
        "--channel",
        default=DEFAULT_CHANNEL,
        help="slcan: COM5 / gs_usb: 0",
    )

    parser.add_argument(
        "--bitrate",
        type=int,
        default=DEFAULT_BITRATE,
        help="CAN bitrate. STM32 CubeMX 설정과 반드시 일치해야 함.",
    )

    return parser.parse_args()


# ============================================================
# Main
# ============================================================
def main():
    global bus
    global running

    args = parse_args()

    print("STM32 CANable steering debugger")
    print(f"interface = {args.interface}")
    print(f"channel   = {args.channel}")
    print(f"bitrate   = {args.bitrate}")
    print(f"TX ID     = 0x{CAN_ID_STEER_REQUEST:03X}")
    print(f"RX ID     = 0x{CAN_ID_STEER_STATUS:03X}")
    print()

    try:
        bus = open_bus(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
        )

    except Exception as e:
        print("[CAN OPEN FAILED]")
        print(e)
        print()
        print("확인할 것:")
        print("  1. CANable이 PC에 정상 연결됐는지")
        print("  2. SLCAN이면 COM 포트가 맞는지")
        print("  3. candleLight이면 --interface gs_usb --channel 0인지")
        print("  4. CAN bitrate가 STM32와 동일한지")
        sys.exit(1)

    print("[CAN] bus opened")

    tx_thread = threading.Thread(
        target=tx_loop,
        daemon=True,
    )

    rx_thread = threading.Thread(
        target=rx_loop,
        daemon=True,
    )

    tx_thread.start()
    rx_thread.start()

    print_help()

    try:
        while True:
            cmd = input("can> ").strip()

            if not cmd:
                continue

            lower = cmd.lower()

            if lower == "q":
                break

            if lower in ("help", "h", "?"):
                print_help()
                continue

            if lower == "idle":
                stop_periodic_tx()
                continue

            if lower in ("e", "estop"):
                command_estop()
                continue

            if lower == "status":
                show_latest_status()
                continue

            if lower == "rx on":
                set_auto_print(True)
                continue

            if lower == "rx off":
                set_auto_print(False)
                continue

            if lower.startswith("once "):
                try:
                    deg = float(cmd.split(maxsplit=1)[1])
                    deg = clamp(
                        deg,
                        STEER_MIN_DEG,
                        STEER_MAX_DEG,
                    )
                    raw = send_steer_once(deg)

                    print(
                        f"[TX ONCE] target={deg:+.2f} deg "
                        f"raw={raw}"
                    )

                except ValueError:
                    print("invalid degree")
                except can.CanError as e:
                    print(f"[CAN TX ERROR] {e}")

                continue

            if lower.startswith("s "):
                try:
                    deg = float(cmd.split(maxsplit=1)[1])
                    set_periodic_target(deg)

                except ValueError:
                    print("invalid degree")

                continue

            # 숫자만 입력해도 periodic target으로 처리
            try:
                deg = float(cmd)
                set_periodic_target(deg)
                continue

            except ValueError:
                pass

            print("invalid command. 'help'를 입력하십시오.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")

    finally:
        running = False

        with tx_state_lock:
            # 종료 시 더 이상 normal steering request가 나가지 않게 함
            globals()["tx_enabled"] = False

        time.sleep(0.15)

        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass

        print("closed")


if __name__ == "__main__":
    main()
