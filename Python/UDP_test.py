import socket
import struct
import threading
import time

STM32_IP = "10.177.21.4"
STM32_PORT = 5000

STEER_MODE_AUTO = 1
STEER_MODE_MANUAL = 2
STEER_MODE_ESTOP = 3

TX_PERIOD_S = 0.02  # 50 Hz

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

active_mode = "idle"   # "pc", "asms", "idle"
target_deg = 0
adc_raw = 2048
running = True


def send_asms(mode: int, adc: int = 2048):
    adc = max(0, min(4095, int(adc)))
    packet = struct.pack("<BBBH", mode, 0, 0, adc)
    sock.sendto(packet, (STM32_IP, STM32_PORT))


def send_pc(deg: int, speed_raw: int = 0, estop: bool = False):
    deg = max(-60, min(60, int(deg)))
    misc = 0x80 if estop else 0x00
    packet = struct.pack("<iIB", deg, int(speed_raw), misc)
    sock.sendto(packet, (STM32_IP, STM32_PORT))


def tx_loop():
    global running, active_mode, target_deg, adc_raw

    while running:
        if active_mode == "pc":
            # PC packet은 AUTO 모드에서만 인정되므로 AUTO packet도 주기적으로 보냅니다.
            send_asms(STEER_MODE_AUTO, 2048)
            send_pc(target_deg)

        elif active_mode == "asms":
            send_asms(STEER_MODE_MANUAL, adc_raw)

        time.sleep(TX_PERIOD_S)


def main():
    global running, active_mode, target_deg, adc_raw

    thread = threading.Thread(target=tx_loop, daemon=True)
    thread.start()

    print("STM32 UDP steering debugger")
    print(f"target = {STM32_IP}:{STM32_PORT}")
    print("commands:")
    print("  pc              : PC angle mode")
    print("  asms            : ASMS adc mode")
    print("  e               : ESTOP")
    print("  q               : quit")

    try:
        while True:
            cmd = input("\nmode(pc/asms/e/q)> ").strip().lower()

            if cmd == "q":
                running = False
                break

            if cmd == "e":
                send_asms(STEER_MODE_ESTOP, 2048)
                active_mode = "idle"
                print("ESTOP sent.")
                continue

            if cmd == "pc":
                active_mode = "pc"
                send_asms(STEER_MODE_AUTO, 2048)
                print("[PC mode] angle deg 입력. q 입력 시 모드 선택 복귀.")

                while True:
                    s = input("pc target deg> ").strip().lower()

                    if s == "q":
                        active_mode = "idle"
                        break

                    if s == "e":
                        send_pc(target_deg, estop=True)
                        active_mode = "idle"
                        print("PC ESTOP sent.")
                        break

                    try:
                        value = int(s)
                    except ValueError:
                        print("invalid input")
                        continue

                    target_deg = max(-60, min(60, value))
                    print(f"PC target updated: {target_deg} deg")

                continue

            if cmd == "asms":
                active_mode = "asms"
                print("[ASMS mode] adc raw 입력. 범위 0~4095, 중립 2048. q 입력 시 모드 선택 복귀.")

                while True:
                    s = input("asms adc raw> ").strip().lower()

                    if s == "q":
                        active_mode = "idle"
                        break

                    if s == "e":
                        send_asms(STEER_MODE_ESTOP, 2048)
                        active_mode = "idle"
                        print("ASMS ESTOP sent.")
                        break

                    try:
                        value = int(s)
                    except ValueError:
                        print("invalid input")
                        continue

                    adc_raw = max(0, min(4095, value))
                    print(f"ASMS adc updated: {adc_raw}")

                continue

            print("invalid command")

    finally:
        running = False
        time.sleep(0.05)
        sock.close()


if __name__ == "__main__":
    main()