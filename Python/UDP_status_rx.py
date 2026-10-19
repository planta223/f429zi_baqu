"""STM32 steering feedback receiver (Python standard library only).

Run from the repository root:
    python Python/UDP_status_rx.py
    python Python/UDP_status_rx.py --verbose

Firmware: 10.177.21.4:5000 -> PC 10.177.21.1:5002, every 20 ms.
Payload: little-endian int16 angle (0.01 deg/LSB), uint32 sample time (ms).
Configure the PC Ethernet adapter as 10.177.21.1/24 before running.
This program only receives feedback; it sends no steering commands and does
not change network settings. UDP_test_v2.py can run alongside it.
"""

import argparse
import math
import socket
import struct
import time


# Match Core/Inc/config.h.
PC_IP = "10.177.21.1"
STATUS_PORT = 5002
STM32_IP = "10.177.21.4"
STM32_PORT = 5000
STEER_SCALE = 100.0
STATUS_PERIOD_MS = 20
STATUS_PACKET = struct.Struct("<hI")  # 6 bytes, no native alignment padding


def positive_seconds(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than 0")
    return number


def udp_port(value):
    number = int(value)
    if not 1 <= number <= 65535:
        raise argparse.ArgumentTypeError("must be between 1 and 65535")
    return number


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bind-ip", default=PC_IP, help="PC local IPv4 address")
    parser.add_argument("--port", type=udp_port, default=STATUS_PORT,
                        help="local feedback port (default: 5002)")
    parser.add_argument("--stm32-ip", default=STM32_IP,
                        help="accept packets only from this IPv4 address")
    parser.add_argument("--source-port", type=udp_port, default=STM32_PORT,
                        help="accept packets only from this UDP source port")
    parser.add_argument("--timeout", type=positive_seconds, default=0.3,
                        help="feedback silence timeout in seconds (default: 0.3)")
    parser.add_argument("--verbose", action="store_true",
                        help="print every valid packet, including payload hex")
    parser.add_argument("--duration", type=positive_seconds,
                        help="stop after this many seconds (default: Ctrl+C)")
    return parser.parse_args()


def receive(args):
    expected_sender = (args.stm32_ip, args.source_port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((args.bind_ip, args.port))
        except OSError as exc:
            print(f"[BIND FAILED] {args.bind_ip}:{args.port}: {exc}", flush=True)
            print("Check the PC Ethernet IPv4 address (default: 10.177.21.1/24) "
                  "and whether another receiver is using this port.", flush=True)
            return 1

        sock.settimeout(min(0.1, args.timeout))
        print(f"[LISTEN] {args.bind_ip}:{args.port} <- "
              f"{args.stm32_ip}:{args.source_port}; "
              f"payload={STATUS_PACKET.size} bytes; "
              f"expected period={STATUS_PERIOD_MS} ms", flush=True)
        print("Receive only. Ctrl+C to stop. sample_ms is MCU uptime, "
              "not PC time or network latency.", flush=True)

        started = time.monotonic()
        last_report = started
        last_rx = None
        last_sample = None
        latest_angle = None
        latest_sample = None
        pc_interval_ms = None
        sample_delta_ms = None
        received = 0
        reported_count = 0
        bad_length = 0
        other_sender = 0
        duplicate_time = 0
        backwards_time = 0
        timed_out = False

        try:
            while True:
                now = time.monotonic()
                if args.duration is not None and now - started >= args.duration:
                    break

                try:
                    data, sender = sock.recvfrom(65535)
                except socket.timeout:
                    pass
                else:
                    now = time.monotonic()
                    if sender != expected_sender:
                        other_sender += 1
                    elif len(data) != STATUS_PACKET.size:
                        bad_length += 1
                        if bad_length == 1 or args.verbose:
                            print(f"[BAD LENGTH] {len(data)} bytes "
                                  f"(expected {STATUS_PACKET.size})", flush=True)
                    else:
                        actual_raw, sample_ms = STATUS_PACKET.unpack(data)
                        latest_angle = actual_raw / STEER_SCALE
                        latest_sample = sample_ms
                        pc_interval_ms = (None if last_rx is None else
                                          (now - last_rx) * 1000.0)
                        sample_delta_ms = None

                        if last_sample is not None:
                            delta = (sample_ms - last_sample) & 0xFFFFFFFF
                            if delta == 0:
                                duplicate_time += 1
                            elif delta < 0x80000000:
                                # Includes normal uint32 timestamp wraparound.
                                sample_delta_ms = delta
                            else:
                                backwards_time += 1
                                print("[TIME BACKWARDS] MCU restart or "
                                      "out-of-order packet; time baseline reset.",
                                      flush=True)

                        # Keep the received values visible even after a restart.
                        last_sample = sample_ms
                        last_rx = now
                        received += 1

                        if timed_out:
                            print("[RECOVERED] Feedback received again.", flush=True)
                            timed_out = False

                        if args.verbose:
                            pc_dt = ("--" if pc_interval_ms is None else
                                     f"{pc_interval_ms:.2f}")
                            sample_dt = ("--" if sample_delta_ms is None else
                                         str(sample_delta_ms))
                            print(f"[RX {received}] angle={latest_angle:+.2f} deg "
                                  f"sample_ms={sample_ms} "
                                  f"pc_dt_ms={pc_dt} sample_dt_ms={sample_dt} "
                                  f"hex={data.hex(' ')}", flush=True)

                now = time.monotonic()
                silence = now - (started if last_rx is None else last_rx)
                if silence >= args.timeout and not timed_out:
                    print(f"[TIMEOUT] No valid feedback for {silence:.2f} s. "
                          "Check board firmware/link, PC IP and UDP firewall.",
                          flush=True)
                    timed_out = True

                if now - last_report >= 1.0:
                    hz = (received - reported_count) / (now - last_report)
                    angle = ("--" if latest_angle is None else
                             f"{latest_angle:+.2f}")
                    sample = "--" if latest_sample is None else str(latest_sample)
                    state = "TIMEOUT" if timed_out else ("WAIT" if last_rx is None else "OK")
                    print(f"[{state}] angle={angle} deg sample_ms={sample} "
                          f"rx={received} rate={hz:.1f} Hz age={silence * 1000:.0f} ms "
                          f"bad_length={bad_length} other_sender={other_sender} "
                          f"duplicate_time={duplicate_time} backwards_time={backwards_time}",
                          flush=True)
                    last_report = now
                    reported_count = received
        except KeyboardInterrupt:
            print("\n[STOP]", flush=True)

        print(f"[TOTAL] rx={received} bad_length={bad_length} "
              f"other_sender={other_sender} duplicate_time={duplicate_time} "
              f"backwards_time={backwards_time}", flush=True)
        return 0


def main():
    args = parse_args()
    try:
        socket.inet_pton(socket.AF_INET, args.bind_ip)
        socket.inet_pton(socket.AF_INET, args.stm32_ip)
    except OSError:
        print("[ERROR] --bind-ip and --stm32-ip must be numeric IPv4 addresses.")
        return 1

    try:
        return receive(args)
    except OSError as exc:
        print(f"[SOCKET ERROR] {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
