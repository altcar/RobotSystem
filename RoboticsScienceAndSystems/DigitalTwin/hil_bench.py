#!/usr/bin/env python3
"""Bench tool for the SEMT0043 HIL firmware.

Connects to the StampC3 access point (192.168.4.1:80), enables HIL, sends
bounded PWM commands, and records the 14-field telemetry. Intended for
characterising the real robot before the sim-vs-physical comparison.

Modes
-----
sensor : hold the motors at zero and log the five reflectance readings.
drive  : apply a sequence of forward PWM values and report encoder rate.
raw    : print parsed telemetry without commanding motion.

Examples
--------
    python hil_bench.py --mode sensor --seconds 10
    python hil_bench.py --mode drive --pwms 20,40,60,80,100 --hold 2
    python hil_bench.py --mode raw --seconds 5

Safety: drive mode moves the robot. Clear the area, place it on the floor
with the wheels free, and keep the WiFi link in range. The firmware also stops
the motors if no PWM command arrives for 500 ms. The script always sends
PWM,0,0 and HIL,0 on exit.
"""

import argparse
import csv
import select
import socket
import sys
import time

TELEMETRY_FIELDS = 14
MM_PER_ENCODER_COUNT = 0.2805  # 16 mm wheel, 358.3 counts/rev (template value)


class Link:
    def __init__(self, host, port, timeout=3.0):
        self.socket = socket.create_connection((host, port), timeout)
        self.socket.setblocking(False)
        self.buffer = b""

    def send(self, line):
        self.socket.sendall((line + "\n").encode("ascii"))

    def read_lines(self):
        lines = []
        while True:
            try:
                data = self.socket.recv(4096)
            except BlockingIOError:
                break
            if not data:
                break
            self.buffer += data
        while b"\n" in self.buffer:
            raw, self.buffer = self.buffer.split(b"\n", 1)
            text = raw.decode("ascii", errors="replace").strip()
            if text:
                lines.append(text)
        return lines

    def close(self):
        try:
            self.socket.close()
        except OSError:
            pass


def parse_telemetry(line):
    fields = line.split(",")
    if len(fields) != TELEMETRY_FIELDS:
        return None
    try:
        values = [float(f) for f in fields]
    except ValueError:
        return None
    return {
        "timestamp": values[0],
        "x": values[1], "y": values[2], "theta": values[3],
        "pwm_left": values[4], "pwm_right": values[5],
        "encoder_left": values[6], "encoder_right": values[7],
        "sensors": values[8:13],
        "signal": int(values[13]),
    }


def command_loop(link, pwm_left, pwm_right, seconds, rows, csv_writer):
    end = time.monotonic() + seconds
    next_send = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_send:
            link.send(f"PWM,{pwm_left:.0f},{pwm_right:.0f}")
            next_send = now + 0.05
        for line in link.read_lines():
            sample = parse_telemetry(line)
            if sample is None:
                continue
            sample["wall"] = time.time()
            rows.append(sample)
            if csv_writer:
                csv_writer.writerow([
                    f"{sample['wall']:.3f}", f"{sample['timestamp']:.0f}",
                    f"{sample['x']:.2f}", f"{sample['y']:.2f}",
                    f"{sample['theta']:.5f}",
                    f"{sample['encoder_left']:.2f}", f"{sample['encoder_right']:.2f}",
                    *[f"{value:.0f}" for value in sample["sensors"]]
                ])
        time.sleep(0.005)
    return rows


def main():
    parser = argparse.ArgumentParser(description="HIL bench tool for the StampC3")
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=80)
    parser.add_argument("--mode", choices=("sensor", "drive", "raw"), default="sensor")
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="duration of a sensor/raw capture")
    parser.add_argument("--hold", type=float, default=2.0,
                        help="seconds per PWM step in drive mode")
    parser.add_argument("--pwms", default="20,40,60,80,100",
                        help="comma-separated forward PWM values")
    parser.add_argument("--out", default="bench_log.csv")
    parser.add_argument("--yes", action="store_true",
                        help="skip the safety confirmation")
    args = parser.parse_args()

    if not args.yes:
        print("This tool may move the robot. Place it on the floor with the")
        print("wheels free and keep the area clear.")
        reply = input("Continue? [y/N] ").strip().lower()
        if reply != "y":
            print("Cancelled.")
            return

    link = Link(args.host, args.port)
    try:
        link.send("HIL,1")
        time.sleep(0.2)
        link.read_lines()  # discard any buffered line before the first command
        with open(args.out, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "wall_time", "robot_ms", "x_mm", "y_mm", "theta_rad",
                "encoder_left", "encoder_right", "DN1", "DN2", "DN3",
                "DN4", "DN5"
            ])

            if args.mode == "sensor":
                print(f"Logging stationary sensor readings for {args.seconds:.0f} s...")
                rows = command_loop(link, 0, 0, args.seconds, [], writer)
                print(f"{len(rows)} samples written to {args.out}")
                if rows:
                    means = [sum(r["sensors"][i] for r in rows) / len(rows)
                             for i in range(5)]
                    print("Mean DN1..DN5:", ", ".join(f"{m:.0f}" for m in means))

            elif args.mode == "raw":
                rows = command_loop(link, 0, 0, args.seconds, [], writer)
                for row in rows[-10:]:
                    print(f"t={row['timestamp']:.0f} x={row['x']:.1f} "
                          f"y={row['y']:.1f} theta={row['theta']:.3f} "
                          f"sensors={row['sensors']}")
                print(f"{len(rows)} samples written to {args.out}")

            else:
                pwms = [float(value) for value in args.pwms.split(",") if value.strip()]
                print("PWM, encoder counts/s, mm/s (nominal)")
                for pwm in pwms:
                    start = time.monotonic()
                    rows = command_loop(link, pwm, pwm, args.hold, [], writer)
                    elapsed = max(1e-6, time.monotonic() - start)
                    if len(rows) >= 2:
                        first = rows[0]
                        last = rows[-1]
                        counts = ((last["encoder_left"] + last["encoder_right"])
                                  - (first["encoder_left"] + first["encoder_right"])) / 2.0
                        rate = counts / elapsed
                        print(f"{pwm:6.1f}, {rate:9.1f}, {rate * MM_PER_ENCODER_COUNT:8.1f}")
                    # stop and settle between steps
                    command_loop(link, 0, 0, 1.0, [], writer)
                print(f"Detail written to {args.out}")
    finally:
        try:
            link.send("PWM,0,0")
            time.sleep(0.05)
            link.send("HIL,0")
        except OSError:
            pass
        link.close()
        print("Stopped; HIL disabled.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)
