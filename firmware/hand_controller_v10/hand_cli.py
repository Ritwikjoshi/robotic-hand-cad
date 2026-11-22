#!/usr/bin/env python3
"""
Robotic Hand v10.0 — Python 3 Serial CLI & Dry-Run Simulator.

Communicates with `hand_controller_v10.ino` over USB-Serial (115200 baud) or
runs offline in `--dry-run` simulation mode to validate commands, preview
differential wrist kinematics mixing, and inspect simulated PWM pulse widths.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

try:
    import serial  # type: ignore
except ImportError:
    serial = None


FW_VERSION = "10.0.0"
DEFAULT_BAUD = 115200

GLOBAL_MIN_PULSE_US = 500
GLOBAL_MAX_PULSE_US = 2500
TRIM_MIN_US = -250
TRIM_MAX_US = 250

WRIST_PITCH_MIN_DEG = -45.0
WRIST_PITCH_MAX_DEG = 45.0
WRIST_YAW_MIN_DEG = -35.0
WRIST_YAW_MAX_DEG = 35.0
WRIST_SERVO_MIN_DEG = -60.0
WRIST_SERVO_MAX_DEG = 60.0
WRIST_PITCH_GAIN = 1.0
WRIST_YAW_GAIN = 1.0


@dataclass(frozen=True)
class ServoSpec:
    channel: int
    name: str
    model: str
    min_us: int
    max_us: int
    neutral_us: int
    min_deg: float
    max_deg: float
    rest_deg: float
    invert: bool
    max_dps: float


# Exact mirror of SERVO_CONFIGS in config.h (CH00 - CH10)
SERVO_SPECS: Tuple[ServoSpec, ...] = (
    ServoSpec(0,  "index",       "MG90S / KST DS215MG",  600, 2300,  868,   0.0, 95.0, 15.0, False, 180.0),
    ServoSpec(1,  "middle",      "MG90S / KST DS215MG",  600, 2300,  868,   0.0, 95.0, 15.0, False, 180.0),
    ServoSpec(2,  "ring",        "MG90S / KST DS215MG",  600, 2300,  868,   0.0, 95.0, 15.0, False, 180.0),
    ServoSpec(3,  "pinky",       "MG90S / KST DS215MG",  600, 2300,  868,   0.0, 95.0, 15.0, False, 180.0),
    ServoSpec(4,  "thumb_flex",  "MG90S / KST DS215MG",  650, 2250,  917,   0.0, 90.0, 15.0, False, 160.0),
    ServoSpec(5,  "thumb_opp",   "MG90S / KST DS215MG",  700, 2200, 1117,   0.0, 90.0, 25.0, False, 150.0),
    ServoSpec(6,  "cup",         "MG90S",                900, 1900, 1067,   0.0, 30.0,  5.0, False,  90.0),
    ServoSpec(7,  "spread",      "MG90S / SG90",        1000, 2000, 1500, -15.0, 15.0,  0.0, False,  90.0),
    ServoSpec(8,  "wrist_left",  "DS3218 / MG996R",      800, 2200, 1500, -60.0, 60.0,  0.0, False, 120.0),
    ServoSpec(9,  "wrist_right", "DS3218 / MG996R",      800, 2200, 1500, -60.0, 60.0,  0.0, True,  120.0),
    ServoSpec(10, "pronation",   "DS3218 / MG996R",      550, 2450, 1500, -85.0, 85.0,  0.0, False, 110.0),
)

NAME_TO_CH: Dict[str, int] = {spec.name: spec.channel for spec in SERVO_SPECS}

# Exact mirror of GRASP_PRESETS in config.h
# Order: idx, mid, rng, pnk, th_flex, th_opp, cup, spread, wpitch, wyaw, pron
GRASP_PRESETS: Dict[str, Tuple[float, ...]] = {
    "REST":     (15.0, 15.0, 15.0, 15.0, 15.0, 25.0,  5.0,  0.0,  0.0,  0.0, 0.0),
    "OPEN":     ( 0.0,  0.0,  0.0,  0.0,  0.0, 10.0,  0.0,  5.0,  0.0,  0.0, 0.0),
    "FIST":     (90.0, 92.0, 92.0, 90.0, 80.0, 75.0, 25.0, -5.0,  5.0,  0.0, 0.0),
    "PINCH":    (68.0, 12.0, 10.0, 10.0, 62.0, 82.0,  8.0,  0.0,  5.0,  0.0, 0.0),
    "CYLINDER": (60.0, 62.0, 62.0, 58.0, 55.0, 65.0, 20.0,  0.0,  5.0,  0.0, 0.0),
    "TRIPOD":   (65.0, 65.0, 15.0, 12.0, 60.0, 80.0, 10.0,  0.0,  5.0,  0.0, 0.0),
    "SPREAD":   ( 0.0,  0.0,  0.0,  0.0,  0.0, 15.0,  0.0, 15.0,  0.0,  0.0, 0.0),
    "WAVE":     ( 5.0,  5.0,  5.0,  5.0, 10.0, 20.0,  0.0, 10.0, 15.0, 25.0, 0.0),
}


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def mix_wrist(pitch_deg: float, yaw_deg: float) -> Tuple[float, float, bool]:
    """
    Differential wrist mixer matching `mixWrist()` in hand_controller_v10.ino.
    Returns (left_deg, right_deg, saturated_clamped).
    """
    p = clamp(pitch_deg, WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG)
    y = clamp(yaw_deg, WRIST_YAW_MIN_DEG, WRIST_YAW_MAX_DEG)

    raw_left = WRIST_PITCH_GAIN * p + WRIST_YAW_GAIN * y
    raw_right = WRIST_PITCH_GAIN * p - WRIST_YAW_GAIN * y

    peak = max(abs(raw_left), abs(raw_right))
    saturated = peak > WRIST_SERVO_MAX_DEG
    if saturated and peak > 1e-4:
        scale = WRIST_SERVO_MAX_DEG / peak
        raw_left *= scale
        raw_right *= scale

    left_deg = clamp(raw_left, SERVO_SPECS[8].min_deg, SERVO_SPECS[8].max_deg)
    right_deg = clamp(raw_right, SERVO_SPECS[9].min_deg, SERVO_SPECS[9].max_deg)
    return left_deg, right_deg, saturated


def unmix_wrist(left_deg: float, right_deg: float) -> Tuple[float, float]:
    pitch = 0.5 * (left_deg + right_deg) / WRIST_PITCH_GAIN
    yaw = 0.5 * (left_deg - right_deg) / WRIST_YAW_GAIN
    return pitch, yaw


def angle_to_pulse_us(ch: int, deg: float, trim_us: int = 0) -> int:
    spec = SERVO_SPECS[ch]
    clamped = clamp(deg, spec.min_deg, spec.max_deg)

    pulse = float(spec.neutral_us)
    if clamped >= spec.rest_deg and spec.max_deg > spec.rest_deg:
        t = (clamped - spec.rest_deg) / (spec.max_deg - spec.rest_deg)
        pulse = spec.neutral_us + t * (spec.max_us - spec.neutral_us)
    elif clamped < spec.rest_deg and spec.rest_deg > spec.min_deg:
        t = (clamped - spec.min_deg) / (spec.rest_deg - spec.min_deg)
        pulse = spec.min_us + t * (spec.neutral_us - spec.min_us)

    if spec.invert:
        pulse = (spec.min_us + spec.max_us) - pulse

    out = int(round(pulse)) + trim_us
    return int(clamp(out, GLOBAL_MIN_PULSE_US, GLOBAL_MAX_PULSE_US))


def resolve_channel(token: str) -> Optional[int]:
    if token.isdigit():
        idx = int(token)
        return idx if 0 <= idx < len(SERVO_SPECS) else None
    return NAME_TO_CH.get(token.lower())


class HandControllerClient:
    """Serial + dry-run client for the v10.0 ESP32-S3 robotic hand controller."""

    def __init__(self, port: Optional[str], baud: int = DEFAULT_BAUD,
                 timeout: float = 1.0, dry_run: bool = False) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.dry_run = dry_run
        self._ser = None

        # Local mirror / simulation state
        self.enabled: bool = True
        self.angles: List[float] = [s.rest_deg for s in SERVO_SPECS]
        self.trims: List[int] = [0 for _ in SERVO_SPECS]
        self.wrist_pitch: float = 0.0
        self.wrist_yaw: float = 0.0

        if not self.dry_run:
            if not self.port:
                raise ValueError("Provide --port <device> or use --dry-run.")
            if serial is None:
                raise RuntimeError(
                    "pyserial is not installed. Install with `pip install pyserial` "
                    "or run with `--dry-run`."
                )
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.25)

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def _sim_status_json(self) -> str:
        cur_pitch, cur_yaw = unmix_wrist(self.angles[8], self.angles[9])
        channels = []
        for spec in SERVO_SPECS:
            ch = spec.channel
            us = angle_to_pulse_us(ch, self.angles[ch], self.trims[ch]) if self.enabled else 0
            channels.append({
                "ch": ch,
                "name": spec.name,
                "cur": round(self.angles[ch], 1),
                "tgt": round(self.angles[ch], 1),
                "us": us,
                "trim": self.trims[ch],
            })
        payload = {
            "fw": FW_VERSION,
            "enabled": self.enabled,
            "wrist_pitch": round(cur_pitch, 1),
            "wrist_yaw": round(cur_yaw, 1),
            "channels": channels,
        }
        return json.dumps(payload, separators=(",", ":"))

    def validate_and_send(self, raw_line: str) -> str:
        line = raw_line.strip()
        if not line:
            return ""
        parts = line.split()
        cmd = parts[0].upper()

        if cmd == "HELP":
            frame = "HELP"
            return self._dispatch(frame, sim_reply=(
                "Commands: PRESET <name>, SET <ch|wrist_pitch|wrist_yaw> <deg>, "
                "POSE <11 angles>, TRIM <ch> <us>, STATUS, RELAX, STOP, DEMO, HELP"
            ))

        if cmd == "STATUS":
            return self._dispatch("STATUS", sim_reply=self._sim_status_json())

        if cmd in ("RELAX", "STOP"):
            self.enabled = False
            return self._dispatch("RELAX", sim_reply="OK RELAXED")

        if cmd == "PRESET":
            if len(parts) != 2:
                raise ValueError("Usage: PRESET <REST|OPEN|FIST|PINCH|CYLINDER|TRIPOD|SPREAD|WAVE>")
            name = parts[1].upper()
            if name not in GRASP_PRESETS:
                raise ValueError(f"Unknown preset '{name}'. Valid: {', '.join(GRASP_PRESETS)}")
            dof = GRASP_PRESETS[name]
            self._update_local_pose(dof)
            left, right, sat = mix_wrist(dof[8], dof[9])
            note = f" [wrist mix: CH08={left:+.1f} deg, CH09={right:+.1f} deg"
            if sat:
                note += ", anti-saturation scaled"
            note += "]"
            return self._dispatch(f"PRESET {name}", sim_reply=f"OK PRESET {name}{note}")

        if cmd == "SET":
            if len(parts) != 3:
                raise ValueError("Usage: SET <ch_name|0..10|wrist_pitch|wrist_yaw> <deg>")
            target_tok = parts[1].lower()
            deg = float(parts[2])

            if target_tok == "wrist_pitch":
                self.enabled = True
                self.wrist_pitch = clamp(deg, WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG)
                left, right, sat = mix_wrist(self.wrist_pitch, self.wrist_yaw)
                self.angles[8], self.angles[9] = left, right
                sat_str = " (anti-saturation clamped)" if sat else ""
                reply = (
                    f"OK SET wrist_pitch {self.wrist_pitch:.1f} -> "
                    f"CH08={left:+.1f} deg ({angle_to_pulse_us(8, left, self.trims[8])} us), "
                    f"CH09={right:+.1f} deg ({angle_to_pulse_us(9, right, self.trims[9])} us){sat_str}"
                )
                return self._dispatch(f"SET wrist_pitch {self.wrist_pitch:.2f}", sim_reply=reply)

            if target_tok == "wrist_yaw":
                self.enabled = True
                self.wrist_yaw = clamp(deg, WRIST_YAW_MIN_DEG, WRIST_YAW_MAX_DEG)
                left, right, sat = mix_wrist(self.wrist_pitch, self.wrist_yaw)
                self.angles[8], self.angles[9] = left, right
                sat_str = " (anti-saturation clamped)" if sat else ""
                reply = (
                    f"OK SET wrist_yaw {self.wrist_yaw:.1f} -> "
                    f"CH08={left:+.1f} deg ({angle_to_pulse_us(8, left, self.trims[8])} us), "
                    f"CH09={right:+.1f} deg ({angle_to_pulse_us(9, right, self.trims[9])} us){sat_str}"
                )
                return self._dispatch(f"SET wrist_yaw {self.wrist_yaw:.2f}", sim_reply=reply)

            ch = resolve_channel(target_tok)
            if ch is None:
                raise ValueError(f"Unknown channel '{parts[1]}'.")
            spec = SERVO_SPECS[ch]
            clamped = clamp(deg, spec.min_deg, spec.max_deg)
            self.enabled = True
            self.angles[ch] = clamped
            if ch in (8, 9):
                self.wrist_pitch, self.wrist_yaw = unmix_wrist(self.angles[8], self.angles[9])
            us = angle_to_pulse_us(ch, clamped, self.trims[ch])
            return self._dispatch(
                f"SET {spec.name} {clamped:.2f}",
                sim_reply=f"OK SET CH{ch:02d} ({spec.name}) {clamped:.1f} deg -> {us} us",
            )

        if cmd == "POSE":
            if len(parts) != 12:
                raise ValueError(
                    "Usage: POSE <idx> <mid> <rng> <pnk> <th_flex> <th_opp> "
                    "<cup> <spread> <wpitch> <wyaw> <pron>"
                )
            vals = tuple(float(x) for x in parts[1:])
            self._update_local_pose(vals)
            left, right, sat = mix_wrist(vals[8], vals[9])
            frame = "POSE " + " ".join(f"{v:.2f}" for v in vals)
            sat_str = " (anti-saturation clamped)" if sat else ""
            return self._dispatch(
                frame,
                sim_reply=f"OK POSE [CH08={left:+.1f} deg, CH09={right:+.1f} deg{sat_str}]",
            )

        if cmd == "TRIM":
            if len(parts) != 3:
                raise ValueError("Usage: TRIM <ch_name|0..10> <us_offset>")
            ch = resolve_channel(parts[1])
            if ch is None:
                raise ValueError(f"Unknown channel '{parts[1]}'.")
            offset = int(clamp(int(parts[2]), TRIM_MIN_US, TRIM_MAX_US))
            self.trims[ch] = offset
            spec = SERVO_SPECS[ch]
            us = angle_to_pulse_us(ch, self.angles[ch], offset) if self.enabled else 0
            return self._dispatch(
                f"TRIM {spec.name} {offset}",
                sim_reply=f"OK TRIM CH{ch:02d} ({spec.name}) {offset:+d} us -> {us} us",
            )

        raise ValueError(f"Unknown command '{cmd}'. Type HELP for usage.")

    def _update_local_pose(self, dof: Tuple[float, ...]) -> None:
        self.enabled = True
        for ch in range(8):
            spec = SERVO_SPECS[ch]
            self.angles[ch] = clamp(dof[ch], spec.min_deg, spec.max_deg)
        self.wrist_pitch = clamp(dof[8], WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG)
        self.wrist_yaw = clamp(dof[9], WRIST_YAW_MIN_DEG, WRIST_YAW_MAX_DEG)
        left, right, _ = mix_wrist(self.wrist_pitch, self.wrist_yaw)
        self.angles[8] = left
        self.angles[9] = right
        self.angles[10] = clamp(dof[10], SERVO_SPECS[10].min_deg, SERVO_SPECS[10].max_deg)

    def _dispatch(self, frame: str, sim_reply: str) -> str:
        if self.dry_run:
            print(f"[DRY-RUN TX] {frame}")
            print(f"[DRY-RUN RX] {sim_reply}")
            return sim_reply

        assert self._ser is not None
        self._ser.write((frame + "\n").encode("ascii"))
        self._ser.flush()
        reply = self._ser.readline().decode("utf-8", errors="replace").strip()
        print(f"[TX] {frame}")
        print(f"[RX] {reply}")
        return reply


def run_demo(client: HandControllerClient, dwell_s: float = 0.0) -> None:
    """Execute a complete 11-DOF scripted demo including wrist differential anti-saturation."""
    sequence = [
        "PRESET REST",
        "PRESET OPEN",
        "PRESET SPREAD",
        "PRESET PINCH",
        "PRESET TRIPOD",
        "PRESET CYLINDER",
        "PRESET FIST",
        "PRESET WAVE",
        "SET wrist_pitch 45",
        "SET wrist_yaw 35",   # Combined +45/+35 triggers anti-saturation scaling to +/-60 deg
        "SET pronation -85",
        "SET pronation 85",
        "SET cup 30",
        "SET spread -15",
        "STATUS",
        "PRESET REST",
        "RELAX",
    ]
    for cmd in sequence:
        client.validate_and_send(cmd)
        if dwell_s > 0.0 and not client.dry_run:
            time.sleep(dwell_s)


def run_repl(client: HandControllerClient) -> None:
    mode = "DRY-RUN" if client.dry_run else f"SERIAL {client.port}@{client.baud}"
    print(f"Robotic Hand v10.0 CLI ({mode}) — type HELP or QUIT.")
    while True:
        try:
            line = input("hand-v10> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        if line.lower() in ("quit", "exit"):
            break
        if line.upper() == "DEMO":
            run_demo(client, dwell_s=1.0)
            continue
        try:
            client.validate_and_send(line)
        except ValueError as exc:
            print(f"[ERR] {exc}")


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Robotic Hand v10.0 — ESP32-S3 + PCA9685 Serial CLI & Simulator"
    )
    p.add_argument("--port", help="Serial port (e.g. /dev/cu.usbserial-0001 or COM3)")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate (default: 115200)")
    p.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds")
    p.add_argument("--dry-run", action="store_true", help="Simulate without hardware attached")
    p.add_argument("--preset", help="Send a single grasp preset (REST, OPEN, FIST, PINCH, etc.)")
    p.add_argument("--set", nargs=2, metavar=("CHANNEL", "DEG"), help="Set a single channel or wrist_pitch/wrist_yaw")
    p.add_argument("--pose", nargs=11, type=float, metavar="DEG", help="Send all 11 logical DOF angles")
    p.add_argument("--status", action="store_true", help="Request JSON STATUS telemetry")
    p.add_argument("--relax", action="store_true", help="Disable all PWM outputs immediately")
    p.add_argument("--demo", action="store_true", help="Run scripted 11-DOF demo sequence")
    p.add_argument("--interactive", action="store_true", help="Force interactive REPL mode")
    return p


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    if not args.dry_run and not args.port:
        parser.error("Specify --port <device> or pass --dry-run for simulation mode.")

    client = HandControllerClient(
        port=args.port,
        baud=args.baud,
        timeout=args.timeout,
        dry_run=args.dry_run,
    )
    try:
        one_shot = False
        if args.preset:
            client.validate_and_send(f"PRESET {args.preset}")
            one_shot = True
        if args.set:
            client.validate_and_send(f"SET {args.set[0]} {args.set[1]}")
            one_shot = True
        if args.pose:
            pose_str = " ".join(f"{v:.2f}" for v in args.pose)
            client.validate_and_send(f"POSE {pose_str}")
            one_shot = True
        if args.status:
            client.validate_and_send("STATUS")
            one_shot = True
        if args.relax:
            client.validate_and_send("RELAX")
            one_shot = True
        if args.demo:
            run_demo(client, dwell_s=1.0)
            one_shot = True

        if args.interactive:
            run_repl(client)
        elif not one_shot:
            if not sys.stdin.isatty():
                piped_lines = [ln.strip() for ln in sys.stdin if ln.strip()]
                if piped_lines:
                    for ln in piped_lines:
                        client.validate_and_send(ln)
                elif args.dry_run:
                    # Non-interactive bare `--dry-run`: run full validation demo
                    run_demo(client, dwell_s=0.0)
            else:
                run_repl(client)
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
