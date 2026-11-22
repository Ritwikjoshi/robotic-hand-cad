# Robotic Hand v10.0 — ESP32-S3 + PCA9685 Firmware & Serial CLI

Production-ready **11-servo** controller firmware and Python 3 CLI for the v10.0 Robotic Hand, built on an **ESP32-S3** driving a **PCA9685 16-Channel 12-Bit I2C PWM** board at `50 Hz`.

## Files

- `config.h` — Hardware pinout, PCA9685 `CH00–CH10` channel mapping, per-channel pulse calibration & slew-rate limits, differential wrist limits, and built-in grasp presets.
- `hand_controller_v10.ino` — Arduino / PlatformIO firmware featuring a `50 Hz` cubic ease-out slew-rate trajectory engine, anti-saturation differential wrist mixer, safe `REST` startup, instant `RELAX`/`STOP` release, and `115200 baud` serial protocol.
- `hand_cli.py` — Interactive Python 3 CLI, scripted demo runner, and `--dry-run` offline simulator.

---

## Hardware & PCA9685 Channel Mapping

- **MCU**: ESP32-S3 (`SDA = GPIO 8`, `SCL = GPIO 9`, `400 kHz` I2C)
- **PWM Driver**: PCA9685 at I2C address `0x40`, `50 Hz` PWM
- **Power**: Keep ESP32-S3 logic (`3.3 V / 5 V USB`) separate from the servo `V+` power rail (`6.0 V – 7.4 V`, `≥ 10 A` buck/UBEC with a `2200 µF` low-ESR capacitor across `V+`/`GND` and common ground with the ESP32-S3).

| Channel | Name | Actuator Function | Servo Class | Software Range | Slew Limit |
| :--- | :--- | :--- | :--- | :---: | :---: |
| `CH00` | `index` | Index Flexor | `MG90S` / `KST DS215MG` | `0° .. +95°` | `180 °/s` |
| `CH01` | `middle` | Middle Flexor | `MG90S` / `KST DS215MG` | `0° .. +95°` | `180 °/s` |
| `CH02` | `ring` | Ring Flexor | `MG90S` / `KST DS215MG` | `0° .. +95°` | `180 °/s` |
| `CH03` | `pinky` | Pinky Flexor | `MG90S` / `KST DS215MG` | `0° .. +95°` | `180 °/s` |
| `CH04` | `thumb_flex` | Thumb Flexor | `MG90S` / `KST DS215MG` | `0° .. +90°` | `160 °/s` |
| `CH05` | `thumb_opp` | Thumb Opposition | `MG90S` / `KST DS215MG` | `0° .. +90°` | `150 °/s` |
| `CH06` | `cup` | Palmar Arch Cupping (4th/5th Ray) | `MG90S` | `0° .. +30°` | `90 °/s` |
| `CH07` | `spread` | Finger Abduction / Spread Linkage | `MG90S` / `SG90` | `-15° .. +15°` | `90 °/s` |
| `CH08` | `wrist_left` | Wrist Differential Left Pushrod | `DS3218` / `MG996R` (20 kg·cm) | `-60° .. +60°` | `120 °/s` |
| `CH09` | `wrist_right` | Wrist Differential Right Pushrod (`invert=true`) | `DS3218` / `MG996R` (20 kg·cm) | `-60° .. +60°` | `120 °/s` |
| `CH10` | `pronation` | Forearm Pronation/Supination (`22T→44T` 2:1) | `DS3218` (270°) / `MG996R` | `-85° .. +85°` | `110 °/s` |

> **Differential Wrist Mixer**: Logical `wrist_pitch ∈ [-45°, +45°]` and `wrist_yaw ∈ [-35°, +35°]` are mixed onto `CH08` (`pitch + yaw`) and `CH09` (`pitch - yaw`, inverted on output). When `max(|left|, |right|) > 60°`, both outputs are proportionally scaled so the pushrods never stall or bind the Cardan ring.

---

## Flashing Instructions

### Option A — Arduino IDE (2.x)

1. Install the **esp32** board package by Espressif Systems via **Boards Manager**.
2. Install **Adafruit PWM Servo Driver Library** (and its dependency `Adafruit BusIO`) via **Library Manager**.
3. Open `firmware/hand_controller_v10/hand_controller_v10.ino` in Arduino IDE (`config.h` loads automatically from the same folder).
4. Select **Board**: `ESP32S3 Dev Module`, **USB CDC On Boot**: `Enabled` (if using native USB), and the matching serial port.
5. Click **Upload** and open Serial Monitor at `115200 baud` (`Newline` line ending).

### Option B — PlatformIO

Create a `platformio.ini` pointing `src_dir = .` (or copy `hand_controller_v10.ino` and `config.h` into `src/`):

```ini
[env:esp32-s3]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
lib_deps =
  adafruit/Adafruit PWM Servo Driver Library@^3.0.2
```

Then build and flash:

```bash
pio run -t upload
pio device monitor -b 115200
```

---

## Serial Command Reference (`115200 baud`)

All commands are case-insensitive ASCII lines terminated by `\n`.

| Command | Description | Example |
| :--- | :--- | :--- |
| `PRESET <name>` | Smoothly transition to a built-in pose (`REST`, `OPEN`, `FIST`, `PINCH`, `CYLINDER`, `TRIPOD`, `SPREAD`, `WAVE`). | `PRESET PINCH` |
| `SET <ch\|idx> <deg>` | Move one channel (`0..10` or name) or virtual wrist DOF (`wrist_pitch`, `wrist_yaw`). | `SET index 75`<br>`SET wrist_pitch 20`<br>`SET wrist_yaw -15`<br>`SET pronation 45` |
| `POSE <11 angles>` | Atomically command `<idx> <mid> <rng> <pnk> <th_flex> <th_opp> <cup> <spread> <wpitch> <wyaw> <pron>`. | `POSE 60 60 60 60 55 65 20 0 5 0 0` |
| `TRIM <ch\|idx> <us>` | Apply live microsecond trim (`-250 .. +250 µs`) for tendon zeroing. | `TRIM index -25` |
| `STATUS` | Output single-line JSON telemetry (firmware version, enabled flag, live wrist pitch/yaw, and per-channel `cur`, `tgt`, `us`, `trim`). | `STATUS` |
| `RELAX` / `STOP` | Emergency stop: immediately disables all 11 PCA9685 PWM outputs (`setPWM(ch, 0, 0)`). | `RELAX` |
| `HELP` | Print command summary. | `HELP` |

---

## Python Serial CLI (`hand_cli.py`)

Install `pyserial` only when talking to physical hardware (`--dry-run` uses Python's standard library only):

```bash
pip install pyserial
```

### Usage Examples

```bash
# Offline dry-run validation & scripted 11-DOF demo (no hardware needed)
python3 firmware/hand_controller_v10/hand_cli.py --dry-run --demo

# Preview a single command in dry-run mode (including wrist differential mixing)
python3 firmware/hand_controller_v10/hand_cli.py --dry-run --set wrist_pitch 45
python3 firmware/hand_controller_v10/hand_cli.py --dry-run --preset PINCH

# Connect to live ESP32-S3 over USB-Serial (interactive REPL)
python3 firmware/hand_controller_v10/hand_cli.py --port /dev/cu.usbserial-0001

# Send a one-shot pose or emergency RELAX to live hardware
python3 firmware/hand_controller_v10/hand_cli.py --port /dev/cu.usbserial-0001 --preset OPEN
python3 firmware/hand_controller_v10/hand_cli.py --port /dev/cu.usbserial-0001 --relax
```

---

## First-Power-On Tendon Zero-Calibration Procedure

Follow this sequence before tensioning any Dyneema tendon or bolting the wrist pushrods:

1. **Disconnect all servo horns & pushrods** from the 11 servos before applying servo rail power.
2. **Flash & boot**: Power the ESP32-S3 via USB, connect via `hand_cli.py --port ...`, and confirm `READY Robotic Hand v10.0` appears. The controller boots directly into `REST`.
3. **Power the servo rail (`6.0 V – 7.4 V`)**: Verify every servo settles quietly at its `REST` neutral pulse (`STATUS` reports `enabled: true`).
4. **Mount horns at neutral**:
   - Send `PRESET OPEN` (`0°` flexion on `CH00–CH04`, `0°` cupping on `CH06`) and mount the winch spools/horns with maximum unwind travel available.
   - Send `SET spread 0`, `SET wrist_pitch 0`, `SET wrist_yaw 0`, `SET pronation 0`, and mount `CH07–CH10` horns/gears centered.
5. **Attach & trim tendons**:
   - Route and tie Dyneema tendons lightly taut at `PRESET OPEN`.
   - Use `TRIM <ch> <us_offset>` in small `10–20 µs` increments until each finger sits cleanly against its `0°` dorsal hard stop with zero slack and zero buzzing.
   - Run `STATUS`, record each channel's trimmed `us`, and write the final `neutral_us` values permanently into `SERVO_CONFIGS` in `config.h`, then re-flash.
6. **Verify range & emergency stop**:
   - Step slowly through `SET index 30`, `SET index 60`, `SET index 90`, then test `PRESET PINCH`, `PRESET CYLINDER`, `PRESET FIST`, and `SET wrist_pitch 20` / `SET wrist_yaw 15`.
   - Keep `RELAX` ready at all times — sending `RELAX` (or `STOP`) immediately kills all PWM outputs (`0` torque) to protect tendons and linkages.
