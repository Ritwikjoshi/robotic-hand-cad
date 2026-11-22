#ifndef HAND_CONTROLLER_V10_CONFIG_H
#define HAND_CONTROLLER_V10_CONFIG_H

#include <stdint.h>

// ============================================================================
// Robotic Hand v10.0 — ESP32-S3 + PCA9685 (16-Ch 12-Bit I2C PWM) Configuration
// ============================================================================

#define FW_VERSION              "10.0.0"
#define SERIAL_BAUD             115200

// ESP32-S3 I2C Bus & PCA9685 Hardware Settings
#define I2C_SDA_PIN             8
#define I2C_SCL_PIN             9
#define I2C_CLOCK_HZ            400000
#define PCA9685_ADDR            0x40
#define PWM_FREQ_HZ             50
// Standard PCA9685 internal clock is 25 MHz; adjust to ~27000000 if measured
// pulse widths on your specific breakout board drift by ~8%.
#define PCA9685_OSCILLATOR_HZ   25000000

// Trajectory Control Loop Timing (50 Hz synchronized with servo PWM frame)
#define CONTROL_LOOP_HZ         50
#define CONTROL_DT_MS           (1000 / CONTROL_LOOP_HZ)
#define CONTROL_DT_S            (1.0f / (float)CONTROL_LOOP_HZ)

// Global Hardware Pulse Safety Clamps (microseconds)
#define GLOBAL_MIN_PULSE_US     500
#define GLOBAL_MAX_PULSE_US     2500
#define TRIM_MIN_US             (-250)
#define TRIM_MAX_US             250

// Differential Wrist Kinematics Limits (degrees)
#define WRIST_PITCH_MIN_DEG     (-45.0f)
#define WRIST_PITCH_MAX_DEG     45.0f
#define WRIST_YAW_MIN_DEG       (-35.0f)
#define WRIST_YAW_MAX_DEG       35.0f
#define WRIST_SERVO_MIN_DEG     (-60.0f)
#define WRIST_SERVO_MAX_DEG     60.0f
#define WRIST_PITCH_GAIN        1.0f
#define WRIST_YAW_GAIN          1.0f

// Forearm Pronation 22T -> 44T (2:1) Gear Reduction Metadata
// For full +/-85 deg forearm output (+/-170 deg pinion), use a 270-deg DS3218.
#define PRONATION_GEAR_RATIO    2.0f

// ============================================================================
// PCA9685 Channel Mapping (CH00 - CH10)
// ============================================================================

enum ServoChannel : uint8_t {
  CH_INDEX       = 0,   // CH00: Index Flexor (MG90S / KST DS215MG)
  CH_MIDDLE      = 1,   // CH01: Middle Flexor (MG90S / KST DS215MG)
  CH_RING        = 2,   // CH02: Ring Flexor (MG90S / KST DS215MG)
  CH_PINKY       = 3,   // CH03: Pinky Flexor (MG90S / KST DS215MG)
  CH_THUMB_FLEX  = 4,   // CH04: Thumb Flexor (MG90S / KST DS215MG)
  CH_THUMB_OPP   = 5,   // CH05: Thumb Opposition (MG90S / KST DS215MG)
  CH_CUP         = 6,   // CH06: Palmar Arch Cupping - 4th/5th Ray (MG90S)
  CH_SPREAD      = 7,   // CH07: Finger Abduction / Spread Linkage (MG90S / SG90)
  CH_WRIST_LEFT  = 8,   // CH08: Wrist Differential Left (DS3218 / MG996R)
  CH_WRIST_RIGHT = 9,   // CH09: Wrist Differential Right (DS3218 / MG996R)
  CH_PRONATION   = 10,  // CH10: Forearm Pronation/Supination 22T->44T (DS3218 / MG996R)
  NUM_SERVOS     = 11
};

// ============================================================================
// Per-Channel Calibration & Slew-Rate Configuration
// ============================================================================

struct ServoConfig {
  uint8_t     channel;          // PCA9685 channel index (0..10)
  const char* name;             // Canonical serial command name
  const char* servo_model;      // Recommended servo class
  uint16_t    min_us;           // Pulse width (us) at min_deg (pre-inversion)
  uint16_t    max_us;           // Pulse width (us) at max_deg (pre-inversion)
  uint16_t    neutral_us;       // Trimmed pulse width (us) at rest_deg
  float       min_deg;          // Hard software lower limit (deg)
  float       max_deg;          // Hard software upper limit (deg)
  float       rest_deg;         // Safe neutral/boot angle (deg)
  bool        invert;           // Reverse horn/tendon direction
  float       max_deg_per_sec;  // Peak angular velocity limit (deg/s)
};

// Builders should tune `neutral_us`, `min_us`, `max_us`, and `invert` on the
// bench during first-power-on tendon zero-calibration before loading tendons.
static const ServoConfig SERVO_CONFIGS[NUM_SERVOS] = {
  // ch              name          model                  min_us  max_us  neutral_us  min_deg  max_deg  rest_deg  invert  max_dps
  { CH_INDEX,       "index",       "MG90S / KST DS215MG",   600,   2300,     868,       0.0f,   95.0f,   15.0f,   false,  180.0f },
  { CH_MIDDLE,      "middle",      "MG90S / KST DS215MG",   600,   2300,     868,       0.0f,   95.0f,   15.0f,   false,  180.0f },
  { CH_RING,        "ring",        "MG90S / KST DS215MG",   600,   2300,     868,       0.0f,   95.0f,   15.0f,   false,  180.0f },
  { CH_PINKY,       "pinky",       "MG90S / KST DS215MG",   600,   2300,     868,       0.0f,   95.0f,   15.0f,   false,  180.0f },
  { CH_THUMB_FLEX,  "thumb_flex",  "MG90S / KST DS215MG",   650,   2250,     917,       0.0f,   90.0f,   15.0f,   false,  160.0f },
  { CH_THUMB_OPP,   "thumb_opp",   "MG90S / KST DS215MG",   700,   2200,    1117,       0.0f,   90.0f,   25.0f,   false,  150.0f },
  { CH_CUP,         "cup",         "MG90S",                 900,   1900,    1067,       0.0f,   30.0f,    5.0f,   false,   90.0f },
  { CH_SPREAD,      "spread",      "MG90S / SG90",         1000,   2000,    1500,     -15.0f,   15.0f,    0.0f,   false,   90.0f },
  { CH_WRIST_LEFT,  "wrist_left",  "DS3218 / MG996R",       800,   2200,    1500,     -60.0f,   60.0f,    0.0f,   false,  120.0f },
  { CH_WRIST_RIGHT, "wrist_right", "DS3218 / MG996R",       800,   2200,    1500,     -60.0f,   60.0f,    0.0f,   true,   120.0f },
  { CH_PRONATION,   "pronation",   "DS3218 / MG996R",       550,   2450,    1500,     -85.0f,   85.0f,    0.0f,   false,  110.0f }
};

// ============================================================================
// Built-In Grasp & Pose Presets (11 Logical DOFs)
// ============================================================================

struct PosePreset {
  const char* name;
  float index_deg;
  float middle_deg;
  float ring_deg;
  float pinky_deg;
  float thumb_flex_deg;
  float thumb_opp_deg;
  float cup_deg;
  float spread_deg;
  float wrist_pitch_deg;
  float wrist_yaw_deg;
  float pronation_deg;
};

static const PosePreset GRASP_PRESETS[] = {
  // name        idx    mid    rng    pnk   th_flx th_opp  cup   sprd  wpitch  wyaw   pron
  { "REST",      15.0f, 15.0f, 15.0f, 15.0f, 15.0f, 25.0f,  5.0f,  0.0f,  0.0f,  0.0f,  0.0f },
  { "OPEN",       0.0f,  0.0f,  0.0f,  0.0f,  0.0f, 10.0f,  0.0f,  5.0f,  0.0f,  0.0f,  0.0f },
  { "FIST",      90.0f, 92.0f, 92.0f, 90.0f, 80.0f, 75.0f, 25.0f, -5.0f,  5.0f,  0.0f,  0.0f },
  { "PINCH",     68.0f, 12.0f, 10.0f, 10.0f, 62.0f, 82.0f,  8.0f,  0.0f,  5.0f,  0.0f,  0.0f },
  { "CYLINDER",  60.0f, 62.0f, 62.0f, 58.0f, 55.0f, 65.0f, 20.0f,  0.0f,  5.0f,  0.0f,  0.0f },
  { "TRIPOD",    65.0f, 65.0f, 15.0f, 12.0f, 60.0f, 80.0f, 10.0f,  0.0f,  5.0f,  0.0f,  0.0f },
  { "SPREAD",     0.0f,  0.0f,  0.0f,  0.0f,  0.0f, 15.0f,  0.0f, 15.0f,  0.0f,  0.0f,  0.0f },
  { "WAVE",       5.0f,  5.0f,  5.0f,  5.0f, 10.0f, 20.0f,  0.0f, 10.0f, 15.0f, 25.0f,  0.0f }
};

static const uint8_t NUM_PRESETS = sizeof(GRASP_PRESETS) / sizeof(GRASP_PRESETS[0]);

#endif  // HAND_CONTROLLER_V10_CONFIG_H
