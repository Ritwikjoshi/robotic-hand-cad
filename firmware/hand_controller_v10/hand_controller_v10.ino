// ============================================================================
// Robotic Hand v10.0 — ESP32-S3 + PCA9685 11-Servo Controller Firmware
// ============================================================================
// Features:
//  - PCA9685 12-bit I2C PWM driver (CH00-CH10) at 50 Hz
//  - Differential Wrist Kinematics Mixer with anti-saturation clamping
//  - Non-blocking 50 Hz cubic ease-out slew-rate trajectory engine
//  - Safe REST boot sequence and instant RELAX / STOP PWM release
//  - Human-readable 115200 baud serial protocol with JSON STATUS telemetry
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

static Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

// Trajectory & calibration runtime state
static float    currentDeg[NUM_SERVOS];
static float    startDeg[NUM_SERVOS];
static float    targetDeg[NUM_SERVOS];
static float    moveElapsed[NUM_SERVOS];
static float    moveDuration[NUM_SERVOS];
static int16_t  trimUs[NUM_SERVOS];
static uint16_t lastPulseUs[NUM_SERVOS];

static float    wristPitchTarget = 0.0f;
static float    wristYawTarget   = 0.0f;
static bool     outputsEnabled   = false;

// Timing & serial RX state
static uint32_t lastTickMs = 0;
static char     rxBuf[192];
static size_t   rxLen = 0;

// ============================================================================
// Utility & Kinematics Helpers
// ============================================================================

static float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int16_t clampInt16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}

static bool equalsIgnoreCase(const char* a, const char* b) {
  if (!a || !b) return false;
  while (*a && *b) {
    if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
      return false;
    }
    ++a;
    ++b;
  }
  return (*a == '\0' && *b == '\0');
}

// Differential Wrist Kinematics Mixer
// Maps Pitch in [-45, +45] deg and Yaw in [-35, +35] deg onto CH08 (Left)
// and CH09 (Right) pushrod servos with proportional anti-saturation clamping
// so combined pitch + yaw never binds or stalls the Cardan linkage.
static void mixWrist(float pitchDeg, float yawDeg, float& leftOutDeg, float& rightOutDeg) {
  float p = clampFloat(pitchDeg, WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG);
  float y = clampFloat(yawDeg, WRIST_YAW_MIN_DEG, WRIST_YAW_MAX_DEG);

  float rawLeft  = WRIST_PITCH_GAIN * p + WRIST_YAW_GAIN * y;
  float rawRight = WRIST_PITCH_GAIN * p - WRIST_YAW_GAIN * y;

  float peak = fmaxf(fabsf(rawLeft), fabsf(rawRight));
  if (peak > WRIST_SERVO_MAX_DEG && peak > 1e-4f) {
    float scale = WRIST_SERVO_MAX_DEG / peak;
    rawLeft  *= scale;
    rawRight *= scale;
  }

  leftOutDeg  = clampFloat(rawLeft,
                           SERVO_CONFIGS[CH_WRIST_LEFT].min_deg,
                           SERVO_CONFIGS[CH_WRIST_LEFT].max_deg);
  rightOutDeg = clampFloat(rawRight,
                           SERVO_CONFIGS[CH_WRIST_RIGHT].min_deg,
                           SERVO_CONFIGS[CH_WRIST_RIGHT].max_deg);
}

static void unmixWrist(float leftDeg, float rightDeg, float& pitchOut, float& yawOut) {
  pitchOut = 0.5f * (leftDeg + rightDeg) / WRIST_PITCH_GAIN;
  yawOut   = 0.5f * (leftDeg - rightDeg) / WRIST_YAW_GAIN;
}

// Piecewise-linear angle-to-microseconds conversion anchored at (rest_deg -> neutral_us)
// so builders can trim neutral_us without shifting hard endpoint bounds.
static uint16_t angleToPulseUs(uint8_t ch, float deg) {
  const ServoConfig& cfg = SERVO_CONFIGS[ch];
  float clamped = clampFloat(deg, cfg.min_deg, cfg.max_deg);

  float pulse = (float)cfg.neutral_us;
  if (clamped >= cfg.rest_deg && cfg.max_deg > cfg.rest_deg) {
    float t = (clamped - cfg.rest_deg) / (cfg.max_deg - cfg.rest_deg);
    pulse = (float)cfg.neutral_us + t * ((float)cfg.max_us - (float)cfg.neutral_us);
  } else if (clamped < cfg.rest_deg && cfg.rest_deg > cfg.min_deg) {
    float t = (clamped - cfg.min_deg) / (cfg.rest_deg - cfg.min_deg);
    pulse = (float)cfg.min_us + t * ((float)cfg.neutral_us - (float)cfg.min_us);
  }

  if (cfg.invert) {
    pulse = (float)(cfg.min_us + cfg.max_us) - pulse;
  }

  int32_t withTrim = (int32_t)lroundf(pulse) + (int32_t)trimUs[ch];
  if (withTrim < GLOBAL_MIN_PULSE_US) withTrim = GLOBAL_MIN_PULSE_US;
  if (withTrim > GLOBAL_MAX_PULSE_US) withTrim = GLOBAL_MAX_PULSE_US;
  return (uint16_t)withTrim;
}

static uint16_t pulseUsToTicks(uint16_t us) {
  const float periodUs = 1000000.0f / (float)PWM_FREQ_HZ;
  int32_t ticks = (int32_t)lroundf(((float)us * 4096.0f) / periodUs);
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

static void writeChannelPwm(uint8_t ch, float deg) {
  if (!outputsEnabled || ch >= NUM_SERVOS) return;
  uint16_t us = angleToPulseUs(ch, deg);
  lastPulseUs[ch] = us;
  pwm.setPWM(SERVO_CONFIGS[ch].channel, 0, pulseUsToTicks(us));
}

// ============================================================================
// Trajectory Engine (Cubic Ease-Out + Slew-Rate Cap)
// ============================================================================

static void setChannelTarget(uint8_t ch, float deg) {
  if (ch >= NUM_SERVOS) return;
  const ServoConfig& cfg = SERVO_CONFIGS[ch];
  float clamped = clampFloat(deg, cfg.min_deg, cfg.max_deg);

  startDeg[ch]    = currentDeg[ch];
  targetDeg[ch]   = clamped;
  moveElapsed[ch] = 0.0f;

  float delta = fabsf(targetDeg[ch] - startDeg[ch]);
  if (delta < 0.01f || cfg.max_deg_per_sec <= 0.0f) {
    moveDuration[ch] = 0.0f;
  } else {
    // Cubic ease-out f(u) = 1 - (1 - u)^3 has max slope f'(0) = 3.0.
    // Scaling duration by 3.0 guarantees peak angular rate <= max_deg_per_sec.
    moveDuration[ch] = (3.0f * delta) / cfg.max_deg_per_sec;
  }

  if (!outputsEnabled) {
    outputsEnabled = true;
    for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
      writeChannelPwm(i, currentDeg[i]);
    }
  }
}

static void applyWristTargets(float pitchDeg, float yawDeg) {
  wristPitchTarget = clampFloat(pitchDeg, WRIST_PITCH_MIN_DEG, WRIST_PITCH_MAX_DEG);
  wristYawTarget   = clampFloat(yawDeg, WRIST_YAW_MIN_DEG, WRIST_YAW_MAX_DEG);

  float leftDeg = 0.0f;
  float rightDeg = 0.0f;
  mixWrist(wristPitchTarget, wristYawTarget, leftDeg, rightDeg);
  setChannelTarget(CH_WRIST_LEFT, leftDeg);
  setChannelTarget(CH_WRIST_RIGHT, rightDeg);
}

static void applyPose(const float dof[NUM_SERVOS]) {
  setChannelTarget(CH_INDEX,      dof[0]);
  setChannelTarget(CH_MIDDLE,     dof[1]);
  setChannelTarget(CH_RING,       dof[2]);
  setChannelTarget(CH_PINKY,      dof[3]);
  setChannelTarget(CH_THUMB_FLEX, dof[4]);
  setChannelTarget(CH_THUMB_OPP,  dof[5]);
  setChannelTarget(CH_CUP,        dof[6]);
  setChannelTarget(CH_SPREAD,     dof[7]);
  applyWristTargets(dof[8], dof[9]);
  setChannelTarget(CH_PRONATION,  dof[10]);
}

static bool applyPresetByName(const char* name) {
  for (uint8_t i = 0; i < NUM_PRESETS; ++i) {
    if (equalsIgnoreCase(name, GRASP_PRESETS[i].name)) {
      const PosePreset& p = GRASP_PRESETS[i];
      const float dof[NUM_SERVOS] = {
        p.index_deg, p.middle_deg, p.ring_deg, p.pinky_deg,
        p.thumb_flex_deg, p.thumb_opp_deg, p.cup_deg, p.spread_deg,
        p.wrist_pitch_deg, p.wrist_yaw_deg, p.pronation_deg
      };
      applyPose(dof);
      return true;
    }
  }
  return false;
}

static void relaxAll() {
  outputsEnabled = false;
  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    pwm.setPWM(SERVO_CONFIGS[ch].channel, 0, 0);
    startDeg[ch]     = currentDeg[ch];
    targetDeg[ch]    = currentDeg[ch];
    moveElapsed[ch]  = 0.0f;
    moveDuration[ch] = 0.0f;
    lastPulseUs[ch]  = 0;
  }
}

static void stepTrajectoryEngine() {
  if (!outputsEnabled) return;

  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    float nextDeg = currentDeg[ch];

    if (moveDuration[ch] <= 0.0f || moveElapsed[ch] >= moveDuration[ch]) {
      nextDeg = targetDeg[ch];
    } else {
      moveElapsed[ch] += CONTROL_DT_S;
      float u = clampFloat(moveElapsed[ch] / moveDuration[ch], 0.0f, 1.0f);
      float inv = 1.0f - u;
      float eased = 1.0f - (inv * inv * inv);  // Cubic ease-out
      nextDeg = startDeg[ch] + (targetDeg[ch] - startDeg[ch]) * eased;

      // Belt-and-suspenders hard slew cap per tick
      float maxStep = SERVO_CONFIGS[ch].max_deg_per_sec * CONTROL_DT_S;
      float step = nextDeg - currentDeg[ch];
      if (step > maxStep)  nextDeg = currentDeg[ch] + maxStep;
      if (step < -maxStep) nextDeg = currentDeg[ch] - maxStep;
    }

    currentDeg[ch] = nextDeg;
    writeChannelPwm(ch, currentDeg[ch]);
  }
}

// ============================================================================
// Serial Protocol Helpers & Command Handlers
// ============================================================================

static int8_t resolveChannelToken(const char* tok) {
  if (!tok || *tok == '\0') return -1;

  // Check pure integer index (0..10)
  char* endPtr = nullptr;
  long idx = strtol(tok, &endPtr, 10);
  if (endPtr != tok && *endPtr == '\0') {
    if (idx >= 0 && idx < NUM_SERVOS) {
      return (int8_t)idx;
    }
    return -1;
  }

  // Match canonical channel names
  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    if (equalsIgnoreCase(tok, SERVO_CONFIGS[ch].name)) {
      return (int8_t)ch;
    }
  }
  return -1;
}

static void printStatusJson() {
  float curPitch = 0.0f;
  float curYaw   = 0.0f;
  unmixWrist(currentDeg[CH_WRIST_LEFT], currentDeg[CH_WRIST_RIGHT], curPitch, curYaw);

  Serial.print(F("{\"fw\":\""));
  Serial.print(F(FW_VERSION));
  Serial.print(F("\",\"enabled\":"));
  Serial.print(outputsEnabled ? F("true") : F("false"));
  Serial.print(F(",\"wrist_pitch\":"));
  Serial.print(curPitch, 1);
  Serial.print(F(",\"wrist_yaw\":"));
  Serial.print(curYaw, 1);
  Serial.print(F(",\"channels\":["));

  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    if (ch > 0) Serial.print(',');
    Serial.print(F("{\"ch\":"));
    Serial.print(ch);
    Serial.print(F(",\"name\":\""));
    Serial.print(SERVO_CONFIGS[ch].name);
    Serial.print(F("\",\"cur\":"));
    Serial.print(currentDeg[ch], 1);
    Serial.print(F(",\"tgt\":"));
    Serial.print(targetDeg[ch], 1);
    Serial.print(F(",\"us\":"));
    Serial.print(lastPulseUs[ch]);
    Serial.print(F(",\"trim\":"));
    Serial.print(trimUs[ch]);
    Serial.print('}');
  }

  Serial.println(F("]}"));
}

static void printHelp() {
  Serial.println(F("=== Robotic Hand v10.0 Controller (ESP32-S3 + PCA9685) ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  PRESET <REST|OPEN|FIST|PINCH|CYLINDER|TRIPOD|SPREAD|WAVE>"));
  Serial.println(F("  SET <ch_name|0..10|wrist_pitch|wrist_yaw> <deg>"));
  Serial.println(F("  POSE <idx> <mid> <rng> <pnk> <th_flex> <th_opp> <cup> <spread> <wpitch> <wyaw> <pron>"));
  Serial.println(F("  TRIM <ch_name|0..10> <us_offset>   (clamped to -250..+250 us)"));
  Serial.println(F("  STATUS                             (JSON telemetry)"));
  Serial.println(F("  RELAX | STOP                       (disable all PWM outputs)"));
  Serial.println(F("  HELP"));
}

static void handleCommand(char* line) {
  // Strip leading whitespace
  while (*line && isspace((unsigned char)*line)) ++line;
  if (*line == '\0') return;

  // Tokenize by whitespace
  const uint8_t MAX_TOKENS = 14;
  char* tokens[MAX_TOKENS];
  uint8_t nTok = 0;

  char* savePtr = nullptr;
  char* tok = strtok_r(line, " \t\r\n", &savePtr);
  while (tok && nTok < MAX_TOKENS) {
    tokens[nTok++] = tok;
    tok = strtok_r(nullptr, " \t\r\n", &savePtr);
  }
  if (nTok == 0) return;

  const char* cmd = tokens[0];

  if (equalsIgnoreCase(cmd, "HELP")) {
    printHelp();
    return;
  }

  if (equalsIgnoreCase(cmd, "STATUS")) {
    printStatusJson();
    return;
  }

  if (equalsIgnoreCase(cmd, "RELAX") || equalsIgnoreCase(cmd, "STOP")) {
    relaxAll();
    Serial.println(F("OK RELAXED"));
    return;
  }

  if (equalsIgnoreCase(cmd, "PRESET")) {
    if (nTok != 2) {
      Serial.println(F("ERR USAGE: PRESET <name>"));
      return;
    }
    if (applyPresetByName(tokens[1])) {
      Serial.print(F("OK PRESET "));
      Serial.println(tokens[1]);
    } else {
      Serial.print(F("ERR UNKNOWN_PRESET "));
      Serial.println(tokens[1]);
    }
    return;
  }

  if (equalsIgnoreCase(cmd, "SET")) {
    if (nTok != 3) {
      Serial.println(F("ERR USAGE: SET <ch_name|idx|wrist_pitch|wrist_yaw> <deg>"));
      return;
    }
    char* endPtr = nullptr;
    float deg = strtof(tokens[2], &endPtr);
    if (endPtr == tokens[2] || *endPtr != '\0') {
      Serial.println(F("ERR INVALID_ANGLE"));
      return;
    }

    if (equalsIgnoreCase(tokens[1], "wrist_pitch")) {
      applyWristTargets(deg, wristYawTarget);
      Serial.print(F("OK SET wrist_pitch "));
      Serial.println(wristPitchTarget, 1);
      return;
    }
    if (equalsIgnoreCase(tokens[1], "wrist_yaw")) {
      applyWristTargets(wristPitchTarget, deg);
      Serial.print(F("OK SET wrist_yaw "));
      Serial.println(wristYawTarget, 1);
      return;
    }

    int8_t ch = resolveChannelToken(tokens[1]);
    if (ch < 0) {
      Serial.print(F("ERR UNKNOWN_CHANNEL "));
      Serial.println(tokens[1]);
      return;
    }
    setChannelTarget((uint8_t)ch, deg);
    if (ch == CH_WRIST_LEFT || ch == CH_WRIST_RIGHT) {
      unmixWrist(targetDeg[CH_WRIST_LEFT], targetDeg[CH_WRIST_RIGHT],
                 wristPitchTarget, wristYawTarget);
    }
    Serial.print(F("OK SET "));
    Serial.print(SERVO_CONFIGS[ch].name);
    Serial.print(' ');
    Serial.println(targetDeg[ch], 1);
    return;
  }

  if (equalsIgnoreCase(cmd, "POSE")) {
    if (nTok != (1 + NUM_SERVOS)) {
      Serial.println(F("ERR USAGE: POSE <11 angles>"));
      return;
    }
    float dof[NUM_SERVOS];
    for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
      char* endPtr = nullptr;
      dof[i] = strtof(tokens[1 + i], &endPtr);
      if (endPtr == tokens[1 + i] || *endPtr != '\0') {
        Serial.println(F("ERR INVALID_POSE_ARG"));
        return;
      }
    }
    applyPose(dof);
    Serial.println(F("OK POSE"));
    return;
  }

  if (equalsIgnoreCase(cmd, "TRIM")) {
    if (nTok != 3) {
      Serial.println(F("ERR USAGE: TRIM <ch_name|idx> <us_offset>"));
      return;
    }
    int8_t ch = resolveChannelToken(tokens[1]);
    if (ch < 0) {
      Serial.print(F("ERR UNKNOWN_CHANNEL "));
      Serial.println(tokens[1]);
      return;
    }
    char* endPtr = nullptr;
    long offset = strtol(tokens[2], &endPtr, 10);
    if (endPtr == tokens[2] || *endPtr != '\0') {
      Serial.println(F("ERR INVALID_TRIM"));
      return;
    }
    trimUs[ch] = clampInt16((int32_t)offset, TRIM_MIN_US, TRIM_MAX_US);
    if (outputsEnabled) {
      writeChannelPwm((uint8_t)ch, currentDeg[ch]);
    }
    Serial.print(F("OK TRIM "));
    Serial.print(SERVO_CONFIGS[ch].name);
    Serial.print(' ');
    Serial.println(trimUs[ch]);
    return;
  }

  Serial.print(F("ERR UNKNOWN_CMD "));
  Serial.println(cmd);
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      handleCommand(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      // Line overflow: flush until newline
      rxLen = 0;
      Serial.println(F("ERR LINE_TOO_LONG"));
    }
  }
}

// ============================================================================
// Arduino Entry Points
// ============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);

  pwm.begin();
  pwm.setOscillatorFrequency(PCA9685_OSCILLATOR_HZ);
  pwm.setPWMFreq(PWM_FREQ_HZ);
  delay(20);

  // Initialize every channel directly to its safe REST neutral angle
  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    float rest = SERVO_CONFIGS[ch].rest_deg;
    currentDeg[ch]   = rest;
    startDeg[ch]     = rest;
    targetDeg[ch]    = rest;
    moveElapsed[ch]  = 0.0f;
    moveDuration[ch] = 0.0f;
    trimUs[ch]       = 0;
    lastPulseUs[ch]  = 0;
  }

  wristPitchTarget = 0.0f;
  wristYawTarget   = 0.0f;
  outputsEnabled   = true;

  // Apply REST pose safely on boot
  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) {
    writeChannelPwm(ch, currentDeg[ch]);
  }
  applyPresetByName("REST");

  lastTickMs = millis();
  Serial.println(F("READY Robotic Hand v10.0 (type HELP for commands)"));
}

void loop() {
  pollSerial();

  uint32_t now = millis();
  if ((uint32_t)(now - lastTickMs) >= CONTROL_DT_MS) {
    lastTickMs = now;
    stepTrajectoryEngine();
  }
}
