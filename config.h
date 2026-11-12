#pragma once
// TMC2209 standalone STEP/DIR. UART is NOT configured by this sketch.
// GPIO stays untouched until operator confirms the shield in the GUI.
const uint8_t STEP_PIN[2] = {3, 2}; // motor1=Y, motor2=X; standard Uno CNC Shield V3
const uint8_t DIR_PIN[2]  = {6, 5};
const uint8_t EN_PIN[2]   = {8, 8}; // shared ENABLE
const bool DIR_INVERT[2] = {false, false};
const bool ENABLE_ACTIVE_LOW[2] = {true, true}; // TMC2209 ENN: LOW enables
// EXAMPLES ONLY: confirm motor step angle and actual MS1/MS2 levels first.
const uint16_t FULL_STEPS_PER_REV[2] = {200, 200}; // 1.8-degree motors, unconfirmed
const uint16_t MICROSTEPS[2] = {8, 8}; // assumed no MS1/MS2 jumpers
// TMC2209 standalone (MS2,MS1): 00=8, 01=32, 10=64, 11=16.
// Internal interpolation to 256 does NOT change the STEP pulse count.
float PULSES_PER_REV[2] = {
  float(FULL_STEPS_PER_REV[0]) * MICROSTEPS[0],
  float(FULL_STEPS_PER_REV[1]) * MICROSTEPS[1]
};
const float YAW_LIMIT = 90.0f, PITCH_LIMIT = 45.0f;
const float DEFAULT_DEG_S = 3.0f, MAX_DEG_S = 10.0f;
const float MAX_MASTER_HZ = 800.0f, MASTER_ACCEL = 400.0f; // pulses/s, pulses/s^2
const uint16_t PULSE_US = 6, ENABLE_SETTLE_MS = 10;
