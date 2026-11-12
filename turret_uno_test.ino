#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

// v07/v08: motor1 = yaw - pitch; motor2 = -2.5 * yaw.
// Counts are emitted pulses, NOT encoder measurements. No automatic homing.
bool boardReady = false;
long position[2] = {0, 0}, target[2], deltaAbs[2], errorDDA[2];
int8_t directionLogical[2];
long totalTicks = 0, tick = 0;
float speedLimit = 0;
uint32_t dueUs = 0;
bool armed = false, referenced = false, moving = false;
char line[80];
uint8_t lineLen = 0;
bool discardLine = false;

void enableDrivers(bool on) {
  if (!boardReady) return;
  for (uint8_t i = 0; i < 2; ++i)
    digitalWrite(EN_PIN[i], (on ^ ENABLE_ACTIVE_LOW[i]) ? HIGH : LOW);
}
void stopNow() {
  moving = false; armed = false; referenced = false;
  if (boardReady) {
    digitalWrite(STEP_PIN[0], LOW); digitalWrite(STEP_PIN[1], LOW);
    enableDrivers(false);
  }
}
// Send only if the complete short response fits; never wait for USB/serial TX.
void reply(const char *s) {
  size_t n = strlen(s);
  if (Serial.availableForWrite() >= (int)(n + 2)) Serial.println(s);
}
float yawNow() { return -360.0f * position[1] / (2.5f * PULSES_PER_REV[1]); }
float pitchNow() { return yawNow() - 360.0f * position[0] / PULSES_PER_REV[0]; }
float boundarySpeed(long k) {
  long distance = k < totalTicks-k ? k : totalTicks-k;
  return fminf(speedLimit, sqrtf(2.0f * MASTER_ACCEL * distance));
}
uint32_t intervalFor(long k) {
  // Interval from boundary k to k+1; symmetric acceleration/deceleration.
  float sum = boundarySpeed(k) + boundarySpeed(k+1);
  float seconds = sum > 0 ? 2.0f / sum : 2.0f / sqrtf(MASTER_ACCEL);
  // Single-tick move also respects the speed ceiling.
  seconds = fmaxf(seconds, 1.0f / speedLimit);
  return (uint32_t)ceilf(seconds * 1000000.0f);
}
bool number(const char *s, float &v) {
  if (!s || !*s) return false;
  char *end;
  v = (float)strtod(s, &end);
  return end != s && *end == '\0' && isfinite(v);
}
void status() {
  // Kept below AVR serial TX buffer capacity, and called only when idle.
  char y[14], p[14], out[64];
  dtostrf(yawNow(), 0, 2, y); dtostrf(pitchNow(), 0, 2, p);
  snprintf(out, sizeof(out), "STATE C%d A%d R%d Y%s P%s",
           boardReady ? 1 : 0, armed, referenced, y, p);
  reply(out);
}
void command(char *s) {
  char *argv[5]; uint8_t argc = 0; char *save;
  for (char *t = strtok_r(s, " \t", &save); t; t = strtok_r(NULL, " \t", &save)) {
    if (argc == 5) { reply("ERR ARGS"); return; }
    argv[argc++] = t;
  }
  if (!argc) return;
  if ((!strcmp(argv[0], "STOP") || !strcmp(argv[0], "OFF")) && argc == 1) {
    stopNow(); reply("STOPPED REFERENCE_LOST"); return;
  }
  if (!strcmp(argv[0], "HELLO") && argc == 1) {
    reply(boardReady ? "TURRET_UNO_TEST_V1 CONFIG1" : "TURRET_UNO_TEST_V1 CONFIG0"); return;
  }
  if (!strcmp(argv[0], "CAPS") && argc == 1) { reply("CAPS JOGXY"); return; }
  if (moving) { reply("ERR BUSY"); return; }
  if (!strcmp(argv[0], "STATUS") && argc == 1) { status(); return; }
  if (!strcmp(argv[0], "HELP") && argc == 1) {
    reply("ARM ZERO MOVE yaw pitch [deg/s] STATUS STOP OFF !"); return;
  }
  if (!strcmp(argv[0], "CONFIRM") && argc == 1) {
    if (!boardReady) { boardReady = true; initMotorPins(); }
    reply("TURRET_UNO_TEST_V1 CONFIG1"); return;
  }
  if (!boardReady) { reply("ERR CONFIG_NOT_CONFIRMED"); return; }
  if (!strcmp(argv[0], "CAL") && argc == 3) {
    float x, y;
    if (!number(argv[1], x) || !number(argv[2], y) || x < 200 || x > 102400 || y < 200 || y > 102400) {
      reply("ERR CAL_RANGE"); return;
    }
    PULSES_PER_REV[1] = x; PULSES_PER_REV[0] = y;
    position[0] = position[1] = 0; referenced = false;
    reply("CAL_SET_REFERENCE_LOST"); return;
  }
  if ((!strcmp(argv[0], "JOG") || !strcmp(argv[0], "JOGXY")) && argc == 3) {
    if (!armed) { reply("ERR ARM_FIRST"); return; }
    float steps[2] = {0, 0}; // motor 1 = Y; motor 2 = X
    bool valid;
    if (!strcmp(argv[0], "JOGXY")) {
      valid = number(argv[1], steps[1]) && number(argv[2], steps[0]);
    } else {
      valid = (!strcmp(argv[1], "X") || !strcmp(argv[1], "Y")) &&
              number(argv[2], steps[!strcmp(argv[1], "X") ? 1 : 0]);
    }
    for (uint8_t i = 0; i < 2; ++i)
      valid = valid && fabsf(steps[i]) <= 200 && steps[i] == truncf(steps[i]);
    if (!valid || (!steps[0] && !steps[1])) { reply("ERR JOG"); return; }
    referenced = false; position[0] = position[1] = 0;
    totalTicks = (long)fmaxf(fabsf(steps[0]), fabsf(steps[1]));
    tick = 0; speedLimit = 50.0f;
    for (uint8_t i = 0; i < 2; ++i) {
      errorDDA[i] = 0; deltaAbs[i] = labs((long)steps[i]);
      directionLogical[i] = steps[i] >= 0 ? 1 : -1;
      digitalWrite(DIR_PIN[i], ((directionLogical[i] > 0) ^ DIR_INVERT[i]) ? HIGH : LOW);
    }
    dueUs = micros() + intervalFor(0); moving = true; reply("MOVING"); return;
  }
  if (!strcmp(argv[0], "ARM") && argc == 1) {
    if (!armed) { enableDrivers(true); delay(ENABLE_SETTLE_MS); armed = true; referenced = false; }
    reply("ARMED ALIGN_THEN_ZERO"); return;
  }
  if (!strcmp(argv[0], "ZERO") && argc == 1) {
    if (!armed) { reply("ERR ARM_FIRST"); return; }
    position[0] = position[1] = 0; referenced = true;
    reply("ZERO_SET_MANUALLY"); return;
  }
  if (strcmp(argv[0], "MOVE") || (argc != 3 && argc != 4)) { reply("ERR COMMAND"); return; }
  if (!armed || !referenced) { reply("ERR NEED_ARM_AND_ZERO"); return; }
  float yaw, pitch, feed = DEFAULT_DEG_S;
  if (!number(argv[1], yaw) || !number(argv[2], pitch) ||
      (argc == 4 && !number(argv[3], feed))) { reply("ERR NUMBER"); return; }
  if (fabsf(yaw) > YAW_LIMIT || fabsf(pitch) > PITCH_LIMIT || feed < 0.1f || feed > MAX_DEG_S) {
    reply("ERR LIMIT"); return;
  }
  target[0] = lroundf((yaw - pitch) * PULSES_PER_REV[0] / 360.0f);
  target[1] = lroundf(-2.5f * yaw * PULSES_PER_REV[1] / 360.0f);
  // Check quantized endpoint too: never intentionally command outside limits.
  float actualYaw = -360.0f * target[1] / (2.5f * PULSES_PER_REV[1]);
  float actualPitch = actualYaw - 360.0f * target[0] / PULSES_PER_REV[0];
  if (fabsf(actualYaw) > YAW_LIMIT + 0.0001f || fabsf(actualPitch) > PITCH_LIMIT + 0.0001f) {
    reply("ERR QUANTIZED_LIMIT"); return;
  }
  totalTicks = 0;
  for (uint8_t i = 0; i < 2; ++i) {
    long delta = target[i] - position[i];
    directionLogical[i] = delta >= 0 ? 1 : -1; deltaAbs[i] = labs(delta);
    if (deltaAbs[i] > totalTicks) totalTicks = deltaAbs[i];
    errorDDA[i] = 0;
  }
  if (!totalTicks) { reply("DONE"); return; }
  float travel = fmaxf(fabsf(actualYaw - yawNow()), fabsf(actualPitch - pitchNow()));
  speedLimit = fminf(MAX_MASTER_HZ, totalTicks * feed / fmaxf(travel, 0.00001f));
  for (uint8_t i = 0; i < 2; ++i)
    digitalWrite(DIR_PIN[i], ((directionLogical[i] > 0) ^ DIR_INVERT[i]) ? HIGH : LOW);
  tick = 0; dueUs = micros() + intervalFor(0); moving = true; reply("MOVING");
}
void serialService() {
  // Bounded input processing, immediate '!' even in a malformed/overflowed line.
  for (uint8_t count = 0; count < 16 && Serial.available(); ++count) {
    char c = (char)Serial.read();
    if (c == '!') { stopNow(); lineLen = 0; discardLine = true; reply("STOPPED REFERENCE_LOST"); continue; }
    if (c == '\r') continue;
    if (c == '\n') {
      if (!discardLine) { line[lineLen] = 0; command(line); }
      lineLen = 0; discardLine = false; continue;
    }
    if (discardLine) continue;
    if (c < 32 || c > 126 || lineLen >= sizeof(line)-1) {
      discardLine = true; lineLen = 0; reply("ERR LINE"); continue;
    }
    line[lineLen++] = c;
  }
}
void motionService() {
  if (!moving || (int32_t)(micros() - dueUs) < 0) return;
  bool pulse[2];
  for (uint8_t i = 0; i < 2; ++i) {
    errorDDA[i] += deltaAbs[i]; pulse[i] = errorDDA[i] >= totalTicks;
    if (pulse[i]) { errorDDA[i] -= totalTicks; digitalWrite(STEP_PIN[i], HIGH); }
  }
  delayMicroseconds(PULSE_US);
  for (uint8_t i = 0; i < 2; ++i) if (pulse[i]) {
    digitalWrite(STEP_PIN[i], LOW); position[i] += directionLogical[i];
  }
  ++tick;
  if (tick >= totalTicks) { moving = false; reply("DONE"); return; }
  // Delays slow movement; never send bursts to catch up after a stalled loop.
  dueUs = micros() + intervalFor(tick);
}
void initMotorPins() {
  if (boardReady) {
    for (uint8_t i = 0; i < 2; ++i) {
      // Set inactive output latch before switching the pin to output mode.
      digitalWrite(EN_PIN[i], ENABLE_ACTIVE_LOW[i] ? HIGH : LOW); pinMode(EN_PIN[i], OUTPUT);
      digitalWrite(STEP_PIN[i], LOW); pinMode(STEP_PIN[i], OUTPUT);
      digitalWrite(DIR_PIN[i], LOW); pinMode(DIR_PIN[i], OUTPUT);
    }
  }
}
void setup() {
  Serial.begin(115200);
  reply(boardReady ? "TURRET READY DISARMED" : "TURRET LOCKED CONFIG_REQUIRED");
}
void loop() { serialService(); motionService(); }
