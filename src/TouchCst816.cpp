#include "TouchCst816.h"
#include <Wire.h>

bool TouchCst816::begin(int sdaPin, int sclPin, int intPin, bool useInterrupt, uint32_t i2cHz) {
  _intPin = intPin;

  // IMPORTANT: GPIO37 (and 34-39) has NO internal pullups.
  // We'll default to polling I2C unless you move IRQ to a pin that supports pulls
  // or add an external pullup on IRQ.
  _useInterrupt = useInterrupt && (_intPin >= 0) && (_intPin < 34);

  Wire.begin(sdaPin, sclPin);
  Wire.setClock(i2cHz);

  if (_intPin >= 0) {
    pinMode(_intPin, INPUT); // NOT INPUT_PULLUP on GPIO37
  }

  // Retry probe for ~1 second (covers shared reset timing)
  _present = false;
  for (int i = 0; i < 20; i++) {
    Wire.beginTransmission(CST816_ADDR);
    if (Wire.endTransmission(true) == 0) { _present = true; break; }
    delay(50);
  }

  Serial.print("[TOUCH] CST816 probe @0x15: ");
  Serial.println(_present ? "FOUND" : "NOT FOUND");
  if (!_present) return false;

  // Disable auto-sleep so touch stays responsive
  Wire.beginTransmission(CST816_ADDR);
  Wire.write(0xFE);           // DisAutoSleep
  Wire.write((uint8_t)0x01);  // non-zero disables auto sleep
  Wire.endTransmission(true);

  return true;
}

bool TouchCst816::i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);

  // IMPORTANT: use STOP to avoid i2cWriteReadNonStop
  if (Wire.endTransmission(true) != 0) return false;

  size_t got = Wire.requestFrom((int)addr, (int)n); // normal requestFrom
  if (got != n) return false;

  for (size_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

TouchPoint TouchCst816::read(uint16_t screenW, uint16_t screenH) {
  TouchPoint tp{false, 0, 0};
  if (!_present) return tp;

  uint32_t now = millis();

  // If we recently had failures, back off to avoid error spam + UI issues
  if (now < _cooldownUntilMs) return tp;

  // Polling mode (default): read at most every 20ms
  if (!_useInterrupt) {
    if (now - _lastPollMs < 20) return tp;
    _lastPollMs = now;
  } else {
    // Interrupt mode: only read when INT says there's touch
    if (_intPin < 0) return tp;
    int lvl = digitalRead(_intPin);
    if (INT_ACTIVE_LOW) {
      if (lvl == HIGH) return tp;
    } else {
      if (lvl == LOW) return tp;
    }
  }

  uint8_t b[6] = {0};
  if (!i2cReadBytes(CST816_ADDR, 0x01, b, sizeof(b))) {
    _failStreak++;

    // back off progressively
    if (_failStreak >= 3)  _cooldownUntilMs = now + 250;
    if (_failStreak >= 10) {
      Serial.println("[TOUCH] Too many I2C failures; disabling touch until reboot.");
      _present = false; // stop any future reads (prevents repeated Wire errors)
    }
    return tp;
  }

  _failStreak = 0;

  uint8_t fingers = b[1];
  if (fingers == 0) return tp;

  uint16_t rawX = ((uint16_t)(b[2] & 0x0F) << 8) | b[3];
  uint16_t rawY = ((uint16_t)(b[4] & 0x0F) << 8) | b[5];

  uint16_t sx = (uint32_t)rawX * screenW  / (RAW_MAX_X ? RAW_MAX_X : 1);
  uint16_t sy = (uint32_t)rawY * screenH  / (RAW_MAX_Y ? RAW_MAX_Y : 1);

  if (SWAP_XY) { uint16_t t = sx; sx = sy; sy = t; }
  if (INVERT_X) sx = screenW - 1 - sx;
  if (INVERT_Y) sy = screenH - 1 - sy;

  tp.touched = true;
  tp.x = (uint16_t)constrain((int)sx, 0, (int)screenW - 1);
  tp.y = (uint16_t)constrain((int)sy, 0, (int)screenH - 1);
  return tp;
}