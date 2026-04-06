#include "ButtonInput.h"

void ButtonInput::begin(int pin, bool activeLow, bool useInternalPulls, uint32_t debounceMs) {
  _pin = pin;
  _activeLow = activeLow;
  _useInternalPulls = useInternalPulls;
  _debounceMs = debounceMs;

  if (_useInternalPulls) {
    pinMode(_pin, _activeLow ? INPUT_PULLUP : INPUT_PULLDOWN);
  } else {
    pinMode(_pin, INPUT); // rely on external pull
  }

  sync();
}

bool ButtonInput::readRawPressed() const {
  int lvl = digitalRead(_pin);
  return _activeLow ? (lvl == LOW) : (lvl == HIGH);
}

void ButtonInput::sync() {
  uint32_t now = millis();
  bool r = readRawPressed();

  _lastRaw = r;
  _stable = r;
  _lastChangeMs = now;

  _pressedMs = r ? now : 0;
  _lastDownMs = r ? now : 0;

  _downEvent = false;
  _tapEvent = false;
  _holdFired = false;

  // IMPORTANT: prevents “release after sync” from generating a tap
  _hadDownEdge = false;
}

void ButtonInput::clearEvents() {
  _downEvent = false;
  _tapEvent = false;
}

void ButtonInput::update() {
  uint32_t now = millis();
  bool r = readRawPressed();

  if (r != _lastRaw) {
    _lastRaw = r;
    _lastChangeMs = now;
  }

  if ((now - _lastChangeMs) >= _debounceMs && _stable != _lastRaw) {
    _stable = _lastRaw;

    if (_stable) {
      _pressedMs = now;
      _lastDownMs = now;
      _holdFired = false;
      _downEvent = true;
      _hadDownEdge = true;
    } else {
      uint32_t dur = now - _pressedMs;
      if (_hadDownEdge && !_holdFired && dur >= MIN_TAP_MS) {
        _tapEvent = true;
      }
      _hadDownEdge = false;
    }
  }
}

bool ButtonInput::isDown() const {
  return _stable;
}

bool ButtonInput::wasPressedDown() {
  bool e = _downEvent;
  _downEvent = false;
  return e;
}

bool ButtonInput::wasTapped() {
  bool e = _tapEvent;
  _tapEvent = false;
  return e;
}

bool ButtonInput::wasHeld(uint32_t holdMs) {
  if (!_stable) return false;
  if (_holdFired) return false;

  uint32_t now = millis();
  if ((now - _pressedMs) >= holdMs) {
    _holdFired = true;
    return true;
  }
  return false;
}