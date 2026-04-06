#pragma once
#include <Arduino.h>

class ButtonInput {
public:
  void begin(int pin, bool activeLow, bool useInternalPulls, uint32_t debounceMs = 30);
  void update();

  // Clears queued edge events (does NOT reset hold timing)
  void clearEvents();

  // Resyncs stable state to current raw state (use sparingly)
  void sync();

  bool isDown() const;

  // Edge events (consume-on-read)
  bool wasPressedDown();   // stable DOWN edge
  bool wasTapped();        // stable UP edge after short press

  // Hold event (one-shot per press)
  bool wasHeld(uint32_t holdMs);

  uint32_t lastDownTimeMs() const { return _lastDownMs; }

private:
  bool readRawPressed() const;

  int  _pin = -1;
  bool _activeLow = false;
  bool _useInternalPulls = false;

  uint32_t _debounceMs = 30;

  bool _lastRaw = false;
  bool _stable  = false;
  uint32_t _lastChangeMs = 0;

  uint32_t _pressedMs = 0;
  uint32_t _lastDownMs = 0;

  bool _downEvent = false;
  bool _tapEvent  = false;

  bool _holdFired = false;
  bool _hadDownEdge = false;

  static constexpr uint32_t MIN_TAP_MS = 40;
};