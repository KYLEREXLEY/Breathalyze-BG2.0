#include "BreathalyzerApp.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <qrcode.h>

#include "app_config.h"
#include "ButtonInput.h"
#include "BleUart.h"
#include "TouchCst816.h"
#include "DataStore.h"   // <-- added

#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_system.h"

// ============================= QR CAPACITY ================================
static const uint16_t QR_BYTE_CAPACITY_LOW[] = {
  0,
  17, 32, 53, 78, 106, 134, 154, 192, 230,
  271, 321, 367, 425, 458, 520, 586, 644, 718, 792,
  858, 929, 1003, 1091, 1171, 1273, 1367, 1465, 1528, 1628,
  1732, 1840, 1952, 2068, 2188, 2303, 2431
};

static uint8_t chooseQrVersionForLength(size_t len) {
  const uint8_t maxVersion = 36;
  for (uint8_t v = 1; v <= maxVersion; v++) {
    if (len <= QR_BYTE_CAPACITY_LOW[v]) return v;
  }
  return maxVersion;
}

// ============================= INTERNAL TYPES =============================
struct SensorSample {
  uint32_t t;
  float v[NUM_SENSORS];
};

static inline float adcToVolts(int raw) {
  return (raw * 3.3f) / 4095.0f;
}

// ============================= BreathalyzerAppImpl ========================
class BreathalyzerAppImpl {
public:
  void begin(const HardwareConfig& hw);
  void update();

private:
  // Hardware
  HardwareConfig _hw{};

  // Button
  ButtonInput _button;
  bool _offArmed = true;
  uint32_t _stableReleasedSinceMs = 0;

  // Ready screen arming (fixes intermission press skipping Ready)
  bool _readyArmed = false;
  uint32_t _readyReleaseSinceMs = 0;

  // TFT
  Adafruit_ST7789* _tft = nullptr;
  bool _displayInited = false;

  // Touch
  TouchCst816 _touch;
  bool _touchInited = false;

  // BLE
  BleUart _ble;

  // QR
  QRCode _qr{};
  uint8_t* _qrBuf = nullptr;
  bool _qrDrawn = false;
  uint32_t _autoOffDeadlineMs = 0;

  // Battery widget
  static constexpr int TOP_H = 36;
  static constexpr int BOTTOM_OVERLAY_H = 34;

  static constexpr int BATT_W = 36;
  static constexpr int BATT_H = 10;
  static constexpr int BATT_NUB_W = 5;
  static constexpr int BATT_PAD_R = 6;
  static constexpr int BATT_Y = 6;
  static constexpr int BATT_TEXT_W = 30;

  uint32_t _lastBattMs = 0;
  uint8_t  _lastBattPct = 255;

  bool     _heaterOn = false;
  uint32_t _lowBattSinceMs = 0;

  // Live overlay
  uint32_t _lastOverlayMs = 0;
  SensorSample _lastSample{};
  bool _haveLastSample = false;

  // Sensor ring buffer
  static constexpr int RING_SIZE = 220; // ~4.4s at 20ms
  SensorSample _ring[RING_SIZE];
  int _ringHead = 0;
  int _ringCount = 0;
  uint32_t _lastSampleMs = 0;

  // DATA MODEL:
  float _baseVals[NUM_SENSORS][3] = {0};
  bool  _baselineCaptured = false;

  float _resVals[2][NUM_SENSORS][3] = {0};

  bool _candidateValid = false;
  uint32_t _candidatePressMs = 0;
  float _candidatePressVals[NUM_SENSORS] = {0};

  bool _glucoseArmed = false;
  uint32_t _glucoseReleaseSinceMs = 0;

  // Glucose entry
  int _userGlucose = -1;
  int _g0 = 0, _g1 = 0, _g2 = 0;
  bool _glucoseLastTouched = false;
  uint32_t _glucoseLastActionMs = 0;

  // NEW: saved-run tracking + completion
  bool _testsComplete = false;
  bool _savedThisRun  = false;
  uint32_t _savedSeq  = 0;
  uint32_t _appTimestamp = 0;
  bool _hasAppTimestamp = false;
  uint32_t _lastBleBinaryTxMs = 0;

  // NEW: Done screen timeout
  uint32_t _doneSleepAtMs = 0;

  // State machine
  enum class State { Warmup, Ready, Blow, Intermission, Glucose, ShowQr, Done };
  State _state = State::Warmup;

  uint32_t _warmupStartMs = 0;
  uint8_t  _lastWarmupPct = 255;

  int _currentTest = 0; // 0 or 1
  uint32_t _blowStartMs = 0;

  uint32_t _interStartMs = 0;
  uint32_t _lastInterSecShown = 0xFFFFFFFF;

  void pinsLowestLeakForSleep() {
#if defined(NEOPIXEL_I2C_POWER)
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, LOW);
#endif

    pinMode(_hw.tftCs,   OUTPUT); digitalWrite(_hw.tftCs,   LOW);
    pinMode(_hw.tftDc,   OUTPUT); digitalWrite(_hw.tftDc,   LOW);
    pinMode(_hw.tftSclk, OUTPUT); digitalWrite(_hw.tftSclk, LOW);
    pinMode(_hw.tftMosi, OUTPUT); digitalWrite(_hw.tftMosi, LOW);

    if (_hw.tftMiso >= 0) pinMode(_hw.tftMiso, INPUT);

    if (_hw.tftRst >= 0) { pinMode(_hw.tftRst, OUTPUT); digitalWrite(_hw.tftRst, LOW); }

    pinMode(_hw.tftBl, OUTPUT);
    digitalWrite(_hw.tftBl, HIGH);

    pinMode(_hw.i2cSda, INPUT);
    pinMode(_hw.i2cScl, INPUT);
    pinMode(_hw.touchInt, INPUT);

    setDisplayPower(false);
  }

private:
  // ===================== Low-level helpers =====================
  inline bool buttonRawPressed() const {
    int lvl = digitalRead(_hw.pinButton);
    return _hw.buttonActiveLow ? (lvl == LOW) : (lvl == HIGH);
  }

  void buzzerOn(bool on) { digitalWrite(_hw.pinBuzzer, on ? HIGH : LOW); }
  void heaterEnable(bool on) { 
    _heaterOn = on;
    digitalWrite(_hw.pinHeaterEn, on ? HIGH : LOW); }

  void beep(uint16_t msOn = 120, uint16_t msOff = 80, uint8_t count = 1) {
    for (uint8_t i = 0; i < count; i++) {
      buzzerOn(true);  delay(msOn);
      buzzerOn(false); delay(msOff);
    }
  }

  // Backlight is ACTIVE-LOW
  void setBacklight(bool on) {
    pinMode(_hw.tftBl, OUTPUT);
    digitalWrite(_hw.tftBl, on ? LOW : HIGH);
  }

  // PNP high-side (active LOW): LOW=ON, HIGH=OFF
  void setDisplayPower(bool on) {
    if (_hw.tftPwr < 0) return;
    pinMode(_hw.tftPwr, OUTPUT);
    digitalWrite(_hw.tftPwr, on ? LOW : HIGH);
  }

  void setDisplayReset(bool released) {
    if (_hw.tftRst < 0) return;
    pinMode(_hw.tftRst, OUTPUT);
    digitalWrite(_hw.tftRst, released ? HIGH : LOW);
  }

  void releaseDeepSleepHolds() {
    gpio_deep_sleep_hold_dis();
    if (_hw.tftBl >= 0) gpio_hold_dis((gpio_num_t)_hw.tftBl);
    if (_hw.tftPwr >= 0) gpio_hold_dis((gpio_num_t)_hw.tftPwr);
    if (_hw.tftRst >= 0) gpio_hold_dis((gpio_num_t)_hw.tftRst);
  }

  void holdTftPinsForSleep() {
    setBacklight(false);
    setDisplayReset(false);
    setDisplayPower(false);

    if (_hw.tftBl >= 0) gpio_hold_en((gpio_num_t)_hw.tftBl);
    if (_hw.tftPwr >= 0) gpio_hold_en((gpio_num_t)_hw.tftPwr);
    if (_hw.tftRst >= 0) gpio_hold_en((gpio_num_t)_hw.tftRst);

    gpio_deep_sleep_hold_en();
  }

  void configRtcPullForWake() {
    rtc_gpio_init((gpio_num_t)_hw.pinButton);
    rtc_gpio_set_direction((gpio_num_t)_hw.pinButton, RTC_GPIO_MODE_INPUT_ONLY);

    if (_hw.useInternalPulls) {
      if (_hw.buttonActiveLow) {
        rtc_gpio_pullup_en((gpio_num_t)_hw.pinButton);
        rtc_gpio_pulldown_dis((gpio_num_t)_hw.pinButton);
      } else {
        rtc_gpio_pullup_dis((gpio_num_t)_hw.pinButton);
        rtc_gpio_pulldown_en((gpio_num_t)_hw.pinButton);
      }
    } else {
      rtc_gpio_pullup_dis((gpio_num_t)_hw.pinButton);
      rtc_gpio_pulldown_dis((gpio_num_t)_hw.pinButton);
    }
  }

  void waitForButtonRelease() {
    uint32_t t0 = millis();
    while (buttonRawPressed()) {
      if (millis() - t0 > 8000) break;
      delay(10);
    }
    delay(40);
  }

  [[noreturn]] void goToDeepSleep(bool userInitiated) {
    heaterEnable(false);
    buzzerOn(false);

    if (_displayInited) {
      uiHeader(userInitiated ? "ZZZ..." : "ZZZ... ");
      uiTextLines(" POWERING OFF ","   GOODBYE    ");
      beep(160, 60, 1);
    }

    holdTftPinsForSleep();
    waitForButtonRelease();

    pinsLowestLeakForSleep();

    configRtcPullForWake();
    int wakeLevel = _hw.buttonActiveLow ? 0 : 1;
    esp_sleep_enable_ext0_wakeup((gpio_num_t)_hw.pinButton, wakeLevel);
    esp_deep_sleep_start();

    while (true) delay(1000);
  }

  bool heldStableBlocking(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      if (!buttonRawPressed()) return false;
      delay(5);
    }
    return true;
  }

  // ===================== Display / UI =====================
  void displayInit() {
    releaseDeepSleepHolds();

    setBacklight(false);
    setDisplayPower(true);
    delay(30);

    setDisplayReset(false);
    delay(50);
    setDisplayReset(true);
    delay(200);

    SPI.begin(_hw.tftSclk, _hw.tftMiso, _hw.tftMosi, _hw.tftCs);

    if (!_tft) {
      _tft = new Adafruit_ST7789(_hw.tftCs, _hw.tftDc, _hw.tftRst);
    }

    _tft->init(AppConfig::TFT_W, AppConfig::TFT_H);
    _tft->setRotation(AppConfig::TFT_ROTATION);
    _tft->setTextWrap(false);
    _tft->setSPISpeed(AppConfig::TFT_SPI_HZ);

    _displayInited = true;
    setBacklight(true);

    _touchInited = _touch.begin(_hw.i2cSda, _hw.i2cScl, _hw.touchInt);

    _tft->fillScreen(ST77XX_WHITE);
    _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    _tft->setTextSize(2);
    _tft->setCursor(6, 10);
    _tft->print("BOOT...");
    //updateBatteryWidget(true);
  }

  void uiClear(uint16_t color = ST77XX_WHITE) {
    if (!_displayInited) return;
    _tft->fillScreen(color);
    _tft->setTextWrap(false);
  }

  float readBatteryVolts() {
  analogReadMilliVolts(_hw.pinBattMon); // throw away first read
  delay(2);

  uint32_t sumMv = 0;
  constexpr int N = 8;
  for (int i = 0; i < N; i++) {
    sumMv += analogReadMilliVolts(_hw.pinBattMon);
    delay(2);
  }

  float mv = (float)sumMv / (float)N;
  return (mv * 2.0f) / 1000.0f;
}

  uint8_t batteryPctFromVolts(float v) {
  // Device-usable percentage under load
  constexpr float BATT_V_EMPTY = 3.20f;  // 0%
  constexpr float BATT_V_FULL  = 4.00f;  // 100% under load, tune if needed

  if (v <= BATT_V_EMPTY) return 0;
  if (v >= BATT_V_FULL)  return 100;

  float pct = (v - BATT_V_EMPTY) * 100.0f / (BATT_V_FULL - BATT_V_EMPTY);
  pct = constrain(pct, 0.0f, 100.0f);
  return (uint8_t)(pct + 0.5f);
}

  void drawBatteryBar(uint8_t pct) {
    if (!_displayInited) return;

    const int x = _tft->width() - (BATT_PAD_R + BATT_NUB_W + BATT_W);
    const int y = BATT_Y;

    const int clearX = x - BATT_TEXT_W - 6;
    _tft->fillRect(clearX, 0, _tft->width() - clearX, TOP_H, ST77XX_WHITE);

    _tft->setTextSize(1);
    _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    _tft->setCursor(x - BATT_TEXT_W, y + 2);
    char pbuf[8];
    snprintf(pbuf, sizeof(pbuf), "%3d%%", (int)pct);
    _tft->print(pbuf);

    _tft->drawRect(x, y, BATT_W, BATT_H, ST77XX_BLACK);
    _tft->drawRect(x + BATT_W, y + 3, BATT_NUB_W, BATT_H - 6, ST77XX_BLACK);

    int fillW = (BATT_W - 2) * (int)pct / 100;
    _tft->fillRect(x + 1, y + 1, fillW, BATT_H - 2, ST77XX_GREEN);
    _tft->fillRect(x + 1 + fillW, y + 1, (BATT_W - 2) - fillW, BATT_H - 2, ST77XX_WHITE);
  }

  void updateBatteryWidget(bool force = false) {
  if (!_displayInited) return;

  uint32_t now = millis();
  if (!force && now - _lastBattMs < 1000) return;
  _lastBattMs = now;

  // First boot draw: allow one sample so the icon is not blank
  if (_lastBattPct == 255) {
    uint8_t pct = batteryPctFromVolts(readBatteryVolts());
    _lastBattPct = pct;
    drawBatteryBar(pct);
    return;
  }

  // When heater/sensors are OFF, freeze the displayed percent.
  // This prevents the battery from "jumping back up" after load sag disappears.
  if (!_heaterOn) {
    if (force) drawBatteryBar(_lastBattPct);
    _lowBattSinceMs = 0;
    return;
  }

  // Heater/sensors ON -> show live device battery %
  uint8_t pct = batteryPctFromVolts(readBatteryVolts());

  // Simple smoothing only while loaded
  uint8_t shownPct = (uint8_t)((pct + _lastBattPct) / 2);

  if (shownPct != _lastBattPct || force) {
    _lastBattPct = shownPct;
    drawBatteryBar(_lastBattPct);
  }

  // Optional: low battery auto-off only while loaded
  if (_lastBattPct <= 1) {
    if (_lowBattSinceMs == 0) _lowBattSinceMs = now;

    if (now - _lowBattSinceMs >= 1500) {
      uiHeader("LOW BATT");
      uiTextLines(" CHARGE DEVICE ", " POWERING OFF ");
      beep(120, 60, 2);
      delay(1200);
      goToDeepSleep(false);
    }
  } else {
    _lowBattSinceMs = 0;
  }
}

  void uiHeader(const char* title) {
    if (!_displayInited) return;
    uiClear(ST77XX_WHITE);

    int reserveRight = BATT_TEXT_W + BATT_W + BATT_NUB_W + BATT_PAD_R + 10;
    int maxTitlePx = _tft->width() - reserveRight - 6;
    if (maxTitlePx < 30) maxTitlePx = 30;

    const int charPx = 6 * 2;
    int maxChars = maxTitlePx / charPx;
    if (maxChars < 1) maxChars = 1;

    char buf[40];
    strncpy(buf, title, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    if ((int)strlen(buf) > maxChars) {
      if (maxChars >= 3) {
        buf[maxChars - 1] = '.';
        buf[maxChars - 2] = '.';
        buf[maxChars - 3] = '.';
        buf[maxChars] = 0;
      } else {
        buf[1] = 0;
      }
    }

    _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    _tft->setTextSize(2);
    _tft->setCursor(3, 10);
    _tft->print(buf);

    _tft->drawFastHLine(3, 32, _tft->width() - 12, ST77XX_BLACK);

    int bodyH = _tft->height() - TOP_H - BOTTOM_OVERLAY_H;
    if (bodyH > 0) _tft->fillRect(0, TOP_H, _tft->width(), bodyH, ST77XX_WHITE);
  }

  void uiTextLines(const char* l1, const char* l2=nullptr, const char* l3=nullptr, const char* l4=nullptr, const char* l5=nullptr, const char* l6=nullptr, const char* l7=nullptr, const char* l8=nullptr, const char* l9=nullptr, const char* l10=nullptr, const char* l11=nullptr) {
    if (!_displayInited) return;

    int y0 = 45;
    int h = _tft->height() - y0 - BOTTOM_OVERLAY_H - 4;
    if (h > 0) _tft->fillRect(0, y0, _tft->width(), h, ST77XX_WHITE);

    _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    _tft->setTextSize(2);

    int y = y0;
    auto line = [&](const char* s) {
      if (!s) return;
      _tft->setCursor(3, y);
      _tft->print(s);
      y += 22;
    };
    line(l1); line(l2); line(l3); line(l4); line(l5); line(l6); line(l7); line(l8); line(l9); line(l10); line(l11);

    updateBatteryWidget(false);
  }
  void uiPrintValueAt(int x, int y, int charsWide, const char* fmt, int value) {
  if (!_displayInited) return;

  const int charW = 6 * 2;   // built-in font, textSize(2)
  const int charH = 8 * 2;

  // Clear only the numeric field, not the whole line
  _tft->fillRect(x, y, charsWide * charW, charH, ST77XX_WHITE);

  _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  _tft->setTextSize(2);
  _tft->setCursor(x, y);

  char buf[16];
  snprintf(buf, sizeof(buf), fmt, value);
  _tft->print(buf);
}

  void uiProgressBar(uint8_t percent) {
  if (!_displayInited) return;

  const int x = 10;
  const int y = _tft->height() - BOTTOM_OVERLAY_H - 111;
  const int w = _tft->width() - 20;
  const int h = 16;

  _tft->drawRect(x, y, w, h, ST77XX_BLACK);

  int fillW = (int)((w - 2) * (percent / 100.0f));
  _tft->fillRect(x + 1, y + 1, fillW, h - 2, ST77XX_GREEN);
  _tft->fillRect(x + 1 + fillW, y + 1, (w - 2) - fillW, h - 2, ST77XX_WHITE);

  // Only redraw the 3-digit percentage, not the whole "WARMUP xxx%" line
  uiPrintValueAt(10 + 7 * 12, y - 20, 3, "%3d", (int)percent);

  updateBatteryWidget(false);
}

  void drawLiveSensorOverlay() {
    if (!_displayInited) return;
    uint32_t now = millis();
    if (now - _lastOverlayMs < 250) return;
    _lastOverlayMs = now;

    float v[NUM_SENSORS] = {0};
    if (_haveLastSample) {
      for (size_t i = 0; i < NUM_SENSORS; i++) v[i] = _lastSample.v[i];
    } else {
      for (size_t i = 0; i < NUM_SENSORS; i++) v[i] = adcToVolts(analogRead(_hw.sensorPins[i]));
    }

    const int y = _tft->height() - BOTTOM_OVERLAY_H;
    _tft->fillRect(0, y, _tft->width(), BOTTOM_OVERLAY_H, ST77XX_WHITE);

    _tft->setTextSize(1);
    _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);

    _tft->setCursor(2, y + 4);
    char l1[64];
    snprintf(l1, sizeof(l1), "S1 %.2f  S2 %.2f  S3 %.2f", v[0], v[1], v[2]);
    _tft->print(l1);

    _tft->setCursor(2, y + 18);
    char l2[64];
    snprintf(l2, sizeof(l2), "S4 %.2f  S5 %.2f", v[3], v[4]);
    _tft->print(l2);
  }

  // ===================== Sensor sampling / ring =====================
  void sampleSensorsToRingIfDue() {
    uint32_t now = millis();
    if (now - _lastSampleMs < AppConfig::SAMPLE_PERIOD_MS) return;
    _lastSampleMs = now;

    SensorSample s{};
    s.t = now;
    for (size_t i = 0; i < NUM_SENSORS; i++) {
      s.v[i] = adcToVolts(analogRead(_hw.sensorPins[i]));
    }

    _ring[_ringHead] = s;
    _ringHead = (_ringHead + 1) % RING_SIZE;
    if (_ringCount < RING_SIZE) _ringCount++;

    _lastSample = s;
    _haveLastSample = true;
  }

  bool ringHasHistory(uint32_t needMs) {
    if (_ringCount < 2) return false;
    int oldestIdx = (_ringHead - _ringCount + RING_SIZE) % RING_SIZE;
    uint32_t oldestT = _ring[oldestIdx].t;
    return (millis() - oldestT) >= needMs;
  }

  bool ringGetAtTime(uint32_t targetT, float out[NUM_SENSORS]) {
    if (_ringCount == 0) return false;

    int bestIdx = -1;
    uint32_t bestErr = 0xFFFFFFFF;

    for (int i = 0; i < _ringCount; i++) {
      int idx = (_ringHead - 1 - i + RING_SIZE) % RING_SIZE;
      uint32_t t = _ring[idx].t;
      uint32_t err = (t > targetT) ? (t - targetT) : (targetT - t);
      if (t > targetT) err += 50;
      if (err < bestErr) {
        bestErr = err;
        bestIdx = idx;
        if (bestErr == 0) break;
      }
    }

    if (bestIdx < 0) return false;
    for (size_t j = 0; j < NUM_SENSORS; j++) out[j] = _ring[bestIdx].v[j];
    return true;
  }

  // ===================== Baseline & Results capture =====================
  bool captureBaselineOnceFromCandidate() {
    if (_baselineCaptured) return true;
    if (!_candidateValid) return false;
    if (!ringHasHistory(AppConfig::PREBASE_NEED_MS)) return false;

    float v_m1[NUM_SENSORS], v_m2[NUM_SENSORS];
    bool ok1 = ringGetAtTime(_candidatePressMs - 1000, v_m1);
    bool ok2 = ringGetAtTime(_candidatePressMs - 2000, v_m2);
    if (!(ok1 && ok2)) return false;

    for (size_t s = 0; s < NUM_SENSORS; s++) {
      _baseVals[s][0] = v_m2[s];
      _baseVals[s][1] = v_m1[s];
      _baseVals[s][2] = _candidatePressVals[s];
    }

    _baselineCaptured = true;
    return true;
  }

  void captureResultsAt456(int testIdx) {
    uint32_t t4 = _blowStartMs + 4000;
    uint32_t t5 = _blowStartMs + 5000;
    uint32_t t6 = _blowStartMs + 6000;

    float v4[NUM_SENSORS] = {0};
    float v5[NUM_SENSORS] = {0};
    float v6[NUM_SENSORS] = {0};

    bool ok4 = ringGetAtTime(t4, v4);
    bool ok5 = ringGetAtTime(t5, v5);
    bool ok6 = ringGetAtTime(t6, v6);

    if (!ok4 && _haveLastSample) for (size_t s = 0; s < NUM_SENSORS; s++) v4[s] = _lastSample.v[s];
    if (!ok5 && _haveLastSample) for (size_t s = 0; s < NUM_SENSORS; s++) v5[s] = _lastSample.v[s];
    if (!ok6 && _haveLastSample) for (size_t s = 0; s < NUM_SENSORS; s++) v6[s] = _lastSample.v[s];

    for (size_t s = 0; s < NUM_SENSORS; s++) {
      _resVals[testIdx][s][0] = v4[s];
      _resVals[testIdx][s][1] = v5[s];
      _resVals[testIdx][s][2] = v6[s];
    }
  }

  void buildSudirValues(uint16_t out[45]) {
    int idx = 0;

    auto pack = [&](float v) {
      if (v < 0.0f) v = 0.0f;
      uint32_t scaled = (uint32_t)lroundf(v * 1000.0f);
      if (scaled > 65535UL) scaled = 65535UL;
      out[idx++] = (uint16_t)scaled;
    };

    for (size_t s = 0; s < NUM_SENSORS; s++) {
      for (int k = 0; k < 3; k++) pack(_baseVals[s][k]);
    }

    for (int test = 0; test < 2; test++) {
      for (size_t s = 0; s < NUM_SENSORS; s++) {
        for (int k = 0; k < 3; k++) pack(_resVals[test][s][k]);
      }
    }
  }

  void updateSavedQueryWithLatestResult() {
    if (!_savedThisRun || _savedSeq == 0) return;

    char qbuf[1800];
    buildResultPayload(qbuf, sizeof(qbuf));
    if (DataStore::updateQuery(_savedSeq, qbuf)) {
      Serial.printf("[FLASH] Updated seq=%lu BG=%03d",
                    (unsigned long)_savedSeq, _userGlucose);
      if (_hasAppTimestamp) {
        Serial.printf(" ts=%lu", (unsigned long)_appTimestamp);
      }
      Serial.println();
    } else {
      Serial.printf("[FLASH] Update FAILED seq=%lu\n", (unsigned long)_savedSeq);
    }
  }

  void maybeSendBlePayload() {
    if (!_ble.isConnected()) return;

    if (_ble.mode() == BleClientMode::Logan) {
      if (_ble.consumeReadyToSend()) {
        char urlBuf[1800];
        buildResultUrlBT(urlBuf, sizeof(urlBuf));
        _ble.sendTextPayload(String(urlBuf) + "\n");
      }
      return;
    }

    if (_ble.shouldBroadcastBinary()) {
      uint32_t now = millis();
      if (now - _lastBleBinaryTxMs >= 2000) {
        uint16_t vals[45];
        buildSudirValues(vals);
        _ble.sendBinaryPacket(AppConfig::DEVICE_ID_NUM, vals);
        _lastBleBinaryTxMs = now;
      }
    }

    if (_ble.mode() == BleClientMode::Unknown && _ble.consumeReadyToSend()) {
      char urlBuf[1800];
      buildResultUrlBT(urlBuf, sizeof(urlBuf));
      _ble.sendTextPayload(String(urlBuf) + "\n");
    }
  }

  uint16_t scaleApiValue(float v) {
    if (v < 0.0f) v = 0.0f;
    uint32_t scaled = (uint32_t)lroundf(v * 1000.0f);
    if (scaled > 65535UL) scaled = 65535UL;
    return (uint16_t)scaled;
  }

  void appendApiField(String& url, char group, int idx, float value) {
    url += "&";
    url += group;
    url += String(idx);
    url += "=";
    url += String(scaleApiValue(value));
  }

  void appendApiSensorGroup(String& url, char group, size_t sensorIdx) {
    int idx = 1;

    // baseline -> 1..3
    for (int k = 0; k < 3; k++) {
      appendApiField(url, group, idx++, _baseVals[sensorIdx][k]);
    }

    // test 1 -> 4..6
    for (int k = 0; k < 3; k++) {
      appendApiField(url, group, idx++, _resVals[0][sensorIdx][k]);
    }

    // test 2 -> 7..9
    for (int k = 0; k < 3; k++) {
      appendApiField(url, group, idx++, _resVals[1][sensorIdx][k]);
    }
  }

  void buildResultUrlCommon(char* buffer, size_t bufferSize, bool includeBaseUrl) {
    static const char GROUP_KEYS[NUM_SENSORS] = { 'a', 'b', 'c', 'e', 'f' };

    String url;
    url.reserve(1800);

    if (includeBaseUrl) {
      url += AppConfig::BASE_URL;
    }

    url += "d=";
    url += String(AppConfig::API_DEVICE_ID);

    url += "&p=";
    url += AppConfig::API_PARAM_P;

    // put bg before sensor fields to match expected Logan BLE payload
    if (_userGlucose >= 0) {
      url += "&bg=";
      char gbuf[8];
      snprintf(gbuf, sizeof(gbuf), "%03d", _userGlucose);
      url += gbuf;
    }

    // S1..S5 => a,b,c,e,f
    for (size_t s = 0; s < NUM_SENSORS && s < 5; s++) {
      appendApiSensorGroup(url, GROUP_KEYS[s], s);
    }

    if (_hasAppTimestamp) {
      url += "&ts=";
      url += String((unsigned long)_appTimestamp);
    }

    strncpy(buffer, url.c_str(), bufferSize - 1);
    buffer[bufferSize - 1] = 0;
  }

  // ===================== PAYLOAD ONLY (for flash storage) =====================
  void buildResultPayload(char* buffer, size_t bufferSize) {
    buildResultUrlCommon(buffer, bufferSize, false);
  }

  // ===================== BT_URL =====================
  void buildResultUrlBT(char* buffer, size_t bufferSize) {
    buildResultUrlCommon(buffer, bufferSize, false);
  }

  // ===================== QR_URL =====================
  void buildResultUrlQR(char* buffer, size_t bufferSize) {
    buildResultUrlCommon(buffer, bufferSize, true);
  }

  bool generateQr(const char* text) {
    if (_qrBuf) { free(_qrBuf); _qrBuf = nullptr; }

    size_t len = strlen(text);
    uint8_t version = chooseQrVersionForLength(len);

    uint16_t bufferSize = qrcode_getBufferSize(version);
    _qrBuf = (uint8_t*)malloc(bufferSize);
    if (!_qrBuf) return false;

    int8_t status = qrcode_initText(&_qr, _qrBuf, version, ECC_LOW, text);
    return (status >= 0);
  }

  void drawQrOnBlack(uint8_t quietModules = 4) {
    const uint16_t qrModules = _qr.size;
    const uint16_t totalModules = qrModules + 2 * quietModules;

    uint16_t screenW = _tft->width();
    uint16_t screenH = _tft->height();

    uint16_t moduleSize = min(screenW / totalModules, screenH / totalModules);
    if (moduleSize < 1) moduleSize = 1;

    const uint16_t totalPx = totalModules * moduleSize;
    const uint16_t quietPx = quietModules * moduleSize;

    int16_t x0 = (screenW - totalPx) / 2;
    int16_t y0 = (screenH - totalPx) / 2;

    _tft->fillScreen(ST77XX_WHITE);
    _tft->fillRect(x0, y0, totalPx, totalPx, ST77XX_BLACK);

    int16_t mx0 = x0 + quietPx;
    int16_t my0 = y0 + quietPx;

    for (uint8_t y = 0; y < _qr.size; y++) {
      for (uint8_t x = 0; x < _qr.size; x++) {
        if (qrcode_getModule(&_qr, x, y)) {
          _tft->fillRect(mx0 + x * moduleSize, my0 + y * moduleSize,
                         moduleSize, moduleSize, ST77XX_WHITE); 
        }
      }
    }

    _tft->setTextSize(2);
    _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    _tft->setCursor(3, 30);
    _tft->print(" SCAN QR CODE ");
  }

  // ===================== Glucose UI =====================
  void glucoseDraw() {
    int boxW = 44;
    int boxH = 64;
    int gap  = 10;
    int startX = (_tft->width() - (3 * boxW + 2 * gap)) / 2;
    int y = 110;

    auto drawBox = [&](int x, int digit) {
      _tft->drawRect(x, y - 42, boxW, 36, ST77XX_BLACK);
      _tft->setTextSize(2);
      _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
      _tft->setCursor(x + 16, y - 36);
      _tft->print("^");

      _tft->drawRect(x, y, boxW, boxH, ST77XX_BLACK);
      _tft->setTextSize(4);
      _tft->setCursor(x + 10, y + 12);
      _tft->print(digit);

      _tft->drawRect(x, y + boxH + 6, boxW, 36, ST77XX_BLACK);
      _tft->setTextSize(2);
      _tft->setCursor(x + 16, y + boxH + 12);
      _tft->print("v");
    };

    drawBox(startX + 0 * (boxW + gap), _g0);
    drawBox(startX + 1 * (boxW + gap), _g1);
    drawBox(startX + 2 * (boxW + gap), _g2);

    int btnY = 232;              // keep above overlay (286)
const int bw = 120;          // big button
const int bh = 44;
const int bx = (_tft->width() - bw) / 2;

_tft->setTextSize(2);
_tft->drawRect(bx, btnY, bw, bh, ST77XX_BLACK);
_tft->setCursor(bx + 42, btnY + 14);
_tft->print("OK");

    updateBatteryWidget(true);
  }

  void glucoseEnter() {
    _glucoseArmed = false;
    _glucoseReleaseSinceMs = 0;
    _button.clearEvents();
    _autoOffDeadlineMs = millis() + AppConfig::AUTO_OFF_AFTER_MS;

    // Start BLE here (so app can fetch data + send BG)
    _ble.begin("BGBT_FEATHER_V2");
    _lastBleBinaryTxMs = 0;

    if (_userGlucose >= 0) {
      _g0 = (_userGlucose / 100) % 10;
      _g1 = (_userGlucose / 10) % 10;
      _g2 = (_userGlucose / 1) % 10;
    } else {
      _g0 = 0; _g1 = 0; _g2 = 0;
    }

    _glucoseLastTouched = false;
    _glucoseLastActionMs = 0;

    _state = State::Glucose;
    heaterEnable(false);
    uiHeader("BG#");
    uiTextLines("  ENTER BG#   ");
    glucoseDraw();
  }

 void glucoseUpdate() {
  if (!_glucoseArmed) {
    _button.clearEvents();

    if (_button.isDown()) {
      _glucoseReleaseSinceMs = 0;
      return;
    }

    if (_glucoseReleaseSinceMs == 0) _glucoseReleaseSinceMs = millis();
    if (millis() - _glucoseReleaseSinceMs < 150) return;

    _glucoseArmed = true;
    _button.clearEvents();
  }

  // Physical button tap = OK (still allowed)
  if (_button.wasTapped()) {
    _userGlucose = _g0 * 100 + _g1 * 10 + _g2;

    // If we already saved this run after test 2, update saved record with BG
    updateSavedQueryWithLatestResult();

    beep(80, 40, 1);
    enterShowQr();
    return;
  }

  if (!_touchInited) return;

  TouchPoint tp = _touch.read(_tft->width(), _tft->height());
  bool touched = tp.touched;

  if (touched && (!_glucoseLastTouched) && (millis() - _glucoseLastActionMs > 120)) {
    _glucoseLastActionMs = millis();
    _autoOffDeadlineMs = millis() + AppConfig::AUTO_OFF_AFTER_MS;

    int boxW = 44;
    int boxH = 64;
    int gap  = 10;
    int startX = (_tft->width() - (3 * boxW + 2 * gap)) / 2;
    int y = 110;

    auto hitDigitCol = [&](int col)->int {
      int x = startX + col * (boxW + gap);
      if (tp.x < x || tp.x > x + boxW) return 0;
      if (tp.y >= (uint16_t)(y - 42) && tp.y <= (uint16_t)(y - 6)) return +1;
      if (tp.y >= (uint16_t)(y + boxH + 6) && tp.y <= (uint16_t)(y + boxH + 42)) return -1;
      return 0;
    };

    int delta0 = hitDigitCol(0);
    int delta1 = hitDigitCol(1);
    int delta2 = hitDigitCol(2);

    auto wrapDigit = [&](int &d, int delta) {
      if (delta == 0) return;
      d = (d + delta) % 10;
      if (d < 0) d += 10;
    };

    if (delta0 || delta1 || delta2) {
      wrapDigit(_g0, delta0);
      wrapDigit(_g1, delta1);
      wrapDigit(_g2, delta2);
      glucoseDraw();
    } else {
      // Touch OK button (forced BG entry, no Skip)
      const int btnY = 232;
      const int bw = 120;
      const int bh = 44;
      const int bx = (_tft->width() - bw) / 2;

      if (tp.x >= (uint16_t)bx && tp.x <= (uint16_t)(bx + bw) &&
          tp.y >= (uint16_t)btnY && tp.y <= (uint16_t)(btnY + bh)) {

        _userGlucose = _g0 * 100 + _g1 * 10 + _g2;

        updateSavedQueryWithLatestResult();

        beep(80, 40, 1);
        enterShowQr();

        _glucoseLastTouched = touched;
        return;
      }
    }
  }

  _glucoseLastTouched = touched;
}
  // ===================== State transitions =====================
  uint8_t warmupPercent(uint32_t elapsed, uint32_t total) {
    if (elapsed >= total) return 100;
    uint32_t fastPhase = (uint32_t)(total * 0.75f);
    uint32_t slowPhase = total - fastPhase;

    if (elapsed <= fastPhase) {
      float x = (float)elapsed / (float)fastPhase;
      return (uint8_t)(x * 90.0f);
    } else {
      float x = (float)(elapsed - fastPhase) / (float)slowPhase;
      float eased = x * x;
      return (uint8_t)(90.0f + eased * 10.0f);
    }
  }
  void enterWarmup() {
  heaterEnable(true);
  _warmupStartMs = millis();
  _lastWarmupPct = 255;

  uiHeader("WARMUP");
  uiTextLines("", "HEATING SENSOR", "PLEASE WAIT...", "", "", "", "", "", "", " HOLD BUTTON: ", " TO POWER OFF ");

  const int y = _tft->height() - BOTTOM_OVERLAY_H - 111;
  _tft->setTextSize(2);
  _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  _tft->setCursor(10, y - 20);
  _tft->print("WARMUP");
  _tft->setCursor(10 + 7 * 12, y - 20);   // after "WARMUP "
  _tft->print("   %");                    // placeholder

  uiProgressBar(0);
  _state = State::Warmup;
}

  void enterReady() {
    _state = State::Ready;
    _candidateValid = false;

    _autoOffDeadlineMs = millis() + AppConfig::AUTO_OFF_AFTER_MS;

    _readyArmed = false;
    _readyReleaseSinceMs = 0;
    _button.clearEvents();

    uiHeader(_currentTest == 0 ? "TEST 1" : "TEST 2");
    if (_currentTest == 0 && !_baselineCaptured) {
      uiTextLines("","  TAP BUTTON  " ,"   TO START   ", " THEN BLOW 6S ","","","","","", " HOLD BUTTON: ", " TO POWER OFF ");
    } else {
      uiTextLines("","  TAP BUTTON  " ,"   TO START   ", " THEN BLOW 6S ","","","","","", " HOLD BUTTON: ", " TO POWER OFF ");
    }
  }

  void enterBlow() {
  if (_currentTest == 0 && !_baselineCaptured) {
    captureBaselineOnceFromCandidate();
  }

  _state = State::Blow;
  _blowStartMs = millis();
  beep(90, 40, 1);

  uiHeader(_currentTest == 0 ? "TEST 1" : "TEST 2");
  uiTextLines("",
              "  BLOW NOW!!  ",
              "",
              "",
              "",
              "",
              "",
              "",
              "",
              " HOLD BUTTON: ",
              " TO POWER OFF ");

  const int lineX = 3;
  const int lineY = 45 + 2 * 22;
  const int numX = lineX + 3 * 12;
  const int suffixX = lineX + 5 * 12;

  _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  _tft->setTextSize(2);
  _tft->setCursor(suffixX, lineY);
  _tft->print("S LEFT");

  uiPrintValueAt(numX, lineY, 2, "%2d", 6);
}
void enterIntermission() {
  _state = State::Intermission;
  _interStartMs = millis();
  _lastInterSecShown = 0xFFFFFFFF;

  uiHeader("REST");
  uiTextLines("",
              " INTERMISSION ",
              "",
              "",
              "",
              "",
              "",
              "",
              "",
              " HOLD BUTTON: ",
              " TO POWER OFF ");

  const int lineX   = 3;
  const int lineY   = 45 + 2 * 22;
  const int charW   = 12;          // built-in font at textSize(2)
  const int numX    = lineX + 3 * charW;
  const int suffixX = lineX + 5 * charW;   // leave a full 3-char number field

  _tft->setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  _tft->setTextSize(2);
  _tft->setCursor(suffixX, lineY);
  _tft->print("S LEFT");

  uiPrintValueAt(numX, lineY, 2, "%2d", 10);
}

  void enterShowQr() {
    _state = State::ShowQr;

    heaterEnable(false);
    _autoOffDeadlineMs = millis() + AppConfig::AUTO_OFF_AFTER_MS;
    _qrDrawn = false;

    _ble.begin("BGBT_FEATHER_V2");
    _lastBleBinaryTxMs = 0;
    _button.clearEvents();
  }

public:
  void appBegin() {
    Serial.begin(AppConfig::SERIAL_BAUD);
    delay(120);

    pinMode(_hw.pinBuzzer, OUTPUT);
    pinMode(_hw.pinHeaterEn, OUTPUT);
    setDisplayPower(false);
    buzzerOn(false);
    heaterEnable(false);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    _button.begin(_hw.pinButton, _hw.buttonActiveLow, _hw.useInternalPulls);

    releaseDeepSleepHolds();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    esp_reset_reason_t rr = esp_reset_reason();

    bool startNow = false;

    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
      startNow = heldStableBlocking(AppConfig::ON_HOLD_TO_BOOT_MS);
    } else if (rr == ESP_RST_POWERON) {
      startNow = buttonRawPressed() && heldStableBlocking(AppConfig::ON_HOLD_TO_BOOT_MS);
    } else {
      startNow = true;
    }

    _offArmed = !buttonRawPressed();
    _stableReleasedSinceMs = 0;

    _baselineCaptured = false;
    _userGlucose = -1;
    _appTimestamp = 0;
    _hasAppTimestamp = false;
    _lastBleBinaryTxMs = 0;

    // NEW resets
    _testsComplete = false;
    _savedThisRun = false;
    _savedSeq = 0;
    _doneSleepAtMs = 0;

    if (!startNow) {
      goToDeepSleep(false);
    }

#if defined(NEOPIXEL_I2C_POWER)
    pinMode(NEOPIXEL_I2C_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_I2C_POWER, HIGH);
    delay(5);
#endif

    displayInit();
    beep(90, 60, 1);

    _currentTest = 0;
    enterWarmup();
  }

  void appUpdate() {
    uint32_t now = millis();
    _button.update();

    // Terminal: type DUMP + Enter to print all saved runs
    static String cmd;
    while (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        cmd.trim();
        if (cmd.equalsIgnoreCase("DUMP")) {
          DataStore::dumpAllQueriesToSerial();
        }
        if (cmd.equalsIgnoreCase("ERASE")) {
  DataStore::erase();
  Serial.println("[FLASH] ERASED");
}
        cmd = "";
      } else if (cmd.length() < 32) {
        cmd += c;
      }
    }

    // If tests complete and app sends BG=###, finish immediately
    if (_testsComplete) {
      BleAppResponse resp;
      if (_ble.consumeResponse(resp)) {
        _userGlucose = resp.bg;
        _hasAppTimestamp = resp.hasTimestamp;
        _appTimestamp = resp.timestamp;

        updateSavedQueryWithLatestResult();

        heaterEnable(false);
        uiHeader("ZZZ...");
        uiTextLines("","","   ALL DONE   ", "   GOODBYE!   ");
        beep(120, 60, 2);

        _state = State::Done;
        _doneSleepAtMs = millis() + 3000;
      }
    }

    if (!_offArmed) {
      if (!_button.isDown()) {
        if (_stableReleasedSinceMs == 0) _stableReleasedSinceMs = now;
        if (now - _stableReleasedSinceMs >= AppConfig::ARM_RELEASE_STABLE_MS) _offArmed = true;
      } else {
        _stableReleasedSinceMs = 0;
      }
    }

    if (_offArmed && _button.wasHeld(AppConfig::OFF_HOLD_TO_SLEEP_MS)) {
      goToDeepSleep(true);
    }

    sampleSensorsToRingIfDue();

    if (_state != State::ShowQr) {
      //drawLiveSensorOverlay(); //uncomment for live sensor overlay
      updateBatteryWidget(false);
    }

    switch (_state) {
      case State::Warmup: {
        uint32_t elapsed = now - _warmupStartMs;
        uint8_t pct = warmupPercent(elapsed, AppConfig::WARMUP_DURATION_MS);
        if (pct != _lastWarmupPct) {
          uiProgressBar(pct);
          _lastWarmupPct = pct;
        }
        if (elapsed >= AppConfig::WARMUP_DURATION_MS) {
          beep(120, 80, 2);
          _currentTest = 0;
          enterReady();
        }
        break;
      }

      case State::Ready: {
        if ((int32_t)(now - _autoOffDeadlineMs) >= 0) {
          goToDeepSleep(true);
        }
        if (!_readyArmed) {
          _button.clearEvents();
          if (_button.isDown()) {
            _readyReleaseSinceMs = 0;
            break;
          }
          if (_readyReleaseSinceMs == 0) _readyReleaseSinceMs = now;
          if (now - _readyReleaseSinceMs >= AppConfig::READY_ARM_RELEASE_MS) {
            _readyArmed = true;
            _button.clearEvents();
          }
          break;
        }

        if (_currentTest == 0 && !_baselineCaptured) {
          if (_button.wasPressedDown()) {
            _candidatePressMs = millis();
            for (size_t s = 0; s < NUM_SENSORS; s++) {
              _candidatePressVals[s] = adcToVolts(analogRead(_hw.sensorPins[s]));
            }
            _candidateValid = true;
          }
        }

        if (_button.wasTapped()) {
          if (_currentTest == 0 && !_baselineCaptured) {
            if (!_candidateValid) {
              _candidatePressMs = millis();
              for (size_t s = 0; s < NUM_SENSORS; s++) {
                _candidatePressVals[s] = adcToVolts(analogRead(_hw.sensorPins[s]));
              }
              _candidateValid = true;
            }
          }

          enterBlow();
        }
        break;
      }

      case State::Blow: {
        uint32_t elapsed = now - _blowStartMs;

        if (elapsed >= AppConfig::BLOW_WINDOW_MS) {
          captureResultsAt456(_currentTest);
          beep(140, 60, 1);

          if (_currentTest == 0) {
            enterIntermission();
          } else {
            // Test 2 done -> save immediately, then go to glucose screen
            _testsComplete = true;

            char qbuf[1800];
            buildResultPayload(qbuf, sizeof(qbuf));
            if (DataStore::appendQuery(qbuf, _savedSeq)) {
              _savedThisRun = true;
              Serial.printf("[FLASH] Saved run seq=%lu\n", (unsigned long)_savedSeq);
            } else {
              Serial.println("[FLASH] Save FAILED");
            }

            glucoseEnter();
          }
          break;
        }

        uint32_t remain = AppConfig::BLOW_WINDOW_MS - elapsed;
        uint32_t sec = (remain + 999) / 1000;

        static uint32_t lastShown = 0xFFFFFFFF;
if (sec != lastShown) {
  lastShown = sec;

  const int lineX = 3;
  const int lineY = 45 + 2 * 22;
  const int numX = lineX + 3 * 12;

  uiPrintValueAt(numX, lineY, 1, "%2d", (int)sec);
}

        break;
      }

      case State::Intermission: {
        uint32_t elapsed = now - _interStartMs;

        if (elapsed >= AppConfig::INTERTEST_DELAY_MS) {
          beep(120, 80, 1);
          _currentTest = 1;
          enterReady();
          break;
        }

        uint32_t remain = AppConfig::INTERTEST_DELAY_MS - elapsed;
        uint32_t sec = (remain + 999) / 1000;

        if (sec != _lastInterSecShown) {
  _lastInterSecShown = sec;

  const int lineX = 3;
  const int lineY = 45 + 2 * 22;
  const int charW = 12;
  const int numX  = lineX + 3 * charW;

  uiPrintValueAt(numX, lineY, 2, "%2d", (int)sec);
}
        break;
      }

      case State::Glucose: {
        if ((int32_t)(now - _autoOffDeadlineMs) >= 0) {
          goToDeepSleep(true);
        }

        maybeSendBlePayload();

        glucoseUpdate();
        break;
      }

      case State::ShowQr: {
        if ((int32_t)(now - _autoOffDeadlineMs) >= 0) {
          goToDeepSleep(true);
        }

        if (!_qrDrawn) {
          char urlBuf[1800];
          //Both Logan and Sudir paths use same QR and BLE URL format now
          if (AppConfig::USE_SUDIR_QR) {
            //buildResultUrlQRSudir(urlBuf, sizeof(urlBuf));
            buildResultUrlQR(urlBuf, sizeof(urlBuf));
          } else {
            buildResultUrlQR(urlBuf, sizeof(urlBuf));
          }

          Serial.print("QR URL: ");
          Serial.println(urlBuf);

          if (generateQr(urlBuf)) {
            drawQrOnBlack(4);
            _qrDrawn = true;
          } else {
            uiTextLines("QR gen failed", " HOLD BUTTON: ", " TO POWER OFF ");
          }
        }
        maybeSendBlePayload();

        if (_button.wasTapped()) _qrDrawn = false;
        break;
      }

      case State::Done: {
        if ((int32_t)(now - _doneSleepAtMs) >= 0) {
          goToDeepSleep(true);
        }
        break;
      }
    }
  }
};

static BreathalyzerAppImpl gImpl;

// ============================= BreathalyzerApp ============================
void BreathalyzerApp::begin(const HardwareConfig& hw) {
  gImpl.begin(hw);
}
void BreathalyzerApp::update() {
  gImpl.update();
}

// ============================= Impl glue ============================
void BreathalyzerAppImpl::begin(const HardwareConfig& hw) {
  _hw = hw;
  appBegin();
}
void BreathalyzerAppImpl::update() {
  appUpdate();
}