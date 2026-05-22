#include <Arduino.h>
#include "esp_task_wdt.h"

#include "hardware_config.h"
#include "BreathalyzerApp.h"

// ============================= FEATHER V2 PINS ============================
// Active-HIGH wake button (pressed=HIGH). External pulldown to GND.
static const int PIN_BUTTON   = 32;   // GPIO32 (RTC-capable)

// Outputs
static const int PIN_BUZZER   = 8;    // TX = GPIO8 (active HIGH buzzer)
static const int PIN_HEATEREN = 7;    // RX = GPIO7

// TFT SPI pins (Feather hardware SPI)
static const int TFT_SCLK = 5;        // SCK  GPIO5
static const int TFT_MOSI = 19;       // MOSI GPIO19
static const int TFT_MISO = 21;       // MISO GPIO21 (not used by ST7789 but ok)

// TFT control pins
static const int TFT_CS   = 14;       // D14 GPIO14
static const int TFT_DC   = 33;       // D33 GPIO33

// TFT backlight control (ACTIVE-LOW).
static const int PIN_TFT_BL  = 27;    // D27 GPIO27

// TFT reset pin (wired)
static const int PIN_TFT_RST = 4;     // GPIO4 (A5) -> TFT RST (active-low)

// TFT VCC control enable pin 
static const int PIN_TFT_PWR = 15;    

// Battery monitor (internal divider): GPIO35
static const int PIN_BATT_MON = 35;

// I2C for CST816 touch
static const int PIN_I2C_SCL = 20;
static const int PIN_I2C_SDA = 22;

// Touch INT
static const int PIN_TOUCH_INT = 37;

// 5 sensor pins
static constexpr size_t NUM_SENS = 5;
static const std::array<int, NUM_SENS> SENSOR_PINS = { 26, 25, 34, 39, 36 };

// ============================= BUTTON POLARITY ============================
// config: Active HIGH + external pulldown (idle LOW, pressed HIGH)
static constexpr bool BUTTON_ACTIVE_LOW = false;

// If button ever feels floaty, set true 
static constexpr bool USE_INTERNAL_PULLS = false;

// ============================= HARDWARE CONFIG ============================
static const HardwareConfig HW = {
  .pinButton        = PIN_BUTTON,
  .buttonActiveLow  = BUTTON_ACTIVE_LOW,
  .useInternalPulls = USE_INTERNAL_PULLS,

  .pinBuzzer        = PIN_BUZZER,
  .pinHeaterEn      = PIN_HEATEREN,

  .tftSclk          = TFT_SCLK,
  .tftMosi          = TFT_MOSI,
  .tftMiso          = TFT_MISO,

  .tftCs            = TFT_CS,
  .tftDc            = TFT_DC,
  .tftBl            = PIN_TFT_BL,
  .tftRst           = PIN_TFT_RST,
  .tftPwr           = PIN_TFT_PWR,

  .pinBattMon       = PIN_BATT_MON,

  .i2cScl           = PIN_I2C_SCL,
  .i2cSda           = PIN_I2C_SDA,
  .touchInt         = PIN_TOUCH_INT,

  .sensorPins       = SENSOR_PINS
};

static BreathalyzerApp app;

// ============================= WATCHDOG ==================================
static constexpr int WDT_TIMEOUT_S = 15;

static void watchdogBegin() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  esp_task_wdt_init(&cfg);
  esp_task_wdt_add(nullptr);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(nullptr);
#endif
}

static inline void watchdogKick() {
  esp_task_wdt_reset();
}

// ============================= setup / loop ===============================
void setup() {
  watchdogBegin();
  #if defined(NEOPIXEL_I2C_POWER)
  gpio_hold_dis((gpio_num_t)NEOPIXEL_I2C_POWER);
#endif
  app.begin(HW);
}

void loop() {
  watchdogKick();
  app.update();
  delay(2);
}