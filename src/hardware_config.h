#pragma once
#include <Arduino.h>
#include <array>

constexpr size_t NUM_SENSORS = 5;

struct HardwareConfig {
  // Button
  int  pinButton;
  bool buttonActiveLow;
  bool useInternalPulls;

  // Outputs
  int pinBuzzer;
  int pinHeaterEn;

  // TFT SPI pins
  int tftSclk;
  int tftMosi;
  int tftMiso;

  // TFT control pins
  int tftCs;
  int tftDc;
  int tftBl;   // backlight ACTIVE-LOW
  int tftRst;  // -1 if unused
  int tftPwr;  // -1 if unused

  // Battery
  int pinBattMon;

  // Touch / I2C
  int i2cScl;
  int i2cSda;
  int touchInt;

  // Sensors
  std::array<int, NUM_SENSORS> sensorPins;
};
