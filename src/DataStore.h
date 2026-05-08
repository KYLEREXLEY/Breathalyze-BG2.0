#pragma once
#include <Arduino.h>

struct SavedRun {
  uint32_t magic;     // 'BGBT'
  uint16_t version;   // 2
  uint16_t size;      // sizeof(SavedRun)
  uint32_t seq;       // increments each save
  int16_t  bg;        // -1 if not available
  uint8_t  hasAppTimestamp;
  uint8_t  reserved0;
  uint32_t appTimestamp; // unix seconds when provided by app

  float base[5][3];       // 15
  float res[2][5][3];     // 30

  uint32_t crc32;     // crc of everything before crc32
};

namespace DataStore {
  // Existing (legacy last-run blob)
  bool save(const SavedRun& r);
  bool load(SavedRun& out);
  void erase();

  // Persistent payload history stored in flash as a fixed-size ring buffer.
  // Only the API payload is stored (no BASE_URL prefix).
  static constexpr uint16_t MAX_SAVED_TESTS = 1000;
  static constexpr size_t   MAX_PAYLOAD_LEN = 480;

  uint32_t lastQuerySeq();
  bool appendQuery(const char* query, uint32_t& outSeq);
  bool updateQuery(uint32_t seq, const char* query);
  bool dumpAllQueriesToSerial();
}
