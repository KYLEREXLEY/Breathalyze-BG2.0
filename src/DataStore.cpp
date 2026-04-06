#include "DataStore.h"
#include <Preferences.h>
#include <stddef.h>   // for offsetof

// ===================== Existing "SavedRun" blob storage =====================
static constexpr uint32_t MAGIC_BGBT = 0x54474242; // 'B''G''B''T'
static constexpr uint16_t VER = 2;

static uint32_t crc32_simple(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

bool DataStore::save(const SavedRun& in) {
  Preferences prefs;
  if (!prefs.begin("bgbt", false)) return false;

  SavedRun r = in;
  r.magic = MAGIC_BGBT;
  r.version = VER;
  r.size = (uint16_t)sizeof(SavedRun);

  uint32_t seq = prefs.getUInt("seq_blob", 0);
  r.seq = seq + 1;

  r.crc32 = 0;
  r.crc32 = crc32_simple((const uint8_t*)&r, offsetof(SavedRun, crc32));

  size_t wrote = prefs.putBytes("last_blob", &r, sizeof(SavedRun));
  prefs.putUInt("seq_blob", r.seq);
  prefs.end();

  return wrote == sizeof(SavedRun);
}

bool DataStore::load(SavedRun& out) {
  Preferences prefs;
  if (!prefs.begin("bgbt", true)) return false;

  size_t len = prefs.getBytesLength("last_blob");
  if (len != sizeof(SavedRun)) { prefs.end(); return false; }

  size_t got = prefs.getBytes("last_blob", &out, sizeof(SavedRun));
  prefs.end();
  if (got != sizeof(SavedRun)) return false;

  if (out.magic != MAGIC_BGBT) return false;
  if (out.version != VER) return false;
  if (out.size != sizeof(SavedRun)) return false;

  uint32_t want = crc32_simple((const uint8_t*)&out, offsetof(SavedRun, crc32));
  return want == out.crc32;
}

void DataStore::erase() {
  Preferences prefs;
  if (!prefs.begin("bgbt", false)) return;
  prefs.clear();
  prefs.end();
}

// ===================== NEW: Append-only query log (no BaseURL) =====================
// Stored as:
//   seq_q = last seq counter
//   q000001, q000002, ... each holds the query string
static const char* NS = "bgbt";

static void makeKey(uint32_t seq, char key[12]) {
  snprintf(key, 12, "q%06lu", (unsigned long)seq);
}

uint32_t DataStore::lastQuerySeq() {
  Preferences prefs;
  if (!prefs.begin(NS, true)) return 0;
  uint32_t seq = prefs.getUInt("seq_q", 0);
  prefs.end();
  return seq;
}

bool DataStore::appendQuery(const char* query, uint32_t& outSeq) {
  Preferences prefs;
  if (!prefs.begin(NS, false)) return false;

  uint32_t seq = prefs.getUInt("seq_q", 0) + 1;
  char key[12]; makeKey(seq, key);

  size_t wrote = prefs.putString(key, query);
  if (wrote == 0) { prefs.end(); return false; }

  prefs.putUInt("seq_q", seq);
  prefs.end();

  outSeq = seq;
  return true;
}

bool DataStore::updateQuery(uint32_t seq, const char* query) {
  if (seq == 0) return false;

  Preferences prefs;
  if (!prefs.begin(NS, false)) return false;

  char key[12]; makeKey(seq, key);
  size_t wrote = prefs.putString(key, query);

  prefs.end();
  return wrote != 0;
}

bool DataStore::dumpAllQueriesToSerial() {
  Preferences prefs;
  if (!prefs.begin(NS, true)) return false;

  uint32_t maxSeq = prefs.getUInt("seq_q", 0);
  Serial.printf("[FLASH] Dump start. maxSeq=%lu\n", (unsigned long)maxSeq);

  for (uint32_t seq = 1; seq <= maxSeq; seq++) {
    char key[12]; makeKey(seq, key);
    String q = prefs.getString(key, "");
    if (q.length() == 0) continue;

    Serial.printf("SEQ=%lu ", (unsigned long)seq);
    Serial.println(q);
    delay(1);
  }

  Serial.println("[FLASH] Dump end.");
  prefs.end();
  return true;
}