#include "DataStore.h"
#include <Preferences.h>
#include <LittleFS.h>
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

// ===================== Fixed-size payload history in SPIFFS =====================
namespace {
  static constexpr const char* HISTORY_PATH = "/bgbt_history.bin";
  static constexpr uint32_t HISTORY_MAGIC = 0x48544742; // 'BGTH'
  static constexpr uint16_t HISTORY_VERSION = 1;

  struct HistoryHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t capacity;
    uint16_t count;
    uint16_t startIndex;   // oldest record index
    uint16_t recordSize;
    uint16_t reserved;
    uint32_t nextSeq;      // next sequence number to allocate
  };

  struct HistoryRecord {
    uint32_t seq;
    uint16_t len;
    uint16_t reserved;
    char payload[DataStore::MAX_PAYLOAD_LEN];
  };

  static_assert(sizeof(HistoryRecord) <= 512, "HistoryRecord unexpectedly large");

bool mountFs() {
  static bool mounted = false;
  if (mounted) return true;

  if (!LittleFS.begin(true)) {
    Serial.println("[FLASH] LittleFS mount failed");
    return false;
  }

  mounted = true;
  Serial.printf("[FLASH] LittleFS mounted. total=%u used=%u free=%u\n",
                (unsigned)LittleFS.totalBytes(),
                (unsigned)LittleFS.usedBytes(),
                (unsigned)(LittleFS.totalBytes() - LittleFS.usedBytes()));
  return true;
}

  bool readHeader(File& f, HistoryHeader& hdr) {
    if (!f.seek(0, SeekSet)) return false;
    return f.read((uint8_t*)&hdr, sizeof(hdr)) == sizeof(hdr);
  }

  bool writeHeader(File& f, const HistoryHeader& hdr) {
    if (!f.seek(0, SeekSet)) return false;
    return f.write((const uint8_t*)&hdr, sizeof(hdr)) == sizeof(hdr);
  }

  size_t recordOffset(uint16_t index) {
    return sizeof(HistoryHeader) + (size_t)index * sizeof(HistoryRecord);
  }

  bool readRecord(File& f, uint16_t index, HistoryRecord& rec) {
    if (!f.seek(recordOffset(index), SeekSet)) return false;
    return f.read((uint8_t*)&rec, sizeof(rec)) == sizeof(rec);
  }

  bool writeRecord(File& f, uint16_t index, const HistoryRecord& rec) {
    if (!f.seek(recordOffset(index), SeekSet)) return false;
    return f.write((const uint8_t*)&rec, sizeof(rec)) == sizeof(rec);
  }

  bool initHistoryFile() {
    File f = LittleFS.open(HISTORY_PATH, FILE_WRITE);
    if (!f) return false;

    HistoryHeader hdr{};
    hdr.magic = HISTORY_MAGIC;
    hdr.version = HISTORY_VERSION;
    hdr.capacity = DataStore::MAX_SAVED_TESTS;
    hdr.count = 0;
    hdr.startIndex = 0;
    hdr.recordSize = sizeof(HistoryRecord);
    hdr.nextSeq = 1;

    bool ok = writeHeader(f, hdr);
    f.close();
    return ok;
  }

  bool ensureHistoryFile(HistoryHeader& hdr, File& f, const char* mode) {
    if (!mountFs()) return false;

    f = LittleFS.open(HISTORY_PATH, mode);
    if (!f) {
      if (!initHistoryFile()) return false;
      f = LittleFS.open(HISTORY_PATH, mode);
      if (!f) return false;
    }

    if (!readHeader(f, hdr) ||
        hdr.magic != HISTORY_MAGIC ||
        hdr.version != HISTORY_VERSION ||
        hdr.capacity != DataStore::MAX_SAVED_TESTS ||
        hdr.recordSize != sizeof(HistoryRecord) ||
        hdr.count > hdr.capacity ||
        hdr.startIndex >= hdr.capacity ||
        hdr.nextSeq == 0) {
      f.close();
      LittleFS.remove(HISTORY_PATH);
      if (!initHistoryFile()) return false;
      f = LittleFS.open(HISTORY_PATH, mode);
      if (!f) return false;
      if (!readHeader(f, hdr)) return false;
    }

    return true;
  }

  bool makeRecord(const char* query, uint32_t seq, HistoryRecord& rec) {
    if (!query) return false;

    size_t len = strnlen(query, DataStore::MAX_PAYLOAD_LEN);
    if (len == 0 || len >= DataStore::MAX_PAYLOAD_LEN) {
      Serial.printf("[FLASH] Payload too large (%u bytes, max %u)\n",
                    (unsigned)strlen(query),
                    (unsigned)(DataStore::MAX_PAYLOAD_LEN - 1));
      return false;
    }

    memset(&rec, 0, sizeof(rec));
    rec.seq = seq;
    rec.len = (uint16_t)len;
    memcpy(rec.payload, query, len);
    rec.payload[len] = 0;
    return true;
  }
}

void DataStore::erase() {
  Preferences prefs;
  if (prefs.begin("bgbt", false)) {
    prefs.clear();
    prefs.end();
  }

  if (mountFs()) {
    LittleFS.remove(HISTORY_PATH);
  }
}

uint32_t DataStore::lastQuerySeq() {
  HistoryHeader hdr{};
  File f;
  if (!ensureHistoryFile(hdr, f, "r")) return 0;
  f.close();
  return hdr.nextSeq - 1;
}

bool DataStore::appendQuery(const char* query, uint32_t& outSeq) {
  outSeq = 0;

  HistoryHeader hdr{};
  File f;
  if (!ensureHistoryFile(hdr, f, "r+")) return false;

  const uint32_t seq = hdr.nextSeq;
  HistoryRecord rec{};
  if (!makeRecord(query, seq, rec)) {
    f.close();
    return false;
  }

  uint16_t writeIndex = 0;
  if (hdr.count < hdr.capacity) {
    writeIndex = (hdr.startIndex + hdr.count) % hdr.capacity;
    hdr.count++;
  } else {
    writeIndex = hdr.startIndex;
    hdr.startIndex = (hdr.startIndex + 1) % hdr.capacity;
  }

  if (!writeRecord(f, writeIndex, rec)) {
    f.close();
    return false;
  }

  hdr.nextSeq = seq + 1;
  if (!writeHeader(f, hdr)) {
    f.close();
    return false;
  }

  f.flush();
  f.close();

  outSeq = seq;
  return true;
}

bool DataStore::updateQuery(uint32_t seq, const char* query) {
  if (seq == 0) return false;

  HistoryHeader hdr{};
  File f;
  if (!ensureHistoryFile(hdr, f, "r+")) return false;

  HistoryRecord updated{};
  if (!makeRecord(query, seq, updated)) {
    f.close();
    return false;
  }

  for (uint16_t i = 0; i < hdr.count; i++) {
    const uint16_t index = (hdr.startIndex + i) % hdr.capacity;
    HistoryRecord rec{};
    if (!readRecord(f, index, rec)) continue;
    if (rec.seq != seq) continue;

    const bool ok = writeRecord(f, index, updated);
    f.flush();
    f.close();
    return ok;
  }

  f.close();
  return false;
}

bool DataStore::dumpAllQueriesToSerial() {
  HistoryHeader hdr{};
  File f;
  if (!ensureHistoryFile(hdr, f, "r")) return false;

  Serial.printf("[FLASH] Dump start. count=%u capacity=%u newestSeq=%lu\n",
                (unsigned)hdr.count,
                (unsigned)hdr.capacity,
                (unsigned long)(hdr.nextSeq - 1));

  for (uint16_t i = 0; i < hdr.count; i++) {
    const uint16_t index = (hdr.startIndex + i) % hdr.capacity;
    HistoryRecord rec{};
    if (!readRecord(f, index, rec)) continue;
    if (rec.seq == 0 || rec.len == 0 || rec.len >= DataStore::MAX_PAYLOAD_LEN) continue;

    rec.payload[DataStore::MAX_PAYLOAD_LEN - 1] = 0;
    Serial.printf("SEQ=%lu ", (unsigned long)rec.seq);
    Serial.println(rec.payload);
    delay(1);
  }

  Serial.println("[FLASH] Dump end.");
  f.close();
  return true;
}
