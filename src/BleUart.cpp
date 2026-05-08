#include "BleUart.h"
#include <NimBLEDevice.h>
#include <string>
#include <ctype.h>

static constexpr size_t TOTAL_VALUES = 45;
static constexpr size_t PACKET_SIZE = 2 + TOTAL_VALUES * 2;
static constexpr uint8_t ACK_VALUE = 0xAA;
static constexpr uint32_t UNKNOWN_TO_SUDIR_GRACE_MS = 1200;
//Logans android app
static NimBLEUUID SERVICE_UUID("6E400021-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID CHAR_RX_UUID("6E400022-B5A3-F393-E0A9-E50E24DCCA9E");
static NimBLEUUID CHAR_TX_UUID("6E400023-B5A3-F393-E0A9-E50E24DCCA9E");
//Sudirs ios app
//static NimBLEUUID SERVICE_UUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
//static NimBLEUUID CHAR_RX_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a9");
//static NimBLEUUID CHAR_TX_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static BleUart* s_bleSelf = nullptr;

static bool isMostlyPrintableAscii(const std::string& val) {
  if (val.empty()) return false;
  for (unsigned char ch : val) {
    if (ch == '\r' || ch == '\n' || ch == '\t') continue;
    if (ch < 32 || ch > 126) return false;
  }
  return true;
}

static bool parseAsciiResponse(const std::string& val, BleAppResponse& out) {
  size_t p = val.find("BG=");
  if (p == std::string::npos) return false;

  int bg = atoi(val.c_str() + p + 3);
  if (bg < 0) bg = 0;
  if (bg > 999) bg = 999;

  out = {};
  out.valid = true;
  out.bg = bg;

  const char* keys[] = {"TS=", "TIME=", "TIMESTAMP=", "&ts=", "&time=", "&timestamp="};
  for (const char* key : keys) {
    size_t tp = val.find(key);
    if (tp != std::string::npos) {
      unsigned long ts = strtoul(val.c_str() + tp + strlen(key), nullptr, 10);
      out.timestamp = (uint32_t)ts;
      out.hasTimestamp = (ts != 0);
      break;
    }
  }

  return true;
}

class _BleUartRxCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* c) { handle(c); }
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) { handle(c); }

private:
  void handle(NimBLECharacteristic* c) {
    if (!s_bleSelf) return;

    std::string val = c->getValue();

    Serial.printf("[BLE] RX write (%u bytes):", (unsigned)val.length());
    for (size_t i = 0; i < val.length(); i++) {
      Serial.printf(" %02X", (uint8_t)val[i]);
    }
    Serial.println();

    if (isMostlyPrintableAscii(val)) {
      Serial.print("[BLE] RX text: ");
      Serial.println(val.c_str());

      if (val.find("READY") != std::string::npos) {
        s_bleSelf->_mode = BleClientMode::Logan;
        s_bleSelf->_broadcastBinary = false;
        s_bleSelf->setReadyFlag();
        Serial.println("[BLE] Logan READY received");
        return;
      }

      BleAppResponse resp;
      if (parseAsciiResponse(val, resp)) {
        s_bleSelf->_mode = BleClientMode::Logan;
        s_bleSelf->_broadcastBinary = false;
        s_bleSelf->setResponse(resp);
        Serial.printf("[BLE] Logan BG received: %d\n", resp.bg);
        if (resp.hasTimestamp) {
          Serial.printf("[BLE] Logan timestamp received: %lu\n", (unsigned long)resp.timestamp);
        }
        return;
      }
    }

    if (val.length() == 1 && (uint8_t)val[0] == ACK_VALUE) {
      s_bleSelf->_mode = BleClientMode::Sudir;
      s_bleSelf->_broadcastBinary = false;
      Serial.println("[BLE] Sudir ACK received");
      return;
    }

    if (val.length() == 6) {
      uint16_t bg = (uint8_t)val[0] | ((uint8_t)val[1] << 8);
      uint32_t ts = (uint8_t)val[2]
                  | ((uint8_t)val[3] << 8)
                  | ((uint8_t)val[4] << 16)
                  | ((uint8_t)val[5] << 24);

      BleAppResponse resp;
      resp.valid = true;
      resp.bg = (int)bg;
      resp.timestamp = ts;
      resp.hasTimestamp = true;

      s_bleSelf->_mode = BleClientMode::Sudir;
      s_bleSelf->_broadcastBinary = false;
      s_bleSelf->setResponse(resp);

      Serial.printf("[BLE] Sudir BG received: %u timestamp=%lu\n",
                    (unsigned)bg, (unsigned long)ts);
      return;
    }

    if (!val.empty()) {
      s_bleSelf->_mode = BleClientMode::Sudir;
    }
  }
};

class _BleUartServerCallbacks : public NimBLEServerCallbacks {
public:
  void onConnect(NimBLEServer*) {
    if (s_bleSelf) s_bleSelf->setConnected(true);
    Serial.println("[BLE] Phone connected");
    NimBLEDevice::startAdvertising();
  }

  void onConnect(NimBLEServer*, NimBLEConnInfo&) {
    if (s_bleSelf) s_bleSelf->setConnected(true);
    Serial.println("[BLE] Phone connected");
    NimBLEDevice::startAdvertising();
  }

  void onDisconnect(NimBLEServer*) {
    if (s_bleSelf) s_bleSelf->setConnected(false);
    Serial.println("[BLE] Phone disconnected");
    NimBLEDevice::startAdvertising();
  }

  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) {
    if (s_bleSelf) s_bleSelf->setConnected(false);
    Serial.print("[BLE] Phone disconnected reason=");
    Serial.println(reason);
    NimBLEDevice::startAdvertising();
  }
};

void BleUart::setReadyFlag() {
  _readyToSend = true;
}

void BleUart::setConnected(bool c) {
  _connected = c;
  if (c) {
    _connectedAtMs = millis();
    _readyToSend = false;
    _responseReady = false;
    _broadcastBinary = true;
    _mode = BleClientMode::Unknown;
    _response = {};
  } else {
    _connectedAtMs = 0;
    _readyToSend = false;
    _responseReady = false;
    _broadcastBinary = true;
    _mode = BleClientMode::Unknown;
    _response = {};
  }
}

void BleUart::setResponse(const BleAppResponse& r) {
  _response = r;
  _responseReady = r.valid;
}

bool BleUart::consumeReadyToSend() {
  if (_readyToSend) {
    _readyToSend = false;
    return true;
  }
  return false;
}

bool BleUart::consumeResponse(BleAppResponse& out) {
  if (_responseReady) {
    _responseReady = false;
    out = _response;
    _response = {};
    return true;
  }
  return false;
}

bool BleUart::shouldBroadcastBinary() const {
  if (!_connected) return false;
  if (!_broadcastBinary) return false;
  if (_mode == BleClientMode::Sudir) return true;
  if (_mode == BleClientMode::Logan) return false;
  return (millis() - _connectedAtMs) >= UNKNOWN_TO_SUDIR_GRACE_MS;
}

void BleUart::begin(const char* deviceName) {
  if (_inited) return;

  s_bleSelf = this;

  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new _BleUartServerCallbacks());

  NimBLEService* service = server->createService(SERVICE_UUID);

  _txChar = service->createCharacteristic(
    CHAR_TX_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  NimBLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new _BleUartRxCallbacks());

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("[BLE] Advertising...");
  _inited = true;
}

void BleUart::notifyChunked(const String& payload) {
  if (!_txChar) return;

  const size_t CHUNK = 160;

  if (payload.length() <= CHUNK) {
    _txChar->setValue((uint8_t*)payload.c_str(), payload.length());
    _txChar->notify();
    return;
  }

  for (size_t i = 0; i < payload.length(); i += CHUNK) {
    size_t end = i + CHUNK;
    if (end > payload.length()) end = payload.length();

    String part = payload.substring(i, end);
    _txChar->setValue((uint8_t*)part.c_str(), part.length());
    _txChar->notify();
    delay(30);
  }
}

void BleUart::sendTextPayload(const String& payload) {
  if (!_inited || !_txChar) return;

  Serial.print("[BLE] Notifying text payload, len=");
  Serial.println(payload.length());

  notifyChunked(payload);
}

void BleUart::sendBinaryPacket(uint16_t deviceId, const uint16_t* values45) {
  if (!_inited || !_txChar || values45 == nullptr) return;

  uint8_t packet[PACKET_SIZE] = {0};
  packet[0] = deviceId & 0xFF;
  packet[1] = (deviceId >> 8) & 0xFF;

  for (size_t i = 0; i < TOTAL_VALUES; i++) {
    size_t off = 2 + i * 2;
    packet[off] = values45[i] & 0xFF;
    packet[off + 1] = (values45[i] >> 8) & 0xFF;
  }

  Serial.printf("[BLE] Notifying Sudir packet (%u bytes)\n", (unsigned)sizeof(packet));
  _txChar->setValue(packet, sizeof(packet));
  _txChar->notify();
}
