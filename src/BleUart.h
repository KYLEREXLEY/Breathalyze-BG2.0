#pragma once
#include <Arduino.h>

class NimBLECharacteristic;

struct BleAppResponse {
  bool valid = false;
  int bg = -1;
  uint32_t timestamp = 0;
  bool hasTimestamp = false;
};

enum class BleClientMode {
  Unknown,
  Logan,
  Sudir
};

class BleUart {
public:
  BleUart() = default;

  void begin(const char* deviceName);

  bool consumeReadyToSend();
  bool consumeResponse(BleAppResponse& out);

  void sendTextPayload(const String& payload);
  void sendBinaryPacket(uint16_t deviceId, const uint16_t* values45);

  bool isInited() const { return _inited; }
  bool isConnected() const { return _connected; }
  bool shouldBroadcastBinary() const;
  BleClientMode mode() const { return _mode; }

private:
  friend class _BleUartRxCallbacks;
  friend class _BleUartServerCallbacks;

  void setReadyFlag();
  void setConnected(bool c);
  void setResponse(const BleAppResponse& r);

  void notifyChunked(const String& payload);

  bool _inited = false;
  bool _connected = false;
  bool _readyToSend = false;
  bool _responseReady = false;
  bool _broadcastBinary = true;

  uint32_t _connectedAtMs = 0;
  BleAppResponse _response{};
  BleClientMode _mode = BleClientMode::Unknown;

  NimBLECharacteristic* _txChar = nullptr;
};
