#pragma once
#ifndef ARMORLINK_TRANSPORT_ESPNOW_H
#define ARMORLINK_TRANSPORT_ESPNOW_H

#include "ArmorLinkDebug.h"
#if ARMORLINK_HAS_ESPNOW
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  extern "C" {
    #include <espnow.h>
    #include <user_interface.h>
  }
#elif defined(ESP32)
  #include <WiFi.h>
  #include <esp_now.h>
  #include <esp_wifi.h>
#endif
#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include <map>
#include "ArmorLinkProtocol.h"

using ArmorLinkManagedReceiveFn = void (*)(const uint8_t* data, int len);
using ArmorLinkManagedSendFn = void (*)(const uint8_t* mac_addr, esp_now_send_status_t status);

class ArmorLinkPeerRegistry {
public:
  void removePeer(const String& name) {
    if (!name.isEmpty()) _peers.erase(name);
  }

  void registerPeer(const String& name, const uint8_t mac[6]) {
    MacAddress addr{};
    memcpy(addr.bytes, mac, 6);
    _peers[name] = addr;
  }

  const uint8_t* getMacForTarget(const String& target) const {
    auto it = _peers.find(target);
    if (it == _peers.end()) return nullptr;
    return it->second.bytes;
  }

  String getNameForMac(const uint8_t* mac) const {
    for (const auto& kv : _peers) {
      if (memcmp(kv.second.bytes, mac, 6) == 0) {
        return kv.first;
      }
    }
    return "Unknown";
  }

private:
  struct MacAddress {
    uint8_t bytes[6];
  };

  std::map<String, MacAddress> _peers;
};

class ArmorLinkTransportEspNow {
public:
  static constexpr size_t DEFAULT_QUEUE_SIZE = ARMORLINK_ESPNOW_QUEUE_SIZE;
  static_assert(DEFAULT_QUEUE_SIZE >= 2, "ESP-NOW queue needs at least two slots");
  static constexpr uint16_t CONFIG_META_SETTLE_DELAY_MS = 200;
  static constexpr uint16_t CONFIG_CHUNK_DELAY_MS = 100;
  static constexpr uint16_t CONFIG_TRANSFER_CHUNK_DATA_SIZE = 140;

  void setManagedHandlers(ArmorLinkManagedReceiveFn receiveFn, ArmorLinkManagedSendFn sendFn) {
    _managedReceiveFn = receiveFn;
    _managedSendFn = sendFn;
  }

  bool begin(
    uint8_t channel = 1,
    esp_now_recv_cb_t onReceive = nullptr,
    esp_now_send_cb_t onSent = nullptr)
  {
    AL_VERBOSELN("?? Initialisiere ESP-NOW im Raw Mode");
#if defined(ESP8266)
    WiFi.mode(WIFI_AP);
    wifi_set_channel(channel);
    if (esp_now_init() != 0) return false;
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    if (onReceive) esp_now_register_recv_cb(onReceive);
    if (onSent) esp_now_register_send_cb(onSent);
    _channel = channel;
    return true;
#elif defined(ESP32)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (esp_now_init() != ESP_OK) return false;
    if (onReceive) esp_now_register_recv_cb(onReceive);
    if (onSent) esp_now_register_send_cb(onSent);
    _channel = channel;
    return true;
#else
    return false;
#endif
  }

  bool beginManaged(uint8_t channel = 1) {
    AL_VERBOSELN("?? Initialisiere ESP-NOW im Managed Mode");
#if defined(ESP32)

  WiFi.mode(WIFI_STA);

  WiFi.disconnect();

  WiFi.setSleep(false);

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_now_deinit();

  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

#endif
    return begin(channel, &ArmorLinkTransportEspNow::onReceiveStatic, &ArmorLinkTransportEspNow::onSentStatic);
  }

  void setPeerRegistry(ArmorLinkPeerRegistry* registry) {
    _registry = registry;
  }

  // WiFi callbacks must not print, allocate Strings, notify BLE or retry sends.
  void recordReceiveError(bool invalidPacket) {
    lockCallbacks();
    uint32_t& counter = invalidPacket ? _invalidReceivedPackets : _droppedReceivedPackets;
    if (counter != UINT32_MAX) ++counter;
    unlockCallbacks();
  }

  void processCallbacks() {
    uint32_t invalid, dropped, droppedStatuses;
    lockCallbacks();
    invalid = _invalidReceivedPackets;
    dropped = _droppedReceivedPackets;
    droppedStatuses = _droppedSendStatuses;
    _invalidReceivedPackets = _droppedReceivedPackets = _droppedSendStatuses = 0;
    unlockCallbacks();
    if (invalid || dropped || droppedStatuses) {
      AL_INTERNAL_PRINTF("[ESPNOW][WARN] invalidPackets=%lu rxQueueFull=%lu txStatusQueueFull=%lu\n",
        (unsigned long)invalid, (unsigned long)dropped, (unsigned long)droppedStatuses);
    }
    // Bound work even when processing a status causes another send.
    for (size_t i = 0; i < SEND_STATUS_QUEUE_SIZE; ++i) {
      PendingSendStatus event{};
      lockCallbacks();
      if (_sendStatusCount == 0) {
        unlockCallbacks();
        break;
      }
      event = _sendStatuses[_sendStatusTail];
      _sendStatusTail = (_sendStatusTail + 1) % SEND_STATUS_QUEUE_SIZE;
      --_sendStatusCount;
      unlockCallbacks();
      if (_managedSendFn) _managedSendFn(event.mac, event.status);
    }
  }

esp_err_t ensurePeer(const uint8_t* mac) {
  if (!mac) return ESP_ERR_NOT_FOUND;

#if defined(ESP32)
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = _channel;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;

  if (esp_now_is_peer_exist(mac)) {
    return ESP_OK;
  }

  esp_err_t addResult = esp_now_add_peer(&peer);
  if (addResult == ESP_OK) return ESP_OK;

  esp_now_del_peer(mac);
  delay(5);
  return esp_now_add_peer(&peer);

#elif defined(ESP8266)
  uint8_t macCopy[6];
  memcpy(macCopy, mac, 6);

  if (esp_now_is_peer_exist(macCopy)) return ESP_OK;

  return esp_now_add_peer(macCopy, ESP_NOW_ROLE_COMBO, _channel, nullptr, 0) == 0
    ? ESP_OK
    : ESP_FAIL;
#else
  return ESP_FAIL;
#endif
}
  bool enqueueReceivedPacket(const ArmorLinkPacket& msg) {
    lockCallbacks();
    size_t next = (_queueHead + 1) % DEFAULT_QUEUE_SIZE;
    if (next == _queueTail) {
      unlockCallbacks();
      return false;
    }
    _queue[_queueHead] = msg;
    _queueHead = next;
    unlockCallbacks();
    return true;
  }

  bool dequeueReceivedPacket(ArmorLinkPacket& msg) {
    lockCallbacks();
    if (_queueTail == _queueHead) {
      unlockCallbacks();
      return false;
    }
    msg = _queue[_queueTail];
    _queueTail = (_queueTail + 1) % DEFAULT_QUEUE_SIZE;
    unlockCallbacks();
    return true;
  }

  bool hasQueuedPackets() const {
    return _queueTail != _queueHead;
  }

  bool decodeIncomingPacket(const uint8_t* data, int len, ArmorLinkPacket& incoming) {
    if (len == (int)sizeof(ArmorLinkPacket)) {
      memcpy(&incoming, data, sizeof(incoming));
      return incoming.version == ARMORLINK_PROTOCOL_VERSION;
    }

    if (len == (int)sizeof(ArmorLinkLegacyPacket)) {
      ArmorLinkLegacyPacket oldMsg{};
      memcpy(&oldMsg, data, sizeof(oldMsg));
      return convertArmorLinkLegacyToPacket(oldMsg, incoming);
    }

    return false;
  }

  esp_err_t sendToMac(const uint8_t* mac, const void* data, size_t len) {
    if (!mac) return ESP_ERR_NOT_FOUND;

#if defined(ESP32)
    // Power saving is configured once in begin(). Do not reconfigure the
    // shared radio for every packet (especially while BLE is active).

    if (millis() - _lastChannelFix > 300000UL) {
      _lastChannelFix = millis();
      esp_wifi_set_channel(_channel, WIFI_SECOND_CHAN_NONE);
    }

    esp_err_t peerResult = ensurePeer(mac);

    if (peerResult != ESP_OK) {

      return peerResult;

    }

    esp_err_t sendResult = esp_now_send(mac, (const uint8_t*)data, len);
    if (sendResult == ESP_OK) {
      _lastSuccessfulSend = millis();
      _consecutiveSendFails = 0;
      return ESP_OK;
    }

    if (sendResult == ESP_ERR_ESPNOW_NOT_FOUND) {
      esp_now_del_peer(mac);
      delay(10);
      if (ensurePeer(mac) != ESP_OK) return ESP_ERR_ESPNOW_NOT_FOUND;
    }

    sendResult = esp_now_send(mac, (const uint8_t*)data, len);
    if (sendResult == ESP_OK) {
      _lastSuccessfulSend = millis();
      _consecutiveSendFails = 0;
      return ESP_OK;
    }

    _consecutiveSendFails++;
    if (_consecutiveSendFails >= 3) {
      esp_now_del_peer(mac);
      delay(10);
      ensurePeer(mac);
      _consecutiveSendFails = 0;
    }

    return sendResult;

#elif defined(ESP8266)
    uint8_t macCopy[6];
    memcpy(macCopy, mac, 6);

    if (!esp_now_is_peer_exist(macCopy)) {
      if (esp_now_add_peer(macCopy, ESP_NOW_ROLE_COMBO, _channel, nullptr, 0) != 0) {
        return ESP_FAIL;
      }
    }

    int result = esp_now_send(macCopy, (uint8_t*)data, len);
    return (result == 0) ? ESP_OK : ESP_FAIL;
#else
    return ESP_FAIL;
#endif
  }

  esp_err_t sendPacketToMac(const uint8_t* mac, const ArmorLinkPacket& packet) {
    return sendToMac(mac, &packet, sizeof(packet));
  }

  esp_err_t sendPacketToTarget(const String& target, const ArmorLinkPacket& packet) {
    if (_registry == nullptr) return ESP_ERR_NOT_FOUND;
    const uint8_t* mac = _registry->getMacForTarget(target);
    if (!mac) return ESP_ERR_NOT_FOUND;
    return sendPacketToMac(mac, packet);
  }

  esp_err_t sendLogToTarget(
    const String& target,
    const char* source,
    ArmorLinkLogLevel level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f
  ) {
    ArmorLinkPacket p = makeArmorLinkBasePacket(AL_MSG_LOG, source, target.c_str(), entity, command);
    p.level = level;
    p.valueInt = valueInt;
    p.valueFloat = valueFloat;
    p.batteryVoltage = batteryVoltage;
    setArmorLinkPacketPayload(p, message);
    return sendPacketToTarget(target, p);
  }

  esp_err_t sendLogToChest(
    const char* source,
    ArmorLinkLogLevel level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f
  ) {
    return sendLogToTarget("Chest", source, level, entity, command, message, valueInt, valueFloat, batteryVoltage);
  }

  esp_err_t sendConfigJsonChunkedToTarget(
    const String& target,
    const char* source,
    uint16_t requestId,
    const char* entity,
    const String& json,
    bool partial = false
  ) {
    const uint16_t totalLen = (uint16_t)json.length();
    const uint16_t chunkCount = (totalLen == 0) ? 1 : (uint16_t)((totalLen + CONFIG_TRANSFER_CHUNK_DATA_SIZE - 1) / CONFIG_TRANSFER_CHUNK_DATA_SIZE);

    if (chunkCount > ARMORLINK_MAX_CHUNKS) return ESP_ERR_INVALID_SIZE;

    ArmorLinkPacket meta = makeArmorLinkBasePacket(AL_MSG_CONFIG_META, source, target.c_str(), entity, "config_meta");
    meta.requestId = requestId;
    meta.chunkCount = chunkCount;
    meta.valueInt = totalLen;
    if (partial) meta.flags |= AL_FLAG_PARTIAL_CFG;

    esp_err_t result = sendPacketToTarget(target, meta);
    if (result != ESP_OK) return result;
    delay(CONFIG_META_SETTLE_DELAY_MS);

    for (uint16_t i = 0; i < chunkCount; i++) {
      const uint16_t start = i * CONFIG_TRANSFER_CHUNK_DATA_SIZE;
      const uint16_t len = min((uint16_t)CONFIG_TRANSFER_CHUNK_DATA_SIZE, (uint16_t)(totalLen - start));

      ArmorLinkPacket chunk = makeArmorLinkBasePacket(AL_MSG_CONFIG_CHUNK, source, target.c_str(), entity, "config_chunk");
      chunk.requestId = requestId;
      chunk.chunkIndex = i;
      chunk.chunkCount = chunkCount;
      chunk.payloadLen = len;
      if (partial) chunk.flags |= AL_FLAG_PARTIAL_CFG;
      memcpy(chunk.payload, json.c_str() + start, len);

      result = sendPacketToTarget(target, chunk);
      if (result != ESP_OK) return result;
      delay(CONFIG_CHUNK_DELAY_MS);
    }

    ArmorLinkPacket end = makeArmorLinkBasePacket(AL_MSG_CONFIG_END, source, target.c_str(), entity, "config_end");
    end.requestId = requestId;
    end.chunkCount = chunkCount;
    end.valueInt = totalLen;
    if (partial) end.flags |= AL_FLAG_PARTIAL_CFG;

    return sendPacketToTarget(target, end);
  }

  esp_err_t sendConfigJsonChunkedToMac(
    const uint8_t* mac,
    const char* target,
    const char* source,
    uint16_t requestId,
    const char* entity,
    const String& json,
    bool partial = false
  ) {
    const uint16_t totalLen = (uint16_t)json.length();
    const uint16_t chunkCount = (totalLen == 0) ? 1 : (uint16_t)((totalLen + CONFIG_TRANSFER_CHUNK_DATA_SIZE - 1) / CONFIG_TRANSFER_CHUNK_DATA_SIZE);

    if (chunkCount > ARMORLINK_MAX_CHUNKS) return ESP_ERR_INVALID_SIZE;

    ArmorLinkPacket meta = makeArmorLinkBasePacket(AL_MSG_CONFIG_META, source, target, entity, "config_meta");
    meta.requestId = requestId;
    meta.chunkCount = chunkCount;
    meta.valueInt = totalLen;
    if (partial) meta.flags |= AL_FLAG_PARTIAL_CFG;

    esp_err_t result = sendPacketToMac(mac, meta);
    if (result != ESP_OK) return result;
    delay(CONFIG_META_SETTLE_DELAY_MS);

    for (uint16_t i = 0; i < chunkCount; i++) {
      const uint16_t start = i * CONFIG_TRANSFER_CHUNK_DATA_SIZE;
      const uint16_t len = min((uint16_t)CONFIG_TRANSFER_CHUNK_DATA_SIZE, (uint16_t)(totalLen - start));

      ArmorLinkPacket chunk = makeArmorLinkBasePacket(AL_MSG_CONFIG_CHUNK, source, target, entity, "config_chunk");
      chunk.requestId = requestId;
      chunk.chunkIndex = i;
      chunk.chunkCount = chunkCount;
      chunk.payloadLen = len;
      if (partial) chunk.flags |= AL_FLAG_PARTIAL_CFG;
      memcpy(chunk.payload, json.c_str() + start, len);

      result = sendPacketToMac(mac, chunk);
      if (result != ESP_OK) return result;
      delay(CONFIG_CHUNK_DELAY_MS);
    }

    ArmorLinkPacket end = makeArmorLinkBasePacket(AL_MSG_CONFIG_END, source, target, entity, "config_end");
    end.requestId = requestId;
    end.chunkCount = chunkCount;
    end.valueInt = totalLen;
    if (partial) end.flags |= AL_FLAG_PARTIAL_CFG;

    return sendPacketToMac(mac, end);
  }

private:
  static constexpr size_t SEND_STATUS_QUEUE_SIZE = 8;
  struct PendingSendStatus {
    uint8_t mac[6];
    esp_now_send_status_t status;
  };
#if defined(ESP32)
  portMUX_TYPE _callbackMux = portMUX_INITIALIZER_UNLOCKED;
#endif
  PendingSendStatus _sendStatuses[SEND_STATUS_QUEUE_SIZE]{};
  size_t _sendStatusHead = 0, _sendStatusTail = 0, _sendStatusCount = 0;
  uint32_t _invalidReceivedPackets = 0, _droppedReceivedPackets = 0, _droppedSendStatuses = 0;

  void lockCallbacks() {
#if defined(ESP32)
    portENTER_CRITICAL(&_callbackMux);
#endif
  }
  void unlockCallbacks() {
#if defined(ESP32)
    portEXIT_CRITICAL(&_callbackMux);
#endif
  }
  void enqueueSendStatus(const uint8_t* mac, esp_now_send_status_t status) {
    if (!mac) return;
    lockCallbacks();
    if (_sendStatusCount == SEND_STATUS_QUEUE_SIZE) {
      if (_droppedSendStatuses != UINT32_MAX) ++_droppedSendStatuses;
    } else {
      PendingSendStatus& event = _sendStatuses[_sendStatusHead];
      memcpy(event.mac, mac, sizeof(event.mac));
      event.status = status;
      _sendStatusHead = (_sendStatusHead + 1) % SEND_STATUS_QUEUE_SIZE;
      ++_sendStatusCount;
    }
    unlockCallbacks();
  }
  ArmorLinkPeerRegistry* _registry = nullptr;
  uint8_t _channel = 1;
  unsigned long _lastChannelFix = 0;
  unsigned long _lastSuccessfulSend = 0;
  int _consecutiveSendFails = 0;

  ArmorLinkManagedReceiveFn _managedReceiveFn = nullptr;
  ArmorLinkManagedSendFn _managedSendFn = nullptr;

  ArmorLinkPacket _queue[DEFAULT_QUEUE_SIZE];
  volatile size_t _queueHead = 0;
  volatile size_t _queueTail = 0;

#if defined(ESP32)
  static void onReceiveStatic(const esp_now_recv_info_t* recvInfo, const uint8_t* data, int len) {
    (void)recvInfo;
    ArmorLinkTransportEspNow* self = instance();
    if (self && self->_managedReceiveFn) {
      self->_managedReceiveFn(data, len);
    }
  }

  static void onSentStatic(const uint8_t* mac_addr, esp_now_send_status_t status) {
    ArmorLinkTransportEspNow* self = instance();
    if (self) self->enqueueSendStatus(mac_addr, status);
  }
#elif defined(ESP8266)
  static void onReceiveStatic(uint8_t* mac, uint8_t* data, uint8_t len) {
    (void)mac;
    ArmorLinkTransportEspNow* self = instance();
    if (self && self->_managedReceiveFn) {
      self->_managedReceiveFn(data, (int)len);
    }
  }

  static void onSentStatic(uint8_t* mac_addr, uint8_t status) {
    ArmorLinkTransportEspNow* self = instance();
    if (self) self->enqueueSendStatus(mac_addr, (esp_now_send_status_t)status);
  }
#endif

  static ArmorLinkTransportEspNow*& instance() {
    static ArmorLinkTransportEspNow* s_instance = nullptr;
    return s_instance;
  }

public:
  ArmorLinkTransportEspNow() {
    instance() = this;
  }
};


#else
#include <Arduino.h>
#include <esp_err.h>
#include <esp_now.h> // callback types only; no ESP-NOW or WiFi implementation
#include "ArmorLinkProtocol.h"
using ArmorLinkManagedReceiveFn = void (*)(const uint8_t*, int);
using ArmorLinkManagedSendFn = void (*)(const uint8_t*, esp_now_send_status_t);
class ArmorLinkPeerRegistry {
public:
  void removePeer(const String&) {}
  void registerPeer(const String&, const uint8_t*) {}
  const uint8_t* getMacForTarget(const String&) const { return nullptr; }
  String getNameForMac(const uint8_t*) const { return "Unknown"; }
};
class ArmorLinkTransportEspNow {
public:
  void processCallbacks() {}
  void recordReceiveError(bool) {}
  static constexpr uint16_t CONFIG_TRANSFER_CHUNK_DATA_SIZE = ARMORLINK_CHUNK_DATA_SIZE;
  void setManagedHandlers(ArmorLinkManagedReceiveFn receiveFn, ArmorLinkManagedSendFn sendFn) {  }
  bool begin(
    uint8_t channel = 1,
    esp_now_recv_cb_t onReceive = nullptr,
    esp_now_send_cb_t onSent = nullptr) { return false; }
  bool beginManaged(uint8_t channel = 1) { return false; }
  void setPeerRegistry(ArmorLinkPeerRegistry* registry) {  }
  esp_err_t ensurePeer(const uint8_t* mac) { return ESP_ERR_NOT_SUPPORTED; }
  bool enqueueReceivedPacket(const ArmorLinkPacket& msg) { return false; }
  bool dequeueReceivedPacket(ArmorLinkPacket& msg) { return false; }
  bool hasQueuedPackets() const { return false; }
  bool decodeIncomingPacket(const uint8_t* data, int len, ArmorLinkPacket& incoming) { return false; }
  esp_err_t sendToMac(const uint8_t* mac, const void* data, size_t len) { return ESP_ERR_NOT_SUPPORTED; }
  esp_err_t sendPacketToMac(const uint8_t* mac, const ArmorLinkPacket& packet) { return ESP_ERR_NOT_SUPPORTED; }
  esp_err_t sendPacketToTarget(const String& target, const ArmorLinkPacket& packet) { return ESP_ERR_NOT_SUPPORTED; }
  esp_err_t sendLogToTarget(
    const String& target,
    const char* source,
    ArmorLinkLogLevel level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f
  ) { return ESP_ERR_NOT_SUPPORTED; }
  esp_err_t sendLogToChest(
    const char* source,
    ArmorLinkLogLevel level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f
  ) { return ESP_ERR_NOT_SUPPORTED; }
  esp_err_t sendConfigJsonChunkedToTarget(
    const String& target,
    const char* source,
    uint16_t requestId,
    const char* entity,
    const String& json,
    bool partial = false
  ) { return ESP_ERR_NOT_SUPPORTED; }
  esp_err_t sendConfigJsonChunkedToMac(
    const uint8_t* mac,
    const char* target,
    const char* source,
    uint16_t requestId,
    const char* entity,
    const String& json,
    bool partial = false
  ) { return ESP_ERR_NOT_SUPPORTED; }
};
#endif // ARMORLINK_HAS_ESPNOW
#endif
