#pragma once
#ifndef ARMORLINK_BLE_H
#define ARMORLINK_BLE_H

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include "ArmorLinkProtocol.h"
#include "ArmorLinkSecurity.h"



// ============================================================================
// UUIDS
// ============================================================================
#define ARMORLINK_SERVICE_UUID     "9b43f08c-6a1d-4dc4-872c-934d8b798207"
#define ARMORLINK_COMMAND_RX_UUID  "b6f8d8f2-4f3d-4d2c-98d3-9c91a2d0e101"
#define ARMORLINK_EVENT_TX_UUID    "b6f8d8f2-4f3d-4d2c-98d3-9c91a2d0e102"
#define ARMORLINK_LOG_TX_UUID      "b6f8d8f2-4f3d-4d2c-98d3-9c91a2d0e103"

using ArmorLinkBleConnectHook = void (*)();
using ArmorLinkBleDisconnectHook = void (*)();

inline ArmorLinkBleConnectHook g_armorLinkBleConnectHook = nullptr;
inline ArmorLinkBleDisconnectHook g_armorLinkBleDisconnectHook = nullptr;

inline void setBleConnectHook(ArmorLinkBleConnectHook hook) {
  g_armorLinkBleConnectHook = hook;
}

inline void setBleDisconnectHook(ArmorLinkBleDisconnectHook hook) {
  g_armorLinkBleDisconnectHook = hook;
}

class ArmorLinkBleCommandHandler {
public:
  ArmorLinkPacket packet;
  String rawJson;
  virtual void handleCommand() = 0;
  virtual ~ArmorLinkBleCommandHandler() = default;
};

class ArmorLinkBleRuntime {
public:
  void begin(const char* deviceName, ArmorLinkBleCommandHandler* handler) {
    _commandHandler = handler;

    BLEDevice::init(deviceName);

    if (ArmorLinkSecurity::isPinSet()) {
      Serial.println("?? Starte BLE im SECURE MODE");

      BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);

  BLEDevice::setSecurityCallbacks(new SecurityCallbackBridge());
  uint32_t pin = ArmorLinkSecurity::getPin();
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key  = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &pin, sizeof(pin));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key));
    } else {
      Serial.println("?? Starte BLE im SETUP MODE (ohne PIN)");
    }

    _server = BLEDevice::createServer();
    _server->setCallbacks(new ServerCallbackBridge(this));

    BLEService* service = _server->createService(ARMORLINK_SERVICE_UUID);

    _commandRxChar = service->createCharacteristic(
      ARMORLINK_COMMAND_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );

    if (ArmorLinkSecurity::isPinSet()) {
      _commandRxChar->setAccessPermissions(ESP_GATT_PERM_WRITE_ENCRYPTED);
    }

    _commandRxChar->setCallbacks(new CommandRxCallbackBridge(this));

    _eventTxChar = service->createCharacteristic(
      ARMORLINK_EVENT_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );

    if (ArmorLinkSecurity::isPinSet()) {
      _eventTxChar->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED);
    }

    _eventTxChar->addDescriptor(new BLE2902());

    _logTxChar = service->createCharacteristic(
      ARMORLINK_LOG_TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );

    if (ArmorLinkSecurity::isPinSet()) {
      _logTxChar->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED);
    }

    _logTxChar->addDescriptor(new BLE2902());

    service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(ARMORLINK_SERVICE_UUID);
    BLEDevice::startAdvertising();
  }

  // Wird in ArmorLink.h definiert, damit es keinen Include-Zirkel gibt.
  void begin(const char* deviceName, const char* localTarget);

  void setDefaultTarget(const char* target) {
    _defaultTarget = (target != nullptr) ? String(target) : String("");
  }

  bool isClientConnected() const {
    return _clientConnected;
  }

  bool isLogStreamEnabled() const {
    return _logStreamEnabled;
  }

  void setLogStreamEnabled(bool enabled) {
    _logStreamEnabled = enabled;
  }

  bool isPinSet() const {
    return ArmorLinkSecurity::isPinSet();
  }

  uint32_t getPin() const {
    return ArmorLinkSecurity::getPin();
  }

  bool savePin(uint32_t pin) {
    return ArmorLinkSecurity::savePin(pin);
  }

  void clearPin() {
    ArmorLinkSecurity::clearPin();
  }

  void notifyEventJson(const String& json) {
    if (!_eventTxChar || !_clientConnected) return;

    const uint32_t minGapMs = 15;
    const uint32_t now = millis();

    if (_lastEventNotifyMs != 0 && (uint32_t)(now - _lastEventNotifyMs) < minGapMs) {
      delay(minGapMs - (uint32_t)(now - _lastEventNotifyMs));
      yield();
    }

    _eventTxChar->setValue((uint8_t*)json.c_str(), json.length());
    _eventTxChar->notify();

    _lastEventNotifyMs = millis();
    yield();
  }

  void notifyLogJson(const String& json) {
    if (!_logTxChar || !_clientConnected || !_logStreamEnabled) return;
    _logTxChar->setValue((uint8_t*)json.c_str(), json.length());
    _logTxChar->notify();
  }

  void notifyConfigJsonChunked(const char* target, const char* entity, uint16_t requestId, const String& json, bool partial) {
    const uint16_t totalLen = (uint16_t)json.length();
    const uint16_t chunkSize = 120;
    const uint16_t chunkCount = (totalLen == 0) ? 1 : (uint16_t)((totalLen + chunkSize - 1) / chunkSize);

    const uint16_t metaDelayMs = 20;
    const uint16_t chunkDelayMs = 12;
    const uint16_t endDelayMs = 20;

    {
      StaticJsonDocument<256> doc;
      doc["type"] = "config_meta";
      doc["requestId"] = requestId;
      doc["target"] = target ? target : "";
      doc["entity"] = entity ? entity : "";
      doc["partial"] = partial;
      doc["totalLength"] = totalLen;
      doc["chunkCount"] = chunkCount;

      String out;
      serializeJson(doc, out);
      notifyEventJson(out);
      delay(metaDelayMs);
      yield();
    }

    for (uint16_t i = 0; i < chunkCount; i++) {
      const uint16_t start = i * chunkSize;
      const uint16_t len = min((uint16_t)chunkSize, (uint16_t)(totalLen - start));

      StaticJsonDocument<340> doc;
      doc["type"] = "config_chunk";
      doc["requestId"] = requestId;
      doc["target"] = target ? target : "";
      doc["entity"] = entity ? entity : "";
      doc["partial"] = partial;
      doc["chunkIndex"] = i;
      doc["chunkCount"] = chunkCount;
      doc["data"] = json.substring(start, start + len);

      String out;
      serializeJson(doc, out);
      notifyEventJson(out);
      delay(chunkDelayMs);
      yield();
    }

    delay(endDelayMs);
    yield();

    {
      StaticJsonDocument<224> doc;
      doc["type"] = "config_end";
      doc["requestId"] = requestId;
      doc["target"] = target ? target : "";
      doc["entity"] = entity ? entity : "";
      doc["partial"] = partial;

      String out;
      serializeJson(doc, out);
      notifyEventJson(out);
    }
  }

  ArmorLinkBleCommandHandler* commandHandler() {
    return _commandHandler;
  }

private:
  String _defaultTarget;
  BLEServer* _server = nullptr;
  BLECharacteristic* _commandRxChar = nullptr;
  BLECharacteristic* _eventTxChar = nullptr;
  BLECharacteristic* _logTxChar = nullptr;
  uint32_t _lastEventNotifyMs = 0;
  ArmorLinkBleCommandHandler* _commandHandler = nullptr;
  bool _clientConnected = false;
  bool _logStreamEnabled = false;

  void onConnected() {
    _clientConnected = true;
    Serial.println("?? BLE Client verbunden");

    if (g_armorLinkBleConnectHook != nullptr) {
      g_armorLinkBleConnectHook();
    }
  }

  void onDisconnected() {
    _clientConnected = false;
    _logStreamEnabled = false;
    Serial.println("?? BLE Client getrennt");  
    if (g_armorLinkBleDisconnectHook != nullptr) {
      g_armorLinkBleDisconnectHook();
    }  
    BLEDevice::startAdvertising();
  }

  void onCommandWritten(BLECharacteristic* characteristic) {
    String json = characteristic->getValue().c_str();
    if (json.isEmpty()) return;

    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
      StaticJsonDocument<256> outDoc;
      outDoc["type"] = "error";
      outDoc["message"] = "Invalid JSON";
      outDoc["detail"] = err.c_str();

      String out;
      serializeJson(outDoc, out);
      notifyEventJson(out);
      return;
    }

    if (!_commandHandler) {
      StaticJsonDocument<192> outDoc;
      outDoc["type"] = "error";
      outDoc["message"] = "No command handler registered";

      String out;
      serializeJson(outDoc, out);
      notifyEventJson(out);
      return;
    }

    armorLinkPacketFromJsonDocument(doc, _commandHandler->packet);
    _commandHandler->rawJson = json;
    _commandHandler->handleCommand();
  }

  void armorLinkPacketFromJsonDocument(JsonDocument& doc, ArmorLinkPacket& pkt) {
    clearArmorLinkPacket(pkt);
    pkt.seq = nextArmorLinkPacketSeq();

    String type    = String((const char*)(doc["type"] | "command"));
    const char* fallbackTarget = _defaultTarget.isEmpty() ? "" : _defaultTarget.c_str();
    String target  = String((const char*)(doc["target"] | fallbackTarget));
    String entity  = String((const char*)(doc["entity"] | ""));
    String command = String((const char*)(doc["command"] | ""));
    String source  = String((const char*)(doc["source"] | "App"));

    pkt.requestId = doc["requestId"] | 0;

    if (type.equalsIgnoreCase("command")) {
      pkt.msgType = AL_MSG_COMMAND;
    } else if (type.equalsIgnoreCase("config_get")) {
      pkt.msgType = AL_MSG_CONFIG_GET;
      if (command.isEmpty()) command = "get";
    } else if (type.equalsIgnoreCase("config_set")) {
      pkt.msgType = AL_MSG_CONFIG_SET;
      if (command.isEmpty()) command = "set";
    } else {
      pkt.msgType = AL_MSG_COMMAND;
    }

    armorlinkCopyString(pkt.source, sizeof(pkt.source), source);
    armorlinkCopyString(pkt.target, sizeof(pkt.target), target);
    armorlinkCopyString(pkt.entity, sizeof(pkt.entity), entity);
    armorlinkCopyString(pkt.command, sizeof(pkt.command), command);

    if (doc["partial"].is<bool>() && doc["partial"].as<bool>()) {
      pkt.flags |= AL_FLAG_PARTIAL_CFG;
    }

    if (doc["value"].is<const char*>()) {
      setArmorLinkPacketPayload(pkt, String((const char*)doc["value"]));
      pkt.valueInt = doc["value"] | 0;
      pkt.valueFloat = doc["value"] | 0.0f;
    } else if (doc["value"].is<int>()) {
      pkt.valueInt = doc["value"].as<int>();
      pkt.valueFloat = (float)pkt.valueInt;
      setArmorLinkPacketPayload(pkt, String(pkt.valueInt));
    } else if (doc["value"].is<float>()) {
      pkt.valueFloat = doc["value"].as<float>();
      pkt.valueInt = (int32_t)pkt.valueFloat;
      setArmorLinkPacketPayload(pkt, String(pkt.valueFloat, 3));
    } else if (doc["value"].is<bool>()) {
      bool b = doc["value"].as<bool>();
      pkt.valueInt = b ? 1 : 0;
      pkt.valueFloat = b ? 1.0f : 0.0f;
      setArmorLinkPacketPayload(pkt, b ? "true" : "false");
    }

    if (pkt.msgType == AL_MSG_CONFIG_SET && doc["config"].is<JsonVariant>()) {
      String configJson;
      serializeJson(doc["config"], configJson);
      setArmorLinkPacketPayload(pkt, configJson);
    }
  }

  class ServerCallbackBridge : public BLEServerCallbacks {
  public:
    explicit ServerCallbackBridge(ArmorLinkBleRuntime* runtime) : _runtime(runtime) {}
    void onConnect(BLEServer* server) override { _runtime->onConnected(); }
    void onDisconnect(BLEServer* server) override { _runtime->onDisconnected(); }
  private:
    ArmorLinkBleRuntime* _runtime;
  };

  class CommandRxCallbackBridge : public BLECharacteristicCallbacks {
  public:
    explicit CommandRxCallbackBridge(ArmorLinkBleRuntime* runtime) : _runtime(runtime) {}
    void onWrite(BLECharacteristic* characteristic) override { _runtime->onCommandWritten(characteristic); }
  private:
    ArmorLinkBleRuntime* _runtime;
  };

  class SecurityCallbackBridge : public BLESecurityCallbacks {

public:

uint32_t onPassKeyRequest() override {

  uint32_t pin = ArmorLinkSecurity::getPin();

  Serial.printf("[BLE] Passkey requested -> %06lu\n", (unsigned long)pin);

  return pin;

}

  void onPassKeyNotify(uint32_t pass_key) override {

    Serial.printf("[BLE] Passkey notify: %06lu\n", (unsigned long)pass_key);

  }
bool onConfirmPIN(uint32_t pass_key) override {

  uint32_t pin = ArmorLinkSecurity::getPin();

  Serial.printf("[BLE] Confirm PIN: received=%06lu expected=%06lu\n",

                (unsigned long)pass_key,

                (unsigned long)pin);

  return pass_key == pin;

}

  bool onSecurityRequest() override {

    return true;

  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {

    if (cmpl.success) {

      Serial.println("?? BLE Authentifizierung erfolgreich");

    } else {

      Serial.println("? BLE Authentifizierung fehlgeschlagen");

    }

  }

};
};

inline ArmorLinkBleRuntime ArmorLinkBLE;

// ============================================================================
// COMPAT HELPERS
// ============================================================================
inline bool isBleClientConnected() {
  return ArmorLinkBLE.isClientConnected();
}

inline bool isBleLogStreamEnabled() {
  return ArmorLinkBLE.isLogStreamEnabled();
}

inline void setBleLogStreamEnabled(bool enabled) {
  ArmorLinkBLE.setLogStreamEnabled(enabled);
}

inline void bleNotifyEventJson(const String& json) {
  ArmorLinkBLE.notifyEventJson(json);
}

inline void bleNotifyLogJson(const String& json) {
  ArmorLinkBLE.notifyLogJson(json);
}

inline void bleNotifyConfigJsonChunked(const char* target, const char* entity, uint16_t requestId, const String& json, bool partial) {
  ArmorLinkBLE.notifyConfigJsonChunked(target, entity, requestId, json, partial);
}

inline void setupBleServer(const char* deviceName, ArmorLinkBleCommandHandler* handler) {
  ArmorLinkBLE.begin(deviceName, handler);
}

#endif
