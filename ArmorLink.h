#pragma once
#ifndef ARMORLINK_H
#define ARMORLINK_H
#include "ArmorLinkDebug.h"
#include <ArduinoJson.h>
#if defined(ESP32) && ARMORLINK_HAS_ESPNOW
  #include <esp_wifi.h>
#endif
#if defined(ESP32) && !ARMORLINK_HAS_ESPNOW
  #include <esp_mac.h>  // localMacString() eFuse fallback
#endif
#include "ArmorLinkModule.h"
#include "ArmorLinkStorage.h"
#include "ArmorLinkDescriptor.h"
#include "ArmorLinkDispatch.h"
#include "ArmorLinkProtocol.h"
#include "ArmorLinkTransportEspNow.h"
#include "ArmorLinkBLE.h"
#include "ArmorLinkSecurity.h"
#include <esp_heap_caps.h>

#if ARMORLINK_HAS_WEB_SERIAL && __has_include(<ESP32Servo.h>)
  #include <ESP32Servo.h>
  #define ARMORLINK_HAS_ESP32_SERVO 1
#else
  #define ARMORLINK_HAS_ESP32_SERVO 0
#endif

struct ArmorLinkOptions {
  String nodeName = "";
  String bleName = "ArmorLink";
  uint8_t espNowChannel = 1;
  bool enableBle = false;
  bool enableEspNow = ARMORLINK_HAS_ESPNOW;
  bool enableGateway = false;
  bool enableSerialLogging = true;
  bool syncLoggingStateOnModuleOnline = false;
  bool requestStateSyncAfterBoot = true;
  uint32_t startupStateSyncDelayMs = 5000;
  uint32_t startupStateSyncRetryMs = 10000;
  String nvsNamespace = "armorlink";
  uint32_t moduleHeartbeatIntervalMs = 30000; // 30 sec
  bool forceRemoteLogging = false;
  uint32_t remoteLogMinIntervalMs = 100;
  // Temporr noch ntig, bis Pairing/Gateway-Routing fertig ist.
  // Danach kann das intern ersetzt werden.
  String defaultLogTarget = "";
  bool enableSerialCommands = true;
  bool enableSerialMenu = false;
  uint32_t modulePresenceCheckIntervalMs = 300000; // 5 min
  uint32_t moduleTimeoutMs = 900000;               // 15 min
};

using ArmorLinkEspNowCommandHook = bool (*)(const ArmorLinkPacket&);
using ArmorLinkModuleTimeoutHook = void (*)(const ArmorLinkStoredPairedModule&, uint32_t silentMs);

struct ArmorLinkPairingCandidate {
  bool occupied = false;
  uint16_t sessionId = 0;
  uint32_t lastSeenMs = 0;
  char name[16] = { 0 };
  char type[16] = { 0 };
  char mac[18] = { 0 };
  char moduleVersion[16] = "1.0";
  char armorLinkVersion[16] = { 0 };
};

struct ArmorLinkModulePresenceState {
  bool occupied = false;
  bool isOnline = false;
  bool timeoutReported = false;
  uint32_t lastSeenMs = 0;
  char mac[18] = { 0 };
};
class ArmorLinkRuntime;

class ArmorLinkDataStream {
public:
  ArmorLinkDataStream(const char* source,
                      const char* stream,
                      const char* target,
                      const String& payload)
    : _source(source ? source : ""),
      _stream(stream ? stream : ""),
      _target(target ? target : ""),
      _payload(payload) {}

  const String& source() const { return _source; }
  const String& stream() const { return _stream; }
  const String& target() const { return _target; }
  const String& payload() const { return _payload; }

  float getFloat(const char* key, float fallback = 0.0f) const {
    StaticJsonDocument<384> doc;
    if (deserializeJson(doc, _payload) != DeserializationError::Ok) {
      return fallback;
    }
    return doc[key] | fallback;
  }

  int getInt(const char* key, int fallback = 0) const {
    StaticJsonDocument<384> doc;
    if (deserializeJson(doc, _payload) != DeserializationError::Ok) {
      return fallback;
    }
    return doc[key] | fallback;
  }

  bool getBool(const char* key, bool fallback = false) const {
    StaticJsonDocument<384> doc;
    if (deserializeJson(doc, _payload) != DeserializationError::Ok) {
      return fallback;
    }
    return doc[key] | fallback;
  }

private:
  String _source;
  String _stream;
  String _target;
  String _payload;
};

using ArmorLinkDataStreamHandler = std::function<void(const ArmorLinkDataStream&)>;

class ArmorLinkDataStreamBuilder {
public:
  ArmorLinkDataStreamBuilder(ArmorLinkRuntime* rt, const char* stream)
    : _rt(rt) {
    armorlinkCopyString(_stream, sizeof(_stream), stream ? stream : "");
  }

  ArmorLinkDataStreamBuilder& target(const char* target) {
    armorlinkCopyString(_target, sizeof(_target), target ? target : "");
    return *this;
  }

  ArmorLinkDataStreamBuilder& value(const char* key, float v) {
    setFloatValue(key, v);
    return *this;
  }

  ArmorLinkDataStreamBuilder& value(const char* key, int v) {
    setIntValue(key, v);
    return *this;
  }

  ArmorLinkDataStreamBuilder& value(const char* key, bool v) {
    setBoolValue(key, v);
    return *this;
  }

  ArmorLinkDataStreamBuilder& payload(const String& payload) {
    _rawPayload = payload;
    return *this;
  }

  ArmorLinkDataStreamBuilder& interval(uint32_t intervalMs) {
    _intervalMs = intervalMs;
    return *this;
  }

  esp_err_t send();

private:
  enum class ValueType : uint8_t {
    Float,
    Int,
    Bool
  };

  struct ValueEntry {
    char key[ARMORLINK_COMMAND_MAX_LEN + 1] = { 0 };
    ValueType type = ValueType::Float;
    float floatValue = 0.0f;
    int intValue = 0;
    bool boolValue = false;
  };

  static constexpr size_t MAX_VALUES = ARMORLINK_MAX_DATA_STREAM_VALUES;

  bool setFloatValue(const char* key, float value) {
    ValueEntry* entry = valueEntry(key);
    if (entry == nullptr) return false;
    entry->type = ValueType::Float;
    entry->floatValue = value;
    return true;
  }

  bool setIntValue(const char* key, int value) {
    ValueEntry* entry = valueEntry(key);
    if (entry == nullptr) return false;
    entry->type = ValueType::Int;
    entry->intValue = value;
    return true;
  }

  bool setBoolValue(const char* key, bool value) {
    ValueEntry* entry = valueEntry(key);
    if (entry == nullptr) return false;
    entry->type = ValueType::Bool;
    entry->boolValue = value;
    return true;
  }

  ValueEntry* valueEntry(const char* key) {
    if (key == nullptr || key[0] == '\0') {
      return nullptr;
    }

    for (size_t i = 0; i < _valueCount; ++i) {
      if (strcasecmp(_values[i].key, key) == 0) {
        return &_values[i];
      }
    }

    if (_valueCount >= MAX_VALUES) {
      return nullptr;
    }

    ValueEntry& entry = _values[_valueCount++];
    armorlinkCopyString(entry.key, sizeof(entry.key), key);
    return &entry;
  }

  ArmorLinkRuntime* _rt = nullptr;
  char _stream[ARMORLINK_COMMAND_MAX_LEN + 1] = { 0 };
  char _target[ARMORLINK_NAME_MAX_LEN + 1] = "*";
  String _rawPayload;
  uint32_t _intervalMs = 0;
  ValueEntry _values[MAX_VALUES]{};
  size_t _valueCount = 0;
};

class ArmorLinkTelemetryBuilder {
public:
  ArmorLinkTelemetryBuilder(ArmorLinkRuntime* rt, const char* group, const char* name)
    : _rt(rt), _group(group), _name(name) {}

  ArmorLinkTelemetryBuilder& value(const char* key, float v) {
    _values[key] = v;
    return *this;
  }

  ArmorLinkTelemetryBuilder& unit(const char* u) {
    _unit = u;
    return *this;
  }

  esp_err_t send();

private:
  ArmorLinkRuntime* _rt = nullptr;
  String _group;
  String _name;
  String _unit;
  std::map<String, float> _values;
};

class ArmorLinkRuntime {
public:
  ArmorLinkRuntime() {
    instance() = this;
  }

  void begin(ArmorLinkModule& module) {
    ArmorLinkOptions defaults;
    begin(module, defaults);
  }

  uint8_t remoteLogLevel() const {
    return _remoteLogLevel;
  }

  static const char* logLevelToKeyword(uint8_t level) {
    switch (level) {
      case AL_LOG_DEBUG: return "DEBUG";
      case AL_LOG_INFO:  return "INFO";
      case AL_LOG_WARN:  return "WARN";
      case AL_LOG_ERROR: return "ERROR";
      default:           return "INFO";
    }
  }

  static uint8_t logLevelFromKeyword(const String& text) {
    if (text.equalsIgnoreCase("DEBUG")) return AL_LOG_DEBUG;
    if (text.equalsIgnoreCase("INFO"))  return AL_LOG_INFO;
    if (text.equalsIgnoreCase("WARN"))  return AL_LOG_WARN;
    if (text.equalsIgnoreCase("ERROR")) return AL_LOG_ERROR;
    return AL_LOG_INFO;
  }

  void setModuleTimeoutHook(ArmorLinkModuleTimeoutHook hook) {
    _moduleTimeoutHook = hook;
  }

  bool isModuleOnline(const String& macText) const {
#if ARMORLINK_HAS_ESPNOW
    int index = findPresenceIndexByMac(macText);
    if (index < 0) {
      return false;
    }
    return _presenceStates[index].isOnline;
#else
    return false;
#endif
  }

  void begin(ArmorLinkModule& module, const ArmorLinkOptions& options) {
    _module = &module;
    _options = options;
    initializeSystemConfig(module, _options);
    registerSystemConfig(module);
    registerSystemActions(module);

    _storage.begin(resolveNamespace(module, options));
    _storage.load(module);
#if ARMORLINK_HAS_ACTIONBUILDER
    _storage.loadActionDefinitions(module);
#endif
    applySystemConfig();

#if ARMORLINK_HAS_BLE
    if(_options.enableBle){
      ArmorLinkBLE.begin(_options.bleName.c_str(), _module->name().c_str());
    }
#endif
    setGatewayMode(_options.enableGateway);

    String storedProfileName;
    String storedProfileIdentifier;
    _storage.loadProfileIdentifier(storedProfileIdentifier);
    if (_storage.loadProfileName(storedProfileName) &&
        !storedProfileName.isEmpty()) {
      module.activeProfile(storedProfileIdentifier, storedProfileName);
    }
#if ARMORLINK_HAS_ESPNOW
    _storage.loadPairingInfo(_pairingInfo);
    _pairedModuleCount = _storage.loadPairedModules(_pairedModules);
    syncPresenceStatesFromStoredModules();

    // Restore the registry before starting the managed transport below.
    _transport.setPeerRegistry(&_peers);

    for (size_t i = 0; i < _pairedModuleCount; ++i) {
      uint8_t mac[6];
      if (armorLinkParseMacString(String(_pairedModules[i].mac), mac)) {
        _peers.registerPeer(String(_pairedModules[i].name), mac);
        AL_INTERNAL_PRINTF("[PAIR][GW] Restored paired peer: %s (%s)\n",
                      _pairedModules[i].name,
                      _pairedModules[i].mac);
      }
    }

    if (_pairingInfo.paired &&
        strlen(_pairingInfo.gatewayName) > 0 &&
        strlen(_pairingInfo.gatewayMac) > 0) {
      uint8_t gatewayMac[6];
      if (armorLinkParseMacString(String(_pairingInfo.gatewayMac), gatewayMac)) {
        String gatewayName = String(_pairingInfo.gatewayName);
        _peers.registerPeer(gatewayName, gatewayMac);
        if (_options.defaultLogTarget.isEmpty()) {
          _options.defaultLogTarget = gatewayName;
        }
        AL_INTERNAL_PRINTF("[PAIR] Restored gateway: %s (%s)\n",
                      gatewayName.c_str(),
                      _pairingInfo.gatewayMac);
      } else {
        AL_VERBOSELN("[PAIR] ERROR: Failed to parse stored gateway MAC");
      }
    } else {
      AL_VERBOSELN("[PAIR] No valid pairing info found");
    }

    if (_options.defaultLogTarget.isEmpty()) {
      AL_VERBOSELN("[PAIR] No defaultLogTarget configured; remote logging/telemetry waits for a paired gateway");
    }
#endif
    if (_options.enableSerialCommands ||
      _options.enableSerialMenu) {
      _serialRuntime.begin(
          Serial,
          _module->serial()
      );

#if ARMORLINK_HAS_WEB_SERIAL
      _serialRuntime.setInternalHandler(
          &ArmorLinkRuntime::handleFlasherSerialPayloadThunk,
          this
      );
#endif
      _serialRuntime.setUnknownHandler(
          &ArmorLinkRuntime::handleUnknownSerialLineThunk,
          this
      );
  }
    initStartupHelloState();

    _dispatch.begin(&module, &_storage);
#if ARMORLINK_HAS_ACTIONBUILDER
    _module->actionBuilder().onLog([this](ArmorLinkLogLevel level, const String& message) {
      sendSimpleLog(level, message);
    });
    _module->actionBuilder().onRemoteAction(
      [this](const String& target, const String& entity, const String& command) {
        return sendActionBuilderCommandToTarget(target, entity, command);
      });
#endif

#if ARMORLINK_HAS_ESPNOW
    _transport.setManagedHandlers(
      &ArmorLinkRuntime::handleManagedReceiveStatic,
      &ArmorLinkRuntime::handleManagedSendStatic
    );
#endif
    setBleConnectHook(&ArmorLinkRuntime::handleBleConnectedStatic);
    setBleDisconnectHook(&ArmorLinkRuntime::handleBleDisconnectedStatic);

#if ARMORLINK_HAS_ESPNOW
    if(_options.enableEspNow)
    {
#if defined(ESP32) && ARMORLINK_HAS_BLE
      if (_options.enableBle) {
        // Hardware testing: an immediate start can lose ESP-NOW reception
        // after the first TX. Allow BLE startup to settle before starting WiFi/ESP-NOW.
        AL_INTERNAL_PRINTLN("[INFO] Waiting 2 seconds before ESP-NOW initialization (BLE + ESP-NOW startup).");
        delay(2000);
      }
#endif
      const bool espNowInitialized = _transport.beginManaged(_options.espNowChannel);
      if (espNowInitialized) {
        AL_INTERNAL_PRINTLN("[INFO] ArmorLink communication initialized.");
      } else {
        AL_INTERNAL_PRINTLN("[ERROR] ArmorLink ESP-NOW initialization failed.");
      }
    }
#endif

#if ARMORLINK_HAS_ACTIONBUILDER
    _module->actionBuilder().armStartup();
#endif

    AL_INTERNAL_PRINTF(
      "[INFO] ArmorLink %s loaded | module=%s | gateway=%s | ble=%s | espnow=%s | channel=%u\n",
      ARMORLINK_VERSION,
      _module != nullptr ? _module->name().c_str() : "unknown",
      _options.enableGateway ? "true" : "false",
      _options.enableBle ? "true" : "false",
      _options.enableEspNow ? "true" : "false",
      _options.espNowChannel
    );
  }

  void loop() {
#if ARMORLINK_HAS_ESPNOW
    if (_options.enableEspNow) {
      _transport.processCallbacks();
    }
#endif
#if ARMORLINK_HAS_ACTIONBUILDER
    // The sketch's setup() has returned before the first loop() call.
    if (_module != nullptr) _module->actionBuilder().runStartupOnce(_module->actions());
#endif
    processPendingPairResponse();
    processLocalConfigTransfer();
    bleProcessPendingConfigTransfer();
    processEspNowQueue();
#if ARMORLINK_HAS_ACTIONBUILDER
    if (_module != nullptr) {
      _module->actionBuilder().tick(_module->actions());
    }
#endif
    remotePairingQuietTick();
    pairingQuietStopRepeatTick();
    pairingTick();
    presenceProbeTick();
    presenceTick();
    startupHelloTick();
    startupStateSyncTick();
    heartbeatTick();
    loggingSyncTick();    
    serialTick();
    rebootTick();
  }

  String buildDescriptor() const {
    if (_module == nullptr) {
      return "{}";
    }
    return ArmorLinkDescriptor::buildWebSerial(*_module);
  }

  String buildBleDescriptor() const {
    if (_module == nullptr) {
      return "{}";
    }
    return ArmorLinkDescriptor::buildBle(*_module);
  }

  void notifyBleErrorEvent(uint16_t requestId,
                           const String& message,
                           const String& detail) {
    StaticJsonDocument<320> doc;
    doc["type"] = "error";
    doc["requestId"] = requestId;
    doc["message"] = message;
    if (!detail.isEmpty()) {
      doc["detail"] = detail;
    }

    String out;
    serializeJson(doc, out);
    doc.clear();
    bleNotifyEventJson(out);
  }

  bool queueLocalConfigTransfer(uint16_t requestId, const char* target) {
#if ARMORLINK_HAS_BLE

    if (_localConfigTransferPending) {
      return false;
    }

    _localConfigTransferPending = true;
    _localConfigTransferRequestId = requestId;
    armorlinkCopyString(
      _localConfigTransferTarget,
      sizeof(_localConfigTransferTarget),
      target ? target : "");
    return true;
#else
    return false;
#endif
  }

  void processLocalConfigTransfer() {
#if ARMORLINK_HAS_BLE
    if (!_localConfigTransferPending) {
      return;
    }

    const uint16_t requestId = _localConfigTransferRequestId;
    char target[ARMORLINK_NAME_MAX_LEN + 1] = { 0 };
    armorlinkCopyString(target, sizeof(target), _localConfigTransferTarget);

    _localConfigTransferPending = false;
    _localConfigTransferRequestId = 0;
    memset(_localConfigTransferTarget, 0, sizeof(_localConfigTransferTarget));

    if (_module == nullptr) {
      notifyBleErrorEvent(
        requestId,
        "Config descriptor build failed",
        "Module not initialized"
      );
      return;
    }

    _localConfigTransferActive = true;

    {
      size_t descriptorLength = 0;
      if (ArmorLinkDescriptor::measureBle(*_module, descriptorLength)) {
        AL_INTERNAL_PRINTF("[CONFIG] local descriptor length=%u | profile=ble\n",
                      static_cast<unsigned>(descriptorLength));
        const bool sent = ArmorLinkBLE.notifyConfigJsonStreamChunked(
          target,
          "",
          requestId,
          descriptorLength,
          [this](Print& out) { return ArmorLinkDescriptor::writeBle(*_module, out); },
          false
        );
        _localConfigTransferActive = false;
        if (sent) return;
      }
    }

    _localConfigTransferActive = false;

    notifyBleErrorEvent(
      requestId,
      "Config descriptor build failed",
      "Descriptor allocation failed"
    );
#endif
  }

  ArmorLinkStorage& storage() {
    return _storage;
  }

  ArmorLinkDispatch& dispatch() {
    return _dispatch;
  }

  ArmorLinkTransportEspNow& transport() {
    return _transport;
  }

  ArmorLinkPeerRegistry& peers() {
    return _peers;
  }

  ArmorLinkModule* module() {
    return _module;
  }

  const ArmorLinkOptions& options() const {
    return _options;
  }

  void setEspNowCommandHook(ArmorLinkEspNowCommandHook hook) {
    _espNowCommandHook = hook;
  }

// Gateway mode is also the BLE root role: it sends the gateway_descriptor that
// tells a phone this node's module name. That needs no ESP-NOW, so it is
// honoured when ESP-NOW is excluded. Every ESP-NOW send is compiled out or
// checks _options.enableEspNow, which stays false there. Notable effects that
// remain with gateway on: addressed forwards are refused with "Module not
// paired"; a BLE disconnect clears remote logging/telemetry; data streams take
// the gateway route; once a PIN is set, the USB serial config_meta carries it
// (enableSerialCommands or enableSerialMenu); enableSerialMenu runs the menu.
void setGatewayMode(bool enabled) {
  _isGatewayMode = enabled;

  if (_isGatewayMode) {
    _startupHelloActive = false;
    _startupHelloAcked = false;
    _startupHelloUntilMs = 0;
    _lastStartupHelloMs = 0;
    _startupHelloAttempt = 0;
  }
  if (_module != nullptr) registerSystemActions(*_module);
}

  bool isGatewayMode() const {
    return _isGatewayMode;
  }

  bool isPaired() const {
    return _pairingInfo.paired;
  }

  // Reset only ESP-NOW associations for the current role. Bluetooth security,
  // configuration, profiles and actions are always kept.
  bool resetPairingInfo() {
#if ARMORLINK_HAS_ESPNOW
    if (_isGatewayMode) {
      if (!_storage.clearPairedModules()) {
        AL_INTERNAL_PRINTLN("[PAIR][GW] Failed to reset paired-node list");
        return false;
      }
      stopPairing();
      clearPendingPairingCandidates();
      // Notify reachable nodes before dropping their peer routing entries.
      // Offline nodes still need their own pairing reset before pairing anew.
      for (size_t i = 0; i < _pairedModuleCount; ++i) {
        const auto& removed = _pairedModules[i];
        uint8_t mac[6];
        if (armorLinkParseMacString(String(removed.mac), mac)) {
          ArmorLinkPacket out = makeArmorLinkBasePacket(
              AL_MSG_COMMAND, _module ? _module->name().c_str() : "Gateway",
              removed.name, "pairing", "unpair");
          setArmorLinkPacketPayload(out, String(removed.mac));
          const esp_err_t result = _transport.sendPacketToMac(mac, out);
          AL_INTERNAL_PRINTF("[PAIR][GW] Reset: unpair sent to %s: %s\n", removed.name, esp_err_to_name(result));
        }
        _peers.removePeer(String(removed.name));
        emitModuleUnpairedEvent(removed);
      }
      memset(static_cast<void*>(_pairedModules), 0, sizeof(_pairedModules));
      _pairedModuleCount = 0;
      syncPresenceStatesFromStoredModules();
      emitModulePresenceSnapshot();
      AL_INTERNAL_PRINTLN("[PAIR][GW] All paired-node associations reset; other settings preserved");
      return true;
    }
    if (!_storage.clearPairingInfo()) {
      AL_INTERNAL_PRINTLN("[PAIR] Failed to reset pairing info");
      return false;
    }
    _peers.removePeer(String(_pairingInfo.gatewayName));
    _pairingInfo = ArmorLinkPairingInfo{};
    _options.defaultLogTarget = "";
    _remoteTelemetryEnabled = false;
    _remoteLoggingEnabled = false;
    _hasPendingPairResponse = false;
    clearRemotePairingQuiet(0);
    initStartupHelloState();
    AL_INTERNAL_PRINTLN("[PAIR] Pairing info reset; all other settings preserved");
    return true;
#else
    return false;
#endif
  }

  const char* pairedGatewayName() const {
    return _pairingInfo.gatewayName;
  }

  const char* pairedGatewayMac() const {
    return _pairingInfo.gatewayMac;
  }

  uint32_t recoveryPin() const {
    return _pairingInfo.recoveryPin;
  }

  void setRecoveryPin(uint32_t pin) {
    _pairingInfo.recoveryPin = pin;
    _storage.savePairingInfo(_pairingInfo);
  }

  void setRemoteLoggingEnabled(bool enabled) {
    _remoteLoggingEnabled = enabled;
  }

  bool isRemoteLoggingEnabled() const {
    return _remoteLoggingEnabled;
  }

  void setRemoteLogLevel(uint8_t level) {
    _remoteLogLevel = level;
  }
void onBleConnected() {
  if (_options.enableSerialLogging) {
    AL_VERBOSELN("[BLE] Client connected");
  }
#if ARMORLINK_HAS_ESPNOW
  if (_isGatewayMode && _options.enableEspNow) _presenceProbePending = true;
#endif
  emitGatewayDescriptorEvent();
}
  void emitGatewayDescriptorEvent() {
    if (_module == nullptr || !_isGatewayMode || !isBleClientConnected()) {
      return;
    }

    StaticJsonDocument<512> doc;
    doc["type"] = "gateway_descriptor";
    doc["projectName"] = ArmorLinkSecurity::getProjectName();

    JsonObject module = doc.createNestedObject("module");
    module["name"] = _module->name();
    module["type"] = moduleTypeToString(_module->type());
    module["mac"] = localMacString();
    module["moduleVersion"] = _module->version();
    module["armorLinkVersion"] = ARMORLINK_VERSION;
    module["isGateway"] = true;

    doc["bleName"] = _options.bleName;

    if (doc.overflowed()) {
      AL_INTERNAL_PRINTF("[BLE][TX] gateway_descriptor allocation failed: heap8=%u largestBlock8=%u\n",
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                    (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
      return;
    }
    String out;
    serializeJson(doc, out);
    doc.clear();
    AL_INTERNAL_PRINTF("[BLE][TX] gateway_descriptor bytes=%u\n",
                  static_cast<unsigned>(out.length()));
    bleNotifyEventJson(out);
  }

void onBleDisconnected() {
  if (_options.enableSerialLogging) {
    AL_VERBOSELN("[BLE] Client disconnected -> disabling BLE and remote logging");
  }

  setBleLogStreamEnabled(false);

  if (_isGatewayMode) {
    _remoteLoggingEnabled = false;
    broadcastLoggingState(false);
    _remoteTelemetryEnabled = false;
    broadcastTelemetryState(false);
  }

  _startupHelloAcked = false;
  _startupHelloActive = false;
}

  void sendGatewayStatePacketToPairedModules(ArmorLinkPacket& packet) {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || !_options.enableEspNow) return;
    if (_pairingActive) return;

    for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
      if (strlen(_pairedModules[i].mac) == 0) {
        continue;
      }

      uint8_t moduleMac[6];
      if (!armorLinkParseMacString(String(_pairedModules[i].mac), moduleMac)) {
        continue;
      }

      armorlinkCopyString(packet.target, sizeof(packet.target), _pairedModules[i].name);
      _transport.sendPacketToMac(moduleMac, packet);
      delay(5);
      yield();
    }
#endif
  }

  void broadcastTelemetryState(bool enabled) {
    if (!_isGatewayMode || _module == nullptr || !_options.enableEspNow) return;

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      "",
      "telemetry",
      "set_enabled"
    );

    out.valueInt = enabled ? 1 : 0;
    sendGatewayStatePacketToPairedModules(out);
  }
  void broadcastLoggingState(bool enabled) {
    if (!_isGatewayMode || _module == nullptr || !_options.enableEspNow) {
      return;
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      "",
      "logs",
      "set_enabled"
    );
    out.valueInt = enabled ? 1 : 0;

    sendGatewayStatePacketToPairedModules(out);
  }

  void broadcastRemoteLogLevel(uint8_t level) {
    if (!_isGatewayMode || _module == nullptr || !_options.enableEspNow) {
      return;
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      "",
      "logs",
      "set_level"
    );
    out.valueInt = level;

    sendGatewayStatePacketToPairedModules(out);
  }

  void broadcastPairingQuietState(bool active, uint16_t sessionId) {
    if (!_isGatewayMode || _module == nullptr || !_options.enableEspNow) {
      return;
    }

    String payload;
    {
      StaticJsonDocument<96> doc;
      doc["sessionId"] = sessionId;
      doc["active"] = active;
      serializeJson(doc, payload);
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      "*",
      "pairing",
      active ? "quiet_start" : "quiet_stop"
    );
    out.valueInt = sessionId;
    setArmorLinkPacketPayload(out, payload);

    const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    esp_err_t result = ESP_FAIL;
    const uint8_t attempts = active ? 2 : 3;

    for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
      result = _transport.sendPacketToMac(broadcastMac, out);
      if (attempt + 1 < attempts) {
        delay(active ? 20 : 35);
        yield();
      }
    }

    AL_INTERNAL_PRINTF("[PAIR][GW] Pairing quiet %s broadcast | sessionId=%u | result=%s\n",
                  active ? "start" : "stop",
                  sessionId,
                  esp_err_to_name(result));
  }

  bool startPairing(uint32_t timeoutMs = 30000) {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode) {
      return false;
    }

    _pairingActive = true;
    _pairingEndsAtMs = millis() + timeoutMs;
    _pairingSessionId = nextArmorLinkPacketSeq();
    if (_pairingSessionId == 0) {
      _pairingSessionId = nextArmorLinkPacketSeq();
    }
    if (_pairingSessionId == 0) {
      _pairingSessionId = 1;
    }
    _lastPairAnnounceMs = 0;
    clearPendingPairingCandidates();
    _pairingQuietStopRepeatSessionId = 0;
    _pairingQuietStopRepeatUntilMs = 0;
    _lastPairingQuietStopMs = 0;

    AL_INTERNAL_PRINTF("[PAIR][GW] Pairing started | sessionId=%u | timeoutMs=%lu\n",
                  _pairingSessionId,
                  (unsigned long)timeoutMs);

    broadcastPairingQuietState(true, _pairingSessionId);
    emitPairingStateEvent("pairing_started");

    const bool ok = sendPairAnnounceBurst();
    if (ok) {
      _lastPairAnnounceMs = millis();
      emitPairingStateEvent("pairing_ping");
    }

    return ok;
#else
    return false;
#endif
  }

  void stopPairing() {
#if ARMORLINK_HAS_ESPNOW
    if (!_pairingActive && _pairingSessionId == 0) {
      return;
    }

    const uint16_t stoppedSessionId = _pairingSessionId;

    broadcastPairingQuietState(false, stoppedSessionId);
    _pairingActive = false;
    _pairingEndsAtMs = 0;
    _pairingSessionId = 0;
    _lastPairAnnounceMs = 0;
    clearPendingPairingCandidates();
    schedulePairingQuietStopRepeat(stoppedSessionId);

    AL_INTERNAL_PRINTF("[PAIR][GW] Pairing stopped | sessionId=%u\n", stoppedSessionId);
    emitPairingStateEvent("pairing_stopped", stoppedSessionId);
#endif
  }

  size_t pairingCandidateCount() const {
#if ARMORLINK_HAS_ESPNOW
    return _pendingCandidateCount;
#else
    return 0;
#endif
  }

  const ArmorLinkPairingCandidate* pairingCandidates() const {
#if ARMORLINK_HAS_ESPNOW
    return _pendingCandidates;
#else
    return nullptr;
#endif
  }

  bool acceptPairingCandidate(const String& macText) {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode) {
      return false;
    }

    int index = findPendingCandidateIndexByMac(macText);
    if (index < 0) {
      return false;
    }

    uint8_t moduleMac[6];
    if (!armorLinkParseMacString(macText, moduleMac)) {
      return false;
    }

    String payload;
    {
      StaticJsonDocument<160> doc;
      doc["sid"] = _pendingCandidates[index].sessionId;
      doc["gn"] = (_module != nullptr) ? _module->name() : "Gateway";
      doc["gm"] = localMacString();
      doc["rp"] = _pairingInfo.recoveryPin;
      serializeJson(doc, payload);
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_PAIR_ACCEPT,
      (_module != nullptr) ? _module->name().c_str() : "Gateway",
      _pendingCandidates[index].name,
      "pairing",
      "accept"
    );
    setArmorLinkPacketPayload(out, payload);

    AL_INTERNAL_PRINTF("[PAIR][GW] Accepting candidate: %s | %s | %s\n",
                  _pendingCandidates[index].name,
                  _pendingCandidates[index].type,
                  _pendingCandidates[index].mac);
    esp_err_t sendResult = ESP_FAIL;
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
      sendResult = _transport.sendPacketToMac(moduleMac, out);
      if (sendResult == ESP_OK) {
        delay(35);
        yield();
      } else {
        delay(20);
        yield();
      }
    }

    AL_INTERNAL_PRINT("[PAIR][GW] Pair accept send result: ");
    AL_VERBOSE(esp_err_to_name(sendResult));

    if (sendResult != ESP_OK) {
      return false; 
    }

    upsertStoredPairedModule(
      _pendingCandidates[index].name,
      _pendingCandidates[index].type,
      _pendingCandidates[index].mac
    );
    if (_options.syncLoggingStateOnModuleOnline) {
      syncLoggingStateToModule(
        _pendingCandidates[index].name,
        _pendingCandidates[index].mac
      );
    }
    syncPresenceStatesFromStoredModules();
    updateModulePresenceByMac(_pendingCandidates[index].mac);

    emitPairingCompletedEvent(_pendingCandidates[index]);
    _pendingCandidates[index].occupied = false;
    rebuildPendingCandidateCount();
    stopPairing();
    emitModulePresenceSnapshot();
    return true;
#else
    return false;
#endif
  }
  ArmorLinkTelemetryBuilder telemetryGroup(const char* group, const char* name) {
    return ArmorLinkTelemetryBuilder(this, group, name);
  }

  ArmorLinkDataStreamBuilder dataStream(const char* stream) {
    return ArmorLinkDataStreamBuilder(this, stream);
  }

  bool subscribeDataStream(const char* source, const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    if (stream == nullptr || strlen(stream) == 0) {
      return false;
    }

    const char* normalizedSource =
      (source == nullptr || strlen(source) == 0) ? "*" : source;

    upsertLocalDataStreamSubscription(normalizedSource, stream);

    if (_isGatewayMode) {
      const char* localName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
      upsertGatewayDataStreamSubscription(localName, normalizedSource, stream);
      return true;
    }

    return sendDataStreamSubscription(normalizedSource, stream, true) == ESP_OK;
#else
    return false;
#endif
  }

  bool unsubscribeDataStream(const char* source, const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    if (stream == nullptr || strlen(stream) == 0) {
      return false;
    }

    const char* normalizedSource =
      (source == nullptr || strlen(source) == 0) ? "*" : source;

    removeLocalDataStreamSubscription(normalizedSource, stream);

    if (_isGatewayMode) {
      const char* localName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
      removeGatewayDataStreamSubscription(localName, normalizedSource, stream);
      return true;
    }

    return sendDataStreamSubscription(normalizedSource, stream, false) == ESP_OK;
#else
    return false;
#endif
  }

  bool onDataStream(const char* stream, ArmorLinkDataStreamHandler handler) {
#if ARMORLINK_HAS_DATASTREAMS
    if (stream == nullptr || strlen(stream) == 0 || !handler) {
      return false;
    }

    for (size_t i = 0; i < MAX_DATA_STREAM_HANDLERS; ++i) {
      if (_dataStreamHandlers[i].occupied &&
          String(_dataStreamHandlers[i].stream).equalsIgnoreCase(stream)) {
#if ARMORLINK_HAS_DATASTREAMS
        _dataStreamHandlers[i].handler = handler;
#endif
        return true;
      }
    }

    for (size_t i = 0; i < MAX_DATA_STREAM_HANDLERS; ++i) {
      if (!_dataStreamHandlers[i].occupied) {
        _dataStreamHandlers[i].occupied = true;
        armorlinkCopyString(_dataStreamHandlers[i].stream,
                            sizeof(_dataStreamHandlers[i].stream),
                            stream);
#if ARMORLINK_HAS_DATASTREAMS
        _dataStreamHandlers[i].handler = handler;
#endif
        return true;
      }
    }

    AL_INTERNAL_PRINTF("[STREAM] No handler slot available for %s\n", stream);
    return false;
#else
    return false;
#endif
  }

  esp_err_t sendCommand(
    const char* target,
    const char* entity,
    const char* command,
    const String& value = "",
    const char* sourceOverride = nullptr)
  {
    const char* sourceName = sourceOverride;
    if (!sourceName || strlen(sourceName) == 0) {
      sourceName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      sourceName,
      target,
      entity,
      command
    );
    setArmorLinkPacketPayload(out, value);

    if (!_isGatewayMode &&
        isRemotePairingQuietActive() &&
        !isPairingTraffic(out)) {
      return ESP_OK;
    }

    return _transport.sendPacketToTarget(String(target), out);
  }

  esp_err_t broadcastCommand(
    const char* entity,
    const char* command,
    const String& value = "",
    const char* sourceOverride = nullptr)
  {
    const char* sourceName = sourceOverride;
    if (!sourceName || strlen(sourceName) == 0) {
      sourceName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      sourceName,
      "*",
      entity,
      command
    );

    setArmorLinkPacketPayload(out, value);

    if (!_isGatewayMode &&
        isRemotePairingQuietActive() &&
        !isPairingTraffic(out)) {
      return ESP_OK;
    }

    const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    return _transport.sendPacketToMac(broadcastMac, out);
  }

  void info(const String& message) {
    sendSimpleLog(AL_LOG_INFO, message);
  }

  void warn(const String& message) {
    sendSimpleLog(AL_LOG_WARN, message);
  }

  void error(const String& message) {
    sendSimpleLog(AL_LOG_ERROR, message);
  }

  void debug(const String& message) {
    sendSimpleLog(AL_LOG_DEBUG, message);
  }

  esp_err_t sendLog(
    ArmorLinkLogLevel level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f)
  {
    return sendLogToDefaultTarget(
      level,
      entity,
      command,
      message,
      valueInt,
      valueFloat,
      batteryVoltage
    );
  }

  esp_err_t sendLogToDefaultTarget(
    ArmorLinkLogLevel level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f)
  {
    const char* sourceName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";

    if (_options.enableSerialLogging) {
      Serial.printf("[%s] %s.%s -> %s\n",
                    armorLinkLogLevelToString(level),
                    entity ? entity : "",
                    command ? command : "",
                    message.c_str());
    }

    if (!_remoteLoggingEnabled && !_options.forceRemoteLogging) {
      return ESP_OK;
    }

    if (level < _remoteLogLevel) {
      return ESP_OK;
    }

#if !ARMORLINK_HAS_ESPNOW
    notifyLocalBleLog(sourceName, level, entity, command, message,
                      valueInt, valueFloat, batteryVoltage);
    return ESP_OK;
#else
    if (_options.defaultLogTarget.isEmpty()) {
      return ESP_ERR_NOT_FOUND;
    }

    if (!_isGatewayMode && isRemotePairingQuietActive()) {
      return ESP_OK;
    }
        if (!shouldSendRemoteLog(level)) {
      return ESP_OK;
    }
    return _transport.sendLogToTarget(
      _options.defaultLogTarget,
      sourceName,
      level,
      entity,
      command,
      message,
      valueInt,
      valueFloat,
      batteryVoltage
    );
#endif
  }

  void logModule(
    const char* source,
    uint8_t level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt = 0,
    float valueFloat = 0.0f,
    float batteryVoltage = 0.0f
  )
  {
    Serial.printf("[%s] %s.%s -> %s\n",
                  armorLinkLogLevelToString(level),
                  entity ? entity : "",
                  command ? command : "",
                  message.c_str());

    notifyLocalBleLog(source, level, entity, command, message,
                      valueInt, valueFloat, batteryVoltage);
}
  void onEspNowDataReceived(const uint8_t* data, int len) {
#if ARMORLINK_HAS_ESPNOW
    ArmorLinkPacket incoming{};
    if (!_transport.decodeIncomingPacket(data, len, incoming)) {
      _transport.recordReceiveError(true);
      return;
    }

    if (_isGatewayMode &&
        _pairingActive &&
        incoming.msgType == AL_MSG_PAIR_RESPONSE) {
      _pendingPairResponse = incoming;
      _hasPendingPairResponse = true;
      return;
    }

    if (!_transport.enqueueReceivedPacket(incoming)) {
      _transport.recordReceiveError(false);
    }
#endif
  }

  void onEspNowSendStatus(const uint8_t* /*mac_addr*/, esp_now_send_status_t status) {
#if ARMORLINK_HAS_ESPNOW
    AL_INTERNAL_PRINTF("[ESPNOW][TX] status=%s\n",
                  status == ESP_NOW_SEND_SUCCESS ? "success" : "fail");
#endif
  }

  void processEspNowQueue() {
#if ARMORLINK_HAS_ESPNOW
    ArmorLinkPacket msg;
    while (_transport.dequeueReceivedPacket(msg)) {
      if (_isGatewayMode) {
        AL_INTERNAL_PRINTF("[ESPNOW][GW][RX] type=%u source=%s target=%s entity=%s command=%s payloadLen=%u\n",
          msg.msgType, msg.source, msg.target, msg.entity, msg.command, msg.payloadLen);
      }
      processIncomingEspNowPacket(msg);
    }
#endif
  }

  void processPendingPairResponse() {
#if ARMORLINK_HAS_ESPNOW
    if (!_hasPendingPairResponse) {
      return;
    }

    ArmorLinkPacket msg = _pendingPairResponse;
    _hasPendingPairResponse = false;

    AL_INTERNAL_PRINTF("[PAIR][GW] Pair response received from %s | entity=%s | command=%s | payload=%s\n",
                  msg.source,
                  msg.entity,
                  msg.command,
                  armorLinkPacketPayloadToString(msg).c_str());
    handlePairResponsePacket(msg);
#endif
  }

  void printLocalMac() const {
#if ARMORLINK_HAS_ESPNOW
#if defined(ESP32)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("?? ESP32 MAC (STA): %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#elif defined(ESP8266)
    Serial.printf("?? ESP8266 MAC: %s\n", WiFi.macAddress().c_str());
#else
    AL_VERBOSELN("?? Local MAC unavailable on this platform");
#endif
#endif
  }

  void logInfo(const String& entity, const String& command, const String& message) {
    logStructuredSerial("INFO", entity, command, message);
  }

  void logWarn(const String& entity, const String& command, const String& message) {
    logStructuredSerial("WARN", entity, command, message);
  }

  void logError(const String& entity, const String& command, const String& message) {
    logStructuredSerial("ERROR", entity, command, message);
  }

  bool handleSecurityPacket(
    const ArmorLinkPacket& packet,
    const String& payload,
    const char* securityEntity,
    void (*notifyAck)(uint16_t, const String&, const String&),
    void (*notifyError)(uint16_t, const String&, const String&),
    void (*notifyEventJson)(const String&),
    bool restartAfterPinSave = true)
  {
    if (!equalsIgnoreCase(packet.entity, securityEntity)) {
      return false;
    }

    if (packet.msgType == AL_MSG_COMMAND &&
        equalsIgnoreCase(packet.command, "get_state")) {

      StaticJsonDocument<192> doc;
      doc["type"] = "status";

      JsonObject security = doc.createNestedObject("security");
      security["mode"] = ArmorLinkSecurity::isPinSet() ? "secure" : "setup";
      security["pinSet"] = ArmorLinkSecurity::isPinSet();
      doc["projectName"] = ArmorLinkSecurity::getProjectName();

      if (doc.overflowed()) {
        AL_INTERNAL_PRINTF("[BLE][TX] security status allocation failed: heap8=%u largestBlock8=%u\n",
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
                      (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        notifyEventJson("{\"type\":\"error\",\"message\":\"Security status allocation failed\"}");
        return true;
      }
      String out;
      serializeJson(doc, out);
      // Do not retain this pool while constructing the gateway descriptor.
      doc.clear();
      AL_INTERNAL_PRINTF("[BLE][TX] status bytes=%u\n",
                    static_cast<unsigned>(out.length()));
      notifyEventJson(out);

      if (_isGatewayMode) {
        emitGatewayDescriptorEvent();
      }
      return true;
    }

    if (packet.msgType == AL_MSG_COMMAND &&
        equalsIgnoreCase(packet.command, "set_ble_pin")) {

      if (ArmorLinkSecurity::isPinSet()) {
        notifyError(packet.requestId, "PIN already set", "");
        return true;
      }

      int pin = packet.valueInt;

      if (pin < 100000 || pin > 999999) {
        notifyError(packet.requestId, "PIN must be 6 digits", "");
        return true;
      }

      if (pin == 123456 || pin == 0) {
        notifyError(packet.requestId, "Weak PIN not allowed", "");
        return true;
      }
      StaticJsonDocument<256> setupDoc;
      if (deserializeJson(setupDoc, payload) == DeserializationError::Ok) {
        const char* projectName = setupDoc["projectName"] | "";
        if (strlen(projectName) > 0) {
          ArmorLinkSecurity::saveProjectName(String(projectName));
        }
      }
      if (!ArmorLinkSecurity::savePin((uint32_t)pin)) {
        notifyError(packet.requestId, "Failed to save PIN", "");
        return true;
      }

      notifyAck(packet.requestId, "ok", "Setup saved, restart required");

      if (restartAfterPinSave) {
        delay(800);
        ESP.restart();
      }

      return true;
    }

    return false;
  }

  bool enforceProvisioning(
    const ArmorLinkPacket& packet,
    const char* securityEntity,
    void (*notifyError)(uint16_t, const String&, const String&))
  {
    if (ArmorLinkSecurity::isPinSet()) {
      return false;
    }

    const bool allowedSecurityCommand =
      equalsIgnoreCase(packet.entity, securityEntity) &&
      (equalsIgnoreCase(packet.command, "set_ble_pin") ||
       equalsIgnoreCase(packet.command, "get_state"));

    if (!allowedSecurityCommand) {
      notifyError(packet.requestId, "Device not provisioned", "");
      return true;
    }

    return false;
  }

  template<
    typename TSetLogStreamEnabled,
    typename TNotifyAck,
    typename TNotifyError>
  bool handlePacket(
    const ArmorLinkPacket& packet,
    const String& payload,
    const char* localTarget,
    TSetLogStreamEnabled setLogStreamEnabled,
    TNotifyAck notifyAck,
    TNotifyError notifyError)
  {
    if (_module == nullptr) {
      notifyError(packet.requestId, "ArmorLink not initialized", "");
      return true;
    }

  if (packet.msgType == AL_MSG_COMMAND &&
      equalsIgnoreCase(packet.entity, "pairing") &&
      equalsIgnoreCase(packet.target, localTarget)) {

      const String textPayload = payload;

      if (equalsIgnoreCase(packet.command, "start")) {
        const uint32_t timeoutMs = packet.valueInt > 0 ? static_cast<uint32_t>(packet.valueInt) : 30000;
        const bool ok = startPairing(timeoutMs);
        if (ok) {
          notifyAck(packet.requestId, "ok", "Pairing started");
        } else {
          notifyError(packet.requestId, "Failed to start pairing", "");
        }
        return true;
      }

      if (equalsIgnoreCase(packet.command, "stop")) {
        stopPairing();
        notifyAck(packet.requestId, "ok", "Pairing stopped");
        return true;
      }

      if (equalsIgnoreCase(packet.command, "accept")) {
        const bool ok = acceptPairingCandidate(textPayload);
        if (ok) {
          notifyAck(packet.requestId, "ok", "Pairing accepted");
        } else {
          notifyError(packet.requestId, "Failed to accept pairing", "");
        }
        return true;
      }
      if (equalsIgnoreCase(packet.command, "unpair")) {
        const bool ok = unpairPairedModuleByMac(textPayload);

        if (ok) {
          notifyAck(packet.requestId, "ok", "Module unpaired");
        } else {
          notifyError(packet.requestId, "Failed to unpair module", "");
        }

        return true;
      }
    }

    if (_isGatewayMode && _pairingActive) {
      notifyError(
        packet.requestId,
        "Pairing active",
        "Only pairing commands are accepted while pairing is active");
      return true;
    }

    if (packet.msgType == AL_MSG_COMMAND &&
        equalsIgnoreCase(packet.entity, "logs") &&
        equalsIgnoreCase(packet.command, "enable")) {

      const bool enabled =
        packet.valueInt != 0 ||
        payload.equalsIgnoreCase("true") ||
        payload.equalsIgnoreCase("on");

      setLogStreamEnabled(enabled);

      if (_isGatewayMode && _pairedModuleCount > 0) {
        _remoteLoggingEnabled = enabled;
        broadcastLoggingState(enabled);
      } else if (_isGatewayMode || !ARMORLINK_HAS_ESPNOW) {
        _remoteLoggingEnabled = enabled;
      }

      notifyAck(
        packet.requestId,
        "ok",
        String("Log stream ") + (enabled ? "enabled" : "disabled"));

      return true;
    }

    if (packet.msgType == AL_MSG_COMMAND &&
        equalsIgnoreCase(packet.entity, "presence") &&
        equalsIgnoreCase(packet.command, "snapshot")) {

#if ARMORLINK_HAS_ESPNOW
      if (_isGatewayMode && _options.enableEspNow) _presenceProbePending = true;
#endif
      emitModulePresenceSnapshot();
      notifyAck(packet.requestId, "ok", "Presence snapshot sent");
      return true;
    }

    if (packet.msgType == AL_MSG_COMMAND &&
        equalsIgnoreCase(packet.entity, "telemetry") &&
        equalsIgnoreCase(packet.command, "enable")) {

      const bool enabled =
        packet.valueInt != 0 ||
        payload.equalsIgnoreCase("true") ||
        payload.equalsIgnoreCase("on");

      _remoteTelemetryEnabled = enabled;

      if (_isGatewayMode && _pairedModuleCount > 0) {
        broadcastTelemetryState(enabled);
      }

      notifyAck(
        packet.requestId,
        "ok",
        String("Telemetry stream ") + (enabled ? "enabled" : "disabled"));

      return true;
    }
        if (packet.msgType == AL_MSG_COMMAND &&
        equalsIgnoreCase(packet.entity, "logs") &&
        equalsIgnoreCase(packet.command, "set_level")) {

      uint8_t level = static_cast<uint8_t>(packet.valueInt);
      if (level < AL_LOG_DEBUG || level > AL_LOG_ERROR) {
        level = logLevelFromKeyword(payload);
      }

      setRemoteLogLevel(level);

      if (_isGatewayMode) {
        broadcastRemoteLogLevel(level);
      }

      notifyAck(
        packet.requestId,
        "ok",
        String("Remote log level set to ") + logLevelToKeyword(level));

      return true;
    }
    

    if (packet.msgType == AL_MSG_CONFIG_GET &&
        equalsIgnoreCase(packet.target, localTarget)) {

      if (!queueLocalConfigTransfer(packet.requestId, localTarget)) {
        notifyError(
          packet.requestId,
          "Config transfer busy",
          ""
        );
      }

      return true;
    }

    if (packet.msgType == AL_MSG_CONFIG_GET &&
        !equalsIgnoreCase(packet.target, localTarget)) {

      if (_isGatewayMode && findPairedModuleIndexByName(packet.target) < 0) {
        notifyError(
          packet.requestId,
          "Module not paired",
          String(packet.target));
        return true;
      }

      const String configEntity = strlen(packet.entity) > 0
        ? String(packet.entity)
        : String("config");

      ArmorLinkPacket out = makeArmorLinkBasePacket(
        AL_MSG_CONFIG_GET,
        localTarget,
        packet.target,
        configEntity.c_str(),
        "get");

      out.requestId = packet.requestId;
      out.flags = packet.flags;

      esp_err_t result = _transport.sendPacketToTarget(String(packet.target), out);
      AL_INTERNAL_PRINTF("[CONFIG][GW] forward get requestId=%u source=%s target=%s entity=%s -> %s\n",
                    out.requestId,
                    out.source,
                    out.target,
                    out.entity,
                    esp_err_to_name(result));
      if (result == ESP_OK) {
        notifyAck(
          packet.requestId,
          "forwarded",
          String("config_get forwarded to ") + packet.target);
      } else {
        notifyError(
          packet.requestId,
          "Failed to forward config_get",
          String(esp_err_to_name(result)));
      }
      return true;
    }

  if (packet.msgType == AL_MSG_CONFIG_SET &&
      equalsIgnoreCase(packet.target, localTarget)) {

    auto configResult = dispatchConfigSetPacket(
      String(packet.entity),
      String(packet.command),
      packet,
      payload);

    if (configResult == ArmorLinkDispatchResult::Ok) {
      notifyAck(packet.requestId, "ok", "Config updated");
      return true;
    }

    notifyError(
      packet.requestId,
      "Config update failed",
      String(ArmorLinkDispatch::toString(configResult)));

    return true;
  }

    if (packet.msgType == AL_MSG_CONFIG_SET &&
        !equalsIgnoreCase(packet.target, localTarget)) {

      if (_isGatewayMode && findPairedModuleIndexByName(packet.target) < 0) {
        notifyError(
          packet.requestId,
          "Module not paired",
          String(packet.target));
        return true;
      }

      ArmorLinkPacket out = makeArmorLinkBasePacket(
        AL_MSG_CONFIG_SET,
        localTarget,
        packet.target,
        packet.entity,
        packet.command);

      out.requestId = packet.requestId;
      out.flags = packet.flags;
      out.valueInt = packet.valueInt;
      out.valueFloat = packet.valueFloat;
      setArmorLinkPacketPayload(out, payload);

      esp_err_t result = _transport.sendPacketToTarget(String(packet.target), out);
      if (result == ESP_OK) {
        notifyAck(
          packet.requestId,
          "forwarded",
          String("config_set forwarded to ") + packet.target);
      } else {
        notifyError(
          packet.requestId,
          "Failed to forward config_set",
          String(esp_err_to_name(result)));
      }
      return true;
    }
    if (packet.msgType == AL_MSG_COMMAND &&
        !equalsIgnoreCase(packet.target, localTarget) &&
        !equalsIgnoreCase(packet.target, "*")) {

      if (_isGatewayMode && findPairedModuleIndexByName(packet.target) < 0) {
        notifyError(
          packet.requestId,
          "Module not paired",
          String(packet.target));
        return true;
      }

      ArmorLinkPacket out = makeArmorLinkBasePacket(
        AL_MSG_COMMAND,
        localTarget,
        packet.target,
        packet.entity,
        packet.command
      );

      out.requestId = packet.requestId;
      out.flags = packet.flags;
      out.valueInt = packet.valueInt;
      out.valueFloat = packet.valueFloat;
      out.batteryVoltage = packet.batteryVoltage;
      setArmorLinkPacketPayload(out, payload);

      esp_err_t result = _transport.sendPacketToTarget(String(packet.target), out);
      if (result == ESP_OK) {
        notifyAck(
          packet.requestId,
          "forwarded",
          String("command forwarded to ") + packet.target);
      } else {
        notifyError(
          packet.requestId,
          "Failed to forward command",
          String(esp_err_to_name(result)));
      }
      return true;
    }
    if (packet.msgType == AL_MSG_COMMAND &&
        (equalsIgnoreCase(packet.target, localTarget) || equalsIgnoreCase(packet.target, "*"))) {

#if ARMORLINK_HAS_ACTIONBUILDER
      if (_module != nullptr) {
        const String actionEntity = String(packet.entity);
        const String actionCommand = String(packet.command);
        const bool hasActionDefinition =
            _module->actionBuilder().findDefinition(actionEntity, actionCommand) != nullptr;

        if (hasActionDefinition && _module->actionBuilder().isRunning()) {
          if (!equalsIgnoreCase(packet.target, "*")) {
            notifyError(packet.requestId, "Action sequence is already running", "");
          }
          return true;
        }

        if (_module->actionBuilder().execute(actionEntity, actionCommand, _module->actions())) {
          if (!equalsIgnoreCase(packet.target, "*")) {
            notifyAck(packet.requestId, "ok", "Action sequence started");
          }
          return true;
        }
      }
#endif
      auto actionResult = _dispatch.handleAction(String(packet.entity), String(packet.command));
      if (actionResult == ArmorLinkDispatchResult::Ok) {
        if (!equalsIgnoreCase(packet.target, "*")) {
          notifyAck(packet.requestId, "ok", "Action executed");
        }
        return true;
      }

      auto configResult = dispatchConfigSetPacket(
        String(packet.entity),
        String(packet.command),
        packet,
        payload);

      if (configResult == ArmorLinkDispatchResult::Ok) {
        if (!equalsIgnoreCase(packet.target, "*")) {
          notifyAck(packet.requestId, "ok", "Config updated");
        }
        return true;
      }

      if (!equalsIgnoreCase(packet.target, "*")) {
        notifyError(
          packet.requestId,
          "Unhandled local command",
          String(ArmorLinkDispatch::toString(configResult)));
      }
      return true;
    }

    return false;
  }

  esp_err_t telemetry(
    const char* group,
    const char* name,
    float value,
    const char* unit = ""
  ) {

    if (!_remoteTelemetryEnabled && !_options.forceRemoteLogging) {
      return ESP_OK;
    }

    if (_isGatewayMode && _pairingActive) {
      return ESP_OK;
    }

    if (!_isGatewayMode && isRemotePairingQuietActive()) {
      return ESP_OK;
    }

    String key = String(group ? group : "") + "." + (name ? name : "");
    if (!allowTelemetrySend(key)) {
      return ESP_OK;
    }

    StaticJsonDocument<256> doc;
    doc["type"] = "telemetry";
    doc["source"] = (_module != nullptr) ? _module->name() : "ArmorLink";
    doc["group"] = group ? group : "";
    doc["name"] = name ? name : "";
    doc["value"] = value;

    if (unit && strlen(unit) > 0) {
      doc["unit"] = unit;
    }

    String payload;
    serializeJson(doc, payload);

    if (_isGatewayMode || !ARMORLINK_HAS_ESPNOW) {
      bleNotifyEventJson(payload);
      emitSerialEventJson(payload);
      return ESP_OK;
    }

    if (_options.defaultLogTarget.isEmpty()) {      
      return ESP_ERR_NOT_FOUND;
    }

    const char* sourceName = (_module != nullptr)
      ? _module->name().c_str()
      : "ArmorLink";

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_TELEMETRY,
      sourceName,
      _options.defaultLogTarget.c_str(),
      group ? group : "",
      name ? name : ""
    );

    setArmorLinkPacketPayload(out, payload);

    esp_err_t result = _transport.sendPacketToTarget(_options.defaultLogTarget, out);
    

    return result;
  }
  
  inline esp_err_t sendTelemetry(const char* group,
                                const char* name,
                                float value,
                                const char* unit = nullptr)
  {
    return telemetry(group, name, value, unit);
  }

  template<typename TSetLogStreamEnabled>
  bool handleDefaultBlePacket(
    const ArmorLinkPacket& packet,
    const String& payload,
    const char* localTarget,
    TSetLogStreamEnabled setLogStreamEnabled,
    void (*notifyAck)(uint16_t, const String&, const String&),
    void (*notifyError)(uint16_t, const String&, const String&),
    void (*notifyEventJson)(const String&),
    bool restartAfterPinSave = true)
  {
    if (handleSecurityPacket(
          packet,
          payload,
          "security",
          notifyAck,
          notifyError,
          notifyEventJson,
          restartAfterPinSave)) {
      return true;
    }

    if (enforceProvisioning(
          packet,
          "security",
          notifyError)) {
      return true;
    }

    return handlePacket(
      packet,
      payload,
      localTarget,
      setLogStreamEnabled,
      notifyAck,
      notifyError);
  }

private:
  void notifyLocalBleLog(
    const char* source,
    uint8_t level,
    const char* entity,
    const char* command,
    const String& message,
    int32_t valueInt,
    float valueFloat,
    float batteryVoltage) {
    if (!isBleLogStreamEnabled()) {
      return;
    }
    if (level < _remoteLogLevel) {
      return;
    }
    StaticJsonDocument<384> doc;
    doc["type"] = "log";
    doc["level"] = armorLinkLogLevelToString(level);
    doc["source"] = source ? source : "";
    doc["entity"] = entity ? entity : "";
    doc["command"] = command ? command : "";
    doc["message"] = message;
    doc["valueInt"] = valueInt;
    doc["valueFloat"] = valueFloat;
    doc["batteryVoltage"] = batteryVoltage;

    String out;
    serializeJson(doc, out);
    doc.clear();
    bleNotifyLogJson(out);
  }

  uint16_t _lastRespondedPairSessionId = 0;
  uint32_t _lastPairResponseMs = 0;
  ArmorLinkModule* _module = nullptr;
  ArmorLinkOptions _options;
  ArmorLinkStorage _storage;
  ArmorLinkDispatch _dispatch;
#if ARMORLINK_HAS_ESP32_SERVO
  Servo _serialServo;
#endif
  int _serialServoPin = -1;
  ArmorLinkTransportEspNow _transport;
  ArmorLinkPeerRegistry _peers;
  ArmorLinkEspNowCommandHook _espNowCommandHook = nullptr;
  ArmorLinkSerialRuntime _serialRuntime;
  friend class ArmorLinkTelemetryBuilder;
  friend class ArmorLinkDataStreamBuilder;
  bool _localConfigTransferPending = false;
  bool _localConfigTransferActive = false;
  uint16_t _localConfigTransferRequestId = 0;
  char _localConfigTransferTarget[ARMORLINK_NAME_MAX_LEN + 1] = { 0 };
#if ARMORLINK_HAS_ESPNOW
  ArmorLinkPacket _pendingPairResponse{};
  volatile bool _hasPendingPairResponse = false;
#endif
  bool _remoteTelemetryEnabled = false;
  uint32_t _telemetryMinIntervalMs = 100;
  bool _serialRebootRequested = false;
  uint32_t _serialRebootAtMs = 0;

  struct ArmorLinkSystemConfig {
    String nodeName;
    bool gatewayMode = false;
    bool bluetoothEnabled = false;
    String bluetoothName;
    bool wirelessEnabled = true;
    int wirelessChannel = 1;
  };

  ArmorLinkSystemConfig _systemConfig;

  void initializeSystemConfig(
      const ArmorLinkModule& module,
      const ArmorLinkOptions& options) {
    _systemConfig.nodeName =
        normalizeSystemName(
            options.nodeName.isEmpty()
                ? module.name()
                : options.nodeName,
            module.name().isEmpty() ? String("ArmorLink") : module.name());

    _systemConfig.gatewayMode = options.enableGateway;
    _systemConfig.bluetoothEnabled = options.enableBle;
    _systemConfig.bluetoothName =
        normalizeSystemName(options.bleName, "ArmorLink");
    _systemConfig.wirelessEnabled = options.enableEspNow;
    _systemConfig.wirelessChannel = options.espNowChannel;
  }

  void registerSystemConfig(ArmorLinkModule& module) {
    if (!module.config().containsKey("nodeName")) {
      module.config().addString("nodeName", &_systemConfig.nodeName, _systemConfig.nodeName)
          .label("Node Name").section("ArmorLink")
          .tooltip("Short name shown to ArmorLink setup tools and nearby nodes.")
          .rebootRequired();
    }

#if ARMORLINK_HAS_ESPNOW
    if (!module.config().containsKey("bridgeMode")) {
      module.config().addBool("bridgeMode", &_systemConfig.gatewayMode, _systemConfig.gatewayMode)
          .label("Gateway Mode").section("ArmorLink")
          .tooltip("Allows this node to connect ArmorLink devices together. Use only one Gateway in an ArmorLink network.")
          .onBoolChange([this](bool enabled) {
            if (enabled) {
              _systemConfig.wirelessEnabled = true;
            }
          })
          .rebootRequired();
    }
#endif
#if ARMORLINK_HAS_BLE
    if (!module.config().containsKey("bluetoothSetup")) {
      module.config().addBool("bluetoothSetup", &_systemConfig.bluetoothEnabled, _systemConfig.bluetoothEnabled)
          .label("Enable Bluetooth").section("ArmorLink")
          .tooltip("Allows phones and setup tools to discover this node over Bluetooth.")
          .rebootRequired();
    }

    if (!module.config().containsKey("bluetoothName")) {
      module.config().addString("bluetoothName", &_systemConfig.bluetoothName, _systemConfig.bluetoothName)
          .label("Bluetooth Name").section("ArmorLink")
          .tooltip("Name shown when pairing or discovering this node over Bluetooth.")
          .visibleWhen("bluetoothSetup", true)
          .rebootRequired();
    }
#endif
#if ARMORLINK_HAS_ESPNOW
    if (!module.config().containsKey("nodeLink")) {
      module.config().addBool("nodeLink", &_systemConfig.wirelessEnabled, _systemConfig.wirelessEnabled)
          .label("Enable Wireless Communication").section("ArmorLink")
          .tooltip("Enables wireless communication with other ArmorLink nodes.")
          .rebootRequired();
    }

    if (!module.config().containsKey("nodeLinkChannel")) {
      module.config().addInt("nodeLinkChannel", &_systemConfig.wirelessChannel, _systemConfig.wirelessChannel)
          .label("Channel").section("ArmorLink")
          .tooltip("Advanced: nearby ArmorLink nodes must use the same channel.")
          .visibleWhen("nodeLink", true)
          .advanced()
          .rebootRequired()
          .range(1, 13).step(1);
    }
#endif
  }

  void registerSystemActions(ArmorLinkModule& module) {
#if ARMORLINK_HAS_ESPNOW
    const char* confirmation = _isGatewayMode
        ? "Remove all ESP-NOW node pairings from this gateway? Reachable nodes will be notified. Offline nodes may need their pairing reset separately. Bluetooth PIN, settings, profiles and Actions are kept."
        : "Remove this node's gateway pairing so it can connect to a new gateway? Bluetooth PIN, settings, profiles and Actions are kept.";
    const char* description = _isGatewayMode
        ? "Remove all paired ESP-NOW nodes from this gateway."
        : "Remove only this node's gateway pairing.";
    // Reuse the reserved action and refresh its text when the runtime role changes.
    ArmorLinkActionDef* existing = findAction("pairing", "reset");
    if (existing != nullptr) {
      existing->label = "Reset Pairing info";
      existing->section = "ArmorLink";
      existing->enabled = true;
      existing->description = description;
      existing->confirmText = confirmation;
      existing->callback = [this]() { resetPairingInfo(); };
      return;
    }
    module.actions().add("resetPairingInfo")
        .label("Reset Pairing info").section("ArmorLink")
        .description(description)
        .command("pairing", "reset")
        .confirm(confirmation)
        .onExecute([this]() { resetPairingInfo(); });
#endif
  }

  void applySystemConfig() {
    if (_module == nullptr) {
      return;
    }

    _systemConfig.nodeName =
        normalizeSystemName(
            _systemConfig.nodeName,
            _module->name().isEmpty() ? String("ArmorLink") : _module->name());
    _systemConfig.bluetoothName =
        normalizeSystemName(_systemConfig.bluetoothName, "ArmorLink");

    if (_systemConfig.gatewayMode) {
      _systemConfig.wirelessEnabled = true;
    }

    _module->name(_systemConfig.nodeName);
    _systemConfig.wirelessEnabled = ARMORLINK_HAS_ESPNOW && _systemConfig.wirelessEnabled;
    _systemConfig.bluetoothEnabled = ARMORLINK_HAS_BLE && _systemConfig.bluetoothEnabled;
    _options.enableGateway = _systemConfig.gatewayMode;
    _options.enableBle = _systemConfig.bluetoothEnabled;
    _options.bleName = _systemConfig.bluetoothName;
    _options.enableEspNow = _systemConfig.wirelessEnabled;
    _options.espNowChannel =
        static_cast<uint8_t>(
            constrain(_systemConfig.wirelessChannel, 1, 13));
  }

  static String normalizeSystemName(
      const String& value,
      const String& fallback) {
    String result = value;
    result.trim();

    if (result.isEmpty()) {
      result = fallback;
      result.trim();
    }

    return result;
  }

  struct TelemetryRateEntry {
    char key[32];
    uint32_t lastSentMs;
  };

  struct DataStreamRateEntry {
    char key[48];
    uint32_t lastSentMs;
  };

  void serialTick() {
    if (!_options.enableSerialCommands &&
        !_options.enableSerialMenu) {
        return;
    }

      _serialRuntime.loop();
  }

  void rebootTick() {
      if (!_serialRebootRequested) {
          return;
      }

      if (static_cast<int32_t>(millis() - _serialRebootAtMs) < 0) {
          return;
      }

      AL_INTERNAL_PRINTLN("[SYSTEM] Restarting now");
      Serial.flush();
      ESP.restart();
  }

  static bool handleFlasherSerialPayloadThunk(
      void* context,
      const String& payload
  ) {
      ArmorLinkRuntime* runtime =
          static_cast<ArmorLinkRuntime*>(context);

      return runtime
          ? runtime->handleFlasherSerialPayload(payload)
          : false;
  }

  static bool handleUnknownSerialLineThunk(
      void* context,
      const String& line
  ) {
      ArmorLinkRuntime* runtime =
          static_cast<ArmorLinkRuntime*>(context);

      return runtime
          ? runtime->handleUnknownSerialLine(line)
          : false;
  }

#if ARMORLINK_HAS_WEB_SERIAL
  bool _serialWebClientActive = false;
  uint32_t _serialWebClientLastSeenMs = 0;

  bool isSerialWebClientActive() const {
    return _serialWebClientActive &&
        (uint32_t)(millis() - _serialWebClientLastSeenMs) < 30000UL;
  }
#endif

 bool handleFlasherSerialPayload(
    const String& payload
  ) {
#if ARMORLINK_HAS_WEB_SERIAL
      if (payload.isEmpty()) {
          sendSerialFlasherError(
              0,
              "empty_payload",
              "Flasher payload is empty"
          );
          return true;
      }

      StaticJsonDocument<2048> doc;

      const DeserializationError error =
          deserializeJson(doc, payload);

      if (error) {
          sendSerialFlasherError(
              0,
              "invalid_json",
              error.c_str()
          );
          return true;
      }

      const String type =
          String((const char*)(doc["type"] | ""));

      if (type.equalsIgnoreCase("session_close")) {
          _serialWebClientActive = false;
          return true;
      }
      // Any valid protocol request also starts/renews the session, preserving
      // compatibility with clients that do not yet send explicit keepalives.
      _serialWebClientActive = true;
      _serialWebClientLastSeenMs = millis();
      if (type.equalsIgnoreCase("session_open") || type.equalsIgnoreCase("session_ping")) {
          return true;
      }

      const uint16_t requestId =
          doc["requestId"] | 0;

      if (type.equalsIgnoreCase("config_get")) {
          return handleSerialConfigGet(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("config_set")) {
          return handleSerialConfigSet(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("profile_set")) {
          return handleSerialProfileSet(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("action_execute")) {
          return handleSerialActionExecute(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("action_definition_set")) {
          return handleSerialActionDefinitionSet(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("action_definition_preview")) {
          return handleSerialActionDefinitionSet(doc, requestId, true);
      }
      if (type.equalsIgnoreCase("action_definition_delete")) {
          return handleSerialActionDefinitionDelete(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("action_component_set")) {
          return handleSerialActionComponentSet(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("action_component_delete")) {
          return handleSerialActionComponentDelete(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("command")) {
          return handleSerialCommand(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("servo_move")) {
          return handleSerialServoMove(
              doc,
              requestId
          );
      }
      if (type.equalsIgnoreCase("reboot")) {
          return handleSerialReboot(
              doc,
              requestId
          );
      }
      sendSerialFlasherError(
          requestId,
          "unsupported_type",
          type.isEmpty()
              ? "Missing message type"
              : String("Unsupported message type: ") + type
      );

      return true;
#else
    return false;
#endif
  }
ArmorLinkActionDef* findAction(
    const String& entity,
    const String& command
) {
    if (_module == nullptr) {
        return nullptr;
    }

    for (auto& action : _module->actions().items()) {
        if (action.entity.equalsIgnoreCase(entity) &&
            action.command.equalsIgnoreCase(command)) {
            return &action;
        }
    }

    return nullptr;
}
ArmorLinkConfigFieldDef* findConfigField(
    const String& entity,
    const String& command
) {
    if (_module == nullptr) {
        return nullptr;
    }

    for (auto& field : _module->config().items()) {
        const bool explicitCommandMatch =
            field.entity.equalsIgnoreCase(entity) &&
            field.command.equalsIgnoreCase(command);

        const bool defaultConfigMatch =
            entity.equalsIgnoreCase("config") &&
            field.key.equalsIgnoreCase(command);

        const bool keyFallbackMatch =
            field.key.equalsIgnoreCase(command) ||
            field.key.equalsIgnoreCase(entity);

        if (explicitCommandMatch ||
            defaultConfigMatch ||
            keyFallbackMatch) {
            return &field;
        }
    }

    return nullptr;
}

bool readConfigFieldValue(
    const ArmorLinkConfigFieldDef& field,
    int32_t& value
) {
    switch (field.kind) {
        case ArmorLinkFieldKind::Int:
            if (field.intBinding.ptr == nullptr) {
                return false;
            }

            value = static_cast<int32_t>(
                *field.intBinding.ptr
            );

            return true;

        case ArmorLinkFieldKind::Bool:
            if (field.boolBinding.ptr == nullptr) {
                return false;
            }

            value = *field.boolBinding.ptr ? 1 : 0;
            return true;

        default:
            return false;
    }
}

bool readConfigFieldValue(
    const ArmorLinkConfigFieldDef& field,
    String& value
) {
    if (field.kind != ArmorLinkFieldKind::String ||
        field.stringBinding.ptr == nullptr) {
        return false;
    }

    value = *field.stringBinding.ptr;
    return true;
}

bool readConfigFieldValue(
    const ArmorLinkConfigFieldDef& field,
    float& value
) {
    if (field.kind != ArmorLinkFieldKind::Float ||
        field.floatBinding.ptr == nullptr) {
        return false;
    }

    value = *field.floatBinding.ptr;
    return true;
}

ArmorLinkDispatchResult dispatchConfigSetPacket(
    const String& entity,
    const String& command,
    const ArmorLinkPacket& packet,
    const String& payload
) {
    ArmorLinkConfigFieldDef* field =
        findConfigField(entity, command);

    if (field == nullptr) {
        return ArmorLinkDispatchResult::NotFound;
    }

    if (!field->editable) {
        return ArmorLinkDispatchResult::NotEditable;
    }

    switch (field->kind) {
        case ArmorLinkFieldKind::String:
            return _dispatch.handleConfigSet(
                entity,
                command,
                payload
            );

        case ArmorLinkFieldKind::Float: {
            const float value =
                payload.isEmpty()
                    ? packet.valueFloat
                    : payload.toFloat();
            return _dispatch.handleConfigSet(
                entity,
                command,
                value
            );
        }

        case ArmorLinkFieldKind::Int: {
            const int32_t value =
                payload.isEmpty()
                    ? packet.valueInt
                    : payload.toInt();
            return _dispatch.handleConfigSet(
                entity,
                command,
                value
            );
        }

        case ArmorLinkFieldKind::Bool: {
            bool value = packet.valueInt != 0;

            if (!payload.isEmpty()) {
                value =
                    payload.equalsIgnoreCase("true") ||
                    payload == "1" ||
                    payload.equalsIgnoreCase("yes") ||
                    payload.equalsIgnoreCase("on");
            }

            return _dispatch.handleConfigSet(
                entity,
                command,
                static_cast<int32_t>(value ? 1 : 0)
            );
        }

        case ArmorLinkFieldKind::Readonly:
        default:
            return ArmorLinkDispatchResult::NotEditable;
    }
}

void sendSerialActionAck(
    uint16_t requestId,
    const ArmorLinkActionDef& action
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "action_execute";

    doc["actionId"] = action.id;
    doc["entity"] = action.entity;
    doc["command"] = action.command;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

void sendSerialConfigSetAck(
    uint16_t requestId,
    const String& entity,
    const String& command,
    const String& requestedValue,
    const String& previousValue,
    const String& appliedValue
) {
    StaticJsonDocument<512> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "config_set";

    doc["entity"] = entity;
    doc["command"] = command;
    doc["requestedValue"] = requestedValue;
    doc["previousValue"] = previousValue;
    doc["appliedValue"] = appliedValue;
    doc["clamped"] = false;
    doc["changed"] = previousValue != appliedValue;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}
void sendSerialConfigSetAck(
    uint16_t requestId,
    const String& entity,
    const String& command,
    int32_t requestedValue,
    int32_t previousValue,
    int32_t appliedValue,
    ArmorLinkFieldKind fieldKind
) {
    StaticJsonDocument<512> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "config_set";

    doc["entity"] = entity;
    doc["command"] = command;

    if (fieldKind == ArmorLinkFieldKind::Bool) {
        const bool requestedBool =
            requestedValue != 0;

        const bool previousBool =
            previousValue != 0;

        const bool appliedBool =
            appliedValue != 0;

        doc["requestedValue"] = requestedBool;
        doc["previousValue"] = previousBool;
        doc["appliedValue"] = appliedBool;

        doc["clamped"] =
            requestedBool != appliedBool;

        doc["changed"] =
            previousBool != appliedBool;
    } else {
        doc["requestedValue"] = requestedValue;
        doc["previousValue"] = previousValue;
        doc["appliedValue"] = appliedValue;

        doc["clamped"] =
            requestedValue != appliedValue;

        doc["changed"] =
            previousValue != appliedValue;
    }

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

void sendSerialServoMoveAck(
    uint16_t requestId,
    const String& servoId,
    int requestedPosition,
    int appliedPosition,
    int pin
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "servo_move";
    doc["servo"] = servoId;
    doc["requestedPosition"] = requestedPosition;
    doc["appliedPosition"] = appliedPosition;
    doc["clamped"] = requestedPosition != appliedPosition;
    doc["pin"] = pin;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

void sendSerialActionDefinitionAck(
    uint16_t requestId,
    const ArmorLinkActionDefinition& definition
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "action_definition_set";
    doc["entity"] = definition.entity;
    doc["command"] = definition.command;
    doc["opCount"] = definition.ops.size();

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

void sendSerialActionDefinitionDeleteAck(
    uint16_t requestId,
    const String& entity,
    const String& command
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "action_definition_delete";
    doc["entity"] = entity;
    doc["command"] = command;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

void sendSerialActionComponentAck(
    uint16_t requestId,
    const ArmorLinkActionComponentDef& component
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "action_component_set";
    doc["id"] = component.id;
    doc["label"] = component.label;
    doc["section"] = component.section;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

void sendSerialActionComponentDeleteAck(
    uint16_t requestId,
    const String& id
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = "ok";
    doc["operation"] = "action_component_delete";
    doc["id"] = id;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

bool handleSerialActionComponentSet(
    JsonDocument& doc,
    uint16_t requestId
) {
#if ARMORLINK_HAS_ACTIONBUILDER
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    JsonObject componentJson = doc["component"].as<JsonObject>();
    if (componentJson.isNull()) {
        sendSerialFlasherError(
            requestId,
            "missing_component",
            "Action component is missing"
        );

        return true;
    }

    const String id = String((const char*)(componentJson["id"] | ""));
    const String kindName = String((const char*)(componentJson["kind"] | ""));
    const String label = String((const char*)(componentJson["label"] | ""));
    const String section = String((const char*)(componentJson["section"] | ""));
    bool defaultValue = componentJson["defaultValue"] | false;
    if (!componentJson["d"].isNull()) {
        defaultValue = componentJson["d"] | false;
    }
    const int servoPin = componentJson["pin"] | -1;
    const int servoOpenPosition = componentJson["openPosition"] | 90;
    const int servoClosedPosition = componentJson["closedPosition"] | 0;
    const int servoMinPulseUs = componentJson["minPulseUs"] | 500;
    const int servoMaxPulseUs = componentJson["maxPulseUs"] | 2400;
    const String servoProfile =
        String((const char*)(componentJson["servoProfile"] | ""));
    const int ledPin = componentJson["ledPin"] | -1;
    const int ledCount = componentJson["ledCount"] | 1;
    const int ledStartIndex = componentJson["ledStartIndex"] | 0;
    const int ledBrightness = componentJson["ledBrightness"] | 128;
    const String ledChipset =
        String((const char*)(componentJson["ledChipset"] | "WS2812B"));
    const String ledColorOrder =
        String((const char*)(componentJson["ledColorOrder"] | "GRB"));
    const int gpioPin = componentJson["gpioPin"] | -1;
    const String gpioMode =
        String((const char*)(componentJson["gpioMode"] | "output"));
    const bool gpioDefaultLevel = componentJson["gpioDefaultLevel"] | false;
    const bool gpioActiveLow = componentJson["gpioActiveLow"] | false;
    const int buttonPin = componentJson["buttonPin"] | -1;
    const bool buttonActiveLow = componentJson["buttonActiveLow"] | true;
    const int buttonDebounceMs = componentJson["buttonDebounceMs"] | 30;
    const int buttonMultiGapMs = componentJson["buttonMultiGapMs"] | 300;
    const int buttonLongPressMs = componentJson["buttonLongPressMs"] | 500;
    const ArmorLinkActionComponentKind kind =
        actionComponentKindFromString(kindName);

    if (id.isEmpty() ||
        !isValidActionComponentKind(kindName) ||
        (kind == ArmorLinkActionComponentKind::Servo &&
            servoMinPulseUs >= servoMaxPulseUs)) {
        sendSerialFlasherError(
            requestId,
            "invalid_component",
            "Action component id or kind is invalid"
        );

        return true;
    }

    ArmorLinkActionComponentDef& component =
        _module->actionBuilder().upsertCustomComponent(
            id,
            kind,
            label,
            section,
            defaultValue);

    if (kind == ArmorLinkActionComponentKind::Servo) {
        _module->actionBuilder().setServoConfig(
            component,
            servoPin,
            constrain(servoOpenPosition, 0, 180),
            constrain(servoClosedPosition, 0, 180),
            constrain(servoMinPulseUs, 400, 1500),
            constrain(servoMaxPulseUs, 1500, 3000),
            servoProfile);
    }

    if (kind == ArmorLinkActionComponentKind::Led) {
        _module->actionBuilder().setLedConfig(
            component,
            constrain(ledPin, -1, 48),
            constrain(ledCount, 1, 1024),
            constrain(ledStartIndex, 0, 1023),
            constrain(ledBrightness, 0, 255),
            ledChipset,
            ledColorOrder);
    }

    if (kind == ArmorLinkActionComponentKind::Gpio) {
        _module->actionBuilder().setGpioConfig(
            component,
            constrain(gpioPin, -1, 48),
            gpioMode,
            gpioDefaultLevel,
            gpioActiveLow);
    }

    if (kind == ArmorLinkActionComponentKind::Button) {
        _module->actionBuilder().setButtonConfig(
            component,
            constrain(buttonPin, -1, 48),
            buttonActiveLow,
            buttonDebounceMs,
            buttonMultiGapMs,
            buttonLongPressMs);
        _module->actionBuilder().clearButtonEventRoutes(component);

        JsonArray buttonEvents =
            componentJson["buttonEvents"].as<JsonArray>();

        for (JsonObject routeJson : buttonEvents) {
            const String event =
                String((const char*)(routeJson["event"] | ""));
            const String mode =
                String((const char*)(routeJson["mode"] | "sendCommand"));
            const bool enabled = routeJson["enabled"] | true;
            const String action =
                String((const char*)(routeJson["action"] | ""));
            const String target =
                String((const char*)(routeJson["target"] | ""));
            const String entity =
                String((const char*)(routeJson["entity"] | ""));
            const String command =
                String((const char*)(routeJson["command"] | ""));

            if (!event.isEmpty()) {
                _module->actionBuilder().addButtonEventRoute(
                    component,
                    event,
                    mode,
                    enabled,
                    action,
                    target,
                    entity,
                    command);
            }
        }
    }

    if (!_storage.saveActionComponents(_module->actionBuilder())) {
        sendSerialFlasherError(
            requestId,
            "storage_error",
            "Action component could not be saved"
        );

        return true;
    }

    sendSerialActionComponentAck(requestId, component);
    return true;
#else
    sendSerialFlasherError(requestId, "unsupported_feature", "Dynamic actions are excluded from this firmware"); return true;
#endif
  }

bool handleSerialActionComponentDelete(
    JsonDocument& doc,
    uint16_t requestId
) {
#if ARMORLINK_HAS_ACTIONBUILDER
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    const String id = String((const char*)(doc["id"] | ""));
    if (id.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_component",
            "Action component id is missing"
        );

        return true;
    }

    const int index = _module->actionBuilder().findComponentIndex(id);
    if (index < 0) {
        sendSerialFlasherError(
            requestId,
            "component_not_found",
            "Action component was not found"
        );

        return true;
    }

    if (!_module->actionBuilder().components()[index].custom) {
        sendSerialFlasherError(
            requestId,
            "firmware_component_readonly",
            "Firmware-defined action components cannot be deleted from WebSerial"
        );

        return true;
    }

    if (!_module->actionBuilder().removeCustomComponent(id) ||
        !_storage.deleteActionComponent(_module->actionBuilder())) {
        sendSerialFlasherError(
            requestId,
            "storage_error",
            "Action component could not be deleted"
        );

        return true;
    }

    sendSerialActionComponentDeleteAck(requestId, id);
    return true;
#else
    sendSerialFlasherError(requestId, "unsupported_feature", "Dynamic actions are excluded from this firmware"); return true;
#endif
  }

bool handleSerialActionDefinitionSet(
    JsonDocument& doc,
    uint16_t requestId,
    bool preview = false
) {
#if ARMORLINK_HAS_ACTIONBUILDER
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    JsonObject definitionJson = doc["definition"].as<JsonObject>();
    if (definitionJson.isNull()) {
        sendSerialFlasherError(
            requestId,
            "missing_definition",
            "Action definition is missing"
        );

        return true;
    }

    ArmorLinkActionDefinition definition;
    definition.entity = String(
        (const char*)(definitionJson["entity"] | ""));
    if (definition.entity.isEmpty()) {
        definition.entity = String(
            (const char*)(definitionJson["e"] | ""));
    }

    definition.command = String(
        (const char*)(definitionJson["command"] | ""));
    if (definition.command.isEmpty()) {
        definition.command = String(
            (const char*)(definitionJson["c"] | ""));
    }
    definition.label = String(
        (const char*)(definitionJson["label"] | ""));
    if (definition.label.isEmpty()) {
        definition.label = String(
            (const char*)(definitionJson["l"] | ""));
    }
    definition.section = String(
        (const char*)(definitionJson["section"] | ""));
    if (definition.section.isEmpty()) {
        definition.section = String(
            (const char*)(definitionJson["s"] | ""));
    }

    const auto* existingDefinition = _module->actionBuilder().findDefinition(definition.entity, definition.command);
    definition.startup = definitionJson["startup"] | (existingDefinition != nullptr && existingDefinition->startup);
    if (definition.startup && (definition.entity.indexOf('|') >= 0 || definition.command.indexOf('|') >= 0)) {
        sendSerialFlasherError(requestId, "invalid_action", "Startup action identity must not contain a pipe character");
        return true;
    }

    if (definition.entity.isEmpty() || definition.command.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_action",
            "Action entity or command is missing"
        );

        return true;
    }

    const bool isCustomAction =
        findAction(definition.entity, definition.command) == nullptr;

    if (!preview && !isCustomAction) {
        sendSerialFlasherError(
            requestId,
            "firmware_action_readonly",
            "Firmware-defined actions cannot be changed from WebSerial"
        );

        return true;
    }

    if (!preview && isCustomAction &&
        _module->actionBuilder().findCustomAction(definition.entity, definition.command) == nullptr) {
        _module->actionBuilder().upsertCustomAction(
            definition.entity,
            definition.command,
            definition.label,
            definition.section);
    }

    JsonArray ops = definitionJson["operations"].as<JsonArray>();
    if (ops.isNull()) {
        ops = definitionJson["o"].as<JsonArray>();
    }
    if (ops.isNull() || ops.size() == 0 || ops.size() > 63) {
        sendSerialFlasherError(
            requestId,
            "invalid_operations",
            "Action definition must contain 1 to 63 operations"
        );

        return true;
    }

    definition.ops.reserve(ops.size());

    for (JsonObject opJson : ops) {
        ArmorLinkActionOp op;
        String opName = String((const char*)(opJson["op"] | ""));
        if (opName.isEmpty()) {
            opName = String((const char*)(opJson["o"] | ""));
        }

        if (opName.equalsIgnoreCase("log_message")) {
            String text = String((const char*)(opJson["text"] | ""));
            String level = String((const char*)(opJson["level"] | "info"));
            ArmorLinkLogLevel logLevel;
            if (level == "debug") logLevel = AL_LOG_DEBUG;
            else if (level == "info") logLevel = AL_LOG_INFO;
            else if (level == "warn") logLevel = AL_LOG_WARN;
            else if (level == "error") logLevel = AL_LOG_ERROR;
            else {
                sendSerialFlasherError(requestId, "invalid_operation", "Log Message requires debug, info, warn or error");
                return true;
            }
            if (text.isEmpty() || text.length() > 128) {
                sendSerialFlasherError(requestId, "invalid_operation", "Log Message text must contain 1 to 128 UTF-8 bytes");
                return true;
            }
            auto message = std::make_shared<ArmorLinkActionLogMessage>();
            message->level = logLevel;
            message->text = text;
            op.code = ArmorLinkActionOpCode::LogMessage;
            op.logMessage = message;
        } else if (opName.equalsIgnoreCase("wait")) {
            op.code = ArmorLinkActionOpCode::Wait;
            op.componentIndex = 0xFF;
            int durationMs = opJson["durationMs"] | 0;
            if (durationMs <= 0) {
                durationMs = opJson["ms"] | 1;
            }
            op.argument = static_cast<uint16_t>(
                constrain(durationMs, 1, 60000));
        } else if (opName.equalsIgnoreCase("repeat_previous")) {
            int repeatCount = opJson["count"] | 0;
            if (repeatCount <= 0) {
                repeatCount = opJson["n"] | 1;
            }
            op.code = ArmorLinkActionOpCode::RepeatPrevious;
            op.componentIndex = 0xFF;
            op.argument = static_cast<uint16_t>(
                constrain(repeatCount, 1, 20));
        } else if (opName.equalsIgnoreCase("send_command_to_target") ||
                   opName.equalsIgnoreCase("send_action_to_target")) {
            String target = String((const char*)(opJson["target"] | ""));
            if (target.isEmpty()) {
                target = String((const char*)(opJson["t"] | ""));
            }

            String entity = String((const char*)(opJson["entity"] | ""));
            if (entity.isEmpty()) {
                entity = String((const char*)(opJson["e"] | ""));
            }

            String command = String((const char*)(opJson["command"] | ""));
            if (command.isEmpty()) {
                command = String((const char*)(opJson["c"] | ""));
            }

            if (target.isEmpty() ||
                entity.isEmpty() ||
                command.isEmpty()) {
                sendSerialFlasherError(
                    requestId,
                    "invalid_operation",
                    "Send Command requires target, entity and command"
                );

                return true;
            }

            ArmorLinkRemoteCommandDef& remoteCommand =
                _module->actionBuilder().upsertRemoteCommand(target, entity, command);
            const int remoteCommandIndex =
                _module->actionBuilder().findRemoteCommandIndex(remoteCommand.id);

            if (remoteCommandIndex < 0 || remoteCommandIndex > 65535) {
                sendSerialFlasherError(
                    requestId,
                    "invalid_operation",
                    "Send Command to Target could not be registered"
                );

                return true;
            }

            op.code = ArmorLinkActionOpCode::SendActionToTarget;
            op.componentIndex = 0xFF;
            op.argument = static_cast<uint16_t>(remoteCommandIndex);
        } else {
            op.code = actionOpCodeFromString(opName);
            String componentId =
                String((const char*)(opJson["component"] | ""));
            if (componentId.isEmpty()) {
                componentId = String((const char*)(opJson["c"] | ""));
            }
            String componentLabel =
                String((const char*)(opJson["componentLabel"] | ""));
            if (componentLabel.isEmpty()) {
                componentLabel = String((const char*)(opJson["cl"] | ""));
            }

            if (!componentId.isEmpty()) {
                if (op.code == ArmorLinkActionOpCode::WaitUntilState ||
                    op.code == ArmorLinkActionOpCode::StateSet) {
                    _module->actionBuilder().upsertCustomComponent(
                        componentId,
                        ArmorLinkActionComponentKind::State,
                        componentLabel,
                        definition.section);
                }
            }

            const int componentIndex =
                op.code == ArmorLinkActionOpCode::ActionCall
                    ? _module->actionBuilder().findActionTargetIndex(_module->actions(), componentId)
                    : _module->actionBuilder().findComponentIndex(componentId);

            if (static_cast<uint8_t>(op.code) == 0 ||
                componentIndex < 0 ||
                componentIndex > 254) {
                sendSerialFlasherError(
                    requestId,
                    "invalid_operation",
                    String("Invalid action operation: ") + opName
                );

                return true;
            }

            if ((op.code == ArmorLinkActionOpCode::ServoAttach || op.code == ArmorLinkActionOpCode::ServoDetach) &&
                (!_module->actionBuilder().supportsServoAttachDetach ||
                 _module->actionBuilder().components()[componentIndex].kind != ArmorLinkActionComponentKind::Servo)) {
                sendSerialFlasherError(requestId, "unsupported_operation", "Servo Attach/Detach requires capable firmware and a Servo component");
                return true;
            }
            op.componentIndex = static_cast<uint8_t>(componentIndex);

            if (op.code == ArmorLinkActionOpCode::WaitUntilState) {
                int timeoutMs = opJson["durationMs"] | 0;
                if (timeoutMs <= 0) {
                    timeoutMs = opJson["ms"] | 5000;
                }
                op.argument = static_cast<uint16_t>(
                    constrain(timeoutMs, 1, 60000));
            } else if (op.code == ArmorLinkActionOpCode::StateSet) {
                op.value = opJson["value"] | true;
                if (!opJson["v"].isNull()) {
                    op.value = opJson["v"] | true;
                }
            } else if (op.code == ArmorLinkActionOpCode::ServoMoveTo) {
                int position = opJson["position"] | -1;
                if (position < 0) {
                    position = opJson["p"] | 90;
                }
                op.argument = static_cast<uint16_t>(
                    constrain(position, 0, 180));
                int moveDurationMs = opJson["durationMs"] | 0;
                if (moveDurationMs <= 0) moveDurationMs = opJson["ms"] | 0;
                op.durationMs = static_cast<uint16_t>(constrain(moveDurationMs, 0, 60000));
                const String easing = String((const char*)(opJson["easing"] | "linear"));
                op.option = easing.equalsIgnoreCase("easeIn") ? 1 : easing.equalsIgnoreCase("easeOut") ? 2 : easing.equalsIgnoreCase("easeInOut") ? 3 : 0;
            } else if (op.code == ArmorLinkActionOpCode::LedSetColor) {
                String color = String((const char*)(opJson["color"] | "#FFFFFF"));
                if (color.startsWith("#")) color.remove(0, 1);
                op.color = static_cast<uint32_t>(strtoul(color.c_str(), nullptr, 16)) & 0xFFFFFF;
            } else if (op.code == ArmorLinkActionOpCode::LedSetBrightness) {
                op.intensity = static_cast<uint8_t>(constrain(opJson["brightness"] | 128, 0, 255));
            } else if (op.code == ArmorLinkActionOpCode::LedAnimation) {
                const String animation = String((const char*)(opJson["animation"] | "rainbow"));
                op.option = animation.equalsIgnoreCase("pulse") ? 1 : animation.equalsIgnoreCase("blink") ? 2 : animation.equalsIgnoreCase("chase") ? 3 : animation.equalsIgnoreCase("solid") ? 4 : 0;
                int animationSpeedMs = opJson["durationMs"] | 0;
                if (animationSpeedMs <= 0) animationSpeedMs = opJson["ms"] | 500;
                op.durationMs = static_cast<uint16_t>(constrain(animationSpeedMs, 25, 10000));
            }
        }

        definition.ops.push_back(op);
    }
    for (const auto& op : definition.ops) {
        if (op.code == ArmorLinkActionOpCode::LogMessage) {
            String ignored;
            if (!_module->actionBuilder().renderLogMessage(op.logMessage->text, ignored, false)) {
                sendSerialFlasherError(requestId, "invalid_operation", "Log Message contains an unknown or invalid {state:id} placeholder");
                return true;
            }
        }
    }


    if (_module->actionBuilder().wouldCreateCycle(_module->actions(), definition)) {
        sendSerialFlasherError(
            requestId,
            "recursive_action",
            "Action definition would create a recursive action call"
        );

        return true;
    }

    if (preview) {
        if (!_module->actionBuilder().executeDefinition(definition, _module->actions())) {
            sendSerialFlasherError(requestId, "preview_failed", "Action preview could not be started");
            return true;
        }
        sendSerialActionDefinitionAck(requestId, definition);
        return true;
    }

    if (!_storage.saveActionDefinition(
            definition,
            isCustomAction,
            &_module->actionBuilder()) ||
        !_module->actionBuilder().setDefinition(definition)) {
        sendSerialFlasherError(
            requestId,
            "storage_error",
            "Action definition could not be saved"
        );

        return true;
    }

    sendSerialActionDefinitionAck(requestId, definition);
    return true;
#else
    sendSerialFlasherError(requestId, "unsupported_feature", "Dynamic actions are excluded from this firmware"); return true;
#endif
  }

bool handleSerialActionDefinitionDelete(
    JsonDocument& doc,
    uint16_t requestId
) {
#if ARMORLINK_HAS_ACTIONBUILDER
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    const String entity = String((const char*)(doc["entity"] | ""));
    const String command = String((const char*)(doc["command"] | ""));

    if (entity.isEmpty() || command.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_action",
            "Action entity or command is missing"
        );

        return true;
    }

    if (findAction(entity, command) != nullptr) {
        sendSerialFlasherError(
            requestId,
            "firmware_action_readonly",
            "Firmware-defined actions cannot be deleted from WebSerial"
        );

        return true;
    }

    if (_module->actionBuilder().findCustomAction(entity, command) == nullptr) {
        sendSerialFlasherError(
            requestId,
            "action_not_found",
            "Custom action was not found"
        );

        return true;
    }

    _module->actionBuilder().removeDefinition(entity, command);
    _module->actionBuilder().removeCustomAction(entity, command);

    if (!_storage.deleteActionDefinition(entity, command)) {
        sendSerialFlasherError(
            requestId,
            "storage_error",
            "Action definition could not be deleted"
        );

        return true;
    }

    sendSerialActionDefinitionDeleteAck(requestId, entity, command);
    return true;
#else
    sendSerialFlasherError(requestId, "unsupported_feature", "Dynamic actions are excluded from this firmware"); return true;
#endif
  }

ArmorLinkActionOpCode actionOpCodeFromString(const String& opName) const {
    if (opName.equalsIgnoreCase("servo_attach")) return ArmorLinkActionOpCode::ServoAttach;
    if (opName.equalsIgnoreCase("servo_detach")) return ArmorLinkActionOpCode::ServoDetach;
    if (opName.equalsIgnoreCase("servo_open")) return ArmorLinkActionOpCode::ServoOpen;
    if (opName.equalsIgnoreCase("servo_close")) return ArmorLinkActionOpCode::ServoClose;
    if (opName.equalsIgnoreCase("servo_move_to")) return ArmorLinkActionOpCode::ServoMoveTo;
    if (opName.equalsIgnoreCase("led_on")) return ArmorLinkActionOpCode::LedOn;
    if (opName.equalsIgnoreCase("led_off")) return ArmorLinkActionOpCode::LedOff;
    if (opName.equalsIgnoreCase("led_set_color")) return ArmorLinkActionOpCode::LedSetColor;
    if (opName.equalsIgnoreCase("led_set_brightness")) return ArmorLinkActionOpCode::LedSetBrightness;
    if (opName.equalsIgnoreCase("led_animation")) return ArmorLinkActionOpCode::LedAnimation;
    if (opName.equalsIgnoreCase("gpio_high")) return ArmorLinkActionOpCode::GpioHigh;
    if (opName.equalsIgnoreCase("gpio_low")) return ArmorLinkActionOpCode::GpioLow;
    if (opName.equalsIgnoreCase("action_call")) return ArmorLinkActionOpCode::ActionCall;
    if (opName.equalsIgnoreCase("wait_until_state")) return ArmorLinkActionOpCode::WaitUntilState;
    if (opName.equalsIgnoreCase("wait_until_done")) return ArmorLinkActionOpCode::WaitUntilState;
    if (opName.equalsIgnoreCase("repeat_previous")) return ArmorLinkActionOpCode::RepeatPrevious;
    if (opName.equalsIgnoreCase("state_set")) return ArmorLinkActionOpCode::StateSet;
    if (opName.equalsIgnoreCase("preset_apply")) return ArmorLinkActionOpCode::StateSet;
    if (opName.equalsIgnoreCase("mode_set")) return ArmorLinkActionOpCode::StateSet;
    if (opName.equalsIgnoreCase("send_command_to_target")) return ArmorLinkActionOpCode::SendActionToTarget;
    if (opName.equalsIgnoreCase("send_action_to_target")) return ArmorLinkActionOpCode::SendActionToTarget;
    return static_cast<ArmorLinkActionOpCode>(0);
}

ArmorLinkActionComponentKind actionComponentKindFromString(
    const String& kindName
) const {
    if (kindName.equalsIgnoreCase("servo")) return ArmorLinkActionComponentKind::Servo;
    if (kindName.equalsIgnoreCase("led")) return ArmorLinkActionComponentKind::Led;
    if (kindName.equalsIgnoreCase("gpio")) return ArmorLinkActionComponentKind::Gpio;
    if (kindName.equalsIgnoreCase("button")) return ArmorLinkActionComponentKind::Button;
    if (kindName.equalsIgnoreCase("state")) return ArmorLinkActionComponentKind::State;
    if (kindName.equalsIgnoreCase("waitTarget")) return ArmorLinkActionComponentKind::State;
    if (kindName.equalsIgnoreCase("preset")) return ArmorLinkActionComponentKind::State;
    if (kindName.equalsIgnoreCase("mode")) return ArmorLinkActionComponentKind::State;
    return ArmorLinkActionComponentKind::Gpio;
}

bool isValidActionComponentKind(
    const String& kindName
) const {
    return kindName.equalsIgnoreCase("servo") ||
        kindName.equalsIgnoreCase("led") ||
        kindName.equalsIgnoreCase("gpio") ||
        kindName.equalsIgnoreCase("button") ||
        kindName.equalsIgnoreCase("state") ||
        kindName.equalsIgnoreCase("waitTarget") ||
        kindName.equalsIgnoreCase("preset") ||
        kindName.equalsIgnoreCase("mode");
}

bool handleSerialActionExecute(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    String requestedTarget =
        String((const char*)(doc["target"] | "*"));

    if (requestedTarget.isEmpty()) {
        requestedTarget = "*";
    }

    const String entity =
        String((const char*)(doc["entity"] | ""));

    const String command =
        String((const char*)(doc["command"] | ""));

    if (entity.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_entity",
            "Action entity is missing"
        );

        return true;
    }

    if (command.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_command",
            "Action command is missing"
        );

        return true;
    }

    if (!requestedTarget.equalsIgnoreCase("*") &&
        !requestedTarget.equalsIgnoreCase(_module->name())) {
        ArmorLinkPacket packet =
            makeArmorLinkBasePacket(
                AL_MSG_COMMAND,
                _module->name().c_str(),
                requestedTarget.c_str(),
                entity.c_str(),
                command.c_str()
            );

        packet.requestId = requestId;
        packet.flags = doc["flags"] | 0;

        return handlePacket(
            packet,
            "",
            _module->name().c_str(),
            [](bool enabled) {
                setBleLogStreamEnabled(enabled);
            },
            [this](uint16_t id, const String& status, const String& message) {
                sendSerialFlasherAck(id, status, message);
            },
            [this](uint16_t id, const String& code, const String& message) {
                sendSerialFlasherError(id, code, message);
            }
        );
    }

    ArmorLinkActionDef* action =
        findAction(
            entity,
            command
        );

#if ARMORLINK_HAS_ACTIONBUILDER
    if (action == nullptr) {
        action = _module->actionBuilder().findCustomAction(
            entity,
            command
        );
    }
#endif
    if (action == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_found",
            String("Action not found: ") +
                entity +
                "/" +
                command
        );

        return true;
    }

    if (!action->enabled) {
        sendSerialFlasherError(
            requestId,
            "action_disabled",
            String("Action is disabled: ") +
                action->id
        );

        return true;
    }

#if ARMORLINK_HAS_ACTIONBUILDER
    const bool hasActionDefinition =
        _module->actionBuilder().findDefinition(action->entity, action->command) != nullptr;

    if (hasActionDefinition && _module->actionBuilder().isRunning()) {
        sendSerialFlasherError(
            requestId,
            "action_busy",
            "Action sequence is already running"
        );

        return true;
    }

    if (_module->actionBuilder().execute(action->entity, action->command, _module->actions())) {
        sendSerialActionAck(
            requestId,
            *action
        );

        return true;
    }
#endif
    if (!action->callback) {
        sendSerialFlasherError(
            requestId,
            "handler_unavailable",
            String("Action has no handler: ") +
                action->id
        );

        return true;
    }

    const ArmorLinkDispatchResult actionResult =
        _dispatch.handleAction(
            action->entity,
            action->command
        );

    if (actionResult !=
        ArmorLinkDispatchResult::Ok) {

        sendSerialFlasherError(
            requestId,
            ArmorLinkDispatch::toString(
                actionResult
            ),
            String("Action execution failed: ") +
                action->id
        );

        return true;
    }

    sendSerialActionAck(
        requestId,
        *action
    );

    return true;
}

bool handleSerialServoMove(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    String requestedTarget =
        String((const char*)(doc["target"] | "*"));

    if (requestedTarget.isEmpty()) {
        requestedTarget = "*";
    }

    const String servoId =
        String((const char*)(doc["servo"] | ""));

    if (servoId.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_servo",
            "Servo id is missing"
        );

        return true;
    }

    if (!doc["position"].is<int>()) {
        sendSerialFlasherError(
            requestId,
            "missing_position",
            "Servo position is missing or invalid"
        );

        return true;
    }

    ArmorLinkServoConfig* servo =
        _module->config().findServoConfig(servoId);

    if (servo == nullptr) {
        sendSerialFlasherError(
            requestId,
            "servo_not_found",
            String("Servo config not found: ") +
                servoId
        );

        return true;
    }

    if (servo->pin < 0 || servo->pin > 48) {
        sendSerialFlasherError(
            requestId,
            "invalid_gpio",
            String("Servo GPIO is not configured: ") +
                servoId
        );

        return true;
    }

    const int requestedPosition =
        doc["position"].as<int>();

    const int appliedPosition =
        constrain(
            requestedPosition,
            0,
            180
        );

#if ARMORLINK_HAS_ESP32_SERVO
    if (_serialServoPin >= 0) {
        _serialServo.detach();
    }

    _serialServo.setPeriodHertz(50);
    _serialServo.attach(
        servo->pin,
        servo->minPulseUs,
        servo->maxPulseUs
    );

    _serialServoPin = servo->pin;
    _serialServo.write(appliedPosition);

    sendSerialServoMoveAck(
        requestId,
        servoId,
        requestedPosition,
        appliedPosition,
        servo->pin
    );
#else
    (void)appliedPosition;
    sendSerialFlasherError(
        requestId,
        "servo_unavailable",
        "ESP32Servo is not available in this build"
    );
#endif

    return true;
}

bool handleSerialReboot(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );
        return true;
    }

    String requestedTarget =
        String((const char*)(doc["target"] | "*"));

    if (requestedTarget.isEmpty()) {
        requestedTarget = "*";
    }

    sendSerialFlasherAck(
        requestId,
        "ok",
        "Rebooting device"
    );

    Serial.flush();

    _serialRebootRequested = true;
    _serialRebootAtMs = millis() + 250;

    return true;
}

bool handleSerialConfigSet(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    String requestedTarget =
        String((const char*)(doc["target"] | "*"));

    if (requestedTarget.isEmpty()) {
        requestedTarget = "*";
    }

    const String entity =
        String((const char*)(doc["entity"] | "config"));

    const String command =
        String((const char*)(doc["command"] | ""));

    if (command.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_command",
            "Config command is missing"
        );

        return true;
    }

    if (!doc.containsKey("value")) {
        sendSerialFlasherError(
            requestId,
            "missing_value",
            "Config value is missing"
        );

        return true;
    }

    if (!requestedTarget.equalsIgnoreCase("*") &&
        !requestedTarget.equalsIgnoreCase(_module->name())) {
        ArmorLinkPacket packet =
            makeArmorLinkBasePacket(
                AL_MSG_CONFIG_SET,
                _module->name().c_str(),
                requestedTarget.c_str(),
                entity.c_str(),
                command.c_str()
            );

        packet.requestId = requestId;
        packet.flags = doc["flags"] | 0;

        String payload;

        if (doc["value"].is<const char*>()) {
            payload = String((const char*)doc["value"]);
        } else if (doc["value"].is<bool>()) {
            const bool value = doc["value"].as<bool>();
            packet.valueInt = value ? 1 : 0;
            payload = value ? "true" : "false";
        } else if (doc["value"].is<int>() ||
                   doc["value"].is<long>() ||
                   doc["value"].is<unsigned int>() ||
                   doc["value"].is<unsigned long>()) {
            packet.valueInt = doc["value"].as<int32_t>();
            packet.valueFloat = static_cast<float>(packet.valueInt);
            payload = String(packet.valueInt);
        } else if (doc["value"].is<float>() ||
                   doc["value"].is<double>()) {
            packet.valueFloat = doc["value"].as<float>();
            packet.valueInt = static_cast<int32_t>(packet.valueFloat);
            payload = String(packet.valueFloat);
        }

        setArmorLinkPacketPayload(packet, payload);

        return handlePacket(
            packet,
            payload,
            _module->name().c_str(),
            [](bool enabled) {
                setBleLogStreamEnabled(enabled);
            },
            [this](uint16_t id, const String& status, const String& message) {
                sendSerialFlasherAck(id, status, message);
            },
            [this](uint16_t id, const String& code, const String& message) {
                sendSerialFlasherError(id, code, message);
            }
        );
    }

    ArmorLinkConfigFieldDef* field =
        findConfigField(
            entity,
            command
        );

    if (field == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_found",
            String("Config field not found: ") +
                entity +
                "/" +
                command
        );

        return true;
    }

    if (!field->editable) {
        sendSerialFlasherError(
            requestId,
            "not_editable",
            String("Config field is read-only: ") +
                command
        );

        return true;
    }

    if (field->kind == ArmorLinkFieldKind::String) {
        if (!doc["value"].is<const char*>()) {
            sendSerialFlasherError(
                requestId,
                "invalid_value_type",
                String("String config field requires ") +
                    "a string value: " +
                    command
            );

            return true;
        }

        String previousValue;

        if (!readConfigFieldValue(
                *field,
                previousValue
            )) {

            sendSerialFlasherError(
                requestId,
                "binding_unavailable",
                String("Config binding is unavailable: ") +
                    command
            );

            return true;
        }

        String requestedValue =
            String((const char*)(doc["value"] | ""));

        requestedValue.trim();

        const ArmorLinkDispatchResult configResult =
            _dispatch.handleConfigSet(
                entity,
                command,
                requestedValue
            );

        if (configResult !=
            ArmorLinkDispatchResult::Ok) {

            sendSerialFlasherError(
                requestId,
                ArmorLinkDispatch::toString(
                    configResult
                ),
                "Config update failed"
            );

            return true;
        }

        String appliedValue;

        if (!readConfigFieldValue(
                *field,
                appliedValue
            )) {

            sendSerialFlasherError(
                requestId,
                "binding_unavailable",
                String(
                    "Config was updated, but the applied "
                    "value could not be read: "
                ) + command
            );

            return true;
        }

        sendSerialConfigSetAck(
            requestId,
            entity,
            command,
            requestedValue,
            previousValue,
            appliedValue
        );

        return true;
    }

    if (field->kind == ArmorLinkFieldKind::Float) {
        if (!doc["value"].is<float>() &&
            !doc["value"].is<double>() &&
            !doc["value"].is<int>() &&
            !doc["value"].is<long>() &&
            !doc["value"].is<unsigned int>() &&
            !doc["value"].is<unsigned long>()) {
            sendSerialFlasherError(
                requestId,
                "invalid_value_type",
                String("Float config field requires ") +
                    "a numeric value: " +
                    command
            );

            return true;
        }

        float previousValue = 0.0f;

        if (!readConfigFieldValue(
                *field,
                previousValue
            )) {

            sendSerialFlasherError(
                requestId,
                "binding_unavailable",
                String("Config binding is unavailable: ") +
                    command
            );

            return true;
        }

        const float requestedValue =
            doc["value"].as<float>();

        const ArmorLinkDispatchResult configResult =
            _dispatch.handleConfigSet(
                entity,
                command,
                requestedValue
            );

        if (configResult !=
            ArmorLinkDispatchResult::Ok) {

            sendSerialFlasherError(
                requestId,
                ArmorLinkDispatch::toString(
                    configResult
                ),
                "Config update failed"
            );

            return true;
        }

        float appliedValue = 0.0f;

        if (!readConfigFieldValue(
                *field,
                appliedValue
            )) {

            sendSerialFlasherError(
                requestId,
                "binding_unavailable",
                String(
                    "Config was updated, but the applied "
                    "value could not be read: "
                ) + command
            );

            return true;
        }

        sendSerialConfigSetAck(
            requestId,
            entity,
            command,
            String(requestedValue, 3),
            String(previousValue, 3),
            String(appliedValue, 3)
        );

        return true;
    }

    int32_t requestedValue = 0;

    switch (field->kind) {
        case ArmorLinkFieldKind::Bool:
            if (doc["value"].is<bool>()) {
                requestedValue =
                    doc["value"].as<bool>()
                        ? 1
                        : 0;
            } else if (
                doc["value"].is<int>() ||
                doc["value"].is<long>() ||
                doc["value"].is<unsigned int>() ||
                doc["value"].is<unsigned long>()
            ) {
                requestedValue =
                    doc["value"].as<int32_t>();
            } else {
                sendSerialFlasherError(
                    requestId,
                    "invalid_value_type",
                    String("Boolean config field requires ") +
                        "a boolean or integer value: " +
                        command
                );

                return true;
            }

            break;

        case ArmorLinkFieldKind::Int:
            if (
                doc["value"].is<int>() ||
                doc["value"].is<long>() ||
                doc["value"].is<unsigned int>() ||
                doc["value"].is<unsigned long>()
            ) {
                requestedValue =
                    doc["value"].as<int32_t>();
            } else {
                sendSerialFlasherError(
                    requestId,
                    "invalid_value_type",
                    String("Integer config field requires ") +
                        "an integer value: " +
                        command
                );

                return true;
            }

            break;

        case ArmorLinkFieldKind::Readonly:
        default:
            sendSerialFlasherError(
                requestId,
                "not_editable",
                String("Config field is read-only: ") +
                    command
            );

            return true;
    }

    int32_t previousValue = 0;

    if (!readConfigFieldValue(
            *field,
            previousValue
        )) {

        sendSerialFlasherError(
            requestId,
            "binding_unavailable",
            String("Config binding is unavailable: ") +
                command
        );

        return true;
    }

    const ArmorLinkDispatchResult configResult =
        _dispatch.handleConfigSet(
            entity,
            command,
            requestedValue
        );

    if (configResult !=
        ArmorLinkDispatchResult::Ok) {

        sendSerialFlasherError(
            requestId,
            ArmorLinkDispatch::toString(
                configResult
            ),
            "Config update failed"
        );

        return true;
    }

    int32_t appliedValue = 0;

    if (!readConfigFieldValue(
            *field,
            appliedValue
        )) {

        sendSerialFlasherError(
            requestId,
            "binding_unavailable",
            String(
                "Config was updated, but the applied "
                "value could not be read: "
            ) + command
        );

        return true;
    }

    sendSerialConfigSetAck(
        requestId,
        entity,
        command,
        requestedValue,
        previousValue,
        appliedValue,
        field->kind
    );

    return true;
}

bool handleSerialProfileSet(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );

        return true;
    }

    if (!doc["name"].is<const char*>()) {
        sendSerialFlasherError(
            requestId,
            "missing_name",
            "Profile name is missing"
        );

        return true;
    }

    String requestedName =
        String((const char*)(doc["name"] | ""));
    String requestedIdentifier =
        String((const char*)(doc["identifier"] | ""));

    requestedName.trim();
    requestedIdentifier.trim();

    if (requestedName.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "invalid_name",
            "Profile name cannot be empty"
        );

        return true;
    }

    if (requestedName.length() > 64) {
        sendSerialFlasherError(
            requestId,
            "invalid_name",
            "Profile name is too long"
        );

        return true;
    }

    if (requestedIdentifier.length() > 180) {
        sendSerialFlasherError(
            requestId,
            "invalid_identifier",
            "Profile identifier is too long"
        );

        return true;
    }

    const String previousIdentifier =
        _module->profileIdentifier();
    const String previousName =
        _module->profileName();

    _module->activeProfile(requestedIdentifier, requestedName);

    if (!_storage.saveProfileMetadata(requestedIdentifier, requestedName)) {
        _module->activeProfile(previousIdentifier, previousName);

        sendSerialFlasherError(
            requestId,
            "storage_error",
            "Profile metadata could not be saved"
        );

        return true;
    }

    StaticJsonDocument<384> outDoc;

    outDoc["type"] = "ack";
    outDoc["requestId"] = requestId;
    outDoc["status"] = "ok";
    outDoc["operation"] = "profile_set";
    outDoc["previousIdentifier"] = previousIdentifier;
    outDoc["previousName"] = previousName;
    outDoc["profileIdentifier"] = requestedIdentifier;
    outDoc["profileName"] = requestedName;
    outDoc["changed"] =
        previousName != requestedName ||
        previousIdentifier != requestedIdentifier;

    String json;
    serializeJson(outDoc, json);

    emitSerialTransportJson(json);

    return true;
}

void sendSerialFlasherAck(
    uint16_t requestId,
    const String& status,
    const String& message
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = status;
    doc["message"] = message;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

bool handleSerialCommand(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );
        return true;
    }

    String target =
        String((const char*)(doc["target"] | "*"));

    if (target.isEmpty()) {
        target = "*";
    }

    const String entity =
        String((const char*)(doc["entity"] | ""));

    const String command =
        String((const char*)(doc["command"] | ""));

    if (entity.isEmpty() || command.isEmpty()) {
        sendSerialFlasherError(
            requestId,
            "missing_command",
            "Command entity or command is missing"
        );
        return true;
    }

    ArmorLinkPacket packet =
        makeArmorLinkBasePacket(
            AL_MSG_COMMAND,
            _module->name().c_str(),
            target.c_str(),
            entity.c_str(),
            command.c_str()
        );

    packet.requestId = requestId;
    packet.flags = doc["flags"] | 0;
    packet.valueInt = doc["valueInt"] | 0;
    packet.valueFloat = doc["valueFloat"] | 0.0f;
    packet.batteryVoltage = doc["batteryVoltage"] | 0.0f;

    String payload;

    if (doc["payload"].is<const char*>()) {
        payload = String((const char*)doc["payload"]);
    } else if (doc["value"].is<bool>()) {
        const bool value = doc["value"].as<bool>();
        packet.valueInt = value ? 1 : 0;
        payload = value ? "true" : "false";
    } else if (doc["value"].is<int>()) {
        packet.valueInt = doc["value"].as<int>();
        packet.valueFloat = static_cast<float>(packet.valueInt);
        payload = String(packet.valueInt);
    } else if (doc["value"].is<float>()) {
        packet.valueFloat = doc["value"].as<float>();
        packet.valueInt = static_cast<int32_t>(packet.valueFloat);
        payload = String(packet.valueFloat);
    } else if (doc["value"].is<const char*>()) {
        payload = String((const char*)doc["value"]);
    }

    setArmorLinkPacketPayload(packet, payload);

    return handlePacket(
        packet,
        payload,
        _module->name().c_str(),
        [](bool enabled) {
            setBleLogStreamEnabled(enabled);
        },
        [this](uint16_t id, const String& status, const String& message) {
            sendSerialFlasherAck(id, status, message);
        },
        [this](uint16_t id, const String& code, const String& message) {
            sendSerialFlasherError(id, code, message);
        }
    );
}

bool handleSerialConfigGet(
    JsonDocument& doc,
    uint16_t requestId
) {
    if (_module == nullptr) {
        sendSerialFlasherError(
            requestId,
            "not_initialized",
            "ArmorLink module is not initialized"
        );
        return true;
    }

    String requestedTarget =
        String((const char*)(doc["target"] | "*"));

    if (requestedTarget.isEmpty()) {
        requestedTarget = "*";
    }

    if (requestedTarget.equalsIgnoreCase("*") ||
        requestedTarget.equalsIgnoreCase(_module->name())) {
        sendSerialConfigDescriptor(
            requestId,
            *_module
        );

        return true;
    }

    ArmorLinkPacket packet =
        makeArmorLinkBasePacket(
            AL_MSG_CONFIG_GET,
            _module->name().c_str(),
            requestedTarget.c_str(),
            "config",
            "get"
        );

    packet.requestId = requestId;

    return handlePacket(
        packet,
        "",
        _module->name().c_str(),
        [](bool enabled) {
            setBleLogStreamEnabled(enabled);
        },
        [this](uint16_t id, const String& status, const String& message) {
            sendSerialFlasherAck(id, status, message);
        },
        [this](uint16_t id, const String& code, const String& message) {
            sendSerialFlasherError(id, code, message);
        }
    );
}

// A bounded sink shared by the measuring and sending passes. Never retains
// the complete descriptor; chunks are independently valid UTF-8 strings.
class ArmorLinkSerialDescriptorPrint : public Print {
public:
    using ChunkHandler = std::function<void(const char*)>;
    explicit ArmorLinkSerialDescriptorPrint(ChunkHandler handler = {}) : _handler(handler) {}

    size_t write(uint8_t value) override {
        if (_size == ChunkSize) {
            size_t end = _size;
            if ((value & 0xC0) == 0x80) {
                // The next byte continues the codepoint at the buffer's tail.
                do { --end; } while (end > 0 && (static_cast<uint8_t>(_buffer[end]) & 0xC0) == 0x80);
            }
            flushPrefix(end);
        }
        _buffer[_size++] = static_cast<char>(value);
        _length++;
        _hash = (_hash ^ value) * 16777619u;
        return 1;
    }

    size_t write(const uint8_t* buffer, size_t size) override {
        for (size_t i = 0; i < size; ++i) write(buffer[i]);
        return size;
    }

    void finish() { if (_size > 0) flushPrefix(_size); }
    size_t length() const { return _length; }
    size_t chunks() const { return _chunks; }
    uint32_t hash() const { return _hash; }

private:
    void flushPrefix(size_t size) {
        const char saved = _buffer[size];
        _buffer[size] = '\0';
        if (_handler) _handler(_buffer);
        _buffer[size] = saved;
        ++_chunks;
        _size -= size;
        memmove(_buffer, _buffer + size, _size);
    }

    static constexpr size_t ChunkSize = 192;
    ChunkHandler _handler;
    char _buffer[ChunkSize + 1] = {};
    size_t _size = 0;
    size_t _length = 0;
    size_t _chunks = 0;
    uint32_t _hash = 2166136261u;
};

void sendSerialConfigDescriptor(
    uint16_t requestId,
    const ArmorLinkModule& module
) {
#if ARMORLINK_HAS_WEB_SERIAL
    // Measure the complete WebSerial profile without allocating its JSON string.
    ArmorLinkSerialDescriptorPrint measured;
    if (!ArmorLinkDescriptor::writeWebSerial(module, measured)) {
        sendSerialFlasherError(requestId, "descriptor_build_failed",
                               "WebSerial config descriptor could not be measured");
        return;
    }
    measured.finish();

    auto sendFrame = [&](const char* type, size_t index, const char* data) {
        StaticJsonDocument<768> doc;
        doc["type"] = type;
        doc["requestId"] = requestId;
        doc["chunkCount"] = measured.chunks();
        doc["totalLength"] = measured.length();
#if ARMORLINK_HAS_BLE
        // USB-only metadata for the directly attached, provisioned gateway.
        // Never add credentials to shared BLE/ESP-NOW descriptors.
        if (strcmp(type, "config_meta") == 0 &&
            &module == _module && _isGatewayMode && ArmorLinkSecurity::isPinSet()) {
            doc["bluetoothPin"] = ArmorLinkSecurity::getPin();
        }
#endif
        if (strcmp(type, "config_chunk") == 0) {
            doc["chunkIndex"] = index;
            doc["data"] = data;
        }
        String frame;
        serializeJson(doc, frame);
        doc.clear();
        emitSerialTransportJson(frame);
    };

    sendFrame("config_meta", 0, "");
    size_t index = 0;
    ArmorLinkSerialDescriptorPrint output([&](const char* data) {
        sendFrame("config_chunk", index++, data);
    });
    const bool written = ArmorLinkDescriptor::writeWebSerial(module, output);
    output.finish();
    // Do not complete a transfer if asynchronous callbacks changed the data
    // between passes, even if its byte count happened to remain unchanged.
    if (!written || output.length() != measured.length() ||
        output.chunks() != measured.chunks() || output.hash() != measured.hash()) {
        sendSerialFlasherError(requestId, "descriptor_changed",
                               "Config descriptor changed during transfer; please retry");
        return;
    }
    sendFrame("config_end", 0, "");
#endif
}

void sendSerialFlasherError(
    uint16_t requestId,
    const String& code,
    const String& message
) {
    StaticJsonDocument<384> doc;

    doc["type"] = "error";
    doc["requestId"] = requestId;
    doc["code"] = code;
    doc["message"] = message;

    String json;
    serializeJson(doc, json);

    emitSerialTransportJson(json);
}

  bool handleUnknownSerialLine(
      const String& line
  ) {
      if (_options.enableSerialMenu &&
          _isGatewayMode) {
          handleSerialMenuCommand(line);
          return true;
      }

      return false;
  }

  void updateStoredModuleMetadataByMac(
    const String& macText,
    const String& moduleName,
    const String& moduleVersion,
    const String& armorLinkVersion)
  {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || macText.isEmpty()) {
      return;
    }

    bool changed = false;

    for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
      if (!macText.equalsIgnoreCase(_pairedModules[i].mac)) {
        continue;
      }

      if (!moduleName.isEmpty() && !moduleName.equals(_pairedModules[i].name)) {
        armorlinkCopyString(_pairedModules[i].name, sizeof(_pairedModules[i].name), moduleName.c_str());
        changed = true;
      }

      const char* finalModuleVersion = moduleVersion.isEmpty() ? "1.0" : moduleVersion.c_str();
      if (!String(_pairedModules[i].moduleVersion).equals(finalModuleVersion)) {
        armorlinkCopyString(_pairedModules[i].moduleVersion, sizeof(_pairedModules[i].moduleVersion), finalModuleVersion);
        changed = true;
      }

      if (!String(_pairedModules[i].armorLinkVersion).equals(armorLinkVersion)) {
        armorlinkCopyString(_pairedModules[i].armorLinkVersion, sizeof(_pairedModules[i].armorLinkVersion), armorLinkVersion.c_str());
        changed = true;
      }

      if (changed) {
        _storage.savePairedModules(_pairedModules, _pairedModuleCount);
        emitModulePresenceEvent(_pairedModules[i], _presenceStates[i].isOnline, 0);
      }

      return;
    }
#endif
  }

void handleSerialMenuCommand(String line) {
  line.trim();

  if (line.length() == 0) {
    return;
  }

  if (line.equalsIgnoreCase("help") || line == "?") {
    printSerialMenuHelp();
    return;
  }

  if (line.equalsIgnoreCase("start") || line.equalsIgnoreCase("startpairing")) {
    const bool ok = startPairing(30000);
    if (ok) Serial.println("[MENU] Pairing started.");
    else Serial.println("[MENU] Pairing start failed.");
    return;
  }

  if (line.equalsIgnoreCase("stop") || line.equalsIgnoreCase("stoppairing")) {
    stopPairing();
    Serial.println("[MENU] Pairing stopped.");
    return;
  }

  if (line.equalsIgnoreCase("candidates") || line.equalsIgnoreCase("list candidates")) {
    printPairingCandidates();
    return;
  }

  if (line.equalsIgnoreCase("modules") || line.equalsIgnoreCase("list")) {
    printPairedModules();
    return;
  }

  if (line.startsWith("pair ")) {
    String indexText = line.substring(5);
    indexText.trim();
    pairCandidateBySerialIndex(indexText.toInt());
    return;
  }

  if (line.startsWith("unpair ")) {
    String value = line.substring(7);
    value.trim();
    unpairModuleFromSerial(value);
    return;
  }

  Serial.println("[MENU] Unknown command. Type 'help'.");
}

void printSerialMenuHelp() {
  Serial.println();
  Serial.println("=== ArmorLink Serial Menu ===");
  Serial.println("help");
  Serial.println("start              - start pairing window");
  Serial.println("stop               - stop pairing window");
  Serial.println("candidates         - list pairing candidates");
  Serial.println("pair <number>      - pair candidate by number");
  Serial.println("modules            - list paired modules");
  Serial.println("unpair <number>    - unpair module by list number");
  Serial.println("unpair <mac>       - unpair module by MAC");
  Serial.println();
  Serial.println("Example:");
  Serial.println("start");
  Serial.println("candidates");
  Serial.println("pair 1");
  Serial.println("=============================");
}

void printPairingCandidates() {
#if ARMORLINK_HAS_ESPNOW
  Serial.println();
  Serial.println("=== Pairing Candidates ===");

  if (_pendingCandidateCount == 0) {
    Serial.println("No candidates.");
    return;
  }

  size_t displayIndex = 1;

  for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
    if (!_pendingCandidates[i].occupied) {
      continue;
    }

    Serial.printf(
      "%u) %s | %s | %s | session=%u\n",
      static_cast<unsigned>(displayIndex),
      _pendingCandidates[i].name,
      _pendingCandidates[i].type,
      _pendingCandidates[i].mac,
      _pendingCandidates[i].sessionId
    );

    displayIndex++;
  }
#endif
  }

  void printPairedModules() {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode) {
      Serial.println("[MENU] Not in gateway mode.");
      return;
    }

    _pairedModuleCount = _storage.loadPairedModules(_pairedModules);
    syncPresenceStatesFromStoredModules();

    Serial.println();
    Serial.println("=== Paired Modules ===");

    if (_pairedModuleCount == 0) {
      Serial.println("No modules paired.");
      return;
    }

    for (size_t i = 0; i < _pairedModuleCount; ++i) {
      Serial.printf(
        "%u) %s | %s | %s\n",
        static_cast<unsigned>(i + 1),
        _pairedModules[i].name,
        _pairedModules[i].type,
        _pairedModules[i].mac
      );
    }

    Serial.println("======================");
#endif
  }

  bool sendActionBuilderCommandToTarget(
      const String& target,
      const String& entity,
      const String& command) {
    if (_module == nullptr ||
        target.isEmpty() ||
        entity.isEmpty() ||
        command.isEmpty()) {
      return false;
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      target.c_str(),
      entity.c_str(),
      command.c_str()
    );

    return _transport.sendPacketToTarget(target, out) == ESP_OK;
  }

  static constexpr size_t MAX_TELEMETRY_KEYS = ARMORLINK_MAX_TELEMETRY_KEYS;
  TelemetryRateEntry _telemetryRates[MAX_TELEMETRY_KEYS]{};
  static constexpr size_t MAX_DATA_STREAM_RATE_KEYS = ARMORLINK_MAX_DATA_STREAM_RATE_KEYS;
#if ARMORLINK_HAS_DATASTREAMS
  DataStreamRateEntry _dataStreamRates[MAX_DATA_STREAM_RATE_KEYS]{};
#endif

  bool _isGatewayMode = false;

  bool _remoteLoggingEnabled = false;
    uint32_t _lastRemoteLogSentMs = 0;
  uint32_t _suppressedRemoteLogCount = 0;
  uint8_t _remoteLogLevel = AL_LOG_INFO;

  ArmorLinkPairingInfo _pairingInfo{};
#if ARMORLINK_HAS_ESPNOW
  ArmorLinkStoredPairedModule _pairedModules[ArmorLinkStorage::MAX_PAIRED_MODULES]{};
#endif
  size_t _pairedModuleCount = 0;

  ArmorLinkModuleTimeoutHook _moduleTimeoutHook = nullptr;
#if ARMORLINK_HAS_ESPNOW
  ArmorLinkModulePresenceState _presenceStates[ArmorLinkStorage::MAX_PAIRED_MODULES]{};
#endif
  uint32_t _lastPresenceCheckMs = 0;
  volatile bool _presenceProbePending = false;
  bool _presenceProbeActive = false;
  size_t _presenceProbeIndex = 0;
  uint32_t _lastPresenceProbeMs = 0;
  uint32_t _lastHeartbeatMs = 0;

  bool _loggingSyncPending = false;
  uint8_t _loggingSyncStage = 0;
  uint32_t _loggingSyncAtMs = 0;
  char _loggingSyncModuleName[16] = { 0 };
  char _loggingSyncModuleMac[18] = { 0 };

  bool _pairingActive = false;
  uint16_t _pairingSessionId = 0;
  uint32_t _pairingEndsAtMs = 0;
  uint32_t _lastPairAnnounceMs = 0;
  bool _remotePairingQuiet = false;
  uint16_t _remotePairingQuietSessionId = 0;
  uint32_t _remotePairingQuietUntilMs = 0;
  uint16_t _pairingQuietStopRepeatSessionId = 0;
  uint32_t _pairingQuietStopRepeatUntilMs = 0;
  uint32_t _lastPairingQuietStopMs = 0;

  bool _startupHelloActive = false;
  bool _startupHelloAcked = false;
  uint32_t _startupHelloUntilMs = 0;
  uint32_t _lastStartupHelloMs = 0;
  uint8_t _startupHelloAttempt = 0;


  bool _startupStateSyncActive = false;
  bool _startupStateSyncCompleted = false;
  uint32_t _startupStateSyncAtMs = 0;
  uint32_t _lastStartupStateSyncMs = 0;
  uint8_t _startupStateSyncAttempt = 0;

  static constexpr uint32_t PAIR_ANNOUNCE_INTERVAL_MS = 2000;
  static constexpr uint32_t PAIR_RESPONSE_REPEAT_MIN_INTERVAL_MS = 2000;
  static constexpr uint8_t PAIR_ANNOUNCE_SEND_ATTEMPTS = 3;
  static constexpr uint8_t PAIR_RESPONSE_SEND_ATTEMPTS = 3;
  static constexpr size_t MAX_PENDING_PAIRING_CANDIDATES = ARMORLINK_MAX_PAIRING_CANDIDATES;
  static constexpr size_t MAX_DATA_STREAM_SUBSCRIPTIONS = ARMORLINK_MAX_DATA_STREAM_SUBSCRIPTIONS;
  static constexpr size_t MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS = ARMORLINK_MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS;
  static constexpr size_t MAX_REQUESTED_DATA_STREAMS = ARMORLINK_MAX_REQUESTED_DATA_STREAMS;
  static constexpr size_t MAX_DATA_STREAM_HANDLERS = ARMORLINK_MAX_DATA_STREAM_HANDLERS;

  struct DataStreamSubscription {
    bool occupied = false;
    char subscriber[ARMORLINK_NAME_MAX_LEN + 1] = { 0 };
    char source[ARMORLINK_NAME_MAX_LEN + 1] = { 0 };
    char stream[ARMORLINK_COMMAND_MAX_LEN + 1] = { 0 };
  };

  struct LocalDataStreamSubscription {
    bool occupied = false;
    char source[ARMORLINK_NAME_MAX_LEN + 1] = { 0 };
    char stream[ARMORLINK_COMMAND_MAX_LEN + 1] = { 0 };
  };

  struct RequestedDataStream {
    bool occupied = false;
    char stream[ARMORLINK_COMMAND_MAX_LEN + 1] = { 0 };
  };

  struct DataStreamHandlerEntry {
    bool occupied = false;
    char stream[ARMORLINK_COMMAND_MAX_LEN + 1] = { 0 };
    ArmorLinkDataStreamHandler handler;
  };

#if ARMORLINK_HAS_ESPNOW
  ArmorLinkPairingCandidate _pendingCandidates[MAX_PENDING_PAIRING_CANDIDATES]{};
#endif
  size_t _pendingCandidateCount = 0;
#if ARMORLINK_HAS_DATASTREAMS
  DataStreamSubscription _dataStreamSubscriptions[MAX_DATA_STREAM_SUBSCRIPTIONS]{};
#endif
#if ARMORLINK_HAS_DATASTREAMS
  LocalDataStreamSubscription _localDataStreamSubscriptions[MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS]{};
#endif
#if ARMORLINK_HAS_DATASTREAMS
  RequestedDataStream _requestedDataStreams[MAX_REQUESTED_DATA_STREAMS]{};
#endif
#if ARMORLINK_HAS_DATASTREAMS
  DataStreamHandlerEntry _dataStreamHandlers[MAX_DATA_STREAM_HANDLERS]{};
#endif

  bool isPairingTraffic(const ArmorLinkPacket& msg) const {
    if (msg.msgType == AL_MSG_PAIR_ANNOUNCE ||
        msg.msgType == AL_MSG_PAIR_RESPONSE ||
        msg.msgType == AL_MSG_PAIR_ACCEPT) {
      return true;
    }

    return msg.msgType == AL_MSG_COMMAND &&
           equalsIgnoreCase(msg.entity, "pairing");
  }

  uint16_t pairingSessionFromPacket(const ArmorLinkPacket& msg) const {
    if (msg.valueInt > 0 && msg.valueInt <= 0xFFFF) {
      return static_cast<uint16_t>(msg.valueInt);
    }

    const String payload = armorLinkPacketPayloadToString(msg);
    if (payload.isEmpty()) {
      return 0;
    }

    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
      return 0;
    }

    return doc["sessionId"] | doc["sid"] | 0;
  }

  bool isRemotePairingQuietActive() {
    if (!_remotePairingQuiet) {
      return false;
    }

    if (_remotePairingQuietUntilMs != 0 &&
        (int32_t)(millis() - _remotePairingQuietUntilMs) >= 0) {
      AL_INTERNAL_PRINTF("[PAIR] Remote pairing quiet expired | sessionId=%u\n",
                    _remotePairingQuietSessionId);
      _remotePairingQuiet = false;
      _remotePairingQuietSessionId = 0;
      _remotePairingQuietUntilMs = 0;
      return false;
    }

    return true;
  }

  void remotePairingQuietTick() {
#if ARMORLINK_HAS_ESPNOW
    isRemotePairingQuietActive();
#endif
  }

  void enterRemotePairingQuiet(uint16_t sessionId, uint32_t durationMs = 45000) {
    if (_isGatewayMode || sessionId == 0) {
      return;
    }

    const bool changed =
      !_remotePairingQuiet ||
      _remotePairingQuietSessionId != sessionId;

    _remotePairingQuiet = true;
    _remotePairingQuietSessionId = sessionId;
    _remotePairingQuietUntilMs = millis() + durationMs;

    if (changed) {
      AL_INTERNAL_PRINTF("[PAIR] Remote pairing quiet started | sessionId=%u | timeoutMs=%lu\n",
                    sessionId,
                    (unsigned long)durationMs);
    }
  }

  void clearRemotePairingQuiet(uint16_t sessionId = 0) {
    if (!_remotePairingQuiet) {
      return;
    }

    if (sessionId != 0 && _remotePairingQuietSessionId != sessionId) {
      return;
    }

    AL_INTERNAL_PRINTF("[PAIR] Remote pairing quiet stopped | sessionId=%u\n",
                  _remotePairingQuietSessionId);

    _remotePairingQuiet = false;
    _remotePairingQuietSessionId = 0;
    _remotePairingQuietUntilMs = 0;
  }

  void schedulePairingQuietStopRepeat(uint16_t sessionId) {
    if (!_isGatewayMode || sessionId == 0) {
      return;
    }

    _pairingQuietStopRepeatSessionId = sessionId;
    _pairingQuietStopRepeatUntilMs = millis() + 2500;
    _lastPairingQuietStopMs = 0;
  }

  void pairingQuietStopRepeatTick() {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || _pairingQuietStopRepeatSessionId == 0) {
      return;
    }

    const uint32_t now = millis();
    if ((int32_t)(now - _pairingQuietStopRepeatUntilMs) >= 0) {
      _pairingQuietStopRepeatSessionId = 0;
      _pairingQuietStopRepeatUntilMs = 0;
      _lastPairingQuietStopMs = 0;
      return;
    }

    if (_lastPairingQuietStopMs != 0 &&
        (uint32_t)(now - _lastPairingQuietStopMs) < 300) {
      return;
    }

    _lastPairingQuietStopMs = now;
    broadcastPairingQuietState(false, _pairingQuietStopRepeatSessionId);
#endif
  }

  bool allowTelemetrySend(const String& key) {
    uint32_t now = millis();

    for (size_t i = 0; i < MAX_TELEMETRY_KEYS; ++i) {
      if (_telemetryRates[i].key[0] == 0) {
        strncpy(_telemetryRates[i].key, key.c_str(), sizeof(_telemetryRates[i].key) - 1);
        _telemetryRates[i].lastSentMs = now;
        return true;
      }

      if (key.equalsIgnoreCase(_telemetryRates[i].key)) {
        if (now - _telemetryRates[i].lastSentMs < _telemetryMinIntervalMs) {
          return false;
        }
        _telemetryRates[i].lastSentMs = now;
        return true;
      }
    }

    return false;
  }

  bool allowDataStreamSend(const char* key, uint32_t intervalMs) {
#if ARMORLINK_HAS_DATASTREAMS
    if (intervalMs == 0) {
      return true;
    }

    if (key == nullptr || key[0] == '\0') {
      return true;
    }

    const uint32_t now = millis();

    for (size_t i = 0; i < MAX_DATA_STREAM_RATE_KEYS; ++i) {
      if (_dataStreamRates[i].key[0] == 0) {
        armorlinkCopyString(_dataStreamRates[i].key, sizeof(_dataStreamRates[i].key), key);
#if ARMORLINK_HAS_DATASTREAMS
        _dataStreamRates[i].lastSentMs = now;
#endif
        return true;
      }

      if (equalsIgnoreCase(key, _dataStreamRates[i].key)) {
        if ((uint32_t)(now - _dataStreamRates[i].lastSentMs) < intervalMs) {
          return false;
        }
#if ARMORLINK_HAS_DATASTREAMS
        _dataStreamRates[i].lastSentMs = now;
#endif
        return true;
      }
    }

    return true;
#else
    return false;
#endif
  }

  bool upsertLocalDataStreamSubscription(const char* source, const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    for (size_t i = 0; i < MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (_localDataStreamSubscriptions[i].occupied &&
          String(_localDataStreamSubscriptions[i].source).equalsIgnoreCase(source) &&
          String(_localDataStreamSubscriptions[i].stream).equalsIgnoreCase(stream)) {
        return true;
      }
    }

    for (size_t i = 0; i < MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (!_localDataStreamSubscriptions[i].occupied) {
        _localDataStreamSubscriptions[i].occupied = true;
        armorlinkCopyString(_localDataStreamSubscriptions[i].source,
                            sizeof(_localDataStreamSubscriptions[i].source),
                            source);
        armorlinkCopyString(_localDataStreamSubscriptions[i].stream,
                            sizeof(_localDataStreamSubscriptions[i].stream),
                            stream);
        return true;
      }
    }

    AL_INTERNAL_PRINTF("[STREAM] Local subscription table full for %s.%s\n", source, stream);
    return false;
#else
    return false;
#endif
  }

  void removeLocalDataStreamSubscription(const char* source, const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    for (size_t i = 0; i < MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (_localDataStreamSubscriptions[i].occupied &&
          String(_localDataStreamSubscriptions[i].source).equalsIgnoreCase(source) &&
          String(_localDataStreamSubscriptions[i].stream).equalsIgnoreCase(stream)) {
        _localDataStreamSubscriptions[i] = LocalDataStreamSubscription{};
      }
    }
#endif
  }

  bool upsertRequestedDataStream(const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    if (stream == nullptr || stream[0] == '\0') {
      return false;
    }

    for (size_t i = 0; i < MAX_REQUESTED_DATA_STREAMS; ++i) {
      if (_requestedDataStreams[i].occupied &&
          equalsIgnoreCase(_requestedDataStreams[i].stream, stream)) {
        return true;
      }
    }

    for (size_t i = 0; i < MAX_REQUESTED_DATA_STREAMS; ++i) {
      if (!_requestedDataStreams[i].occupied) {
        _requestedDataStreams[i].occupied = true;
        armorlinkCopyString(_requestedDataStreams[i].stream,
                            sizeof(_requestedDataStreams[i].stream),
                            stream);
        AL_INTERNAL_PRINTF("[STREAM] Source stream requested: %s\n", stream);
        return true;
      }
    }

    AL_INTERNAL_PRINTF("[STREAM] Requested stream table full for %s\n", stream);
    return false;
#else
    return false;
#endif
  }

  void removeRequestedDataStream(const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    if (stream == nullptr || stream[0] == '\0') {
      return;
    }

    for (size_t i = 0; i < MAX_REQUESTED_DATA_STREAMS; ++i) {
      if (_requestedDataStreams[i].occupied &&
          equalsIgnoreCase(_requestedDataStreams[i].stream, stream)) {
        _requestedDataStreams[i] = RequestedDataStream{};
        AL_INTERNAL_PRINTF("[STREAM] Source stream released: %s\n", stream);
      }
    }
#endif
  }

  bool isDataStreamRequested(const char* stream) const {
#if ARMORLINK_HAS_DATASTREAMS
    if (stream == nullptr || stream[0] == '\0') {
      return false;
    }

    for (size_t i = 0; i < MAX_REQUESTED_DATA_STREAMS; ++i) {
      if (_requestedDataStreams[i].occupied &&
          equalsIgnoreCase(_requestedDataStreams[i].stream, stream)) {
        return true;
      }
    }

    return false;
#else
    return false;
#endif
  }

  esp_err_t sendDataStreamSubscription(const char* source, const char* stream, bool subscribe) {
#if ARMORLINK_HAS_DATASTREAMS
    if (_options.defaultLogTarget.isEmpty()) {
      return ESP_ERR_NOT_FOUND;
    }

    if (!_isGatewayMode && isRemotePairingQuietActive()) {
      return ESP_OK;
    }

    StaticJsonDocument<160> doc;
    doc["source"] = source ? source : "*";
    doc["stream"] = stream ? stream : "";

    String payload;
    serializeJson(doc, payload);

    const char* localName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
    ArmorLinkPacket out = makeArmorLinkBasePacket(
      subscribe ? AL_MSG_DATA_STREAM_SUBSCRIBE : AL_MSG_DATA_STREAM_UNSUBSCRIBE,
      localName,
      _options.defaultLogTarget.c_str(),
      "stream",
      subscribe ? "subscribe" : "unsubscribe"
    );
    setArmorLinkPacketPayload(out, payload);

    esp_err_t result = _transport.sendPacketToTarget(_options.defaultLogTarget, out);
    AL_INTERNAL_PRINTF("[STREAM] %s %s.%s -> %s (%s)\n",
                  subscribe ? "Subscribe" : "Unsubscribe",
                  source ? source : "*",
                  stream ? stream : "",
                  _options.defaultLogTarget.c_str(),
                  esp_err_to_name(result));
    return result;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
  }

  void sendAllDataStreamSubscriptions() {
#if ARMORLINK_HAS_DATASTREAMS
    if (_isGatewayMode || _options.defaultLogTarget.isEmpty()) {
      return;
    }

    for (size_t i = 0; i < MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (!_localDataStreamSubscriptions[i].occupied) {
        continue;
      }
      sendDataStreamSubscription(
        _localDataStreamSubscriptions[i].source,
        _localDataStreamSubscriptions[i].stream,
        true);
      delay(15);
      yield();
    }
#endif
  }

  bool upsertGatewayDataStreamSubscription(const char* subscriber,
                                           const char* source,
                                           const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    if (subscriber == nullptr || strlen(subscriber) == 0 ||
        stream == nullptr || strlen(stream) == 0) {
      return false;
    }

    const char* normalizedSource =
      (source == nullptr || strlen(source) == 0) ? "*" : source;

    for (size_t i = 0; i < MAX_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (_dataStreamSubscriptions[i].occupied &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].subscriber, subscriber) &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].source, normalizedSource) &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].stream, stream)) {
        return true;
      }
    }

    for (size_t i = 0; i < MAX_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (!_dataStreamSubscriptions[i].occupied) {
        _dataStreamSubscriptions[i].occupied = true;
        armorlinkCopyString(_dataStreamSubscriptions[i].subscriber,
                            sizeof(_dataStreamSubscriptions[i].subscriber),
                            subscriber);
        armorlinkCopyString(_dataStreamSubscriptions[i].source,
                            sizeof(_dataStreamSubscriptions[i].source),
                            normalizedSource);
        armorlinkCopyString(_dataStreamSubscriptions[i].stream,
                            sizeof(_dataStreamSubscriptions[i].stream),
                            stream);
        AL_INTERNAL_PRINTF("[STREAM][GW] %s subscribed to %s.%s\n",
                      subscriber,
                      normalizedSource,
                      stream);
        return true;
      }
    }

    AL_INTERNAL_PRINTF("[STREAM][GW] Subscription table full for %s -> %s.%s\n",
                  subscriber,
                  normalizedSource,
                  stream);
    return false;
#else
    return false;
#endif
  }

  void removeGatewayDataStreamSubscription(const char* subscriber,
                                           const char* source,
                                           const char* stream) {
#if ARMORLINK_HAS_DATASTREAMS
    const char* normalizedSource =
      (source == nullptr || strlen(source) == 0) ? "*" : source;

    for (size_t i = 0; i < MAX_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (_dataStreamSubscriptions[i].occupied &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].subscriber, subscriber) &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].source, normalizedSource) &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].stream, stream)) {
        AL_INTERNAL_PRINTF("[STREAM][GW] %s unsubscribed from %s.%s\n",
                      subscriber,
                      normalizedSource,
                      stream);
        _dataStreamSubscriptions[i] = DataStreamSubscription{};
      }
    }
#endif
  }

  bool dataStreamSubscriptionMatches(const DataStreamSubscription& sub,
                                     const ArmorLinkPacket& msg) const {
    if (!sub.occupied) {
      return false;
    }

    if (!equalsIgnoreCase(sub.stream, msg.command)) {
      return false;
    }

    return equalsIgnoreCase(sub.source, "*") ||
           equalsIgnoreCase(sub.source, msg.source);
  }

  size_t gatewayDataStreamDemandCount(const char* source, const char* stream) const {
#if ARMORLINK_HAS_DATASTREAMS
    if (source == nullptr || stream == nullptr || stream[0] == '\0') {
      return 0;
    }

    size_t count = 0;
    for (size_t i = 0; i < MAX_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (_dataStreamSubscriptions[i].occupied &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].source, source) &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].stream, stream)) {
        count++;
      }
    }
    return count;
#else
    return 0;
#endif
  }

  bool isLocalDataStreamSource(const char* source) const {
    if (_module == nullptr || source == nullptr || source[0] == '\0') {
      return false;
    }

    return equalsIgnoreCase(source, _module->name().c_str());
  }

  esp_err_t forwardDataStreamDemandToSource(const char* source,
                                            const char* stream,
                                            bool subscribe) {
#if ARMORLINK_HAS_DATASTREAMS
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || source == nullptr || stream == nullptr ||
        source[0] == '\0' || stream[0] == '\0' ||
        isLocalDataStreamSource(source)) {
      return ESP_OK;
    }

    StaticJsonDocument<128> doc;
    doc["source"] = source;
    doc["stream"] = stream;

    String payload;
    serializeJson(doc, payload);

    const char* localName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
    ArmorLinkPacket out = makeArmorLinkBasePacket(
      subscribe ? AL_MSG_DATA_STREAM_SUBSCRIBE : AL_MSG_DATA_STREAM_UNSUBSCRIBE,
      localName,
      source,
      "stream",
      subscribe ? "subscribe" : "unsubscribe"
    );
    setArmorLinkPacketPayload(out, payload);

    esp_err_t result = ESP_OK;

    if (equalsIgnoreCase(source, "*")) {
      result = ESP_ERR_NOT_FOUND;
      for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
        const esp_err_t current = _transport.sendPacketToTarget(String(_pairedModules[i].name), out);
        if (current == ESP_OK) {
          result = ESP_OK;
        }
        delay(5);
        yield();
      }
    } else {
      result = _transport.sendPacketToTarget(String(source), out);
    }

    AL_INTERNAL_PRINTF("[STREAM][GW] %s source %s.%s -> %s\n",
                  subscribe ? "request" : "release",
                  source,
                  stream,
                  esp_err_to_name(result));
    return result;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
  }

  bool hasGatewayDataStreamRoute(const char* source, const char* stream) const {
#if ARMORLINK_HAS_DATASTREAMS
    if (!_isGatewayMode || stream == nullptr || stream[0] == '\0') {
      return false;
    }

    for (size_t i = 0; i < MAX_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (!_dataStreamSubscriptions[i].occupied) {
        continue;
      }

      if (!equalsIgnoreCase(_dataStreamSubscriptions[i].stream, stream)) {
        continue;
      }

      if (source != nullptr &&
          equalsIgnoreCase(_dataStreamSubscriptions[i].subscriber, source)) {
        continue;
      }

      if (equalsIgnoreCase(_dataStreamSubscriptions[i].source, "*") ||
          (source != nullptr &&
           equalsIgnoreCase(_dataStreamSubscriptions[i].source, source))) {
        return true;
      }
    }

    return false;
#else
    return false;
#endif
  }

  void invokeLocalDataStreamHandlers(const ArmorLinkPacket& msg) {
#if ARMORLINK_HAS_DATASTREAMS
    const String payload = armorLinkPacketPayloadToString(msg);
    ArmorLinkDataStream stream(msg.source, msg.command, msg.target, payload);

    for (size_t i = 0; i < MAX_DATA_STREAM_HANDLERS; ++i) {
      if (!_dataStreamHandlers[i].occupied ||
          !String(_dataStreamHandlers[i].stream).equalsIgnoreCase(msg.command) ||
          !_dataStreamHandlers[i].handler) {
        continue;
      }

      _dataStreamHandlers[i].handler(stream);
    }
#endif
  }

  void emitDataStreamBleEvent(const ArmorLinkPacket& msg, const char* routedTo = nullptr) {
#if ARMORLINK_HAS_DATASTREAMS

    StaticJsonDocument<384> doc;
    doc["type"] = "data_stream";
    doc["source"] = msg.source;
    doc["target"] = msg.target;
    doc["stream"] = msg.command;
    if (routedTo != nullptr && routedTo[0] != '\0') {
      doc["routedTo"] = routedTo;
    }

    StaticJsonDocument<192> values;
    const String payload = armorLinkPacketPayloadToString(msg);
    if (!payload.isEmpty() &&
        deserializeJson(values, payload) == DeserializationError::Ok) {
      doc["values"] = values.as<JsonVariant>();
    }

    String out;
    serializeJson(doc, out);
    doc.clear();
    values.clear();
    bleNotifyEventJson(out);

#endif
  }

  void handleDataStreamSubscribePacket(const ArmorLinkPacket& msg, bool subscribe) {
#if ARMORLINK_HAS_DATASTREAMS

    StaticJsonDocument<160> doc;
    const String payload = armorLinkPacketPayloadToString(msg);
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {
      AL_INTERNAL_PRINTF("[STREAM] Invalid subscription payload from %s\n", msg.source);
      return;
    }

    const String source = String((const char*)(doc["source"] | "*"));
    const String stream = String((const char*)(doc["stream"] | msg.command));
    if (stream.isEmpty()) {
      return;
    }

    if (!_isGatewayMode) {
      const bool targetMatches =
        _module != nullptr &&
        (equalsIgnoreCase(msg.target, _module->name().c_str()) ||
         equalsIgnoreCase(msg.target, "*"));

      const bool sourceMatches =
        _module != nullptr &&
        (equalsIgnoreCase(source.c_str(), _module->name().c_str()) ||
         equalsIgnoreCase(source.c_str(), "*"));

      if (targetMatches && sourceMatches) {
        if (subscribe) {
          upsertRequestedDataStream(stream.c_str());
        } else {
          removeRequestedDataStream(stream.c_str());
        }
      }
      return;
    }

    const size_t beforeDemand =
      gatewayDataStreamDemandCount(source.c_str(), stream.c_str());

    if (subscribe) {
      upsertGatewayDataStreamSubscription(msg.source, source.c_str(), stream.c_str());
      if (beforeDemand == 0) {
        forwardDataStreamDemandToSource(source.c_str(), stream.c_str(), true);
      }
    } else {
      removeGatewayDataStreamSubscription(msg.source, source.c_str(), stream.c_str());
      if (gatewayDataStreamDemandCount(source.c_str(), stream.c_str()) == 0) {
        forwardDataStreamDemandToSource(source.c_str(), stream.c_str(), false);
      }
    }

#endif
  }

  esp_err_t routeDataStreamFromGateway(const ArmorLinkPacket& msg) {
#if ARMORLINK_HAS_DATASTREAMS
    if (!_isGatewayMode) {
      return ESP_ERR_INVALID_STATE;
    }

    if (_pairingActive) {
      return ESP_OK;
    }

    esp_err_t lastResult = ESP_ERR_NOT_FOUND;
    size_t forwarded = 0;

    for (size_t i = 0; i < MAX_DATA_STREAM_SUBSCRIPTIONS; ++i) {
      if (!dataStreamSubscriptionMatches(_dataStreamSubscriptions[i], msg)) {
        continue;
      }

      if (equalsIgnoreCase(_dataStreamSubscriptions[i].subscriber, msg.source)) {
        continue;
      }

      ArmorLinkPacket out = msg;
      armorlinkCopyString(out.target,
                          sizeof(out.target),
                          _dataStreamSubscriptions[i].subscriber);

      lastResult = _transport.sendPacketToTarget(
        String(_dataStreamSubscriptions[i].subscriber),
        out);

      forwarded++;
      delay(5);
      yield();
    }

    if (forwarded == 0) {
      return ESP_OK;
    }

    static uint32_t lastRouteLogMs = 0;
    const uint32_t now = millis();
    if (lastRouteLogMs == 0 || (uint32_t)(now - lastRouteLogMs) >= 1000) {
      lastRouteLogMs = now;
      AL_INTERNAL_PRINTF("[STREAM][GW] Routed %s.%s to %u subscriber(s) | result=%s\n",
                    msg.source,
                    msg.command,
                    static_cast<unsigned>(forwarded),
                    esp_err_to_name(lastResult));
    }

    return forwarded > 0 ? lastResult : ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
  }

  void handleDataStreamPacket(const ArmorLinkPacket& msg) {
#if ARMORLINK_HAS_DATASTREAMS
    if (_isGatewayMode) {
      routeDataStreamFromGateway(msg);

      const bool targetMatches =
        _module != nullptr &&
        (equalsIgnoreCase(msg.target, _module->name().c_str()) ||
         equalsIgnoreCase(msg.target, "*"));

      if (targetMatches) {
        invokeLocalDataStreamHandlers(msg);
      }
      return;
    }

    const bool targetMatches =
      _module != nullptr &&
      (equalsIgnoreCase(msg.target, _module->name().c_str()) ||
       equalsIgnoreCase(msg.target, "*"));

    if (targetMatches) {
      invokeLocalDataStreamHandlers(msg);
    }
#endif
  }
ArmorLinkPairingCandidate* pairingCandidateBySerialIndex(size_t serialIndex) {
#if ARMORLINK_HAS_ESPNOW
  if (serialIndex == 0) {
    return nullptr;
  }

  size_t displayIndex = 1;

  for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
    if (!_pendingCandidates[i].occupied) {
      continue;
    }

    if (displayIndex == serialIndex) {
      return &_pendingCandidates[i];
    }

    displayIndex++;
  }

  return nullptr;
#else
    return nullptr;
#endif
  }

void pairCandidateBySerialIndex(int index) {
  if (index <= 0) {
    Serial.println("[MENU] Usage: pair <number>");
    return;
  }

  ArmorLinkPairingCandidate* candidate = pairingCandidateBySerialIndex(static_cast<size_t>(index));
  if (candidate == nullptr) {
    Serial.println("[MENU] Candidate not found.");
    printPairingCandidates();
    return;
  }

  const String mac = String(candidate->mac);
  const String name = String(candidate->name);

  Serial.printf("[MENU] Pairing candidate %d: %s (%s)\n",
                index,
                name.c_str(),
                mac.c_str());

  const bool ok = acceptPairingCandidate(mac);

  if (ok) {
    Serial.printf("[MENU] Paired: %s\n", name.c_str());
  } else {
    Serial.println("[MENU] Pairing failed.");
  }

  printPairingCandidates();
}

void unpairModuleFromSerial(const String& value) {
#if ARMORLINK_HAS_ESPNOW
  if (value.isEmpty()) {
    Serial.println("[MENU] Usage: unpair <number|mac>");
    return;
  }

  String mac = value;

  // number-based unpair
  bool isNumber = true;
  for (size_t i = 0; i < value.length(); ++i) {
    if (!isDigit(value[i])) {
      isNumber = false;
      break;
    }
  }

  if (isNumber) {
    int index = value.toInt();

    if (index <= 0 || static_cast<size_t>(index) > _pairedModuleCount) {
      Serial.println("[MENU] Module index not found.");
      printPairedModules();
      return;
    }

    mac = String(_pairedModules[index - 1].mac);
  }

  const bool ok = unpairPairedModuleByMac(mac);

  if (ok) {
    Serial.println("[MENU] Module unpaired.");
  } else {
    Serial.println("[MENU] Unpair failed.");
  }

  printPairedModules();
#endif
  }

  bool shouldSendRemoteLog(uint8_t level) {
    // WARN und ERROR immer senden
    if (level >= AL_LOG_WARN) {
      _lastRemoteLogSentMs = millis();
      return true;
    }

    const uint32_t minIntervalMs = _options.remoteLogMinIntervalMs;
    if (minIntervalMs == 0) {
      _lastRemoteLogSentMs = millis();
      return true;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - _lastRemoteLogSentMs) < minIntervalMs) {
      _suppressedRemoteLogCount++;
      return false;
    }

    _lastRemoteLogSentMs = now;
    return true;
  }
  void syncPresenceStatesFromStoredModules() {
#if ARMORLINK_HAS_ESPNOW
    memset(static_cast<void*>(_presenceStates), 0, sizeof(_presenceStates));

    for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
      _presenceStates[i].occupied = true;
      _presenceStates[i].isOnline = false;
      _presenceStates[i].timeoutReported = false;
      _presenceStates[i].lastSeenMs = 0;
      armorlinkCopyString(_presenceStates[i].mac, sizeof(_presenceStates[i].mac), _pairedModules[i].mac);
    }
#endif
  }

  int findPresenceIndexByMac(const String& macText) const {
#if ARMORLINK_HAS_ESPNOW
    for (size_t i = 0; i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
      if (_presenceStates[i].occupied && macText.equalsIgnoreCase(_presenceStates[i].mac)) {
        return static_cast<int>(i);
      }
    }
    return -1;
#else
    return -1;
#endif
  }

bool shouldReportPairingRequired(const ArmorLinkPacket& msg) const {
  if (_isGatewayMode || _module == nullptr) {
    return false;
  }

  if (_pairingInfo.paired) {
    return false;
  }

  const bool targetMatches =
    equalsIgnoreCase(msg.target, "*") ||
    equalsIgnoreCase(msg.target, _module->name().c_str());

  if (!targetMatches) {
    return false;
  }

  if (msg.msgType == AL_MSG_PAIR_ANNOUNCE ||
      msg.msgType == AL_MSG_PAIR_ACCEPT ||
      msg.msgType == AL_MSG_PAIR_RESPONSE) {
    return false;
  }

  if (msg.msgType == AL_MSG_COMMAND &&
      equalsIgnoreCase(msg.entity, "pairing")) {
    return false;
  }

  return msg.msgType == AL_MSG_COMMAND ||
         msg.msgType == AL_MSG_CONFIG_GET ||
         msg.msgType == AL_MSG_CONFIG_SET;
}

void sendPairingRequiredEvent(const ArmorLinkPacket& msg) {
  if (_module == nullptr) {
    return;
  }

  StaticJsonDocument<256> eventDoc;
  eventDoc["type"] = "module_pairing_required";

  JsonObject module = eventDoc.createNestedObject("module");
  module["name"] = _module->name();
  module["type"] = moduleTypeToString(_module->type());
  module["mac"] = localMacString();

  String eventJson;
  serializeJson(eventDoc, eventJson);

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    strlen(msg.source) > 0 ? msg.source : "Gateway",
    "pairing",
    "required"
  );
  setArmorLinkPacketPayload(out, eventJson);

  const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  esp_err_t result = _transport.sendPacketToMac(broadcastMac, out);

  AL_INTERNAL_PRINTF("[PAIR] Pairing required reported to gateway | source=%s | result=%s | payloadLen=%u | payload=%s\n",
                msg.source,
                esp_err_to_name(result),
                static_cast<unsigned>(eventJson.length()),
                eventJson.c_str());
}
  void emitCompactPairingRequiredEvent(const String& payload, const char* fallbackName) {
    StaticJsonDocument<256> inDoc;
    const bool hasPayload = payload.length() > 0 &&
      deserializeJson(inDoc, payload) == DeserializationError::Ok;

    const char* safeFallbackName = fallbackName != nullptr && strlen(fallbackName) > 0
      ? fallbackName
      : "Unknown";
    const char* moduleName = hasPayload
      ? (inDoc["module"]["name"] | safeFallbackName)
      : safeFallbackName;
    const char* moduleType = hasPayload
      ? (inDoc["module"]["type"] | "Unknown")
      : "Unknown";
    const char* moduleMac = hasPayload
      ? (inDoc["module"]["mac"] | "")
      : "";

    StaticJsonDocument<192> outDoc;
    outDoc["type"] = "module_pairing_required";
    JsonObject module = outDoc.createNestedObject("module");
    module["name"] = moduleName;
    module["type"] = moduleType;
    module["mac"] = moduleMac;

    String out;
    serializeJson(outDoc, out);
    outDoc.clear();
    inDoc.clear();
    AL_INTERNAL_PRINTF("[PAIR][GW] BLE pairing required compact event: %s\n", out.c_str());
    bleNotifyEventJson(out);
  }

  bool handleInternalEspNowCommand(const ArmorLinkPacket& msg) {

    AL_INTERNAL_PRINTF(
      "[RX] type=%u entity='%s' command='%s' target='%s' source='%s'\n",
      msg.msgType,
      msg.entity,
      msg.command,
      msg.target,
      msg.source
    );

    if (msg.msgType != AL_MSG_COMMAND) {
      return false;
    }

    if (!_isGatewayMode && _pairingInfo.paired &&
        equalsIgnoreCase(msg.entity, "system") &&
        equalsIgnoreCase(msg.command, "presence_get")) {
      // Only the paired gateway may request an immediate presence reply.
      if (equalsIgnoreCase(msg.source, _pairingInfo.gatewayName)) {
        sendHeartbeat();
      }
      return true;
    }

    if (!_isGatewayMode &&
        equalsIgnoreCase(msg.entity, "pairing") &&
        equalsIgnoreCase(msg.command, "quiet_start")) {
      const uint16_t sessionId = pairingSessionFromPacket(msg);
      enterRemotePairingQuiet(sessionId);
      return true;
    }

    if (!_isGatewayMode &&
        equalsIgnoreCase(msg.entity, "pairing") &&
        equalsIgnoreCase(msg.command, "quiet_stop")) {
      const uint16_t sessionId = pairingSessionFromPacket(msg);
      clearRemotePairingQuiet(sessionId);
      return true;
    }

    if (!_isGatewayMode &&
        equalsIgnoreCase(msg.entity, "pairing") &&
        equalsIgnoreCase(msg.command, "unpair")) {

      AL_VERBOSELN("[PAIR] Unpair command received");

      resetPairingInfo();

      AL_VERBOSELN("[PAIR] Module unpaired locally");
      return true;
    }
    if (_isGatewayMode &&
        equalsIgnoreCase(msg.entity, "pairing") &&
        equalsIgnoreCase(msg.command, "required")) {

      const String payload = armorLinkPacketPayloadToString(msg);

      AL_INTERNAL_PRINTF("[PAIR][GW] Module pairing required event from %s\n", msg.source);

      emitCompactPairingRequiredEvent(payload, msg.source);

      return true;
    }
    const bool targetMatches =
      equalsIgnoreCase(msg.target, "*") ||
      (_module != nullptr && equalsIgnoreCase(msg.target, _module->name().c_str()));

    if (!targetMatches) {
      return false;
    }

    const String payload = armorLinkPacketPayloadToString(msg);
    if (_isGatewayMode &&
        equalsIgnoreCase(msg.entity, "system") &&
        equalsIgnoreCase(msg.command, "hello")) {

      StaticJsonDocument<224> doc;
      if (deserializeJson(doc, payload) != DeserializationError::Ok) {
        AL_VERBOSELN("[HELLO][GW] Invalid hello payload");
        return true;
      }

      const String moduleName = String((const char*)(doc["mn"] | doc["moduleName"] | ""));
      const String moduleMac  = String((const char*)(doc["mm"] | doc["moduleMac"] | ""));
      const String moduleVersion = String((const char*)(doc["mv"] | doc["moduleVersion"] | "1.0"));
      const String armorLinkVersion = String((const char*)(doc["av"] | doc["armorLinkVersion"] | ""));

      if (moduleName.isEmpty() || moduleMac.isEmpty()) {
        AL_VERBOSELN("[HELLO][GW] Missing moduleName/moduleMac");
        return true;
      }

      AL_INTERNAL_PRINTF("[HELLO][GW] Hello from %s (%s)\n",
                    moduleName.c_str(),
                    moduleMac.c_str());

      updateStoredModuleMetadataByMac(moduleMac, moduleName, moduleVersion, armorLinkVersion);
      updateModulePresenceByMac(moduleMac);
      
      sendHelloAckToModule(moduleName.c_str(), moduleMac.c_str());
      return true;
    }
  if (_isGatewayMode &&
      equalsIgnoreCase(msg.entity, "system") &&
      equalsIgnoreCase(msg.command, "heartbeat")) {

    StaticJsonDocument<224> doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {      
      return true;
    }

    const String moduleName = String((const char*)(doc["mn"] | doc["moduleName"] | msg.source));
    const String moduleMac  = String((const char*)(doc["mm"] | doc["moduleMac"] | ""));
    const String moduleVersion = String((const char*)(doc["mv"] | doc["moduleVersion"] | "1.0"));
    const String armorLinkVersion = String((const char*)(doc["av"] | doc["armorLinkVersion"] | ""));

    if (moduleMac.isEmpty()) {      
      return true;
    }    
    AL_INTERNAL_PRINTF(
      "[HEARTBEAT][GW] Received from %s (%s)\n",
      moduleName.c_str(),
      moduleMac.c_str()
    );
    updateStoredModuleMetadataByMac(moduleMac, moduleName, moduleVersion, armorLinkVersion);
    updateModulePresenceByMac(moduleMac, true);
    return true;
  }

  if (_isGatewayMode &&
      equalsIgnoreCase(msg.entity, "system") &&
      equalsIgnoreCase(msg.command, "sync_request")) {

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok) {      
      return true;
    }

    const String moduleName = String((const char*)(doc["mn"] | doc["moduleName"] | msg.source));
    const String moduleMac  = String((const char*)(doc["mm"] | doc["moduleMac"] | ""));
    const String moduleVersion = String((const char*)(doc["mv"] | doc["moduleVersion"] | "1.0"));
    const String armorLinkVersion = String((const char*)(doc["av"] | doc["armorLinkVersion"] | ""));

    if (moduleName.isEmpty() || moduleMac.isEmpty()) {      
      return true;
    }

    updateStoredModuleMetadataByMac(moduleMac, moduleName, moduleVersion, armorLinkVersion);
    updateModulePresenceByMac(moduleMac, true);
    sendStateSyncToModule(moduleName.c_str(), moduleMac.c_str());
    return true;
  }
    if (!_isGatewayMode &&
    equalsIgnoreCase(msg.entity, "system") &&
    equalsIgnoreCase(msg.command, "hello_ack")) {

  AL_INTERNAL_PRINTF(
    "[HELLO] hello_ack received target='%s' source='%s'\n",
    msg.target,
    msg.source
  );

  _startupHelloAcked = true;
  _startupHelloActive = false;
  sendAllDataStreamSubscriptions();

  AL_VERBOSELN("[HELLO] Startup hello acknowledged");

  return true;
}

  if (!_isGatewayMode &&
      equalsIgnoreCase(msg.entity, "system") &&
      equalsIgnoreCase(msg.command, "sync_state")) {

    StaticJsonDocument<192> doc;
    auto err = deserializeJson(
        doc,
        armorLinkPacketPayloadToString(msg));
    if (err != DeserializationError::Ok)
    {
        AL_INTERNAL_PRINT("[PAIR] JSON parse failed: ");
        AL_VERBOSE(err.c_str());
        return true;
    }

    _remoteLoggingEnabled = doc["loggingEnabled"] | false;
    _remoteLogLevel = doc["logLevel"] | AL_LOG_INFO;
    _remoteTelemetryEnabled = doc["telemetryEnabled"] | false;

    _startupStateSyncCompleted = true;
    _startupStateSyncActive = false;    
    sendAllDataStreamSubscriptions();

    return true;
  }

    if (equalsIgnoreCase(msg.entity, "telemetry") &&
        equalsIgnoreCase(msg.command, "set_enabled")) {

      bool enabled = msg.valueInt != 0;

      _remoteTelemetryEnabled = enabled;
      
      return true;
    }
    if (equalsIgnoreCase(msg.entity, "logs") &&
        equalsIgnoreCase(msg.command, "set_enabled")) {

      const bool enabled =
        msg.valueInt != 0 ||
        payload.equalsIgnoreCase("true") ||
        payload.equalsIgnoreCase("on");

      setRemoteLoggingEnabled(enabled);
      

      if (_startupStateSyncActive) {
        _startupStateSyncCompleted = true;
        _startupStateSyncActive = false;
      }

      return true;
    }
    if (equalsIgnoreCase(msg.entity, "logs") &&
        equalsIgnoreCase(msg.command, "set_level")) {

      uint8_t level = static_cast<uint8_t>(msg.valueInt);
      if (level < AL_LOG_DEBUG || level > AL_LOG_ERROR) {
        const String payload = armorLinkPacketPayloadToString(msg);
        level = logLevelFromKeyword(payload);
      }

      setRemoteLogLevel(level);      
      logLevelToKeyword(level);

      if (_startupStateSyncActive) {

        _startupStateSyncCompleted = true;

        _startupStateSyncActive = false;

      }

      return true;
    }
    return false;
  }

void updateModulePresenceByMac(const String& macText, bool forceEmitOnlineEvent = false) {
#if ARMORLINK_HAS_ESPNOW
  AL_INTERNAL_PRINTF(
  "[PRESENCE] updateModulePresenceByMac(%s)\n",
  macText.c_str()
);
  if (!_isGatewayMode || macText.isEmpty()) {
    return;
  }

  int index = findPresenceIndexByMac(macText);
  if (index < 0) {
    return;
  }

  ArmorLinkModulePresenceState& state = _presenceStates[index];
  const bool wasOnline = state.isOnline;

  state.occupied = true;
  state.isOnline = true;
  state.lastSeenMs = millis();
  state.timeoutReported = false;
    AL_INTERNAL_PRINTF(
      "[PRESENCE] %s marked ONLINE at %lu\n",
      macText.c_str(),
      (unsigned long)state.lastSeenMs
    );
  if (index < static_cast<int>(_pairedModuleCount)) {
    if (!wasOnline && _options.syncLoggingStateOnModuleOnline) {
      syncLoggingStateToModule(_pairedModules[index].name, _pairedModules[index].mac);
    }

    if (!wasOnline || forceEmitOnlineEvent) {
      emitModulePresenceEvent(_pairedModules[index], true, 0);
    }
  }
#endif
  }

void startupStateSyncTick() {
#if ARMORLINK_HAS_ESPNOW
  if (_isGatewayMode || _module == nullptr) return;
  if (!_startupStateSyncActive || _startupStateSyncCompleted) return;

  if (!_pairingInfo.paired) {
    _startupStateSyncActive = false;
    return;
  }

  const uint32_t now = millis();

  if ((int32_t)(now - _startupStateSyncAtMs) < 0) return;

  if (_lastStartupStateSyncMs != 0 &&
      (uint32_t)(now - _lastStartupStateSyncMs) < _options.startupStateSyncRetryMs) {
    return;
  }

  if (sendStateSyncRequest()) {
    _lastStartupStateSyncMs = now;
    _startupStateSyncAttempt++;    
  }
#endif
  }

bool sendStateSyncRequest() {
  if (_module == nullptr) return false;

  if (isRemotePairingQuietActive()) {
    return false;
  }

  if (strlen(_pairingInfo.gatewayName) == 0 || strlen(_pairingInfo.gatewayMac) == 0) {
    return false;
  }

  uint8_t gatewayMacBytes[6];
  if (!armorLinkParseMacString(String(_pairingInfo.gatewayMac), gatewayMacBytes)) {
    return false;
  }
  String payload;
  {
    StaticJsonDocument<320> doc;
    doc["mn"] = _module->name();
    doc["mm"] = localMacString();
    // The gateway already accepts these compact keys. Keep the request
    // within one ESP-NOW payload; the long keys truncated LeftHand's JSON.
    doc["mv"] = _module->version();
    doc["av"] = ARMORLINK_VERSION;
    doc["wantLogs"] = true;
    doc["wantTelemetry"] = true;
    serializeJson(doc, payload);
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    _pairingInfo.gatewayName,
    "system",
    "sync_request"
  );
  setArmorLinkPacketPayload(out, payload);

  esp_err_t result = _transport.sendPacketToMac(gatewayMacBytes, out);
  

  return result == ESP_OK;
}

void heartbeatTick() {
#if ARMORLINK_HAS_ESPNOW
  if (_isGatewayMode || _module == nullptr) {
    return;
  }

  if (!_pairingInfo.paired) {
    return;
  }

  if (isRemotePairingQuietActive()) {
    return;
  }

  const uint32_t intervalMs = _options.moduleHeartbeatIntervalMs;
  if (intervalMs == 0) {
    return;
  }

  const uint32_t now = millis();
  if (_lastHeartbeatMs != 0 && (uint32_t)(now - _lastHeartbeatMs) < intervalMs) {
    return;
  }

  if (sendHeartbeat()) {
    _lastHeartbeatMs = now;
  }
#endif
  }

bool sendHeartbeat() {
  if (_module == nullptr) {
    return false;
  }

  if (isRemotePairingQuietActive()) {
    return false;
  }

  if (strlen(_pairingInfo.gatewayName) == 0 || strlen(_pairingInfo.gatewayMac) == 0) {
    return false;
  }

  uint8_t gatewayMacBytes[6];
  if (!armorLinkParseMacString(String(_pairingInfo.gatewayMac), gatewayMacBytes)) {
    return false;
  }
  String payload;
  {
    StaticJsonDocument<288> doc;
    doc["mn"] = _module->name();
    doc["moduleType"] = moduleTypeToString(_module->type());
    doc["mm"] = localMacString();
    doc["moduleVersion"] = _module->version();
    doc["armorLinkVersion"] = ARMORLINK_VERSION;
    doc["up"] = millis();
    serializeJson(doc, payload);
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    _pairingInfo.gatewayName,
    "system",
    "heartbeat"
  );

  setArmorLinkPacketPayload(out, payload);
  esp_err_t result = _transport.sendPacketToMac(gatewayMacBytes, out);  
  AL_INTERNAL_PRINTF(
    "[HEARTBEAT] Sent -> gateway=%s (%s) payloadLen=%u result=%s\n",
    _pairingInfo.gatewayName,
    _pairingInfo.gatewayMac,
    out.payloadLen,
    esp_err_to_name(result)
  );
  return result == ESP_OK;
}
  void presenceProbeTick() {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || !_options.enableEspNow || _module == nullptr) return;
    if (_presenceProbePending) {
      _presenceProbePending = false;
      _presenceProbeActive = true;
      _presenceProbeIndex = 0;
      _lastPresenceProbeMs = millis() - 100UL;
      AL_INTERNAL_PRINTF("[PRESENCE] Probing %u paired nodes\n", (unsigned)_pairedModuleCount);
    }
    if (!_presenceProbeActive) return;
    if (_presenceProbeIndex >= _pairedModuleCount) {
      _presenceProbeActive = false;
      return;
    }
    const uint32_t now = millis();
    if ((uint32_t)(now - _lastPresenceProbeMs) < 100UL) return;
    _lastPresenceProbeMs = now;
    const auto& node = _pairedModules[_presenceProbeIndex++];
    uint8_t mac[6];
    if (!armorLinkParseMacString(String(node.mac), mac)) return;
    const ArmorLinkPacket query = makeArmorLinkBasePacket(
        AL_MSG_COMMAND, _module->name().c_str(), node.name, "system", "presence_get");
    const esp_err_t result = _transport.sendPacketToMac(mac, query);
    AL_INTERNAL_PRINTF("[PRESENCE] Probe -> %s result=%s\n", node.name, esp_err_to_name(result));
#endif
  }

  void presenceTick() {
#if ARMORLINK_HAS_ESPNOW

    if (!_isGatewayMode) {
      return;
    }

    if (_pairingActive) {
      return;
    }

    const uint32_t interval = _options.modulePresenceCheckIntervalMs;
    const uint32_t timeoutMs = _options.moduleTimeoutMs;

    if (interval == 0 || timeoutMs == 0) {
      return;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - _lastPresenceCheckMs) < interval) {
      return;
    }

    _lastPresenceCheckMs = now;
    if (_pairingQuietStopRepeatSessionId != 0) {
      broadcastPairingQuietState(false, _pairingQuietStopRepeatSessionId);
    }

    AL_INTERNAL_PRINTF(
      "[PRESENCE] Check running. paired=%u\n",
      (unsigned)_pairedModuleCount
    );
    for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
      ArmorLinkModulePresenceState& state = _presenceStates[i];
      if (!state.occupied || !state.isOnline) {
        continue;
      }

      const uint32_t silentMs = now - state.lastSeenMs;      
      if (silentMs < timeoutMs) {
        continue;
      }
      AL_INTERNAL_PRINTF(
        "[PRESENCE] TIMEOUT %s silent=%lu ms timeout=%lu ms\n",
        _pairedModules[i].name,
        (unsigned long)silentMs,
        (unsigned long)timeoutMs
      );
      state.isOnline = false;

      if (!state.timeoutReported) {
        state.timeoutReported = true;        

        if (_moduleTimeoutHook != nullptr) {
          _moduleTimeoutHook(_pairedModules[i], silentMs);
        }

        emitModulePresenceEvent(_pairedModules[i], false, silentMs);
      }
    }
#endif
  }
void initStartupHelloState() {
  _startupHelloActive = false;
  _startupHelloAcked = false;
  _startupHelloUntilMs = 0;
  _lastStartupHelloMs = 0;
  _startupHelloAttempt = 0;
  _lastHeartbeatMs = 0;
  _startupStateSyncActive = false;
  _startupStateSyncCompleted = false;
  _startupStateSyncAtMs = 0;
  _lastStartupStateSyncMs = 0;
  _startupStateSyncAttempt = 0;

  if (_isGatewayMode) {
    return;
  }

  if (!_pairingInfo.paired) {
    return;
  }

  if (strlen(_pairingInfo.gatewayName) == 0 || strlen(_pairingInfo.gatewayMac) == 0) {
    return;
  }

  _startupHelloActive = true;
  _startupHelloAcked = false;
  _startupHelloUntilMs = millis() + 300000UL; // 5 minutes
  _lastStartupHelloMs = 0;
  _startupHelloAttempt = 0;

  AL_VERBOSELN("[HELLO] Startup hello activated");
  if (_options.requestStateSyncAfterBoot) {
    _startupStateSyncActive = true;
    _startupStateSyncCompleted = false;
    _startupStateSyncAtMs = millis() + _options.startupStateSyncDelayMs;
    _lastStartupStateSyncMs = 0;
    _startupStateSyncAttempt = 0;    
  }
}

uint32_t nextStartupHelloIntervalMs() const {
  switch (_startupHelloAttempt) {
    case 0: return 0;
    case 1: return 2000;
    case 2: return 5000;
    case 3: return 10000;
    default: return 30000;
  }
}

void startupHelloTick() {
#if ARMORLINK_HAS_ESPNOW
  if (_isGatewayMode) {
    return;
  }

  if (!_startupHelloActive || _startupHelloAcked) {
    return;
  }

  if (!_pairingInfo.paired) {
    _startupHelloActive = false;
    return;
  }

  if (isRemotePairingQuietActive()) {
    return;
  }

  const uint32_t now = millis();

  if ((int32_t)(now - _startupHelloUntilMs) >= 0) {
    _startupHelloActive = false;
    AL_VERBOSELN("[HELLO] Startup hello window expired");
    return;
  }

  const uint32_t intervalMs = nextStartupHelloIntervalMs();
  if (_startupHelloAttempt > 0 && (uint32_t)(now - _lastStartupHelloMs) < intervalMs) {
    return;
  }

  if (sendStartupHello()) {
    _lastStartupHelloMs = now;
    _startupHelloAttempt++;

    AL_INTERNAL_PRINTF("[HELLO] Sent startup hello attempt %u\n", _startupHelloAttempt);
  }
#endif
  }

bool sendStartupHello() {
  if (_module == nullptr) {
    return false;
  }

  if (isRemotePairingQuietActive()) {
    return false;
  }

  if (strlen(_pairingInfo.gatewayName) == 0 || strlen(_pairingInfo.gatewayMac) == 0) {
    return false;
  }

  uint8_t gatewayMacBytes[6];
  if (!armorLinkParseMacString(String(_pairingInfo.gatewayMac), gatewayMacBytes)) {
    return false;
  }

  String payload;
  {
    StaticJsonDocument<288> doc;
    doc["mn"] = _module->name();
    doc["moduleType"] = moduleTypeToString(_module->type());
    doc["mm"] = localMacString();
    doc["moduleVersion"] = _module->version();
    doc["armorLinkVersion"] = ARMORLINK_VERSION;
    serializeJson(doc, payload);
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    _pairingInfo.gatewayName,
    "system",
    "hello"
  );
  setArmorLinkPacketPayload(out, payload);

  esp_err_t result = _transport.sendPacketToMac(gatewayMacBytes, out);

  AL_INTERNAL_PRINT("[HELLO] sendStartupHello result: ");
  AL_VERBOSE(esp_err_to_name(result));

  return result == ESP_OK;
}

void sendHelloAckToModule(const char* moduleName, const char* moduleMac) {
  if (!_isGatewayMode || _module == nullptr) {
    return;
  }

  uint8_t macBytes[6];
  if (!armorLinkParseMacString(String(moduleMac), macBytes)) {
    return;
  }

  String payload;
  {
    StaticJsonDocument<160> doc;
    doc["gatewayName"] = _module->name();
    doc["gatewayMac"] = localMacString();
    serializeJson(doc, payload);
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    moduleName,
    "system",
    "hello_ack"
  );
  setArmorLinkPacketPayload(out, payload);
  esp_err_t result = _transport.sendPacketToMac(macBytes, out);

  AL_INTERNAL_PRINTF("[HELLO][GW] hello_ack -> %s (%s): %s\n",
                moduleName,
                moduleMac,
                esp_err_to_name(result));
}
void emitModulePresenceSnapshot() {
#if ARMORLINK_HAS_ESPNOW
  if (!_isGatewayMode) {
    return;
  }
  if (_pairingQuietStopRepeatSessionId != 0) {
    broadcastPairingQuietState(false, _pairingQuietStopRepeatSessionId);
  }
  AL_INTERNAL_PRINTF("[PRESENCE] Snapshot start. pairedModuleCount=%u\n",
                  (unsigned)_pairedModuleCount);
  emitModulePresenceSnapshotEvent();

  const uint32_t now = millis();
  const uint32_t timeoutMs = _options.moduleTimeoutMs;

  for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
    ArmorLinkModulePresenceState& state = _presenceStates[i];
    AL_INTERNAL_PRINTF("[PRESENCE] Emitting %s (%s)\n",
              _pairedModules[i].name,
              _pairedModules[i].mac);
    if (!state.occupied) {
      continue;
    }

    uint32_t silentMs = 0;
    bool online = state.isOnline;

    if (state.lastSeenMs > 0) {
      silentMs = now - state.lastSeenMs;

      if (timeoutMs > 0 && silentMs >= timeoutMs) {
        online = false;
        state.isOnline = false;
        state.timeoutReported = true;
      }
    } else {
      online = false;
    }

    emitModulePresenceEvent(_pairedModules[i], online, silentMs);
  }
#endif
  }

  void emitModulePresenceSnapshotEvent() {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode) {
      return;
    }

    StaticJsonDocument<3072> doc;
    doc["type"] = "module_presence_snapshot";
    doc["pairedModuleCount"] = static_cast<uint32_t>(_pairedModuleCount);

    JsonArray modules = doc.createNestedArray("modules");
    const uint32_t now = millis();
    for (size_t i = 0; i < _pairedModuleCount && i < ArmorLinkStorage::MAX_PAIRED_MODULES; ++i) {
      JsonObject mod = modules.createNestedObject();
      const bool online = _presenceStates[i].occupied && _presenceStates[i].isOnline;
      const uint32_t silentMs =
        _presenceStates[i].lastSeenMs > 0
          ? now - _presenceStates[i].lastSeenMs
          : 0;

      mod["name"] = _pairedModules[i].name;
      mod["type"] = _pairedModules[i].type;
      mod["mac"] = _pairedModules[i].mac;
      mod["moduleVersion"] = strlen(_pairedModules[i].moduleVersion) > 0 ? _pairedModules[i].moduleVersion : "1.0";
      mod["armorLinkVersion"] = _pairedModules[i].armorLinkVersion;
      mod["online"] = online;
      mod["status"] = online ? "online" : "offline";
      mod["silentMs"] = silentMs;
    }

    String out;
    serializeJson(doc, out);
    doc.clear();
    AL_VERBOSE("[PRESENCE] BLE SNAPSHOT EVENT: %s\n", out.c_str());
    AL_INTERNAL_PRINTF("[BLE][TX] module_presence_snapshot bytes=%u modules=%u\n",
                  static_cast<unsigned>(out.length()),
                  static_cast<unsigned>(_pairedModuleCount));
    bleNotifyEventJson(out);
    emitSerialEventJson(out);
#endif
  }

  void emitModulePresenceEvent(const ArmorLinkStoredPairedModule& module, bool isOnline, uint32_t silentMs) {
    if (!_isGatewayMode) {
      return;
    }

    StaticJsonDocument<256> doc;
    doc["type"] = "module_presence";
    doc["status"] = isOnline ? "online" : "offline";
    doc["silentMs"] = silentMs;    

    JsonObject mod = doc.createNestedObject("module");
    mod["name"] = module.name;
    mod["type"] = module.type;
    mod["mac"] = module.mac;
    mod["moduleVersion"] = strlen(module.moduleVersion) > 0 ? module.moduleVersion : "1.0";
    mod["armorLinkVersion"] = module.armorLinkVersion;

    String out;
    serializeJson(doc, out);
    doc.clear();
    AL_VERBOSE("[PRESENCE] BLE EVENT: %s\n", out.c_str());

    AL_VERBOSE("[PRESENCE] JSON length=%u\n", static_cast<unsigned>(out.length()));
    bleNotifyEventJson(out);
    emitSerialEventJson(out);
  }
  

  void pairingTick() {
#if ARMORLINK_HAS_ESPNOW
    if (!_pairingActive) {
      return;
    }

    if (_pairingSessionId == 0 || _pairingEndsAtMs == 0) {
      AL_VERBOSELN("[PAIR][GW] Invalid pairing state detected -> reset");
      _pairingActive = false;
      _pairingEndsAtMs = 0;
      _pairingSessionId = 0;
      _lastPairAnnounceMs = 0;
      clearPendingPairingCandidates();
      return;
    }

    const uint32_t now = millis();

    if ((int32_t)(now - _pairingEndsAtMs) >= 0) {
      const uint16_t timedOutSessionId = _pairingSessionId;
      if (!_pairingActive || timedOutSessionId == 0) {
        return;
      }
      AL_INTERNAL_PRINTF("[PAIR][GW] Pairing timeout | sessionId=%u\n", timedOutSessionId);
      emitPairingStateEvent("pairing_timeout", timedOutSessionId);
      stopPairing();
      return;
    }

    if ((uint32_t)(now - _lastPairAnnounceMs) >= PAIR_ANNOUNCE_INTERVAL_MS) {
      const bool announced = sendPairAnnounceBurst();
      AL_INTERNAL_PRINTF("[PAIR][GW] Announcing pairing session %u | result=%s\n",
                    _pairingSessionId,
                    announced ? "sent" : "failed");
      _lastPairAnnounceMs = now;
      emitPairingStateEvent("pairing_ping");
      emitPendingPairingCandidateEvents();
    }
#endif
  }

  void sendSimpleLog(ArmorLinkLogLevel level, const String& message) {
    const char* levelName = armorLinkLogLevelToString(level);

    if (_options.enableSerialLogging) {
      Serial.printf("[%s] %s\n", levelName, message.c_str());
    }

      if (!_remoteLoggingEnabled && !_options.forceRemoteLogging) {        
        return;
      }

      if (level < _remoteLogLevel) {        
        return;
      }

      if (_isGatewayMode && _pairingActive) {
        return;
      }

      if (!_isGatewayMode && isRemotePairingQuietActive()) {
        return;
      }

      if (_localConfigTransferPending || _localConfigTransferActive) {
        return;
      }

#if defined(ESP32)
      if (ESP.getMaxAllocHeap() < 4096) {
        return;
      }
#endif

      if ((_isGatewayMode || !ARMORLINK_HAS_ESPNOW) && isBleLogStreamEnabled()) {
      const char* sourceName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";

      StaticJsonDocument<384> doc;
      doc["type"] = "log";
      doc["level"] = levelName;
      doc["source"] = sourceName;
      doc["entity"] = "log";
      doc["command"] = "message";
      doc["message"] = message;
      doc["valueInt"] = static_cast<int32_t>(millis());
      doc["valueFloat"] = 0.0f;
      doc["batteryVoltage"] = 0.0f;

      String out;
      serializeJson(doc, out);
      doc.clear();
      bleNotifyLogJson(out);
      return;
    }

    if (_options.defaultLogTarget.isEmpty()) {      
      return;
    }


    if (!shouldSendRemoteLog(level)) {
      return;
    }
    const char* sourceName = (_module != nullptr) ? _module->name().c_str() : "ArmorLink";
    

    (void)_transport.sendLogToTarget(
      _options.defaultLogTarget,
      sourceName,
      level,
      "log",
      "message",
      message,
      static_cast<int32_t>(millis()),
      0.0f,
      0.0f
    );
    
    AL_VERBOSE(esp_err_to_name(result));
  }

static void handleBleConnectedStatic() {
  ArmorLinkRuntime* self = instance();
  if (self != nullptr) {
    self->onBleConnected();
  }
}

  static void handleBleDisconnectedStatic() {
  ArmorLinkRuntime* self = instance();
  if (self != nullptr) {
    self->onBleDisconnected();
  }
}
  static void handleManagedReceiveStatic(const uint8_t* data, int len) {
#if ARMORLINK_HAS_ESPNOW
    ArmorLinkRuntime* self = instance();
    if (self != nullptr) {
      self->onEspNowDataReceived(data, len);
    }
#endif
  }

  static void handleManagedSendStatic(const uint8_t* mac_addr, esp_now_send_status_t status) {
#if ARMORLINK_HAS_ESPNOW
    ArmorLinkRuntime* self = instance();
    if (self != nullptr) {
      self->onEspNowSendStatus(mac_addr, status);
    }
#endif
  }

  static ArmorLinkRuntime*& instance() {
    static ArmorLinkRuntime* s_instance = nullptr;
    return s_instance;
  }

  bool sendPairAnnounce() {
    if (_module == nullptr) {
      return false;
    }

    if (_pairingSessionId == 0) {
      _pairingSessionId = nextArmorLinkPacketSeq();
      if (_pairingSessionId == 0) {
        _pairingSessionId = 1;
      }
    }

    String payload;
    {
      StaticJsonDocument<160> doc;
      doc["sid"] = _pairingSessionId;
      doc["gn"] = _module->name();      
      doc["gm"] = localMacString();
      doc["rp"] = _pairingInfo.recoveryPin;
      serializeJson(doc, payload);
    }

    const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_PAIR_ANNOUNCE,
      _module->name().c_str(),
      "*",
      "pairing",
      "announce"
    );
    setArmorLinkPacketPayload(out, payload);

    return _transport.sendPacketToMac(broadcastMac, out) == ESP_OK;
  }

  bool sendPairAnnounceBurst() {
    bool anyOk = false;

    for (uint8_t attempt = 0; attempt < PAIR_ANNOUNCE_SEND_ATTEMPTS; ++attempt) {
      if (sendPairAnnounce()) {
        anyOk = true;
      }

      if (attempt + 1 < PAIR_ANNOUNCE_SEND_ATTEMPTS) {
        delay(45);
        yield();
      }
    }

    return anyOk;
  }

  void handlePairAnnouncePacket(const ArmorLinkPacket& msg) {
    AL_VERBOSELN("[PAIR] Announce packet received");

    if (_isGatewayMode || _module == nullptr) {
      AL_VERBOSELN("[PAIR] Ignored: gateway mode or no module");
      return;
    }

    const String announcePayload = armorLinkPacketPayloadToString(msg);
    StaticJsonDocument<160> doc;
    auto err = deserializeJson(doc, announcePayload);
    if (err != DeserializationError::Ok) {
      AL_INTERNAL_PRINT("[PAIR] Announce JSON parse failed: ");
      AL_VERBOSE(err.c_str());
      return;
    }

    AL_VERBOSELN("[PAIR] Valid announce payload parsed");

    const uint16_t sessionId = doc["sid"] | doc["sessionId"] | 0;
    const String gatewayName = String((const char*)(doc["gn"] | doc["gatewayName"] | ""));
    const String gatewayMac  = String((const char*)(doc["gm"] | doc["gatewayMac"] | ""));
    const uint32_t recoveryPin = doc["rp"] | doc["recoveryPin"] | 0;
    AL_INTERNAL_PRINTF("[PAIR] announce gatewayName=%s gatewayMac=%s sessionId=%u\n",
              gatewayName.c_str(),
              gatewayMac.c_str(),
              sessionId);
    if (sessionId == 0 || gatewayName.isEmpty() || gatewayMac.isEmpty()) {
      AL_VERBOSELN("[PAIR] Ignored: invalid announce payload");
      return;
    }

    enterRemotePairingQuiet(sessionId);

    if (_pairingInfo.paired) {
      AL_VERBOSELN("[PAIR] Quiet active; paired module does not respond to announce");
      return;
    }

    const uint32_t now = millis();
    if (_lastRespondedPairSessionId == sessionId &&
        _lastPairResponseMs != 0 &&
        (uint32_t)(now - _lastPairResponseMs) < PAIR_RESPONSE_REPEAT_MIN_INTERVAL_MS) {
      AL_VERBOSELN("[PAIR] Ignored: response rate limited for this session");
      return;
    }

    String payload;
    {
      StaticJsonDocument<320> reply;
      reply["sessionId"] = sessionId;
      reply["gatewayMac"] = gatewayMac;
      reply["moduleName"] = _module->name();
      reply["moduleType"] = moduleTypeToString(_module->type());
      reply["moduleMac"] = localMacString();            
      serializeJson(reply, payload);
    }

    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_PAIR_RESPONSE,
      _module->name().c_str(),
      gatewayName.c_str(),
      "pairing",
      "response"
    );
    setArmorLinkPacketPayload(out, payload);

    if (recoveryPin != 0) {
      _pairingInfo.recoveryPin = recoveryPin;
    }

    uint8_t gatewayMacBytes[6];
    const bool hasGatewayMac = armorLinkParseMacString(gatewayMac, gatewayMacBytes);
    esp_err_t unicastResult = ESP_ERR_NOT_FOUND;
    esp_err_t broadcastResult = ESP_ERR_NOT_FOUND;

    if (!hasGatewayMac) {
      AL_VERBOSELN("[PAIR] Invalid gateway MAC in announce payload");
    }

    const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    for (uint8_t attempt = 0; attempt < PAIR_RESPONSE_SEND_ATTEMPTS; ++attempt) {
      if (hasGatewayMac) {
        unicastResult = _transport.sendPacketToMac(gatewayMacBytes, out);
      }

      delay(20);
      yield();

      broadcastResult = _transport.sendPacketToMac(broadcastMac, out);

      if (attempt + 1 < PAIR_RESPONSE_SEND_ATTEMPTS) {
        delay(45);
        yield();
      }
    }

    AL_INTERNAL_PRINTF("[PAIR] Response results | attempts=%u | unicast=%s | broadcast=%s | gateway=%s (%s) | sessionId=%u | payloadLen=%u\n",
                  PAIR_RESPONSE_SEND_ATTEMPTS,
                  esp_err_to_name(unicastResult),
                  esp_err_to_name(broadcastResult),
                  gatewayName.c_str(),
                  gatewayMac.c_str(),
                  sessionId,
                  out.payloadLen);

    if (unicastResult == ESP_OK || broadcastResult == ESP_OK) {
      _lastRespondedPairSessionId = sessionId;
      _lastPairResponseMs = now;
    }
  }

  void handlePairResponsePacket(const ArmorLinkPacket& msg) {
#if ARMORLINK_HAS_ESPNOW
    AL_INTERNAL_PRINTLN("[PAIR][GW] handlePairResponsePacket entered");

    if (!_isGatewayMode) {
      AL_INTERNAL_PRINTLN("[PAIR][GW] Ignored: gateway mode is false");
      return;
    }

    if (!_pairingActive) {
      AL_INTERNAL_PRINTLN("[PAIR][GW] Ignored: pairing is not active");
      return;
    }

    if ((int32_t)(millis() - _pairingEndsAtMs) >= 0) {
      AL_INTERNAL_PRINTLN("[PAIR][GW] Ignored: pairing timeout reached");
      stopPairing();
      return;
    }

    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, armorLinkPacketPayloadToString(msg)) != DeserializationError::Ok) {
      AL_INTERNAL_PRINTF("[PAIR][GW] JSON parse failed | payloadLen=%u | payload=%s\n",
                    msg.payloadLen,
                    armorLinkPacketPayloadToString(msg).c_str());
      return;
    }

    AL_INTERNAL_PRINTLN("[PAIR][GW] JSON parse ok");

    const String responseGatewayMac = String((const char*)(doc["gatewayMac"] | ""));
    const String localGatewayMac = localMacString();

    if (responseGatewayMac.isEmpty()) {
      AL_INTERNAL_PRINTLN("[PAIR][GW] ignored: response missing gatewayMac");
      return;
    }

    if (!responseGatewayMac.equalsIgnoreCase(localGatewayMac)) {
      AL_INTERNAL_PRINTF("[PAIR][GW] ignored: response for another gateway | responseGatewayMac=%s local=%s\n",
                    responseGatewayMac.c_str(),
                    localGatewayMac.c_str());
      return;
    }

    const uint16_t sessionId = doc["sessionId"] | 0;

    AL_INTERNAL_PRINTF("[PAIR][GW] sessionId=%u expected=%u\n",
                  sessionId,
                  _pairingSessionId);

    if (sessionId != _pairingSessionId) {
      AL_INTERNAL_PRINTLN("[PAIR][GW] Ignored: session mismatch");
      return;
    }

    const String moduleName = String((const char*)(doc["moduleName"] | ""));
    const String moduleType = String((const char*)(doc["moduleType"] | ""));
    const String moduleMac  = String((const char*)(doc["moduleMac"] | ""));
    const String moduleVersion = String((const char*)(doc["moduleVersion"] | "1.0"));
    const String armorLinkVersion = String((const char*)(doc["armorLinkVersion"] | ""));

    AL_INTERNAL_PRINTF("[PAIR][GW] moduleName=%s moduleType=%s moduleMac=%s\n",
                  moduleName.c_str(),
                  moduleType.c_str(),
                  moduleMac.c_str());

    if (moduleName.isEmpty() || moduleMac.isEmpty()) {
      AL_INTERNAL_PRINTLN("[PAIR][GW] Ignored: missing module name or MAC");
      return;
    }

    int index = findPendingCandidateIndexByMac(moduleMac);
    if (index < 0) {
      index = findFreePendingCandidateIndex();
      if (index < 0) {
        AL_INTERNAL_PRINTLN("[PAIR][GW] Ignored: candidate table full");
        return;
      }
    }

    _pendingCandidates[index].occupied = true;
    _pendingCandidates[index].sessionId = sessionId;
    _pendingCandidates[index].lastSeenMs = millis();
    armorlinkCopyString(_pendingCandidates[index].name, sizeof(_pendingCandidates[index].name), moduleName);
    armorlinkCopyString(_pendingCandidates[index].type, sizeof(_pendingCandidates[index].type), moduleType);
    armorlinkCopyString(_pendingCandidates[index].mac, sizeof(_pendingCandidates[index].mac), moduleMac);
    armorlinkCopyString(_pendingCandidates[index].moduleVersion, sizeof(_pendingCandidates[index].moduleVersion), moduleVersion);
    armorlinkCopyString(_pendingCandidates[index].armorLinkVersion, sizeof(_pendingCandidates[index].armorLinkVersion), armorLinkVersion);

    rebuildPendingCandidateCount();
    AL_INTERNAL_PRINTLN("[PAIR][GW] Emitting pairing_candidate BLE event");
    emitPairingCandidateEvent(_pendingCandidates[index]);
#endif
  }

  void handlePairAcceptPacket(const ArmorLinkPacket& msg) {
    AL_VERBOSELN("[PAIR] ACCEPT packet received");
    initStartupHelloState();
    if (_isGatewayMode || _module == nullptr) {
      return;
    }

    StaticJsonDocument<160> doc;
    if (deserializeJson(doc, armorLinkPacketPayloadToString(msg)) != DeserializationError::Ok) {
      return;
    }

    AL_VERBOSELN("[PAIR] ACCEPT payload parsed");

    const String gatewayName = String((const char*)(doc["gn"] | doc["gatewayName"] | ""));
    const String gatewayMac  = String((const char*)(doc["gm"] | doc["gatewayMac"] | ""));
    const uint32_t recoveryPin = doc["rp"] | doc["recoveryPin"] | 0;

    AL_INTERNAL_PRINTF("[PAIR] gatewayName=%s gatewayMac=%s\n",
                  gatewayName.c_str(),
                  gatewayMac.c_str());

    if (gatewayName.isEmpty() || gatewayMac.isEmpty()) {
      return;
    }

    _pairingInfo.paired = true;
    armorlinkCopyString(_pairingInfo.gatewayName, sizeof(_pairingInfo.gatewayName), gatewayName);
    armorlinkCopyString(_pairingInfo.gatewayMac, sizeof(_pairingInfo.gatewayMac), gatewayMac);
    _pairingInfo.recoveryPin = recoveryPin;
    _storage.savePairingInfo(_pairingInfo);
    AL_VERBOSELN("[PAIR] Pairing info saved");

    uint8_t macBytes[6];
if (armorLinkParseMacString(gatewayMac, macBytes)) {
  _peers.registerPeer(gatewayName, macBytes);
  AL_VERBOSELN("[PAIR] Gateway peer registered");
  _options.defaultLogTarget = gatewayName;
}

initStartupHelloState();
_lastHeartbeatMs = 0;
AL_VERBOSELN("[PAIR] Module is now paired");
emitPairingLinkedEvent(gatewayName, gatewayMac);
  }

  static const char* moduleTypeToString(ArmorLinkModuleType type) {
    switch (type) {
      case ArmorLinkModuleType::Generic: return "Generic";
      case ArmorLinkModuleType::Chest:   return "Chest";
      case ArmorLinkModuleType::Helmet:  return "Helmet";
      case ArmorLinkModuleType::Back:    return "Back";
      case ArmorLinkModuleType::Arm:     return "Arm";
      case ArmorLinkModuleType::Hand:    return "Hand";
      case ArmorLinkModuleType::Leg:     return "Leg";
      case ArmorLinkModuleType::Prop:    return "Prop";
      default:                           return "Unknown";
    }
  }

  String localMacString() const {
#if ARMORLINK_HAS_ESPNOW
#if defined(ESP32)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    return armorLinkMacToString(mac);
#elif defined(ESP8266)
    return WiFi.macAddress();
#else
    return "";
#endif
#elif defined(ESP32)
    // No WiFi driver without ESP-NOW; read the station MAC from eFuse instead,
    // so the gateway_descriptor still names this node.
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) return String();
    return armorLinkMacToString(mac);
#else
    return String();
#endif
  }

  int findPendingCandidateIndexByMac(const String& macText) const {
#if ARMORLINK_HAS_ESPNOW
    for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
      if (_pendingCandidates[i].occupied && macText.equalsIgnoreCase(_pendingCandidates[i].mac)) {
        return static_cast<int>(i);
      }
    }
    return -1;
#else
    return -1;
#endif
  }

  int findFreePendingCandidateIndex() const {
#if ARMORLINK_HAS_ESPNOW
    for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
      if (!_pendingCandidates[i].occupied) {
        return static_cast<int>(i);
      }
    }
    return -1;
#else
    return -1;
#endif
  }

  void clearPendingPairingCandidates() {
#if ARMORLINK_HAS_ESPNOW
    memset(static_cast<void*>(_pendingCandidates), 0, sizeof(_pendingCandidates));
    _pendingCandidateCount = 0;
#endif
  }

  void rebuildPendingCandidateCount() {
#if ARMORLINK_HAS_ESPNOW
    size_t count = 0;
    for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
      if (_pendingCandidates[i].occupied) {
        ++count;
      }
    }
    _pendingCandidateCount = count;
#endif
  }

  void sendStateSyncToModule(const char* moduleName, const char* moduleMac) {
  if (!_isGatewayMode || _module == nullptr) {
    return;
  }

  if (moduleName == nullptr || moduleMac == nullptr ||
      strlen(moduleName) == 0 || strlen(moduleMac) == 0) {
    return;
  }

  String payload;
  {
    StaticJsonDocument<192> doc;
    doc["loggingEnabled"] = _remoteLoggingEnabled;
    doc["logLevel"] = _remoteLogLevel;
    doc["telemetryEnabled"] = _remoteTelemetryEnabled;
    serializeJson(doc, payload);
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    moduleName,
    "system",
    "sync_state"
  );

  setArmorLinkPacketPayload(out, payload);

  const uint8_t broadcastMac[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
  };
  (void)_transport.sendPacketToMac(broadcastMac, out);
}

void syncLoggingStateToModule(const char* moduleName, const char* moduleMac) {
  if (!_isGatewayMode || _module == nullptr) {
    return;
  }

  if (moduleName == nullptr || moduleMac == nullptr || strlen(moduleName) == 0 || strlen(moduleMac) == 0) {
    return;
  }

  armorlinkCopyString(_loggingSyncModuleName, sizeof(_loggingSyncModuleName), moduleName);
  armorlinkCopyString(_loggingSyncModuleMac, sizeof(_loggingSyncModuleMac), moduleMac);
  _loggingSyncStage = 0;
  _loggingSyncPending = true;
  _loggingSyncAtMs = millis() + 1000;
  
}

void loggingSyncTick() {
#if ARMORLINK_HAS_ESPNOW
  if (!_loggingSyncPending || !_isGatewayMode || _module == nullptr) {
    return;
  }

  const uint32_t now = millis();
  if ((int32_t)(now - _loggingSyncAtMs) < 0) {
    return;
  }

  const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  if (_loggingSyncStage == 0) {
    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      _loggingSyncModuleName,
      "logs",
      "set_enabled"
    );
    out.valueInt = _remoteLoggingEnabled ? 1 : 0;
    setArmorLinkPacketPayload(out, _remoteLoggingEnabled ? "true" : "false");

    (void)_transport.sendPacketToMac(broadcastMac, out);

    _loggingSyncStage = 1;
    _loggingSyncAtMs = now + 250;

    _loggingSyncStage = 1;
    _loggingSyncAtMs = now + 150;
    return;
  }

  if (_loggingSyncStage == 1) {
    ArmorLinkPacket out = makeArmorLinkBasePacket(
      AL_MSG_COMMAND,
      _module->name().c_str(),
      _loggingSyncModuleName,
      "logs",
      "set_level"
    );
    out.valueInt = _remoteLogLevel;
    setArmorLinkPacketPayload(out, logLevelToKeyword(_remoteLogLevel));

    (void)_transport.sendPacketToMac(broadcastMac, out);

    _loggingSyncPending = false;
    _loggingSyncStage = 0;    
    return;
  }

  _loggingSyncPending = false;
  _loggingSyncStage = 0;
#endif
  }

  bool unpairPairedModuleByMac(const String& macText) {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || macText.isEmpty()) {
      return false;
    }

    size_t index = ArmorLinkStorage::MAX_PAIRED_MODULES;

    for (size_t i = 0; i < _pairedModuleCount; ++i) {
      if (String(_pairedModules[i].mac).equalsIgnoreCase(macText)) {
        index = i;
        break;
      }
    }

    if (index == ArmorLinkStorage::MAX_PAIRED_MODULES) {
      AL_INTERNAL_PRINTF("[PAIR][GW] Unpair failed, module not found: %s\n", macText.c_str());
      return false;
    }

    ArmorLinkStoredPairedModule removed = _pairedModules[index];

    uint8_t moduleMacBytes[6];
    if (armorLinkParseMacString(String(removed.mac), moduleMacBytes)) {
      ArmorLinkPacket out = makeArmorLinkBasePacket(
        AL_MSG_COMMAND,
        (_module != nullptr) ? _module->name().c_str() : "Gateway",
        removed.name,
        "pairing",
        "unpair"
      );

      setArmorLinkPacketPayload(out, String(removed.mac));

      esp_err_t sendResult = _transport.sendPacketToMac(moduleMacBytes, out);

      AL_INTERNAL_PRINTF("[PAIR][GW] Unpair command sent to %s (%s): %s\n",
                    removed.name,
                    removed.mac,
                    esp_err_to_name(sendResult));
    } else {
      AL_INTERNAL_PRINTF("[PAIR][GW] Invalid MAC while unpairing: %s\n", removed.mac);
    }

    for (size_t i = index; i + 1 < _pairedModuleCount; ++i) {
      _pairedModules[i] = _pairedModules[i + 1];
    }

    if (_pairedModuleCount > 0) {
      memset(static_cast<void*>(&_pairedModules[_pairedModuleCount - 1]), 0, sizeof(ArmorLinkStoredPairedModule));
      _pairedModuleCount--;
    }

    _storage.savePairedModules(_pairedModules, _pairedModuleCount);
    syncPresenceStatesFromStoredModules();

    emitModuleUnpairedEvent(removed);

    AL_INTERNAL_PRINTF("[PAIR][GW] Module unpaired: %s (%s)\n",
                  removed.name,
                  removed.mac);

    return true;
#else
    return false;
#endif
  }
void emitModuleUnpairedEvent(const ArmorLinkStoredPairedModule& module) {
  if (!_isGatewayMode) {
    return;
  }

  StaticJsonDocument<256> doc;
  doc["type"] = "module_unpaired";

  JsonObject mod = doc.createNestedObject("module");
  mod["name"] = module.name;
  mod["type"] = module.type;
  mod["mac"] = module.mac;

  String out;
  serializeJson(doc, out);
  doc.clear();
  bleNotifyEventJson(out);
}
  void upsertStoredPairedModule(const char* name, const char* type, const char* mac) {
#if ARMORLINK_HAS_ESPNOW
    size_t index = ArmorLinkStorage::MAX_PAIRED_MODULES;

    for (size_t i = 0; i < _pairedModuleCount; ++i) {
      if (String(_pairedModules[i].mac).equalsIgnoreCase(mac)) {
        index = i;
        break;
      }
    }

    if (index == ArmorLinkStorage::MAX_PAIRED_MODULES) {
      if (_pairedModuleCount >= ArmorLinkStorage::MAX_PAIRED_MODULES) {
        return;
      }
      index = _pairedModuleCount++;
    }

    armorlinkCopyString(_pairedModules[index].name, sizeof(_pairedModules[index].name), name);
    armorlinkCopyString(_pairedModules[index].type, sizeof(_pairedModules[index].type), type);
    armorlinkCopyString(_pairedModules[index].mac, sizeof(_pairedModules[index].mac), mac);

    _storage.savePairedModules(_pairedModules, _pairedModuleCount);

    AL_INTERNAL_PRINTF("[PAIR][GW] Stored paired modules count=%u\n",
                  static_cast<unsigned>(_pairedModuleCount));
    syncPresenceStatesFromStoredModules();

    uint8_t macBytes[6];

    if (armorLinkParseMacString(String(mac), macBytes)) {

      _peers.registerPeer(String(name), macBytes);

      AL_INTERNAL_PRINTF("[PAIR][GW] Registered paired peer: %s (%s)\n",

                    name,

                    mac);

    }
#endif
  }

  void emitPairingStateEvent(const char* type, uint16_t sessionIdOverride = 0) {
    if (!_isGatewayMode) {
      return;
    }

    const uint16_t sessionId = sessionIdOverride != 0 ? sessionIdOverride : _pairingSessionId;
    if (equalsIgnoreCase(type, "pairing_timeout") &&
        (!_pairingActive || sessionId == 0)) {
      AL_INTERNAL_PRINTF("[PAIR][GW] Suppressed stale pairing_timeout | sessionId=%u | active=%s\n",
                    sessionId,
                    _pairingActive ? "true" : "false");
      return;
    }

    StaticJsonDocument<160> doc;
    doc["type"] = type;
    doc["sessionId"] = sessionId;

    String out;
    serializeJson(doc, out);
    doc.clear();
    bleNotifyEventJson(out);
    emitSerialTransportJson(out);
  }

  void emitPairingCandidateEvent(const ArmorLinkPairingCandidate& candidate) {
    if (!_isGatewayMode) {
      return;
    }

    StaticJsonDocument<256> doc;
    doc["type"] = "pairing_candidate";
    doc["sessionId"] = candidate.sessionId;

    JsonObject module = doc.createNestedObject("module");
    module["name"] = candidate.name;
    module["type"] = candidate.type;
    module["mac"] = candidate.mac;
    module["moduleVersion"] = strlen(candidate.moduleVersion) > 0 ? candidate.moduleVersion : "1.0";
    module["armorLinkVersion"] = candidate.armorLinkVersion;
    if (_options.enableSerialMenu) {
      printPairingCandidates();
    }
    String out;
    serializeJson(doc, out);
    AL_INTERNAL_PRINTF("[PAIR][GW] BLE pairing_candidate event: %s\n", out.c_str());
    bleNotifyEventJson(out);
    emitSerialTransportJson(out);
  }

  void emitPendingPairingCandidateEvents() {
#if ARMORLINK_HAS_ESPNOW
    if (!_isGatewayMode || !_pairingActive) {
      return;
    }

    for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
      if (!_pendingCandidates[i].occupied ||
          _pendingCandidates[i].sessionId != _pairingSessionId) {
        continue;
      }

      emitPairingCandidateEvent(_pendingCandidates[i]);
      delay(20);
      yield();
    }
#endif
  }

  void emitPairingCompletedEvent(const ArmorLinkPairingCandidate& candidate) {
    if (!_isGatewayMode) {
      return;
    }

    StaticJsonDocument<256> doc;
    doc["type"] = "pairing_completed";
    doc["sessionId"] = candidate.sessionId;

    JsonObject module = doc.createNestedObject("module");
    module["name"] = candidate.name;
    module["type"] = candidate.type;
    module["mac"] = candidate.mac;

    AL_INTERNAL_PRINTF("[PAIR][GW] Pairing completed for %s | %s\n",
                  candidate.name,
                  candidate.mac);

    String out;
    serializeJson(doc, out);
    doc.clear();
    bleNotifyEventJson(out);
    emitSerialTransportJson(out);
  }

  void emitPairingLinkedEvent(const String& gatewayName, const String& gatewayMac) {
    StaticJsonDocument<224> doc;
    doc["type"] = "pairing_linked";
    doc["gatewayName"] = gatewayName;
    doc["gatewayMac"] = gatewayMac;

    String out;
    serializeJson(doc, out);
    doc.clear();
    bleNotifyEventJson(out);
    emitSerialTransportJson(out);
  }

  static String resolveNamespace(const ArmorLinkModule& module, const ArmorLinkOptions& options) {
    if (!options.nvsNamespace.isEmpty() && options.nvsNamespace != "armorlink") {
      return options.nvsNamespace;
    }

    String ns = module.name();
    if (ns.isEmpty()) {
      ns = "armorlink";
    }

    return ns;
  }

  static bool equalsIgnoreCase(const char* a, const char* b) {
    return a && b && strcasecmp(a, b) == 0;
  }

  void logStructuredSerial(const String& level, const String& entity, const String& command, const String& message) {
    if (_options.enableSerialLogging) {
      Serial.printf("[%s] %s.%s -> %s\n",
                    level.c_str(),
                    entity.c_str(),
                    command.c_str(),
                    message.c_str());
    }
  }

  void emitSerialEventJson(const String& json) {
#if ARMORLINK_HAS_WEB_SERIAL
    if (json.isEmpty() ||
        _localConfigTransferPending ||
        _localConfigTransferActive) {
      return;
    }

    emitSerialTransportJson(json);
#endif
  }

  void emitSerialTransportJson(const String& json) {
#if ARMORLINK_HAS_WEB_SERIAL
    if (json.isEmpty() || !isSerialWebClientActive()) return;
    Serial.print("@ALF:");
    Serial.println(json);
#endif
  }

int findPairedModuleIndexByName(const char* name) const {
#if ARMORLINK_HAS_ESPNOW
  if (name == nullptr || strlen(name) == 0) return -1;

  for (size_t i = 0; i < _pairedModuleCount; ++i) {
    if (String(_pairedModules[i].name).equalsIgnoreCase(name)) {
      return static_cast<int>(i);
    }
  }

  return -1;
#else
    return -1;
#endif
  }

bool isPendingPairingCandidateName(const char* name) const {
#if ARMORLINK_HAS_ESPNOW
  if (name == nullptr || strlen(name) == 0) return false;

  for (size_t i = 0; i < MAX_PENDING_PAIRING_CANDIDATES; ++i) {
    if (_pendingCandidates[i].occupied &&
        _pendingCandidates[i].sessionId == _pairingSessionId &&
        equalsIgnoreCase(_pendingCandidates[i].name, name)) {
      return true;
    }
  }

  return false;
#else
    return false;
#endif
  }

bool isGatewayPacketAllowedFromUnknownSource(const ArmorLinkPacket& msg) const {
  if (!_isGatewayMode) return true;
  if (strlen(msg.source) == 0) return true;

  if (_module != nullptr && equalsIgnoreCase(msg.source, _module->name().c_str())) {
    return true;
  }

  if (findPairedModuleIndexByName(msg.source) >= 0) {
    return true;
  }

  if (_pairingActive && isPendingPairingCandidateName(msg.source)) {
    return true;
  }

  // Pairing muss natrlich von unbekannten Modulen erlaubt bleiben
  if (msg.msgType == AL_MSG_PAIR_RESPONSE ||
      msg.msgType == AL_MSG_PAIR_ANNOUNCE ||
      msg.msgType == AL_MSG_PAIR_ACCEPT) {
    return true;
  }

  if (msg.msgType == AL_MSG_COMMAND &&
      equalsIgnoreCase(msg.entity, "pairing") &&
      (equalsIgnoreCase(msg.command, "response") ||
       equalsIgnoreCase(msg.command, "required"))) {
    return true;
  }

  return false;
}

String extractModuleMacFromPayload(const ArmorLinkPacket& msg) const {
  const String payload = armorLinkPacketPayloadToString(msg);
  if (payload.isEmpty()) return "";

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload) != DeserializationError::Ok) {
    return "";
  }

  const String moduleMac = String((const char*)(doc["moduleMac"] | ""));
  if (!moduleMac.isEmpty()) return moduleMac;

  if (doc["module"].is<JsonObject>()) {
    return String((const char*)(doc["module"]["mac"] | ""));
  }

  return "";
}

void sendUnpairToUnknownModule(const ArmorLinkPacket& msg) {
  if (!_isGatewayMode || _module == nullptr || strlen(msg.source) == 0) {
    return;
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_COMMAND,
    _module->name().c_str(),
    msg.source,
    "pairing",
    "unpair"
  );

  setArmorLinkPacketPayload(out, "unknown_to_gateway");

  const String moduleMac = extractModuleMacFromPayload(msg);

  esp_err_t result = ESP_ERR_NOT_FOUND;

  if (!moduleMac.isEmpty()) {
    uint8_t macBytes[6];
    if (armorLinkParseMacString(moduleMac, macBytes)) {
      result = _transport.sendPacketToMac(macBytes, out);
    }
  }

  if (result != ESP_OK) {
    const uint8_t broadcastMac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    result = _transport.sendPacketToMac(broadcastMac, out);
  }

  AL_INTERNAL_PRINTF("[PAIR][GW] Unknown module packet from %s -> unpair notice sent (%s)\n",
                msg.source,
                esp_err_to_name(result));
}

  void processIncomingEspNowPacket(const ArmorLinkPacket& msg) {
#if ARMORLINK_HAS_ESPNOW
    if (_isGatewayMode && _pairingActive && !isPairingTraffic(msg)) {
      AL_INTERNAL_PRINTF("[PAIR][GW] Quiet drop while pairing | source=%s type=%u entity=%s command=%s\n",
                    msg.source,
                    msg.msgType,
                    msg.entity,
                    msg.command);
      return;
    }

    if (!_isGatewayMode &&
        isRemotePairingQuietActive() &&
        !isPairingTraffic(msg)) {
      AL_INTERNAL_PRINTF("[PAIR] Quiet ignore while gateway pairing | source=%s type=%u entity=%s command=%s\n",
                    msg.source,
                    msg.msgType,
                    msg.entity,
                    msg.command);
      return;
    }

    if (_isGatewayMode && !isGatewayPacketAllowedFromUnknownSource(msg)) {
      AL_INTERNAL_PRINTF("[PAIR][GW] Dropping packet from unknown module source=%s type=%u entity=%s command=%s\n",
                    msg.source,
                    msg.msgType,
                    msg.entity,
                    msg.command);

      sendUnpairToUnknownModule(msg);
      return;
    }

    if (_isGatewayMode && strlen(msg.source) > 0) {
      const int pairedIndex = findPairedModuleIndexByName(msg.source);
      const bool configTransferPacket =
        msg.msgType == AL_MSG_CONFIG_META ||
        msg.msgType == AL_MSG_CONFIG_CHUNK ||
        msg.msgType == AL_MSG_CONFIG_END;

      const bool handledSystemPresencePacket =
        equalsIgnoreCase(msg.entity, "system") &&
        (equalsIgnoreCase(msg.command, "hello") ||
         equalsIgnoreCase(msg.command, "heartbeat") ||
         equalsIgnoreCase(msg.command, "sync_request"));

      if (pairedIndex >= 0 && !configTransferPacket && !handledSystemPresencePacket) {
        updateModulePresenceByMac(String(_pairedModules[pairedIndex].mac));
      }
    }

    if (shouldReportPairingRequired(msg)) {
      sendPairingRequiredEvent(msg);
      return;
    }
    switch (msg.msgType) {
      case AL_MSG_LOG: {
        String text = armorLinkPacketPayloadToString(msg);

        AL_INTERNAL_PRINTF("[%s] %s | %s.%s -> %s\n",
                      armorLinkLogLevelToString(msg.level),
                      msg.source,
                      msg.entity,
                      msg.command,
                      text.c_str());

        StaticJsonDocument<384> doc;
        doc["type"] = "log";
        doc["level"] = armorLinkLogLevelToString(msg.level);
        doc["source"] = msg.source;
        doc["entity"] = msg.entity;
        doc["command"] = msg.command;
        doc["message"] = text;
        doc["valueInt"] = msg.valueInt;
        doc["valueFloat"] = msg.valueFloat;
        doc["batteryVoltage"] = msg.batteryVoltage;

        String out;
        serializeJson(doc, out);
        doc.clear();
        bleNotifyLogJson(out);
        break;
      }
      case AL_MSG_TELEMETRY: {        
        String payload = armorLinkPacketPayloadToString(msg);

        StaticJsonDocument<384> inDoc;
        DeserializationError err = deserializeJson(inDoc, payload);
        if (err) {
          inDoc.clear();
          logWarn("telemetry", "decode", String("Invalid telemetry JSON: ") + err.c_str());
          break;
        }

        StaticJsonDocument<384> doc;
        doc["type"] = "telemetry";
        doc["source"] = strlen(msg.source) > 0 ? msg.source : (inDoc["source"] | "");
        doc["group"] = inDoc["group"] | "";
        doc["name"] = inDoc["name"] | "";

        if (inDoc["unit"].is<const char*>()) {
          doc["unit"] = inDoc["unit"];
        }

        if (inDoc["values"].is<JsonObject>()) {
          doc["values"] = inDoc["values"];
        } else if (inDoc["value"].is<float>() || inDoc["value"].is<int>()) {
          doc["value"] = inDoc["value"];
        }

        String out;
        serializeJson(doc, out);
        doc.clear();
        inDoc.clear();
        bleNotifyEventJson(out);
        emitSerialEventJson(out);
        break;
      }
      case AL_MSG_DATA_STREAM_SUBSCRIBE: {
        handleDataStreamSubscribePacket(msg, true);
        break;
      }

      case AL_MSG_DATA_STREAM_UNSUBSCRIBE: {
        handleDataStreamSubscribePacket(msg, false);
        break;
      }

      case AL_MSG_DATA_STREAM: {
        handleDataStreamPacket(msg);
        break;
      }
      case AL_MSG_CONFIG_GET: {
        const bool targetMatches =
          _module != nullptr &&
          (equalsIgnoreCase(msg.target, _module->name().c_str()) ||
          equalsIgnoreCase(msg.target, "*"));

        if (!targetMatches) {
          break;
        }

        String json = buildBleDescriptor();
        AL_INTERNAL_PRINTF("[CONFIG] BLE descriptor length=%u for requestId=%u\n",
                      static_cast<unsigned>(json.length()),
                      msg.requestId);

        if (json.isEmpty()) {
          logError("config", "get", "Descriptor allocation failed");
          break;
        }

        esp_err_t result = ESP_ERR_NOT_FOUND;
        uint8_t gatewayMacBytes[6];
        if (!_isGatewayMode &&
            strlen(_pairingInfo.gatewayMac) > 0 &&
            armorLinkParseMacString(String(_pairingInfo.gatewayMac), gatewayMacBytes)) {
          result = _transport.sendConfigJsonChunkedToMac(
            gatewayMacBytes,
            msg.source,
            _module->name().c_str(),
            msg.requestId,
            msg.entity,
            json,
            false
          );
        } else {
          result = _transport.sendConfigJsonChunkedToTarget(
            String(msg.source),
            _module->name().c_str(),
            msg.requestId,
            msg.entity,
            json,
            false
          );
        }

        AL_INTERNAL_PRINTF("[CONFIG] sent descriptor to %s requestId=%u -> %s\n",
                      msg.source,
                      msg.requestId,
                      esp_err_to_name(result));
        break;
      }

      case AL_MSG_CONFIG_SET: {
        const bool targetMatches =
          _module != nullptr &&
          (equalsIgnoreCase(msg.target, _module->name().c_str()) ||
          equalsIgnoreCase(msg.target, "*"));

        if (!targetMatches) {
          break;
        }

        const String payload = armorLinkPacketPayloadToString(msg);
        auto configResult = dispatchConfigSetPacket(
          String(msg.entity),
          String(msg.command),
          msg,
          payload);

        const bool ok = configResult == ArmorLinkDispatchResult::Ok;

        ArmorLinkPacket response = makeArmorLinkBasePacket(
          ok ? AL_MSG_ACK : AL_MSG_ERROR,
          _module->name().c_str(),
          msg.source,
          msg.entity,
          msg.command
        );

        response.requestId = msg.requestId;

        setArmorLinkPacketPayload(
          response,
          ok
            ? "Config updated"
            : String("Config update failed: ") + ArmorLinkDispatch::toString(configResult)
        );

        esp_err_t sendResult = _transport.sendPacketToTarget(String(msg.source), response);

        AL_INTERNAL_PRINTF("[CONFIG] set %s.%s -> %s | ack=%s\n",
                      msg.entity,
                      msg.command,
                      ArmorLinkDispatch::toString(configResult),
                      esp_err_to_name(sendResult));
        break;
      }
      case AL_MSG_CONFIG_META: {
        StaticJsonDocument<256> doc;
        doc["type"] = "config_meta";
        doc["requestId"] = msg.requestId;
        doc["target"] = msg.source;
        doc["entity"] = msg.entity;
        doc["partial"] = (msg.flags & AL_FLAG_PARTIAL_CFG) != 0;
        doc["chunkCount"] = msg.chunkCount;
        doc["totalLength"] = msg.valueInt;

        String out;
        serializeJson(doc, out);
        doc.clear();
        bleNotifyEventJson(out);
        emitSerialTransportJson(out);
        break;
      }

      case AL_MSG_PAIR_ANNOUNCE: {
        handlePairAnnouncePacket(msg);
        break;
      }

      case AL_MSG_PAIR_RESPONSE: {
        AL_INTERNAL_PRINTF("[PAIR][GW] Pair response received from %s | entity=%s | command=%s | payload=%s\n",
                      msg.source,
                      msg.entity,
                      msg.command,
                      armorLinkPacketPayloadToString(msg).c_str());
        handlePairResponsePacket(msg);
        break;
      }

      case AL_MSG_PAIR_ACCEPT: {
        handlePairAcceptPacket(msg);
        break;
      }

      case AL_MSG_CONFIG_CHUNK: {
        StaticJsonDocument<340> doc;
        doc["type"] = "config_chunk";
        doc["requestId"] = msg.requestId;
        doc["target"] = msg.source;
        doc["entity"] = msg.entity;
        doc["partial"] = (msg.flags & AL_FLAG_PARTIAL_CFG) != 0;
        doc["chunkIndex"] = msg.chunkIndex;
        doc["chunkCount"] = msg.chunkCount;
        doc["data"] = armorLinkPacketPayloadToString(msg);

        String out;
        serializeJson(doc, out);
        doc.clear();
        bleNotifyEventJson(out);
        emitSerialTransportJson(out);
        break;
      }

      case AL_MSG_CONFIG_END: {
        StaticJsonDocument<224> doc;
        doc["type"] = "config_end";
        doc["requestId"] = msg.requestId;
        doc["target"] = msg.source;
        doc["entity"] = msg.entity;
        doc["partial"] = (msg.flags & AL_FLAG_PARTIAL_CFG) != 0;
        doc["totalLength"] = msg.valueInt;

        String out;
        serializeJson(doc, out);
        doc.clear();
        bleNotifyEventJson(out);
        emitSerialTransportJson(out);
        break;
      }

      case AL_MSG_ACK: {
        StaticJsonDocument<256> doc;
        doc["type"] = "ack";
        doc["requestId"] = msg.requestId;
        doc["status"] = "ok";
        doc["source"] = msg.source;
        doc["message"] = armorLinkPacketPayloadToString(msg);

        String out;
        serializeJson(doc, out);
        doc.clear();
        bleNotifyEventJson(out);
        emitSerialTransportJson(out);
        break;
      }

      case AL_MSG_ERROR: {
        StaticJsonDocument<256> doc;  
        doc["type"] = "error";
        doc["requestId"] = msg.requestId;
        doc["source"] = msg.source;
        doc["message"] = armorLinkPacketPayloadToString(msg);

        String out;
        serializeJson(doc, out);
        doc.clear();
        bleNotifyEventJson(out);
        emitSerialTransportJson(out);
        break;
      }

        case AL_MSG_COMMAND: {
          if (handleInternalEspNowCommand(msg)) {
            return;
          }

          const bool targetMatches =
            _module != nullptr &&
            (equalsIgnoreCase(msg.target, _module->name().c_str()) ||
            equalsIgnoreCase(msg.target, "*"));

          if (targetMatches) {
            auto actionResult = _dispatch.handleAction(String(msg.entity), String(msg.command));
            if (actionResult == ArmorLinkDispatchResult::Ok) {
              return;
            }

            const String payload = armorLinkPacketPayloadToString(msg);
            auto configResult = dispatchConfigSetPacket(
              String(msg.entity),
              String(msg.command),
              msg,
              payload);

            if (configResult == ArmorLinkDispatchResult::Ok) {
              return;
            }
          }

          if (_espNowCommandHook && _espNowCommandHook(msg)) {
            return;
          }

          logWarn("espnow", "command", String("Unhandled command entity: ") + msg.entity);
          break;
        }
      }
#endif
  }
  
};

inline ArmorLinkRuntime ArmorLink;

inline esp_err_t ArmorLinkDataStreamBuilder::send() {
#if ARMORLINK_HAS_DATASTREAMS
  if (_rt == nullptr || _stream[0] == '\0') {
    return ESP_ERR_INVALID_STATE;
  }

  if (!_rt->_isGatewayMode && _rt->isRemotePairingQuietActive()) {
    return ESP_OK;
  }

  if (_rt->_isGatewayMode && _rt->_pairingActive) {
    return ESP_OK;
  }

  if (_rt->_localConfigTransferPending || _rt->_localConfigTransferActive) {
    return ESP_OK;
  }

  const char* sourceName = (_rt->_module != nullptr)
    ? _rt->_module->name().c_str()
    : "ArmorLink";

  const char* targetName = (_target[0] == '\0') ? "*" : _target;
  const char* effectiveTargetName = targetName;

  if (_rt->_isGatewayMode) {
    const bool hasRoute = _rt->hasGatewayDataStreamRoute(sourceName, _stream);
    if (!hasRoute) {
      static uint32_t lastNoRouteLogMs = 0;
      const uint32_t now = millis();
      if (lastNoRouteLogMs == 0 || (uint32_t)(now - lastNoRouteLogMs) >= 2000) {
        lastNoRouteLogMs = now;
        AL_INTERNAL_PRINTF("[STREAM][GW] No route for local stream %s.%s\n",
                      sourceName,
                      _stream);
      }
      return ESP_OK;
    }
  }

  if (!_rt->_isGatewayMode) {
    if (_rt->_options.defaultLogTarget.isEmpty()) {
      return ESP_ERR_NOT_FOUND;
    }
    if (!_rt->isDataStreamRequested(_stream)) {
      return ESP_OK;
    }
    effectiveTargetName = _rt->_options.defaultLogTarget.c_str();
  }

  if (_intervalMs > 0) {
    char rateKey[64] = { 0 };
    snprintf(rateKey,
             sizeof(rateKey),
             "%s.%s.%s",
             sourceName,
             effectiveTargetName,
             _stream);

    if (!_rt->allowDataStreamSend(rateKey, _intervalMs)) {
      return ESP_OK;
    }
  }

  String payload = _rawPayload;
  if (payload.isEmpty()) {
    if (_valueCount > 0) {
      StaticJsonDocument<384> doc;
      for (size_t i = 0; i < _valueCount; ++i) {
        switch (_values[i].type) {
          case ArmorLinkDataStreamBuilder::ValueType::Float:
            doc[_values[i].key] = _values[i].floatValue;
            break;

          case ArmorLinkDataStreamBuilder::ValueType::Int:
            doc[_values[i].key] = _values[i].intValue;
            break;

          case ArmorLinkDataStreamBuilder::ValueType::Bool:
            doc[_values[i].key] = _values[i].boolValue;
            break;
        }
      }
      serializeJson(doc, payload);
    } else {
      payload = "{}";
    }
  }

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_DATA_STREAM,
    sourceName,
    effectiveTargetName,
    "stream",
    _stream
  );
  setArmorLinkPacketPayload(out, payload);

  if (_rt->_isGatewayMode) {
    return _rt->routeDataStreamFromGateway(out);
  }

  return _rt->_transport.sendPacketToTarget(_rt->_options.defaultLogTarget, out);
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

inline esp_err_t ArmorLinkTelemetryBuilder::send() {
  if (_rt == nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!_rt->_remoteTelemetryEnabled && !_rt->_options.forceRemoteLogging) {
    return ESP_OK;
  }

  if (!_rt->_isGatewayMode && _rt->isRemotePairingQuietActive()) {
    return ESP_OK;
  }

  String rateKey = _group + "." + _name;
  if (!_rt->allowTelemetrySend(rateKey)) {
    return ESP_OK;
  }

  StaticJsonDocument<384> doc;
  doc["type"] = "telemetry";
  doc["source"] = (_rt->_module != nullptr) ? _rt->_module->name() : "ArmorLink";
  doc["group"] = _group;
  doc["name"] = _name;

  if (_unit.length() > 0) {
    doc["unit"] = _unit;
  }

  JsonObject values = doc.createNestedObject("values");
  for (auto kv : _values) {
    values[kv.first] = kv.second;
  }

  String payload;
  serializeJson(doc, payload);

  if (_rt->_isGatewayMode || !ARMORLINK_HAS_ESPNOW) {
    bleNotifyEventJson(payload);
    _rt->emitSerialEventJson(payload);
    return ESP_OK;
  }

  if (_rt->_options.defaultLogTarget.isEmpty()) {
    return ESP_ERR_NOT_FOUND;
  }

  const char* sourceName = (_rt->_module != nullptr)
    ? _rt->_module->name().c_str()
    : "ArmorLink";

  ArmorLinkPacket out = makeArmorLinkBasePacket(
    AL_MSG_TELEMETRY,
    sourceName,
    _rt->_options.defaultLogTarget.c_str(),
    _group.c_str(),
    _name.c_str()
  );

  setArmorLinkPacketPayload(out, payload);

  return _rt->_transport.sendPacketToTarget(
    _rt->_options.defaultLogTarget,
    out
  );
}
// ============================================================================
// DEFAULT BLE HANDLER
// ============================================================================
#if ARMORLINK_HAS_BLE
class ArmorLinkAutoBleCommandHandler : public ArmorLinkBleCommandHandler {
public:
  explicit ArmorLinkAutoBleCommandHandler(const char* localTarget)
    : _localTarget(localTarget) {}

  void handleCommand() override {
    const String payload = armorLinkPacketPayloadToString(packet);

    if (ArmorLink.handleDefaultBlePacket(
          packet,
          payload,
          _localTarget,
          [](bool enabled) {
            setBleLogStreamEnabled(enabled);
          },
          &ArmorLinkAutoBleCommandHandler::sendAck,
          &ArmorLinkAutoBleCommandHandler::sendError,
          &ArmorLinkAutoBleCommandHandler::sendEventJson)) {
      return;
    }

    sendError(packet.requestId, "Unhandled BLE command", "");
  }

private:
  const char* _localTarget;

  static void sendAck(uint16_t requestId, const String& status, const String& message) {
    StaticJsonDocument<256> doc;
    doc["type"] = "ack";
    doc["requestId"] = requestId;
    doc["status"] = status;
    if (!message.isEmpty()) {
      doc["message"] = message;
    }

    String out;
    serializeJson(doc, out);
    doc.clear();
    AL_INTERNAL_PRINTF("[BLE][TX] ack requestId=%u status=%s bytes=%u\n",
                  requestId,
                  status.c_str(),
                  static_cast<unsigned>(out.length()));
    bleNotifyEventJson(out);
  }

  static void sendError(uint16_t requestId, const String& message, const String& detail) {
    StaticJsonDocument<320> doc;
    doc["type"] = "error";
    doc["requestId"] = requestId;
    doc["message"] = message;
    if (!detail.isEmpty()) {
      doc["detail"] = detail;
    }

    String out;
    serializeJson(doc, out);
    doc.clear();
    bleNotifyEventJson(out);
  }

  static void sendEventJson(const String& json) {
    bleNotifyEventJson(json);
  }
};

inline void ArmorLinkBleRuntime::begin(const char* deviceName, const char* localTarget) {
  setDefaultTarget(localTarget);
  begin(deviceName, new ArmorLinkAutoBleCommandHandler(localTarget));
}
#endif // ARMORLINK_HAS_BLE
#endif
