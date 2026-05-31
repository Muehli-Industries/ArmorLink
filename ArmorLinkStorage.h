#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "ArmorLinkModule.h"

struct ArmorLinkPairingInfo {
  bool paired = false;
  char gatewayName[16] = { 0 };
  char gatewayMac[18] = { 0 };
  uint32_t recoveryPin = 0;
};

struct ArmorLinkStoredPairedModule {
  char name[16] = { 0 };
  char type[16] = { 0 };
  char mac[18] = { 0 };
};

class ArmorLinkStorage {
public:
  static constexpr size_t MAX_PAIRED_MODULES = 16;

  ArmorLinkStorage() = default;

  bool begin(const String& nvsNamespace) {
    _namespace = sanitizeNamespace(nvsNamespace);
    return !_namespace.isEmpty();
  }

  bool load(ArmorLinkModule& module) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), true)) {
      return false;
    }

    for (auto& field : module.config().items()) {
      const String logicalStorageKey = field.nvsKey.isEmpty()
        ? field.key
        : field.nvsKey;

      const String storageKey = sanitizeKey(logicalStorageKey);

      if (storageKey.isEmpty()) {
        continue;
      }

      switch (field.kind) {
        case ArmorLinkFieldKind::Int:
          if (field.intBinding.ptr) {
            const int loadedValue = prefs.getInt(storageKey.c_str(), *field.intBinding.ptr);
            int finalValue = loadedValue;

            if (field.hasIntRange) {
              finalValue = clampIntValue(field, loadedValue);

              if (finalValue != loadedValue) {
                Serial.printf("[CONFIG] Clamped persisted value for %s from %d to %d\n",
                              field.key.c_str(),
                              loadedValue,
                              finalValue);
              }
            }

            *field.intBinding.ptr = finalValue;
          }
          break;

        case ArmorLinkFieldKind::Bool:
          if (field.boolBinding.ptr) {
            *field.boolBinding.ptr = prefs.getBool(storageKey.c_str(), *field.boolBinding.ptr);
          }
          break;

        case ArmorLinkFieldKind::Readonly:
        default:
          break;
      }
    }

    prefs.end();
    return true;
  }

  bool saveField(const ArmorLinkConfigFieldDef& field) {
    const String logicalStorageKey = field.nvsKey.isEmpty()
      ? field.key
      : field.nvsKey;

    const String storageKey = sanitizeKey(logicalStorageKey);

    if (_namespace.isEmpty() || storageKey.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), false)) {
      return false;
    }

    bool ok = false;

    switch (field.kind) {
      case ArmorLinkFieldKind::Int:
        if (field.intBinding.ptr) {
          int value = *field.intBinding.ptr;

          if (field.hasIntRange) {
            value = clampIntValue(field, value);
          }

          ok = prefs.putInt(storageKey.c_str(), value) > 0;
        }
        break;

      case ArmorLinkFieldKind::Bool:
        if (field.boolBinding.ptr) {
          ok = prefs.putBool(storageKey.c_str(), *field.boolBinding.ptr);
        }
        break;

      case ArmorLinkFieldKind::Readonly:
      default:
        ok = false;
        break;
    }

    prefs.end();
    return ok;
  }
static String sanitizeKey(const String& input) {
  String key;
  key.reserve(15);

  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < input.length(); ++i) {
    hash ^= static_cast<uint8_t>(input[i]);
    hash *= 16777619UL;
  }

  if (input.length() <= 15) {
    for (size_t i = 0; i < input.length(); ++i) {
      char c = input[i];
      key += (isalnum(c) || c == '_' || c == '-') ? c : '_';
    }
    return key;
  }

  for (size_t i = 0; i < input.length() && key.length() < 10; ++i) {
    char c = input[i];
    key += (isalnum(c) || c == '_' || c == '-') ? c : '_';
  }

  char suffix[6];
  snprintf(suffix, sizeof(suffix), "_%04X", static_cast<unsigned int>(hash & 0xFFFF));
  key += suffix;
  return key;
}
  bool saveAll(ArmorLinkModule& module) {
    bool allOk = true;

    for (const auto& field : module.config().items()) {
      const String logicalStorageKey = field.nvsKey.isEmpty()
        ? field.key
        : field.nvsKey;

      const String storageKey = sanitizeKey(logicalStorageKey);

      if (storageKey.isEmpty()) {
        continue;
      }

      if (!saveField(field)) {
        allOk = false;
      }
    }

    return allOk;
  }

  bool loadPairingInfo(ArmorLinkPairingInfo& outInfo) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), true)) {
      return false;
    }

    outInfo.paired = prefs.getBool("pair_paired", false);
    getStringIntoBuffer(prefs, "pair_gw_name", outInfo.gatewayName, sizeof(outInfo.gatewayName));
    getStringIntoBuffer(prefs, "pair_gw_mac", outInfo.gatewayMac, sizeof(outInfo.gatewayMac));
    outInfo.recoveryPin = prefs.getUInt("pair_rpin", 0);

    prefs.end();
    return true;
  }

  bool savePairingInfo(const ArmorLinkPairingInfo& info) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), false)) {
      return false;
    }

    prefs.putBool("pair_paired", info.paired);
    prefs.putString("pair_gw_name", info.gatewayName);
    prefs.putString("pair_gw_mac", info.gatewayMac);
    prefs.putUInt("pair_rpin", info.recoveryPin);

    prefs.end();
    return true;
  }

  bool clearPairingInfo() {
    ArmorLinkPairingInfo empty{};
    return savePairingInfo(empty);
  }

  size_t loadPairedModules(ArmorLinkStoredPairedModule outModules[MAX_PAIRED_MODULES]) {
    if (_namespace.isEmpty()) {
      return 0;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), true)) {
      return 0;
    }

    const size_t count = min(
      static_cast<size_t>(prefs.getUInt("paired_count", 0)),
      MAX_PAIRED_MODULES
    );

    for (size_t i = 0; i < count; ++i) {
      String keyName = String("pm_name_") + i;
      String keyType = String("pm_type_") + i;
      String keyMac  = String("pm_mac_") + i;

      getStringIntoBuffer(prefs, keyName.c_str(), outModules[i].name, sizeof(outModules[i].name));
      getStringIntoBuffer(prefs, keyType.c_str(), outModules[i].type, sizeof(outModules[i].type));
      getStringIntoBuffer(prefs, keyMac.c_str(), outModules[i].mac, sizeof(outModules[i].mac));
    }

    prefs.end();
    return count;
  }

  bool savePairedModules(const ArmorLinkStoredPairedModule modules[MAX_PAIRED_MODULES], size_t count) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), false)) {
      return false;
    }

    const size_t clampedCount = min(count, MAX_PAIRED_MODULES);
    prefs.putUInt("paired_count", clampedCount);

    for (size_t i = 0; i < MAX_PAIRED_MODULES; ++i) {
      String keyName = String("pm_name_") + i;
      String keyType = String("pm_type_") + i;
      String keyMac  = String("pm_mac_") + i;

      if (i < clampedCount) {
        prefs.putString(keyName.c_str(), modules[i].name);
        prefs.putString(keyType.c_str(), modules[i].type);
        prefs.putString(keyMac.c_str(), modules[i].mac);
      } else {
        prefs.remove(keyName.c_str());
        prefs.remove(keyType.c_str());
        prefs.remove(keyMac.c_str());
      }
    }

    prefs.end();
    return true;
  }

private:
  String _namespace;

  static String sanitizeNamespace(const String& input) {
    String ns;
    ns.reserve(15);

    for (size_t i = 0; i < input.length() && ns.length() < 15; ++i) {
      char c = input[i];
      if (isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
        ns += (char)tolower(static_cast<unsigned char>(c));
      }
    }

    if (ns.isEmpty()) {
      ns = "armorlink";
    }

    return ns;
  }

  static int clampIntValue(const ArmorLinkConfigFieldDef& field, int value) {
    if (!field.hasIntRange) {
      return value;
    }

    if (value < field.minInt) {
      return field.minInt;
    }

    if (value > field.maxInt) {
      return field.maxInt;
    }

    return value;
  }

  static void getStringIntoBuffer(Preferences& prefs, const char* key, char* buffer, size_t bufferSize) {
    if (!buffer || bufferSize == 0) {
      return;
    }

    memset(buffer, 0, bufferSize);
    String value = prefs.getString(key, "");
    strncpy(buffer, value.c_str(), bufferSize - 1);
  }
};