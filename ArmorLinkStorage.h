#pragma once

#include "ArmorLinkDebug.h"
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
  char moduleVersion[16] = "1.0";
  char armorLinkVersion[16] = { 0 };
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

        case ArmorLinkFieldKind::String:
          if (field.stringBinding.ptr) {
            *field.stringBinding.ptr = prefs.getString(storageKey.c_str(), *field.stringBinding.ptr);
          }
          break;

        case ArmorLinkFieldKind::Float:
          if (field.floatBinding.ptr) {
            const float loadedValue = prefs.getFloat(storageKey.c_str(), *field.floatBinding.ptr);
            float finalValue = loadedValue;

            if (field.hasFloatRange) {
              finalValue = clampFloatValue(field, loadedValue);

              if (finalValue != loadedValue) {
                Serial.printf("[CONFIG] Clamped persisted value for %s from %.3f to %.3f\n",
                              field.key.c_str(),
                              loadedValue,
                              finalValue);
              }
            }

            *field.floatBinding.ptr = finalValue;
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

      case ArmorLinkFieldKind::String:
        if (field.stringBinding.ptr) {
          prefs.putString(storageKey.c_str(), *field.stringBinding.ptr);
          ok = true;
        }
        break;

      case ArmorLinkFieldKind::Float:
        if (field.floatBinding.ptr) {
          float value = *field.floatBinding.ptr;

          if (field.hasFloatRange) {
            value = clampFloatValue(field, value);
          }

          ok = prefs.putFloat(storageKey.c_str(), value) > 0;
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

  bool loadProfileName(String& outName) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), true)) {
      return false;
    }

    outName = prefs.getString("profile_name", "");
    prefs.end();
    return true;
  }

  bool saveProfileName(const String& name) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), false)) {
      return false;
    }

    prefs.putString("profile_name", name);
    prefs.end();
    return true;
  }

  bool loadActionDefinitions(ArmorLinkModule& module) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), true)) {
      return false;
    }

    const String actionList = prefs.getString("act_list", "");
    int start = 0;
    while (start < static_cast<int>(actionList.length())) {
      const int end = actionList.indexOf('\n', start);
      const String line =
          end < 0
              ? actionList.substring(start)
              : actionList.substring(start, end);

      const int firstSep = line.indexOf('|');
      const int secondSep = firstSep < 0 ? -1 : line.indexOf('|', firstSep + 1);
      const int thirdSep = secondSep < 0 ? -1 : line.indexOf('|', secondSep + 1);

      if (firstSep > 0 && secondSep > firstSep && thirdSep > secondSep) {
        module.actionBuilder().upsertCustomAction(
            line.substring(0, firstSep),
            line.substring(firstSep + 1, secondSep),
            line.substring(secondSep + 1, thirdSep),
            line.substring(thirdSep + 1));
      }

      if (end < 0) {
        break;
      }
      start = end + 1;
    }

    const String componentList = prefs.getString("act_comp", "");
    start = 0;
    while (start < static_cast<int>(componentList.length())) {
      const int end = componentList.indexOf('\n', start);
      const String line =
          end < 0
              ? componentList.substring(start)
              : componentList.substring(start, end);

      const int firstSep = line.indexOf('|');
      const int secondSep = firstSep < 0 ? -1 : line.indexOf('|', firstSep + 1);
      const int thirdSep = secondSep < 0 ? -1 : line.indexOf('|', secondSep + 1);
      const int fourthSep = thirdSep < 0 ? -1 : line.indexOf('|', thirdSep + 1);

      if (firstSep > 0 && secondSep > firstSep) {
        module.actionBuilder().upsertCustomComponent(
            line.substring(0, firstSep),
            actionComponentKindFromString(line.substring(firstSep + 1, secondSep)),
            thirdSep > secondSep
                ? line.substring(secondSep + 1, thirdSep)
                : line.substring(secondSep + 1),
            thirdSep > secondSep
                ? (fourthSep > thirdSep
                    ? line.substring(thirdSep + 1, fourthSep)
                    : line.substring(thirdSep + 1))
                : String("Actions"),
            fourthSep > thirdSep &&
                line.substring(fourthSep + 1).equals("1"));
      }

      if (end < 0) {
        break;
      }
      start = end + 1;
    }

    const String remoteCommandList = prefs.getString("act_rcmd", "");
    start = 0;
    while (start < static_cast<int>(remoteCommandList.length())) {
      const int end = remoteCommandList.indexOf('\n', start);
      const String line =
          end < 0
              ? remoteCommandList.substring(start)
              : remoteCommandList.substring(start, end);

      const int firstSep = line.indexOf('|');
      const int secondSep = firstSep < 0 ? -1 : line.indexOf('|', firstSep + 1);

      if (firstSep > 0 && secondSep > firstSep) {
        module.actionBuilder().upsertRemoteCommand(
            line.substring(0, firstSep),
            line.substring(firstSep + 1, secondSep),
            line.substring(secondSep + 1));
      }

      if (end < 0) {
        break;
      }
      start = end + 1;
    }

    for (const auto& action : module.actions().items()) {
      loadActionDefinitionForAction(prefs, module, action);
    }

    for (const auto& action : module.actionBuilder().customActions()) {
      loadActionDefinitionForAction(prefs, module, action);
    }

    prefs.end();
    return true;
  }

  bool saveActionDefinition(
      const ArmorLinkActionDefinition& definition,
      bool saveAsCustomAction = false,
      const ArmorLinkActionBuilderRuntime* runtime = nullptr) {
    if (_namespace.isEmpty() || definition.ops.size() > 63) {
      return false;
    }

    const String storageKey =
        sanitizeKey(String("act_") + definition.entity + "_" + definition.command);

    if (storageKey.isEmpty()) {
      return false;
    }

    std::vector<uint8_t> data;
    data.reserve(1 + definition.ops.size() * 5);
    data.push_back(static_cast<uint8_t>(definition.ops.size()));

    for (const auto& op : definition.ops) {
      data.push_back(static_cast<uint8_t>(op.code));
      data.push_back(op.componentIndex);
      data.push_back(static_cast<uint8_t>(op.argument & 0xFF));
      data.push_back(static_cast<uint8_t>((op.argument >> 8) & 0xFF));
      data.push_back(op.value ? 1 : 0);
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), false)) {
      return false;
    }

    const size_t written =
        prefs.putBytes(storageKey.c_str(), data.data(), data.size());

    if (saveAsCustomAction) {
      saveCustomActionList(prefs, definition);
    }
    if (runtime != nullptr) {
      saveCustomComponentList(prefs, *runtime);
      saveRemoteCommandList(prefs, *runtime);
    }
    prefs.end();

    return written == data.size();
  }

  bool saveActionComponents(
      const ArmorLinkActionBuilderRuntime& runtime) {
    if (_namespace.isEmpty()) {
      return false;
    }

    Preferences prefs;
    if (!prefs.begin(_namespace.c_str(), false)) {
      return false;
    }

    saveCustomComponentList(prefs, runtime);
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
      String keyModuleVersion = String("pm_modver_") + i;
      String keyArmorLinkVersion = String("pm_alver_") + i;

      getStringIntoBuffer(prefs, keyName.c_str(), outModules[i].name, sizeof(outModules[i].name));
      getStringIntoBuffer(prefs, keyType.c_str(), outModules[i].type, sizeof(outModules[i].type));
      getStringIntoBuffer(prefs, keyMac.c_str(), outModules[i].mac, sizeof(outModules[i].mac));
      getStringIntoBuffer(prefs, keyModuleVersion.c_str(), outModules[i].moduleVersion, sizeof(outModules[i].moduleVersion), "1.0");
      getStringIntoBuffer(prefs, keyArmorLinkVersion.c_str(), outModules[i].armorLinkVersion, sizeof(outModules[i].armorLinkVersion));
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
      String keyModuleVersion = String("pm_modver_") + i;
      String keyArmorLinkVersion = String("pm_alver_") + i;

      if (i < clampedCount) {
        prefs.putString(keyName.c_str(), modules[i].name);
        prefs.putString(keyType.c_str(), modules[i].type);
        prefs.putString(keyMac.c_str(), modules[i].mac);
        prefs.putString(keyModuleVersion.c_str(), strlen(modules[i].moduleVersion) > 0 ? modules[i].moduleVersion : "1.0");
        prefs.putString(keyArmorLinkVersion.c_str(), modules[i].armorLinkVersion);
      } else {
        prefs.remove(keyName.c_str());
        prefs.remove(keyType.c_str());
        prefs.remove(keyMac.c_str());
        prefs.remove(keyModuleVersion.c_str());
        prefs.remove(keyArmorLinkVersion.c_str());
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

  static float clampFloatValue(const ArmorLinkConfigFieldDef& field, float value) {
    if (!field.hasFloatRange) {
      return value;
    }

    if (value < field.minFloat) {
      return field.minFloat;
    }

    if (value > field.maxFloat) {
      return field.maxFloat;
    }

    return value;
  }

  static void getStringIntoBuffer(Preferences& prefs,
                                  const char* key,
                                  char* buffer,
                                  size_t bufferSize,
                                  const char* fallback = "") {
    if (!buffer || bufferSize == 0) {
      return;
    }

    memset(buffer, 0, bufferSize);
    String value = prefs.getString(key, fallback);
    strncpy(buffer, value.c_str(), bufferSize - 1);
  }

  static String cleanActionListValue(const String& value) {
    String cleaned = value;
    cleaned.replace("|", " ");
    cleaned.replace("\n", " ");
    cleaned.replace("\r", " ");
    return cleaned;
  }

  static void saveCustomActionList(
      Preferences& prefs,
      const ArmorLinkActionDefinition& definition) {
    String actionList = prefs.getString("act_list", "");
    const String prefix = definition.entity + "|" + definition.command + "|";
    String nextList;

    int start = 0;
    while (start < static_cast<int>(actionList.length())) {
      const int end = actionList.indexOf('\n', start);
      const String line =
          end < 0
              ? actionList.substring(start)
              : actionList.substring(start, end);

      if (!line.startsWith(prefix) && !line.isEmpty()) {
        nextList += line;
        nextList += "\n";
      }

      if (end < 0) {
        break;
      }
      start = end + 1;
    }

    nextList += prefix;
    nextList += cleanActionListValue(definition.label);
    nextList += "|";
    nextList += cleanActionListValue(definition.section.isEmpty() ? String("Actions") : definition.section);
    nextList += "\n";
    prefs.putString("act_list", nextList);
  }

  static const char* actionComponentKindToString(
      ArmorLinkActionComponentKind kind) {
    switch (kind) {
      case ArmorLinkActionComponentKind::Servo: return "servo";
      case ArmorLinkActionComponentKind::Led: return "led";
      case ArmorLinkActionComponentKind::Gpio: return "gpio";
      case ArmorLinkActionComponentKind::State: return "state";
      default: return "gpio";
    }
  }

  static ArmorLinkActionComponentKind actionComponentKindFromString(
      const String& kind) {
    if (kind.equalsIgnoreCase("servo")) return ArmorLinkActionComponentKind::Servo;
    if (kind.equalsIgnoreCase("led")) return ArmorLinkActionComponentKind::Led;
    if (kind.equalsIgnoreCase("gpio")) return ArmorLinkActionComponentKind::Gpio;
    if (kind.equalsIgnoreCase("state")) return ArmorLinkActionComponentKind::State;
    if (kind.equalsIgnoreCase("waitTarget")) return ArmorLinkActionComponentKind::State;
    if (kind.equalsIgnoreCase("preset")) return ArmorLinkActionComponentKind::State;
    if (kind.equalsIgnoreCase("mode")) return ArmorLinkActionComponentKind::State;
    return ArmorLinkActionComponentKind::Gpio;
  }

  static void saveCustomComponentList(
      Preferences& prefs,
      const ArmorLinkActionBuilderRuntime& runtime) {
    String componentList;

    for (const auto& component : runtime.components()) {
      if (!component.custom) {
        continue;
      }

      componentList += cleanActionListValue(component.id);
      componentList += "|";
      componentList += actionComponentKindToString(component.kind);
      componentList += "|";
      componentList += cleanActionListValue(component.label);
      componentList += "|";
      componentList += cleanActionListValue(component.section.isEmpty() ? String("Actions") : component.section);
      componentList += "|";
      componentList += component.defaultValue ? "1" : "0";
      componentList += "\n";
    }

    prefs.putString("act_comp", componentList);
  }

  static void saveRemoteCommandList(
      Preferences& prefs,
      const ArmorLinkActionBuilderRuntime& runtime) {
    String remoteCommandList;

    for (const auto& remoteCommand : runtime.remoteCommands()) {
      if (remoteCommand.target.isEmpty() ||
          remoteCommand.entity.isEmpty() ||
          remoteCommand.command.isEmpty()) {
        continue;
      }

      remoteCommandList += cleanActionListValue(remoteCommand.target);
      remoteCommandList += "|";
      remoteCommandList += cleanActionListValue(remoteCommand.entity);
      remoteCommandList += "|";
      remoteCommandList += cleanActionListValue(remoteCommand.command);
      remoteCommandList += "\n";
    }

    prefs.putString("act_rcmd", remoteCommandList);
  }

  static void loadActionDefinitionForAction(
      Preferences& prefs,
      ArmorLinkModule& module,
      const ArmorLinkActionDef& action) {
    const String storageKey =
        sanitizeKey(String("act_") + action.entity + "_" + action.command);

    const size_t length =
        prefs.getBytesLength(storageKey.c_str());

    if (length < 1) {
      return;
    }

    std::vector<uint8_t> data(length);
    if (prefs.getBytes(storageKey.c_str(), data.data(), data.size()) != length) {
      return;
    }

    const uint8_t count = data[0];
    const size_t opSize =
        length >= 1 + static_cast<size_t>(count) * 5 ? 5 : 4;
    if (length < 1 + static_cast<size_t>(count) * opSize) {
      return;
    }

    ArmorLinkActionDefinition definition;
    definition.entity = action.entity;
    definition.command = action.command;
    definition.label = action.label;
    definition.section = action.section;
    definition.ops.reserve(count);

    for (uint8_t i = 0; i < count; ++i) {
      const size_t offset = 1 + static_cast<size_t>(i) * opSize;
      ArmorLinkActionOp op;
      op.code = static_cast<ArmorLinkActionOpCode>(data[offset]);
      if (data[offset] == 12) {
        op.code = ArmorLinkActionOpCode::StateSet;
      }
      op.componentIndex = data[offset + 1];
      op.argument =
          static_cast<uint16_t>(data[offset + 2]) |
          (static_cast<uint16_t>(data[offset + 3]) << 8);
      if (opSize == 5) {
        op.value = data[offset + 4] != 0;
      }
      definition.ops.push_back(op);
    }

    module.actionBuilder().setDefinition(definition);
  }
};
