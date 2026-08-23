#pragma once

#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "ArmorLinkModule.h"

class ArmorLinkDescriptor {
public:
  enum class Profile {
    Ble,
    WebSerial
  };

  static constexpr size_t WebSerialDescriptorJsonCapacity = 65536;
  static constexpr size_t BleDescriptorJsonCapacity = 65536;
  static constexpr size_t BleDescriptorBufferSize = 12288;
  static constexpr size_t RichDescriptorJsonCapacity = WebSerialDescriptorJsonCapacity;
  static constexpr size_t CompactDescriptorJsonCapacity = BleDescriptorJsonCapacity;
  static constexpr size_t CompactDescriptorStringReserve = 32768;

  static String build(const ArmorLinkModule& module) {
    return buildWebSerial(module);
  }

  static String buildBle(const ArmorLinkModule& module) {
    return buildWithProfile(module, Profile::Ble, BleDescriptorJsonCapacity);
  }

  static String buildWebSerial(const ArmorLinkModule& module) {
    String json = buildWithProfile(module, Profile::WebSerial, WebSerialDescriptorJsonCapacity);
    if (!json.isEmpty()) {
      return json;
    }

    Serial.println("[DESCRIPTOR] WebSerial descriptor failed, retrying BLE descriptor");
    return buildBle(module);
  }

  static bool write(const ArmorLinkModule& module, Print& out, size_t* length = nullptr) {
    return writeWebSerial(module, out, length);
  }

  static bool writeWebSerial(const ArmorLinkModule& module, Print& out, size_t* length = nullptr) {
    ArmorLinkDescriptorCountingPrint counter;
    if (!writeDocumentStreaming(module, counter, Profile::WebSerial)) {
      return false;
    }

    const size_t measuredLength = counter.length();
    if (length != nullptr) {
      *length = measuredLength;
    }

    ArmorLinkDescriptorCountingPrint countedOut(&out);
    if (!writeDocumentStreaming(module, countedOut, Profile::WebSerial)) {
      return false;
    }

    const size_t writtenLength = countedOut.length();
    if (writtenLength != measuredLength) {
      Serial.printf(
          "[DESCRIPTOR] JSON stream incomplete (%u of %u bytes)\n",
          static_cast<unsigned>(writtenLength),
          static_cast<unsigned>(measuredLength));
      return false;
    }

    return true;
  }

  static bool measure(const ArmorLinkModule& module, size_t& length) {
    return measureWebSerial(module, length);
  }

  static bool measureWebSerial(const ArmorLinkModule& module, size_t& length) {
    ArmorLinkDescriptorCountingPrint counter;

    if (!writeDocumentStreaming(module, counter, Profile::WebSerial)) {
      length = 0;
      return false;
    }

    length = counter.length();
    return true;
  }

  static bool populateJson(const ArmorLinkModule& module,
                           DynamicJsonDocument& doc,
                           bool richMetadata) {
    return populateDocument(
      module,
      doc,
      richMetadata ? Profile::WebSerial : Profile::Ble);
  }

  static bool populateBleJson(const ArmorLinkModule& module,
                              DynamicJsonDocument& doc) {
    return populateDocument(module, doc, Profile::Ble);
  }

  static bool populateWebSerialJson(const ArmorLinkModule& module,
                                    DynamicJsonDocument& doc) {
    return populateDocument(module, doc, Profile::WebSerial);
  }

  static bool buildCompactInto(const ArmorLinkModule& module,
                               char* buffer,
                               size_t bufferSize,
                               size_t& length) {
    return buildBleInto(module, buffer, bufferSize, length);
  }

  static bool buildBleInto(const ArmorLinkModule& module,
                           char* buffer,
                           size_t bufferSize,
                           size_t& length) {
    length = 0;
    if (buffer == nullptr || bufferSize == 0) {
      return false;
    }

    buffer[0] = '\0';

    DynamicJsonDocument doc(BleDescriptorJsonCapacity);
    if (!populateDocument(module, doc, Profile::Ble)) {
      return false;
    }

    const size_t measuredLength = measureJson(doc);
    if (measuredLength + 1 > bufferSize) {
      Serial.printf(
          "[DESCRIPTOR] BLE JSON buffer too small (%u of %u bytes)\n",
          static_cast<unsigned>(measuredLength),
          static_cast<unsigned>(bufferSize));
      return false;
    }

    length = serializeJson(doc, buffer, bufferSize);
    if (length != measuredLength) {
      Serial.printf(
          "[DESCRIPTOR] BLE JSON buffer serialization incomplete (%u of %u bytes)\n",
          static_cast<unsigned>(length),
          static_cast<unsigned>(measuredLength));
      length = 0;
      buffer[0] = '\0';
      return false;
    }

    return true;
  }

private:
  class ArmorLinkDescriptorCountingPrint : public Print {
  public:
    explicit ArmorLinkDescriptorCountingPrint(Print* out = nullptr)
        : _out(out) {}

    size_t write(uint8_t value) override {
      if (_out != nullptr) {
        _out->write(value);
      }

      _length++;
      return 1;
    }

    size_t write(const uint8_t* buffer, size_t size) override {
      if (_out != nullptr) {
        _out->write(buffer, size);
      }

      _length += size;
      return size;
    }

    size_t length() const {
      return _length;
    }

  private:
    Print* _out;
    size_t _length = 0;
  };

  static bool writeDocumentStreaming(const ArmorLinkModule& module,
                                     Print& out,
                                     Profile profile) {
    const bool isWebSerial = profile == Profile::WebSerial;

    out.print('{');
    bool first = true;

    writeStringProperty(out, first, "module", module.name());
    writeStringProperty(out, first, "name", module.name());
    writeStringProperty(out, first, "moduleVersion", module.version());
    writeStringProperty(out, first, "armorLinkVersion", ARMORLINK_VERSION);

    if (isWebSerial && !module.profileName().isEmpty()) {
      writeStringProperty(out, first, "profileName", module.profileName());
      writeStringProperty(out, first, "activeProfileName", module.profileName());
      writeStringProperty(
          out,
          first,
          "profileNameSource",
          module.profileNameImported() ? "imported" : "firmware");
    }

    if (isWebSerial && !module.defaultProfileName().isEmpty()) {
      writeStringProperty(out, first, "defaultProfileName", module.defaultProfileName());
    }

    writeBoolProperty(out, first, "supportsPartialConfigGet", false);
    writeBoolProperty(out, first, "supportsConfigSet", true);
    writeStringProperty(out, first, "moduleType", moduleTypeToString(module.type()));

    if (isWebSerial && !module.profileTarget().isEmpty()) {
      writeStringProperty(out, first, "profileTarget", module.profileTarget());
    }

    writePropertyName(out, first, "sections");
    out.print('[');
    writeSectionsStreaming(module, out, profile);
    out.print(']');

    out.print('}');
    return true;
  }

  static void writeSectionsStreaming(const ArmorLinkModule& module,
                                     Print& out,
                                     Profile profile) {
    const bool isWebSerial = profile == Profile::WebSerial;
    std::vector<String> orderedSections;
    bool hasArmorLinkSection = false;

    for (const auto& field : module.config().items()) {
      if (field.section.equalsIgnoreCase("ArmorLink")) {
        hasArmorLinkSection = true;
        continue;
      }

      bool found = false;
      for (const auto& existing : orderedSections) {
        if (existing == field.section) {
          found = true;
          break;
        }
      }

      if (!found) {
        orderedSections.push_back(field.section);
      }
    }

    for (const auto& action : module.actions().items()) {
      const String actionSection =
          action.section.isEmpty() ? String("General") : action.section;

      bool found = false;
      for (const auto& existing : orderedSections) {
        if (normalizeId(existing) == normalizeId(actionSection)) {
          found = true;
          break;
        }
      }

      if (!found) {
        orderedSections.push_back(actionSection);
      }
    }

    if (isWebSerial && hasArmorLinkSection) {
      orderedSections.insert(orderedSections.begin(), "ArmorLink");
    }

    bool firstSection = true;

    for (const auto& sectionName : orderedSections) {
      if (!firstSection) {
        out.print(',');
      }
      firstSection = false;

      writeSectionStreaming(module, out, profile, sectionName);
    }
  }

  static void writeSectionStreaming(const ArmorLinkModule& module,
                                    Print& out,
                                    Profile profile,
                                    const String& sectionName) {
    const bool isWebSerial = profile == Profile::WebSerial;

    out.print('{');
    bool first = true;

    if (isWebSerial) {
      writeStringProperty(out, first, "id", normalizeId(sectionName));
    }

    writeStringProperty(out, first, "title", sectionName);

    writePropertyName(out, first, "fields");
    out.print('[');
    writeFieldsStreaming(module, out, profile, sectionName);
    out.print(']');

    writePropertyName(out, first, "actions");
    out.print('[');
    writeActionsStreaming(module, out, profile, sectionName);
    out.print(']');

    out.print('}');
  }

  static void writeFieldsStreaming(const ArmorLinkModule& module,
                                   Print& out,
                                   Profile profile,
                                   const String& sectionName) {
    const bool isWebSerial = profile == Profile::WebSerial;
    bool firstField = true;

    for (const auto& field : module.config().items()) {
      if (field.section != sectionName) {
        continue;
      }

      if (!isWebSerial && field.kind == ArmorLinkFieldKind::Readonly) {
        continue;
      }

      if (!firstField) {
        out.print(',');
      }
      firstField = false;

      writeFieldStreaming(field, out, profile);
    }
  }

  static void writeFieldStreaming(const ArmorLinkConfigFieldDef& field,
                                  Print& out,
                                  Profile profile) {
    const bool isWebSerial = profile == Profile::WebSerial;

    out.print('{');
    bool first = true;

    writeStringProperty(out, first, "key", field.key);
    writeStringProperty(out, first, "label", field.label);
    writeStringProperty(out, first, "kind", fieldKindToString(field.kind));

    if (field.editable) {
      writeBoolProperty(out, first, "editable", true);
    }

    if (isWebSerial && field.advanced) {
      writeBoolProperty(out, first, "advanced", true);
    }

    if (field.rebootRequired) {
      writeBoolProperty(out, first, "rebootRequired", true);
    }

    if (isWebSerial && !field.description.isEmpty()) {
      writeStringProperty(out, first, "description", field.description);
    }

    if (isWebSerial &&
        field.tooltip != nullptr &&
        field.tooltip[0] != '\0') {
      writeStringProperty(out, first, "tooltip", field.tooltip);
    }

    if (isWebSerial &&
        field.visibleWhen.enabled &&
        !field.visibleWhen.key.isEmpty()) {
      writePropertyName(out, first, "visibleWhen");
      out.print('{');
      bool visibleFirst = true;
      writeStringProperty(out, visibleFirst, "key", field.visibleWhen.key);
      writeStringProperty(out, visibleFirst, "equals", field.visibleWhen.value);
      out.print('}');
    }

    if (isWebSerial && !field.unit.isEmpty()) {
      writeStringProperty(out, first, "unit", field.unit);
    }

    if (isWebSerial && !field.semantic.isEmpty()) {
      writeStringProperty(out, first, "semantic", field.semantic);
    }

    if (isWebSerial && !field.semanticGroup.isEmpty()) {
      writeStringProperty(out, first, "semanticGroup", field.semanticGroup);
    }

    if (field.editable) {
      const String effectiveEntity =
          field.entity.isEmpty() ? String("config") : field.entity;
      const String effectiveCommand =
          field.command.isEmpty() ? field.key : field.command;

      if (isWebSerial || !effectiveEntity.equalsIgnoreCase("config")) {
        writeStringProperty(out, first, "entity", effectiveEntity);
      }

      if (isWebSerial || !effectiveCommand.equalsIgnoreCase(field.key)) {
        writeStringProperty(out, first, "command", effectiveCommand);
      }
    }

    writeFieldValueAndRange(field, out, first);

    out.print('}');
  }

  static void writeFieldValueAndRange(const ArmorLinkConfigFieldDef& field,
                                      Print& out,
                                      bool& first) {
    switch (field.kind) {
      case ArmorLinkFieldKind::Int:
        writeIntProperty(
            out,
            first,
            "value",
            field.intBinding.ptr ? *field.intBinding.ptr : 0);

        if (field.hasIntRange) {
          writeIntProperty(out, first, "min", field.minInt);
          writeIntProperty(out, first, "max", field.maxInt);
        }

        writeIntProperty(out, first, "step", field.stepInt);
        break;

      case ArmorLinkFieldKind::Bool:
        writeBoolProperty(
            out,
            first,
            "value",
            field.boolBinding.ptr ? *field.boolBinding.ptr : false);
        break;

      case ArmorLinkFieldKind::String:
        writeStringProperty(
            out,
            first,
            "value",
            field.stringBinding.ptr ? *field.stringBinding.ptr : String(""));
        break;

      case ArmorLinkFieldKind::Float:
        writeFloatProperty(
            out,
            first,
            "value",
            field.floatBinding.ptr ? *field.floatBinding.ptr : 0.0f);

        if (field.hasFloatRange) {
          writeFloatProperty(out, first, "min", field.minFloat);
          writeFloatProperty(out, first, "max", field.maxFloat);
        }

        writeFloatProperty(out, first, "step", field.stepFloat);
        break;

      case ArmorLinkFieldKind::Readonly:
      default:
        writeStringProperty(out, first, "value", field.readonlyBinding.value);
        break;
    }
  }

  static void writeActionsStreaming(const ArmorLinkModule& module,
                                    Print& out,
                                    Profile profile,
                                    const String& sectionName) {
    bool firstAction = true;

    for (const auto& action : module.actions().items()) {
      const String actionSection =
          action.section.isEmpty() ? String("General") : action.section;

      if (normalizeId(actionSection) != normalizeId(sectionName)) {
        continue;
      }

      if (!firstAction) {
        out.print(',');
      }
      firstAction = false;

      out.print('{');
      appendActionStreaming(action, out, profile);
      out.print('}');
    }
  }

  static void appendActionStreaming(const ArmorLinkActionDef& action,
                                    Print& out,
                                    Profile profile) {
    const bool isWebSerial = profile == Profile::WebSerial;
    bool first = true;

    writeStringProperty(out, first, "id", action.id);
    writeStringProperty(out, first, "label", action.label);
    writeStringProperty(out, first, "entity", action.entity);
    writeStringProperty(out, first, "command", action.command);

    if (!action.enabled) {
      writeBoolProperty(out, first, "enabled", false);
    }

    if (isWebSerial && action.style != ArmorLinkActionStyle::Secondary) {
      writeStringProperty(out, first, "style", actionStyleToString(action.style));
    }

    if (isWebSerial && action.advanced) {
      writeBoolProperty(out, first, "advanced", true);
    }

    if (isWebSerial && !action.description.isEmpty()) {
      writeStringProperty(out, first, "description", action.description);
    }

    if (!action.confirmText.isEmpty()) {
      writeStringProperty(out, first, "confirmText", action.confirmText);
    }
  }

  static void writePropertyName(Print& out,
                                bool& first,
                                const char* name) {
    if (!first) {
      out.print(',');
    }
    first = false;
    writeQuoted(out, name);
    out.print(':');
  }

  static void writeStringProperty(Print& out,
                                  bool& first,
                                  const char* name,
                                  const String& value) {
    writePropertyName(out, first, name);
    writeQuoted(out, value);
  }

  static void writeStringProperty(Print& out,
                                  bool& first,
                                  const char* name,
                                  const char* value) {
    writePropertyName(out, first, name);
    writeQuoted(out, value == nullptr ? "" : value);
  }

  static void writeBoolProperty(Print& out,
                                bool& first,
                                const char* name,
                                bool value) {
    writePropertyName(out, first, name);
    out.print(value ? "true" : "false");
  }

  static void writeIntProperty(Print& out,
                               bool& first,
                               const char* name,
                               int value) {
    writePropertyName(out, first, name);
    out.print(value);
  }

  static void writeFloatProperty(Print& out,
                                 bool& first,
                                 const char* name,
                                 float value) {
    writePropertyName(out, first, name);
    out.print(value, 3);
  }

  static void writeQuoted(Print& out, const String& value) {
    writeQuoted(out, value.c_str());
  }

  static void writeQuoted(Print& out, const char* value) {
    out.print('"');

    if (value != nullptr) {
      for (const char* cursor = value; *cursor != '\0'; ++cursor) {
        const char c = *cursor;

        switch (c) {
          case '"':
            out.print("\\\"");
            break;
          case '\\':
            out.print("\\\\");
            break;
          case '\b':
            out.print("\\b");
            break;
          case '\f':
            out.print("\\f");
            break;
          case '\n':
            out.print("\\n");
            break;
          case '\r':
            out.print("\\r");
            break;
          case '\t':
            out.print("\\t");
            break;
          default:
            if (static_cast<uint8_t>(c) < 0x20) {
              out.print("\\u00");
              const uint8_t valueByte = static_cast<uint8_t>(c);
              if (valueByte < 0x10) {
                out.print('0');
              }
              out.print(valueByte, HEX);
            } else {
              out.print(c);
            }
            break;
        }
      }
    }

    out.print('"');
  }

  static String buildWithProfile(const ArmorLinkModule& module,
                                 Profile profile,
                                 size_t jsonCapacity) {
    if (profile == Profile::Ble) {
      size_t measuredLength = 0;

      {
        DynamicJsonDocument doc(jsonCapacity);
        if (!populateDocument(module, doc, Profile::Ble)) {
          return "";
        }

        measuredLength = measureJson(doc);
      }

      if (measuredLength + 1 > CompactDescriptorStringReserve) {
        Serial.printf(
            "[DESCRIPTOR] BLE JSON reserve too small (%u of %u bytes)\n",
            static_cast<unsigned>(measuredLength),
            static_cast<unsigned>(CompactDescriptorStringReserve));
        return "";
      }

      String json;
      if (!json.reserve(measuredLength + 1)) {
        Serial.printf(
            "[DESCRIPTOR] BLE JSON reserve failed (%u bytes)\n",
            static_cast<unsigned>(measuredLength + 1));
        return "";
      }

      DynamicJsonDocument doc(jsonCapacity);
      if (!populateDocument(module, doc, Profile::Ble)) {
        return "";
      }

      serializeJson(doc, json);

      if (json.length() != measuredLength) {
        Serial.printf(
            "[DESCRIPTOR] JSON serialization incomplete (%u of %u bytes, profile=ble)\n",
            static_cast<unsigned>(json.length()),
            static_cast<unsigned>(measuredLength));
        return "";
      }

      return json;
    }

    size_t measuredLength = 0;

    {
      DynamicJsonDocument doc(jsonCapacity);

      if (!populateDocument(module, doc, profile)) {
        return "";
      }

      measuredLength = measureJson(doc);
    }

    String json;
    if (!json.reserve(measuredLength + 1)) {
      Serial.printf(
          "[DESCRIPTOR] JSON string allocation failed (%u bytes, profile=%s)\n",
          static_cast<unsigned>(measuredLength),
          profileName(profile));
      return "";
    }

    DynamicJsonDocument doc(jsonCapacity);
    if (!populateDocument(module, doc, profile)) {
      return "";
    }

    serializeJson(doc, json);

    if (json.length() != measuredLength) {
      Serial.printf(
          "[DESCRIPTOR] JSON serialization incomplete (%u of %u bytes, profile=%s)\n",
          static_cast<unsigned>(json.length()),
          static_cast<unsigned>(measuredLength),
          profileName(profile));
      return "";
    }

    return json;
  }

  static bool populateDocument(const ArmorLinkModule& module,
                               DynamicJsonDocument& doc,
                               Profile profile) {
    doc.clear();

    const bool isWebSerial = profile == Profile::WebSerial;

    doc["module"] = module.name();
    doc["name"] = module.name();
    doc["moduleVersion"] = module.version();
    doc["armorLinkVersion"] = ARMORLINK_VERSION;
    if (isWebSerial && !module.profileName().isEmpty()) {
      doc["profileName"] = module.profileName();
      doc["activeProfileName"] = module.profileName();
      doc["profileNameSource"] = module.profileNameImported()
          ? "imported"
          : "firmware";
    }
    if (isWebSerial && !module.defaultProfileName().isEmpty()) {
      doc["defaultProfileName"] = module.defaultProfileName();
    }
    doc["supportsPartialConfigGet"] = false;
    doc["supportsConfigSet"] = true;
    doc["moduleType"] = moduleTypeToString(module.type());
    if (isWebSerial && !module.profileTarget().isEmpty()) {
      doc["profileTarget"] = module.profileTarget();
    }
    JsonArray sections = doc.createNestedArray("sections");
    appendSections(module, sections, profile);

    if (doc.overflowed()) {
      Serial.printf("[DESCRIPTOR] JSON document overflowed (profile=%s)\n",
                    profileName(profile));
      return false;
    }

    return true;
  }

  static const char* profileName(Profile profile) {
    switch (profile) {
      case Profile::WebSerial: return "webserial";
      case Profile::Ble:
      default:
        return "ble";
    }
  }

  static const char* moduleTypeToString(ArmorLinkModuleType type) {
    switch (type) {
      case ArmorLinkModuleType::Chest: return "Chest";
      case ArmorLinkModuleType::Helmet: return "Helmet";
      case ArmorLinkModuleType::Back: return "Back";
      case ArmorLinkModuleType::Arm: return "Arm";
      case ArmorLinkModuleType::Hand: return "Hand";
      case ArmorLinkModuleType::Leg: return "Leg";
      case ArmorLinkModuleType::Prop: return "Prop";
      case ArmorLinkModuleType::Generic:
      default:
        return "Generic";
    }
  }

  static const char* fieldKindToString(ArmorLinkFieldKind kind) {
    switch (kind) {
      case ArmorLinkFieldKind::Int: return "int";
      case ArmorLinkFieldKind::Float: return "float";
      case ArmorLinkFieldKind::Bool: return "bool";
      case ArmorLinkFieldKind::String: return "string";
      case ArmorLinkFieldKind::Readonly:
      default:
        return "readonly";
    }
  }

  static const char* actionStyleToString(ArmorLinkActionStyle style) {
    switch (style) {
      case ArmorLinkActionStyle::Primary: return "primary";
      case ArmorLinkActionStyle::Danger: return "danger";
      case ArmorLinkActionStyle::Secondary:
      default:
        return "secondary";
    }
  }

  static void appendSections(const ArmorLinkModule& module, JsonArray sections, Profile profile) {
    const bool isWebSerial = profile == Profile::WebSerial;
    std::vector<String> orderedSections;
    bool hasArmorLinkSection = false;

    for (const auto& field : module.config().items()) {
      if (field.section.equalsIgnoreCase("ArmorLink")) {
        hasArmorLinkSection = true;
        continue;
      }

      bool found = false;
      for (const auto& existing : orderedSections) {
        if (existing == field.section) {
          found = true;
          break;
        }
      }

      if (!found) {
        orderedSections.push_back(field.section);
      }
    }

    for (const auto& action : module.actions().items()) {
      const String actionSection =
          action.section.isEmpty() ? String("General") : action.section;

      bool found = false;
      for (const auto& existing : orderedSections) {
        if (normalizeId(existing) == normalizeId(actionSection)) {
          found = true;
          break;
        }
      }

      if (!found) {
        orderedSections.push_back(actionSection);
      }
    }

    if (isWebSerial && hasArmorLinkSection) {
      orderedSections.insert(orderedSections.begin(), "ArmorLink");
    }

    for (const auto& sectionName : orderedSections) {
      JsonObject section = sections.createNestedObject();
      if (isWebSerial) {
        section["id"] = normalizeId(sectionName);
      }
      section["title"] = sectionName;

      JsonArray fields = section.createNestedArray("fields");

      for (const auto& field : module.config().items()) {
        if (field.section != sectionName) {
          continue;
        }

        if (!isWebSerial && field.kind == ArmorLinkFieldKind::Readonly) {
          continue;
        }

        JsonObject out = fields.createNestedObject();
        out["key"] = field.key;
        out["label"] = field.label;
        out["kind"] = fieldKindToString(field.kind);

        if (field.editable) {
          out["editable"] = true;
        }

        if (isWebSerial && field.advanced) {
          out["advanced"] = true;
        }

        if (field.rebootRequired) {
          out["rebootRequired"] = true;
        }

        if (isWebSerial && !field.description.isEmpty()) {
          out["description"] = field.description;
        }

        if (isWebSerial &&
            field.tooltip != nullptr &&
            field.tooltip[0] != '\0') {
          out["tooltip"] = field.tooltip;
        }

        if (isWebSerial &&
            field.visibleWhen.enabled &&
            !field.visibleWhen.key.isEmpty()) {
          JsonObject visibleWhen =
              out.createNestedObject("visibleWhen");
          visibleWhen["key"] = field.visibleWhen.key;
          visibleWhen["equals"] = field.visibleWhen.value;
        }

        if (isWebSerial && !field.unit.isEmpty()) {
          out["unit"] = field.unit;
        }

        if (isWebSerial && !field.semantic.isEmpty()) {
          out["semantic"] = field.semantic;
        }

        if (isWebSerial && !field.semanticGroup.isEmpty()) {
          out["semanticGroup"] = field.semanticGroup;
        }

        if (field.editable) {
          const String effectiveEntity =
              field.entity.isEmpty() ? String("config") : field.entity;
          const String effectiveCommand =
              field.command.isEmpty() ? field.key : field.command;

          if (isWebSerial || !effectiveEntity.equalsIgnoreCase("config")) {
            out["entity"] = effectiveEntity;
          }

          if (isWebSerial || !effectiveCommand.equalsIgnoreCase(field.key)) {
            out["command"] = effectiveCommand;
          }
        }

        switch (field.kind) {
          case ArmorLinkFieldKind::Int:
            if (field.intBinding.ptr) {
              out["value"] = *field.intBinding.ptr;
            } else {
              out["value"] = 0;
            }

            if (field.hasIntRange) {
              out["min"] = field.minInt;
              out["max"] = field.maxInt;
            }

            out["step"] = field.stepInt;
            break;

          case ArmorLinkFieldKind::Bool:
            if (field.boolBinding.ptr) {
              out["value"] = *field.boolBinding.ptr;
            } else {
              out["value"] = false;
            }
            break;

          case ArmorLinkFieldKind::String:
            if (field.stringBinding.ptr) {
              out["value"] = *field.stringBinding.ptr;
            } else {
              out["value"] = "";
            }
            break;

          case ArmorLinkFieldKind::Float:
            if (field.floatBinding.ptr) {
              out["value"] = *field.floatBinding.ptr;
            } else {
              out["value"] = 0.0f;
            }

            if (field.hasFloatRange) {
              out["min"] = field.minFloat;
              out["max"] = field.maxFloat;
            }

            out["step"] = field.stepFloat;
            break;

          case ArmorLinkFieldKind::Readonly:
          default:
            out["value"] = field.readonlyBinding.value;
            break;
        }
      }

      JsonArray actions = section.createNestedArray("actions");

      for (const auto& action : module.actions().items()) {
        const String actionSection =
            action.section.isEmpty() ? String("General") : action.section;

        if (normalizeId(actionSection) != normalizeId(sectionName)) {
          continue;
        }

        JsonObject out = actions.createNestedObject();
        appendAction(action, out, profile);
      }
    }
  }

  static void appendAction(const ArmorLinkActionDef& action, JsonObject out, Profile profile) {
    const bool isWebSerial = profile == Profile::WebSerial;
    out["id"] = action.id;
    out["label"] = action.label;
    out["entity"] = action.entity;
    out["command"] = action.command;

    if (!action.enabled) {
      out["enabled"] = false;
    }

    if (isWebSerial && action.style != ArmorLinkActionStyle::Secondary) {
      out["style"] = actionStyleToString(action.style);
    }

    if (isWebSerial && action.advanced) {
      out["advanced"] = true;
    }

    if (isWebSerial && !action.description.isEmpty()) {
      out["description"] = action.description;
    }

    if (!action.confirmText.isEmpty()) {
      out["confirmText"] = action.confirmText;
    }
  }

  static String normalizeId(const String& input) {
    String result;
    result.reserve(input.length());

    for (size_t i = 0; i < input.length(); ++i) {
      char c = input[i];
      if (isalnum(static_cast<unsigned char>(c))) {
        result += (char)tolower(static_cast<unsigned char>(c));
      } else if (c == ' ' || c == '_' || c == '-') {
        result += '_';
      }
    }

    if (result.isEmpty()) {
      result = "section";
    }

    return result;
  }
};
