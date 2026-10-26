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

    AL_INTERNAL_PRINTLN("[DESCRIPTOR] WebSerial descriptor failed, retrying BLE descriptor");
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
      AL_INTERNAL_PRINTF(
          "[DESCRIPTOR] JSON stream incomplete (%u of %u bytes)\n",
          static_cast<unsigned>(writtenLength),
          static_cast<unsigned>(measuredLength));
      return false;
    }

    return true;
  }

  static bool measureBle(const ArmorLinkModule& module, size_t& length) {
    ArmorLinkDescriptorCountingPrint counter;
    const bool ok = writeDocumentStreaming(module, counter, Profile::Ble);
    length = ok ? counter.length() : 0;
    return ok;
  }

  static bool writeBle(const ArmorLinkModule& module, Print& out) {
    return writeDocumentStreaming(module, out, Profile::Ble);
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
      AL_INTERNAL_PRINTF(
          "[DESCRIPTOR] BLE JSON buffer too small (%u of %u bytes)\n",
          static_cast<unsigned>(measuredLength),
          static_cast<unsigned>(bufferSize));
      return false;
    }

    length = serializeJson(doc, buffer, bufferSize);
    if (length != measuredLength) {
      AL_INTERNAL_PRINTF(
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
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;
    const bool hasBuilderSurface = hasActionBuilderSurface(module);
    const bool hasBuilderDescriptor =
        isWebSerial &&
        (ARMORLINK_HAS_ACTIONBUILDER && (hasBuilderSurface || module.actions().items().size() > 0));

    out.print('{');
    bool first = true;

    writeStringProperty(out, first, "module", module.name());
    writeStringProperty(out, first, "name", module.name());
    writeStringProperty(out, first, "moduleVersion", module.version());
    writeStringProperty(out, first, "armorLinkVersion", ARMORLINK_VERSION);
    writePropertyName(out, first, "features");
    out.print('{');
    bool firstFeature = true;
    writeBoolProperty(out, firstFeature, "espNow", ARMORLINK_HAS_ESPNOW);
    writeBoolProperty(out, firstFeature, "ble", ARMORLINK_HAS_BLE);
    writeBoolProperty(out, firstFeature, "actionBuilder", ARMORLINK_HAS_ACTIONBUILDER);
    writeBoolProperty(out, firstFeature, "dataStreams", ARMORLINK_HAS_DATASTREAMS);
    writeBoolProperty(out, firstFeature, "webSerial", ARMORLINK_HAS_WEB_SERIAL);
    out.print('}');
    if (isWebSerial) {
      writeStringProperty(out, first, "chipId", buildChipId());
      writeStringProperty(out, first, "espChipModel", String(ESP.getChipModel()));
      writeIntProperty(out, first, "espChipRevision", ESP.getChipRevision());
      writeIntProperty(out, first, "espChipCores", ESP.getChipCores());
      writeStringProperty(out, first, "espSdkVersion", String(ESP.getSdkVersion()));
    }

    if (isWebSerial && !module.profileName().isEmpty()) {
      if (!module.profileIdentifier().isEmpty()) {
        writeStringProperty(out, first, "profileIdentifier", module.profileIdentifier());
      }
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
    if (isWebSerial) {
      writeBoolProperty(out, first, "supportsStartupSequence", ARMORLINK_HAS_ACTIONBUILDER);
      writeBoolProperty(out, first, "supportsLogMessages", ARMORLINK_HAS_ACTIONBUILDER);
#if ARMORLINK_HAS_ACTIONBUILDER
      writeBoolProperty(out, first, "supportsServoAttachDetach", module.actionBuilder().supportsServoAttachDetach);
#else
      writeBoolProperty(out, first, "supportsServoAttachDetach", false);
#endif
      writeBoolProperty(out, first, "supportsActionBuilder",
                        ARMORLINK_HAS_ACTIONBUILDER && (hasBuilderSurface ||
                        module.actions().items().size() > 0));
    }
    writeStringProperty(out, first, "moduleType", moduleTypeToString(module.type()));

    if (isWebSerial && !module.profileTarget().isEmpty()) {
      writeStringProperty(out, first, "profileTarget", module.profileTarget());
    }

    if (hasBuilderDescriptor) {
      writePropertyName(out, first, "actionComponents");
      writeActionComponentsStreaming(module, out);

      writePropertyName(out, first, "actionDefinitions");
      writeActionDefinitionsStreaming(module, out);

      writePropertyName(out, first, "actionTargets");
      writeActionTargetsStreaming(module, out);
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
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;
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

#if ARMORLINK_HAS_ACTIONBUILDER
    for (const auto& action : module.actionBuilder().customActions()) {
      const String actionSection =
          action.section.isEmpty() ? String("Actions") : action.section;

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
#endif

    if (isWebSerial && hasArmorLinkSection) {
      // System actions may already have registered this section. Move it to
      // the front instead of emitting the same fields and actions twice.
      for (auto it = orderedSections.begin(); it != orderedSections.end();) {
        if (normalizeId(*it) == normalizeId("ArmorLink")) {
          it = orderedSections.erase(it);
        } else {
          ++it;
        }
      }
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
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;

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
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;
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
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;

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
      appendActionStreaming(action, out, profile, false);
      out.print('}');
    }

#if ARMORLINK_HAS_ACTIONBUILDER
    for (const auto& action : module.actionBuilder().customActions()) {
      const String actionSection =
          action.section.isEmpty() ? String("Actions") : action.section;

      if (normalizeId(actionSection) != normalizeId(sectionName)) {
        continue;
      }

      if (!firstAction) {
        out.print(',');
      }
      firstAction = false;

      out.print('{');
      appendActionStreaming(action, out, profile, true);
      out.print('}');
    }
#endif

  }

  static void appendActionStreaming(const ArmorLinkActionDef& action,
                                    Print& out,
                                    Profile profile,
                                    bool isCustomAction = false) {
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;
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

    if (isWebSerial) {
      writeBoolProperty(out, first, "custom", isCustomAction);
      writeBoolProperty(out, first, "editableInWeb", isCustomAction);
    }

    if (isWebSerial && !action.description.isEmpty()) {
      writeStringProperty(out, first, "description", action.description);
    }

    if (!action.confirmText.isEmpty()) {
      writeStringProperty(out, first, "confirmText", action.confirmText);
    }
  }

  static void writeActionComponentsStreaming(const ArmorLinkModule& module,
                                             Print& out) {
    out.print('[');

#if ARMORLINK_HAS_ACTIONBUILDER
    bool firstComponent = true;
    for (const auto& component : module.actionBuilder().components()) {
      if (!firstComponent) {
        out.print(',');
      }
      firstComponent = false;

      bool first = true;
      out.print('{');
      writeStringProperty(out, first, "id", component.id);
      writeStringProperty(out, first, "label", component.label);
      writeStringProperty(out, first, "kind",
                          actionComponentKindToString(component.kind));
      writeBoolProperty(out, first, "custom", component.custom);
      writeBoolProperty(out, first, "editableInWeb", component.custom);
      if (!component.section.isEmpty()) {
        writeStringProperty(out, first, "section", component.section);
      }
      if (component.kind == ArmorLinkActionComponentKind::State) {
        writeBoolProperty(out, first, "defaultValue", component.defaultValue);
        writeBoolProperty(out, first, "value", component.ready);
      }
      if (component.kind == ArmorLinkActionComponentKind::Servo &&
          component.hasServoConfig) {
        writeIntProperty(out, first, "pin", component.servoPin);
        writeIntProperty(out, first, "openPosition", component.servoOpenPosition);
        writeIntProperty(out, first, "closedPosition", component.servoClosedPosition);
        writeIntProperty(out, first, "minPulseUs", component.servoMinPulseUs);
        writeIntProperty(out, first, "maxPulseUs", component.servoMaxPulseUs);
        if (!component.servoProfile.isEmpty()) {
          writeStringProperty(out, first, "servoProfile", component.servoProfile);
        }
      }
      if (component.kind == ArmorLinkActionComponentKind::Led &&
          component.hasLedConfig) {
        writeIntProperty(out, first, "ledPin", component.ledPin);
        writeIntProperty(out, first, "ledCount", component.ledCount);
        writeIntProperty(out, first, "ledStartIndex", component.ledStartIndex);
        writeIntProperty(out, first, "ledBrightness", component.ledBrightness);
        writeStringProperty(out, first, "ledChipset", component.ledChipset);
        writeStringProperty(out, first, "ledColorOrder", component.ledColorOrder);
      }
      if (component.kind == ArmorLinkActionComponentKind::Gpio &&
          component.hasGpioConfig) {
        writeIntProperty(out, first, "gpioPin", component.gpioPin);
        writeStringProperty(out, first, "gpioMode", component.gpioMode);
        writeBoolProperty(out, first, "gpioDefaultLevel", component.gpioDefaultLevel);
        writeBoolProperty(out, first, "gpioActiveLow", component.gpioActiveLow);
      }
      if (component.kind == ArmorLinkActionComponentKind::Button &&
          component.hasButtonConfig) {
        writeIntProperty(out, first, "buttonPin", component.buttonPin);
        writeBoolProperty(out, first, "buttonActiveLow", component.buttonActiveLow);
        writeIntProperty(out, first, "buttonDebounceMs", component.buttonDebounceMs);
        writeIntProperty(out, first, "buttonMultiGapMs", component.buttonMultiGapMs);
        writeIntProperty(out, first, "buttonLongPressMs", component.buttonLongPressMs);

        if (!component.buttonEvents.empty()) {
          if (!first) {
            out.print(',');
          }
          first = false;
          out.print("\"buttonEvents\":[");

          bool firstRoute = true;
          for (const auto& route : component.buttonEvents) {
            if (!firstRoute) {
              out.print(',');
            }
            firstRoute = false;

            bool firstRouteProperty = true;
            out.print('{');
            writeStringProperty(out, firstRouteProperty, "event", route.event);
            writeStringProperty(out, firstRouteProperty, "mode", route.mode);
            writeBoolProperty(out, firstRouteProperty, "enabled", route.enabled);
            writeStringProperty(out, firstRouteProperty, "action", route.action);
            writeStringProperty(out, firstRouteProperty, "target", route.target);
            writeStringProperty(out, firstRouteProperty, "entity", route.entity);
            writeStringProperty(out, firstRouteProperty, "command", route.command);
            out.print('}');
          }

          out.print(']');
        }
      }
      out.print('}');
    }
#endif

    out.print(']');
  }

  static void writeActionDefinitionsStreaming(const ArmorLinkModule& module,
                                              Print& out) {
    out.print('[');

#if ARMORLINK_HAS_ACTIONBUILDER
    bool firstDefinition = true;
    for (const auto& definition : module.actionBuilder().definitions()) {
      if (!firstDefinition) {
        out.print(',');
      }
      firstDefinition = false;

      bool first = true;
      out.print('{');
      writeBoolProperty(out, first, "startup", definition.startup);
      writeStringProperty(out, first, "entity", definition.entity);
      writeStringProperty(out, first, "command", definition.command);
      if (!definition.label.isEmpty()) {
        writeStringProperty(out, first, "label", definition.label);
      }
      if (!definition.section.isEmpty()) {
        writeStringProperty(out, first, "section", definition.section);
      }
      writePropertyName(out, first, "operations");
      out.print('[');

      bool firstOp = true;
      for (const auto& op : definition.ops) {
        if (!firstOp) {
          out.print(',');
        }
        firstOp = false;

        bool firstOpProperty = true;
        out.print('{');
        writeStringProperty(out, firstOpProperty, "op", actionOpCodeToString(op.code));
        if (op.code == ArmorLinkActionOpCode::Wait ||
            op.code == ArmorLinkActionOpCode::WaitUntilState) {
          writeIntProperty(out, firstOpProperty, "durationMs", op.argument);
        }
        if (op.code == ArmorLinkActionOpCode::ServoMoveTo) {
          writeIntProperty(out, firstOpProperty, "position", op.argument);
          writeIntProperty(out, firstOpProperty, "durationMs", op.durationMs);
          writeStringProperty(out, firstOpProperty, "easing", op.option == 1 ? "easeIn" : op.option == 2 ? "easeOut" : op.option == 3 ? "easeInOut" : "linear");
        }
        if (op.code == ArmorLinkActionOpCode::LedSetColor) {
          char color[8]; snprintf(color, sizeof(color), "#%06lX", static_cast<unsigned long>(op.color & 0xFFFFFF));
          writeStringProperty(out, firstOpProperty, "color", color);
        }
        if (op.code == ArmorLinkActionOpCode::LedSetBrightness) {
          writeIntProperty(out, firstOpProperty, "brightness", op.intensity);
        }
        if (op.code == ArmorLinkActionOpCode::LedAnimation) {
          writeStringProperty(out, firstOpProperty, "animation", op.option == 1 ? "pulse" : op.option == 2 ? "blink" : op.option == 3 ? "chase" : op.option == 4 ? "solid" : "rainbow");
          writeIntProperty(out, firstOpProperty, "durationMs", op.durationMs);
        }
        if (op.code == ArmorLinkActionOpCode::LogMessage && op.logMessage) {
          writeStringProperty(out, firstOpProperty, "level", op.logMessage->level == AL_LOG_DEBUG ? "debug" : op.logMessage->level == AL_LOG_WARN ? "warn" : op.logMessage->level == AL_LOG_ERROR ? "error" : "info");
          writeStringProperty(out, firstOpProperty, "text", op.logMessage->text);
        }
        if (op.code == ArmorLinkActionOpCode::StateSet) {
          writeBoolProperty(out, firstOpProperty, "value", op.value);
        }
        if (op.code == ArmorLinkActionOpCode::RepeatPrevious) {
          writeIntProperty(out, firstOpProperty, "count", op.argument);
        } else if (op.code == ArmorLinkActionOpCode::SendActionToTarget &&
                   op.argument < module.actionBuilder().remoteCommands().size()) {
          const auto& remoteCommand = module.actionBuilder().remoteCommands()[op.argument];
          writeStringProperty(out, firstOpProperty, "target", remoteCommand.target);
          writeStringProperty(out, firstOpProperty, "entity", remoteCommand.entity);
          writeStringProperty(out, firstOpProperty, "command", remoteCommand.command);
        } else if (op.code != ArmorLinkActionOpCode::Wait &&
                   op.componentIndex < module.actionBuilder().components().size()) {
          writeStringProperty(
              out,
              firstOpProperty,
              "component",
              module.actionBuilder().components()[op.componentIndex].id);
        }
        out.print('}');
      }

      out.print(']');
      out.print('}');
    }
#endif

    out.print(']');
  }

  static void writeActionTargetsStreaming(const ArmorLinkModule& module,
                                          Print& out) {
    out.print('[');

#if ARMORLINK_HAS_ACTIONBUILDER
    bool firstTarget = true;
    for (const auto& target : module.actionBuilder().actionTargets(module.actions())) {
      if (!firstTarget) {
        out.print(',');
      }
      firstTarget = false;

      bool first = true;
      out.print('{');
      writeStringProperty(out, first, "id", target.id);
      writeStringProperty(out, first, "label", target.label);
      writeStringProperty(out, first, "entity", target.entity);
      writeStringProperty(out, first, "command", target.command);
      out.print('}');
    }
#endif

    out.print(']');
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

  class ArmorLinkDescriptorStringPrint : public Print {
  public:
    explicit ArmorLinkDescriptorStringPrint(String& value) : _value(value) {}
    size_t write(uint8_t value) override {
      if (!_value.concat(static_cast<char>(value))) {
        _failed = true;
        return 0;
      }
      return 1;
    }
    bool failed() const { return _failed; }
  private:
    String& _value;
    bool _failed = false;
  };

  static String buildWithProfile(const ArmorLinkModule& module,
                                 Profile profile,
                                 size_t /* jsonCapacity */) {
    // Serialize directly without allocating the full JSON object tree, which
    // can exhaust a fragmented heap even for a small serialized descriptor.
    String json;
    ArmorLinkDescriptorStringPrint out(json);
    if (!writeDocumentStreaming(module, out, profile) || out.failed()) {
      AL_INTERNAL_PRINTF("[DESCRIPTOR] JSON streaming failed (profile=%s)\n",
                    profileName(profile));
      return "";
    }
    return json;
  }

  static bool populateDocument(const ArmorLinkModule& module,
                               DynamicJsonDocument& doc,
                               Profile profile) {
    doc.clear();

    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;

    doc["module"] = module.name();
    doc["name"] = module.name();
    doc["moduleVersion"] = module.version();
    doc["armorLinkVersion"] = ARMORLINK_VERSION;
    JsonObject features = doc.createNestedObject("features");
    features["espNow"] = static_cast<bool>(ARMORLINK_HAS_ESPNOW);
    features["ble"] = static_cast<bool>(ARMORLINK_HAS_BLE);
    features["actionBuilder"] = static_cast<bool>(ARMORLINK_HAS_ACTIONBUILDER);
    features["dataStreams"] = static_cast<bool>(ARMORLINK_HAS_DATASTREAMS);
    features["webSerial"] = static_cast<bool>(ARMORLINK_HAS_WEB_SERIAL);
    if (isWebSerial) {
      doc["chipId"] = buildChipId();
      doc["espChipModel"] = String(ESP.getChipModel());
      doc["espChipRevision"] = ESP.getChipRevision();
      doc["espChipCores"] = ESP.getChipCores();
      doc["espSdkVersion"] = String(ESP.getSdkVersion());
    }
    if (isWebSerial && !module.profileName().isEmpty()) {
      if (!module.profileIdentifier().isEmpty()) {
        doc["profileIdentifier"] = module.profileIdentifier();
      }
      doc["profileName"] = module.profileName();
      doc["activeProfileName"] = module.profileName();
      doc["profileNameSource"] = module.profileNameImported()
          ? "imported"
          : "firmware";
    }
    if (isWebSerial && !module.defaultProfileName().isEmpty()) {
      doc["defaultProfileName"] = module.defaultProfileName();
    }
    const bool hasBuilderSurface = hasActionBuilderSurface(module);
    const bool hasBuilderDescriptor =
        isWebSerial &&
        (ARMORLINK_HAS_ACTIONBUILDER && (hasBuilderSurface || module.actions().items().size() > 0));
    doc["supportsPartialConfigGet"] = false;
    doc["supportsConfigSet"] = true;
    if (isWebSerial) {
      doc["supportsStartupSequence"] = bool(ARMORLINK_HAS_ACTIONBUILDER);
      doc["supportsLogMessages"] = bool(ARMORLINK_HAS_ACTIONBUILDER);
#if ARMORLINK_HAS_ACTIONBUILDER
      doc["supportsServoAttachDetach"] = module.actionBuilder().supportsServoAttachDetach;
#else
      doc["supportsServoAttachDetach"] = false;
#endif
      doc["supportsActionBuilder"] =
          ARMORLINK_HAS_ACTIONBUILDER &&
          (hasBuilderSurface || module.actions().items().size() > 0);
    }
    doc["moduleType"] = moduleTypeToString(module.type());
    if (isWebSerial && !module.profileTarget().isEmpty()) {
      doc["profileTarget"] = module.profileTarget();
    }
    if (hasBuilderDescriptor) {
      JsonArray actionComponents =
          doc.createNestedArray("actionComponents");
      appendActionComponents(module, actionComponents);

      JsonArray actionDefinitions =
          doc.createNestedArray("actionDefinitions");
      appendActionDefinitions(module, actionDefinitions);

      JsonArray actionTargets =
          doc.createNestedArray("actionTargets");
      appendActionTargets(module, actionTargets);
    }
    JsonArray sections = doc.createNestedArray("sections");
    appendSections(module, sections, profile);

    if (doc.overflowed()) {
      AL_INTERNAL_PRINTF("[DESCRIPTOR] JSON document overflowed (profile=%s)\n",
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

  static bool hasActionBuilderSurface(const ArmorLinkModule& module) {
#if ARMORLINK_HAS_ACTIONBUILDER
    return module.actionBuilder().components().size() > 0 ||
           module.actionBuilder().definitions().size() > 0 ||
           module.actionBuilder().customActions().size() > 0;
#else
    return false;
#endif
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

  static const char* actionComponentKindToString(
      ArmorLinkActionComponentKind kind) {
    switch (kind) {
      case ArmorLinkActionComponentKind::Servo: return "servo";
      case ArmorLinkActionComponentKind::Led: return "led";
      case ArmorLinkActionComponentKind::Gpio: return "gpio";
      case ArmorLinkActionComponentKind::Button: return "button";
      case ArmorLinkActionComponentKind::State: return "state";
      default: return "gpio";
    }
  }

  static const char* actionOpCodeToString(ArmorLinkActionOpCode op) {
    switch (op) {
      case ArmorLinkActionOpCode::ServoAttach: return "servo_attach";
      case ArmorLinkActionOpCode::ServoDetach: return "servo_detach";
      case ArmorLinkActionOpCode::ServoOpen: return "servo_open";
      case ArmorLinkActionOpCode::ServoClose: return "servo_close";
      case ArmorLinkActionOpCode::ServoMoveTo: return "servo_move_to";
      case ArmorLinkActionOpCode::LedOn: return "led_on";
      case ArmorLinkActionOpCode::LedOff: return "led_off";
      case ArmorLinkActionOpCode::LedSetColor: return "led_set_color";
      case ArmorLinkActionOpCode::LedSetBrightness: return "led_set_brightness";
      case ArmorLinkActionOpCode::LedAnimation: return "led_animation";
      case ArmorLinkActionOpCode::Wait: return "wait";
      case ArmorLinkActionOpCode::GpioHigh: return "gpio_high";
      case ArmorLinkActionOpCode::GpioLow: return "gpio_low";
      case ArmorLinkActionOpCode::ActionCall: return "action_call";
      case ArmorLinkActionOpCode::WaitUntilState: return "wait_until_state";
      case ArmorLinkActionOpCode::RepeatPrevious: return "repeat_previous";
      case ArmorLinkActionOpCode::LogMessage: return "log_message";
      case ArmorLinkActionOpCode::StateSet: return "state_set";
      case ArmorLinkActionOpCode::SendActionToTarget: return "send_command_to_target";
      default: return "wait";
    }
  }

  static void appendActionComponents(
      const ArmorLinkModule& module,
      JsonArray actionComponents) {

#if ARMORLINK_HAS_ACTIONBUILDER
    for (const auto& component : module.actionBuilder().components()) {
      JsonObject out = actionComponents.createNestedObject();
      out["id"] = component.id;
      out["label"] = component.label;
      out["kind"] = actionComponentKindToString(component.kind);
      out["custom"] = component.custom;
      out["editableInWeb"] = component.custom;
      if (!component.section.isEmpty()) {
        out["section"] = component.section;
      }
      if (component.kind == ArmorLinkActionComponentKind::State) {
        out["defaultValue"] = component.defaultValue;
        out["value"] = component.ready;
      }
      if (component.kind == ArmorLinkActionComponentKind::Servo &&
          component.hasServoConfig) {
        out["pin"] = component.servoPin;
        out["openPosition"] = component.servoOpenPosition;
        out["closedPosition"] = component.servoClosedPosition;
        out["minPulseUs"] = component.servoMinPulseUs;
        out["maxPulseUs"] = component.servoMaxPulseUs;
        if (!component.servoProfile.isEmpty()) {
          out["servoProfile"] = component.servoProfile;
        }
      }
      if (component.kind == ArmorLinkActionComponentKind::Led &&
          component.hasLedConfig) {
        out["ledPin"] = component.ledPin;
        out["ledCount"] = component.ledCount;
        out["ledStartIndex"] = component.ledStartIndex;
        out["ledBrightness"] = component.ledBrightness;
        out["ledChipset"] = component.ledChipset;
        out["ledColorOrder"] = component.ledColorOrder;
      }
      if (component.kind == ArmorLinkActionComponentKind::Gpio &&
          component.hasGpioConfig) {
        out["gpioPin"] = component.gpioPin;
        out["gpioMode"] = component.gpioMode;
        out["gpioDefaultLevel"] = component.gpioDefaultLevel;
        out["gpioActiveLow"] = component.gpioActiveLow;
      }
      if (component.kind == ArmorLinkActionComponentKind::Button &&
          component.hasButtonConfig) {
        out["buttonPin"] = component.buttonPin;
        out["buttonActiveLow"] = component.buttonActiveLow;
        out["buttonDebounceMs"] = component.buttonDebounceMs;
        out["buttonMultiGapMs"] = component.buttonMultiGapMs;
        out["buttonLongPressMs"] = component.buttonLongPressMs;

        JsonArray buttonEvents = out.createNestedArray("buttonEvents");
        for (const auto& route : component.buttonEvents) {
          JsonObject routeJson = buttonEvents.createNestedObject();
          routeJson["event"] = route.event;
          routeJson["mode"] = route.mode;
          routeJson["enabled"] = route.enabled;
          routeJson["action"] = route.action;
          routeJson["target"] = route.target;
          routeJson["entity"] = route.entity;
          routeJson["command"] = route.command;
        }
      }
    }
#endif

  }

  static void appendActionDefinitions(
      const ArmorLinkModule& module,
      JsonArray actionDefinitions) {

#if ARMORLINK_HAS_ACTIONBUILDER
    for (const auto& definition : module.actionBuilder().definitions()) {
      JsonObject out = actionDefinitions.createNestedObject();
      out["startup"] = definition.startup;
      out["entity"] = definition.entity;
      out["command"] = definition.command;
      if (!definition.label.isEmpty()) {
        out["label"] = definition.label;
      }
      if (!definition.section.isEmpty()) {
        out["section"] = definition.section;
      }

      JsonArray operations = out.createNestedArray("operations");
      for (const auto& op : definition.ops) {
        JsonObject opOut = operations.createNestedObject();
        opOut["op"] = actionOpCodeToString(op.code);

        if (op.code == ArmorLinkActionOpCode::Wait ||
            op.code == ArmorLinkActionOpCode::WaitUntilState) {
          opOut["durationMs"] = op.argument;
        }
        if (op.code == ArmorLinkActionOpCode::ServoMoveTo) {
          opOut["position"] = op.argument;
          opOut["durationMs"] = op.durationMs;
          opOut["easing"] = op.option == 1 ? "easeIn" : op.option == 2 ? "easeOut" : op.option == 3 ? "easeInOut" : "linear";
        }
        if (op.code == ArmorLinkActionOpCode::LedSetColor) {
          char color[8]; snprintf(color, sizeof(color), "#%06lX", static_cast<unsigned long>(op.color & 0xFFFFFF));
          opOut["color"] = color;
        }
        if (op.code == ArmorLinkActionOpCode::LedSetBrightness) opOut["brightness"] = op.intensity;
        if (op.code == ArmorLinkActionOpCode::LedAnimation) {
          opOut["animation"] = op.option == 1 ? "pulse" : op.option == 2 ? "blink" : op.option == 3 ? "chase" : op.option == 4 ? "solid" : "rainbow";
          opOut["durationMs"] = op.durationMs;
        }
        if (op.code == ArmorLinkActionOpCode::LogMessage && op.logMessage) {
          opOut["level"] = op.logMessage->level == AL_LOG_DEBUG ? "debug" : op.logMessage->level == AL_LOG_WARN ? "warn" : op.logMessage->level == AL_LOG_ERROR ? "error" : "info";
          opOut["text"] = op.logMessage->text;
        }
        if (op.code == ArmorLinkActionOpCode::StateSet) {
          opOut["value"] = op.value;
        }
        if (op.code == ArmorLinkActionOpCode::RepeatPrevious) {
          opOut["count"] = op.argument;
        } else if (op.code == ArmorLinkActionOpCode::SendActionToTarget &&
                   op.argument < module.actionBuilder().remoteCommands().size()) {
          const auto& remoteCommand = module.actionBuilder().remoteCommands()[op.argument];
          opOut["target"] = remoteCommand.target;
          opOut["entity"] = remoteCommand.entity;
          opOut["command"] = remoteCommand.command;
        } else if (op.code != ArmorLinkActionOpCode::Wait &&
                   op.componentIndex < module.actionBuilder().components().size()) {
          opOut["component"] =
              module.actionBuilder().components()[op.componentIndex].id;
        }
      }
    }
#endif

  }

  static void appendActionTargets(
      const ArmorLinkModule& module,
      JsonArray actionTargets) {

#if ARMORLINK_HAS_ACTIONBUILDER
    for (const auto& target : module.actionBuilder().actionTargets(module.actions())) {
      JsonObject out = actionTargets.createNestedObject();
      out["id"] = target.id;
      out["label"] = target.label;
      out["entity"] = target.entity;
      out["command"] = target.command;
    }
#endif

  }

  static void appendSections(const ArmorLinkModule& module, JsonArray sections, Profile profile) {
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;
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

#if ARMORLINK_HAS_ACTIONBUILDER
      for (const auto& action : module.actionBuilder().customActions()) {
      const String actionSection =
          action.section.isEmpty() ? String("Actions") : action.section;

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
#endif

    if (isWebSerial && hasArmorLinkSection) {
      // System actions may already have registered this section. Move it to
      // the front instead of emitting the same fields and actions twice.
      for (auto it = orderedSections.begin(); it != orderedSections.end();) {
        if (normalizeId(*it) == normalizeId("ArmorLink")) {
          it = orderedSections.erase(it);
        } else {
          ++it;
        }
      }
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
        appendAction(action, out, profile, false);
      }

#if ARMORLINK_HAS_ACTIONBUILDER
      for (const auto& action : module.actionBuilder().customActions()) {
        const String actionSection =
            action.section.isEmpty() ? String("Actions") : action.section;

        if (normalizeId(actionSection) != normalizeId(sectionName)) {
          continue;
        }

        JsonObject out = actions.createNestedObject();
        appendAction(action, out, profile, true);
      }
#endif

    }
  }

  static void appendAction(
      const ArmorLinkActionDef& action,
      JsonObject out,
      Profile profile,
      bool isCustomAction = false) {
    const bool isWebSerial = ARMORLINK_HAS_WEB_SERIAL && profile == Profile::WebSerial;
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

    if (isWebSerial) {
      out["custom"] = isCustomAction;
      out["editableInWeb"] = isCustomAction;
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

  static String buildChipId() {
    const uint64_t chipId = ESP.getEfuseMac();
    char uniqueId[18] = { 0 };
    snprintf(
        uniqueId,
        sizeof(uniqueId),
        "%04X%08lX",
        static_cast<unsigned>(chipId >> 32) & 0xFFFFu,
        static_cast<unsigned long>(chipId & 0xFFFFFFFFu));
    return String(uniqueId);
  }
};
