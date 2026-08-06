#pragma once

#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "ArmorLinkModule.h"

class ArmorLinkDescriptor {
public:
  static String build(const ArmorLinkModule& module) {
    String json = buildWithOptions(module, true);
    if (!json.isEmpty()) {
      return json;
    }

    Serial.println("[DESCRIPTOR] Full descriptor failed, retrying compact descriptor");
    return buildWithOptions(module, false);
  }

  static bool write(const ArmorLinkModule& module, Print& out, size_t* length = nullptr) {
    DynamicJsonDocument doc(DescriptorJsonCapacity);

    if (!populateDocument(module, doc, true) &&
        !populateDocument(module, doc, false)) {
      return false;
    }

    const size_t measuredLength = measureJson(doc);
    if (length != nullptr) {
      *length = measuredLength;
    }

    const size_t writtenLength = serializeJson(doc, out);
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
    DynamicJsonDocument doc(DescriptorJsonCapacity);

    if (!populateDocument(module, doc, true) &&
        !populateDocument(module, doc, false)) {
      length = 0;
      return false;
    }

    length = measureJson(doc);
    return true;
  }

private:
  static constexpr size_t DescriptorJsonCapacity = 65536;

  static String buildWithOptions(const ArmorLinkModule& module, bool richMetadata) {
    DynamicJsonDocument doc(DescriptorJsonCapacity);

    if (!populateDocument(module, doc, richMetadata)) {
      return "";
    }

    const size_t measuredLength = measureJson(doc);

    String json;
    if (!json.reserve(measuredLength + 1)) {
      Serial.printf(
          "[DESCRIPTOR] JSON string allocation failed (%u bytes, rich=%s)\n",
          static_cast<unsigned>(measuredLength),
          richMetadata ? "true" : "false");
      return "";
    }

    serializeJson(doc, json);

    if (json.length() != measuredLength) {
      Serial.printf(
          "[DESCRIPTOR] JSON serialization incomplete (%u of %u bytes, rich=%s)\n",
          static_cast<unsigned>(json.length()),
          static_cast<unsigned>(measuredLength),
          richMetadata ? "true" : "false");
      return "";
    }

    return json;
  }

  static bool populateDocument(const ArmorLinkModule& module, DynamicJsonDocument& doc, bool richMetadata) {
    doc.clear();

    doc["module"] = module.name();
    doc["name"] = module.name();
    doc["moduleVersion"] = module.version();
    doc["armorLinkVersion"] = ARMORLINK_VERSION;
    if (richMetadata && !module.profileName().isEmpty()) {
      doc["profileName"] = module.profileName();
      doc["activeProfileName"] = module.profileName();
      doc["profileNameSource"] = module.profileNameImported()
          ? "imported"
          : "firmware";
    }
    if (richMetadata && !module.defaultProfileName().isEmpty()) {
      doc["defaultProfileName"] = module.defaultProfileName();
    }
    doc["supportsPartialConfigGet"] = false;
    doc["supportsConfigSet"] = true;
    doc["moduleType"] = moduleTypeToString(module.type());
    if (richMetadata && !module.profileTarget().isEmpty()) {
      doc["profileTarget"] = module.profileTarget();
    }
    JsonArray sections = doc.createNestedArray("sections");
    appendSections(module, sections, richMetadata);

    if (doc.overflowed()) {
      Serial.printf("[DESCRIPTOR] JSON document overflowed (rich=%s)\n",
                    richMetadata ? "true" : "false");
      return false;
    }

    return true;
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

  static void appendSections(const ArmorLinkModule& module, JsonArray sections, bool richMetadata) {
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

    if (hasArmorLinkSection) {
      orderedSections.insert(orderedSections.begin(), "ArmorLink");
    }

    for (const auto& sectionName : orderedSections) {
      JsonObject section = sections.createNestedObject();
      section["id"] = normalizeId(sectionName);
      section["title"] = sectionName;

      JsonArray fields = section.createNestedArray("fields");

      for (const auto& field : module.config().items()) {
        if (field.section != sectionName) {
          continue;
        }

        JsonObject out = fields.createNestedObject();
        out["key"] = field.key;
        out["label"] = field.label;
        out["kind"] = fieldKindToString(field.kind);

        if (field.editable) {
          out["editable"] = true;
        }

        if (richMetadata && field.advanced) {
          out["advanced"] = true;
        }

        if (richMetadata && field.rebootRequired) {
          out["rebootRequired"] = true;
        }

        if (richMetadata && !field.description.isEmpty()) {
          out["description"] = field.description;
        }

        if (richMetadata &&
            field.tooltip != nullptr &&
            field.tooltip[0] != '\0') {
          out["tooltip"] = field.tooltip;
        }

        if (richMetadata &&
            field.visibleWhen.enabled &&
            !field.visibleWhen.key.isEmpty()) {
          JsonObject visibleWhen =
              out.createNestedObject("visibleWhen");
          visibleWhen["key"] = field.visibleWhen.key;
          visibleWhen["equals"] = field.visibleWhen.value;
        }

        if (richMetadata && !field.unit.isEmpty()) {
          out["unit"] = field.unit;
        }

        if (richMetadata && !field.semantic.isEmpty()) {
          out["semantic"] = field.semantic;
        }

        if (richMetadata && !field.semanticGroup.isEmpty()) {
          out["semanticGroup"] = field.semanticGroup;
        }

        if (field.editable) {
          out["entity"] = field.entity.isEmpty() ? "config" : field.entity;
          out["command"] = field.command.isEmpty() ? field.key : field.command;
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
        appendAction(action, out, richMetadata);
      }
    }
  }

  static void appendAction(const ArmorLinkActionDef& action, JsonObject out, bool richMetadata) {
    out["id"] = action.id;
    out["label"] = action.label;
    out["entity"] = action.entity;
    out["command"] = action.command;

    if (!action.enabled) {
      out["enabled"] = false;
    }

    if (richMetadata && action.style != ArmorLinkActionStyle::Secondary) {
      out["style"] = actionStyleToString(action.style);
    }

    if (richMetadata && action.advanced) {
      out["advanced"] = true;
    }

    if (richMetadata && !action.description.isEmpty()) {
      out["description"] = action.description;
    }

    if (richMetadata && !action.confirmText.isEmpty()) {
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
