#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <map>
#include <vector>
#include "ArmorLinkModule.h"

class ArmorLinkDescriptor {
public:
  static String build(const ArmorLinkModule& module) {
    StaticJsonDocument<4096> doc;

    doc["module"] = module.name();
    doc["configVersion"] = 1;
    doc["supportsPartialConfigGet"] = false;
    doc["supportsConfigSet"] = true;
    doc["moduleType"] = moduleTypeToString(module.type());

    JsonArray sections = doc.createNestedArray("sections");
    appendSections(module, sections);

    JsonArray actions = doc.createNestedArray("actions");
    appendActions(module, actions);

    String json;
    serializeJson(doc, json);
    return json;
  }

private:
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
      case ArmorLinkFieldKind::Bool: return "bool";
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

  static void appendSections(const ArmorLinkModule& module, JsonArray sections) {
    std::vector<String> orderedSections;

    for (const auto& field : module.config().items()) {
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
        out["editable"] = field.editable;
        out["advanced"] = field.advanced;

        if (!field.description.isEmpty()) {
          out["description"] = field.description;
        }

        if (!field.unit.isEmpty()) {
          out["unit"] = field.unit;
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

          case ArmorLinkFieldKind::Readonly:
          default:
            out["value"] = field.readonlyBinding.value;
            break;
        }
      }
    }
  }

  static void appendActions(const ArmorLinkModule& module, JsonArray actions) {
    for (const auto& action : module.actions().items()) {
      JsonObject out = actions.createNestedObject();
      out["id"] = action.id;
      out["label"] = action.label;
      out["entity"] = action.entity;
      out["command"] = action.command;
      out["enabled"] = action.enabled;
      out["style"] = actionStyleToString(action.style);
      out["advanced"] = action.advanced;

      if (!action.description.isEmpty()) {
        out["description"] = action.description;
      }

      if (!action.confirmText.isEmpty()) {
        out["confirmText"] = action.confirmText;
      }
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