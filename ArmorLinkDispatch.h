#pragma once

#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include "ArmorLinkModule.h"
#include "ArmorLinkStorage.h"

enum class ArmorLinkDispatchResult {
  Ok,
  NotFound,
  InvalidValue,
  NotEditable,
  StorageError,
  NoCallback
};

class ArmorLinkDispatch {
public:
  void begin(ArmorLinkModule* module, ArmorLinkStorage* storage) {
    _module = module;
    _storage = storage;
  }

  ArmorLinkDispatchResult handleConfigSet(
      const String& entity,
      const String& command,
      int32_t value)
  {
    if (!_module) return ArmorLinkDispatchResult::NotFound;

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

      if (!explicitCommandMatch && !defaultConfigMatch && !keyFallbackMatch) continue;
      if (!field.editable) return ArmorLinkDispatchResult::NotEditable;

      switch (field.kind) {
        case ArmorLinkFieldKind::Int:
          return handleInt(field, static_cast<int>(value));

        case ArmorLinkFieldKind::Float:
          return handleFloat(field, static_cast<float>(value));

        case ArmorLinkFieldKind::Bool:
          return handleBool(field, value != 0);

        case ArmorLinkFieldKind::Readonly:
        default:
          return ArmorLinkDispatchResult::NotEditable;
      }
    }

    return ArmorLinkDispatchResult::NotFound;
  }

  ArmorLinkDispatchResult handleConfigSet(
      const String& entity,
      const String& command,
      float value)
  {
    if (!_module) return ArmorLinkDispatchResult::NotFound;

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

      if (!explicitCommandMatch && !defaultConfigMatch && !keyFallbackMatch) continue;
      if (!field.editable) return ArmorLinkDispatchResult::NotEditable;

      switch (field.kind) {
        case ArmorLinkFieldKind::Float:
          return handleFloat(field, value);

        default:
          return ArmorLinkDispatchResult::InvalidValue;
      }
    }

    return ArmorLinkDispatchResult::NotFound;
  }

  ArmorLinkDispatchResult handleConfigSet(
      const String& entity,
      const String& command,
      const String& value)
  {
    if (!_module) return ArmorLinkDispatchResult::NotFound;

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

      if (!explicitCommandMatch && !defaultConfigMatch && !keyFallbackMatch) continue;
      if (!field.editable) return ArmorLinkDispatchResult::NotEditable;

      switch (field.kind) {
        case ArmorLinkFieldKind::String:
          return handleString(field, value);

        case ArmorLinkFieldKind::Float:
          return handleFloat(field, value.toFloat());

        case ArmorLinkFieldKind::Int:
          return handleInt(field, value.toInt());

        case ArmorLinkFieldKind::Bool:
          return handleBool(
            field,
            value.equalsIgnoreCase("true") ||
            value == "1" ||
            value.equalsIgnoreCase("yes") ||
            value.equalsIgnoreCase("on"));

        default:
          return ArmorLinkDispatchResult::InvalidValue;
      }
    }

    return ArmorLinkDispatchResult::NotFound;
  }

  ArmorLinkDispatchResult handleAction(
      const String& entity,
      const String& command)
  {
    if (!_module) return ArmorLinkDispatchResult::NotFound;

    for (auto& action : _module->actions().items()) {
      if (action.entity != entity || action.command != command) continue;
      if (!action.enabled) return ArmorLinkDispatchResult::NotEditable;
      if (!action.callback) return ArmorLinkDispatchResult::NoCallback;

      action.callback();
      return ArmorLinkDispatchResult::Ok;
    }

    return ArmorLinkDispatchResult::NotFound;
  }

  static const char* toString(ArmorLinkDispatchResult result) {
    switch (result) {
      case ArmorLinkDispatchResult::Ok: return "ok";
      case ArmorLinkDispatchResult::NotFound: return "not_found";
      case ArmorLinkDispatchResult::InvalidValue: return "invalid_value";
      case ArmorLinkDispatchResult::NotEditable: return "not_editable";
      case ArmorLinkDispatchResult::StorageError: return "storage_error";
      case ArmorLinkDispatchResult::NoCallback: return "no_callback";
      default: return "unknown";
    }
  }

private:
  ArmorLinkModule* _module = nullptr;
  ArmorLinkStorage* _storage = nullptr;

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

  ArmorLinkDispatchResult handleInt(ArmorLinkConfigFieldDef& field, int value) {
    if (!field.intBinding.ptr) {
      return ArmorLinkDispatchResult::InvalidValue;
    }

    const int originalValue = value;
    value = clampIntValue(field, value);

    if (value != originalValue) {
      Serial.printf("[CONFIG] Clamped incoming value for %s from %d to %d\n",
                    field.key.c_str(),
                    originalValue,
                    value);
    }

    *field.intBinding.ptr = value;

    if (!_storage || !_storage->saveField(field)) {
      return ArmorLinkDispatchResult::StorageError;
    }

    if (field.onIntChange) {
      field.onIntChange(value);
    }

    return ArmorLinkDispatchResult::Ok;
  }

  ArmorLinkDispatchResult handleFloat(ArmorLinkConfigFieldDef& field, float value) {
    if (!field.floatBinding.ptr) {
      return ArmorLinkDispatchResult::InvalidValue;
    }

    const float originalValue = value;
    value = clampFloatValue(field, value);

    if (value != originalValue) {
      Serial.printf("[CONFIG] Clamped incoming value for %s from %.3f to %.3f\n",
                    field.key.c_str(),
                    originalValue,
                    value);
    }

    *field.floatBinding.ptr = value;

    if (!_storage || !_storage->saveField(field)) {
      return ArmorLinkDispatchResult::StorageError;
    }

    if (field.onFloatChange) {
      field.onFloatChange(value);
    }

    return ArmorLinkDispatchResult::Ok;
  }

  ArmorLinkDispatchResult handleBool(ArmorLinkConfigFieldDef& field, bool value) {
    if (!field.boolBinding.ptr) {
      return ArmorLinkDispatchResult::InvalidValue;
    }

    *field.boolBinding.ptr = value;

    if (!_storage || !_storage->saveField(field)) {
      return ArmorLinkDispatchResult::StorageError;
    }

    if (field.onBoolChange) {
      field.onBoolChange(value);
    }

    return ArmorLinkDispatchResult::Ok;
  }

  ArmorLinkDispatchResult handleString(ArmorLinkConfigFieldDef& field, const String& value) {
    if (!field.stringBinding.ptr) {
      return ArmorLinkDispatchResult::InvalidValue;
    }

    *field.stringBinding.ptr = value;

    if (!_storage || !_storage->saveField(field)) {
      return ArmorLinkDispatchResult::StorageError;
    }

    if (field.onStringChange) {
      field.onStringChange(value);
    }

    return ArmorLinkDispatchResult::Ok;
  }
};
