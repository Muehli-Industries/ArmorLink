#pragma once

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

        case ArmorLinkFieldKind::Bool:
          return handleBool(field, value != 0);

        case ArmorLinkFieldKind::Readonly:
        default:
          return ArmorLinkDispatchResult::NotEditable;
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
};