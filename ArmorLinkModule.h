#pragma once

#include <Arduino.h>
#include <functional>
#include <vector>
#include "ArmorLinkProtocol.h"

enum class ArmorLinkModuleType {
  Generic,
  Chest,
  Helmet,
  Back,
  Arm,
  Hand,
  Leg,
  Prop
};

enum class ArmorLinkFieldKind {
  Int,
  Float,
  Bool,
  Readonly
};

enum class ArmorLinkActionStyle {
  Primary,
  Secondary,
  Danger
};

struct ArmorLinkIntBinding {
  int* ptr = nullptr;
};

struct ArmorLinkFloatBinding {
  float* ptr = nullptr;
};

struct ArmorLinkBoolBinding {
  bool* ptr = nullptr;
};

struct ArmorLinkReadonlyBinding {
  String value;
};

struct ArmorLinkConfigFieldDef {
  String key;
  String label;
  String section = "General";
  String description;
  String unit;

  ArmorLinkFieldKind kind = ArmorLinkFieldKind::Readonly;
  bool editable = false;
  bool advanced = false;

  String entity;
  String command;

  bool persistent = false;
  String nvsKey;

  int minInt = 0;
  int maxInt = 0;
  int stepInt = 1;
  bool hasIntRange = false;

  float minFloat = 0.0f;
  float maxFloat = 0.0f;
  float stepFloat = 0.1f;
  bool hasFloatRange = false;

  ArmorLinkFloatBinding floatBinding;

std::function<void(float)> onFloatChange;
  ArmorLinkIntBinding intBinding;
  ArmorLinkBoolBinding boolBinding;
  ArmorLinkReadonlyBinding readonlyBinding;

  std::function<void(int)> onIntChange;
  std::function<void(bool)> onBoolChange;
};

struct ArmorLinkActionDef {
  String id;
  String label;
  String description;
  String entity;
  String command;

  ArmorLinkActionStyle style = ArmorLinkActionStyle::Secondary;
  bool enabled = true;
  bool advanced = false;
  String confirmText;

  std::function<void()> callback;
};

class ArmorLinkConfigFieldBuilder {
public:
  explicit ArmorLinkConfigFieldBuilder(ArmorLinkConfigFieldDef& field)
      : _field(field) {}

  ArmorLinkConfigFieldBuilder& label(const String& value) {
    _field.label = value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& section(const String& value) {
    _field.section = value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& description(const String& value) {
    _field.description = value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& unit(const String& value) {
    _field.unit = value;
    return *this;
  }
  ArmorLinkConfigFieldBuilder& range(float minValue, float maxValue) {
    if (minValue > maxValue) {
      const float tmp = minValue;
      minValue = maxValue;
      maxValue = tmp;
    }

    _field.minFloat = minValue;
    _field.maxFloat = maxValue;
    _field.hasFloatRange = true;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& step(float value) {
    _field.stepFloat = value <= 0.0f ? 0.1f : value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& onFloatChange(std::function<void(float)> cb) {
    _field.onFloatChange = cb;
    return *this;
  }
  ArmorLinkConfigFieldBuilder& range(int minValue, int maxValue) {
    if (minValue > maxValue) {
      const int tmp = minValue;
      minValue = maxValue;
      maxValue = tmp;
    }

    _field.minInt = minValue;
    _field.maxInt = maxValue;
    _field.hasIntRange = true;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& step(int value) {
    _field.stepInt = value <= 0 ? 1 : value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& persist(const String& key) {
    _field.persistent = true;
    _field.nvsKey = key;
    return *this;
  }
  template <size_t EntityN, size_t CommandN>
  ArmorLinkConfigFieldBuilder& command(const char (&entity)[EntityN], const char (&command)[CommandN]) {
    static_assert(EntityN <= ARMORLINK_ENTITY_MAX_LEN + 1,
                  "ArmorLink config command entity max length is 23 characters.");
    static_assert(CommandN <= ARMORLINK_COMMAND_MAX_LEN + 1,
                  "ArmorLink config command name max length is 23 characters.");
    return this->command(String(entity), String(command));
  }
  ArmorLinkConfigFieldBuilder& command(const String& entity, const String& command) {
    _field.entity = entity;
    _field.command = command;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& readonly() {
    _field.editable = false;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& editable() {
    _field.editable = true;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& advanced() {
    _field.advanced = true;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& onIntChange(std::function<void(int)> cb) {
    _field.onIntChange = cb;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& onBoolChange(std::function<void(bool)> cb) {
    _field.onBoolChange = cb;
    return *this;
  }

private:
  ArmorLinkConfigFieldDef& _field;
};

class ArmorLinkActionBuilder {
public:
  explicit ArmorLinkActionBuilder(ArmorLinkActionDef& action)
      : _action(action) {}

  ArmorLinkActionBuilder& label(const String& value) {
    _action.label = value;
    return *this;
  }

  ArmorLinkActionBuilder& description(const String& value) {
    _action.description = value;
    return *this;
  }
  template <size_t EntityN, size_t CommandN>
  ArmorLinkActionBuilder& command(const char (&entity)[EntityN], const char (&command)[CommandN]) {
    static_assert(EntityN <= ARMORLINK_ENTITY_MAX_LEN + 1,
                  "ArmorLink command entity max length is 23 characters.");
    static_assert(CommandN <= ARMORLINK_COMMAND_MAX_LEN + 1,
                  "ArmorLink command name max length is 23 characters.");
    return this->command(String(entity), String(command));
  }
  ArmorLinkActionBuilder& command(const String& entity, const String& command) {
    _action.entity = entity;
    _action.command = command;
    return *this;
  }

  ArmorLinkActionBuilder& stylePrimary() {
    _action.style = ArmorLinkActionStyle::Primary;
    return *this;
  }

  ArmorLinkActionBuilder& styleSecondary() {
    _action.style = ArmorLinkActionStyle::Secondary;
    return *this;
  }

  ArmorLinkActionBuilder& styleDanger() {
    _action.style = ArmorLinkActionStyle::Danger;
    return *this;
  }

  ArmorLinkActionBuilder& enabled(bool value = true) {
    _action.enabled = value;
    return *this;
  }

  ArmorLinkActionBuilder& advanced() {
    _action.advanced = true;
    return *this;
  }

  ArmorLinkActionBuilder& confirm(const String& text) {
    _action.confirmText = text;
    return *this;
  }

  ArmorLinkActionBuilder& onExecute(std::function<void()> callback) {
    _action.callback = callback;
    return *this;
  }

private:
  ArmorLinkActionDef& _action;
};

class ArmorLinkConfigRegistry {
public:
  template <size_t KeyN>
  ArmorLinkConfigFieldBuilder addInt(const char (&key)[KeyN], int* binding, int defaultValue = 0) {
    static_assert(KeyN <= ARMORLINK_CONFIG_KEY_MAX_LEN + 1,
                  "ArmorLink config key max length is 23 characters. Use a shorter technical key and a longer .label(...).");
    return addInt(String(key), binding, defaultValue);
  }
  ArmorLinkConfigFieldBuilder addInt(const String& key, int* binding, int defaultValue = 0) {
    ArmorLinkConfigFieldDef field;
    field.key = key;
    field.label = key;
    field.kind = ArmorLinkFieldKind::Int;
    field.editable = true;
    field.persistent = true;
    field.nvsKey = key;
    field.entity = "config";
    field.command = key;
    field.intBinding.ptr = binding;

    if (binding) {
      *binding = defaultValue;
    }

    _fields.push_back(field);
    return ArmorLinkConfigFieldBuilder(_fields.back());
  }

  template <size_t KeyN>
  ArmorLinkConfigFieldBuilder addFloat(const char (&key)[KeyN], float* binding, float defaultValue = 0.0f) {
    static_assert(KeyN <= ARMORLINK_CONFIG_KEY_MAX_LEN + 1,
                  "ArmorLink config key max length is 23 characters. Use a shorter technical key and a longer .label(...).");
    return addFloat(String(key), binding, defaultValue);
  }

  ArmorLinkConfigFieldBuilder addFloat(const String& key, float* binding, float defaultValue = 0.0f) {
    ArmorLinkConfigFieldDef field;
    field.key = key;
    field.label = key;
    field.kind = ArmorLinkFieldKind::Float;
    field.editable = true;
    field.persistent = true;
    field.nvsKey = key;
    field.entity = "config";
    field.command = key;
    field.floatBinding.ptr = binding;

    if (binding) {
      *binding = defaultValue;
    }

    _fields.push_back(field);
    return ArmorLinkConfigFieldBuilder(_fields.back());
  }



  template <size_t KeyN>
  ArmorLinkConfigFieldBuilder addBool(const char (&key)[KeyN], bool* binding, bool defaultValue = false) {
    static_assert(KeyN <= ARMORLINK_CONFIG_KEY_MAX_LEN + 1,
                  "ArmorLink config key max length is 23 characters. Use a shorter technical key and a longer .label(...).");
    return addBool(String(key), binding, defaultValue);
  }
  ArmorLinkConfigFieldBuilder addBool(const String& key, bool* binding, bool defaultValue = false) {
    ArmorLinkConfigFieldDef field;
    field.key = key;
    field.label = key;
    field.kind = ArmorLinkFieldKind::Bool;
    field.editable = true;
    field.persistent = true;
    field.nvsKey = key;
    field.entity = "config";
    field.command = key;
    field.boolBinding.ptr = binding;

    if (binding) {
      *binding = defaultValue;
    }

    _fields.push_back(field);
    return ArmorLinkConfigFieldBuilder(_fields.back());
  }

  ArmorLinkConfigFieldBuilder addReadonly(const String& key, const String& value) {
    ArmorLinkConfigFieldDef field;
    field.key = key;
    field.label = key;
    field.kind = ArmorLinkFieldKind::Readonly;
    field.editable = false;
    field.readonlyBinding.value = value;

    _fields.push_back(field);
    return ArmorLinkConfigFieldBuilder(_fields.back());
  }

  std::vector<ArmorLinkConfigFieldDef>& items() { return _fields; }
  const std::vector<ArmorLinkConfigFieldDef>& items() const { return _fields; }

private:
  std::vector<ArmorLinkConfigFieldDef> _fields;
};

class ArmorLinkActionRegistry {
public:
  template <size_t IdN>
  ArmorLinkActionBuilder add(const char (&id)[IdN]) {
    static_assert(IdN <= ARMORLINK_COMMAND_MAX_LEN + 1,
                  "ArmorLink action id max length is 23 characters. Use a shorter id and a longer .label(...).");
    return add(String(id));
  }
  ArmorLinkActionBuilder add(const String& id) {
    ArmorLinkActionDef action;
    action.id = id;
    action.label = id;

    _actions.push_back(action);
    return ArmorLinkActionBuilder(_actions.back());
  }

  std::vector<ArmorLinkActionDef>& items() { return _actions; }
  const std::vector<ArmorLinkActionDef>& items() const { return _actions; }

private:
  std::vector<ArmorLinkActionDef> _actions;
};

class ArmorLinkModule {
public:
  ArmorLinkModule(const String& name, ArmorLinkModuleType type = ArmorLinkModuleType::Generic)
      : _name(name), _type(type) {}

  const String& name() const { return _name; }
  ArmorLinkModuleType type() const { return _type; }

  ArmorLinkConfigRegistry& config() { return _config; }
  const ArmorLinkConfigRegistry& config() const { return _config; }

  ArmorLinkActionRegistry& actions() { return _actions; }
  const ArmorLinkActionRegistry& actions() const { return _actions; }

private:
  String _name;
  ArmorLinkModuleType _type;
  ArmorLinkConfigRegistry _config;
  ArmorLinkActionRegistry _actions;
};