#pragma once

#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include <functional>
#include <vector>
#include "ArmorLinkProtocol.h"
#include "ArmorLinkSerial.h"

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
  String,
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

struct ArmorLinkStringBinding {
  String* ptr = nullptr;
};

struct ArmorLinkReadonlyBinding {
  String value;
};

struct ArmorLinkServoConfig {
  int pin = -1;
  int openPosition = 90;
  int closedPosition = 90;
  int minPulseUs = 500;
  int maxPulseUs = 2400;
};

struct ArmorLinkRegisteredServoConfig {
  String id;
  ArmorLinkServoConfig* servo = nullptr;
};

struct ArmorLinkVisibleWhenDef {
  String key;
  String value;
  bool enabled = false;
};

struct ArmorLinkConfigFieldDef {
  String key;
  String label;
  String section = "General";
  String description;
  const char* tooltip = nullptr;
  String unit;
  String semantic;
  String semanticGroup;

  ArmorLinkFieldKind kind = ArmorLinkFieldKind::Readonly;
  bool editable = false;
  bool advanced = false;
  bool rebootRequired = false;

  String entity;
  String command;
  ArmorLinkVisibleWhenDef visibleWhen;

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
  ArmorLinkStringBinding stringBinding;
  ArmorLinkReadonlyBinding readonlyBinding;

  std::function<void(int)> onIntChange;
  std::function<void(bool)> onBoolChange;
  std::function<void(const String&)> onStringChange;
};

struct ArmorLinkActionDef {
  String id;
  String label;
  String section = "General";
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

  ArmorLinkConfigFieldBuilder& tooltip(const char* value) {
    _field.tooltip = value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& visibleWhen(
      const String& key,
      const String& value) {
    _field.visibleWhen.key = key;
    _field.visibleWhen.value = value;
    _field.visibleWhen.enabled = true;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& visibleWhen(
      const String& key,
      const char* value) {
    return visibleWhen(key, String(value ? value : ""));
  }

  ArmorLinkConfigFieldBuilder& visibleWhen(
      const String& key,
      int value) {
    return visibleWhen(key, String(value));
  }

  ArmorLinkConfigFieldBuilder& visibleWhen(
      const String& key,
      bool value) {
    return visibleWhen(
        key,
        String(value ? "true" : "false"));
  }

  ArmorLinkConfigFieldBuilder& unit(const String& value) {
    _field.unit = value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& semantic(const String& value) {
    _field.semantic = value;
    return *this;
  }

  ArmorLinkConfigFieldBuilder& semanticGroup(const String& value) {
    _field.semanticGroup = value;
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

  ArmorLinkConfigFieldBuilder& rebootRequired(bool value = true) {
    _field.rebootRequired = value;
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

  ArmorLinkConfigFieldBuilder& onStringChange(std::function<void(const String&)> cb) {
    _field.onStringChange = cb;
    return *this;
  }

private:
  ArmorLinkConfigFieldDef& _field;
};

class ArmorLinkConfigRegistry;

class ArmorLinkServoConfigBuilder {
public:
  ArmorLinkServoConfigBuilder(
      ArmorLinkConfigRegistry& registry,
      const String& id,
      ArmorLinkServoConfig* servo)
      : _registry(registry),
        _id(id),
        _servo(servo),
        _section(id) {}

  ArmorLinkServoConfigBuilder& section(const String& value);
  ArmorLinkServoConfigBuilder& visibleWhen(const String& key, const String& value);
  ArmorLinkServoConfigBuilder& visibleWhen(const String& key, const char* value);
  ArmorLinkServoConfigBuilder& visibleWhen(const String& key, int value);
  ArmorLinkServoConfigBuilder& visibleWhen(const String& key, bool value);
  ArmorLinkServoConfigBuilder& gpio(int defaultValue);
  ArmorLinkServoConfigBuilder& openPosition(int defaultValue);
  ArmorLinkServoConfigBuilder& closedPosition(int defaultValue);
  ArmorLinkServoConfigBuilder& pulseRange(int minPulseUs, int maxPulseUs);

private:
  void applyVisibleWhen(ArmorLinkConfigFieldBuilder& field);

  ArmorLinkConfigRegistry& _registry;
  String _id;
  ArmorLinkServoConfig* _servo;
  String _section;
  ArmorLinkVisibleWhenDef _visibleWhen;
};

class ArmorLinkActionBuilder {
public:
  explicit ArmorLinkActionBuilder(ArmorLinkActionDef& action)
      : _action(action) {}

  ArmorLinkActionBuilder& label(const String& value) {
    _action.label = value;
    return *this;
  }

  ArmorLinkActionBuilder& section(const String& value) {
    _action.section = value.isEmpty() ? "General" : value;
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

  template <size_t KeyN>
  ArmorLinkConfigFieldBuilder addString(const char (&key)[KeyN], String* binding, const String& defaultValue = "") {
    static_assert(KeyN <= ARMORLINK_CONFIG_KEY_MAX_LEN + 1,
                  "ArmorLink config key max length is 23 characters. Use a shorter technical key and a longer .label(...).");
    return addString(String(key), binding, defaultValue);
  }

  ArmorLinkConfigFieldBuilder addString(const String& key, String* binding, const String& defaultValue = "") {
    ArmorLinkConfigFieldDef field;
    field.key = key;
    field.label = key;
    field.kind = ArmorLinkFieldKind::String;
    field.editable = true;
    field.persistent = true;
    field.nvsKey = key;
    field.entity = "config";
    field.command = key;
    field.stringBinding.ptr = binding;

    if (binding) {
      *binding = defaultValue;
    }

    _fields.push_back(field);
    return ArmorLinkConfigFieldBuilder(_fields.back());
  }

  ArmorLinkServoConfigBuilder addServoConfig(
      const String& id,
      ArmorLinkServoConfig* servo) {
    registerServoConfig(id, servo);
    return ArmorLinkServoConfigBuilder(*this, id, servo);
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

  std::vector<ArmorLinkRegisteredServoConfig>& servoConfigs() { return _servos; }
  const std::vector<ArmorLinkRegisteredServoConfig>& servoConfigs() const { return _servos; }

  ArmorLinkServoConfig* findServoConfig(const String& id) const {
    for (const auto& item : _servos) {
      if (item.id.equalsIgnoreCase(id)) {
        return item.servo;
      }
    }

    return nullptr;
  }

  bool containsKey(const String& key) const {
    for (const auto& field : _fields) {
      if (field.key.equalsIgnoreCase(key)) {
        return true;
      }
    }

    return false;
  }

private:
  void registerServoConfig(
      const String& id,
      ArmorLinkServoConfig* servo) {
    for (auto& item : _servos) {
      if (item.id.equalsIgnoreCase(id)) {
        item.servo = servo;
        return;
      }
    }

    ArmorLinkRegisteredServoConfig registeredServo;
    registeredServo.id = id;
    registeredServo.servo = servo;
    _servos.push_back(registeredServo);
  }

  std::vector<ArmorLinkConfigFieldDef> _fields;
  std::vector<ArmorLinkRegisteredServoConfig> _servos;
};

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::section(
    const String& value) {
  _section = value.isEmpty() ? _id : value;
  return *this;
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::visibleWhen(
    const String& key,
    const String& value) {
  _visibleWhen.key = key;
  _visibleWhen.value = value;
  _visibleWhen.enabled = true;
  return *this;
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::visibleWhen(
    const String& key,
    const char* value) {
  return visibleWhen(key, String(value ? value : ""));
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::visibleWhen(
    const String& key,
    int value) {
  return visibleWhen(key, String(value));
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::visibleWhen(
    const String& key,
    bool value) {
  return visibleWhen(key, String(value ? "true" : "false"));
}

inline void ArmorLinkServoConfigBuilder::applyVisibleWhen(
    ArmorLinkConfigFieldBuilder& field) {
  if (_visibleWhen.enabled) {
    field.visibleWhen(_visibleWhen.key, _visibleWhen.value);
  }
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::gpio(
    int defaultValue) {
  auto field =
      _registry.addInt(_id + "Pin", _servo ? &_servo->pin : nullptr, defaultValue);

  field.label("GPIO")
      .section(_section)
      .tooltip("The ESP32 pin connected to this servo.")
      .semantic("servo.gpio")
      .semanticGroup(_id)
      .rebootRequired();

  applyVisibleWhen(field);

  field.range(-1, 48)
      .step(1);

  return *this;
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::openPosition(
    int defaultValue) {
  auto field =
      _registry.addInt(
          _id + "Open", _servo ? &_servo->openPosition : nullptr, defaultValue);

  field.label("Open Position")
      .section(_section)
      .tooltip("Servo angle when the mechanism is open.")
      .semantic("servo.openPosition")
      .semanticGroup(_id);

  applyVisibleWhen(field);

  field.range(0, 180)
      .step(1);

  return *this;
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::closedPosition(
    int defaultValue) {
  auto field =
      _registry.addInt(
          _id + "Closed", _servo ? &_servo->closedPosition : nullptr, defaultValue);

  field.label("Closed Position")
      .section(_section)
      .tooltip("Servo angle when the mechanism is closed.")
      .semantic("servo.closedPosition")
      .semanticGroup(_id);

  applyVisibleWhen(field);

  field.range(0, 180)
      .step(1);

  return *this;
}

inline ArmorLinkServoConfigBuilder& ArmorLinkServoConfigBuilder::pulseRange(
    int minPulseUs,
    int maxPulseUs) {
  auto minField =
      _registry.addInt(
          _id + "MinPulseUs", _servo ? &_servo->minPulseUs : nullptr, minPulseUs);

  minField.label("Minimum Pulse")
      .section(_section)
      .tooltip("Lowest signal pulse used by this servo.")
      .semantic("servo.minPulse")
      .semanticGroup(_id)
      .rebootRequired();

  applyVisibleWhen(minField);

  minField.range(400, 1500)
      .step(10);

  auto maxField =
      _registry.addInt(
          _id + "MaxPulseUs", _servo ? &_servo->maxPulseUs : nullptr, maxPulseUs);

  maxField.label("Maximum Pulse")
      .section(_section)
      .tooltip("Highest signal pulse used by this servo.")
      .semantic("servo.maxPulse")
      .semanticGroup(_id)
      .rebootRequired();

  applyVisibleWhen(maxField);

  maxField.range(1500, 3000)
      .step(10);

  return *this;
}

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
  template <size_t NameN>
  ArmorLinkModule(const char (&name)[NameN],
                  ArmorLinkModuleType type = ArmorLinkModuleType::Generic,
                  const String& version = "1.0")
      : _name(name),
        _type(type),
        _version(version.isEmpty() ? "1.0" : version) {
    static_assert(NameN <= 16,
                  "ArmorLink module name max length is 15 characters. Use a shorter technical name.");
  }

  ArmorLinkModule(const String& name,
                  ArmorLinkModuleType type = ArmorLinkModuleType::Generic,
                  const String& version = "1.0")
      : _name(name),
        _type(type),
        _version(version.isEmpty() ? "1.0" : version) {
    clampName();
  }

  const String& name() const { return _name; }
  ArmorLinkModuleType type() const { return _type; }
  const String& version() const { return _version; }
  const String& profileTarget() const { return _profileTarget; }
  const String& profileName() const { return _profileName; }
  const String& defaultProfileName() const { return _defaultProfileName; }
  bool profileNameImported() const { return _profileNameImported; }

  ArmorLinkModule& name(const String& value) {
    if (value.isEmpty()) {
      return *this;
    }

    _name = value;
    clampName();

    return *this;
  }

  ArmorLinkModule& version(const String& value) {
    _version = value.isEmpty() ? "1.0" : value;
    return *this;
  }

  ArmorLinkModule& profileTarget(const String& value) {
    _profileTarget = value;
    return *this;
  }

  ArmorLinkModule& profileName(const String& value) {
    if (_defaultProfileName.isEmpty()) {
      _defaultProfileName = value;
    }

    _profileName = value;
    return *this;
  }

  ArmorLinkModule& activeProfileName(const String& value) {
    _profileName = value;
    _profileNameImported = true;
    return *this;
  }

  ArmorLinkConfigRegistry& config() { return _config; }
  const ArmorLinkConfigRegistry& config() const { return _config; }

  ArmorLinkActionRegistry& actions() { return _actions; }
  const ArmorLinkActionRegistry& actions() const { return _actions; }

  ArmorLinkSerialRegistry& serial() {
      return _serial;
  }

  const ArmorLinkSerialRegistry& serial() const {
      return _serial;
  }

private:
  void clampName() {
    if (_name.length() > ARMORLINK_NAME_MAX_LEN) {
      _name.remove(ARMORLINK_NAME_MAX_LEN);
    }
  }

  String _name;
  ArmorLinkModuleType _type;
  String _version = "1.0";
  String _profileTarget;
  String _profileName;
  String _defaultProfileName;
  bool _profileNameImported = false;
  ArmorLinkConfigRegistry _config;
  ArmorLinkActionRegistry _actions;
  ArmorLinkSerialRegistry _serial;
};
