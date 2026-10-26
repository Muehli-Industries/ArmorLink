#pragma once

#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include <functional>
#include <memory>
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

enum class ArmorLinkActionComponentKind : uint8_t {
  Servo,
  Led,
  Gpio,
  Button,
  State
};

enum class ArmorLinkActionOpCode : uint8_t {
  ServoOpen = 1,
  ServoClose = 2,
  LedOn = 3,
  LedOff = 4,
  Wait = 5,
  GpioHigh = 6,
  GpioLow = 7,
  ActionCall = 8,
  WaitUntilState = 9,
  RepeatPrevious = 10,
  StateSet = 11,
  SendActionToTarget = 13,
  ServoMoveTo = 14,
  LedSetColor = 15,
  LedSetBrightness = 16,
  LedAnimation = 17,
  LogMessage = 32,
  ServoAttach = 33,
  ServoDetach = 34
};

struct ArmorLinkActionLogMessage {
  ArmorLinkLogLevel level = AL_LOG_INFO;
  String text;
};

struct ArmorLinkActionOp {
  ArmorLinkActionOpCode code = ArmorLinkActionOpCode::Wait;
  uint8_t componentIndex = 0xFF;
  uint16_t argument = 0;
  uint16_t durationMs = 0;
  uint32_t color = 0xFFFFFF;
  uint8_t option = 0;
  uint8_t intensity = 255;
  bool value = true;
  std::shared_ptr<const ArmorLinkActionLogMessage> logMessage;
};

struct ArmorLinkActionDefinition {
  String entity;
  String command;
  String label;
  String section = "Actions";
  std::vector<ArmorLinkActionOp> ops;
  bool startup = false;
};

struct ArmorLinkActionTargetDef {
  String id;
  String label;
  String entity;
  String command;
};

struct ArmorLinkRemoteCommandDef {
  String id;
  String target;
  String entity;
  String command;
};

struct ArmorLinkButtonEventRouteDef {
  String event;
  String mode;
  bool enabled = true;
  String action;
  String target;
  String entity;
  String command;
};

using ArmorLinkActionComponentHandler =
    std::function<void(ArmorLinkActionOpCode op)>;

using ArmorLinkActionComponentArgumentHandler =
    std::function<void(ArmorLinkActionOpCode op, uint16_t argument)>;

using ArmorLinkActionComponentOperationHandler =
    std::function<void(const ArmorLinkActionOp& op)>;

using ArmorLinkActionReadyPredicate =
    std::function<bool()>;

using ArmorLinkRemoteActionHandler =
    std::function<bool(const String& target, const String& entity, const String& command)>;

struct ArmorLinkActionComponentDef {
  String id;
  String label;
  String section;
  ArmorLinkActionComponentKind kind = ArmorLinkActionComponentKind::Gpio;
  ArmorLinkActionComponentHandler handler;
  ArmorLinkActionComponentArgumentHandler argumentHandler;
  ArmorLinkActionComponentOperationHandler operationHandler;
  ArmorLinkActionReadyPredicate isReady;
  bool custom = false;
  bool ready = false;
  bool defaultValue = false;
  bool hasServoConfig = false;
  int servoPin = -1;
  int servoOpenPosition = 90;
  int servoClosedPosition = 0;
  int servoMinPulseUs = 500;
  int servoMaxPulseUs = 2400;
  String servoProfile;
  bool hasLedConfig = false;
  int ledPin = -1;
  int ledCount = 1;
  int ledStartIndex = 0;
  int ledBrightness = 128;
  String ledChipset = "WS2812B";
  String ledColorOrder = "GRB";
  bool hasGpioConfig = false;
  int gpioPin = -1;
  String gpioMode = "output";
  bool gpioDefaultLevel = false;
  bool gpioActiveLow = false;
  bool hasButtonConfig = false;
  int buttonPin = -1;
  bool buttonActiveLow = true;
  int buttonDebounceMs = 30;
  int buttonMultiGapMs = 300;
  int buttonLongPressMs = 500;
  std::vector<ArmorLinkButtonEventRouteDef> buttonEvents;
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

#if ARMORLINK_HAS_ACTIONBUILDER
class ArmorLinkActionComponentBuilder {
public:
  explicit ArmorLinkActionComponentBuilder(ArmorLinkActionComponentDef& component)
      : _component(component) {}

  ArmorLinkActionComponentBuilder& label(const String& value) {
    _component.label = value.isEmpty() ? _component.id : value;
    return *this;
  }

  ArmorLinkActionComponentBuilder& onExecute(ArmorLinkActionComponentHandler handler) {
    _component.handler = handler;
    return *this;
  }

  ArmorLinkActionComponentBuilder& onExecute(
      ArmorLinkActionComponentArgumentHandler handler) {
    _component.argumentHandler = handler;
    return *this;
  }

  ArmorLinkActionComponentBuilder& onExecute(
      ArmorLinkActionComponentOperationHandler handler) {
    _component.operationHandler = handler;
    return *this;
  }

  ArmorLinkActionComponentBuilder& onReady(ArmorLinkActionReadyPredicate predicate) {
    _component.isReady = predicate;
    return *this;
  }

private:
  ArmorLinkActionComponentDef& _component;
};

class ArmorLinkActionBuilderRuntime {
public:
  // Opt in only when the firmware hardware handler supports both operations.
  bool supportsServoAttachDetach = false;
  ArmorLinkActionComponentBuilder addComponent(
      const String& id,
      ArmorLinkActionComponentKind kind) {
    for (auto& component : _components) {
      if (component.id.equalsIgnoreCase(id)) {
        component.kind = kind;
        if (component.section.isEmpty()) {
          component.section = "Actions";
        }
        return ArmorLinkActionComponentBuilder(component);
      }
    }

    ArmorLinkActionComponentDef component;
    component.id = id;
    component.label = id;
    component.section = "Actions";
    component.kind = kind;
    _components.push_back(component);
    return ArmorLinkActionComponentBuilder(_components.back());
  }

  ArmorLinkActionComponentDef& upsertCustomComponent(
      const String& id,
      ArmorLinkActionComponentKind kind,
      const String& label = "",
      const String& section = "",
      bool defaultValue = false) {
    for (auto& component : _components) {
      if (component.id.equalsIgnoreCase(id)) {
        component.kind = kind;
        if (!label.isEmpty()) {
          component.label = label;
        }
        if (!section.isEmpty()) {
          component.section = section;
        }
        component.defaultValue = defaultValue;
        component.ready = defaultValue;
        component.custom = true;
        return component;
      }
    }

    ArmorLinkActionComponentDef component;
    component.id = id;
    component.label = label.isEmpty() ? id : label;
    component.section = section.isEmpty() ? String("Actions") : section;
    component.kind = kind;
    component.custom = true;
    component.defaultValue = defaultValue;
    component.ready = defaultValue;
    _components.push_back(component);
    return _components.back();
  }

  void setServoConfig(
      ArmorLinkActionComponentDef& component,
      int pin,
      int openPosition,
      int closedPosition,
      int minPulseUs,
      int maxPulseUs,
      const String& servoProfile = "") {
    component.hasServoConfig = true;
    component.servoPin = pin;
    component.servoOpenPosition = openPosition;
    component.servoClosedPosition = closedPosition;
    component.servoMinPulseUs = minPulseUs;
    component.servoMaxPulseUs = maxPulseUs;
    component.servoProfile = servoProfile;
  }

  void setLedConfig(
      ArmorLinkActionComponentDef& component,
      int pin,
      int count,
      int startIndex,
      int brightness,
      const String& chipset,
      const String& colorOrder) {
    component.hasLedConfig = true;
    component.ledPin = pin;
    component.ledCount = count;
    component.ledStartIndex = startIndex;
    component.ledBrightness = brightness;
    component.ledChipset = chipset.isEmpty() ? String("WS2812B") : chipset;
    component.ledColorOrder = colorOrder.isEmpty() ? String("GRB") : colorOrder;
  }

  void setGpioConfig(
      ArmorLinkActionComponentDef& component,
      int pin,
      const String& mode,
      bool defaultLevel,
      bool activeLow) {
    component.hasGpioConfig = true;
    component.gpioPin = pin;
    component.gpioMode = mode.equalsIgnoreCase("input") ? String("input") : String("output");
    component.gpioDefaultLevel = defaultLevel;
    component.gpioActiveLow = activeLow;
  }

  void setButtonConfig(
      ArmorLinkActionComponentDef& component,
      int pin,
      bool activeLow,
      int debounceMs,
      int multiGapMs,
      int longPressMs) {
    component.hasButtonConfig = true;
    component.buttonPin = pin;
    component.buttonActiveLow = activeLow;
    component.buttonDebounceMs = constrain(debounceMs, 1, 250);
    component.buttonMultiGapMs = constrain(multiGapMs, 50, 1500);
    component.buttonLongPressMs = constrain(longPressMs, 100, 5000);
  }

  void clearButtonEventRoutes(ArmorLinkActionComponentDef& component) {
    component.buttonEvents.clear();
  }

  void addButtonEventRoute(
      ArmorLinkActionComponentDef& component,
      const String& event,
      const String& mode,
      bool enabled,
      const String& action,
      const String& target,
      const String& entity,
      const String& command) {
    ArmorLinkButtonEventRouteDef route;
    route.event = event;
    route.mode = mode.equalsIgnoreCase("action") ? String("action") : String("sendCommand");
    route.enabled = enabled;
    route.action = action;
    route.target = target;
    route.entity = entity;
    route.command = command;
    component.buttonEvents.push_back(route);
  }

  ArmorLinkActionComponentBuilder addServo(const String& id) {
    return addComponent(id, ArmorLinkActionComponentKind::Servo);
  }

  ArmorLinkActionComponentBuilder addLed(const String& id) {
    return addComponent(id, ArmorLinkActionComponentKind::Led);
  }

  ArmorLinkActionComponentBuilder addGpio(const String& id) {
    return addComponent(id, ArmorLinkActionComponentKind::Gpio);
  }

  ArmorLinkActionComponentBuilder addButton(const String& id) {
    return addComponent(id, ArmorLinkActionComponentKind::Button);
  }

  ArmorLinkActionComponentBuilder addState(const String& id) {
    return addComponent(id, ArmorLinkActionComponentKind::State);
  }

  ArmorLinkActionComponentBuilder addWaitTarget(const String& id) {
    return addState(id);
  }

  ArmorLinkActionComponentBuilder addPreset(const String& id) {
    return addState(id);
  }

  ArmorLinkActionComponentBuilder addMode(const String& id) {
    return addState(id);
  }

  const std::vector<ArmorLinkActionComponentDef>& components() const {
    return _components;
  }

  bool removeCustomComponent(const String& id) {
    for (auto it = _components.begin(); it != _components.end(); ++it) {
      if (it->custom && it->id.equalsIgnoreCase(id)) {
        _components.erase(it);
        return true;
      }
    }

    return false;
  }

  const std::vector<ArmorLinkActionDef>& customActions() const {
    return _customActions;
  }

  const std::vector<ArmorLinkRemoteCommandDef>& remoteCommands() const {
    return _remoteCommands;
  }

  std::vector<ArmorLinkActionDef>& customActions() {
    return _customActions;
  }

  ArmorLinkRemoteCommandDef& upsertRemoteCommand(
      const String& target,
      const String& entity,
      const String& command) {
    const String id = remoteCommandId(target, entity, command);

    for (auto& remoteCommand : _remoteCommands) {
      if (remoteCommand.id.equalsIgnoreCase(id)) {
        remoteCommand.target = target;
        remoteCommand.entity = entity;
        remoteCommand.command = command;
        return remoteCommand;
      }
    }

    ArmorLinkRemoteCommandDef remoteCommand;
    remoteCommand.id = id;
    remoteCommand.target = target;
    remoteCommand.entity = entity;
    remoteCommand.command = command;
    _remoteCommands.push_back(remoteCommand);
    return _remoteCommands.back();
  }

  int findRemoteCommandIndex(const String& id) const {
    for (size_t i = 0; i < _remoteCommands.size(); ++i) {
      if (_remoteCommands[i].id.equalsIgnoreCase(id)) {
        return static_cast<int>(i);
      }
    }

    return -1;
  }

  using LogHandler = std::function<void(ArmorLinkLogLevel, const String&)>;
  void onLog(LogHandler handler) { _logHandler = handler; }

  // Templates are bounded in UTF-8 bytes and resolve state values at execution.
  bool renderLogMessage(const String& text, String& output, bool resolve = true) const {
    if (text.isEmpty() || text.length() > 128) return false;
    output = "";
    size_t cursor = 0;
    while (cursor < text.length()) {
      int start = text.indexOf("{state:", cursor);
      if (start < 0) { output += text.substring(cursor); break; }
      output += text.substring(cursor, start);
      int end = text.indexOf('}', start + 7);
      if (end < 0) return false;
      String id = text.substring(start + 7, end);
      int index = findComponentIndex(id);
      if (index < 0 || _components[index].kind != ArmorLinkActionComponentKind::State) return false;
      const auto& state = _components[index];
      bool value = resolve ? (state.isReady ? state.isReady() : state.ready) : false;
      output += value ? "true" : "false";
      cursor = end + 1;
    }
    return true;
  }

  void onRemoteAction(ArmorLinkRemoteActionHandler handler) {
    _remoteActionHandler = handler;
  }

  static String remoteCommandId(
      const String& target,
      const String& entity,
      const String& command) {
    return target + "|" + entity + "/" + command;
  }

  ArmorLinkActionDef* findCustomAction(
      const String& entity,
      const String& command) {
    for (auto& action : _customActions) {
      if (action.entity.equalsIgnoreCase(entity) &&
          action.command.equalsIgnoreCase(command)) {
        return &action;
      }
    }

    return nullptr;
  }

  ArmorLinkActionDef& upsertCustomAction(
      const String& entity,
      const String& command,
      const String& label,
      const String& section) {
    ArmorLinkActionDef* existing = findCustomAction(entity, command);
    if (existing != nullptr) {
      existing->label = label.isEmpty() ? command : label;
      existing->section = section.isEmpty() ? String("Actions") : section;
      existing->enabled = true;
      return *existing;
    }

    ArmorLinkActionDef action;
    action.id = entity + "." + command;
    action.label = label.isEmpty() ? command : label;
    action.section = section.isEmpty() ? String("Actions") : section;
    action.entity = entity;
    action.command = command;
    action.style = ArmorLinkActionStyle::Secondary;
    action.enabled = true;
    _customActions.push_back(action);
    return _customActions.back();
  }

  bool removeCustomAction(
      const String& entity,
      const String& command) {
    for (auto it = _customActions.begin(); it != _customActions.end(); ++it) {
      if (it->entity.equalsIgnoreCase(entity) &&
          it->command.equalsIgnoreCase(command)) {
        _customActions.erase(it);
        return true;
      }
    }

    return false;
  }

  int findComponentIndex(const String& id) const {
    for (size_t i = 0; i < _components.size(); ++i) {
      if (_components[i].id.equalsIgnoreCase(id)) {
        return static_cast<int>(i);
      }
    }

    return -1;
  }

  const ArmorLinkActionDefinition* findDefinition(
      const String& entity,
      const String& command) const {
    for (const auto& definition : _definitions) {
      if (definition.entity.equalsIgnoreCase(entity) &&
          definition.command.equalsIgnoreCase(command)) {
        return &definition;
      }
    }

    return nullptr;
  }

  const std::vector<ArmorLinkActionDefinition>& definitions() const {
    return _definitions;
  }

  std::vector<ArmorLinkActionTargetDef> actionTargets(
      const ArmorLinkActionRegistry& actions) const {
    std::vector<ArmorLinkActionTargetDef> targets;

    for (const auto& action : actions.items()) {
      if (!action.enabled) continue;
      ArmorLinkActionTargetDef target;
      target.id = action.entity + "/" + action.command;
      target.label = action.label;
      target.entity = action.entity;
      target.command = action.command;
      targets.push_back(target);
    }

    for (const auto& action : _customActions) {
      if (!action.enabled) continue;
      ArmorLinkActionTargetDef target;
      target.id = action.entity + "/" + action.command;
      target.label = action.label;
      target.entity = action.entity;
      target.command = action.command;
      targets.push_back(target);
    }

    return targets;
  }

  int findActionTargetIndex(
      const ArmorLinkActionRegistry& actions,
      const String& id) const {
    auto targets = actionTargets(actions);
    for (size_t i = 0; i < targets.size(); ++i) {
      if (targets[i].id.equalsIgnoreCase(id)) {
        return static_cast<int>(i);
      }
    }

    return -1;
  }

  const ArmorLinkActionDefinition* startupDefinition() const {
    for (const auto& definition : _definitions) if (definition.startup) return &definition;
    return nullptr;
  }

  void armStartup() { _startupPending = startupDefinition() != nullptr; }

  bool runStartupOnce(const ArmorLinkActionRegistry& actions) {
    if (!_startupPending || _running) return false;
    _startupPending = false;
    const auto* definition = startupDefinition();
    return definition != nullptr && executeDefinition(*definition, actions);
  }

  bool setDefinition(const ArmorLinkActionDefinition& definition) {
    if (definition.startup) {
      for (auto& existing : _definitions) {
        if (!existing.entity.equalsIgnoreCase(definition.entity) || !existing.command.equalsIgnoreCase(definition.command)) existing.startup = false;
      }
    }
    for (auto& existing : _definitions) {
      if (existing.entity.equalsIgnoreCase(definition.entity) &&
          existing.command.equalsIgnoreCase(definition.command)) {
        existing = definition;
        return true;
      }
    }

    _definitions.push_back(definition);
    return true;
  }

  bool removeDefinition(
      const String& entity,
      const String& command) {
    for (auto it = _definitions.begin(); it != _definitions.end(); ++it) {
      if (it->entity.equalsIgnoreCase(entity) &&
          it->command.equalsIgnoreCase(command)) {
        _definitions.erase(it);
        return true;
      }
    }

    return false;
  }

  bool wouldCreateCycle(
      const ArmorLinkActionRegistry& actions,
      const ArmorLinkActionDefinition& candidate) const {
    return reachesAction(actions, candidate, candidate.entity, candidate.command, 0);
  }

  bool execute(
      const String& entity,
      const String& command,
      const ArmorLinkActionRegistry& actions) {
    if (_running) {
      return false;
    }

    const ArmorLinkActionDefinition* definition =
        findDefinition(entity, command);

    if (definition == nullptr || definition->ops.empty()) {
      return false;
    }

    _activeOps.clear();
    _activeIndex = 0;
    clearWaitState();

    if (!appendDefinitionOps(*definition, actions, 0)) {
      _activeOps.clear();
      return false;
    }

    if (_activeOps.empty()) {
      return false;
    }

    _running = true;
    tick(actions);
    return true;
  }

  bool executeDefinition(
      const ArmorLinkActionDefinition& definition,
      const ArmorLinkActionRegistry& actions) {
    if (_running || definition.ops.empty()) return false;
    _activeOps.clear();
    _activeIndex = 0;
    clearWaitState();
    if (!appendDefinitionOpsTo(definition, actions, 0, _activeOps) || _activeOps.empty()) {
      _activeOps.clear();
      return false;
    }
    _running = true;
    tick(actions);
    return true;
  }

  bool isRunning() const {
    return _running;
  }

  void tick(const ArmorLinkActionRegistry& actions) {
    if (!_running) {
      return;
    }

    if (_waitingForDelay) {
      if (static_cast<uint32_t>(millis() - _waitStartedAtMs) < _waitDurationMs) {
        return;
      }

      _waitingForDelay = false;
      _activeIndex++;
    }

    if (_waitingForState) {
      if (_waitComponentIndex >= _components.size()) {
        stop();
        return;
      }

      auto& component = _components[_waitComponentIndex];
      const bool ready = component.isReady ? component.isReady() : component.ready;
      if (ready) {
        _waitingForState = false;
        _activeIndex++;
      } else if (static_cast<uint32_t>(millis() - _waitStartedAtMs) >= _waitDurationMs) {
        stop();
      }

      return;
    }

    while (_running && _activeIndex < _activeOps.size()) {
      const ArmorLinkActionOp& op = _activeOps[_activeIndex];

      if (op.code == ArmorLinkActionOpCode::Wait) {
        _waitStartedAtMs = millis();
        _waitDurationMs = op.argument;
        _waitingForDelay = true;
        if (_waitDurationMs == 0) {
          _waitingForDelay = false;
          _activeIndex++;
          continue;
        }
        return;
      }

      if (op.code == ArmorLinkActionOpCode::WaitUntilState) {
        if (op.componentIndex >= _components.size()) {
          stop();
          return;
        }

        auto& component = _components[op.componentIndex];
        const bool ready = component.isReady ? component.isReady() : component.ready;
        if (ready) {
          _activeIndex++;
          continue;
        }

        _waitComponentIndex = op.componentIndex;
        _waitStartedAtMs = millis();
        _waitDurationMs = op.argument == 0 ? 5000 : op.argument;
        _waitingForState = true;
        return;
      }

      if (!executeImmediateOp(op, actions)) {
        stop();
        return;
      }

      if (op.code == ArmorLinkActionOpCode::ServoMoveTo &&
          op.durationMs > 0 && op.componentIndex < _components.size()) {
        _waitComponentIndex = op.componentIndex;
        _waitStartedAtMs = millis();
        _waitDurationMs = op.durationMs + 2000;
        _waitingForState = true;
        return;
      }

      _activeIndex++;
    }

    if (_running && _activeIndex >= _activeOps.size()) {
      stop();
    }
  }

  void stop() {
    _running = false;
    _activeOps.clear();
    _activeIndex = 0;
    clearWaitState();
  }

private:
  static constexpr size_t MaxActiveOps = 256;

  void clearWaitState() {
    _waitingForDelay = false;
    _waitingForState = false;
    _waitStartedAtMs = 0;
    _waitDurationMs = 0;
    _waitComponentIndex = 0xFF;
  }

  bool appendDefinitionOps(
      const ArmorLinkActionDefinition& definition,
      const ArmorLinkActionRegistry& actions,
      uint8_t depth) {
    return appendDefinitionOpsTo(definition, actions, depth, _activeOps);
  }

  bool appendDefinitionOpsTo(
      const ArmorLinkActionDefinition& definition,
      const ArmorLinkActionRegistry& actions,
      uint8_t depth,
      std::vector<ArmorLinkActionOp>& targetOps) {
    if (depth > 8) {
      return false;
    }

    std::vector<ArmorLinkActionOp> previousGroup;

    for (const auto& op : definition.ops) {
      if (op.code == ArmorLinkActionOpCode::RepeatPrevious) {
        if (previousGroup.empty()) {
          return false;
        }

        const uint16_t repeatCount = min<uint16_t>(op.argument, 20);
        for (uint16_t i = 0; i < repeatCount; ++i) {
          if (!appendOpsTo(previousGroup, targetOps)) {
            return false;
          }
        }
        continue;
      }

      std::vector<ArmorLinkActionOp> group;
      if (!buildOpGroup(op, actions, depth, group) ||
          group.empty() ||
          !appendOpsTo(group, targetOps)) {
        return false;
      }

      previousGroup = group;
    }

    return true;
  }

  bool buildOpGroup(
      const ArmorLinkActionOp& op,
      const ArmorLinkActionRegistry& actions,
      uint8_t depth,
      std::vector<ArmorLinkActionOp>& out) {
    if (op.code == ArmorLinkActionOpCode::RepeatPrevious) {
      return false;
    }

    if (op.code != ArmorLinkActionOpCode::ActionCall) {
      out.push_back(op);
      return true;
    }

    auto targets = actionTargets(actions);
    if (op.componentIndex >= targets.size()) {
      return false;
    }

    const auto& target = targets[op.componentIndex];
    const ArmorLinkActionDefinition* nested =
        findDefinition(target.entity, target.command);

    if (nested == nullptr) {
      out.push_back(op);
      return true;
    }

    const size_t before = out.size();
    if (!appendDefinitionOpsTo(*nested, actions, depth + 1, out)) {
      return false;
    }

    return out.size() > before;
  }

  static bool appendOpsTo(
      const std::vector<ArmorLinkActionOp>& source,
      std::vector<ArmorLinkActionOp>& target) {
    if (target.size() + source.size() > MaxActiveOps) {
      return false;
    }

    target.insert(target.end(), source.begin(), source.end());
    return true;
  }

  bool executeImmediateOp(
      const ArmorLinkActionOp& op,
      const ArmorLinkActionRegistry& actions) {
    if (op.code == ArmorLinkActionOpCode::Wait ||
        op.code == ArmorLinkActionOpCode::WaitUntilState ||
        op.code == ArmorLinkActionOpCode::RepeatPrevious) {
      return false;
    }

    if (op.code == ArmorLinkActionOpCode::LogMessage) {
      if (!_logHandler || !op.logMessage || op.logMessage->level > AL_LOG_ERROR) return false;
      String message;
      if (!renderLogMessage(op.logMessage->text, message)) return false;
      _logHandler(op.logMessage->level, message);
      return true;
    }

    if (op.code == ArmorLinkActionOpCode::SendActionToTarget) {
      if (!_remoteActionHandler ||
          op.argument >= _remoteCommands.size()) {
        return false;
      }

      const auto& remoteCommand = _remoteCommands[op.argument];
      return _remoteActionHandler(
          remoteCommand.target,
          remoteCommand.entity,
          remoteCommand.command);
    }

    if (op.code == ArmorLinkActionOpCode::ActionCall) {
      auto targets = actionTargets(actions);
      if (op.componentIndex >= targets.size()) {
        return false;
      }

      const auto& target = targets[op.componentIndex];
      for (const auto& action : actions.items()) {
        if (action.entity.equalsIgnoreCase(target.entity) &&
            action.command.equalsIgnoreCase(target.command) &&
            action.callback) {
          action.callback();
          return true;
        }
      }

      return false;
    }

    if (op.componentIndex >= _components.size()) {
      return false;
    }

    auto& component = _components[op.componentIndex];
    if ((op.code == ArmorLinkActionOpCode::ServoAttach || op.code == ArmorLinkActionOpCode::ServoDetach) &&
        (!supportsServoAttachDetach || component.kind != ArmorLinkActionComponentKind::Servo)) return false;

    if (!component.handler && !component.argumentHandler && !component.operationHandler) {
      if (op.code == ArmorLinkActionOpCode::StateSet &&
          component.kind == ArmorLinkActionComponentKind::State) {
        component.ready = op.value;
        return true;
      }

      return false;
    }

    if (component.operationHandler) {
      component.operationHandler(op);
    } else if (component.argumentHandler) {
      component.argumentHandler(op.code, op.argument);
    } else {
      component.handler(op.code);
    }
    if (op.code == ArmorLinkActionOpCode::StateSet &&
        component.kind == ArmorLinkActionComponentKind::State) {
      component.ready = op.value;
    }
    return true;
  }

  const ArmorLinkActionDefinition* findEffectiveDefinition(
      const ArmorLinkActionDefinition& candidate,
      const String& entity,
      const String& command) const {
    if (candidate.entity.equalsIgnoreCase(entity) &&
        candidate.command.equalsIgnoreCase(command)) {
      return &candidate;
    }

    return findDefinition(entity, command);
  }

  bool reachesAction(
      const ArmorLinkActionRegistry& actions,
      const ArmorLinkActionDefinition& candidate,
      const String& entity,
      const String& command,
      uint8_t depth) const {
    if (depth > 8) {
      return true;
    }

    const ArmorLinkActionDefinition* definition =
        findEffectiveDefinition(candidate, entity, command);

    if (definition == nullptr) {
      return false;
    }

    auto targets = actionTargets(actions);
    for (const auto& op : definition->ops) {
      if (op.code != ArmorLinkActionOpCode::ActionCall ||
          op.componentIndex >= targets.size()) {
        continue;
      }

      const auto& target = targets[op.componentIndex];
      if (target.entity.equalsIgnoreCase(candidate.entity) &&
          target.command.equalsIgnoreCase(candidate.command)) {
        return true;
      }

      if (reachesAction(actions, candidate, target.entity, target.command, depth + 1)) {
        return true;
      }
    }

    return false;
  }

  std::vector<ArmorLinkActionComponentDef> _components;
  std::vector<ArmorLinkActionDef> _customActions;
  std::vector<ArmorLinkActionDefinition> _definitions;
  std::vector<ArmorLinkRemoteCommandDef> _remoteCommands;
  bool _startupPending = false;
  LogHandler _logHandler;
  std::vector<ArmorLinkActionOp> _activeOps;
  size_t _activeIndex = 0;
  uint32_t _waitStartedAtMs = 0;
  uint32_t _waitDurationMs = 0;
  uint8_t _waitComponentIndex = 0xFF;
  bool _running = false;
  bool _waitingForDelay = false;
  bool _waitingForState = false;
  ArmorLinkRemoteActionHandler _remoteActionHandler;
};

#endif // ARMORLINK_HAS_ACTIONBUILDER

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
  const String& profileIdentifier() const { return _profileIdentifier; }
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

  ArmorLinkModule& profileIdentifier(const String& value) {
    _profileIdentifier = value;
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

  ArmorLinkModule& activeProfile(
      const String& identifier,
      const String& name) {
    _profileIdentifier = identifier;
    _profileName = name;
    _profileNameImported = true;
    return *this;
  }

  ArmorLinkConfigRegistry& config() { return _config; }
  const ArmorLinkConfigRegistry& config() const { return _config; }

  ArmorLinkActionRegistry& actions() { return _actions; }
  const ArmorLinkActionRegistry& actions() const { return _actions; }

#if ARMORLINK_HAS_ACTIONBUILDER
  ArmorLinkActionBuilderRuntime& actionBuilder() { return _actionBuilder; }
  const ArmorLinkActionBuilderRuntime& actionBuilder() const { return _actionBuilder; }

#endif

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
  String _profileIdentifier;
  String _profileName;
  String _defaultProfileName;
  bool _profileNameImported = false;
  ArmorLinkConfigRegistry _config;
  ArmorLinkActionRegistry _actions;
#if ARMORLINK_HAS_ACTIONBUILDER
  ArmorLinkActionBuilderRuntime _actionBuilder;
#endif
  ArmorLinkSerialRegistry _serial;
};
