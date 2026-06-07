#pragma once
#ifndef ARMORLINK_SECURITY_H
#define ARMORLINK_SECURITY_H

#include <Arduino.h>
#include <Preferences.h>

namespace ArmorLinkSecurity {

  inline Preferences& prefs() {
    static Preferences instance;
    return instance;
  }

  inline bool& initialized() {
    static bool value = false;
    return value;
  }
  static constexpr const char* KEY_PROJECT_NAME = "project_name";
  static constexpr const char* NAMESPACE_NAME = "armor_sec";
  static constexpr const char* KEY_PIN = "ble_pin";

  inline void begin() {
    if (!initialized()) {
      prefs().begin(NAMESPACE_NAME, false);
      initialized() = true;
    }
  }

  inline String getProjectName() {
    begin();

    String value =
        prefs().getString(KEY_PROJECT_NAME,
                          "ArmorLink Project");
    

    return value;
  }

  inline bool saveProjectName(const String& name) {
    begin();

    String trimmed = name;
    trimmed.trim();

    if (trimmed.isEmpty()) {
      trimmed = "ArmorLink Project";
    }

    if (trimmed.length() > 32) {
      trimmed = trimmed.substring(0, 32);
    }    
    return prefs().putString(KEY_PROJECT_NAME, trimmed) > 0;
  }

  inline bool isPinSet() {
    begin();
    return prefs().isKey(KEY_PIN);
  }

  inline uint32_t getPin() {
    begin();
    return prefs().getUInt(KEY_PIN, 0);
  }

  inline bool savePin(uint32_t pin) {
    begin();
    return prefs().putUInt(KEY_PIN, pin) > 0;
  }

  inline void clearPin() {
    begin();
    prefs().remove(KEY_PIN);
  }
}

#endif