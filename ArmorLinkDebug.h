#pragma once
#include "ArmorLinkFeatures.h"

#ifndef ARMORLINK_VERBOSE
#define ARMORLINK_VERBOSE 0
#endif

#if ARMORLINK_INTERNAL_LOG_LEVEL > 0 && (ARMORLINK_VERBOSE || ARMORLINK_INTERNAL_LOG_LEVEL >= 2)
  #define AL_VERBOSE(...) do { \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while(0)

  #define AL_VERBOSELN(x) Serial.println(x)
#else
  #define AL_VERBOSE(...)
  #define AL_VERBOSELN(x)
#endif

#if ARMORLINK_INTERNAL_LOG_LEVEL >= 1
#define AL_INTERNAL_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while (0)
#define AL_INTERNAL_PRINT(...) do { Serial.print(__VA_ARGS__); } while (0)
#define AL_INTERNAL_PRINTLN(...) do { Serial.println(__VA_ARGS__); } while (0)
#else
#define AL_INTERNAL_PRINTF(...) do {} while (0)
#define AL_INTERNAL_PRINT(...) do {} while (0)
#define AL_INTERNAL_PRINTLN(...) do {} while (0)
#endif
