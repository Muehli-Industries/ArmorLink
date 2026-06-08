#pragma once

#ifndef ARMORLINK_VERBOSE
#define ARMORLINK_VERBOSE 0
#endif

#if ARMORLINK_VERBOSE
  #define AL_VERBOSE(...) do { \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while(0)

  #define AL_VERBOSELN(x) Serial.println(x)
#else
  #define AL_VERBOSE(...)
  #define AL_VERBOSELN(x)
#endif