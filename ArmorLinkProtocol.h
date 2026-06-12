#pragma once
#ifndef ARMORLINK_PROTOCOL_H
#define ARMORLINK_PROTOCOL_H

#include "ArmorLinkDebug.h"
#include <Arduino.h>

#ifndef ARMORLINK_VERSION
#define ARMORLINK_VERSION "0.3.0"
#endif

static constexpr uint8_t  ARMORLINK_PROTOCOL_VERSION = 2;
static constexpr uint16_t ARMORLINK_CHUNK_DATA_SIZE  = 140;
static constexpr uint16_t ARMORLINK_MAX_CHUNKS       = 64;

static constexpr size_t ARMORLINK_NAME_MAX_LEN       = 15;
static constexpr size_t ARMORLINK_ENTITY_MAX_LEN     = 23;
static constexpr size_t ARMORLINK_COMMAND_MAX_LEN    = 23;
static constexpr size_t ARMORLINK_CONFIG_KEY_MAX_LEN = 23;

enum ArmorLinkMessageType : uint8_t {
  AL_MSG_UNKNOWN       = 0,
  AL_MSG_COMMAND       = 1,
  AL_MSG_ACK           = 2,
  AL_MSG_LOG           = 3,
  AL_MSG_HEARTBEAT     = 4,
  AL_MSG_CONFIG_GET    = 5,
  AL_MSG_CONFIG_SET    = 6,
  AL_MSG_CONFIG_META   = 7,
  AL_MSG_CONFIG_CHUNK  = 8,
  AL_MSG_CONFIG_END    = 9,
  AL_MSG_ERROR         = 10,
  AL_MSG_TELEMETRY     = 30,
  // Pairing
  AL_MSG_PAIR_ANNOUNCE = 20,
  AL_MSG_PAIR_RESPONSE = 21,
  AL_MSG_PAIR_ACCEPT   = 22,
  AL_MSG_PAIR_REJECT   = 23

};

enum ArmorLinkLogLevel : uint8_t {
  AL_LOG_DEBUG = 0,
  AL_LOG_INFO  = 1,
  AL_LOG_WARN  = 2,
  AL_LOG_ERROR = 3
};

enum ArmorLinkPacketFlags : uint8_t {
  AL_FLAG_NONE        = 0,
  AL_FLAG_PARTIAL_CFG = 1 << 0
};

struct ArmorLinkPacket {
  uint8_t  version;
  uint8_t  msgType;
  uint8_t  level;
  uint8_t  flags;

  uint16_t seq;
  uint16_t requestId;

  uint16_t chunkIndex;
  uint16_t chunkCount;
  uint16_t payloadLen;

  int32_t  valueInt;
  float    valueFloat;
  float    batteryVoltage;

  char source[ARMORLINK_NAME_MAX_LEN + 1];
  char target[ARMORLINK_NAME_MAX_LEN + 1];
  char entity[ARMORLINK_ENTITY_MAX_LEN + 1];
  char command[ARMORLINK_COMMAND_MAX_LEN + 1];

  uint8_t payload[ARMORLINK_CHUNK_DATA_SIZE];
};

struct ArmorLinkLegacyPacket {
  char source[12];
  char target[12];
  char entity[24];
  char command[24];
  char valueStr[16];
  int valueInt;
  float batteryVoltage;
};

static_assert(sizeof(ArmorLinkPacket) <= 250, "ArmorLinkPacket too large for ESP-NOW");

inline void armorlinkCopyString(char* dst, size_t dstSize, const char* src) {
  if (!dst || dstSize == 0) return;
  memset(dst, 0, dstSize);
  if (!src) return;
  strncpy(dst, src, dstSize - 1);
}

inline void armorlinkCopyString(char* dst, size_t dstSize, const String& src) {
  armorlinkCopyString(dst, dstSize, src.c_str());
}

inline void clearArmorLinkPacket(ArmorLinkPacket& p) {
  memset(&p, 0, sizeof(ArmorLinkPacket));
  p.version = ARMORLINK_PROTOCOL_VERSION;
}

inline const char* armorLinkLogLevelToString(uint8_t level) {
  switch (level) {
    case AL_LOG_DEBUG: return "DEBUG";
    case AL_LOG_INFO:  return "INFO";
    case AL_LOG_WARN:  return "WARN";
    case AL_LOG_ERROR: return "ERROR";
    default:           return "UNKNOWN";
  }
}

inline ArmorLinkLogLevel armorLinkLogLevelFromString(const String& s) {
  if (s.equalsIgnoreCase("DEBUG")) return AL_LOG_DEBUG;
  if (s.equalsIgnoreCase("INFO"))  return AL_LOG_INFO;
  if (s.equalsIgnoreCase("WARN"))  return AL_LOG_WARN;
  if (s.equalsIgnoreCase("ERROR")) return AL_LOG_ERROR;
  return AL_LOG_INFO;
}

inline String armorLinkPacketPayloadToString(const ArmorLinkPacket& p) {
  if (p.payloadLen == 0) return "";
  String out;
  out.reserve(p.payloadLen);
  for (uint16_t i = 0; i < p.payloadLen && i < ARMORLINK_CHUNK_DATA_SIZE; i++) {
    out += (char)p.payload[i];
  }
  return out;
}

inline void setArmorLinkPacketPayload(ArmorLinkPacket& p, const String& text) {
  memset(p.payload, 0, sizeof(p.payload));
  p.payloadLen = min((size_t)ARMORLINK_CHUNK_DATA_SIZE, text.length());
  memcpy(p.payload, text.c_str(), p.payloadLen);
}

inline uint16_t nextArmorLinkPacketSeq() {
  static uint16_t seq = 0;
  seq++;
  if (seq == 0) seq = 1;
  return seq;
}

inline ArmorLinkPacket makeArmorLinkBasePacket(
  ArmorLinkMessageType type,
  const char* source,
  const char* target,
  const char* entity,
  const char* command
) {
  ArmorLinkPacket p{};
  clearArmorLinkPacket(p);
  p.msgType = type;
  p.seq = nextArmorLinkPacketSeq();
  armorlinkCopyString(p.source, sizeof(p.source), source);
  armorlinkCopyString(p.target, sizeof(p.target), target);
  armorlinkCopyString(p.entity, sizeof(p.entity), entity);
  armorlinkCopyString(p.command, sizeof(p.command), command);
  return p;
}

inline bool convertArmorLinkLegacyToPacket(const ArmorLinkLegacyPacket& oldMsg, ArmorLinkPacket& out) {
  clearArmorLinkPacket(out);
  out.msgType = AL_MSG_COMMAND;
  out.level = AL_LOG_INFO;
  out.seq = nextArmorLinkPacketSeq();
  out.valueInt = oldMsg.valueInt;
  out.batteryVoltage = oldMsg.batteryVoltage;
  armorlinkCopyString(out.source, sizeof(out.source), oldMsg.source);
  armorlinkCopyString(out.target, sizeof(out.target), oldMsg.target);
  armorlinkCopyString(out.entity, sizeof(out.entity), oldMsg.entity);
  armorlinkCopyString(out.command, sizeof(out.command), oldMsg.command);
  setArmorLinkPacketPayload(out, String(oldMsg.valueStr));
  return true;
}

inline String armorLinkMacToString(const uint8_t mac[6]) {
  if (mac == nullptr) return "";
  char buffer[18];
  snprintf(buffer, sizeof(buffer), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buffer);
}

inline bool armorLinkParseMacString(const String& text, uint8_t outMac[6]) {
  if (!outMac) return false;

  unsigned int values[6];
  if (sscanf(text.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) != 6) {
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    outMac[i] = static_cast<uint8_t>(values[i]);
  }

  return true;
}

#endif