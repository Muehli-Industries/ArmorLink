#pragma once

// Define exclusions/capacities before any ArmorLink header, consistently in all
// translation units (compiler -D flags are recommended for multi-file sketches).
#ifdef ARMORLINK_EXCLUDE_ESPNOW
#define ARMORLINK_HAS_ESPNOW 0
#else
#define ARMORLINK_HAS_ESPNOW 1
#endif
#ifdef ARMORLINK_EXCLUDE_BLE
#define ARMORLINK_HAS_BLE 0
#else
#define ARMORLINK_HAS_BLE 1
#endif
#ifdef ARMORLINK_EXCLUDE_ACTIONBUILDER
#define ARMORLINK_HAS_ACTIONBUILDER 0
#else
#define ARMORLINK_HAS_ACTIONBUILDER 1
#endif
#ifdef ARMORLINK_EXCLUDE_DATASTREAMS
#define ARMORLINK_HAS_DATASTREAMS 0
#else
#define ARMORLINK_HAS_DATASTREAMS 1
#endif
#ifdef ARMORLINK_EXCLUDE_WEB_SERIAL
#define ARMORLINK_HAS_WEB_SERIAL 0
#else
#define ARMORLINK_HAS_WEB_SERIAL 1
#endif

#ifndef ARMORLINK_ESPNOW_QUEUE_SIZE
#define ARMORLINK_ESPNOW_QUEUE_SIZE 64
#endif
#ifndef ARMORLINK_MAX_PAIRED_MODULES
#define ARMORLINK_MAX_PAIRED_MODULES 16
#endif
#ifndef ARMORLINK_MAX_PAIRING_CANDIDATES
#define ARMORLINK_MAX_PAIRING_CANDIDATES 16
#endif
#ifndef ARMORLINK_MAX_DATA_STREAM_SUBSCRIPTIONS
#define ARMORLINK_MAX_DATA_STREAM_SUBSCRIPTIONS 24
#endif
#ifndef ARMORLINK_MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS
#define ARMORLINK_MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS 12
#endif
#ifndef ARMORLINK_MAX_REQUESTED_DATA_STREAMS
#define ARMORLINK_MAX_REQUESTED_DATA_STREAMS 12
#endif
#ifndef ARMORLINK_MAX_DATA_STREAM_HANDLERS
#define ARMORLINK_MAX_DATA_STREAM_HANDLERS 12
#endif
#ifndef ARMORLINK_MAX_TELEMETRY_KEYS
#define ARMORLINK_MAX_TELEMETRY_KEYS 16
#endif
#ifndef ARMORLINK_MAX_DATA_STREAM_RATE_KEYS
#define ARMORLINK_MAX_DATA_STREAM_RATE_KEYS 8
#endif

#ifndef ARMORLINK_SERIAL_COMMAND_MAX_LEN
#define ARMORLINK_SERIAL_COMMAND_MAX_LEN 31
#endif
#ifndef ARMORLINK_SERIAL_LINE_MAX_LEN
#define ARMORLINK_SERIAL_LINE_MAX_LEN 1024
#endif
#ifndef ARMORLINK_MAX_DATA_STREAM_VALUES
#define ARMORLINK_MAX_DATA_STREAM_VALUES 8
#endif

// 0 removes internal diagnostics, 1 preserves existing regular diagnostics,
// 2 also enables verbose diagnostics. Explicit application logs remain separate.
#ifndef ARMORLINK_INTERNAL_LOG_LEVEL
#define ARMORLINK_INTERNAL_LOG_LEVEL 1
#endif
static_assert(ARMORLINK_INTERNAL_LOG_LEVEL >= 0 && ARMORLINK_INTERNAL_LOG_LEVEL <= 2,
              "ArmorLink internal log level must be 0, 1 or 2");
#if ARMORLINK_HAS_ESPNOW
static_assert(ARMORLINK_ESPNOW_QUEUE_SIZE >= 2, "ESP-NOW queue needs at least two slots");
static_assert(ARMORLINK_MAX_PAIRED_MODULES > 0, "Paired module capacity must be positive");
static_assert(ARMORLINK_MAX_PAIRING_CANDIDATES > 0, "Pairing candidate capacity must be positive");
#endif
#if ARMORLINK_HAS_DATASTREAMS
static_assert(ARMORLINK_MAX_DATA_STREAM_SUBSCRIPTIONS > 0 &&
              ARMORLINK_MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS > 0 &&
              ARMORLINK_MAX_REQUESTED_DATA_STREAMS > 0 &&
              ARMORLINK_MAX_DATA_STREAM_HANDLERS > 0 &&
              ARMORLINK_MAX_DATA_STREAM_RATE_KEYS > 0 &&
              ARMORLINK_MAX_DATA_STREAM_VALUES > 0,
              "Data stream capacities must be positive");
#endif
static_assert(ARMORLINK_MAX_TELEMETRY_KEYS > 0, "Telemetry capacity must be positive");

static_assert(ARMORLINK_SERIAL_COMMAND_MAX_LEN > 0 && ARMORLINK_SERIAL_LINE_MAX_LEN > 0,
              "Serial command and line capacities must be positive");
