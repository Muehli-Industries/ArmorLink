# 🛡️ ArmorLink

<!-- Replace with your logo once available -->
<!-- <p align="center">
  <img src="docs/logo.png" width="220" />
</p> -->

<p align="center">
  Lightweight communication framework for ESP32 systems  
  <br>
  Real-time • Modular • App-ready
</p>

<p align="center">
  <img src="https://img.shields.io/badge/platform-ESP32-blue">
  <img src="https://img.shields.io/badge/license-MIT-green">
  <img src="https://img.shields.io/badge/status-active-success">
</p>

---

ArmorLink is a lightweight communication framework for ESP32-based systems, designed for real-time interaction between hardware modules and a central gateway.

Built for modular systems like wearables, robotics, and distributed control setups — but just as powerful for single-device projects.

---

## ✨ Key Capabilities

### ⚡ ESP-NOW Communication
- Ultra low latency  
- No WiFi infrastructure required  
- Battery-friendly  

### 🔐 Seamless Pairing
- Gateway-controlled discovery  
- Persistent pairing storage  

### 🧩 Flexible System Design
- Single ESP32 projects  
- Fully modular multi-device systems  

### 📡 Optional App Control (BLE)
- Live control, logs, and configuration  
- JSON-based protocol  
- Works on both BLE backends of arduino-esp32 3.x: Bluedroid (ESP32) and NimBLE (ESP32-C3/C6/H2…)  
- With a PIN set, characteristics require an authenticated (passkey/MITM) link; Just Works peers are disconnected and unpaired  

### 🧠 Smart Logging
- Module → Gateway forwarding  
- Runtime enable / disable  
- Log level filtering  

### ❤️ Implicit Presence Detection
- No heartbeat packets required  
- Activity-based liveness detection  
- Built-in timeout callbacks  

---

## 🏗️ Architecture

### Single Device
```text
[ Mobile App (optional) ] → [ ESP32 ]
```

### Modular System
```text
[ Mobile App (optional) ]
            ↓
      [ Gateway ESP32 ]
            ↓
   [ Module ] [ Module ] [ Module ]
```

---

## 🚀 Quick Start

### Minimal Setup
```cpp
ArmorLink.begin(module, options);
```

### Gateway Mode
```cpp
ArmorLink.setGatewayMode(true);
ArmorLink.begin(module, options);
```

---

## ⚙️ Configuration & Actions

`ArmorLink.begin()` automatically registers **Reset Pairing info** in the
**ArmorLink** section (`pairing/reset`), including the Web Serial descriptor.
It removes only this node's gateway association: paired flag, gateway name,
gateway MAC and recovery PIN. Configuration, profiles, action definitions,
components and a gateway's paired-module list are preserved. The node becomes
unpaired immediately without a reboot; flash updated firmware to add the action
to existing devices. An unpaired module can then join a new gateway on the same
ESP-NOW channel.

ArmorLink modules define their own configuration and actions.  
The app automatically adapts to it.

### Compile-time features

All features remain available by default. To remove unused functionality from
flash and RAM, define exclusions **before any ArmorLink header**:

```cpp
// Single node with BLE, ordinary actions and custom serial commands:
#define ARMORLINK_EXCLUDE_ESPNOW
#define ARMORLINK_EXCLUDE_ACTIONBUILDER
#define ARMORLINK_EXCLUDE_DATASTREAMS
#include <ArmorLink.h>
```

| Define | Removes |
| --- | --- |
| `ARMORLINK_EXCLUDE_ESPNOW` | ESP-NOW/WiFi transport, receive queue, peer map, pairing processing and network tables. Gateway mode is disabled; local BLE logs and telemetry still work. |
| `ARMORLINK_EXCLUDE_BLE` | BLE stack integration, server, callbacks and transfers. BLE notification helpers become no-ops. |
| `ARMORLINK_EXCLUDE_ACTIONBUILDER` | Dynamic sequences, components, custom actions and their storage/descriptor integration. Ordinary `module.actions().add(...).command(...).onExecute(...)` remains available; `module.actionBuilder()` is absent. |
| `ARMORLINK_EXCLUDE_DATASTREAMS` | Stream routing, subscriptions, handlers and rate tables. Registration/subscription returns `false`; sending returns `ESP_ERR_NOT_SUPPORTED`. Telemetry remains available. |
| `ARMORLINK_EXCLUDE_WEB_SERIAL` | The `@ALF:` JSON input/event protocol and its reachable setup/import handlers. Custom `module.serial()` commands and the interactive serial menu remain available. |

The central configuration is [ArmorLinkFeatures.h](ArmorLinkFeatures.h).
Exclusions remove the corresponding built-in system settings and pairing-reset
action where applicable; persisted settings cannot re-enable excluded radios.
Existing NVS data is preserved. Dynamic ActionBuilder commands report
`unsupported_feature` when excluded, and descriptors report
`supportsActionBuilder: false` without advertising components or sequences.
Remote transport operations return `ESP_ERR_NOT_SUPPORTED` when ESP-NOW is
excluded.

For sketches with multiple translation units, apply the same defines everywhere,
preferably using compiler `-D` flags. Calls to the dynamic ActionBuilder in your
own sketch must also be guarded with `#if ARMORLINK_HAS_ACTIONBUILDER`.

### Configurable capacities and diagnostics

These defaults are centralized in `ArmorLinkFeatures.h` and can be overridden
before including ArmorLink:

| Define | Default |
| --- | ---: |
| `ARMORLINK_ESPNOW_QUEUE_SIZE` | 64 |
| `ARMORLINK_MAX_PAIRED_MODULES` | 16 |
| `ARMORLINK_MAX_PAIRING_CANDIDATES` | 16 |
| `ARMORLINK_MAX_DATA_STREAM_SUBSCRIPTIONS` | 24 |
| `ARMORLINK_MAX_LOCAL_DATA_STREAM_SUBSCRIPTIONS` | 12 |
| `ARMORLINK_MAX_REQUESTED_DATA_STREAMS` | 12 |
| `ARMORLINK_MAX_DATA_STREAM_HANDLERS` | 12 |
| `ARMORLINK_MAX_DATA_STREAM_RATE_KEYS` | 8 |
| `ARMORLINK_MAX_DATA_STREAM_VALUES` | 8 |
| `ARMORLINK_MAX_TELEMETRY_KEYS` | 16 |
| `ARMORLINK_SERIAL_COMMAND_MAX_LEN` | 31 |
| `ARMORLINK_SERIAL_LINE_MAX_LEN` | 1024 |

Enabled tables require positive capacities; the ESP-NOW queue requires at least
two slots. Reducing paired-module capacity limits the number of stored peers
that can be loaded and subsequently saved. Packet sizes and protocol identifiers
remain unchanged.

`ARMORLINK_INTERNAL_LOG_LEVEL` controls internal diagnostics:

- `0`: omit diagnostics and their argument evaluation.
- `1` (default): retain existing regular diagnostics; `ARMORLINK_VERBOSE` retains
  its existing role.
- `2`: include regular and verbose diagnostics.

Application logs (`ArmorLink.info/warn/error/debug`, structured logs), menu output
and enabled protocol responses remain available at level `0`.

[Feature footprint chart](docs/feature-footprint.png) compares flash and static RAM usage.
[Runtime heap chart](docs/heap-benchmark.png) compares feature variants on a physical ESP32.

### RAM usage

Local BLE configuration descriptors are streamed directly into 80-byte chunks;
the gateway does not retain the complete descriptor string during this transfer.

The ESP-NOW receive queue defaults to 64 slots. For nodes with limited free RAM,
define its size before including ArmorLink (use the same value in all translation
units, preferably through a compiler define):

```cpp
#define ARMORLINK_ESPNOW_QUEUE_SIZE 16
#include <ArmorLink.h>
```

Reducing 64 slots to 16 saves about 12 KB with the current packet format. A smaller
queue accepts fewer pending packets and can drop more packets during bursts or
when the main loop is blocked. At least two slots are required; the ring buffer
keeps one slot unused to distinguish full from empty.

```cpp
void setupModule() {

  module.config()
    .addInt("volume", &volume, 15)
      .label("Volume")
      .section("Audio")
      .range(0, 30)
      .step(1)
      .persist("volume")
      .command("audio", "volume");

  module.config()
    .addReadonly("dfReady", dfReady ? "true" : "false")
      .label("DFPlayer Ready")
      .section("Audio");

  module.actions()
    .add("open")
      .label("Open")
      .command("servo", "open");

  module.actions()
    .add("close")
      .label("Close")
      .command("servo", "close");
}
```

---

## 📱 App Experience

Your module defines how it behaves — the app adapts automatically.

- sliders, inputs and toggles for configuration  
- live status values  
- buttons for actions  

Everything appears instantly, without building custom UI.

---

## 🖼️ UI Preview

> *(Add screenshots later)*

```
/docs/screenshots/dashboard.png
/docs/screenshots/module-detail.png
/docs/screenshots/logs.png
```

---

## 🔄 Communication Flow

### Pairing
```text
Gateway → broadcast pairing
Module  → responds
Gateway → accepts
```

### Runtime
```text
Module → sends logs / state
Gateway → forwards to app
App → sends commands via BLE
Gateway → routes to modules
```

---

## 🧠 Concept

> One app. Any setup.

Whether you control a single ESP32 or a full modular system —  
the same app works out of the box.

Just define your module in firmware, and you're ready to go.

---

## 📦 Design Goals

- Minimal network overhead  
- Battery efficiency  
- Deterministic behavior  
- Full developer control  

---

## 📄 License

MIT License

### ActionBuilder log messages

The dynamic ActionBuilder supports **Log Message** blocks with Debug, Info,
Warning or Error and custom text. Insert live State values using
`{state:componentId}`, for example `Faceplate open: {state:faceplateOpen}`.
The block uses the existing application logging settings and is covered by
`ARMORLINK_EXCLUDE_ACTIONBUILDER`.

Web Serial configuration transfers stream the complete WebSerial descriptor in
bounded, UTF-8-safe 192-byte chunks. The firmware no longer builds a full JSON
String for this transport or silently falls back to the BLE descriptor when a
large descriptor does not fit in a contiguous heap block. Metadata is measured
before sending; changed content is rejected without a successful end frame.
This preserves ActionBuilder capabilities and definitions on memory-constrained
nodes. No stored configuration is erased or reset. Focused regression:
`python3 tests/serial_descriptor_stream_test.py`.

### Startup sequence

Create a dedicated **Startup sequence** in Web Setup using normal ActionBuilder
blocks. It can call existing Actions such as **Close Faceplate** via **Run
Action**. The marked Action starts once in the first `ArmorLink.loop()` after
initialization; saving it never triggers execution. It persists across reboots
and is included in profiles.

### Stable release

ArmorLink 0.5.0 includes the configurable features, ActionBuilder Log Messages,
Startup Actions and bounded full WebSerial descriptor streaming. Default feature
flags remain enabled; existing device configurations are preserved.

**App compatibility:** ArmorLink 0.5.0 requires ArmorLink App 1.6 or later for
correct operation. Update the app together with the firmware.
