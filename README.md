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
