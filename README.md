# 🔧 ODrive Arduino CAN Replacement

This project is a rewrite of the CAN aspect of the ODrive Arduino library.
The goal is to provide a cleaner, more flexible implementation that is **CAN library independent** and easier to extend/maintain.

---

## 🚀 Getting Started

This repo uses **PlatformIO** with the Arduino framework (ESP32 target).

### 1. Prerequisites

* [Visual Studio Code](https://code.visualstudio.com/)
* [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)
* (Optional) [Git](https://git-scm.com/)

### 2. Open the Project

Clone or download this repo, then in VS Code choose **File → Open Folder…** and select the project root (where `platformio.ini` lives).

### 3. First Build & Upload

* Connect your ESP32 board over USB
* Run **PlatformIO: Build** → **PlatformIO: Upload**
* Open **PlatformIO: Serial Monitor** to see logs

👉 For step-by-step setup help (including fixing `Arduino.h` IntelliSense errors), see [SETUP.md](./SETUP.md).

---

## 🧩 Implementation

### Sending CAN Messages & Constructor

To implement this library you must create a function with the following prototype:

```cpp
int send_can (uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);
```

This function pointer and the motor’s Node ID are passed into the constructor:

```cpp
ODriveCanMtr mtr1(&send_can, 0);
```

See [Implementation Details](#implementation) below for full breakdown.

---

## 📚 Documentation

* The ODrive documentation includes a `.dbc` file for CAN messages (see `/assets`)
  👉 View it with a [DBC editor](https://www.csselectronics.com/pages/dbc-editor-can-bus-database)
* ODrive CAN protocol reference: [docs.odriverobotics.com → CAN Protocol](https://docs.odriverobotics.com/v/latest/manual/can-protocol.html)

---

## 🛠 Requirements

* Uses Arduino functions (`millis()`, `Serial`) by default — can be adapted for other MCUs.
* Your upper-level code must handle **CAN bus reads** and pass messages into motor objects.

---

## 📡 Background Knowledge

This repo includes detailed explanations of:

* CAN 2.0 basics
* Frame structure (Arbitration, RTR, DLC, ACK)
* ODrive’s CAN standard (Node IDs, Command IDs, command list)

👉 Scroll down to [Background Knowledge](#background-knowledge).

---

## 🤝 Contributing

* Build and test with PlatformIO (ESP32 target)
* Use the included `.gitignore` (don’t commit `.pio/` build output or machine-specific settings)
* Shared VS Code configs live in `.vscode/`

---

## 📜 License

MIT License — see [LICENSE](./LICENSE) for details.
