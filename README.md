# 🔧 ODrive Arduino CAN Replacement

This project rewrites the CAN interface of the ODrive Arduino library with a cleaner, callback‑driven approach.
It is designed to be **CAN‑library independent** and lightweight, while still exposing the ODrive CAN protocol.

---

## 🚀 Quick Start

1) **Install VS Code** and the **PlatformIO IDE** extension.
2) **Open this folder** (the one with `platformio.ini`) in VS Code.
3) Connect your ESP32 via USB.
4) Use the PlatformIO toolbar (bottom bar) to **Build → Upload → Monitor**.

> Detailed onboarding (including IntelliSense fixes) is in [docs/SETUP.md](docs/SETUP.md).

---

## 🧩 Library Design

- You provide a function pointer used to transmit raw CAN frames. The library doesn’t depend on any specific CAN stack.
- Your application must read CAN frames from the bus and feed them back to the library for parsing.

### Minimal usage

```cpp
// Your CAN TX function (you implement this)
int send_can(uint16_t can_id, uint8_t len, uint8_t* data, bool rtr);

// Construct for node 0
ODriveCanMtr mtr1(&send_can, 0);

// Push incoming frames to the driver
mtr1.process_msg(can_id, data_len, data_bytes);

// Typical control (velocity example)
mtr1.set_cont_mode(velocity_control, vel_ramp);
mtr1.set_axis_state(closed_loop_control);
mtr1.set_ip_vel(5.0f, 0.0f);
```

More API details live in the header (`lib/odrive_can/include/ODriveCan.h`).

---

## 📁 Project Structure

```
lib/
  odrive_can/
    include/
      ODriveCan.h
    src/
      ODriveCan.cpp
src/
  main.cpp
docs/
  SETUP.md
  ODrive_CAN_Background.md
.vscode/
  settings.json
  c_cpp_properties.json
platformio.ini
```

- The **`.vscode/`** files are safe to commit and are provider‑based (no hardcoded user paths).
  
---

## 🧰 Requirements

- Arduino core functions (`millis()`, `Serial`) are used by default.
- The upper layer must:
  - implement `send_can` (raw CAN TX)
  - poll your CAN driver and call `mtr1.process_msg(...)` with received frames.

---

## 📚 Deeper Docs

- Setup & dependencies: [docs/SETUP.md](docs/SETUP.md)
- Background on CAN & ODrive protocol: [docs/ODrive_CAN_Background.md](docs/ODrive_CAN_Background.md)
- Official ODrive CAN docs: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html

---

## 🤝 Contributing

- Use PlatformIO to build/test (ESP32 Arduino framework).
- Don’t commit `/.pio/` or other machine artifacts (see `.gitignore`).

## 📜 License

MIT — see `LICENSE`.
