# 🚀 Setting up PlatformIO for ESP32 + ODrive CAN Projects

This guide walks you through setting up your development environment for **ESP32 projects with PlatformIO**.
It covers installing PlatformIO, configuring VS Code, and getting dependencies right so you can start coding immediately.

---

## 📦 1. Install the Tools

1. **Visual Studio Code (VS Code)**
   Download and install from:
   👉 [https://code.visualstudio.com/](https://code.visualstudio.com/)

2. **PlatformIO IDE Extension**

   * Open VS Code
   * Go to Extensions (`Ctrl+Shift+X`)
   * Search for **PlatformIO IDE**
   * Install it

3. **Git** (optional, but recommended)
   👉 [https://git-scm.com/](https://git-scm.com/)
   This lets you clone repositories and manage versions of your code.

---

## ⚙️ 2. Open the Project

1. Clone/download this repository.
   Example with Git:

   ```bash
   git clone https://github.com/YourOrg/SRT_ODrive_Arduino.git
   ```

   Or download ZIP and extract.

2. In VS Code → **File → Open Folder** → select the project root (the folder containing `platformio.ini`).

> ⚠️ Do **not** just open `src/` or `lib/`. PlatformIO only works if the root with `platformio.ini` is open.

---

## 🔧 3. Configure VS Code IntelliSense

PlatformIO sometimes leaves red squiggles (`Arduino.h not found`). We’ve included a pre-configured `.vscode` folder in this repo.

If you ever break IntelliSense:

1. Press **Ctrl+Shift+P** → run **C/C++: Reset IntelliSense database**
2. Then **PlatformIO: Rebuild IntelliSense Index**
3. Reload VS Code

Squiggles gone ✅

---

## 🛠 4. PlatformIO Basics

PlatformIO uses **environments** defined in `platformio.ini`.
For example:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

### Key commands (run from VS Code terminal or sidebar):

* **Build:**

  ```
  pio run
  ```
* **Upload to board:**

  ```
  pio run -t upload
  ```
* **Open Serial Monitor:**

  ```
  pio device monitor
  ```

You can also use the **PlatformIO Toolbar** (bottom of VS Code) for these actions.

---

## 🔌 5. Installing Dependencies

PlatformIO manages dependencies automatically:

* Arduino-ESP32 framework
* ESP32 toolchains
* CAN/TWAI drivers (part of ESP-IDF, already included)

If you ever see `MissingPackageManifestError`, your local PlatformIO cache is corrupt. To fix:

```powershell
pio pkg uninstall -g espressif32
pio pkg install -g espressif32
```

This forces PlatformIO to re-download everything clean.

---

## 🧪 6. Testing Your Setup

1. Connect your ESP32 board via USB.

2. Create a simple test file in `src/blink.cpp`:

   ```cpp
   #include <Arduino.h>

   void setup() {
       pinMode(2, OUTPUT); // onboard LED on many ESP32 dev boards
   }

   void loop() {
       digitalWrite(2, HIGH);
       delay(500);
       digitalWrite(2, LOW);
       delay(500);
   }
   ```

3. Build + Upload → your ESP32 should blink its onboard LED.

---

## 🚦 7. Next Steps

* Look at `src/main.cpp` for project entry point
* Explore `lib/odrive_can/` for ODrive CAN driver code
* Use `pio device monitor` to view debug messages

---

## 🆘 Troubleshooting

* **Red squiggles on includes** → Reset IntelliSense (see Section 3)
* **Build fails with missing package.json** → delete broken package in
  `C:\Users\<you>\.platformio\packages\` and rebuild
* **Serial monitor shows garbage** → check `monitor_speed` matches your `Serial.begin()` baud rate

---

## ✅ You’re Ready!

With PlatformIO + VS Code set up, you can now:

* Write C++ for ESP32
* Build/upload with one click
* Communicate with ODrive over CAN bus