# 🚀 PlatformIO Setup (ESP32 + Arduino)

This guide gets new developers building this project with **PlatformIO** quickly and reliably.

---

## 1) Install the Tools

- **Visual Studio Code**: https://code.visualstudio.com/
- **PlatformIO IDE** (VS Code extension): search “PlatformIO IDE” in the Extensions tab
- **Git** (optional but recommended): https://git-scm.com/

---

## 2) Open the Project Correctly

- In VS Code, **File → Open Folder…** and select the project **root** (the folder containing `platformio.ini`).  
- Do *not* open only `src/` or `lib/` — PlatformIO won’t detect the project.

---

## 3) First Build

Open the VS Code terminal (``Ctrl+` ``) and run:
```bash
pio run
```
Or use the PlatformIO bottom toolbar: **Build**.

This will download the ESP32 platform, the Arduino‑ESP32 framework, and the correct toolchains.

---

## 4) Upload & Monitor

- **Upload**: `pio run -t upload` (or click Upload on the toolbar)
- **Serial Monitor**: `pio device monitor`  
  Make sure your `monitor_speed` in `platformio.ini` matches `Serial.begin(...)` (we use `115200`).

---

## 5) IntelliSense (fixing `Arduino.h` squiggles)

We ship provider‑based VS Code settings. If you still see red underlines:

1. **Ctrl+Shift+P → “C/C++: Reset IntelliSense database”**  
2. **Ctrl+Shift+P → “PlatformIO: Rebuild IntelliSense Index”**  
3. Reload VS Code

> The configuration uses the PlatformIO provider so no user‑specific paths are hardcoded. It will auto‑point to the correct `Arduino.h` on each machine.

---

## 6) Common Issues

### “MissingPackageManifestError: Could not find one of 'package.json'”
A PlatformIO package in `~/.platformio/packages` is corrupt. Fix:
```powershell
# Remove broken packages (Windows PowerShell example)
Get-ChildItem "$env:USERPROFILE\.platformio\packages" -Directory |
  Where-Object { -not (Test-Path "$($_.FullName)\package.json") } |
  Remove-Item -Recurse -Force

# Reinstall the espressif32 platform
pio pkg uninstall -g platformio/espressif32
pio pkg install   -g platformio/espressif32
```

### “cannot open source file 'Arduino.h'”
- Run a **Build** once so PlatformIO downloads frameworks.
- Then run the IntelliSense steps above.

### Serial monitor shows gibberish
- Ensure `monitor_speed = 115200` in `platformio.ini`
- Ensure `Serial.begin(115200)` in your code.

---

## 7) Minimal `platformio.ini`

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev        ; swap for your exact board if different
framework = arduino
monitor_speed = 115200

build_flags =
  -D ESP_PLATFORM
  -D ARDUINO_ARCH_ESP32
```

---

## 8) Sanity Probe (optional)

Create `src/zz_probe.cpp`:
```cpp
#include <Arduino.h>
void setup(){}
void loop(){}
```
Build. If it compiles, the toolchain is healthy.

---

You’re ready to develop. For CAN/ODrive specifics, see [ODrive_CAN_Background.md](ODrive_CAN_Background.md).
