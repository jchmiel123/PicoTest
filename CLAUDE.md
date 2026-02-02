# PicoTest - Espresso Machine Controller

Raspberry Pi Pico 2W (RP2350) based espresso machine controller with WiFi.

## Current Hardware

**Pico 2W (RP2350 + CYW43 WiFi)**
- Dual ARM Cortex-M33 @ 150MHz
- 520KB SRAM, 4MB Flash
- WiFi 2.4GHz (CYW43439)
- 26 GPIO pins

## Pinout (Active HIGH Relays)

| Pin | Function | Notes |
|-----|----------|-------|
| GP2 | Pump relay | Active HIGH |
| GP3 | Boiler relay | Active HIGH (check module!) |
| GP4 | Solenoid relay | Active HIGH |
| GP5 | Cup warmer relay | Active HIGH |
| GP6 | Flow sensor | Pulse input with ISR |
| GP26 | Thermistor | ADC0 (10K NTC) |
| LED_BUILTIN | Status LED | Via CYW43 chip |

**Reserved (WiFi chip):** GP23, GP24, GP25, GP29

## WiFi Networks

Configured networks (tries in order):
1. `Founders3-Office` / `Gu1fR3serVe13`
2. `DropitlikeitsHotspot` / `Nutmeg21`

Connection is **non-blocking** - relays work immediately while WiFi connects in background.

**Fallback AP Mode:** If WiFi fails after 3 retry cycles, creates hotspot:
- SSID: `Espresso`
- Password: `coffee123`
- IP: `192.168.4.1`

Serial command `C` retries WiFi from AP mode.

## Web Interface

After WiFi connects, access at:
- `http://<IP>` (shown on serial)
- `http://espresso.local` (mDNS)

**Features:**
- Real-time temperature display (°C or °F)
- Clickable relay toggles (with debounce)
- BREW/STOP buttons
- Adjustable target temp (50-100°C)
- Adjustable brew time (5-60 seconds)
- Flow rate and volume display
- WiFi status and reconnect button
- SVG coffee cup logo
- **Celsius/Fahrenheit toggle**
- **Password-protected Info panel** (password: `Coffee4Me!`)
  - System stats (uptime, free heap, SSID)
  - Reboot device
  - Deep sleep mode
- ~~OTA firmware update~~ (broken on RP2350, use USB)

## Brew Cycle State Machine

```
IDLE → PREHEATING → BREWING → FINISHING → IDLE
         ↓             ↓
     (timeout)    (complete)
         ↓             ↓
       ABORT         STOP
```

1. **IDLE**: All off, waiting for BOOTSEL or web BREW
2. **PREHEATING**: Boiler + warmer on, wait for MIN_BREW_TEMP (85°C)
3. **BREWING**: Pump + solenoid on for brewTimeMs (default 25s)
4. **FINISHING**: Brief pause, then back to IDLE

## Serial Commands (115200 baud)

| Key | Action |
|-----|--------|
| H | Help |
| S | Status |
| B | Start/Stop brew |
| P | Toggle pump |
| O | Toggle solenoid |
| W | Toggle cup warmer |
| +/- | Adjust target temp |
| R | Reset flow counter |
| X | EMERGENCY STOP |
| C | Retry WiFi connection |
| 1-4 | Direct relay toggle |

## PlatformIO

```bash
cd PicoTest
pio run -e pico2w           # Build
pio run -e pico2w -t upload # Flash
pio device monitor          # Serial (115200)
```

**platformio.ini:**
```ini
[env:pico2w]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2w
framework = arduino
board_build.core = earlephilhower
monitor_speed = 115200
```

## Key Settings

```cpp
const bool RELAY_ACTIVE_LOW = false;  // Your board is ACTIVE HIGH
const float MIN_BREW_TEMP = 85.0;     // Min temp to start brewing
unsigned long brewTimeMs = 25000;     // Adjustable via web (5-60s)
float targetTemp = 93.0;              // Default brew temp
float tempHysteresis = 2.0;           // Bang-bang control band
```

## Known Issues

- **OTA not working** - RP2350 Updater library broken (Error 4). Use USB/BOOTSEL upload.

- GP3 (boiler) may need different pin if relay module is faulty
- WiFi can take 8-15 seconds to connect on first boot
- Web page loads slowly on weak WiFi signal

## Files

- `src/main.cpp` - Main controller code (~895 lines)
- `platformio.ini` - Build configuration
- `CLAUDE.md` - This file
