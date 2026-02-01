# PicoTest

Raspberry Pi Pico / Pico 2 development project.

## Hardware

**Raspberry Pi Pico (RP2040)**
- Dual ARM Cortex-M0+ @ 133MHz
- 264KB SRAM, 2MB Flash
- 26 GPIO pins (23 usable)
- **12mA per GPIO pin** - stronger than ESP32!
- 3x 12-bit ADC
- 2x UART, 2x I2C, 2x SPI
- 16x PWM channels
- USB 1.1 Host/Device
- PIO (Programmable I/O) - custom protocols!

**Pico 2 (RP2350)**
- Dual ARM Cortex-M33 @ 150MHz
- 520KB SRAM, 4MB Flash
- Same GPIO count, improved peripherals
- Hardware floating point
- Better security features

## GPIO Drive Strength

**This is the key info you need:**

```
ESP32:   ~12mA max per pin, but practically weaker
Pico:    12mA per pin (configurable 2/4/8/12mA)
Teensy:  4mA default, up to 150mA TOTAL for all GPIO

For relay control:
- Direct drive: Only for solid-state relays rated for 3.3V/12mA
- Transistor: 2N2222/2N3904 (NPN) - Pico can drive the base fine
- MOSFET: IRLZ44N, AO3400 - logic-level, 3.3V gate threshold
```

## Pin Reference

### Digital I/O (GP0-GP28)
- GP0-GP22: General purpose
- GP23: Controls on-board SMPS power save (leave alone)
- GP24: VBUS sense (USB power detection)
- GP25: On-board LED
- GP26-GP28: ADC capable (also digital)

### ADC Inputs
- GP26 (ADC0)
- GP27 (ADC1)
- GP28 (ADC2)
- Internal temp sensor (ADC4)

### Communication
- UART0: GP0 (TX), GP1 (RX)
- UART1: GP4 (TX), GP5 (RX)
- I2C0: GP4 (SDA), GP5 (SCL)
- I2C1: GP6 (SDA), GP7 (SCL)
- SPI0: GP16 (MISO), GP19 (MOSI), GP18 (SCK), GP17 (CS)
- SPI1: GP8 (MISO), GP11 (MOSI), GP10 (SCK), GP9 (CS)

### PWM Channels
- All GPIO pins can do PWM
- 8 PWM slices, 2 channels each

## Relay Driver Circuit

```
For mechanical relay (5V coil, ~70mA):

Pico GPIO ─────┬───[1kΩ]───┬──── Base (2N2222)
               │           │
              [10kΩ]      Emitter ──── GND
               │
              GND         Collector ──┬──── Relay Coil (-)
                                      │
                          Relay (+) ──┴──── 5V
                                      │
                              Flyback [1N4007] ──┘

For SSR (Solid State Relay):
- Many SSRs trigger at 3-32VDC, 5-25mA
- Pico can drive directly if SSR is 3.3V compatible
```

## PlatformIO Commands

```bash
cd PicoTest
pio run                    # Build
pio run -t upload          # Flash via USB (hold BOOTSEL)
pio device monitor         # Serial monitor
```

**IMPORTANT for Pico 2 (RP2350):**
Standard PlatformIO doesn't support Pico 2 yet. Use maxgerhardt's fork:

```ini
[env:pico2]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2
framework = arduino
board_build.core = earlephilhower
```

## Project Ideas

- [x] Basic GPIO test with relay driver
- [x] Coffee machine controller (main.cpp)
- [ ] I2C sensor hub
- [ ] PWM motor control
- [ ] USB HID device
- [ ] PIO-based custom protocols

## Coffee Machine Controller

The current `main.cpp` implements a coffee machine controller with:

**Features:**
- 4 relays: pump, boiler heater, 3-way solenoid, main power
- Flow sensor with pulse counting (ISR-based)
- Thermistor temperature reading (Steinhart-Hart equation)
- On/off heater control with hysteresis (bang-bang)
- Brew cycle with temperature safety check
- Serial command interface

**Commands (115200 baud):**
| Key | Action |
|-----|--------|
| H | Help |
| S | Status |
| B | Start/Stop brew |
| P | Toggle pump |
| O | Toggle solenoid |
| M | Toggle main power |
| +/- | Adjust target temp |
| R | Reset flow counter |
| X | EMERGENCY STOP |
| 1-4 | Direct relay toggle |

**Pinout:**
- GP2-5: Relay outputs (active LOW)
- GP6: Flow sensor (pulses)
- GP26: Thermistor ADC
- GP25: Onboard LED (heartbeat)

**Calibration needed:**
- `PULSES_PER_ML` - adjust for your flow sensor
- Thermistor coefficients if using different NTC
- Heating rate estimate (currently ~2C/min)
