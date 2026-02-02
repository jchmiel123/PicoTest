/**
 * Coffee Machine Controller - Pico 2 (RP2350)
 *
 * Controls:
 * - 4 Relays (pump, boiler, group head solenoid, main power)
 * - Flow sensor (pulse counting)
 * - Thermistor/thermocouple for boiler temp
 * - Heater element (via SSR or relay)
 *
 * Pinout:
 *   GP2-5:   Relay outputs (active LOW typically)
 *   GP6:     Flow sensor input (pulses)
 *   GP26:    Thermistor ADC input (ADC0)
 *   GP27:    Optional second temp sensor (ADC1)
 *   GP28:    Optional pressure sensor (ADC2)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ============ WIFI CONFIG ============
const char* WIFI_SSID = "DropitlikeitsHotspot";
const char* WIFI_PASS = "Nutmeg21";

WebServer server(80);

// ============ PIN DEFINITIONS ============
// Relays (active LOW for most relay boards)
const int RELAY_PUMP = 2;        // Water pump
const int RELAY_BOILER = 3;      // Boiler heater element
const int RELAY_SOLENOID = 4;    // Solenoid valve (RO water in)
const int RELAY_WARMER = 5;      // Cup warmer (optional heating element)

// BOOTSEL button - directly readable on Pico
// Note: BOOTSEL is directly supported via BOOTSEL macro in Arduino-Pico

// Flow sensor (Hall effect, outputs pulses)
const int FLOW_SENSOR = 6;

// Temperature sensing
const int TEMP_BOILER = 26;      // ADC0 - Thermistor on boiler
const int TEMP_GROUP = 27;       // ADC1 - Optional group head temp

// Onboard LED
// Pico 2W: LED is on WiFi chip, use LED_BUILTIN (maps to CYW43 GPIO)
// Pico 2: LED is on GP25
const int LED_PIN = LED_BUILTIN;

// ============ RELAY CONFIG ============
// Set to true if your relay board is active LOW (most are)
const bool RELAY_ACTIVE_LOW = true;

#define RELAY_ON  (RELAY_ACTIVE_LOW ? LOW : HIGH)
#define RELAY_OFF (RELAY_ACTIVE_LOW ? HIGH : LOW)

// ============ FLOW SENSOR ============
// Typical: YF-S201 = 7.5 pulses per mL (450 pulses/L)
// Adjust based on your sensor's datasheet
const float PULSES_PER_ML = 7.5;

volatile unsigned long flowPulseCount = 0;
unsigned long lastFlowCheck = 0;
float flowRate = 0;        // mL/sec
float totalVolume = 0;     // mL

// ============ TEMPERATURE ============
// For 10K NTC thermistor with 10K pullup to 3.3V
// Steinhart-Hart coefficients (generic 10K NTC)
const float THERM_A = 0.001129148;
const float THERM_B = 0.000234125;
const float THERM_C = 0.0000000876741;
const float THERM_NOMINAL = 10000;  // 10K at 25C
const float TEMP_NOMINAL = 25;
const float THERM_BCOEFF = 3950;    // Beta coefficient
const float SERIES_RESISTOR = 10000; // 10K pullup

float boilerTemp = 0;
float targetTemp = 93.0;   // Default brew temp (espresso sweet spot)
bool heaterOn = false;
float tempHysteresis = 2.0; // +/- degrees

// ============ BREW SETTINGS ============
const unsigned long BREW_TIME_MS = 25000;  // 25 seconds (standard espresso)
const unsigned long PREHEAT_TIMEOUT_MS = 60000;  // Max 60s to heat up
const float MIN_BREW_TEMP = 85.0;  // Minimum temp to start brewing
unsigned long brewTimeMs = BREW_TIME_MS;  // Adjustable via web UI

// ============ STATE ============
bool pumpOn = false;
bool solenoidOn = false;
bool warmerOn = false;
bool brewingActive = false;
bool preheating = false;
unsigned long brewStartTime = 0;
unsigned long preheatStartTime = 0;

// State machine for brew cycle
enum BrewState {
    IDLE,           // Everything off, waiting for button
    PREHEATING,     // Boiler heating up
    BREWING,        // Pump + solenoid on, extracting
    FINISHING       // Cleanup/cooldown
};
BrewState brewState = IDLE;

// ============ FORWARD DECLARATIONS ============
void flowSensorISR();
void updateFlowRate();
float readTemperature(int pin);
void updateHeaterControl();
void setRelay(int pin, bool on, const char* name);
void toggleRelayDirect(int relayNum);
void allRelaysOff();
void printStatus();
void printHelp();
void handleCommand(char cmd);
void startBrewCycle();
void updateBrewCycle();
void stopBrew();
void abortBrew(const char* reason);
void setupWiFi();
void setupWebServer();
String getStatusJson();
String getWebPage();

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(2000);

    // Relay pins - all OFF at start
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_BOILER, OUTPUT);
    pinMode(RELAY_SOLENOID, OUTPUT);
    pinMode(RELAY_WARMER, OUTPUT);

    digitalWrite(RELAY_PUMP, RELAY_OFF);
    digitalWrite(RELAY_BOILER, RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, RELAY_OFF);
    digitalWrite(RELAY_WARMER, RELAY_OFF);

    // Flow sensor with pullup, interrupt on rising edge
    pinMode(FLOW_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), flowSensorISR, RISING);

    // ADC setup
    analogReadResolution(12);  // 12-bit ADC

    // LED
    pinMode(LED_PIN, OUTPUT);

    Serial.println("================================");
    Serial.println("Coffee Machine Controller");
    Serial.println("Pico 2 (RP2350)");
    Serial.println("================================");
    Serial.print("Relay mode: ");
    Serial.println(RELAY_ACTIVE_LOW ? "ACTIVE LOW" : "ACTIVE HIGH");
    Serial.print("Target temp: ");
    Serial.print(targetTemp);
    Serial.println(" C");
    Serial.println();

    // Setup WiFi and web server
    setupWiFi();
    setupWebServer();

    printHelp();
}

// ============ MAIN LOOP ============
void loop() {
    static unsigned long lastTempRead = 0;
    static unsigned long lastStatusPrint = 0;
    static unsigned long lastBlink = 0;
    static bool ledState = false;
    static bool lastButtonState = false;

    unsigned long now = millis();

    // Read BOOTSEL button (built into Pico)
    // BOOTSEL is LOW when pressed
    bool buttonPressed = BOOTSEL;

    // Button press detection (rising edge - just pressed)
    if (buttonPressed && !lastButtonState) {
        Serial.println("BOOTSEL pressed!");
        if (brewState == IDLE) {
            startBrewCycle();
        } else {
            // Button during brew = abort
            abortBrew("User cancelled");
        }
    }
    lastButtonState = buttonPressed;

    // Heartbeat LED pattern based on state
    unsigned long blinkInterval;
    switch (brewState) {
        case PREHEATING: blinkInterval = 250; break;  // Medium blink - heating
        case BREWING:    blinkInterval = 100; break;  // Fast blink - brewing
        default:         blinkInterval = 1000; break; // Slow blink - idle
    }
    if (now - lastBlink >= blinkInterval) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    // Read temperature every 250ms
    if (now - lastTempRead >= 250) {
        lastTempRead = now;
        boilerTemp = readTemperature(TEMP_BOILER);

        // Only run heater control during preheat/brew
        if (brewState == PREHEATING || brewState == BREWING) {
            updateHeaterControl();
        }
    }

    // Update flow rate every 500ms
    if (now - lastFlowCheck >= 500) {
        updateFlowRate();
        lastFlowCheck = now;
    }

    // Run the brew state machine
    updateBrewCycle();

    // Auto status print every 2 seconds if not idle
    if (brewState != IDLE && (now - lastStatusPrint >= 2000)) {
        lastStatusPrint = now;

        const char* stateNames[] = {"IDLE", "PREHEAT", "BREW", "FINISH"};
        Serial.print("[");
        Serial.print(stateNames[brewState]);

        if (brewState == BREWING) {
            unsigned long brewTime = (now - brewStartTime) / 1000;
            Serial.print(" ");
            Serial.print(brewTime);
            Serial.print("s");
        }

        Serial.print("] Temp: ");
        Serial.print(boilerTemp, 1);
        Serial.print("C");

        if (brewState == BREWING) {
            Serial.print(", Flow: ");
            Serial.print(flowRate, 1);
            Serial.print(" mL/s, Vol: ");
            Serial.print(totalVolume, 1);
            Serial.print(" mL");
        }
        Serial.println();
    }

    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    // Handle web requests
    server.handleClient();
}

// ============ FLOW SENSOR ISR ============
void flowSensorISR() {
    flowPulseCount++;
}

void updateFlowRate() {
    static unsigned long lastPulseCount = 0;
    static unsigned long lastTime = 0;

    unsigned long now = millis();
    unsigned long pulses = flowPulseCount - lastPulseCount;
    unsigned long dt = now - lastTime;

    if (dt > 0) {
        // mL per second
        float ml = pulses / PULSES_PER_ML;
        flowRate = ml * 1000.0 / dt;  // mL/sec
        totalVolume += ml;
    }

    lastPulseCount = flowPulseCount;
    lastTime = now;
}

// ============ TEMPERATURE READING ============
float readTemperature(int pin) {
    int raw = analogRead(pin);

    // Convert to resistance
    // Assuming thermistor between pin and GND, pullup to 3.3V
    float voltage = raw * 3.3 / 4095.0;
    float resistance = SERIES_RESISTOR * voltage / (3.3 - voltage);

    // Steinhart-Hart equation
    float steinhart;
    steinhart = resistance / THERM_NOMINAL;          // (R/Ro)
    steinhart = log(steinhart);                      // ln(R/Ro)
    steinhart /= THERM_BCOEFF;                       // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMP_NOMINAL + 273.15);     // + (1/To)
    steinhart = 1.0 / steinhart;                     // Invert
    steinhart -= 273.15;                             // Convert to C

    return steinhart;
}

// ============ HEATER CONTROL ============
void updateHeaterControl() {
    // Simple on/off with hysteresis (bang-bang control)
    // Future: implement PID for more precise temperature control

    if (boilerTemp < (targetTemp - tempHysteresis)) {
        // Boiler too cold - turn on heater
        if (!heaterOn) {
            setRelay(RELAY_BOILER, true, "Boiler");
            heaterOn = true;
        }
    } else if (boilerTemp > (targetTemp + tempHysteresis)) {
        // Boiler at/above target - turn off heater
        if (heaterOn) {
            setRelay(RELAY_BOILER, false, "Boiler");
            heaterOn = false;
        }
    }
    // Within hysteresis band - maintain current state
}

// ============ RELAY CONTROL ============
void setRelay(int pin, bool on, const char* name) {
    digitalWrite(pin, on ? RELAY_ON : RELAY_OFF);
    Serial.print(name);
    Serial.print(": ");
    Serial.println(on ? "ON" : "OFF");
}

// Toggle relay by number (1-4) for direct testing
void toggleRelayDirect(int relayNum) {
    const int pins[] = {0, RELAY_PUMP, RELAY_BOILER, RELAY_SOLENOID, RELAY_WARMER};
    const char* names[] = {"", "Pump", "Boiler", "Solenoid", "Warmer"};
    bool* states[] = {nullptr, &pumpOn, &heaterOn, &solenoidOn, &warmerOn};

    if (relayNum < 1 || relayNum > 4) return;

    *states[relayNum] = !(*states[relayNum]);
    digitalWrite(pins[relayNum], *states[relayNum] ? RELAY_ON : RELAY_OFF);
    Serial.print("Relay ");
    Serial.print(relayNum);
    Serial.print(" (");
    Serial.print(names[relayNum]);
    Serial.print("): ");
    Serial.println(*states[relayNum] ? "ON" : "OFF");
}

// Safety function: turn all relays off
void allRelaysOff() {
    digitalWrite(RELAY_PUMP, RELAY_OFF);
    digitalWrite(RELAY_BOILER, RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, RELAY_OFF);
    digitalWrite(RELAY_WARMER, RELAY_OFF);
    pumpOn = false;
    heaterOn = false;
    solenoidOn = false;
    warmerOn = false;
    brewingActive = false;
}

// ============ BREW CYCLE STATE MACHINE ============

// Start a new brew cycle
void startBrewCycle() {
    if (brewState != IDLE) {
        Serial.println("Already running!");
        return;
    }

    Serial.println("=== STARTING BREW CYCLE ===");
    Serial.print("Target: ");
    Serial.print(targetTemp, 1);
    Serial.println("C");

    // Reset counters
    totalVolume = 0;
    flowPulseCount = 0;

    // Turn on cup warmer (optional)
    setRelay(RELAY_WARMER, true, "Cup Warmer");
    warmerOn = true;

    // Start preheating
    brewState = PREHEATING;
    preheatStartTime = millis();
    preheating = true;

    // Boiler ON
    setRelay(RELAY_BOILER, true, "Boiler");
    heaterOn = true;

    Serial.println("Preheating...");
}

// Update the brew cycle state machine (called every loop)
void updateBrewCycle() {
    unsigned long now = millis();

    switch (brewState) {
        case IDLE:
            // Nothing to do
            break;

        case PREHEATING:
            // Check if we've reached target temp
            if (boilerTemp >= MIN_BREW_TEMP) {
                Serial.println("Temperature reached!");
                Serial.print("Temp: ");
                Serial.print(boilerTemp, 1);
                Serial.println("C");

                // Start brewing - solenoid and pump ON together
                brewState = BREWING;
                brewStartTime = now;
                brewingActive = true;

                Serial.println("=== BREWING ===");

                // Solenoid and pump ON at the same time
                setRelay(RELAY_SOLENOID, true, "Solenoid");
                solenoidOn = true;

                setRelay(RELAY_PUMP, true, "Pump");
                pumpOn = true;
            }
            // Check for preheat timeout
            else if (now - preheatStartTime > PREHEAT_TIMEOUT_MS) {
                abortBrew("Preheat timeout - check heater!");
            }
            break;

        case BREWING:
            // Check if brew time is complete
            if (now - brewStartTime >= brewTimeMs) {
                // Normal completion
                stopBrew();
            }
            // Keep heater control running during brew
            updateHeaterControl();
            break;

        case FINISHING:
            // Brief pause then back to idle
            brewState = IDLE;
            break;
    }
}

// Normal brew completion
void stopBrew() {
    unsigned long brewTime = (millis() - brewStartTime) / 1000;

    // Turn off pump and solenoid
    setRelay(RELAY_PUMP, false, "Pump");
    pumpOn = false;

    setRelay(RELAY_SOLENOID, false, "Solenoid");
    solenoidOn = false;

    // Turn off boiler
    setRelay(RELAY_BOILER, false, "Boiler");
    heaterOn = false;

    // Cup warmer stays on a bit? Or turn off
    setRelay(RELAY_WARMER, false, "Cup Warmer");
    warmerOn = false;

    brewingActive = false;
    preheating = false;
    brewState = FINISHING;

    Serial.println("=== BREW COMPLETE ===");
    Serial.print("Time: ");
    Serial.print(brewTime);
    Serial.println(" seconds");
    Serial.print("Volume: ");
    Serial.print(totalVolume, 1);
    Serial.println(" mL");
    Serial.print("Final temp: ");
    Serial.print(boilerTemp, 1);
    Serial.println("C");
}

// Abort brew due to error or user cancel
void abortBrew(const char* reason) {
    Serial.print("!!! ABORT: ");
    Serial.println(reason);

    // Turn everything off
    allRelaysOff();

    brewState = IDLE;
    brewingActive = false;
    preheating = false;

    Serial.println("All relays OFF");
}

// ============ COMMAND HANDLING ============
void handleCommand(char cmd) {
    switch (cmd) {
        case 'h': case 'H': case '?':
            printHelp();
            break;

        case 's': case 'S':
            printStatus();
            break;

        case 'b': case 'B':  // Start/stop brew cycle
            if (brewState != IDLE) {
                abortBrew("User stopped");
            } else {
                startBrewCycle();
            }
            break;

        case 'p': case 'P':  // Toggle pump manually
            pumpOn = !pumpOn;
            setRelay(RELAY_PUMP, pumpOn, "Pump");
            break;

        case 'o': case 'O':  // Toggle solenoid
            solenoidOn = !solenoidOn;
            setRelay(RELAY_SOLENOID, solenoidOn, "Solenoid");
            break;

        case 'w': case 'W':  // Toggle cup warmer
            warmerOn = !warmerOn;
            setRelay(RELAY_WARMER, warmerOn, "Cup Warmer");
            break;

        case '+':  // Increase target temp
            targetTemp += 1;
            Serial.print("Target temp: ");
            Serial.print(targetTemp);
            Serial.println(" C");
            break;

        case '-':  // Decrease target temp
            targetTemp -= 1;
            Serial.print("Target temp: ");
            Serial.print(targetTemp);
            Serial.println(" C");
            break;

        case 'r': case 'R':  // Reset flow counter
            totalVolume = 0;
            flowPulseCount = 0;
            Serial.println("Flow counter reset");
            break;

        case 'x': case 'X':  // Emergency stop - all relays off immediately
            allRelaysOff();
            Serial.println("!!! EMERGENCY STOP - ALL OFF !!!");
            break;

        case '1':  // Direct relay toggles (for testing/debug)
        case '2':
        case '3':
        case '4':
            toggleRelayDirect(cmd - '0');
            break;

        case '\n': case '\r': case ' ':
            // Ignore whitespace
            break;

        default:
            // Input validation: only print warning for printable characters
            if (cmd >= 32 && cmd <= 126) {
                Serial.print("Unknown command: '");
                Serial.print(cmd);
                Serial.println("' - press H for help");
            }
            break;
    }
}

// ============ STATUS ============
void printStatus() {
    Serial.println("\n========== STATUS ==========");
    Serial.print("Boiler Temp:  ");
    Serial.print(boilerTemp, 1);
    Serial.print(" C (target: ");
    Serial.print(targetTemp, 1);
    Serial.println(" C)");

    Serial.print("Flow Rate:    ");
    Serial.print(flowRate, 2);
    Serial.println(" mL/sec");

    Serial.print("Total Volume: ");
    Serial.print(totalVolume, 1);
    Serial.println(" mL");

    Serial.print("Flow Pulses:  ");
    Serial.println(flowPulseCount);

    Serial.println();
    Serial.print("Pump:     "); Serial.println(pumpOn ? "ON" : "OFF");
    Serial.print("Boiler:   "); Serial.println(heaterOn ? "ON" : "OFF");
    Serial.print("Solenoid: "); Serial.println(solenoidOn ? "ON" : "OFF");
    Serial.print("Warmer:   "); Serial.println(warmerOn ? "ON" : "OFF");

    const char* stateNames[] = {"IDLE", "PREHEATING", "BREWING", "FINISHING"};
    Serial.print("State:    "); Serial.println(stateNames[brewState]);
    Serial.println("============================\n");
}

void printHelp() {
    Serial.println("Commands:");
    Serial.println("  H     Help");
    Serial.println("  S     Status");
    Serial.println("  B     Start/Stop Brew Cycle");
    Serial.println("  P     Toggle Pump");
    Serial.println("  O     Toggle Solenoid");
    Serial.println("  W     Toggle Cup Warmer");
    Serial.println("  +/-   Adjust target temp");
    Serial.println("  R     Reset flow counter");
    Serial.println("  X     EMERGENCY STOP");
    Serial.println("  1-4   Direct relay toggle");
    Serial.println();
    Serial.println("BOOTSEL button also starts/stops brew");
    Serial.print("Web UI: http://");
    Serial.println(WiFi.localIP());
    Serial.println();
}

// ============ WIFI & WEB SERVER ============

void setupWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi failed - continuing without network");
    }
}

String getStatusJson() {
    const char* stateNames[] = {"IDLE", "PREHEATING", "BREWING", "FINISHING"};
    unsigned long brewElapsed = (brewState == BREWING) ? (millis() - brewStartTime) / 1000 : 0;

    String json = "{";
    json += "\"temp\":" + String(boilerTemp, 1) + ",";
    json += "\"target\":" + String(targetTemp, 1) + ",";
    json += "\"flow\":" + String(flowRate, 2) + ",";
    json += "\"volume\":" + String(totalVolume, 1) + ",";
    json += "\"state\":\"" + String(stateNames[brewState]) + "\",";
    json += "\"pump\":" + String(pumpOn ? "true" : "false") + ",";
    json += "\"boiler\":" + String(heaterOn ? "true" : "false") + ",";
    json += "\"solenoid\":" + String(solenoidOn ? "true" : "false") + ",";
    json += "\"warmer\":" + String(warmerOn ? "true" : "false") + ",";
    json += "\"brewTime\":" + String(brewTimeMs / 1000) + ",";
    json += "\"brewElapsed\":" + String(brewElapsed) + ",";
    json += "\"minTemp\":" + String(MIN_BREW_TEMP, 0);
    json += "}";
    return json;
}

String getWebPage() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <title>Espresso</title>
    <style>
        * { box-sizing: border-box; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; background: #0d1117; color: #c9d1d9; margin: 0; padding: 15px; }
        .header { display: flex; align-items: center; gap: 12px; margin-bottom: 15px; }
        .logo { width: 50px; height: 50px; }
        h1 { margin: 0; font-size: 24px; color: #58a6ff; }
        .card { background: #161b22; border: 1px solid #30363d; border-radius: 12px; padding: 16px; margin: 12px 0; }
        .temp-display { text-align: center; }
        .temp { font-size: 56px; font-weight: bold; color: #f78166; }
        .temp-info { color: #8b949e; margin-top: 4px; }
        .state-bar { display: flex; align-items: center; justify-content: space-between; flex-wrap: wrap; gap: 10px; }
        .state { font-size: 18px; font-weight: bold; padding: 8px 16px; border-radius: 20px; }
        .state.IDLE { background: #238636; color: #fff; }
        .state.PREHEATING { background: #d29922; color: #000; }
        .state.BREWING { background: #f78166; color: #000; }
        .state.FINISHING { background: #8957e5; color: #fff; }
        .timer { font-size: 24px; font-weight: bold; color: #58a6ff; }
        .relays { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-top: 12px; }
        .relay-btn { padding: 14px; border-radius: 8px; border: 2px solid #30363d; cursor: pointer; font-size: 14px; font-weight: 600; transition: all 0.2s; text-align: center; }
        .relay-btn.off { background: #21262d; color: #8b949e; }
        .relay-btn.on { background: #238636; color: #fff; border-color: #2ea043; }
        .relay-btn:active { transform: scale(0.95); }
        .brew-btns { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .btn { padding: 18px; border: none; border-radius: 10px; font-size: 18px; font-weight: bold; cursor: pointer; transition: all 0.2s; }
        .btn:active { transform: scale(0.95); }
        .btn-brew { background: linear-gradient(135deg, #238636, #2ea043); color: #fff; }
        .btn-stop { background: linear-gradient(135deg, #da3633, #f85149); color: #fff; }
        .settings { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
        .setting { background: #21262d; border-radius: 8px; padding: 12px; text-align: center; }
        .setting-label { font-size: 12px; color: #8b949e; margin-bottom: 6px; }
        .setting-value { font-size: 20px; font-weight: bold; color: #58a6ff; }
        .setting-btns { display: flex; justify-content: center; gap: 8px; margin-top: 8px; }
        .adj-btn { width: 36px; height: 36px; border-radius: 50%; border: none; font-size: 18px; font-weight: bold; cursor: pointer; }
        .adj-btn.minus { background: #f85149; color: #fff; }
        .adj-btn.plus { background: #238636; color: #fff; }
        .flow-info { display: flex; justify-content: space-around; text-align: center; }
        .flow-item { flex: 1; }
        .flow-value { font-size: 24px; font-weight: bold; color: #79c0ff; }
        .flow-label { font-size: 12px; color: #8b949e; }
        .reset-btn { background: #30363d; color: #8b949e; border: none; padding: 8px 16px; border-radius: 6px; cursor: pointer; margin-top: 10px; font-size: 12px; }
    </style>
</head>
<body>
    <div class="header">
        <svg class="logo" viewBox="0 0 100 100">
            <defs>
                <linearGradient id="steam" x1="0%" y1="100%" x2="0%" y2="0%">
                    <stop offset="0%" style="stop-color:#8b949e;stop-opacity:0.8"/>
                    <stop offset="100%" style="stop-color:#8b949e;stop-opacity:0"/>
                </linearGradient>
                <linearGradient id="cup" x1="0%" y1="0%" x2="0%" y2="100%">
                    <stop offset="0%" style="stop-color:#f78166"/>
                    <stop offset="100%" style="stop-color:#da3633"/>
                </linearGradient>
            </defs>
            <!-- Steam -->
            <path d="M35 35 Q32 25, 38 15" stroke="url(#steam)" stroke-width="3" fill="none" stroke-linecap="round"/>
            <path d="M50 30 Q47 20, 53 10" stroke="url(#steam)" stroke-width="3" fill="none" stroke-linecap="round"/>
            <path d="M65 35 Q62 25, 68 15" stroke="url(#steam)" stroke-width="3" fill="none" stroke-linecap="round"/>
            <!-- Cup body -->
            <path d="M20 45 L25 85 Q27 92, 35 92 L65 92 Q73 92, 75 85 L80 45 Z" fill="url(#cup)"/>
            <!-- Handle -->
            <path d="M80 50 Q95 50, 95 65 Q95 80, 80 80" stroke="#f78166" stroke-width="6" fill="none" stroke-linecap="round"/>
            <!-- Coffee surface -->
            <ellipse cx="50" cy="48" rx="28" ry="6" fill="#3d2817"/>
            <!-- Shine -->
            <path d="M30 55 Q35 52, 45 55" stroke="rgba(255,255,255,0.3)" stroke-width="2" fill="none" stroke-linecap="round"/>
        </svg>
        <h1>Espresso Controller</h1>
    </div>

    <div class="card temp-display">
        <div class="temp" id="temp">--</div>
        <div class="temp-info">Current / Target: <span id="target">--</span>C (min <span id="minTemp">--</span>C)</div>
    </div>

    <div class="card">
        <div class="state-bar">
            <span class="state IDLE" id="state">IDLE</span>
            <span class="timer" id="timer">--</span>
        </div>
        <div class="relays">
            <button class="relay-btn off" id="pump" onclick="toggle('pump')">Pump</button>
            <button class="relay-btn off" id="boiler" onclick="toggle('boiler')">Boiler</button>
            <button class="relay-btn off" id="solenoid" onclick="toggle('solenoid')">Solenoid</button>
            <button class="relay-btn off" id="warmer" onclick="toggle('warmer')">Warmer</button>
        </div>
    </div>

    <div class="card brew-btns">
        <button class="btn btn-brew" onclick="brew()">BREW</button>
        <button class="btn btn-stop" onclick="stop()">STOP</button>
    </div>

    <div class="card settings">
        <div class="setting">
            <div class="setting-label">Target Temp (C)</div>
            <div class="setting-value" id="targetVal">93</div>
            <div class="setting-btns">
                <button class="adj-btn minus" onclick="adj('temp',-1)">-</button>
                <button class="adj-btn plus" onclick="adj('temp',1)">+</button>
            </div>
        </div>
        <div class="setting">
            <div class="setting-label">Brew Time (sec)</div>
            <div class="setting-value" id="brewTimeVal">25</div>
            <div class="setting-btns">
                <button class="adj-btn minus" onclick="adj('time',-5)">-</button>
                <button class="adj-btn plus" onclick="adj('time',5)">+</button>
            </div>
        </div>
    </div>

    <div class="card">
        <div class="flow-info">
            <div class="flow-item">
                <div class="flow-value" id="flow">0</div>
                <div class="flow-label">mL/sec</div>
            </div>
            <div class="flow-item">
                <div class="flow-value" id="volume">0</div>
                <div class="flow-label">mL total</div>
            </div>
        </div>
        <center><button class="reset-btn" onclick="fetch('/reset')">Reset Flow Counter</button></center>
    </div>

    <script>
        function update() {
            fetch('/status').then(r=>r.json()).then(d => {
                document.getElementById('temp').textContent = d.temp + 'C';
                document.getElementById('target').textContent = d.target;
                document.getElementById('targetVal').textContent = d.target;
                document.getElementById('minTemp').textContent = d.minTemp;
                document.getElementById('flow').textContent = d.flow;
                document.getElementById('volume').textContent = d.volume;
                document.getElementById('brewTimeVal').textContent = d.brewTime;

                // State badge
                let stateEl = document.getElementById('state');
                stateEl.textContent = d.state;
                stateEl.className = 'state ' + d.state;

                // Timer
                let timer = document.getElementById('timer');
                if (d.state === 'BREWING') {
                    timer.textContent = d.brewElapsed + '/' + d.brewTime + 's';
                } else if (d.state === 'PREHEATING') {
                    timer.textContent = 'Heating...';
                } else {
                    timer.textContent = '';
                }

                // Relay buttons
                ['pump','boiler','solenoid','warmer'].forEach(r => {
                    let el = document.getElementById(r);
                    el.className = 'relay-btn ' + (d[r] ? 'on' : 'off');
                });
            }).catch(e => {});
        }
        function toggle(r) { fetch('/toggle?r=' + r); }
        function brew() { fetch('/brew'); }
        function stop() { fetch('/stop'); }
        function adj(what, delta) { fetch('/adj?what=' + what + '&delta=' + delta); }
        setInterval(update, 500);
        update();
    </script>
</body>
</html>
)rawliteral";
    return html;
}

void setupWebServer() {
    // Main page
    server.on("/", []() {
        server.send(200, "text/html", getWebPage());
    });

    // Status API (JSON)
    server.on("/status", []() {
        server.send(200, "application/json", getStatusJson());
    });

    // Brew button
    server.on("/brew", []() {
        if (brewState == IDLE) {
            startBrewCycle();
            server.send(200, "text/plain", "Brewing started");
        } else {
            server.send(200, "text/plain", "Already running");
        }
    });

    // Stop button
    server.on("/stop", []() {
        if (brewState != IDLE) {
            abortBrew("Web stop");
        }
        server.send(200, "text/plain", "Stopped");
    });

    // Toggle individual relays
    server.on("/toggle", []() {
        String relay = server.arg("r");
        if (relay == "pump") {
            pumpOn = !pumpOn;
            setRelay(RELAY_PUMP, pumpOn, "Pump");
        } else if (relay == "boiler") {
            heaterOn = !heaterOn;
            setRelay(RELAY_BOILER, heaterOn, "Boiler");
        } else if (relay == "solenoid") {
            solenoidOn = !solenoidOn;
            setRelay(RELAY_SOLENOID, solenoidOn, "Solenoid");
        } else if (relay == "warmer") {
            warmerOn = !warmerOn;
            setRelay(RELAY_WARMER, warmerOn, "Warmer");
        }
        server.send(200, "text/plain", "OK");
    });

    // Adjust settings (temp or brew time)
    server.on("/adj", []() {
        String what = server.arg("what");
        int delta = server.arg("delta").toInt();

        if (what == "temp") {
            targetTemp += delta;
            if (targetTemp < 70) targetTemp = 70;
            if (targetTemp > 100) targetTemp = 100;
            Serial.print("Target temp: ");
            Serial.print(targetTemp);
            Serial.println(" C");
        } else if (what == "time") {
            brewTimeMs += delta * 1000;
            if (brewTimeMs < 5000) brewTimeMs = 5000;
            if (brewTimeMs > 60000) brewTimeMs = 60000;
            Serial.print("Brew time: ");
            Serial.print(brewTimeMs / 1000);
            Serial.println(" sec");
        }
        server.send(200, "text/plain", "OK");
    });

    // Reset flow counter
    server.on("/reset", []() {
        totalVolume = 0;
        flowPulseCount = 0;
        Serial.println("Flow counter reset");
        server.send(200, "text/plain", "OK");
    });

    server.begin();
    Serial.println("Web server started");
}
