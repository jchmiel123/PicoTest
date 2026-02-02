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
#include <LEAmDNS.h>
// OTA removed - Updater.h doesn't work on RP2350 (Error 4). See ForgeRepo/CAPABILITIES.md

// ============ WIFI CONFIG ============
// Multiple networks - will try each in order
struct WiFiNetwork {
    const char* ssid;
    const char* pass;
};

const WiFiNetwork WIFI_NETWORKS[] = {
    {"Founders3-Office", "Gu1fR3serVe13"},
    {"DropitlikeitsHotspot", "Nutmeg21"}
};
const int NUM_NETWORKS = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// Fallback AP mode settings
const char* AP_SSID = "Espresso";
const char* AP_PASS = "coffee123";  // Min 8 chars
bool apMode = false;

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
// Your board is ACTIVE HIGH - HIGH turns relay ON
const bool RELAY_ACTIVE_LOW = false;

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
bool useFahrenheit = false; // Display in F if true

// Info page password
const char* INFO_PASSWORD = "Coffee4Me!";

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
void updateWiFi();
void setupWebServer();
String getStatusJson();
String getWebPage();

// WiFi state (defined here for forward reference)
extern int wifiNetworkIndex;
extern unsigned long wifiConnectStart;
extern bool wifiConnecting;
extern bool wifiSetupDone;
extern int wifiRetryCount;
extern unsigned int pollInterval;
extern bool apMode;

// ============ SETUP ============
void setup() {
    // CRITICAL: Set relay pins HIGH (OFF) IMMEDIATELY before anything else
    // Active LOW relays will turn ON if pins float during boot!
    digitalWrite(RELAY_PUMP, RELAY_OFF);
    digitalWrite(RELAY_BOILER, RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, RELAY_OFF);
    digitalWrite(RELAY_WARMER, RELAY_OFF);
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_BOILER, OUTPUT);
    pinMode(RELAY_SOLENOID, OUTPUT);
    pinMode(RELAY_WARMER, OUTPUT);

    Serial.begin(115200);
    delay(2000);

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

    // Setup WiFi (non-blocking - connects in background)
    setupWiFi();
    // Web server starts after WiFi connects in updateWiFi()

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

    // Non-blocking WiFi connection
    updateWiFi();

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

    // Auto status print every 2 seconds if not idle, or every 30s if idle (with IP)
    static unsigned long lastIdleStatus = 0;
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

    // Print IP and status every 30 seconds when idle
    if (brewState == IDLE && (now - lastIdleStatus >= 30000)) {
        lastIdleStatus = now;
        Serial.print("[IDLE] Temp: ");
        Serial.print(boilerTemp, 1);
        Serial.print("C | ");
        if (apMode) {
            Serial.print("AP: ");
            Serial.print(AP_SSID);
            Serial.print(" | http://");
            Serial.println(WiFi.softAPIP());
        } else if (WiFi.status() == WL_CONNECTED) {
            Serial.print("http://");
            Serial.print(WiFi.localIP());
            Serial.print(" | espresso.local | ");
            Serial.print(WiFi.RSSI());
            Serial.println("dBm");
        } else {
            Serial.println("WiFi disconnected");
        }
    }

    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }

    // Handle web requests (WiFi STA or AP mode)
    if (wifiSetupDone && (apMode || WiFi.status() == WL_CONNECTED)) {
        server.handleClient();
        if (!apMode) MDNS.update();  // mDNS only works in STA mode
    }
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

// ============ EXPLICIT RELAY STATE CONTROL ============
// Set ALL relays to explicit states - no assumptions, no toggles
// This prevents "oops coffee everywhere" situations

void setAllRelays(bool pump, bool boiler, bool solenoid, bool warmer, const char* context) {
    Serial.print("[");
    Serial.print(context);
    Serial.print("] Relays: P=");
    Serial.print(pump ? "ON" : "off");
    Serial.print(" B=");
    Serial.print(boiler ? "ON" : "off");
    Serial.print(" S=");
    Serial.print(solenoid ? "ON" : "off");
    Serial.print(" W=");
    Serial.println(warmer ? "ON" : "off");

    // Set all at once
    digitalWrite(RELAY_PUMP, pump ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_BOILER, boiler ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, solenoid ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_WARMER, warmer ? RELAY_ON : RELAY_OFF);

    // Update state tracking
    pumpOn = pump;
    heaterOn = boiler;
    solenoidOn = solenoid;
    warmerOn = warmer;
}

// Safety function: turn all relays off
void allRelaysOff() {
    setAllRelays(false, false, false, false, "ALL OFF");
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

    // PREHEAT STATE: Boiler ON, Warmer ON, Pump OFF, Solenoid OFF
    setAllRelays(false, true, false, true, "PREHEAT");

    brewState = PREHEATING;
    preheatStartTime = millis();
    preheating = true;
    brewingActive = true;

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

                // BREW STATE: Pump ON, Solenoid ON, Boiler ON (for temp control), Warmer ON
                setAllRelays(true, true, true, true, "BREW");

                brewState = BREWING;
                brewStartTime = now;

                Serial.println("=== BREWING ===");
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

    // STOP STATE: ALL OFF - explicit, no ambiguity
    setAllRelays(false, false, false, false, "STOP");

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

    // ABORT: ALL OFF immediately - no exceptions
    setAllRelays(false, false, false, false, "ABORT");

    brewState = IDLE;
    brewingActive = false;
    preheating = false;
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

        case 'c': case 'C':  // Retry WiFi connection
            Serial.println("Retrying WiFi...");
            apMode = false;
            wifiSetupDone = false;
            wifiConnecting = false;
            wifiNetworkIndex = 0;
            wifiRetryCount = 0;
            WiFi.disconnect();
            WiFi.mode(WIFI_STA);
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
    Serial.println("  C     Retry WiFi connection");
    Serial.println("  1-4   Direct relay toggle");
    Serial.println();
    Serial.println("BOOTSEL button also starts/stops brew");
    Serial.print("Web UI: http://");
    Serial.println(WiFi.localIP());
    Serial.println();
}

// ============ WIFI & WEB SERVER ============

// WiFi state for non-blocking connection
int wifiNetworkIndex = 0;
unsigned long wifiConnectStart = 0;
bool wifiConnecting = false;
bool wifiSetupDone = false;
int wifiRetryCount = 0;

void startAPMode() {
    Serial.println("Starting AP mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    apMode = true;
    wifiSetupDone = true;

    Serial.println("=================================");
    Serial.print("AP Mode: ");
    Serial.println(AP_SSID);
    Serial.print("Password: ");
    Serial.println(AP_PASS);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("=================================");

    setupWebServer();
}

void startWiFiConnect() {
    if (wifiNetworkIndex >= NUM_NETWORKS) {
        wifiNetworkIndex = 0;  // Loop back for retry
        wifiRetryCount++;
        if (wifiRetryCount > 2) {
            // Fall back to AP mode instead of giving up
            Serial.println("WiFi failed - switching to AP mode");
            startAPMode();
            return;
        }
    }

    Serial.print("Trying WiFi: ");
    Serial.println(WIFI_NETWORKS[wifiNetworkIndex].ssid);
    WiFi.begin(WIFI_NETWORKS[wifiNetworkIndex].ssid, WIFI_NETWORKS[wifiNetworkIndex].pass);
    wifiConnectStart = millis();
    wifiConnecting = true;
}

void updateWiFi() {
    if (wifiSetupDone) return;

    if (!wifiConnecting) {
        // Initial delay before first connect attempt
        if (millis() > 3000) {
            startWiFiConnect();
        }
        return;
    }

    // Check connection status (non-blocking)
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("Network: ");
        Serial.println(WIFI_NETWORKS[wifiNetworkIndex].ssid);
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        wifiSetupDone = true;
        wifiConnecting = false;
        setupWebServer();
        return;
    }

    // Timeout with exponential backoff: 5s, 8s, 12s per retry cycle
    unsigned long timeout = 5000 + (wifiRetryCount * 3000);  // 5s, 8s, 11s...
    if (timeout > 15000) timeout = 15000;  // Cap at 15s

    if (millis() - wifiConnectStart > timeout) {
        Serial.print(" timeout (");
        Serial.print(timeout / 1000);
        Serial.println("s)");
        WiFi.disconnect();
        wifiConnecting = false;
        wifiNetworkIndex++;
        // Small delay then try next
        delay(500);
        startWiFiConnect();
    }
}

void setupWiFi() {
    // Just initialize - actual connection happens in updateWiFi() non-blocking
    WiFi.mode(WIFI_STA);
    Serial.println("WiFi will connect in background...");
}

// Helper: convert C to F
float toFahrenheit(float c) {
    return c * 9.0 / 5.0 + 32.0;
}

String getStatusJson() {
    const char* stateNames[] = {"IDLE", "PREHEATING", "BREWING", "FINISHING"};
    unsigned long brewElapsed = (brewState == BREWING) ? (millis() - brewStartTime) / 1000 : 0;

    String json = "{";
    json += "\"temp\":" + String(boilerTemp, 1) + ",";
    json += "\"tempF\":" + String(toFahrenheit(boilerTemp), 1) + ",";
    json += "\"target\":" + String(targetTemp, 1) + ",";
    json += "\"targetF\":" + String(toFahrenheit(targetTemp), 1) + ",";
    json += "\"useF\":" + String(useFahrenheit ? "true" : "false") + ",";
    json += "\"flow\":" + String(flowRate, 2) + ",";
    json += "\"volume\":" + String(totalVolume, 1) + ",";
    json += "\"state\":\"" + String(stateNames[brewState]) + "\",";
    json += "\"pump\":" + String(pumpOn ? "true" : "false") + ",";
    json += "\"boiler\":" + String(heaterOn ? "true" : "false") + ",";
    json += "\"solenoid\":" + String(solenoidOn ? "true" : "false") + ",";
    json += "\"warmer\":" + String(warmerOn ? "true" : "false") + ",";
    json += "\"brewTime\":" + String(brewTimeMs / 1000) + ",";
    json += "\"brewElapsed\":" + String(brewElapsed) + ",";
    json += "\"minTemp\":" + String(MIN_BREW_TEMP, 0) + ",";
    json += "\"wifiConnected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"ssid\":\"" + WiFi.SSID() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"uptime\":" + String(millis() / 1000) + ",";
    json += "\"freeHeap\":" + String(rp2040.getFreeHeap()) + ",";
    json += "\"pollInterval\":" + String(pollInterval);
    json += "}";
    return json;
}

// Poll interval in ms - adjustable via web
unsigned int pollInterval = 2000;  // Default 2 seconds

// HTML stored in PROGMEM (flash) - saves RAM
const char MAIN_PAGE[] PROGMEM = R"rawliteral(<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><meta charset="UTF-8"><title>Espresso</title><style>
*{box-sizing:border-box}body{font-family:Arial;background:#0d1117;color:#c9d1d9;margin:0;padding:10px}
.h{display:flex;align-items:center;gap:10px;margin-bottom:10px}.logo{width:40px;height:40px}h1{margin:0;font-size:20px;color:#58a6ff}
.c{background:#161b22;border:1px solid #30363d;border-radius:10px;padding:12px;margin:8px 0}
.t{font-size:48px;font-weight:bold;color:#f78166;text-align:center}.ti{color:#8b949e;text-align:center}
.sb{display:flex;align-items:center;justify-content:space-between;gap:8px}
.st{font-size:16px;font-weight:bold;padding:6px 12px;border-radius:16px}
.st.IDLE{background:#238636;color:#fff}.st.PREHEATING{background:#d29922;color:#000}
.st.BREWING{background:#f78166;color:#000}.st.FINISHING{background:#8957e5;color:#fff}
.tm{font-size:20px;font-weight:bold;color:#58a6ff}
.rl{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
.rb{padding:12px;border-radius:6px;border:2px solid #30363d;cursor:pointer;font-size:13px;font-weight:600;text-align:center}
.rb.off{background:#21262d;color:#8b949e}.rb.on{background:#238636;color:#fff;border-color:#2ea043}
.bb{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.bn{padding:14px;border:none;border-radius:8px;font-size:16px;font-weight:bold;cursor:pointer}
.bg{background:#238636;color:#fff}.br{background:#da3633;color:#fff}
.ss{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.se{background:#21262d;border-radius:6px;padding:10px;text-align:center}
.sl{font-size:11px;color:#8b949e}.sv{font-size:18px;font-weight:bold;color:#58a6ff}
.ab{width:32px;height:32px;border-radius:50%;border:none;font-size:16px;font-weight:bold;cursor:pointer;margin:4px}
.am{background:#f85149;color:#fff}.ap{background:#238636;color:#fff}
.fi{display:flex;justify-content:space-around;text-align:center}
.fv{font-size:20px;font-weight:bold;color:#79c0ff}.fl{font-size:11px;color:#8b949e}
.rt{background:#30363d;color:#8b949e;border:none;padding:6px 12px;border-radius:4px;cursor:pointer;font-size:11px}
.ib{background:#58a6ff;color:#0d1117;border:none;padding:8px 16px;border-radius:6px;cursor:pointer;font-weight:bold;font-size:12px}
.tu{cursor:pointer;text-decoration:underline}
</style></head><body>
<div class="h"><svg class="logo" viewBox="0 0 100 100"><defs><linearGradient id="s" x1="0%" y1="100%" x2="0%" y2="0%"><stop offset="0%" stop-color="#8b949e" stop-opacity="0.8"/><stop offset="100%" stop-color="#8b949e" stop-opacity="0"/></linearGradient><linearGradient id="u" x1="0%" y1="0%" x2="0%" y2="100%"><stop offset="0%" stop-color="#f78166"/><stop offset="100%" stop-color="#da3633"/></linearGradient></defs><path d="M35 35Q32 25 38 15" stroke="url(#s)" stroke-width="3" fill="none"/><path d="M50 30Q47 20 53 10" stroke="url(#s)" stroke-width="3" fill="none"/><path d="M65 35Q62 25 68 15" stroke="url(#s)" stroke-width="3" fill="none"/><path d="M20 45L25 85Q27 92 35 92L65 92Q73 92 75 85L80 45Z" fill="url(#u)"/><path d="M80 50Q95 50 95 65Q95 80 80 80" stroke="#f78166" stroke-width="6" fill="none"/><ellipse cx="50" cy="48" rx="28" ry="6" fill="#3d2817"/></svg><h1>Espresso</h1><button class="ib" onclick="showInfo()" style="margin-left:auto">Info</button></div>
<div class="c"><div class="t" id="temp">--</div><div class="ti">Target: <span id="target">--</span><span id="unit" class="tu" onclick="toggleUnit()">C</span></div></div>
<div class="c"><div class="sb"><span class="st IDLE" id="state">IDLE</span><span class="tm" id="timer"></span></div>
<div class="rl"><button class="rb off" id="pump" onclick="T('pump')">Pump</button><button class="rb off" id="boiler" onclick="T('boiler')">Boiler</button><button class="rb off" id="solenoid" onclick="T('solenoid')">Solenoid</button><button class="rb off" id="warmer" onclick="T('warmer')">Warmer</button></div></div>
<div class="c bb"><button class="bn bg" onclick="B()">BREW</button><button class="bn br" onclick="S()">STOP</button></div>
<div class="c ss"><div class="se"><div class="sl">Temp (<span id="tul">C</span>)</div><div class="sv" id="tv">93</div><button class="ab am" onclick="A('temp',-1)">-</button><button class="ab ap" onclick="A('temp',1)">+</button></div>
<div class="se"><div class="sl">Time (s)</div><div class="sv" id="bv">25</div><button class="ab am" onclick="A('time',-5)">-</button><button class="ab ap" onclick="A('time',5)">+</button></div></div>
<div class="c"><div class="fi"><div><div class="fv" id="flow">0</div><div class="fl">mL/s</div></div><div><div class="fv" id="vol">0</div><div class="fl">mL</div></div></div><center><button class="rt" onclick="fetch('/reset')">Reset</button></center></div>
<div class="c" style="font-size:11px;color:#8b949e"><span id="wifi">WiFi: --</span> | <span id="rssi">--</span>dBm | <span id="ip">--</span></div>
<div id="infopanel" style="display:none;position:fixed;top:0;left:0;right:0;bottom:0;background:rgba(0,0,0,0.95);padding:20px;z-index:100;overflow-y:auto">
<div style="max-width:400px;margin:auto">
<h2 style="color:#58a6ff;margin-top:0">System Info</h2>
<div id="infolock">
<p style="color:#8b949e">Enter password to access system controls:</p>
<input type="password" id="infopw" style="background:#21262d;border:1px solid #30363d;color:#c9d1d9;padding:10px;width:100%;border-radius:6px;margin-bottom:10px" placeholder="Password">
<button class="ib" onclick="checkPw()">Unlock</button>
<button class="rt" onclick="hideInfo()" style="margin-left:10px">Cancel</button>
</div>
<div id="infocontent" style="display:none">
<div class="c"><div class="sl">Pico 2W (RP2350)</div>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:8px">
<div><span class="fl">Uptime:</span><br><span class="fv" id="iup">--</span></div>
<div><span class="fl">Free RAM:</span><br><span class="fv" id="iheap">--</span></div>
<div><span class="fl">WiFi SSID:</span><br><span class="fv" id="issid">--</span></div>
<div><span class="fl">Signal:</span><br><span class="fv" id="irssi">--</span>dBm</div>
<div><span class="fl">IP Address:</span><br><span class="fv" id="iip">--</span></div>
<div><span class="fl">Temp Unit:</span><br><span class="fv" id="iunit">C</span></div>
<div><span class="fl">Poll (ms):</span><br><span class="fv" id="ipoll">2000</span></div>
</div></div>
<div class="c"><div class="sl">System Controls</div>
<div style="display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px">
<button class="bn" style="background:#d29922;color:#000" onclick="sysCmd('reboot')">Reboot</button>
<button class="bn br" onclick="sysCmd('sleep')">Deep Sleep</button>
<button class="bn" style="background:#8957e5;color:#fff" onclick="sysCmd('alloff')">All Relays Off</button>
<button class="bn" style="background:#238636;color:#fff" onclick="toggleUnit()">Toggle C/F</button>
<button class="bn" style="background:#30363d;color:#c9d1d9" onclick="sysCmd('defaults')">Reset Defaults</button>
<button class="bn" style="background:#30363d;color:#c9d1d9" onclick="sysCmd('wifireset')">WiFi Reconnect</button>
<button class="bn" style="background:#21262d;color:#8b949e" onclick="adjPoll(-500)">Poll -</button>
<button class="bn" style="background:#21262d;color:#8b949e" onclick="adjPoll(500)">Poll +</button>
</div></div>
<div class="c"><div class="sl">GPIO Pins</div>
<div style="font-size:11px;color:#8b949e;margin-top:8px">
GP2: Pump | GP3: Boiler | GP4: Solenoid | GP5: Warmer<br>
GP6: Flow Sensor | GP26: Thermistor (ADC0)<br>
Active HIGH relays (HIGH = ON)
</div></div>
<button class="bn" style="background:#30363d;color:#c9d1d9;width:100%;margin-top:10px" onclick="hideInfo()">Close</button>
</div></div></div>
<script>
var uF=false;
function U(){fetch('/status').then(r=>r.json()).then(d=>{
uF=d.useF;var tp=uF?d.tempF:d.temp;var tg=uF?d.targetF:d.target;var u=uF?'F':'C';
document.getElementById('temp').textContent=tp.toFixed(1)+u;
document.getElementById('target').textContent=tg.toFixed(1);
document.getElementById('unit').textContent=u;
document.getElementById('tul').textContent=u;
document.getElementById('tv').textContent=tg.toFixed(0);
document.getElementById('flow').textContent=d.flow;
document.getElementById('vol').textContent=d.volume;
document.getElementById('bv').textContent=d.brewTime;
var s=document.getElementById('state');s.textContent=d.state;s.className='st '+d.state;
var t=document.getElementById('timer');t.textContent=d.state==='BREWING'?d.brewElapsed+'/'+d.brewTime+'s':d.state==='PREHEATING'?'Heating...':'';
['pump','boiler','solenoid','warmer'].forEach(r=>{document.getElementById(r).className='rb '+(d[r]?'on':'off')});
document.getElementById('wifi').textContent='WiFi: '+(d.wifiConnected?d.ssid:'--');
document.getElementById('rssi').textContent=d.rssi;
document.getElementById('ip').textContent=d.ip;
document.getElementById('iup').textContent=formatUptime(d.uptime);
document.getElementById('iheap').textContent=(d.freeHeap/1024).toFixed(1)+'KB';
document.getElementById('issid').textContent=d.ssid;
document.getElementById('irssi').textContent=d.rssi;
document.getElementById('iip').textContent=d.ip;
document.getElementById('iunit').textContent=u;
document.getElementById('ipoll').textContent=d.pollInterval;
}).catch(e=>{})}
function formatUptime(s){var h=Math.floor(s/3600);var m=Math.floor((s%3600)/60);var sec=s%60;return h+'h '+m+'m '+sec+'s'}
var L=0;function T(r){if(L)return;L=1;fetch('/toggle?r='+r).then(x=>x.json()).then(d=>{L=0;['pump','boiler','solenoid','warmer'].forEach(r=>{document.getElementById(r).className='rb '+(d[r]?'on':'off')})}).catch(e=>{L=0})}
function B(){fetch('/brew')}function S(){fetch('/stop')}function A(w,d){fetch('/adj?what='+w+'&delta='+d)}
function toggleUnit(){fetch('/togglef').then(r=>r.text()).then(u=>{uF=(u==='F');U()})}
function showInfo(){document.getElementById('infopanel').style.display='block';document.getElementById('infolock').style.display='block';document.getElementById('infocontent').style.display='none';document.getElementById('infopw').value='';document.getElementById('infopw').focus()}
function hideInfo(){document.getElementById('infopanel').style.display='none'}
function checkPw(){fetch('/checkpw?pw='+encodeURIComponent(document.getElementById('infopw').value)).then(r=>r.text()).then(x=>{if(x==='OK'){document.getElementById('infolock').style.display='none';document.getElementById('infocontent').style.display='block'}else{alert('Wrong password')}})}
function sysCmd(c){if(c==='reboot'&&!confirm('Reboot the Pico?'))return;if(c==='sleep'&&!confirm('Enter deep sleep? Power cycle to wake.'))return;if(c==='defaults'&&!confirm('Reset settings to defaults?'))return;fetch('/sys?cmd='+c).then(r=>r.text()).then(x=>{alert(x);if(c==='reboot'||c==='sleep')hideInfo()})}
function adjPoll(d){fetch('/poll?delta='+d).then(r=>r.text()).then(x=>{document.getElementById('ipoll').textContent=x})}
var PI=2000;setInterval(U,PI);U()
</script></body></html>)rawliteral";

String getWebPage() {
    // Read from PROGMEM and inject current poll interval
    String html = FPSTR(MAIN_PAGE);
    html.replace("var PI=2000", "var PI=" + String(pollInterval));
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

    // Stop button - ALWAYS turns everything off, no matter what state
    server.on("/stop", []() {
        abortBrew("Web stop");  // Always stop, even if IDLE - safety first
        server.send(200, "text/plain", "Stopped");
    });

    // Toggle individual relays - returns JSON with current state
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
        // Return full status so UI stays in sync
        server.send(200, "application/json", getStatusJson());
    });

    // Adjust settings (temp or brew time)
    server.on("/adj", []() {
        String what = server.arg("what");
        int delta = server.arg("delta").toInt();

        if (what == "temp") {
            targetTemp += delta;
            if (targetTemp < 50) targetTemp = 50;   // Lower limit for testing
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

    // Toggle Fahrenheit mode
    server.on("/togglef", []() {
        useFahrenheit = !useFahrenheit;
        Serial.print("Temp unit: ");
        Serial.println(useFahrenheit ? "Fahrenheit" : "Celsius");
        server.send(200, "text/plain", useFahrenheit ? "F" : "C");
    });

    // Reset flow counter
    server.on("/reset", []() {
        totalVolume = 0;
        flowPulseCount = 0;
        Serial.println("Flow counter reset");
        server.send(200, "text/plain", "OK");
    });

    // Poll interval adjustment
    server.on("/poll", []() {
        int delta = server.arg("delta").toInt();
        pollInterval += delta;
        if (pollInterval < 500) pollInterval = 500;    // Min 0.5s
        if (pollInterval > 10000) pollInterval = 10000; // Max 10s
        Serial.print("Poll interval: ");
        Serial.print(pollInterval);
        Serial.println(" ms");
        server.send(200, "text/plain", String(pollInterval));
    });

    // WiFi reconnect
    server.on("/wifi", []() {
        Serial.println("WiFi reconnect requested");
        wifiSetupDone = false;
        wifiConnecting = false;
        wifiNetworkIndex = 0;
        wifiRetryCount = 0;
        WiFi.disconnect();
        server.send(200, "text/plain", "Reconnecting...");
    });

    // Password check for info panel
    server.on("/checkpw", []() {
        String pw = server.arg("pw");
        if (pw == INFO_PASSWORD) {
            Serial.println("Info panel unlocked");
            server.send(200, "text/plain", "OK");
        } else {
            Serial.println("Wrong password attempt");
            server.send(200, "text/plain", "FAIL");
        }
    });

    // System commands (reboot, sleep, alloff, defaults, wifireset)
    server.on("/sys", []() {
        String cmd = server.arg("cmd");
        Serial.print("System command: ");
        Serial.println(cmd);

        if (cmd == "reboot") {
            server.send(200, "text/plain", "Rebooting...");
            delay(500);
            rp2040.reboot();
        } else if (cmd == "sleep") {
            server.send(200, "text/plain", "Entering deep sleep. Power cycle to wake.");
            delay(500);
            allRelaysOff();
            // Deep sleep - requires power cycle to wake
            // Note: Pico 2 doesn't have true deep sleep like ESP32, this is dormant mode
            rp2040.reboot();  // For now just reboot, dormant mode needs RTC
        } else if (cmd == "alloff") {
            allRelaysOff();
            server.send(200, "text/plain", "All relays OFF");
        } else if (cmd == "defaults") {
            targetTemp = 93.0;
            brewTimeMs = 25000;
            useFahrenheit = false;
            allRelaysOff();
            server.send(200, "text/plain", "Settings reset to defaults");
        } else if (cmd == "wifireset") {
            server.send(200, "text/plain", "WiFi reconnecting...");
            wifiSetupDone = false;
            wifiConnecting = false;
            wifiNetworkIndex = 0;
            wifiRetryCount = 0;
            WiFi.disconnect();
        } else {
            server.send(400, "text/plain", "Unknown command");
        }
    });

    // OTA removed - Updater.h doesn't work on RP2350 (Error 4)
    // Code preserved in ForgeRepo/CAPABILITIES.md for when SDK improves

    server.begin();
    Serial.println("Web server started");

    // Start mDNS so you can access via http://espresso.local
    if (MDNS.begin("espresso")) {
        Serial.println("mDNS: http://espresso.local");
        MDNS.addService("http", "tcp", 80);
    }
}
