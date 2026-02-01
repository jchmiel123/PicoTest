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

// ============ PIN DEFINITIONS ============
// Relays (accent LOW for most relay boards)
const int RELAY_PUMP = 2;        // Water pump
const int RELAY_BOILER = 3;      // Boiler heater element
const int RELAY_SOLENOID = 4;    // 3-way solenoid (group head)
const int RELAY_MAIN = 5;        // Main power / spare

// Flow sensor (Hall effect, outputs pulses)
const int FLOW_SENSOR = 6;

// Temperature sensing
const int TEMP_BOILER = 26;      // ADC0 - Thermistor on boiler
const int TEMP_GROUP = 27;       // ADC1 - Optional group head temp

// Onboard LED
const int LED_PIN = 25;

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
float targetTemp = 93.0;   // Default brew temp
bool heaterOn = false;
float tempHysteresis = 2.0; // +/- degrees

// ============ STATE ============
bool pumpOn = false;
bool solenoidOn = false;
bool mainPowerOn = false;
bool brewingActive = false;
unsigned long brewStartTime = 0;

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
void startBrew();
void stopBrew();

// ============ SETUP ============
void setup() {
    Serial.begin(115200);
    delay(2000);

    // Relay pins - all OFF at start
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_BOILER, OUTPUT);
    pinMode(RELAY_SOLENOID, OUTPUT);
    pinMode(RELAY_MAIN, OUTPUT);

    digitalWrite(RELAY_PUMP, RELAY_OFF);
    digitalWrite(RELAY_BOILER, RELAY_OFF);
    digitalWrite(RELAY_SOLENOID, RELAY_OFF);
    digitalWrite(RELAY_MAIN, RELAY_OFF);

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
    printHelp();
}

// ============ MAIN LOOP ============
void loop() {
    static unsigned long lastTempRead = 0;
    static unsigned long lastStatusPrint = 0;
    static unsigned long lastBlink = 0;
    static bool ledState = false;

    unsigned long now = millis();

    // Heartbeat LED (fast blink if brewing)
    unsigned long blinkInterval = brewingActive ? 100 : 500;
    if (now - lastBlink >= blinkInterval) {
        lastBlink = now;
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
    }

    // Read temperature every 250ms
    if (now - lastTempRead >= 250) {
        lastTempRead = now;
        boilerTemp = readTemperature(TEMP_BOILER);
        updateHeaterControl();
    }

    // Update flow rate every 500ms
    if (now - lastFlowCheck >= 500) {
        updateFlowRate();
        lastFlowCheck = now;
    }

    // Auto status print every 2 seconds if brewing
    if (brewingActive && (now - lastStatusPrint >= 2000)) {
        lastStatusPrint = now;
        unsigned long brewTime = (now - brewStartTime) / 1000;
        Serial.print("[BREW ");
        Serial.print(brewTime);
        Serial.print("s] Temp: ");
        Serial.print(boilerTemp, 1);
        Serial.print("C, Flow: ");
        Serial.print(flowRate, 1);
        Serial.print(" mL/s, Total: ");
        Serial.print(totalVolume, 1);
        Serial.println(" mL");
    }

    // Serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
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
    const int pins[] = {0, RELAY_PUMP, RELAY_BOILER, RELAY_SOLENOID, RELAY_MAIN};
    const char* names[] = {"", "Pump", "Boiler", "Solenoid", "Main"};
    bool* states[] = {nullptr, &pumpOn, &heaterOn, &solenoidOn, &mainPowerOn};

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
    digitalWrite(RELAY_MAIN, RELAY_OFF);
    pumpOn = false;
    heaterOn = false;
    solenoidOn = false;
    mainPowerOn = false;
    brewingActive = false;
}

// ============ BREW CONTROL ============
void startBrew() {
    if (brewingActive) {
        Serial.println("Already brewing!");
        return;
    }

    // Safety check: don't brew if boiler is too cold
    if (boilerTemp < (targetTemp - 10)) {
        float tempDiff = (targetTemp - 10) - boilerTemp;
        // Rough estimate: ~2 degrees per minute heating rate
        int estMinutes = (int)(tempDiff / 2.0) + 1;

        Serial.print("Boiler too cold! ");
        Serial.print(boilerTemp, 1);
        Serial.print("C < ");
        Serial.print(targetTemp - 10, 1);
        Serial.println("C");
        Serial.print("Estimated wait: ~");
        Serial.print(estMinutes);
        Serial.println(" minutes");
        Serial.println("Waiting for heat...");
        return;
    }

    Serial.println("=== STARTING BREW ===");
    brewingActive = true;
    brewStartTime = millis();
    totalVolume = 0;
    flowPulseCount = 0;

    setRelay(RELAY_PUMP, true, "Pump");
    pumpOn = true;

    setRelay(RELAY_SOLENOID, true, "Solenoid");
    solenoidOn = true;
}

void stopBrew() {
    if (!brewingActive) {
        Serial.println("Not brewing.");
        return;
    }

    unsigned long brewTime = (millis() - brewStartTime) / 1000;

    setRelay(RELAY_PUMP, false, "Pump");
    pumpOn = false;

    setRelay(RELAY_SOLENOID, false, "Solenoid");
    solenoidOn = false;

    brewingActive = false;

    Serial.println("=== BREW COMPLETE ===");
    Serial.print("Time: ");
    Serial.print(brewTime);
    Serial.println(" seconds");
    Serial.print("Volume: ");
    Serial.print(totalVolume, 1);
    Serial.println(" mL");
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

        case 'b': case 'B':  // Start/stop brew
            if (brewingActive) {
                stopBrew();
            } else {
                startBrew();
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

        case 'm': case 'M':  // Toggle main power
            mainPowerOn = !mainPowerOn;
            setRelay(RELAY_MAIN, mainPowerOn, "Main Power");
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
    Serial.print("Main:     "); Serial.println(mainPowerOn ? "ON" : "OFF");
    Serial.print("Brewing:  "); Serial.println(brewingActive ? "YES" : "NO");
    Serial.println("============================\n");
}

void printHelp() {
    Serial.println("Commands:");
    Serial.println("  H     Help");
    Serial.println("  S     Status");
    Serial.println("  B     Start/Stop Brew");
    Serial.println("  P     Toggle Pump");
    Serial.println("  O     Toggle Solenoid");
    Serial.println("  M     Toggle Main Power");
    Serial.println("  +/-   Adjust target temp");
    Serial.println("  R     Reset flow counter");
    Serial.println("  X     EMERGENCY STOP");
    Serial.println("  1-4   Direct relay toggle");
    Serial.println();
}
