// ============================================================================
// Task 4: XPLORER Remote Control Panel (Pico W, Wokwi)  — Student Stub
//
// Architecture:
//   The Wokwi Pico W is a thin remote-control + mirror.  Authoritative LED
//   state lives on the XPLORER (real hardware on the Pico-on-XPLORER board).
//
//   Wokwi  →  XPLORER  (commands, via MQTT → Nano IoT bridge → UART):
//     - Press button N (1..4) on Wokwi  →  publish "LED:N:NEXT"
//                                           (XPLORER advances LED N color)
//     - Turn potentiometer in Wokwi     →  publish "SERVO:angle"
//                                           (XPLORER moves the real servo)
//
//   XPLORER  →  Wokwi  (feedback, same path in reverse):
//     - LED color changed (any reason — own button OR our LED:N:NEXT cmd):
//                                       →  "ACK:LED:N:COLOR"  (we mirror)
//     - Servo position changed          →  "ACK:SERVO:angle"  (we just log)
//     - DHT11 sensor reading            →  "TEMP:value" / "HUM:value"
//     - Full state on STATUS request    →  "STATUS:LED1=…,SERVO=…,TEMP=…"
//
// Robustness features (already implemented for you, do not remove):
//   - Shared Token        token="iem2026"
//   - Source Check        expects source="nano" from XPLORER
//   - Sequence Numbers    gap detection and replay protection
//   - Watchdog            15 second timeout triggers safe state
//
// Topics:
//   PUBLISH    iem/task4/xplorer/cmd    (commands going to XPLORER)
//   SUBSCRIBE  iem/task4/xplorer/data   (feedback coming from XPLORER)
//
// What you implement (TODO markers):
//   - handleLedButton()    publish LED:N:NEXT on each Wokwi button press
//   - handlePotentiometer() throttled publish of SERVO:angle on pot change
//   - processUARTFeedback() dispatch incoming UART events to local actions
//
// Institut fuer Elektromobilitaet (IEM), Hochschule Ravensburg-Weingarten
// ============================================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "SimpleJson.h"
// Servo.h is intentionally NOT included — the servo is XPLORER-side hardware
// only.  The Wokwi board has no virtual servo.

// ── WiFi configuration (Wokwi virtual AP) ──────────────────────────
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

// ── MQTT configuration ─────────────────────────────────────────────
const char* MQTT_BROKER = "broker.emqx.io";
// const char* MQTT_BROKER  = "141.69.95.10";  // fbe-mqtt.hs-weingarten.de
const int   MQTT_PORT    = 1883;
const char* TOPIC_CMD    = "iem/task4/xplorer/cmd";
const char* TOPIC_DATA   = "iem/task4/xplorer/data";
const char* CLIENT_ID    = "iem-pico-w-task4-";

// ── Robustness configuration ──────────────────────────────────────
const char* SHARED_TOKEN    = "iem2026";
const char* EXPECTED_SOURCE = "nano";
const unsigned long WATCHDOG_TIMEOUT = 15000;   // 15 seconds

// ── Hardware pins ──────────────────────────────────────────────────
const int NEOPIXEL_PIN     = 28;   // WS2812 strip on GP28
const int BUTTON_LED1_PIN  = 2;    // Button "LED 1" on GP2
const int BUTTON_LED2_PIN  = 3;    // Button "LED 2" on GP3
const int BUTTON_LED3_PIN  = 4;    // Button "LED 3" on GP4
const int BUTTON_LED4_PIN  = 5;    // Button "LED 4" on GP5
const int POT_PIN          = 26;   // Potentiometer on GP26 (ADC)
const int OLED_SDA_PIN     = 0;    // SSD1306 OLED SDA on GP0 (I2C0)
const int OLED_SCL_PIN     = 1;    // SSD1306 OLED SCL on GP1 (I2C0)

// ── SSD1306 OLED display ──────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// Cache last seen DHT11 readings so the OLED stays up to date even when
// only TEMP or only HUM arrives (events come in independently).
float lastTempC  = NAN;
int   lastHumPct = -1;

// ── NeoPixel ──────────────────────────────────────────────────────
#define NUM_LEDS 4
Adafruit_NeoPixel pixels(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// ── State variables ───────────────────────────────────────────────
bool lastButton1State = HIGH;
bool lastButton2State = HIGH;
bool lastButton3State = HIGH;
bool lastButton4State = HIGH;

int  lastServoAngle    = 90;
int  currentServoAngle = 90;          // mirrored from XPLORER
unsigned long lastServoUpdate = 0;
const unsigned long SERVO_READ_INTERVAL = 100;   // throttle: 100 ms
const int SERVO_THRESHOLD = 5;                   // threshold: ±5°

// ── Robustness state ──────────────────────────────────────────────
unsigned long lastValidCmd = 0;
bool inSafeState = false;
int  expectedSeqNr = -1;
unsigned long seqOutgoing = 0;

// Statistics
unsigned long msgAccepted     = 0;
unsigned long msgRejectedJson = 0;
unsigned long msgRejectedAuth = 0;
unsigned long msgSeqGaps      = 0;

// ── Objects ───────────────────────────────────────────────────────
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
SimpleJson   jsonOut;
SimpleJson   jsonIn;

// Forward declarations
void publishCommand(const char* cmd);
void parseStatusReport(const String& statusStr);

// ════════════════════════════════════════════════════════════════════
//  Render TEMP / HUM on the SSD1306 OLED  (TODO — student work)
// ════════════════════════════════════════════════════════════════════
void renderDisplay() {

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 8);
  display.print("T: ");
  if (isnan(lastTempC)) display.print(F("--.-"));
  else display.print(lastTempC, 1);
  display.println(F(" C"));

  display.setCursor(0, 36);
  display.print("H: ");
  if (lastHumPct < 0) display.print(F("--"));
  else display.print(lastHumPct);
  display.println(F(" %"));

  display.display();
}

// ════════════════════════════════════════════════════════════════════
//  WiFi connection
// ════════════════════════════════════════════════════════════════════
void setupWiFi() {
    Serial.print(F("WiFi connecting: "));
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print('.');
        timeout++;
    }
    Serial.println();
    Serial.print(F("Connected, IP: "));
    Serial.println(WiFi.localIP());
}

// ════════════════════════════════════════════════════════════════════
//  Helper: color name → NeoPixel color (used to apply ACK:LED:N:COLOR)
// ════════════════════════════════════════════════════════════════════
uint32_t getColorFromName(const char* name) {
    if (strcmp(name, "RED")    == 0) return pixels.Color(255, 0, 0);
    if (strcmp(name, "GREEN")  == 0) return pixels.Color(0, 255, 0);
    if (strcmp(name, "BLUE")   == 0) return pixels.Color(0, 0, 255);
    if (strcmp(name, "YELLOW") == 0) return pixels.Color(255, 255, 0);
    if (strcmp(name, "CYAN")   == 0) return pixels.Color(0, 255, 255);
    if (strcmp(name, "PURPLE") == 0) return pixels.Color(128, 0, 128);
    if (strcmp(name, "WHITE")  == 0) return pixels.Color(255, 255, 255);
    if (strcmp(name, "OFF")    == 0) return pixels.Color(0, 0, 0);
    return pixels.Color(0, 0, 0);
}

// ════════════════════════════════════════════════════════════════════
//  Robustness: token + source validation
// ════════════════════════════════════════════════════════════════════
bool validateMessage(const SimpleJson& msg) {
    if (!msg.hasKey("token")) {
        Serial.println(F("  REJECT: no token"));
        return false;
    }
    if (strcmp(msg.getString("token"), SHARED_TOKEN) != 0) {
        Serial.println(F("  REJECT: invalid token"));
        return false;
    }
    if (!msg.hasKey("source")) {
        Serial.println(F("  REJECT: no source"));
        return false;
    }
    if (strcmp(msg.getString("source"), EXPECTED_SOURCE) != 0) {
        Serial.print(F("  REJECT: unexpected source "));
        Serial.println(msg.getString("source"));
        return false;
    }
    return true;
}

// ════════════════════════════════════════════════════════════════════
//  Robustness: sequence number check
// ════════════════════════════════════════════════════════════════════
bool checkSequence(const SimpleJson& msg) {
    if (!msg.hasKey("seq")) {
        Serial.println(F("  WARN: no seq field"));
        return true;
    }
    int seq = msg.getInt("seq");
    if (expectedSeqNr < 0 || seq == 0) {
        expectedSeqNr = seq + 1;
        Serial.print(F("  SEQ sync to ")); Serial.println(seq);
        return true;
    }
    if (seq == expectedSeqNr) {
        expectedSeqNr = seq + 1;
        return true;
    }
    if (seq > expectedSeqNr) {
        msgSeqGaps++;
        Serial.print(F("  WARN: seq gap, expected="));
        Serial.print(expectedSeqNr);
        Serial.print(F(" got="));
        Serial.println(seq);
        expectedSeqNr = seq + 1;
        return true;
    }
    Serial.print(F("  REJECT: replay/stale, expected="));
    Serial.print(expectedSeqNr);
    Serial.print(F(" got="));
    Serial.println(seq);
    return false;
}

// Forward declaration for the dispatcher
void processUARTFeedback(const char* uartData);

// ════════════════════════════════════════════════════════════════════
//  MQTT callback: receive feedback from XPLORER
// ════════════════════════════════════════════════════════════════════
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    char message[384];
    unsigned int copyLen = (length < sizeof(message) - 1) ? length : sizeof(message) - 1;
    memcpy(message, payload, copyLen);
    message[copyLen] = '\0';

    Serial.print(F("MQTT received [")); Serial.print(topic); Serial.print(F("]: "));
    Serial.println(message);

    if (!jsonIn.parse(message))            { msgRejectedJson++; Serial.println(F("  REJECT: JSON parse")); return; }
    if (!validateMessage(jsonIn))          { msgRejectedAuth++; return; }
    if (!checkSequence(jsonIn))            { return; }

    msgAccepted++;
    lastValidCmd = millis();
    inSafeState  = false;

    if (jsonIn.hasKey("uart")) {
        processUARTFeedback(jsonIn.getString("uart"));
    }
}

// ════════════════════════════════════════════════════════════════════
//  Process incoming UART feedback (TODO — student work)
// ════════════════════════════════════════════════════════════════════
void processUARTFeedback(const char* uartData) {
    String data(uartData);
    data.trim();
    Serial.print(F("  UART: ")); Serial.println(data);

    if (data.startsWith("ACK:LED:")) {
        int colon1 = data.indexOf(':');
        int colon2 = data.indexOf(':', colon1 + 1);
        int colon3 = data.indexOf(':', colon2 + 1);

        if (colon2 > 0 && colon3 > 0) {
            int ledNum = data.substring(colon2 + 1, colon3).toInt();
            String colorName = data.substring(colon3 + 1);
            colorName.trim();

            Serial.print(F("  [MIRROR] Virtual LED ")); Serial.print(ledNum);
            Serial.print(F(" updated to: ")); Serial.println(colorName);

            if (ledNum >= 1 && ledNum <= NUM_LEDS) {
                pixels.setPixelColor(ledNum - 1, getColorFromName(colorName.c_str()));
                pixels.show();
            }
        }
    }

    else if (data.startsWith("ACK:SERVO:")) {
        currentServoAngle = data.substring(10).toInt();
        Serial.print(F("  [MIRROR] Servo updated to: "));
        Serial.println(currentServoAngle);
    }

    else if (data.startsWith("TEMP:")) {
        lastTempC = data.substring(5).toFloat();
        Serial.print(F("Temp: ")); Serial.println(lastTempC);
        renderDisplay();
    }

    else if (data.startsWith("HUM:")) {
        lastHumPct = data.substring(4).toInt();
        Serial.print(F("Humidty: ")); Serial.println(lastHumPct);
        renderDisplay();
    }

    else if (data.startsWith("STATUS:")) {
        parseStatusReport(data);
    }

}

// ════════════════════════════════════════════════════════════════════
//  Parse STATUS:LED1=RED,LED2=GREEN,LED3=OFF,LED4=BLUE,SERVO=90,TEMP=23.5,HUM=45
// ════════════════════════════════════════════════════════════════════
void parseStatusReport(const String& statusStr) {
    Serial.println(F("  [STATUS] parsing full report..."));

    for (int i = 1; i <= NUM_LEDS; i++) {
        String key = "LED";
        key += i;
        key += "=";
        int idx = statusStr.indexOf(key);
        if (idx < 0) continue;
        idx += key.length();
        int endIdx = statusStr.indexOf(',', idx);
        if (endIdx < 0) endIdx = statusStr.length();
        String colorName = statusStr.substring(idx, endIdx);
        colorName.trim();
        pixels.setPixelColor(i - 1, getColorFromName(colorName.c_str()));
        Serial.print(F("    LED")); Serial.print(i);
        Serial.print(F("=")); Serial.println(colorName);
    }
    pixels.show();

    int servIdx = statusStr.indexOf(F("SERVO="));
    if (servIdx >= 0) {
        servIdx += 6;
        int endIdx = statusStr.indexOf(',', servIdx);
        if (endIdx < 0) endIdx = statusStr.length();
        currentServoAngle = statusStr.substring(servIdx, endIdx).toInt();
        Serial.print(F("    SERVO=")); Serial.println(currentServoAngle);
    }

    int tempIdx = statusStr.indexOf(F("TEMP="));
    if (tempIdx >= 0) {
        tempIdx += 5;
        int endIdx = statusStr.indexOf(',', tempIdx);
        if (endIdx < 0) endIdx = statusStr.length();
        Serial.print(F("    TEMP="));
        Serial.println(statusStr.substring(tempIdx, endIdx).toFloat());
    }

    int humIdx = statusStr.indexOf(F("HUM="));
    if (humIdx >= 0) {
        humIdx += 4;
        int endIdx = statusStr.indexOf(',', humIdx);
        if (endIdx < 0) endIdx = statusStr.length();
        Serial.print(F("    HUM="));
        Serial.println(statusStr.substring(humIdx, endIdx).toInt());
    }
}

// ════════════════════════════════════════════════════════════════════
//  Publish a UART command toward the XPLORER
// ════════════════════════════════════════════════════════════════════
void publishCommand(const char* cmd) {
    jsonOut.clear();
    jsonOut.setString("token",  SHARED_TOKEN);
    jsonOut.setString("source", "pico");
    jsonOut.setInt   ("seq",    (int)seqOutgoing);
    jsonOut.setString("cmd",    cmd);

    char buf[256];
    jsonOut.toCharArray(buf, sizeof(buf));
    mqtt.publish(TOPIC_CMD, buf);

    Serial.print(F("MQTT publish [")); Serial.print(TOPIC_CMD); Serial.print(F("]: "));
    Serial.println(buf);

    seqOutgoing++;
}

// ════════════════════════════════════════════════════════════════════
//  MQTT (re)connect
// ════════════════════════════════════════════════════════════════════
void mqttReconnect() {
    while (!mqtt.connected()) {
        String clientId = CLIENT_ID + String(random(10000, 99999));
        Serial.print(F("MQTT connecting as ")); Serial.print(clientId); Serial.print(F("..."));
        if (mqtt.connect(clientId.c_str())) {
            Serial.println(F(" connected!"));
            mqtt.subscribe(TOPIC_DATA);
            Serial.print(F("Subscribed: ")); Serial.println(TOPIC_DATA);
            lastValidCmd = millis();
        } else {
            Serial.print(F(" failed, rc="));
            Serial.print(mqtt.state());
            Serial.println(F(" — retry in 3s"));
            delay(3000);
        }
    }
}

// ════════════════════════════════════════════════════════════════════
//  Button N pressed → publish "LED:N:NEXT"  (TODO — student work)
//  XPLORER will advance LED N's color and reply ACK:LED:N:COLOR.
// ════════════════════════════════════════════════════════════════════
void handleLedButton(int ledNum, int pin, bool& lastState) {
    bool currentState = digitalRead(pin);

    // Detect HIGH→LOW edge (falling = press, INPUT_PULLUP)
    if (lastState == HIGH && currentState == LOW) {
        char buf[32];
        snprintf(buf, sizeof(buf), "LED:%d:NEXT", ledNum);
        publishCommand(buf);
    }
    lastState = currentState;
}

// ════════════════════════════════════════════════════════════════════
//  Potentiometer → SERVO:angle  (TODO — student work)
// ════════════════════════════════════════════════════════════════════
void handlePotentiometer() {

  unsigned long now = millis();
  
  if (now - lastServoUpdate >= SERVO_READ_INTERVAL) {
      lastServoUpdate = now;

      int rawValue = analogRead(POT_PIN);
      int newAngle = map(rawValue, 0, 4095, 0, 180);

      // Threshold check to prevent jitter-spamming MQTT
      if (abs(newAngle - lastServoAngle) >= SERVO_THRESHOLD) {
          char buf[32];
          snprintf(buf, sizeof(buf), "SERVO:%d", newAngle);
          publishCommand(buf);
          lastServoAngle = newAngle;
      }
  }
}

// ════════════════════════════════════════════════════════════════════
//  Serial Monitor commands (debugging)
//    led:1   led:2   led:3   led:4    → cycle that LED's color (same as button)
//    servo:90                          → set servo angle
//    status                            → request full status
// ════════════════════════════════════════════════════════════════════
void handleSerialInput() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    Serial.print(F(">> Serial: ")); Serial.println(input);

    if (input.startsWith(F("led:"))) {
        int n = input.substring(4).toInt();
        if (n >= 1 && n <= NUM_LEDS) {
            char cmd[32];
            snprintf(cmd, sizeof(cmd), "LED:%d:NEXT", n);
            publishCommand(cmd);
        }
    } else if (input.startsWith(F("servo:"))) {
        int angle = input.substring(6).toInt();
        char cmd[32];
        snprintf(cmd, sizeof(cmd), "SERVO:%d", angle);
        publishCommand(cmd);
    } else if (input == F("status")) {
        publishCommand("STATUS");
    } else {
        // Treat unknown input as raw command for power users
        publishCommand(input.c_str());
    }
}

// ════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════
void setup() {
    pinMode(BUTTON_LED1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LED2_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LED3_PIN, INPUT_PULLUP);
    pinMode(BUTTON_LED4_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(1000);

    Serial.println(F("============================================"));
    Serial.println(F("  Task 4: XPLORER Remote Control Panel"));
    Serial.println(F("  (Pico W, Wokwi)  —  4-button mirror"));
    Serial.println(F("============================================"));
    Serial.println();

    // OLED first so a "Connecting..." splash is visible during WiFi/MQTT setup
    Wire.setSDA(OLED_SDA_PIN);
    Wire.setSCL(OLED_SCL_PIN);
    Wire.begin();
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println(F("Task 4"));
        display.println(F("Connecting..."));
        display.display();
        Serial.println(F("[INIT] SSD1306 OLED initialized (I2C 0x3C on GP0/GP1)"));
    } else {
        Serial.println(F("[WARN] SSD1306 not detected at 0x3C"));
    }

    setupWiFi();

    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setSocketTimeout(30);
    mqtt.setKeepAlive(60);

    pixels.begin();
    pixels.clear();

    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 255)); 
    pixels.setPixelColor(2, pixels.Color(0, 255, 0)); 
    pixels.setPixelColor(3, pixels.Color(255, 255, 255)); 
 
    pixels.show();


    Serial.println(F("[INIT] NeoPixel initialized (4x WS2812 on GP28)"));

    randomSeed(analogRead(POT_PIN) ^ micros());
    lastValidCmd = millis();

    Serial.println();
    Serial.println(F("[READY] waiting for MQTT connection..."));
    Serial.println(F("Commands: 'led:1' .. 'led:4', 'servo:<angle>', 'status'"));
    Serial.println();


}

// ════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════
void loop() {

    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 255)); 
    pixels.setPixelColor(2, pixels.Color(0, 255, 0)); 
    pixels.setPixelColor(3, pixels.Color(255, 255, 255)); 
 
    pixels.show();

    if (!mqtt.connected()) {
        mqttReconnect();
    }
    mqtt.loop();

    // Watchdog
    if (millis() - lastValidCmd >= WATCHDOG_TIMEOUT) {
        if (!inSafeState) {
            inSafeState = true;
            Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
            Serial.println(F("! WATCHDOG: Safe State activated!     !"));
            Serial.println(F("! No valid feedback from XPLORER!     !"));
            Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
        }
    }

    // Buttons → LED:N:NEXT
    handleLedButton(1, BUTTON_LED1_PIN, lastButton1State);
    handleLedButton(2, BUTTON_LED2_PIN, lastButton2State);
    handleLedButton(3, BUTTON_LED3_PIN, lastButton3State);
    handleLedButton(4, BUTTON_LED4_PIN, lastButton4State);

    // Pot → SERVO:angle
    handlePotentiometer();

    // Serial debug
    handleSerialInput();

    // WOKWI TCP flush workaround to prevent message buffer overload
    static unsigned long lastNetworkPing = 0;
    if (millis() - lastNetworkPing >= 500) {
        lastNetworkPing = millis();
        mqtt.publish("iem/task4/xplorer/ping", "1"); 
    }

    delay(20);
}
