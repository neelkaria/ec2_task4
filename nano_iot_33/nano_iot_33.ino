// ============================================================================
// Task 4: MQTT-UART Bridge (Arduino Nano 33 IoT)
//
// Concept:
//   The Nano bridges between MQTT (Wokwi Pico W) and UART (RPi Pico
//   on XPLORER board). Commands flow: Wokwi → MQTT → Nano → UART → Pico.
//   Feedback flows back: Pico → UART → Nano → MQTT → Wokwi.
//   The Serial Monitor (USB) can also be used to send commands directly.
//
// Commands forwarded to XPLORER Pico:
//   LED:1:NEXT      Cycle LED 1 to its next color
//   LED:2:NEXT      Cycle LED 2 to its next color
//   LED:3:NEXT      Cycle LED 3 to its next color
//   SERVO:90        Move servo to 90 degrees
//   STATUS          Query Pico status
//   HELP            Show available commands
//
// Robustness features:
//   - Shared Token:   Messages without a valid token are discarded
//   - Source Check:   Only source="pico" is accepted from MQTT
//   - Sequence Nr:    Gaps and replayed messages are detected
//   - Watchdog:       Warning if XPLORER Pico has not responded for 20s
//   - Onboard LED:    Heartbeat blink to show bridge is running
//   - JSON validation: Ensures all incoming/outgoing messages are valid
//
// MQTT Topics:
//   SUBSCRIBE:  iem/task4/xplorer/cmd     (commands from Wokwi Pico W)
//   PUBLISH:    iem/task4/xplorer/data    (feedback from XPLORER Pico)
//
// Serial interfaces:
//   Serial (USB):     Serial Monitor at 115200 (user input & debug output)
//   Serial1 (D0/D1):  UART to RPi Pico at 115200 (transparent bridge)
//
// Dependencies:
//   - WiFiNINA
//   - PubSubClient
//   - SimpleJson.h (custom lightweight JSON helper)
//
// Institute of Electric Mobility (IEM), Ravensburg-Weingarten University
// ============================================================================

// ── Include libraries ───────────────────────────────────────────────────────
#include <WiFiNINA.h>      // WiFi support for the Nano 33 IoT
#include <PubSubClient.h>  // MQTT client library
#include "SimpleJson.h"    // Lightweight custom JSON helper class

// ── WiFi credentials ────────────────────────────────────────────────────────
// Must be adapted to the local network environment
// const char* WIFI_SSID = "WLAN-Pi-1";
// const char* WIFI_PASS = "raspberry";
//const char* WIFI_SSID = "RWUioT";
//const char* WIFI_PASS = "ioTistGaPSKistB";
const char* WIFI_SSID = "Vodafone-3DE8";
const char* WIFI_PASS = "GyDvEgPkd6GdJECW";

// ── MQTT configuration ──────────────────────────────────────────────────────
const char* MQTT_BROKER = "broker.emqx.io";
// const char* MQTT_BROKER  = "141.69.95.10";  // fbe-mqtt.hs-weingarten.de (IP – DNS fallback)
const int   MQTT_PORT    = 1883;
// Topic the Nano listens to (commands published by the Wokwi Pico W)
const char* TOPIC_CMD    = "iem/task4/xplorer/cmd";
// Topic the Nano publishes to (feedback from the XPLORER Pico)
const char* TOPIC_DATA   = "iem/task4/xplorer/data";
// Prefix for the MQTT client ID; a random suffix is added at runtime
const char* CLIENT_ID    = "iem-nano-";

// ── Robustness configuration ────────────────────────────────────────────────
// Shared secret – any message without this exact token is rejected
const char* SHARED_TOKEN    = "iem2026";
// Only messages from this source are accepted in MQTT
const char* EXPECTED_SOURCE = "pico";
// Time (ms) without a valid UART response before the watchdog fires
const unsigned long WATCHDOG_TIMEOUT = 20000;  // 20 seconds

// ── Hardware ─────────────────────────────────────────────────────────────────
// The built-in LED blinks as a heartbeat to show the bridge is running
const int HEARTBEAT_LED = LED_BUILTIN;
// Heartbeat timing
const unsigned long HEARTBEAT_ON_MS  = 50;   // LED on for 50ms
const unsigned long HEARTBEAT_OFF_MS = 950;  // LED off for 950ms = ~1 second cycle

// ── UART Configuration ──────────────────────────────────────────────────────
// Serial1 (D0 RX, D1 TX) to the XPLORER Pico at 115200 baud
const unsigned long SERIAL1_BAUD = 115200;
// Maximum length of a UART line
const int UART_LINE_MAX_LEN = 256;

// ── Last known XPLORER state ────────────────────────────────────────────────
unsigned long lastUartResponse = 0;  // Timestamp of last UART message received
bool xplorerTimeout = false;         // True if the watchdog has fired
int  expectedSeqNr  = -1;            // Next expected sequence number (-1 = not synced)
unsigned long seqOutgoing = 0;       // Sequence counter for MQTT publish

// ── Message statistics ──────────────────────────────────────────────────────
unsigned long msgAccepted     = 0;  // Valid MQTT messages processed
unsigned long msgRejectedJson = 0;  // Dropped – JSON parse error
unsigned long msgRejectedAuth = 0;  // Dropped – wrong token or source
unsigned long msgSeqGaps      = 0;  // Sequence gaps detected
unsigned long uartSent        = 0;  // UART commands sent
unsigned long uartReceived    = 0;  // UART responses received

// ── Heartbeat state ─────────────────────────────────────────────────────────
unsigned long lastHeartbeatToggle = 0;
bool heartbeatLedOn = false;

// ── Objects ──────────────────────────────────────────────────────────────────
WiFiClient   wifiClient;       // Underlying TCP/IP socket for MQTT
PubSubClient mqtt(wifiClient); // MQTT client built on top of wifiClient
SimpleJson   jsonOut;          // Reusable JSON builder for outgoing messages
SimpleJson   jsonIn;           // Reusable JSON parser for incoming messages

// ════════════════════════════════════════════════════════════════════
//  setupWiFi()
//  Connects to the configured WiFi network. Blocks forever if the
//  WiFi module is missing or the connection cannot be established
//  within 30 attempts (~15 s).
// ════════════════════════════════════════════════════════════════════
void setupWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    // Verify that the WiFi module (u-blox NINA) is present and responsive
    if (WiFi.status() == WL_NO_MODULE) {
        Serial.println("ERROR: WiFi module not found!");
        while (true) { delay(1000); }  // Halt – nothing else will work
    }

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Poll the connection status every 500 ms, give up after 30 tries
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nERROR: WiFi connection failed!");
        while (true) { delay(1000); }  // Halt
    }

    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
}

// ════════════════════════════════════════════════════════════════════
//  validateMessage()
//  Checks whether a parsed JSON message carries the correct shared
//  token and the expected source identifier.
//  Returns true only if both fields are present and match exactly.
// ════════════════════════════════════════════════════════════════════
bool validateMessage(const SimpleJson& msg) {
    // Reject if the "token" field is absent
    if (!msg.hasKey("token")) {
        Serial.println("  REJECT: No token.");
        return false;
    }
    // Reject if the token value does not match the shared secret
    if (strcmp(msg.getString("token"), SHARED_TOKEN) != 0) {
        Serial.println("  REJECT: Invalid token.");
        return false;
    }
    // Reject if the "source" field is absent
    if (!msg.hasKey("source")) {
        Serial.println("  REJECT: No source field.");
        return false;
    }
    // Reject messages from unexpected senders
    if (strcmp(msg.getString("source"), EXPECTED_SOURCE) != 0) {
        Serial.print("  REJECT: source=");
        Serial.println(msg.getString("source"));
        return false;
    }
    return true;  // Token and source are both valid
}

// ════════════════════════════════════════════════════════════════════
//  checkSequence()
//  Validates the "seq" field of an incoming MQTT message to detect:
//    - Gaps: one or more messages were lost in transit
//    - Replays: an old message arrived again (duplicate / attack)
//
//  A gap is logged but the message is still accepted.
//  A replay is rejected entirely.
//  Returns true if the message should be processed.
// ════════════════════════════════════════════════════════════════════
bool checkSequence(const SimpleJson& msg) {
    // If there is no seq field, skip the check entirely
    if (!msg.hasKey("seq")) return true;

    int seq = msg.getInt("seq");

    // First message ever, or Pico W restarted (seq resets to 0)
    if (expectedSeqNr < 0 || seq == 0) {
        expectedSeqNr = seq + 1;
        return true;
    }

    // Happy path: sequence is exactly as expected
    if (seq == expectedSeqNr) {
        expectedSeqNr = seq + 1;
        return true;
    }

    // Future seq: gap detected – some messages were lost
    if (seq > expectedSeqNr) {
        int gap = seq - expectedSeqNr;
        msgSeqGaps++;
        Serial.print("  WARN: Sequence gap! ");
        Serial.print(gap);
        Serial.println(" message(s) lost.");
        expectedSeqNr = seq + 1;
        return true;  // Still accept this message
    }

    // Past seq: replay (duplicate or deliberate attack) – reject
    Serial.print("  REJECT: Replay (seq=");
    Serial.print(seq);
    Serial.println(")");
    return false;
}

// ════════════════════════════════════════════════════════════════════
//  mqttCallback()
//  Called automatically by PubSubClient whenever a message arrives
//  on a subscribed topic. Validates the message and extracts the
//  "cmd" field, then forwards it to the XPLORER Pico via Serial1.
// ════════════════════════════════════════════════════════════════════
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Copy the raw bytes into a null-terminated C-string for parsing
    char message[512];
    unsigned int copyLen = (length < sizeof(message) - 1) ? length : sizeof(message) - 1;
    memcpy(message, payload, copyLen);
    message[copyLen] = '\0';

    Serial.print("MQTT RX: ");
    Serial.println(message);

    // ── Step 1: Parse JSON ──────────────────────────────────────────
    if (!jsonIn.parse(message)) {
        msgRejectedJson++;
        Serial.println("  REJECT: JSON parse error");
        return;
    }

    // ── Step 2: Validate shared token and source field ──────────────
    if (!validateMessage(jsonIn)) {
        msgRejectedAuth++;
        return;
    }

    // ── Step 3: Sequence number check ──────────────────────────────
    if (!checkSequence(jsonIn)) {
        return;  // Replay detected – discard
    }

    // ── All checks passed ───────────────────────────────────────────
    msgAccepted++;
    lastUartResponse = millis();  // Reset the watchdog timer

    // If the Pico had timed out before, announce that it is reachable again
    if (xplorerTimeout) {
        xplorerTimeout = false;
        Serial.println(">> XPLORER Pico reachable again!");
        Serial.println(F("────────────────────────────────────"));
    }

    // ── Extract and forward the command ─────────────────────────────
    if (!jsonIn.hasKey("cmd")) {
        Serial.println("  REJECT: No 'cmd' field.");
        return;
    }

    // TODO: Extract the "cmd" field from jsonIn with getString("cmd"), then:
    //   - Send it to Serial1 with Serial1.println(cmd)  (the XPLORER expects '\n' termination)
    //   - Increment the uartSent counter
    //   - Log to the USB Serial Monitor for debugging, e.g.:

    const char* cmd = jsonIn.getString("cmd");
    Serial1.println(cmd);
    uartSent++;
    
    Serial.print("  -> Forwarding to UART: ");
    Serial.println(cmd);

}

// ════════════════════════════════════════════════════════════════════
//  mqttReconnect()
//  Blocks until a connection to the MQTT broker is established.
//  A random suffix is appended to CLIENT_ID to avoid conflicts.
//  After connecting, re-subscribes to TOPIC_CMD.
// ════════════════════════════════════════════════════════════════════
void mqttReconnect() {
    while (!mqtt.connected()) {
        // Build a unique client ID for this session
        String clientId = CLIENT_ID + String(random(10000, 99999));
        Serial.print("Connecting to MQTT as ");
        Serial.print(clientId);
        Serial.print("...");

        if (mqtt.connect(clientId.c_str())) {
            Serial.println(" OK!");
            mqtt.subscribe(TOPIC_CMD);  // Re-subscribe after reconnect
            Serial.print("Subscribed: ");
            Serial.println(TOPIC_CMD);
            lastUartResponse = millis();  // Prevent immediate watchdog trigger
        } else {
            Serial.print(" FAILED rc=");
            Serial.print(mqtt.state());
            Serial.println(" | Retry in 3s");
            delay(3000);
        }
    }
}

// ════════════════════════════════════════════════════════════════════
//  publishUartResponse()
//  Wraps a UART line in JSON with metadata and publishes it to
//  TOPIC_DATA. The "uart" field contains the raw message from the
//  XPLORER Pico.
// ════════════════════════════════════════════════════════════════════
void publishUartResponse(const char* uartLine) {
    jsonOut.clear();
    jsonOut.setString("token",  SHARED_TOKEN);
    jsonOut.setString("source", "nano");
    jsonOut.setInt("seq",       (int)seqOutgoing);
    jsonOut.setString("uart",   uartLine);
    seqOutgoing++;

    // Serialise to a stack buffer and publish
    char buf[512];
    jsonOut.toCharArray(buf, sizeof(buf));
    mqtt.publish(TOPIC_DATA, buf);

    Serial.print("  MQTT TX: ");
    Serial.println(buf);
}

// ════════════════════════════════════════════════════════════════════
//  processUartInput()
//  Reads complete lines from Serial1 (XPLORER Pico) and wraps them
//  in JSON before publishing to MQTT. Also echoes to Serial Monitor
//  for debugging. Uses readStringUntil('\n') for non-blocking reads.
// ════════════════════════════════════════════════════════════════════
void processUartInput() {
    // TODO: Check if Serial1 has data available. If so:
    //   1. Read a line with Serial1.readStringUntil('\n').
    //   2. trim() the line and skip it if empty.
    //   3. Log "UART RX: <line>" to the USB Serial Monitor.
    //   4. Increment uartReceived and reset the watchdog timer
    //        (lastUartResponse = millis();)
    //   5. If xplorerTimeout was previously set (bridge was in timeout state),
    //        clear it and announce ">> XPLORER Pico responsive again!"
    //        Otherwise the watchdog log stays silent when the Pico comes back.
    //   6. Wrap the line in JSON and publish it via publishUartResponse(line.c_str()).

    if(Serial1.available())
    {
        String uartInput = Serial1.readStringUntil('\n');
        uartInput.trim();
        
        if (uartInput.length() == 0) return;

        Serial1.print(" UART RX: ");Serial1.println(uartInput);
        uartReceived++;
        lastUartResponse = millis();    // Reste Watchdog

        if(xplorerTimeout)
        {
            xplorerTimeout = false;
            Serial1.println(">> XPLORER Pico responsive again!");
        }

        publishUartResponse(uartInput.c_str());

    }
}

// ════════════════════════════════════════════════════════════════════
//  processSerialInput()
//  Reads one line from the Serial Monitor (USB), converts it to
//  uppercase, and dispatches it. Commands can be forwarded to XPLORER
//  Pico directly via UART, or can query statistics / help.
// ════════════════════════════════════════════════════════════════════
void processSerialInput() {
    if (!Serial.available()) return;

    // Read until newline, strip whitespace, normalise to uppercase
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    if (input.length() == 0) return;  // Ignore blank lines

    Serial.print("CMD: ");
    Serial.println(input);

    // ── STATUS: Display bridge statistics ───────────────────────────
    if (input == "STATUS") {
        Serial.println("=== Bridge Status ===");
        Serial.print("  MQTT: ");
        Serial.println(mqtt.connected() ? "CONNECTED" : "DISCONNECTED");
        Serial.print("  XPLORER Pico: ");
        Serial.println(xplorerTimeout ? "TIMEOUT (!)" : "OK");
        Serial.print("  UART (D0/D1): ");
        Serial.println(Serial1 ? "ACTIVE" : "ERROR");
        Serial.println("=== Statistics ===");
        Serial.print("  MQTT messages accepted: ");
        Serial.println(msgAccepted);
        Serial.print("  MQTT messages rejected: ");
        Serial.println(msgRejectedAuth);
        Serial.print("  UART commands sent: ");
        Serial.println(uartSent);
        Serial.print("  UART responses received: ");
        Serial.println(uartReceived);
        Serial.println("==================");
    }

    // ── STATS: Show detailed message statistics ─────────────────────
    else if (input == "STATS") {
        Serial.println("======= Message Statistics =======");
        Serial.print("  MQTT Accepted:       ");
        Serial.println(msgAccepted);
        Serial.print("  MQTT JSON errors:    ");
        Serial.println(msgRejectedJson);
        Serial.print("  MQTT Auth errors:    ");
        Serial.println(msgRejectedAuth);
        Serial.print("  Sequence gaps:       ");
        Serial.println(msgSeqGaps);
        Serial.print("  UART sent:           ");
        Serial.println(uartSent);
        Serial.print("  UART received:       ");
        Serial.println(uartReceived);
        Serial.println("==================================");
    }

    // ── HELP: Show command reference ────────────────────────────────
    else if (input == "HELP") {
        printHelp();
    }

    // ── Any other input: forward to XPLORER Pico via UART ───────────
    else {
        Serial.print("  -> Forwarding to UART: ");
        Serial.println(input);
        Serial1.println(input);
        uartSent++;
    }

    Serial.println(F("────────────────────────────────────"));
}

// ════════════════════════════════════════════════════════════════════
//  updateHeartbeat()
//  Blinks the onboard LED to show the bridge is running.
//  Pattern: 50ms on, 950ms off (approx 1-second cycle).
// ════════════════════════════════════════════════════════════════════
void updateHeartbeat() {
    unsigned long now = millis();
    unsigned long elapsed = now - lastHeartbeatToggle;

    if (heartbeatLedOn) {
        // LED is currently on – turn it off after HEARTBEAT_ON_MS
        if (elapsed >= HEARTBEAT_ON_MS) {
            digitalWrite(HEARTBEAT_LED, LOW);
            heartbeatLedOn = false;
            lastHeartbeatToggle = now;
        }
    } else {
        // LED is currently off – turn it on after HEARTBEAT_OFF_MS
        if (elapsed >= HEARTBEAT_OFF_MS) {
            digitalWrite(HEARTBEAT_LED, HIGH);
            heartbeatLedOn = true;
            lastHeartbeatToggle = now;
        }
    }
}

// ════════════════════════════════════════════════════════════════════
//  printHelp()
//  Prints the command reference to the Serial Monitor.
//  Uses F() macro to store strings in flash memory (saves SRAM).
// ════════════════════════════════════════════════════════════════════
void printHelp() {
    Serial.println(F("=== Bridge Commands (MQTT-UART) ==="));
    Serial.println(F("  HELP           Show this help"));
    Serial.println(F("  STATUS         Show bridge and UART status"));
    Serial.println(F("  STATS          Show message statistics"));
    Serial.println(F(""));
    Serial.println(F("Any other input is forwarded to XPLORER Pico via UART:"));
    Serial.println(F("  LED:1:NEXT     Cycle LED 1 to its next color"));
    Serial.println(F("  LED:2:NEXT     Cycle LED 2 to its next color"));
    Serial.println(F("  LED:3:NEXT     Cycle LED 3 to its next color"));
    Serial.println(F("  SERVO:90       Move servo to 90 degrees"));
    Serial.println(F("====================================="));
}

// ════════════════════════════════════════════════════════════════════
//  setup()
//  Runs once at power-on / reset:
//    1. Configure the heartbeat LED pin as output
//    2. Open the Serial connection and wait for the monitor to open
//    3. Initialize Serial1 (UART to XPLORER Pico)
//    4. Print a startup banner
//    5. Connect to WiFi
//    6. Configure and initialise the MQTT client
//    7. Seed the random number generator for unique client IDs
// ════════════════════════════════════════════════════════════════════
void setup() {
    pinMode(HEARTBEAT_LED, OUTPUT);  // Heartbeat LED as output
    digitalWrite(HEARTBEAT_LED, LOW);
    heartbeatLedOn = false;
    lastHeartbeatToggle = millis();

    // ── Serial (USB) ─────────────────────────────────────────────────
    Serial.begin(115200);
    delay(1000);                      // Give the monitor time to open

    // ── Serial1 (UART to XPLORER Pico) ──────────────────────────────
    Serial1.begin(SERIAL1_BAUD);
    delay(100);  // Wait for Serial1 to stabilize

    // ── Startup banner ───────────────────────────────────────────────
    Serial.println(F("========================================"));
    Serial.println(F("  Task 4: MQTT-UART Bridge (Nano)"));
    Serial.println(F("  Wokwi Pico W <-> XPLORER Pico"));
    Serial.println(F("  Broker: 141.69.95.10:1883"));
    Serial.println(F("========================================"));
    Serial.print(F("  Subscribe: "));
    Serial.println(TOPIC_CMD);
    Serial.print(F("  Publish:   "));
    Serial.println(TOPIC_DATA);
    Serial.print(F("  Token:     "));
    Serial.println(SHARED_TOKEN);
    Serial.print(F("  UART:      "));
    Serial.print(SERIAL1_BAUD);
    Serial.println(F(" baud (D0/D1)"));
    Serial.println(F("────────────────────────────────────"));

    // ── WiFi ─────────────────────────────────────────────────────────
    setupWiFi();

    // ── MQTT ─────────────────────────────────────────────────────────
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    mqtt.setCallback(mqttCallback);

    // Seed the RNG with analog noise XOR'd with microsecond timer
    randomSeed(analogRead(A0) ^ micros());

    Serial.println(F("────────────────────────────────────"));
    Serial.println(F("Type HELP for available commands"));
    Serial.println(F("────────────────────────────────────"));
}

// ════════════════════════════════════════════════════════════════════
//  loop()
//  Main execution loop – runs continuously after setup():
//    1. Keep the MQTT connection alive (reconnect if dropped)
//    2. Let PubSubClient process incoming messages and keepalives
//    3. Update the heartbeat LED
//    4. Read UART input from XPLORER Pico and publish to MQTT
//    5. Read Serial Monitor input and forward to XPLORER or handle locally
//    6. Run the watchdog to detect XPLORER Pico timeouts
// ════════════════════════════════════════════════════════════════════
void loop() {
    // ── MQTT keep-alive ──────────────────────────────────────────────
    if (!mqtt.connected()) {
        mqttReconnect();
    }
    // Must be called every loop iteration to process incoming packets
    mqtt.loop();

    // ── Heartbeat LED ────────────────────────────────────────────────
    updateHeartbeat();

    // ── UART input (from XPLORER Pico) ──────────────────────────────
    // Non-blocking: only processes when data is available on Serial1
    processUartInput();

    // ── Serial input (from USB monitor) ──────────────────────────────
    // Non-blocking: only processes when data is available on Serial
    processSerialInput();

    // ── Watchdog: detect XPLORER Pico timeout ───────────────────────
    // Fire only once per timeout event (xplorerTimeout flag prevents spam)
    // Reset when the next valid UART message arrives
    if (!xplorerTimeout && lastUartResponse > 0
        && millis() - lastUartResponse >= WATCHDOG_TIMEOUT) {
        xplorerTimeout = true;
        Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
        Serial.println(F("! WATCHDOG: XPLORER Pico unreachable! !"));
        Serial.println(F("! No UART data for 20s.               !"));
        Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
        Serial.println(F("────────────────────────────────────"));
    }
}
