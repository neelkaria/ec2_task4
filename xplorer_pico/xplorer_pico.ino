/*
  ════════════════════════════════════════════════════════════════════════════════
  Institut fuer Elektromobilitaet (IEM)
  Hochschule Ravensburg-Weingarten

  Projekt:      XPLORER Pico - UART Bridge Controller
  Embedded Systems - SS 2026, Task 4

  Hardware:     Raspberry Pi Pico on Joy-IT XPLORER board
  Betriebssystem: RP2040 (Pico)
  Programmiersprache: C++ (Arduino)

  Beschreibung: Pico-basierte UART-Kommandozentrale mit NeoPixel-LEDs, Servo,
                DHT11-Sensor und Taster-Input. Kommuniziert mit Arduino Nano 33 IoT
                über serielles UART-Protokoll (115200 baud).

  Author:       Embedded Systems Lab
  Datum:        SS 2026
  Version:      1.0

  ════════════════════════════════════════════════════════════════════════════════
  LIBRARIES REQUIRED:
  - Adafruit NeoPixel (https://github.com/adafruit/Adafruit_NeoPixel)
  - DHT sensor library (https://github.com/adafruit/DHT-sensor-library)
  - Servo (built into arduino-pico core)
  ════════════════════════════════════════════════════════════════════════════════
*/

#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <DHT.h>

// ════════════════════════════════════════════════════════════════════════════════
// GPIO PIN DEFINITIONS
// ════════════════════════════════════════════════════════════════════════════════

#define PIN_NEOPIXEL    1      // WS2812 NeoPixel data line (GP1)
#define PIN_SERVO       7      // Servo signal line (GP7)
#define PIN_DHT11       0     // DHT11 sensor line (GP22)

#define PIN_BTN_TOP     10     // Button: Top (GP10)
#define PIN_BTN_RIGHT   11     // Button: Right (GP11)
#define PIN_BTN_BOTTOM  14     // Button: Bottom (GP14)
#define PIN_BTN_LEFT    15     // Button: Left (GP15)

#define PIN_UART_TX     4      // UART TX (GP4)
#define PIN_UART_RX     5      // UART RX (GP5)
#define PIN_HEARTBEAT   23
// ────────────────────────────────────────────────────────────────────────────────
// HARDWARE CONFIGURATION
// ────────────────────────────────────────────────────────────────────────────────

#define NUM_LEDS        4      // Number of WS2812 LEDs
#define DHTTYPE         DHT11  // DHT sensor type

#define UART_BAUD       115200 // Serial2 (UART) baud rate
#define USB_BAUD        115200 // Serial (USB) baud rate for debug

#define SERVO_MIN       0      // Minimum servo angle
#define SERVO_MAX       180    // Maximum servo angle

#define BUTTON_DEBOUNCE 50     // Button debounce time (ms)
#define DHT_READ_INTERVAL 2000 // DHT read interval (2 seconds)



// ════════════════════════════════════════════════════════════════════════════════
// OBJECT DECLARATIONS
// ════════════════════════════════════════════════════════════════════════════════

Adafruit_NeoPixel pixels(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Servo servo1;
DHT dht(PIN_DHT11, DHTTYPE);

// ────────────────────────────────────────────────────────────────────────────────
// STATE VARIABLES
// ────────────────────────────────────────────────────────────────────────────────

// LED color storage
String ledColorNames[NUM_LEDS] = {"OFF", "OFF", "OFF", "OFF"};
uint32_t ledColors[NUM_LEDS] = {0x000000, 0x000000, 0x000000, 0x000000};

// Color cycle sequence (used by LED:N:NEXT and by physical XPLORER buttons).
// Each LED has an independent index that advances on every "NEXT" event.
const char* COLOR_CYCLE[] = {"RED", "GREEN", "BLUE", "YELLOW", "CYAN", "PURPLE", "WHITE", "OFF"};
const int   NUM_COLORS    = 8;
int ledColorIdx[NUM_LEDS] = {7, 7, 7, 7};   // start at OFF; first NEXT -> RED

// Servo state
int servoAngle = 90;

// Button state tracking (edge detection)
bool lastButtonState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long lastButtonTime[4] = {0, 0, 0, 0};

// DHT sensor timing
unsigned long lastDHTReadTime = 0;
float lastTemperature = 0.0;
int lastHumidity = 0;

// Heartbeat LED
bool hbOutputState = false;
unsigned long hbCurrentTime = 0;
unsigned long hbPreviousTime = 0;

// ════════════════════════════════════════════════════════════════════════════════
// COLOR MAPPING FUNCTION
// ════════════════════════════════════════════════════════════════════════════════

/*
  Converts color name string to RGB uint32_t value for NeoPixel
  Supported colors: RED, GREEN, BLUE, WHITE, YELLOW, CYAN, PURPLE, OFF
*/
uint32_t getColorFromName(String colorName) {
  // TODO: Convert colorName (case-insensitive) to NeoPixel RGB color.
  // Map "RED", "GREEN", "BLUE", "WHITE", "YELLOW", "CYAN", "PURPLE" to appropriate RGB values.
  // Return pixels.Color(r, g, b) for each; return Color(0,0,0) for unrecognized or "OFF".

  colorName.toUpperCase();
  if (colorName == "RED")    return pixels.Color(255, 0, 0);
  if (colorName == "GREEN")  return pixels.Color(0, 255, 0);
  if (colorName == "BLUE")   return pixels.Color(0, 0, 255);
  if (colorName == "YELLOW") return pixels.Color(255, 255, 0);
  if (colorName == "CYAN")   return pixels.Color(0, 255, 255);
  if (colorName == "PURPLE") return pixels.Color(128, 0, 128);
  if (colorName == "WHITE")  return pixels.Color(255, 255, 255);

  return pixels.Color(0, 0, 0); // "OFF" or unrecognized

}

// ════════════════════════════════════════════════════════════════════════════════
// LED COLOR CYCLE
// ════════════════════════════════════════════════════════════════════════════════

/*
  Advance LED n (1..NUM_LEDS) by one step in COLOR_CYCLE, drive the pixel,
  and emit ACK:LED:n:COLOR over UART so the Wokwi side mirrors the new state.
  Same code path is called from the LED:N:NEXT command handler AND from the
  local button handler — XPLORER stays the single source of truth.

  This helper is provided fully implemented; you do not need to modify it.
*/
void advanceLedColor(int n) {
  if (n < 1 || n > NUM_LEDS) return;
  int idx = n - 1;
  ledColorIdx[idx] = (ledColorIdx[idx] + 1) % NUM_COLORS;
  String colorName = String(COLOR_CYCLE[ledColorIdx[idx]]);
  uint32_t color = getColorFromName(colorName);
  pixels.setPixelColor(idx, color);
  pixels.show();
  ledColorNames[idx] = colorName;
  ledColors[idx]     = color;

  Serial2.print(F("ACK:LED:"));
  Serial2.print(n);
  Serial2.print(':');
  Serial2.println(colorName);

  Serial.print(F("[LED")); Serial.print(n);
  Serial.print(F("] cycled -> "));
  Serial.println(colorName);
}

// ════════════════════════════════════════════════════════════════════════════════
// UART COMMAND PROCESSING
// ════════════════════════════════════════════════════════════════════════════════

/*
  Parses and executes UART commands received from Nano 33 IoT.
  Supported formats:
    - LED:N:NEXT     (cycle LED N to the next color in COLOR_CYCLE)
    - SERVO:angle    (set servo angle, 0..180)
    - STATUS         (request a full STATUS report back)
*/
void processUARTCommand(String command) {
  command.trim();
  command.toUpperCase();

  Serial.print(F("[DEBUG] Received: "));
  Serial.println(command);

  // TODO: Implement the command dispatcher.
  //
  // For "LED:N:NEXT":
  //   - Parse N from the command (use indexOf(':') + substring()).
  //   - Verify the action token equals "NEXT" (the only LED action in this design).
  //   - Call the provided helper advanceLedColor(N) — it advances the color,
  //     drives the LED, AND emits ACK:LED:N:COLOR via Serial2 for you.
  
  if(command.startsWith("LED")){
    int colon1 = command.indexOf(":");
    int colon2 = command.indexOf(":", colon1 + 1);

    // checking incomplete commands
    if(colon2 > -1) {
      int N = command.substring(colon1 + 1, colon2).toInt();
      String next = command.substring(colon2 + 1);
     
      if (next == "NEXT"){
        advanceLedColor(N);
      }
    }
  }

  // For "SERVO:angle":
  //   - Extract angle with substring(6).toInt()
  //   - Constrain to [SERVO_MIN, SERVO_MAX] and call servo1.write(angle)
  //   - Update the servoAngle state variable (needed for STATUS reports)
  //   - Send ACK "ACK:SERVO:angle" via Serial2.
  
  else if (command.startsWith("SERVO")) {
    
    servoAngle = command.substring(6).toInt();

    // constraining to [SERVO_MIN, SERVO_MAX]
    servo1.write((servoAngle >= SERVO_MAX) ? SERVO_MAX : (servoAngle <= SERVO_MIN) ? SERVO_MIN : servoAngle);
    Serial2.print("ACK:SERVO:");
    Serial2.println(servoAngle);
  }

  // For "STATUS": simply call the provided sendStatusReport() helper.
  else if (command == "STATUS") sendStatusReport();
}

// ════════════════════════════════════════════════════════════════════════════════
// STATUS REPORTING
// ════════════════════════════════════════════════════════════════════════════════

/*
  Sends full status report to Nano 33 IoT
  Format: STATUS:LED1=COLOR,LED2=COLOR,LED3=COLOR,LED4=COLOR,SERVO=angle,TEMP=xx.x,HUM=xx
*/
void sendStatusReport() {
  Serial2.print(F("STATUS:LED1="));
  Serial2.print(ledColorNames[0]);
  Serial2.print(F(",LED2="));
  Serial2.print(ledColorNames[1]);
  Serial2.print(F(",LED3="));
  Serial2.print(ledColorNames[2]);
  Serial2.print(F(",LED4="));
  Serial2.print(ledColorNames[3]);
  Serial2.print(F(",SERVO="));
  Serial2.print(servoAngle);
  Serial2.print(F(",TEMP="));
  Serial2.print(lastTemperature);
  Serial2.print(F(",HUM="));
  Serial2.println(lastHumidity);

  Serial.println(F("[STATUS] Report sent"));
}

// ════════════════════════════════════════════════════════════════════════════════
// BUTTON HANDLING
// ════════════════════════════════════════════════════════════════════════════════

/*
  Reads all buttons and detects falling edge (press) events
  Uses debouncing with millis() tracking
*/
void readButtons() {
  // TODO: Read digital state of all 4 buttons (TOP, RIGHT, BOTTOM, LEFT) and
  // detect falling edges (HIGH to LOW transition) with debounce.
  //
  // Mapping (single source of truth — same path as the LED:N:NEXT command):
  //   TOP    -> advanceLedColor(1)
  //   RIGHT  -> advanceLedColor(2)
  //   BOTTOM -> advanceLedColor(3)
  //   LEFT   -> advanceLedColor(4)
  // advanceLedColor() already drives the LED AND emits ACK:LED:N:COLOR via UART,
  // so you do NOT need to publish anything else from this function.
  //
  // Hint: The state arrays lastButtonState[4] and lastButtonTime[4] are already
  // declared at the top of the file. Use BUTTON_DEBOUNCE (ms) as the debounce window.
  // Example pin array: const int buttons[4] = {PIN_BTN_TOP, PIN_BTN_RIGHT, PIN_BTN_BOTTOM, PIN_BTN_LEFT};

  const int pins[4] = {PIN_BTN_TOP, PIN_BTN_RIGHT, PIN_BTN_BOTTOM, PIN_BTN_LEFT};
  unsigned long now = millis();

  for (int i = 0; i < 4; i++) {
      bool currentState = digitalRead(pins[i]);
      
      // Detect HIGH to LOW edge
      if (lastButtonState[i] == HIGH && currentState == LOW) {
          if (now - lastButtonTime[i] > BUTTON_DEBOUNCE) {
              // i + 1 maps correctly to LEDs 1 through 4
              advanceLedColor(i + 1); 
              lastButtonTime[i] = now;
          }
      }
      lastButtonState[i] = currentState;
  }
}

// ════════════════════════════════════════════════════════════════════════════════
// DHT SENSOR HANDLING
// ════════════════════════════════════════════════════════════════════════════════

/*
  Reads DHT11 sensor and sends temperature/humidity readings
  Reads occur every DHT_READ_INTERVAL milliseconds
*/
void readDHTSensor() {
  // TODO: Read DHT11 temperature and humidity every DHT_READ_INTERVAL ms.
  // Use lastDHTReadTime + millis() for the interval check.
  // Read both humidity (dht.readHumidity()) and temperature (dht.readTemperature()).
  // Validate BOTH with isnan() — only send values if both reads are valid
  //   (otherwise you would publish "nan" over UART and break parsers downstream).
  // On success: update lastTemperature/lastHumidity, send "TEMP:value" and "HUM:value"
  //   via Serial2 (newline-terminated), and log to USB Serial for debugging.
  // On failure: log a DHT read error to USB Serial and skip this cycle.

  unsigned long now = millis();
  if (now - lastDHTReadTime >= DHT_READ_INTERVAL) {
      lastDHTReadTime = now;

      // faulty sensor
      float h = dht.readHumidity();
      float t = dht.readTemperature();

      // Validate BOTH readings before publishing
      if (isnan(h) || isnan(t)) {
          Serial.println(" Invalid values from DHT sensor!");
          return;
      }

      lastHumidity = (int)h;
      lastTemperature = t;

      // Emit to UART bridge
      Serial2.print(F("TEMP:"));Serial2.println(lastTemperature);
      Serial2.print(F("HUM:"));Serial2.println(lastHumidity);

      // Debug logging
      Serial.print("Temp: ");Serial.print(lastTemperature);
      Serial.print("Hum: ");Serial.print(lastHumidity);Serial.println((" %"));
  }
}

void heartBeat()
{
  hbCurrentTime = millis();
  if ((hbCurrentTime - hbPreviousTime) >= 500) 
  {
    hbPreviousTime = hbCurrentTime;
    hbOutputState ^= 1;
    digitalWrite(LED_BUILTIN, hbOutputState);
  }
}

// ════════════════════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════════════════════

void setup() {
  // Initialize USB Serial for debug output
  Serial.begin(USB_BAUD);

  delay(2000);

  Serial.println();
  Serial.println(F("+---------------------------------------------------------------+"));
  Serial.println(F("|       XPLORER Pico - UART Bridge Controller (Task 4)          |"));
  Serial.println(F("|     Institut fuer Elektromobilitaet, HS Ravensburg-Weingarten |"));
  Serial.println(F("|                    Embedded Lab SS 2026                       |"));
  Serial.println(F("+---------------------------------------------------------------+"));
  Serial.println();

  // Initialize UART1 on GPIO4 (TX) and GPIO5 (RX)
  Serial2.setTX(PIN_UART_TX);
  Serial2.setRX(PIN_UART_RX);
  Serial2.begin(UART_BAUD);

  Serial.print(F("[INIT] UART1 configured: TX=GP"));
  Serial.print(PIN_UART_TX);
  Serial.print(F(", RX=GP"));
  Serial.print(PIN_UART_RX);
  Serial.print(F(", Baud="));
  Serial.println(UART_BAUD);

  // Initialize NeoPixel LEDs
  pixels.begin();
  pixels.clear();
  
  // testing the LEDs on init
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.setPixelColor(1, pixels.Color(255, 0, 0)); 
  pixels.setPixelColor(2, pixels.Color(0, 255, 0)); 
  pixels.setPixelColor(3, pixels.Color(0, 0, 255)); 
  
  pixels.show();
  Serial.println(F("[INIT] NeoPixel initialized (4x WS2812 on GP1)"));

  // Initialize Servo
  servo1.attach(PIN_SERVO);
  servo1.write(servoAngle);
  Serial.print(F("[INIT] Servo initialized on GP"));
  Serial.print(PIN_SERVO);
  Serial.print(F(", angle="));
  Serial.println(servoAngle);

  // Initialize DHT sensor
  delay(2000);
  dht.begin();
  Serial.print(F("[INIT] DHT11 initialized on GP"));
  Serial.println(PIN_DHT11);

  // Initialize buttons with pull-up
  pinMode(PIN_BTN_TOP, INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BTN_BOTTOM, INPUT_PULLUP);
  pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
  
  Serial.println(F("[INIT] Buttons initialized (GP10, GP11, GP14, GP15)"));
  
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.println();
  Serial.println(F("[READY] System ready - waiting for UART commands..."));
  Serial.println();

  pixels.clear();
  pixels.show();

}

// ════════════════════════════════════════════════════════════════════════════════
// MAIN LOOP
// ════════════════════════════════════════════════════════════════════════════════

void loop() {
  // ────────────────────────────────────────────────────────────────────────────────
  // Process incoming UART commands (non-blocking)
  // ────────────────────────────────────────────────────────────────────────────────
  // heartBeat();
  
heartBeat();

  if (Serial2.available()) {
    static String uartBuffer = "";
    char inChar = Serial2.read();

    if (inChar == '\n') {
      // Complete command received
      if (uartBuffer.length() > 0) {
        processUARTCommand(uartBuffer);
      }
      uartBuffer = "";
    } else if (inChar != '\r') {
      // Accumulate characters (skip carriage returns)
      uartBuffer += inChar;
    }
  }

  // ────────────────────────────────────────────────────────────────────────────────
  // Non-blocking sensor and I/O handling
  // ────────────────────────────────────────────────────────────────────────────────

  readButtons();
  readDHTSensor();

  // Small delay to prevent CPU overload
  delayMicroseconds(100);
}


// ════════════════════════════════════════════════════════════════════════════════
// END OF FILE
// ════════════════════════════════════════════════════════════════════════════════
