// Seeed Studio XIAO ESP32C3 + Keyestudio KS0008 Joystick Module
//
// Wiring:
//   KS0008 GND -> GND
//   KS0008 VCC -> 3V3  (do NOT use 5V: output would exceed the ESP32-C3 ADC range)
//   KS0008 Y   -> D1 (GPIO3, ADC1)
//   KS0008 X   -> D2 (GPIO4, ADC1)
//   (KS0008 SW is not used and stays unconnected.)
//   Encoder GND -> GND, + -> 3V3 (shared with KS0008; not 5V, the module's
//                  pull-ups would put 5V on the GPIOs)
//   Encoder CLK -> D4 (GPIO6)
//   Encoder DT  -> D5 (GPIO7)
//   Encoder SW  -> D3 (GPIO5)  (this is the sw in the CSV)
//   Left  button (micro switch) NO -> D10 (GPIO10), COM -> GND (NC unused)
//   Right button (micro switch) NO -> D7  (GPIO20), COM -> GND (NC unused)
// The on-board BOOT button recalibrates the center.
//
// The rotary encoder goes out on its own line, "E:<detents>", only when it
// turns: detents since the last E line (+ = clockwise). The buttons go out as
// "B:<n>,<state>" (n = 1 left, 2 right; state 0 = pressed, like sw) when
// they change.
// The x,y,sw lines are unchanged.
//
// Built with -DSIMULATE (env xiao-esp32c3-sim), no wiring is needed: x,y go
// round in circles and the BOOT button acts as sw.
//
// Output goes to USB serial. Built with -DENABLE_BLE (env xiao-esp32c3-ble),
// it also goes, while a central is connected, to BLE (Nordic UART Service,
// device name "JoyStick-XIAO") in the same format. Without it, neither BLE
// nor WiFi is started (the radio stays off, which keeps the chip cooler).

#include <Arduino.h>
#ifdef ENABLE_BLE
#include <NimBLEDevice.h>
#endif

constexpr int PIN_X    = A2;
constexpr int PIN_Y    = A1;
constexpr int PIN_SW   = D3;
constexpr int PIN_BOOT = 9;  // on-board BOOT button, LOW while pressed
constexpr int PIN_ENC_A  = D4;   // CLK
constexpr int PIN_ENC_B  = D5;   // DT
constexpr int PIN_BTN[] = {D10, D7};  // left (1), right (2)

int centerX = 2048;
int centerY = 2048;

// ---- Rotary encoder -----------------------------------------------------------
// Quadrature decoding on every edge of both lines: bounce just steps back and
// forth, so no debounce is needed. One detent = one full cycle = 4 steps; flip
// ENC_DIR if clockwise counts down with your module.
constexpr int ENC_STEPS_PER_DETENT = 4;
constexpr int ENC_DIR = 1;

volatile int32_t encSteps = 0;
volatile uint8_t encState = 0;  // previous (A << 1 | B)

static void IRAM_ATTR onEncoderEdge() {
  // Index: previous AB << 2 | current AB. Invalid (skipped) transitions count 0.
  static const int8_t DELTA[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  const uint32_t in = REG_READ(GPIO_IN_REG);  // digitalRead() isn't IRAM-safe
  const uint8_t ab = ((in >> PIN_ENC_A) & 1) << 1 | ((in >> PIN_ENC_B) & 1);
  encSteps += DELTA[encState << 2 | ab];
  encState = ab;
}

static void setupEncoder() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encState = digitalRead(PIN_ENC_A) << 1 | digitalRead(PIN_ENC_B);
  attachInterrupt(PIN_ENC_A, onEncoderEdge, CHANGE);
  attachInterrupt(PIN_ENC_B, onEncoderEdge, CHANGE);
}

// Detents from steps, rounded to the nearest detent so jitter around a rest
// position doesn't flicker between two counts.
static int32_t encDetents() {
  noInterrupts();
  const int32_t s = ENC_DIR * encSteps;
  interrupts();
  const int32_t half = ENC_STEPS_PER_DETENT / 2;
  return s >= -half ? (s + half) / ENC_STEPS_PER_DETENT
                    : -((-s - half + ENC_STEPS_PER_DETENT - 1) / ENC_STEPS_PER_DETENT);
}

// ---- Buttons ------------------------------------------------------------------
// Micro switches between the pin and GND (internal pull-up: LOW while
// pressed). They bounce, so a new level counts only after it has held for
// BTN_DEBOUNCE_MS.
constexpr int NUM_BTN = sizeof(PIN_BTN) / sizeof(PIN_BTN[0]);
constexpr uint32_t BTN_DEBOUNCE_MS = 20;

static void setupButtons() {
  for (int pin : PIN_BTN) pinMode(pin, INPUT_PULLUP);
}

// ---- BLE (Nordic UART Service) ----------------------------------------------
#ifdef ENABLE_BLE
#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // central -> device
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // device -> central

NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* bleTx = nullptr;

// RX writes arrive on the NimBLE task; queue them for loop() to parse.
// (Fixed buffer: no heap allocation inside the critical section.)
portMUX_TYPE bleRxMux = portMUX_INITIALIZER_UNLOCKED;
char bleRx[128];
size_t bleRxLen = 0;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onDisconnect(NimBLEServer*) override { NimBLEDevice::startAdvertising(); }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) override {
    NimBLEAttValue v = c->getValue();
    portENTER_CRITICAL(&bleRxMux);
    for (size_t i = 0; i < v.length() && bleRxLen < sizeof(bleRx); i++) bleRx[bleRxLen++] = v.data()[i];
    portEXIT_CRITICAL(&bleRxMux);
  }
};

static void setupBle() {
  NimBLEDevice::init("JoyStick-XIAO");
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());
  NimBLEService* svc = bleServer->createService(NUS_SERVICE);
  bleTx = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* rx =
      svc->createCharacteristic(NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  svc->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);  // the name doesn't fit next to the 128-bit UUID
  adv->start();
}

static bool bleConnected() { return bleServer && bleServer->getConnectedCount() > 0; }
#endif

// Send one line (with '\n') to USB serial and, if connected, BLE.
static void emit(const char* line) {
  Serial.print(line);
#ifdef ENABLE_BLE
  if (bleConnected()) {
    bleTx->setValue((const uint8_t*)line, strlen(line));
    bleTx->notify();
  }
#endif
}

// JoyOrbit sends "M:<mode>\n"; with no display, echo it as a comment line.
static void feedHost(char c) {
  static String line;
  if (c == '\n') {
    line.trim();
    if (line.startsWith("M:")) {
      String msg = "# mode: " + line.substring(2) + "\n";
      emit(msg.c_str());
    }
    line = "";
  } else if (line.length() < 32) {
    line += c;
  }
}

static void pollHost() {
  while (Serial.available()) feedHost(Serial.read());
#ifdef ENABLE_BLE
  char rx[sizeof(bleRx)];
  portENTER_CRITICAL(&bleRxMux);
  size_t n = bleRxLen;
  memcpy(rx, bleRx, n);
  bleRxLen = 0;
  portEXIT_CRITICAL(&bleRxMux);
  for (size_t i = 0; i < n; i++) feedHost(rx[i]);
#endif
}

// ---- Input ------------------------------------------------------------------
#ifdef SIMULATE
// Circle around the center (~0.5 rev/s, radius 1500) with a little noise.
// Stays at the center for the first 1.5 s (JoyOrbit calibrates from the first
// samples) and while BOOT (= sw) is held.
static void readXY(int& x, int& y, int = 0) {
  static uint32_t lastMs = millis();
  static float angle = 0;
  const uint32_t now = millis();
  const bool moving = now >= 1500 && digitalRead(PIN_BOOT) == HIGH;
  if (moving) angle = fmodf(angle + TWO_PI * 0.5f * (now - lastMs) / 1000.0f, TWO_PI);
  lastMs = now;
  x = 2048 + random(-20, 21) + (moving ? (int)(1500 * cosf(angle)) : 0);
  y = 2048 + random(-20, 21) + (moving ? (int)(1500 * sinf(angle)) : 0);
}

static int readSW() { return digitalRead(PIN_BOOT); }
#else
static int readAvg(int pin, int samples) {
  long sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return sum / samples;
}

// Rotated 90 degrees clockwise (raw y grows downward): right -> down,
// up -> right. The calibrated center goes through this too.
static void readXY(int& x, int& y, int samples = 8) {
  const int rawX = readAvg(PIN_X, samples);
  const int rawY = readAvg(PIN_Y, samples);
  x = 4095 - rawY;
  y = rawX;
}

// The encoder's SW pulls the pin to GND while pressed, which already matches
// the KS0008 sample-sketch convention: sw = 0 while pressed.
static int readSW() { return digitalRead(PIN_SW); }
#endif

// "B:<n>,<state>" for each debounced change (state 0 = pressed, like sw).
static void pollButtons() {
  static int stable[NUM_BTN] = {HIGH, HIGH};  // last level sent
  static int last[NUM_BTN] = {HIGH, HIGH};    // last level read
  static uint32_t changedMs[NUM_BTN] = {};
  const uint32_t now = millis();
  for (int i = 0; i < NUM_BTN; i++) {
    const int v = digitalRead(PIN_BTN[i]);
    if (v != last[i]) {
      last[i] = v;
      changedMs[i] = now;
    } else if (v != stable[i] && now - changedMs[i] >= BTN_DEBOUNCE_MS) {
      stable[i] = v;
      char buf[16];
      snprintf(buf, sizeof(buf), "B:%d,%d\n", i + 1, v);
      emit(buf);
    }
  }
}

static void calibrate() {
#ifndef SIMULATE
  readXY(centerX, centerY, 32);
#endif
  char buf[48];
  snprintf(buf, sizeof(buf), "# calibrated center: x=%d y=%d\n", centerX, centerY);
  emit(buf);
}

void setup() {
  // 160 -> 80 MHz to run cooler. Don't go lower: below 80 MHz the PLL stops
  // and USB Serial/JTAG (our only output without BLE) stops with it.
  setCpuFrequencyMhz(80);
  Serial.begin(115200);

  pinMode(PIN_SW, INPUT_PULLUP);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setupEncoder();
  setupButtons();

#ifdef ENABLE_BLE
  setupBle();
#endif

  delay(100);
  calibrate();
}

void loop() {
  // 100 Hz, on a fixed period (the reads below take time of their own)
  constexpr uint32_t PERIOD_MS = 10;
  static uint32_t nextMs = millis();
  while ((int32_t)(millis() - nextMs) < 0) delay(1);
  nextMs += PERIOD_MS;
  if ((int32_t)(millis() - nextMs) > 0) nextMs = millis();  // fell behind: don't burst

#ifndef SIMULATE
  // BOOT button: recalibrate on press
  static int prevBoot = HIGH;
  const int boot = digitalRead(PIN_BOOT);
  if (prevBoot == HIGH && boot == LOW) calibrate();
  prevBoot = boot;
#endif

  int x, y;
  readXY(x, y);
  x = constrain(x, 0, 4095);
  y = constrain(y, 0, 4095);
  int sw = readSW();
  pollHost();

  // KS0008 sample-compatible CSV: x,y,sw
  char csv[24];
  snprintf(csv, sizeof(csv), "%d,%d,%d\n", x, y, sw);
  emit(csv);

  // Encoder: "E:<detents since last E line>", only when it turned
  static int32_t sentDetents = 0;
  const int32_t detents = encDetents();
  if (detents != sentDetents) {
    char enc[24];
    snprintf(enc, sizeof(enc), "E:%ld\n", (long)(detents - sentDetents));
    emit(enc);
    sentDetents = detents;
  }

  pollButtons();
}
