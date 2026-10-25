// Seeed Studio XIAO ESP32C3 + Keyestudio KS0008 Joystick Module
//
// Wiring:
//   KS0008 GND -> GND
//   KS0008 VCC -> 3V3  (do NOT use 5V: output would exceed the ESP32-C3 ADC range)
//   KS0008 Y   -> D1 (GPIO3, ADC1)
//   KS0008 X   -> D2 (GPIO4, ADC1)
//   KS0008 SW  -> D3 (GPIO5)
// The on-board BOOT button recalibrates the center.
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

int centerX = 2048;
int centerY = 2048;

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

static void readXY(int& x, int& y, int samples = 8) {
  x = readAvg(PIN_X, samples);
  y = readAvg(PIN_Y, samples);
}

// The module drives SW HIGH while pressed (LOW at rest). Invert it so the
// CSV keeps the KS0008 sample-sketch convention: sw = 0 while pressed.
static int readSW() { return digitalRead(PIN_SW) == HIGH ? LOW : HIGH; }
#endif

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

  pinMode(PIN_SW, INPUT_PULLDOWN);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

#ifdef ENABLE_BLE
  setupBle();
#endif

  delay(100);
  calibrate();
}

void loop() {
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

  delay(50);
}
