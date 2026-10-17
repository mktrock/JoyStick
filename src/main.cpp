// M5StickC Plus + Keyestudio KS0008 Joystick Module
//
// Wiring:
//   KS0008 GND -> GND
//   KS0008 VCC -> 3V3  (do NOT use 5V: output would exceed the ESP32 ADC range)
//   KS0008 X   -> G33 (Grove)
//   KS0008 Y   -> G32 (Grove)
//   KS0008 SW  -> G26 (header)

#include <M5Unified.h>

M5Canvas canvas(&M5.Display);
int centerX = 2048;
int centerY = 2048;

// Current JoyOrbit mode, sent by the add-in as "M:<mode>\n" ("M:" = add-in stopped).
// The device only displays it; JoyOrbit owns the mode state.
String hostMode;

static void pollHost() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.startsWith("M:")) hostMode = line.substring(2);
      line = "";
    } else if (line.length() < 32) {
      line += c;
    }
  }
}

constexpr int PIN_X  = 33;
constexpr int PIN_Y  = 32;
constexpr int PIN_SW = 26;

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

static void calibrate() {
  readXY(centerX, centerY, 32);
  Serial.printf("# calibrated center: x=%d y=%d\n", centerX, centerY);
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  M5.Display.setRotation(1);
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  pinMode(PIN_SW, INPUT_PULLDOWN);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // 0-3.3V -> 0-4095

  delay(100);
  calibrate();
}

void loop() {
  M5.update();
  if (M5.BtnA.wasPressed()) calibrate();

  int x, y;
  readXY(x, y);
  int sw = readSW();
  pollHost();

  int dx = x - centerX;
  int dy = y - centerY;

  // KS0008 sample-compatible CSV: x,y,sw
  Serial.printf("%d,%d,%d\n", x, y, sw);

  const int w = canvas.width();
  const int h = canvas.height();
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextSize(2);
  canvas.setCursor(4, 4);
  canvas.printf("X:%4d %+5d\n", x, dx);
  canvas.setCursor(4, 24);
  canvas.printf("Y:%4d %+5d\n", y, dy);
  canvas.setCursor(4, 44);
  canvas.setTextColor(sw == LOW ? TFT_RED : TFT_GREEN);
  canvas.print(sw == LOW ? "SW: PRESSED" : "SW: ----");

  // JoyOrbit mode (top right)
  if (hostMode.length()) {
    String label = hostMode;
    label.toUpperCase();
    canvas.setTextColor(hostMode == "pan" ? TFT_YELLOW : TFT_CYAN);
    canvas.setTextDatum(top_right);
    canvas.drawString(label, w - 4, 4);
    canvas.setTextDatum(top_left);
  }

  // Battery (AXP192 is polled once per second)
  static uint32_t lastBatMs = 0;
  static int batLevel = -1;
  static int batMv = 0;
  static bool charging = false;
  if (batLevel < 0 || millis() - lastBatMs >= 1000) {
    lastBatMs = millis();
    batLevel = M5.Power.getBatteryLevel();
    batMv = M5.Power.getBatteryVoltage();
    charging = M5.Power.isCharging() == m5::Power_Class::is_charging;
  }
  canvas.setCursor(4, 68);
  canvas.setTextColor(batLevel <= 20 ? TFT_RED : batLevel <= 50 ? TFT_YELLOW : TFT_GREEN);
  canvas.printf("BAT:%3d%%", batLevel);
  canvas.setTextSize(1);
  canvas.setCursor(4, 88);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.printf("%d.%02dV %s", batMv / 1000, (batMv % 1000) / 10, charging ? "CHARGING" : "");
  canvas.setTextSize(2);

  // Position indicator
  const int r = 28;
  const int cx = w - r - 8;
  const int cy = h / 2;
  canvas.drawCircle(cx, cy, r, TFT_DARKGREY);
  int px = cx + dx * r / 2048;
  int py = cy + dy * r / 2048;
  canvas.fillCircle(px, py, 5, sw == LOW ? TFT_RED : TFT_CYAN);

  canvas.setTextColor(TFT_DARKGREY);
  canvas.setTextSize(1);
  canvas.setCursor(4, h - 12);
  canvas.print("BtnA: recalibrate");

  canvas.pushSprite(0, 0);
  delay(50);
}
