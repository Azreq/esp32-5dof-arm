#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Firmware starter: manual servo-angle control only.
// FK/IK theory and calculations are demonstrated in simulation/simulator.py.
// Validate mechanics, servo limits, and power design before connecting real servos.

constexpr uint8_t SERVO_COUNT = 5;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int OLED_RESET = -1;
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint8_t OLED_ADDRESS = 0x3C;

const char *AP_SSID = "RobotArm-5DOF";
const char *AP_PASSWORD = "robotarm5"; // Change before use on a shared/public area.

const char *JOINT_NAMES[SERVO_COUNT] = {
  "Base", "Shoulder", "Elbow", "Wrist", "End joint"
};

const uint8_t SERVO_CHANNELS[SERVO_COUNT] = {0, 1, 2, 3, 4};
int servoAngles[SERVO_COUNT] = {90, 90, 90, 90, 90};

// Calibrate these pulse limits for the exact servo and linkage.
// Do not assume all servos safely support the full 0-180 degree range.
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int PCA_FREQUENCY_HZ = 50;

Adafruit_PWMServoDriver pwm(PCA9685_ADDRESS);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

uint16_t microsecondsToTicks(int pulseUs) {
  // PCA9685 period at 50 Hz is 20,000 microseconds; 4096 ticks per period.
  return static_cast<uint16_t>((static_cast<uint32_t>(pulseUs) * 4096UL) / 20000UL);
}

void writeServoAngle(uint8_t index, int angle) {
  if (index >= SERVO_COUNT) return;
  angle = constrain(angle, 0, 180);
  servoAngles[index] = angle;

  const int pulseUs = SERVO_MIN_US +
      (static_cast<long>(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
  pwm.setPWM(SERVO_CHANNELS[index], 0, microsecondsToTicks(pulseUs));
}

String makePage() {
  String html = R"HTML(
<!doctype html><html lang="id"><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Arm 5-DOF</title>
<style>
body{font-family:system-ui,sans-serif;max-width:680px;margin:24px auto;padding:0 16px;background:#f4f6f8;color:#17202a}
section{background:white;padding:16px;margin:12px 0;border-radius:12px}
label{display:block;font-weight:600;margin-bottom:8px}input{width:100%}output{font-variant-numeric:tabular-nums}
</style></head><body><h1>Robot Arm 5-DOF</h1>
<p>Kontrol sudut servo untuk demonstrasi joint. FK/IK dihitung di simulator Python.</p>
)HTML";

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    html += "<section><label>" + String(JOINT_NAMES[i]) +
            " — <output id='v" + String(i) + "'>" + String(servoAngles[i]) +
            "</output>&deg;</label><input type='range' min='0' max='180' value='" +
            String(servoAngles[i]) + "' oninput='document.getElementById(\"v" +
            String(i) + "\").value=this.value;fetch(\"/set?joint=" + String(i) +
            "&angle=\"+this.value)'></section>";
  }

  html += R"HTML(
<script>
</script><p>Wi-Fi AP lokal; tidak memerlukan internet.</p></body></html>
)HTML";
  return html;
}

void sendPage() {
  server.send(200, "text/html; charset=utf-8", makePage());
}

void handleSet() {
  if (!server.hasArg("joint") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Parameter joint dan angle diperlukan.");
    return;
  }

  const int joint = server.arg("joint").toInt();
  const int angle = server.arg("angle").toInt();

  if (joint < 0 || joint >= SERVO_COUNT || angle < 0 || angle > 180) {
    server.send(400, "text/plain", "Joint atau sudut di luar rentang.");
    return;
  }

  writeServoAngle(static_cast<uint8_t>(joint), angle);
  server.send(200, "text/plain", "OK");
  Serial.printf("%s: %d deg\n", JOINT_NAMES[joint], angle);
}

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Robot Arm 5-DOF");
  display.println(WiFi.softAPIP());
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    display.print(JOINT_NAMES[i]);
    display.print(": ");
    display.print(servoAngles[i]);
    display.println(" deg");
  }
  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Default ESP32 I2C pins: SDA 21, SCL 22 on common DevKit boards.

  pwm.begin();
  pwm.setPWMFreq(PCA_FREQUENCY_HZ);
  delay(10);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED tidak ditemukan; periksa alamat dan koneksi I2C.");
  }

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    writeServoAngle(i, servoAngles[i]);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("Wi-Fi AP aktif. Sambungkan ke ");
  Serial.println(AP_SSID);
  Serial.print("Buka http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, sendPage);
  server.on("/set", HTTP_GET, handleSet);
  server.begin();
  updateOLED();
}

void loop() {
  server.handleClient();
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = millis();
    updateOLED();
  }
}
