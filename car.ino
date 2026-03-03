#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// --- Wi-Fi credentials ---
const char* ssid = "wifiname";
const char* password = "password";

// --- Motor driver pins ---
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENA 14
#define ENB 12

// --- PWM Channels ---
#define CH_A 0
#define CH_B 1
#define PWM_FREQ 1000
#define PWM_RES 8  // 8-bit resolution (0–255)

// --- Motor control variables ---
String lastCommand = "STOP";
int motorSpeed = 180;  // Default speed
AsyncWebServer server(80);

// ---------------- Helper Functions ----------------
void setSpeed(int speedVal) {
  motorSpeed = constrain(speedVal, 0, 180);
  ledcWrite(CH_A, motorSpeed);
  ledcWrite(CH_B, motorSpeed);
  Serial.printf("Speed set to: %d\n", motorSpeed);
}

void moveForward() {
  ledcWrite(CH_A, motorSpeed);
  ledcWrite(CH_B, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  Serial.println("→ Moving Forward");
}

void moveLeft() {
  ledcWrite(CH_A, motorSpeed);
  ledcWrite(CH_B, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  Serial.println("↩ Turning Left");
}

void moveRight() {
  ledcWrite(CH_A, motorSpeed);
  ledcWrite(CH_B, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("↪ Turning Right");
}

void stopCar() {
  ledcWrite(CH_A, 0);
  ledcWrite(CH_B, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("⛔ Stop");
}

void executeCommand(String cmd) {
  cmd.toUpperCase();
  if (cmd == "FORWARD" || cmd == "STRAIGHT") moveForward();
  else if (cmd == "LEFT") moveLeft();
  else if (cmd == "RIGHT") moveRight();
  else if (cmd == "STOP") stopCar();
  else Serial.println("❓ Unknown command: " + cmd);

  lastCommand = cmd;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // PWM setup
  ledcAttachPin(ENA, CH_A);
  ledcAttachPin(ENB, CH_B);
  ledcSetup(CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);

  setSpeed(180);

  // Wi-Fi setup
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi Connected!");
  Serial.print("📡 ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "🚗 ESP32 Car Controller Running.\nUse /cmd?dir=LEFT/RIGHT/STRAIGHT/STOP");
  });

  server.on("/cmd", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("dir")) {
      String cmd = request->getParam("dir")->value();
      executeCommand(cmd);
      request->send(200, "text/plain", "✅ Command received: " + cmd);
    } else {
      request->send(400, "text/plain", "⚠️ Missing 'dir' parameter");
    }
  });

  server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("val")) {
      int val = request->getParam("val")->value().toInt();
      setSpeed(val);
      request->send(200, "text/plain", "✅ Speed set to " + String(val));
    } else {
      request->send(400, "text/plain", "⚠️ Missing 'val' parameter");
    }
  });

  server.begin();
  Serial.println("🌐 Web Server Started");
}

void loop() {
  // Async server → nothing here
}
