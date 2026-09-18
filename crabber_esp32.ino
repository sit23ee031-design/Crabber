/*
 * ╔══════════════════════════════════════════════════════════════════╗
 * ║              CRABBER ROV — ESP32 Firmware v2.4                   ║
 * ║──────────────────────────────────────────────────────────────────║
 * ║  Broker  : broker.emqx.io : 8883 (TLS)                          ║
 * ║  Command topics (subscribe to all 3):                            ║
 * ║    chennai2026 | 96260706 | 900398                               ║
 * ║  Telemetry : chennai2026/status  (JSON every 2 s)                ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  COMMANDS                                                        ║
 * ║    M1:<0-255>   Thruster PWM  (0=stop, 255=full)                 ║
 * ║    M2:<0-255>   Pump PWM — dashboard pre-inverts before sending  ║
 * ║                 (BTS7960 inverted: 255=stopped, 0=full speed)    ║
 * ║    SA:<30-150>  Servo absolute angle in degrees                  ║
 * ║    STOP_M1      Kill thruster                                    ║
 * ║    STOP_M2      Kill pump (writes 255 to inverted driver)        ║
 * ║    EMERGENCY    Kill both motors + centre servo to 90°           ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  BUZZER — PIN 22                                                 ║
 * ║    WiFi / MQTT connected    → 3 short beeps (100ms on/off)       ║
 * ║    WiFi / MQTT disconnected → 3 long  beeps (400ms on, 200ms off)║
 * ║    Idle                     → silent                             ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  RECONNECT STRATEGY                                              ║
 * ║    WiFi boot timeout : 20 s → hard restart via ESP.restart()     ║
 * ║    WiFi lost in loop : reconnectWiFi() — retries until back,     ║
 * ║                        restarts after 20 s of no signal          ║
 * ║    MQTT lost in loop : reconnectMQTT() — retries until back,     ║
 * ║                        re-checks WiFi first on every attempt     ║
 * ║    Both run BEFORE every client.loop() call — ROV never runs     ║
 * ║    with a dead connection silently.                              ║
 * ╠══════════════════════════════════════════════════════════════════╣
 * ║  BUGS FIXED                                                      ║
 * ║  v2.1 — m2_pwm init=0 → pump ran at full speed on boot.         ║
 * ║          Fix: m2_pwm=255, ledcWrite(chM2,255) in setup().        ║
 * ║  v2.4 — smoothServo() blocked loop() up to 960ms → broker        ║
 * ║          dropped keepalive → mid-op MQTT disconnects.            ║
 * ║          Fix: client.loop() inside sweep + keepalive=60s         ║
 * ║          + setBufferSize(512) + setSocketTimeout(30).            ║
 * ╚══════════════════════════════════════════════════════════════════╝
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP32Servo.h>

// ═══════════════════════════════════════════════════════════
//  PIN MAP
// ═══════════════════════════════════════════════════════════
#define ONE_WIRE_BUS   4
#define TURBIDITY_PIN  34
#define BUZZER_PIN     22   // Active buzzer — HIGH = beep

// Thruster M1 — BTS7960 (normal: 0=stop, 255=full)
#define M1_RPWM  33
#define M1_LPWM  32
#define M1_REN   18
#define M1_LEN   19

// Pump M2 — BTS7960 INVERTED (255=stop, 0=full speed)
#define M2_RPWM  26
#define M2_LPWM  25
#define M2_REN   27
#define M2_LEN   14

#define SERVO_PIN  23

// ═══════════════════════════════════════════════════════════
//  LEDC CONFIG
// ═══════════════════════════════════════════════════════════
const int chM1       = 4;
const int chM2       = 5;
const int freq       = 2000;
const int resolution = 8;

// ═══════════════════════════════════════════════════════════
//  TIMEOUTS
// ═══════════════════════════════════════════════════════════
const unsigned long WIFI_TIMEOUT_MS = 20000;  // 20 s → restart if no WiFi
const unsigned long MQTT_RETRY_MS   = 3000;   // 3 s between MQTT attempts

// ═══════════════════════════════════════════════════════════
//  NETWORK
// ═══════════════════════════════════════════════════════════
const char* ssid        = "Airtel_laks_7975";
const char* password    = "Air@20814";
const char* mqtt_server = "broker.emqx.io";
const int   mqtt_port   = 8883;

const char* CMD_TOPICS[] = { "chennai2026", "96260706", "900398" };
const int   NUM_TOPICS   = 3;
const char* STATUS_TOPIC = "chennai2026/status";

// ═══════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════
Servo             rudder;
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
WiFiClientSecure  espClient;
PubSubClient      client(espClient);

// ═══════════════════════════════════════════════════════════
//  STATE
// ═══════════════════════════════════════════════════════════
int  servoAngle      = 90;
int  m1_pwm          = 0;
int  m2_pwm          = 255;   // 255 = stopped on inverted driver ← v2.1 FIX
bool mqttWasUp       = false;
bool wifiWasUp       = false;

unsigned long lastSensorTime = 0;

// ═══════════════════════════════════════════════════════════
//  BUZZER
// ═══════════════════════════════════════════════════════════
void beep(int times, int onMs, int offMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(offMs);
  }
}
void beepConnected()    { beep(3, 100, 100); }  // 3 short — connected
void beepDisconnected() { beep(3, 400, 200); }  // 3 long  — lost connection

// ═══════════════════════════════════════════════════════════
//  SMOOTH SERVO
// ═══════════════════════════════════════════════════════════
void smoothServo(int target) {
  target = constrain(target, 30, 150);
  if (target == servoAngle) return;
  int step = (target > servoAngle) ? 1 : -1;
  while (servoAngle != target) {
    servoAngle += step;
    rudder.write(servoAngle);
    client.loop();   // keep MQTT alive during sweep — prevents keepalive timeout
    delay(8);
  }
  Serial.printf("  Servo → %d°\n", servoAngle);
}

// ═══════════════════════════════════════════════════════════
//  STOP ALL
// ═══════════════════════════════════════════════════════════
void stopAll() {
  m1_pwm = 0;   ledcWrite(chM1, 0);
  m2_pwm = 255; ledcWrite(chM2, 255);
  smoothServo(90);
  Serial.println("  [STOP] Both motors halted · servo centred");
}

// ═══════════════════════════════════════════════════════════
//  WIFI RECONNECT
//  Blocks until WiFi is back or 20 s passes → hard restart.
//  Called whenever WiFi.status() != WL_CONNECTED.
// ═══════════════════════════════════════════════════════════
void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  // Alert once on first drop
  if (wifiWasUp) {
    Serial.println("[WiFi] *** DISCONNECTED *** — beeping alert");
    beepDisconnected();   // 3 long beeps
    wifiWasUp = false;
    // Also flag MQTT as down since WiFi is gone
    mqttWasUp = false;
  }

  Serial.printf("[WiFi] Reconnecting to %s ", ssid);
  WiFi.disconnect();
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    // 20 second hard timeout → restart
    if (millis() - start >= WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] 20 s timeout — restarting ESP32");
      beepDisconnected();   // final 3 long beeps before restart
      delay(200);
      ESP.restart();
    }
  }

  Serial.printf("\n[WiFi] Reconnected  IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  beepConnected();    // 3 short beeps — WiFi back up
  wifiWasUp = true;
}

// ═══════════════════════════════════════════════════════════
//  MQTT RECONNECT
//  Always checks WiFi first. Retries until MQTT is up.
//  Beeps once on first disconnect, beeps on reconnect.
// ═══════════════════════════════════════════════════════════
void reconnectMQTT() {
  if (client.connected()) return;

  // Alert once on first MQTT drop (only if WiFi is still up —
  // WiFi loss has its own beep in reconnectWiFi)
  if (mqttWasUp && WiFi.status() == WL_CONNECTED) {
    Serial.println("[MQTT] *** DISCONNECTED *** — beeping alert");
    beepDisconnected();   // 3 long beeps
  }
  mqttWasUp = false;

  while (!client.connected()) {

    // WiFi must be up before attempting MQTT
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[MQTT] WiFi down — recovering WiFi first");
      reconnectWiFi();
    }

    Serial.print("[MQTT] Connecting...");
    String clientId = "AquaBot_" + WiFi.macAddress().substring(9);
    clientId.replace(":", "");

    if (client.connect(clientId.c_str())) {
      Serial.printf(" OK (%s)\n", clientId.c_str());

      // Re-subscribe to all command topics
      for (int i = 0; i < NUM_TOPICS; i++) {
        client.subscribe(CMD_TOPICS[i]);
        Serial.printf("  Subscribed → %s\n", CMD_TOPICS[i]);
      }

      beepConnected();    // 3 short beeps — MQTT back up
      mqttWasUp = true;

    } else {
      Serial.printf(" Failed rc=%d — retry in %lu s\n",
                    client.state(), MQTT_RETRY_MS / 1000);
      delay(MQTT_RETRY_MS);
    }
  }
}

// ═══════════════════════════════════════════════════════════
//  MQTT CALLBACK
// ═══════════════════════════════════════════════════════════
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.print("[CMD] "); Serial.print(topic);
  Serial.print(" → "); Serial.println(msg);

  if (msg.startsWith("M1:")) {
    m1_pwm = constrain(msg.substring(3).toInt(), 0, 255);
    ledcWrite(chM1, m1_pwm);
    Serial.printf("  M1 PWM = %d\n", m1_pwm);

  } else if (msg.startsWith("M2:")) {
    m2_pwm = constrain(msg.substring(3).toInt(), 0, 255);
    ledcWrite(chM2, m2_pwm);
    Serial.printf("  M2 raw = %d  (effective = %d)\n", m2_pwm, 255 - m2_pwm);

  } else if (msg.startsWith("SA:")) {
    smoothServo(constrain(msg.substring(3).toInt(), 30, 150));

  } else if (msg == "STOP_M1") {
    m1_pwm = 0;
    ledcWrite(chM1, 0);
    Serial.println("  THRUSTER STOPPED");

  } else if (msg == "STOP_M2") {
    m2_pwm = 255;
    ledcWrite(chM2, 255);
    Serial.println("  PUMP STOPPED");

  } else if (msg == "EMERGENCY") {
    Serial.println("  *** EMERGENCY STOP ***");
    stopAll();

  } else {
    Serial.printf("  [WARN] Unknown: \"%s\"\n", msg.c_str());
  }
}

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("╔══════════════════════════════════╗");
  Serial.println("║   CRABBER ROV  v2.3  Booting     ║");
  Serial.println("╚══════════════════════════════════╝");

  // ── Buzzer ──────────────────────────────────────────────
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("[BUZZER] Pin 22 ready");

  // ── Motor driver enable pins ─────────────────────────────
  pinMode(M1_REN, OUTPUT); pinMode(M1_LEN, OUTPUT);
  pinMode(M2_REN, OUTPUT); pinMode(M2_LEN, OUTPUT);
  digitalWrite(M1_REN, HIGH); digitalWrite(M1_LEN, HIGH);
  digitalWrite(M2_REN, HIGH); digitalWrite(M2_LEN, HIGH);

  // ── Direction pins held LOW (forward only) ────────────────
  pinMode(M1_LPWM, OUTPUT); digitalWrite(M1_LPWM, LOW);
  pinMode(M2_LPWM, OUTPUT); digitalWrite(M2_LPWM, LOW);

  // ── LEDC — Thruster M1 ───────────────────────────────────
  ledcSetup(chM1, freq, resolution);
  ledcAttachPin(M1_RPWM, chM1);
  ledcWrite(chM1, 0);
  Serial.println("[M1] Thruster stopped (duty=0)");

  // ── LEDC — Pump M2 ───────────────────────────────────────
  ledcSetup(chM2, freq, resolution);
  ledcAttachPin(M2_RPWM, chM2);
  ledcWrite(chM2, 255);           // 255 = stopped (inverted) ← v2.1 FIX
  Serial.println("[M2] Pump stopped (duty=255, inverted driver)");

  // ── Servo ─────────────────────────────────────────────────
  // GPIO 23 floats during the several seconds WiFi takes to
  // connect. The servo reads that floating signal as random
  // pulses and twitches or sweeps by itself.
  //
  // Fix — three steps:
  //   1. Drive pin LOW explicitly before attach() — kills float
  //   2. Use attach(pin, minUs, maxUs) with explicit pulse range
  //      so the servo does not hunt on ambiguous centre values
  //   3. Servo init happens HERE — before WiFi.begin() — so the
  //      servo is locked at 90° the entire time WiFi connects
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);       // kill float immediately
  delay(20);                           // let pin settle
  rudder.attach(SERVO_PIN, 500, 2500); // 500–2500 µs explicit range
  rudder.write(90);
  servoAngle = 90;
  Serial.println("[SERVO] Centred at 90° — locked before WiFi init");

  // ── Sensors ───────────────────────────────────────────────
  sensors.begin();
  pinMode(TURBIDITY_PIN, INPUT);
  Serial.println("[SENSORS] DS18B20 + Turbidity ready");

  // ── WiFi — 20 s hard timeout then restart ─────────────────
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("[WiFi] Connecting to %s ", ssid);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiStart >= WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] 20 s timeout — restarting ESP32");
      beepDisconnected();   // 3 long beeps before restart
      delay(200);
      ESP.restart();
    }
  }
  Serial.printf("\n[WiFi] Connected  IP: %s  RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  beepConnected();            // 3 short beeps — WiFi up ✓
  wifiWasUp = true;

  // ── MQTT ──────────────────────────────────────────────────
  espClient.setInsecure();          // skip cert verification (public broker)
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(512);        // default 256 is too small for telemetry JSON
  client.setKeepAlive(60);          // 60 s keepalive — default 15 s drops mid-op
  client.setSocketTimeout(30);      // 30 s socket timeout
  client.setCallback(callback);
  // First connect handled by reconnectMQTT() at top of loop()

  Serial.println("[ROV] Boot complete — entering main loop");
  Serial.println("─────────────────────────────────────────");
}

// ═══════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════
void loop() {

  // ── Connection guard — runs every iteration ───────────────
  // WiFi must be healthy before MQTT is even attempted.
  // Both functions return immediately if already connected.
  if (WiFi.status() != WL_CONNECTED) reconnectWiFi();
  if (!client.connected())            reconnectMQTT();

  client.loop();

  // ── Telemetry every 2 s ───────────────────────────────────
  if (millis() - lastSensorTime >= 2000) {
    lastSensorTime = millis();

    sensors.requestTemperatures();
    float temp    = sensors.getTempCByIndex(0);
    int   turbRaw = analogRead(TURBIDITY_PIN);
    float voltage = turbRaw * (3.3f / 4095.0f);

    char json[256];
    snprintf(json, sizeof(json),
      "{\"temp\":%.1f,\"turb\":%d,\"volt\":%.2f,"
       "\"rssi\":%d,\"ip\":\"%s\","
       "\"m1\":%d,\"m2\":%d,\"servo\":%d,\"uptime\":%lu}",
      temp, turbRaw, voltage,
      (int)WiFi.RSSI(), WiFi.localIP().toString().c_str(),
      m1_pwm, m2_pwm, servoAngle,
      millis() / 1000UL
    );

    client.publish(STATUS_TOPIC, json);
    Serial.println(json);
  }
}
