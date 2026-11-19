#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <vector>
#include <time.h>

#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRutils.h>

// =====================================================
// CONFIGURAZIONE UTENTE
// =====================================================

const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// EMQX Cloud
const char* MQTT_HOST     = "MQTT_URL";
const uint16_t MQTT_PORT  = 8883;
const char* MQTT_USERNAME = "USERNAME";
const char* MQTT_PASSWORD = "PASSWORD";

String deviceId;
String topicCommand;   // devices/<id>/cmd
String topicStatus;    // devices/<id>/status

// =====================================================
// PIN ESP32-C3
// =====================================================

constexpr uint8_t PIN_IR_RX = 10;   // uscita scheda fototransistor
constexpr uint8_t PIN_IR_TX = 3;    // -> resistore base del transistor
constexpr uint8_t PIN_BOOT  = 9;    // pulsante BOOT (strapping, ok dopo l'avvio)
constexpr uint8_t PIN_LED   = 8;    // LED blu di bordo (verifica polarita'!)
constexpr bool    LED_ACTIVE_LOW = true;

constexpr uint8_t OLED_SDA = 5;
constexpr uint8_t OLED_SCL = 6;

constexpr uint16_t IR_CARRIER_KHZ = 38;

// =====================================================
// CERTIFICATO CA EMQX (DigiCert Global Root G2)
// =====================================================

static const char ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG
9w0BAQsFADBhMQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNl
cnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSAwHgYD
VQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBHMjAeFw0xMzA4MDEx
MjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVTMRUw
EwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdp
Y2VydC5jb20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290
IEcyMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfN
NNx7a8myaJCtSnX/RrohCgiN9RlUyfuI2/Ou8jqJkTx65qsGGmvP
rC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx1x7e/dfg
y5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY
92eQq2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd13
4/tJS7SsVQepj5WztCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS
0xZKBgyMUNGPHgm+F6HmIcr9g+UQvIOlCsRnKPZzFBQ9RnbDhxSJ
ITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAPBgNVHRMBAf8E
BTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRv
Dkhj6zHd6mcY1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVn
NeyIv/iPsGEMNKSuIEyExtv4NeF22d+mQrvHRAiGfzZ0JFrabA0U
WTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NGFdtom/DzMNU+MeKN
hJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ918rGO
maFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5
S9kJRaTepLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld192
7jyNxF1WW6LZZm6zNTflMrY=
-----END CERTIFICATE-----
)EOF";

// =====================================================
// OGGETTI
// =====================================================

U8G2_SSD1306_72X40_ER_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
Preferences preferences;

// buffer 1024, timeout 50 ms (necessario per i messaggi lunghi dei condizionatori),
// save_buffer = true
IRrecv irrecv(PIN_IR_RX, 1024, 50, true);
IRsend irsend(PIN_IR_TX);
decode_results irResults;

// =====================================================
// PAYLOAD MEMORIZZATI (timing grezzi in microsecondi)
// Salvare i timing raw funziona con QUALSIASI protocollo
// (LG, Daikin, Mitsubishi...) senza dover decodificare.
// =====================================================

std::vector<uint16_t> onRaw;
std::vector<uint16_t> offRaw;

// =====================================================
// MODALITA'
// =====================================================

enum DeviceMode { MODE_TRANSMIT, MODE_LEARN_ON, MODE_LEARN_OFF };
DeviceMode currentMode = MODE_TRANSMIT;

// BOOT
bool lastButtonState = HIGH;
uint32_t lastButtonChange = 0;
constexpr uint32_t DEBOUNCE_MS = 50;

// LED
uint32_t lastBlink = 0;
bool blinkState = false;

// IR learning
uint32_t ignoreIrUntil = 0;
constexpr uint16_t MIN_RAW_LEN = 16;   // scarta rumore

// Connessioni
uint32_t lastWifiAttempt = 0;
uint32_t lastMqttAttempt = 0;
bool ntpStarted = false;

// =====================================================
// LED
// =====================================================

void setLed(bool on) {
  digitalWrite(PIN_LED, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

void updateLed() {
  if (currentMode == MODE_TRANSMIT) {
    setLed(true);                       // fisso
  } else if (millis() - lastBlink >= 250) {
    lastBlink = millis();               // lampeggiante
    blinkState = !blinkState;
    setLed(blinkState);
  }
}

// =====================================================
// DISPLAY (testo centrato)
// =====================================================

void drawCentered(const uint8_t* font, int baseline, const char* text) {
  display.setFont(font);
  int w = display.getStrWidth(text);
  display.drawStr((display.getDisplayWidth() - w) / 2, baseline, text);
}

void showDisplay(const char* small, const char* big) {
  display.clearBuffer();
  drawCentered(u8g2_font_6x10_tf, 12, small);
  drawCentered(u8g2_font_10x20_tf, 34, big);
  display.sendBuffer();
}

String lastDisplayKey;

void updateDisplay() {
  const char* small = "";
  const char* big = "";

  switch (currentMode) {
    case MODE_LEARN_ON:  small = "Comando:"; big = "ON";  break;
    case MODE_LEARN_OFF: small = "Comando:"; big = "OFF"; break;
    case MODE_TRANSMIT:
      small = (WiFi.status() == WL_CONNECTED) ? "WiFi OK" : "No WiFi";
      big   = mqtt.connected() ? "MQTT" : "...";
      break;
  }

  String key = String(small) + "|" + big;
  if (key == lastDisplayKey) return;    // ridisegna solo se cambia
  lastDisplayKey = key;
  showDisplay(small, big);
}

// =====================================================
// MEMORIA NVS (chiavi "on" e "off")
// =====================================================

void loadSignal(const char* key, std::vector<uint16_t>& out) {
  size_t bytes = preferences.getBytesLength(key);
  out.clear();
  if (bytes == 0 || bytes % 2 != 0) return;
  out.resize(bytes / 2);
  preferences.getBytes(key, out.data(), bytes);
}

void saveSignal(const char* key, const std::vector<uint16_t>& data) {
  preferences.putBytes(key, data.data(), data.size() * sizeof(uint16_t));
}

// =====================================================
// APPRENDIMENTO IR
// =====================================================

bool sameSignal(const std::vector<uint16_t>& a, const std::vector<uint16_t>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    int diff = abs((int)a[i] - (int)b[i]);
    if (diff > (int)(a[i] / 4) + 100) return false;   // tolleranza ~25%
  }
  return true;
}

void startLearning() {
  currentMode = MODE_LEARN_ON;
  irrecv.enableIRIn();
  ignoreIrUntil = millis() + 500;
  Serial.println("Learning ON: premi il tasto ACCENDI del telecomando");
}

void stopLearning() {
  irrecv.disableIRIn();
  currentMode = MODE_TRANSMIT;
  Serial.println("Modalita' trasmissione");
}

void processIRLearning() {
  if (!irrecv.decode(&irResults)) return;

  if (millis() < ignoreIrUntil) {       // ignora ripetizioni dello stesso tasto
    irrecv.resume();
    return;
  }

  Serial.print(resultToHumanReadableBasic(&irResults));

  uint16_t len = getCorrectedRawLength(&irResults);

  if (irResults.overflow || len < MIN_RAW_LEN) {
    Serial.println("Segnale scartato (troppo corto o buffer overflow)");
    irrecv.resume();
    return;
  }

  uint16_t* raw = resultToRawArray(&irResults);
  std::vector<uint16_t> captured(raw, raw + len);
  delete[] raw;

  if (currentMode == MODE_LEARN_ON) {
    onRaw = captured;
    saveSignal("on", onRaw);
    Serial.printf("ON salvato (%u timing)\n", (unsigned)onRaw.size());

    currentMode = MODE_LEARN_OFF;
    ignoreIrUntil = millis() + 1500;
    irrecv.resume();

  } else if (currentMode == MODE_LEARN_OFF) {

    if (sameSignal(captured, onRaw)) {
      Serial.println("Uguale a ON: ignorato, premi il tasto SPEGNI");
      ignoreIrUntil = millis() + 500;
      irrecv.resume();
      return;
    }

    offRaw = captured;
    saveSignal("off", offRaw);
    Serial.printf("OFF salvato (%u timing)\n", (unsigned)offRaw.size());

    showDisplay("Salvato", "OK");
    delay(1200);
    stopLearning();
  }
}

// =====================================================
// TRASMISSIONE IR
// =====================================================

// 0 = ok, 1 = comando sconosciuto, 2 = payload non appreso, 3 = learning attivo
int transmitIR(const char* command) {
  const std::vector<uint16_t>* sig = nullptr;

  if (strcasecmp(command, "on") == 0)       sig = &onRaw;
  else if (strcasecmp(command, "off") == 0) sig = &offRaw;
  else return 1;

  if (currentMode != MODE_TRANSMIT) return 3;
  if (sig->empty()) return 2;

  irsend.sendRaw(sig->data(), sig->size(), IR_CARRIER_KHZ);
  Serial.printf("IR inviato: %s (%u timing)\n", command, (unsigned)sig->size());
  return 0;
}

// =====================================================
// MQTT
// =====================================================

void publishResponse(bool success, const char* error = nullptr) {
  JsonDocument response;
  if (success) {
    response["status"] = "ok";
  } else {
    response["status"] = "failed";
    response["error"] = error ? error : "Command unknown";
  }

  char buffer[128];
  serializeJson(response, buffer, sizeof(buffer));
  mqtt.publish(topicStatus.c_str(), buffer, false);
  Serial.printf("MQTT response: %s\n", buffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("MQTT message on %s\n", topic);

  JsonDocument doc;
  if (deserializeJson(doc, payload, length)) {
    publishResponse(false, "Invalid JSON");
    return;
  }

  const char* command = doc["command"] | "";
  Serial.printf("Command: %s\n", command);

  switch (transmitIR(command)) {
    case 0: publishResponse(true); break;
    case 2: publishResponse(false, "Payload not configured"); break;
    case 3: publishResponse(false, "Learning mode active"); break;
    default: publishResponse(false, "Command unknown"); break;
  }
}

// =====================================================
// WIFI / NTP / MQTT (non bloccanti: BOOT e learning
// funzionano anche senza rete)
// =====================================================

void maintainConnections() {
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    ntpStarted = false;
    if (now - lastWifiAttempt > 10000 || lastWifiAttempt == 0) {
      lastWifiAttempt = now;
      Serial.println("WiFi: connessione...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    return;
  }

  // L'orologio serve alla verifica del certificato TLS
  if (!ntpStarted) {
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    ntpStarted = true;
    Serial.print("WiFi connesso: ");
    Serial.println(WiFi.localIP());
  }
  if (time(nullptr) < 1700000000) return;   // NTP non ancora sincronizzato

  if (mqtt.connected()) return;

  // Non aprire TLS mentre si apprende (handshake bloccante)
  if (currentMode != MODE_TRANSMIT) return;

  if (now - lastMqttAttempt < 5000 && lastMqttAttempt != 0) return;
  lastMqttAttempt = now;

  Serial.print("Connessione EMQX...");
  if (mqtt.connect(deviceId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("ok");
    mqtt.subscribe(topicCommand.c_str());
    Serial.println("Subscribed: " + topicCommand);
  } else {
    Serial.printf("fallita, rc=%d\n", mqtt.state());
  }
}

// =====================================================
// BOOT BUTTON: a rotazione  TRASMISSIONE -> LEARN -> TRASMISSIONE
// In learn (ON o OFF) una pressione annulla/esce.
// =====================================================

void checkBootButton() {
  bool state = digitalRead(PIN_BOOT);

  if (state == LOW && lastButtonState == HIGH &&
      millis() - lastButtonChange > DEBOUNCE_MS) {
    lastButtonChange = millis();

    if (currentMode == MODE_TRANSMIT) startLearning();
    else                              stopLearning();
  }

  lastButtonState = state;
}

// =====================================================
// SETUP / LOOP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nAC IR MQTT Controller");

  deviceId = "esp32-c3-" + String((uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF), HEX);
  topicCommand = "devices/" + deviceId + "/cmd";
  topicStatus  = "devices/" + deviceId + "/status";

  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  setLed(true);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin();
  display.setPowerSave(0);
  showDisplay("Avvio", "...");

  irsend.begin();
  irrecv.disableIRIn();

  preferences.begin("ir", false);
  loadSignal("on", onRaw);
  loadSignal("off", offRaw);
  Serial.printf("ON: %u timing, OFF: %u timing\n",
                (unsigned)onRaw.size(), (unsigned)offRaw.size());

  secureClient.setCACert(ROOT_CA);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(5);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  Serial.println("Topic cmd: " + topicCommand);
}

void loop() {
  checkBootButton();
  maintainConnections();
  mqtt.loop();

  if (currentMode != MODE_TRANSMIT) processIRLearning();

  updateLed();
  updateDisplay();
  delay(2);
}
