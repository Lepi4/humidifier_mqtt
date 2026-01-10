#include <Arduino.h>

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#include <PubSubClient.h>

#include <ArduinoOTA.h>
#include <ESPmDNS.h>

#include <Update.h>

#ifndef HUM_DEVICE_NAME
#define HUM_DEVICE_NAME "humidifier-esp32"
#endif

#ifndef HUM_DEFAULT_RELAY_PIN
#define HUM_DEFAULT_RELAY_PIN 0
#endif

#ifndef HUM_DEFAULT_RELAY_INVERTED
#define HUM_DEFAULT_RELAY_INVERTED 1
#endif

#ifndef HUM_DEFAULT_AP_SSID
#define HUM_DEFAULT_AP_SSID "Humidifier-Setup"
#endif

#ifndef HUM_DEFAULT_AP_PASS
#define HUM_DEFAULT_AP_PASS "12345678"
#endif

static constexpr uint16_t HTTP_PORT = 80;
static constexpr uint16_t DNS_PORT = 53;

static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static constexpr uint32_t MQTT_RECONNECT_MIN_MS = 1000;
static constexpr uint32_t MQTT_RECONNECT_MAX_MS = 30000;

static constexpr uint32_t DEFAULT_HUMIDITY_MIN_INTERVAL_MS = 5U * 60U * 1000U; // 5 minutes
static constexpr float DEFAULT_HYSTERESIS = 2.0f;
static constexpr float DEFAULT_SETPOINT = 45.0f;

struct AppConfig {
  char wifiSsid[33] = {0};
  char wifiPass[65] = {0};

  char mqttHost[65] = {0};
  uint16_t mqttPort = 1883;
  char mqttUser[65] = {0};
  char mqttPass[65] = {0};

  char baseTopic[129] = {0};
  char topicHumidityIn[129] = {0};
  char topicSetpointIn[129] = {0};
  char topicEnableIn[129] = {0};

  int relayPin = HUM_DEFAULT_RELAY_PIN;
  bool relayInverted = HUM_DEFAULT_RELAY_INVERTED != 0;

  uint32_t humidityMinIntervalMs = DEFAULT_HUMIDITY_MIN_INTERVAL_MS;
  float hysteresis = DEFAULT_HYSTERESIS;
};

static Preferences prefs;
static AppConfig config;

static WebServer web(HTTP_PORT);
static DNSServer dns;

static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

static bool captivePortalActive = false;
static bool otaActive = false;

#ifndef HUM_OTA_PASSWORD
#define HUM_OTA_PASSWORD ""
#endif

static bool systemEnabled = true;
static bool relayOn = false;

static float targetHumidity = DEFAULT_SETPOINT;
static float currentHumidity = NAN;

static uint32_t lastHumidityAcceptMs = 0;
static uint32_t lastHumiditySeenMs = 0;

static uint8_t humiditySamplesSinceMqttConnect = 0;

static uint32_t lastMqttAttemptMs = 0;
static uint32_t mqttBackoffMs = MQTT_RECONNECT_MIN_MS;
static uint32_t lastStatePublishMs = 0;

static String deviceId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "%s-%04X", HUM_DEVICE_NAME, (unsigned)(mac & 0xFFFF));
  return String(buf);
}

static void setupOta() {
  if (otaActive) return;

  const String host = deviceId();
  ArduinoOTA.setHostname(host.c_str());

  static constexpr const char *OTA_PASSWORD = HUM_OTA_PASSWORD;
  if (OTA_PASSWORD[0] != '\0') ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) return;
    const unsigned int pct = (progress * 100U) / total;
    static unsigned int lastPct = 101;
    if (pct != lastPct) {
      lastPct = pct;
      Serial.printf("[OTA] %u%%\n", pct);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error: %u\n", (unsigned)error);
  });

  ArduinoOTA.begin();
  otaActive = true;
  Serial.printf("[OTA] Ready. Hostname: %s\n", host.c_str());
}

static bool parseBool(const String &value, bool defaultValue) {
  String v = value;
  v.trim();
  v.toLowerCase();
  if (v == "1" || v == "on" || v == "true" || v == "yes" || v == "enable" || v == "enabled") return true;
  if (v == "0" || v == "off" || v == "false" || v == "no" || v == "disable" || v == "disabled") return false;
  return defaultValue;
}

static bool parseFloat(const String &value, float &out) {
  String v = value;
  v.trim();
  v.replace(',', '.');
  char *endptr = nullptr;
  float f = strtof(v.c_str(), &endptr);
  if (endptr == v.c_str()) return false;
  out = f;
  return true;
}

static void relayWrite(bool on) {
  relayOn = on;
  bool level = on;
  if (config.relayInverted) level = !level;
  digitalWrite(config.relayPin, level ? HIGH : LOW);
}

static String topicOf(const char *suffix) {
  String base = String(config.baseTopic);
  if (base.length() == 0) base = "humidifier/" + deviceId();
  if (!base.endsWith("/")) base += "/";
  return base + suffix;
}

static void saveConfig() {
  prefs.begin("hum", false);
  prefs.putString("wifiSsid", config.wifiSsid);
  prefs.putString("wifiPass", config.wifiPass);
  prefs.putString("mqttHost", config.mqttHost);
  prefs.putUShort("mqttPort", config.mqttPort);
  prefs.putString("mqttUser", config.mqttUser);
  prefs.putString("mqttPass", config.mqttPass);
  prefs.putString("baseTopic", config.baseTopic);
  prefs.putString("tHumIn", config.topicHumidityIn);
  prefs.putString("tSetIn", config.topicSetpointIn);
  prefs.putString("tEnIn", config.topicEnableIn);
  prefs.putInt("relayPin", config.relayPin);
  prefs.putBool("relayInv", config.relayInverted);
  prefs.putULong("humInt", config.humidityMinIntervalMs);
  prefs.putFloat("hyst", config.hysteresis);
  prefs.putFloat("target", targetHumidity);
  prefs.putBool("sysEn", systemEnabled);
  prefs.end();
}

static void loadConfig() {
  prefs.begin("hum", true);

  String wifiSsid = prefs.getString("wifiSsid", "");
  String wifiPass = prefs.getString("wifiPass", "");
  String mqttHost = prefs.getString("mqttHost", "");
  uint16_t mqttPort = prefs.getUShort("mqttPort", 1883);
  String mqttUser = prefs.getString("mqttUser", "");
  String mqttPass = prefs.getString("mqttPass", "");

  String baseTopic = prefs.getString("baseTopic", ("humidifier/" + deviceId()).c_str());
  String baseForDefaults = baseTopic;
  if (baseForDefaults.length() == 0) baseForDefaults = "humidifier/" + deviceId();
  if (!baseForDefaults.endsWith("/")) baseForDefaults += "/";

  String tHumIn = prefs.getString("tHumIn", "");
  String tSetIn = prefs.getString("tSetIn", (baseForDefaults + "cmd/setpoint").c_str());
  String tEnIn = prefs.getString("tEnIn", (baseForDefaults + "cmd/enabled").c_str());

  int relayPin = prefs.getInt("relayPin", HUM_DEFAULT_RELAY_PIN);
  bool relayInv = prefs.getBool("relayInv", HUM_DEFAULT_RELAY_INVERTED != 0);
  uint32_t humInt = prefs.getULong("humInt", DEFAULT_HUMIDITY_MIN_INTERVAL_MS);
  float hyst = prefs.getFloat("hyst", DEFAULT_HYSTERESIS);
  float storedTarget = prefs.getFloat("target", DEFAULT_SETPOINT);
  bool storedEnabled = prefs.getBool("sysEn", true);

  prefs.end();

  strncpy(config.wifiSsid, wifiSsid.c_str(), sizeof(config.wifiSsid) - 1);
  strncpy(config.wifiPass, wifiPass.c_str(), sizeof(config.wifiPass) - 1);

  strncpy(config.mqttHost, mqttHost.c_str(), sizeof(config.mqttHost) - 1);
  config.mqttPort = mqttPort;
  strncpy(config.mqttUser, mqttUser.c_str(), sizeof(config.mqttUser) - 1);
  strncpy(config.mqttPass, mqttPass.c_str(), sizeof(config.mqttPass) - 1);

  strncpy(config.baseTopic, baseTopic.c_str(), sizeof(config.baseTopic) - 1);
  strncpy(config.topicHumidityIn, tHumIn.c_str(), sizeof(config.topicHumidityIn) - 1);
  strncpy(config.topicSetpointIn, tSetIn.c_str(), sizeof(config.topicSetpointIn) - 1);
  strncpy(config.topicEnableIn, tEnIn.c_str(), sizeof(config.topicEnableIn) - 1);

  config.relayPin = relayPin;
  config.relayInverted = relayInv;
  config.humidityMinIntervalMs = humInt;
  config.hysteresis = hyst;

  if (!isnan(storedTarget)) targetHumidity = storedTarget;
  systemEnabled = storedEnabled;
}

static void saveRuntimeState() {
  prefs.begin("hum", false);
  prefs.putFloat("target", targetHumidity);
  prefs.putBool("sysEn", systemEnabled);
  prefs.end();
}

static void mqttPublishState(bool force);

static String htmlEscape(const String &s) {
  String o;
  o.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += c; break;
    }
  }
  return o;
}

static String configPage(const String &notice = "") {
  String apMode = (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) ? "AP" : "STA";

  String page;
  page.reserve(4000);
  page += "<!doctype html><html><head><meta charset='utf-8'>";
  page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<title>Humidifier Setup</title></head><body>";
  page += "<h2>Humidifier Setup</h2>";
  page += "<div>Device: <b>" + htmlEscape(deviceId()) + "</b></div>";
  page += "<div>WiFi mode: <b>" + apMode + "</b></div>";
  page += "<p><a href='/update'>Firmware update</a></p>";

  if (notice.length() > 0) {
    page += "<p><b>" + htmlEscape(notice) + "</b></p>";
  }

  page += "<form method='POST' action='/save'>";

  page += "<h3>WiFi</h3>";
  page += "SSID:<br><input name='wifi_ssid' maxlength='32' value='" + htmlEscape(String(config.wifiSsid)) + "'><br>";
  page += "Password:<br><input name='wifi_pass' type='password' maxlength='64' value='" + htmlEscape(String(config.wifiPass)) + "'><br>";

  page += "<h3>MQTT</h3>";
  page += "Host:<br><input name='mqtt_host' maxlength='64' value='" + htmlEscape(String(config.mqttHost)) + "'><br>";
  page += "Port:<br><input name='mqtt_port' type='number' min='1' max='65535' value='" + String(config.mqttPort) + "'><br>";
  page += "User:<br><input name='mqtt_user' maxlength='64' value='" + htmlEscape(String(config.mqttUser)) + "'><br>";
  page += "Password:<br><input name='mqtt_pass' type='password' maxlength='64' value='" + htmlEscape(String(config.mqttPass)) + "'><br>";

  page += "<h3>Topics</h3>";
  page += "Base topic:<br><input name='base_topic' maxlength='128' value='" + htmlEscape(String(config.baseTopic)) + "'><br>";
  page += "External humidity topic (subscribe):<br><input name='t_hum_in' maxlength='128' value='" + htmlEscape(String(config.topicHumidityIn)) + "'><br>";
  page += "Setpoint topic (subscribe):<br><input name='t_set_in' maxlength='128' value='" + htmlEscape(String(config.topicSetpointIn)) + "'><br>";
  page += "Enable topic (subscribe):<br><input name='t_en_in' maxlength='128' value='" + htmlEscape(String(config.topicEnableIn)) + "'><br>";

  page += "<h3>Control</h3>";
  page += "Relay pin (GPIO):<br><input name='relay_pin' type='number' min='0' max='39' value='" + String(config.relayPin) + "'><br>";
  page += "Relay inverted (1=ON->LOW):<br><input name='relay_inv' maxlength='5' value='" + String(config.relayInverted ? "1" : "0") + "'><br>";
  page += "Hysteresis (%RH):<br><input name='hyst' type='number' step='0.1' value='" + String(config.hysteresis, 1) + "'><br>";
  page += "Humidity min interval (sec):<br><input name='hum_int_sec' type='number' min='0' value='" + String(config.humidityMinIntervalMs / 1000U) + "'><br>";

  page += "<p><button type='submit'>Save & Reboot</button></p>";
  page += "</form>";

  page += "<hr>";
  page += "<h3>Quick control</h3>";
  page += "<form method='POST' action='/control'>";
  page += "Enable automation: <select name='enabled'>";
  page += String("<option value='1'") + (systemEnabled ? " selected" : "") + ">ON</option>";
  page += String("<option value='0'") + (!systemEnabled ? " selected" : "") + ">OFF</option>";
  page += "</select><br>";
  page += "Target humidity (%RH):<br><input name='setpoint' type='number' step='0.1' value='" + String(targetHumidity, 1) + "'><br>";
  page += "<p><button type='submit'>Apply</button></p>";
  page += "</form>";

  page += "<hr>";
  page += "<h3>Status</h3>";
  page += "Enabled: <b>" + String(systemEnabled ? "YES" : "NO") + "</b><br>";
  page += "Relay: <b>" + String(relayOn ? "ON" : "OFF") + "</b><br>";
  page += "Target humidity: <b>" + String(targetHumidity, 1) + "</b><br>";
  page += "Current humidity: <b>" + (isnan(currentHumidity) ? String("N/A") : String(currentHumidity, 1)) + "</b><br>";
  if (lastHumiditySeenMs > 0) {
    page += "Last humidity seen: <b>" + String((millis() - lastHumiditySeenMs) / 1000U) + "s ago</b><br>";
  }
  page += "WiFi IP: <b>" + WiFi.localIP().toString() + "</b><br>";
  page += "MQTT: <b>" + String(mqtt.connected() ? "connected" : "disconnected") + "</b><br>";

  page += "</body></html>";
  return page;
}

static void httpSetupHandlers() {
  web.on("/", HTTP_GET, []() {
    web.send(200, "text/html", configPage());
  });

  web.on("/control", HTTP_POST, []() {
    auto arg = [&](const char *name) -> String {
      return web.hasArg(name) ? web.arg(name) : String("");
    };

    bool changed = false;

    String enabledStr = arg("enabled");
    bool newEnabled = parseBool(enabledStr, systemEnabled);
    if (newEnabled != systemEnabled) {
      systemEnabled = newEnabled;
      changed = true;
    }
    if (!systemEnabled && relayOn) relayWrite(false);

    String setpointStr = arg("setpoint");
    float newSetpoint;
    if (parseFloat(setpointStr, newSetpoint) && !isnan(newSetpoint) && newSetpoint != targetHumidity) {
      targetHumidity = newSetpoint;
      changed = true;
    }

    if (changed) saveRuntimeState();
    mqttPublishState(true);

    web.send(200, "text/html", configPage(changed ? "Applied." : "No changes."));
  });

  web.on("/update", HTTP_GET, []() {
    String page;
    page.reserve(1200);
    page += "<!doctype html><html><head><meta charset='utf-8'>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    page += "<title>Firmware Update</title></head><body>";
    page += "<h2>Firmware Update</h2>";
    page += "<p>Upload <b>firmware.bin</b> built by PlatformIO.</p>";
    page += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    page += "<input type='file' name='update' accept='.bin' required><br><br>";
    page += "<button type='submit'>Upload & Flash</button>";
    page += "</form>";
    page += "<p><a href='/'>Back</a></p>";
    page += "</body></html>";
    web.send(200, "text/html", page);
  });

  web.on(
      "/update",
      HTTP_POST,
      []() {
        const bool ok = !Update.hasError();
        web.send(200, "text/plain", ok ? "OK\nRebooting..." : "FAIL\n");
        delay(300);
        if (ok) ESP.restart();
      },
      []() {
        HTTPUpload &upload = web.upload();
        if (upload.status == UPLOAD_FILE_START) {
          Serial.printf("[WEB OTA] Start: %s, size=%u\n", upload.filename.c_str(), (unsigned)upload.totalSize);
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true)) {
            Serial.printf("[WEB OTA] Success: %u bytes\n", (unsigned)upload.totalSize);
          } else {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          Update.end();
          Serial.println("[WEB OTA] Aborted");
        }
      });

  web.on("/save", HTTP_POST, []() {
    auto arg = [&](const char *name) -> String {
      return web.hasArg(name) ? web.arg(name) : String("");
    };

    String wifiSsid = arg("wifi_ssid");
    String wifiPass = arg("wifi_pass");

    String mqttHost = arg("mqtt_host");
    String mqttPort = arg("mqtt_port");
    String mqttUser = arg("mqtt_user");
    String mqttPass = arg("mqtt_pass");

    String baseTopic = arg("base_topic");
    String tHumIn = arg("t_hum_in");
    String tSetIn = arg("t_set_in");
    String tEnIn = arg("t_en_in");

    String relayPin = arg("relay_pin");
    String relayInv = arg("relay_inv");
    String hyst = arg("hyst");
    String humIntSec = arg("hum_int_sec");

    wifiSsid.trim();
    mqttHost.trim();
    baseTopic.trim();
    tHumIn.trim();
    tSetIn.trim();
    tEnIn.trim();

    if (wifiSsid.length() >= sizeof(config.wifiSsid)) wifiSsid = wifiSsid.substring(0, sizeof(config.wifiSsid) - 1);
    if (wifiPass.length() >= sizeof(config.wifiPass)) wifiPass = wifiPass.substring(0, sizeof(config.wifiPass) - 1);
    if (mqttHost.length() >= sizeof(config.mqttHost)) mqttHost = mqttHost.substring(0, sizeof(config.mqttHost) - 1);
    if (mqttUser.length() >= sizeof(config.mqttUser)) mqttUser = mqttUser.substring(0, sizeof(config.mqttUser) - 1);
    if (mqttPass.length() >= sizeof(config.mqttPass)) mqttPass = mqttPass.substring(0, sizeof(config.mqttPass) - 1);
    if (baseTopic.length() >= sizeof(config.baseTopic)) baseTopic = baseTopic.substring(0, sizeof(config.baseTopic) - 1);
    if (tHumIn.length() >= sizeof(config.topicHumidityIn)) tHumIn = tHumIn.substring(0, sizeof(config.topicHumidityIn) - 1);
    if (tSetIn.length() >= sizeof(config.topicSetpointIn)) tSetIn = tSetIn.substring(0, sizeof(config.topicSetpointIn) - 1);
    if (tEnIn.length() >= sizeof(config.topicEnableIn)) tEnIn = tEnIn.substring(0, sizeof(config.topicEnableIn) - 1);

    strncpy(config.wifiSsid, wifiSsid.c_str(), sizeof(config.wifiSsid) - 1);
    strncpy(config.wifiPass, wifiPass.c_str(), sizeof(config.wifiPass) - 1);

    strncpy(config.mqttHost, mqttHost.c_str(), sizeof(config.mqttHost) - 1);
    config.mqttPort = (uint16_t)mqttPort.toInt();
    strncpy(config.mqttUser, mqttUser.c_str(), sizeof(config.mqttUser) - 1);
    strncpy(config.mqttPass, mqttPass.c_str(), sizeof(config.mqttPass) - 1);

    strncpy(config.baseTopic, baseTopic.c_str(), sizeof(config.baseTopic) - 1);
    strncpy(config.topicHumidityIn, tHumIn.c_str(), sizeof(config.topicHumidityIn) - 1);
    strncpy(config.topicSetpointIn, tSetIn.c_str(), sizeof(config.topicSetpointIn) - 1);
    strncpy(config.topicEnableIn, tEnIn.c_str(), sizeof(config.topicEnableIn) - 1);

    config.relayPin = relayPin.toInt();
    config.relayInverted = parseBool(relayInv, config.relayInverted);

    float hystF;
    if (parseFloat(hyst, hystF)) config.hysteresis = hystF;

    uint32_t sec = (uint32_t)humIntSec.toInt();
    config.humidityMinIntervalMs = sec * 1000U;

    saveConfig();

    web.send(200, "text/html", configPage("Saved. Rebooting..."));
    delay(500);
    ESP.restart();
  });

  web.onNotFound([]() {
    // Captive portal: always redirect to /
    web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    web.send(302, "text/plain", "");
  });
}

static void startCaptivePortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(HUM_DEFAULT_AP_SSID, HUM_DEFAULT_AP_PASS);

  IPAddress apIP = WiFi.softAPIP();
  dns.start(DNS_PORT, "*", apIP);
  captivePortalActive = true;

  httpSetupHandlers();
  web.begin();

  Serial.printf("[AP] SSID: %s\n", HUM_DEFAULT_AP_SSID);
  Serial.printf("[AP] IP: %s\n", apIP.toString().c_str());
}

static bool connectWiFiSta() {
  if (strlen(config.wifiSsid) == 0) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(config.wifiSsid, config.wifiPass);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    return false;
  }

  captivePortalActive = false;

  httpSetupHandlers();
  web.begin();

  setupOta();

  Serial.printf("[WiFi] Connected, IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

static void mqttPublishBool(const String &topic, bool value, bool retain = true) {
  mqtt.publish(topic.c_str(), value ? "1" : "0", retain);
}

static void mqttPublishFloat(const String &topic, float value, uint8_t decimals = 1, bool retain = true) {
  char buf[32];
  dtostrf(value, 0, decimals, buf);
  mqtt.publish(topic.c_str(), buf, retain);
}

static void mqttPublishState(bool force) {
  uint32_t now = millis();
  if (!force && (now - lastStatePublishMs) < 60000U) return;
  lastStatePublishMs = now;

  mqttPublishBool(topicOf("state/enabled"), systemEnabled);
  mqtt.publish(topicOf("state/relay").c_str(), relayOn ? "ON" : "OFF", true);
  mqttPublishFloat(topicOf("state/setpoint"), targetHumidity);
  if (!isnan(currentHumidity)) mqttPublishFloat(topicOf("state/humidity"), currentHumidity);
  if (lastHumiditySeenMs > 0) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)(now - lastHumiditySeenMs));
    mqtt.publish(topicOf("state/humidity_age_ms").c_str(), buf, true);
  }
}

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String t(topic);
  String p;
  p.reserve(length);
  for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
  p.trim();

  if (strlen(config.topicEnableIn) > 0 && t == String(config.topicEnableIn)) {
    bool newEnabled = parseBool(p, systemEnabled);
    if (newEnabled != systemEnabled) {
      systemEnabled = newEnabled;
      if (!systemEnabled) relayWrite(false);
      saveRuntimeState();
    } else if (!newEnabled) {
      // ensure relay stays off if command repeats disable
      relayWrite(false);
    }
    mqttPublishState(true);
    return;
  }

  if (strlen(config.topicSetpointIn) > 0 && t == String(config.topicSetpointIn)) {
    float v;
    if (parseFloat(p, v)) {
      if (!isnan(v) && v != targetHumidity) {
        targetHumidity = v;
        saveRuntimeState();
      }
      mqttPublishState(true);
    }
    return;
  }

  if (strlen(config.topicHumidityIn) > 0 && t == String(config.topicHumidityIn)) {
    uint32_t now = millis();

    // Always count received messages as valid samples for connection stability check
    if (humiditySamplesSinceMqttConnect < 255) humiditySamplesSinceMqttConnect++;

    // Throttle: accept no more often than configured interval
    if (config.humidityMinIntervalMs > 0 && lastHumidityAcceptMs > 0 && (now - lastHumidityAcceptMs) < config.humidityMinIntervalMs) {
      lastHumiditySeenMs = now; // still mark seen, but don't change value
      return;
    }

    float v;
    if (parseFloat(p, v)) {
      currentHumidity = v;
      lastHumidityAcceptMs = now;
      lastHumiditySeenMs = now;
      mqttPublishState(true);
    }
    return;
  }
}

static bool mqttConnectIfNeeded() {
  if (mqtt.connected()) return true;
  if (strlen(config.mqttHost) == 0) return false;

  uint32_t now = millis();
  if ((now - lastMqttAttemptMs) < mqttBackoffMs) return false;

  lastMqttAttemptMs = now;

  mqtt.setServer(config.mqttHost, config.mqttPort);
  mqtt.setCallback(mqttCallback);

  String clientId = deviceId();

  String willTopic = topicOf("status/online");

  bool ok;
  if (strlen(config.mqttUser) > 0) {
    ok = mqtt.connect(clientId.c_str(), config.mqttUser, config.mqttPass, willTopic.c_str(), 0, true, "0");
  } else {
    ok = mqtt.connect(clientId.c_str(), nullptr, nullptr, willTopic.c_str(), 0, true, "0");
  }

  if (!ok) {
    uint32_t nextBackoffMs = mqttBackoffMs * 2U;
    if (nextBackoffMs > MQTT_RECONNECT_MAX_MS) nextBackoffMs = MQTT_RECONNECT_MAX_MS;
    mqttBackoffMs = nextBackoffMs;
    return false;
  }

  mqttBackoffMs = MQTT_RECONNECT_MIN_MS;

  // Mark MQTT session as (re)connected; require fresh humidity samples before turning relay ON
  humiditySamplesSinceMqttConnect = 0;

  mqtt.publish(willTopic.c_str(), "1", true);

  // Subscriptions
  if (strlen(config.topicHumidityIn) > 0) mqtt.subscribe(config.topicHumidityIn);
  if (strlen(config.topicSetpointIn) > 0) mqtt.subscribe(config.topicSetpointIn);
  if (strlen(config.topicEnableIn) > 0) mqtt.subscribe(config.topicEnableIn);

  mqttPublishState(true);
  return true;
}

static void controlLoopTick() {
  // Without MQTT, we cannot receive external humidity reliably -> safe OFF
  if (!mqtt.connected()) {
    if (relayOn) relayWrite(false);
    return;
  }

  if (!systemEnabled) {
    if (relayOn) relayWrite(false);
    return;
  }

  if (isnan(currentHumidity)) {
    // No sensor yet -> safe OFF
    if (relayOn) relayWrite(false);
    return;
  }

  float low = targetHumidity - config.hysteresis;
  float high = targetHumidity + config.hysteresis;

  if (!relayOn && currentHumidity < low) {
    // Avoid turning ON based on a potentially stale retained value after reconnect.
    if (humiditySamplesSinceMqttConnect < 2) return;
    relayWrite(true);
    mqttPublishState(true);
  } else if (relayOn && currentHumidity > high) {
    relayWrite(false);
    mqttPublishState(true);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  loadConfig();

  pinMode(config.relayPin, OUTPUT);
  relayWrite(false);

  Serial.printf("Device: %s\n", deviceId().c_str());
  Serial.printf("Relay pin: %d, inverted: %s\n", config.relayPin, config.relayInverted ? "yes" : "no");

  bool staOk = connectWiFiSta();
  if (!staOk) {
    Serial.println("[WiFi] STA failed or not configured -> starting captive portal");
    startCaptivePortal();
  }
}

void loop() {
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  static bool lastMqttConnected = false;

  wl_status_t wifiStatus = WiFi.status();
  if (lastWifiStatus == WL_CONNECTED && wifiStatus != WL_CONNECTED) {
    // WiFi dropped -> external humidity likely stale; safe OFF
    currentHumidity = NAN;
    lastHumidityAcceptMs = 0;
    lastHumiditySeenMs = 0;
    humiditySamplesSinceMqttConnect = 0;
    if (relayOn) relayWrite(false);
  }
  lastWifiStatus = wifiStatus;

  if (captivePortalActive) dns.processNextRequest();
  web.handleClient();

  if (wifiStatus == WL_CONNECTED) {
    mqttConnectIfNeeded();
    mqtt.loop();
    if (otaActive) ArduinoOTA.handle();
  }

  bool mqttConnected = mqtt.connected();
  if (lastMqttConnected && !mqttConnected) {
    // MQTT dropped -> external humidity stale; safe OFF
    currentHumidity = NAN;
    lastHumidityAcceptMs = 0;
    lastHumiditySeenMs = 0;
    humiditySamplesSinceMqttConnect = 0;
    if (relayOn) relayWrite(false);
  }
  lastMqttConnected = mqttConnected;

  static uint32_t lastControlMs = 0;
  uint32_t now = millis();
  if ((now - lastControlMs) >= 1000U) {
    lastControlMs = now;
    controlLoopTick();
    if (mqtt.connected()) mqttPublishState(false);
  }
}
