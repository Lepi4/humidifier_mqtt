#include <Arduino.h>

#include <stdarg.h>

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

#include <PubSubClient.h>

#include <ArduinoOTA.h>
#include <ESPmDNS.h>

#include <Update.h>

#include <driver/gpio.h>

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

static constexpr float SETPOINT_MIN = 10.0f;
static constexpr float SETPOINT_MAX = 80.0f;

struct AppConfig {
  char wifiSsid[33] = {0};
  char wifiPass[65] = {0};

  char webUser[33] = {0};
  char webPass[65] = {0};

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

  uint8_t logLevel = 2; // 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG

  uint32_t hangTimeoutSec = 0; // 0 disables
  uint8_t hangAction = 1;      // 1=restart MQTT, 2=reboot

  bool haDiscoveryEnabled = false;
  char haDiscoveryPrefix[33] = "homeassistant";
  char haDeviceName[65] = {0};
};

static Preferences prefs;
static AppConfig config;

static float clampSetpoint(float v) {
  if (isnan(v)) return v;
  if (v < SETPOINT_MIN) return SETPOINT_MIN;
  if (v > SETPOINT_MAX) return SETPOINT_MAX;
  return v;
}

enum LogLevel : uint8_t {
  LOG_ERROR = 0,
  LOG_WARN = 1,
  LOG_INFO = 2,
  LOG_DEBUG = 3,
};

static constexpr size_t LOG_LINE_MAX = 180;
static constexpr size_t LOG_LINES_MAX = 120;
static char logLines[LOG_LINES_MAX][LOG_LINE_MAX];
static uint16_t logHead = 0;
static uint16_t logCount = 0;

static const char *logLevelName(uint8_t lvl) {
  switch (lvl) {
    case LOG_ERROR: return "ERROR";
    case LOG_WARN: return "WARN";
    case LOG_INFO: return "INFO";
    case LOG_DEBUG: return "DEBUG";
    default: return "INFO";
  }
}

static void logWriteLine(uint8_t level, const char *line) {
  if (level > config.logLevel) return;

  char buf[LOG_LINE_MAX];
  snprintf(buf, sizeof(buf), "[%10lu] %-5s %s", (unsigned long)millis(), logLevelName(level), line);
  Serial.println(buf);

  strncpy(logLines[logHead], buf, LOG_LINE_MAX - 1);
  logLines[logHead][LOG_LINE_MAX - 1] = '\0';
  logHead = (uint16_t)((logHead + 1) % LOG_LINES_MAX);
  if (logCount < LOG_LINES_MAX) logCount++;
}

static void logf(uint8_t level, const char *fmt, ...) {
  if (level > config.logLevel) return;
  char msg[LOG_LINE_MAX];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  logWriteLine(level, msg);
}

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

static String lastAutomationReason;

// Arduino-ESP32 calls initVariant() before setup().
// Use it to drive the relay GPIO to a safe state as early as possible.
extern "C" void initVariant() {
  const int pin = HUM_DEFAULT_RELAY_PIN;
  if (pin < 0 || pin > 39) return;

  const bool inverted = (HUM_DEFAULT_RELAY_INVERTED != 0);
  bool level = false; // OFF
  if (inverted) level = !level;

  const gpio_num_t gpio = (gpio_num_t)pin;
  gpio_reset_pin(gpio);
  gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
  gpio_set_level(gpio, level ? 1 : 0);
}

static uint32_t mqttDisconnectedSinceMs = 0;
static uint32_t lastHangActionMs = 0;

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
    logWriteLine(LOG_INFO, "[OTA] Start");
  });
  ArduinoOTA.onEnd([]() {
    logWriteLine(LOG_INFO, "[OTA] End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) return;
    const unsigned int pct = (progress * 100U) / total;
    static unsigned int lastPct = 101;
    if (pct != lastPct) {
      lastPct = pct;
      logf(LOG_INFO, "[OTA] %u%%", pct);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    logf(LOG_ERROR, "[OTA] Error: %u", (unsigned)error);
  });

  ArduinoOTA.begin();
  otaActive = true;
  logf(LOG_INFO, "[OTA] Ready. Hostname: %s", host.c_str());
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
  prefs.putString("webUser", config.webUser);
  prefs.putString("webPass", config.webPass);
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
  prefs.putUChar("logLvl", config.logLevel);
  prefs.putULong("hangSec", config.hangTimeoutSec);
  prefs.putUChar("hangAct", config.hangAction);
  prefs.putBool("haDisc", config.haDiscoveryEnabled);
  prefs.putString("haPref", config.haDiscoveryPrefix);
  prefs.putString("haName", config.haDeviceName);
  prefs.putFloat("target", clampSetpoint(targetHumidity));
  prefs.putBool("sysEn", systemEnabled);
  prefs.end();
}

static void loadConfig() {
  prefs.begin("hum", true);

  String wifiSsid = prefs.getString("wifiSsid", "");
  String wifiPass = prefs.getString("wifiPass", "");

  String webUser = prefs.getString("webUser", "admin");
  String webPass = prefs.getString("webPass", "admin");

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
  uint8_t logLvl = prefs.getUChar("logLvl", (uint8_t)LOG_INFO);
  uint32_t hangSec = prefs.getULong("hangSec", 0);
  uint8_t hangAct = prefs.getUChar("hangAct", 1);

  bool haDisc = prefs.getBool("haDisc", false);
  String haPref = prefs.getString("haPref", "homeassistant");
  String haName = prefs.getString("haName", "");

  float storedTarget = prefs.getFloat("target", DEFAULT_SETPOINT);
  bool storedEnabled = prefs.getBool("sysEn", true);

  prefs.end();

  strncpy(config.wifiSsid, wifiSsid.c_str(), sizeof(config.wifiSsid) - 1);
  strncpy(config.wifiPass, wifiPass.c_str(), sizeof(config.wifiPass) - 1);

  strncpy(config.webUser, webUser.c_str(), sizeof(config.webUser) - 1);
  strncpy(config.webPass, webPass.c_str(), sizeof(config.webPass) - 1);

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
  config.logLevel = logLvl;
  config.hangTimeoutSec = hangSec;
  config.hangAction = hangAct;

  config.haDiscoveryEnabled = haDisc;
  strncpy(config.haDiscoveryPrefix, haPref.c_str(), sizeof(config.haDiscoveryPrefix) - 1);
  strncpy(config.haDeviceName, haName.c_str(), sizeof(config.haDeviceName) - 1);

  if (!isnan(storedTarget)) targetHumidity = clampSetpoint(storedTarget);
  systemEnabled = storedEnabled;

  targetHumidity = clampSetpoint(targetHumidity);
}

static void saveRuntimeState() {
  prefs.begin("hum", false);
  prefs.putFloat("target", clampSetpoint(targetHumidity));
  prefs.putBool("sysEn", systemEnabled);
  prefs.end();
}

static void mqttPublishState(bool force);

static String automationReason() {
  if (!mqtt.connected()) return "mqtt_disconnected";
  if (!systemEnabled) return "disabled";
  if (isnan(currentHumidity)) return "no_humidity";
  if (humiditySamplesSinceMqttConnect < 2) return "waiting_samples";

  float low = targetHumidity - config.hysteresis;
  float high = targetHumidity + config.hysteresis;

  if (relayOn) return "humidifying";
  if (currentHumidity < low) return "below_low";
  if (currentHumidity > high) return "above_high";
  return "within_band";
}

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

static String jsonEscape(const String &s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '\\': o += "\\\\"; break;
      case '"': o += "\\\""; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
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
  page += "<p><a href='/logs'>Logs</a></p>";

  if (notice.length() > 0) {
    page += "<p><b>" + htmlEscape(notice) + "</b></p>";
  }

  page += "<form method='POST' action='/save'>";

  page += "<h3>WiFi</h3>";
  page += "SSID:<br><input name='wifi_ssid' maxlength='32' value='" + htmlEscape(String(config.wifiSsid)) + "'><br>";
  page += "Password:<br><input name='wifi_pass' type='password' maxlength='64' value='" + htmlEscape(String(config.wifiPass)) + "'><br>";

  page += "<h3>Web Access</h3>";
  page += "Username:<br><input name='web_user' maxlength='32' value='" + htmlEscape(String(config.webUser)) + "'><br>";
  page += "New password (leave blank to keep):<br><input name='web_pass' type='password' maxlength='64' value=''><br>";
  page += "Confirm new password:<br><input name='web_pass2' type='password' maxlength='64' value=''><br>";

  page += "<h3>MQTT</h3>";
  page += "Host:<br><input name='mqtt_host' maxlength='64' value='" + htmlEscape(String(config.mqttHost)) + "'><br>";
  page += "Port:<br><input name='mqtt_port' type='number' min='1' max='65535' value='" + String(config.mqttPort) + "'><br>";
  page += "User:<br><input name='mqtt_user' maxlength='64' value='" + htmlEscape(String(config.mqttUser)) + "'><br>";
  page += "Password:<br><input name='mqtt_pass' type='password' maxlength='64' value='" + htmlEscape(String(config.mqttPass)) + "'><br>";

  page += "<h3>Home Assistant</h3>";
  page += String("Enable MQTT Discovery: <input type='checkbox' name='ha_disc' value='1'") + (config.haDiscoveryEnabled ? " checked" : "") + "><br>";
  page += "Discovery prefix:<br><input name='ha_prefix' maxlength='32' value='" + htmlEscape(String(config.haDiscoveryPrefix)) + "'><br>";
  page += "Device name in HA (optional):<br><input name='ha_name' maxlength='64' value='" + htmlEscape(String(config.haDeviceName)) + "'><br>";
  page += "<p><button type='submit'>Save & Reboot</button></p>";

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

  page += "<h3>Diagnostics</h3>";
  page += "Log level:<br><select name='log_level'>";
  page += String("<option value='0'") + (config.logLevel == 0 ? " selected" : "") + ">ERROR</option>";
  page += String("<option value='1'") + (config.logLevel == 1 ? " selected" : "") + ">WARN</option>";
  page += String("<option value='2'") + (config.logLevel == 2 ? " selected" : "") + ">INFO</option>";
  page += String("<option value='3'") + (config.logLevel == 3 ? " selected" : "") + ">DEBUG</option>";
  page += "</select><br>";

  page += "Hang timeout (sec, 0=off):<br><input name='hang_sec' type='number' min='0' max='86400' value='" + String(config.hangTimeoutSec) + "'><br>";
  page += "Hang action:<br><select name='hang_act'>";
  page += String("<option value='1'") + (config.hangAction == 1 ? " selected" : "") + ">Restart MQTT</option>";
  page += String("<option value='2'") + (config.hangAction == 2 ? " selected" : "") + ">Reboot device</option>";
  page += "</select><br>";

  page += "<p><button type='submit'>Save & Reboot</button></p>";
  page += "</form>";

  page += "<hr>";
  page += "<h3>Quick control</h3>";
  page += "<form method='POST' action='/control'>";
  page += "Enable automation: <select name='enabled'>";
  page += String("<option value='1'") + (systemEnabled ? " selected" : "") + ">ON</option>";
  page += String("<option value='0'") + (!systemEnabled ? " selected" : "") + ">OFF</option>";
  page += "</select><br>";
  page += "Target humidity (%RH):<br><input name='setpoint' type='number' min='" + String(SETPOINT_MIN, 0) + "' max='" + String(SETPOINT_MAX, 0) + "' step='0.1' value='" + String(targetHumidity, 1) + "'><br>";
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

static bool httpIsAuthorized() {
  if (captivePortalActive) return true;
  return web.authenticate(config.webUser, config.webPass);
}

static bool httpRequireAuthorized() {
  if (httpIsAuthorized()) return true;
  web.requestAuthentication(BASIC_AUTH, "Humidifier", "Authentication required");
  return false;
}

static void httpSetupHandlers() {
  web.on("/", HTTP_GET, []() {
    if (!httpRequireAuthorized()) return;
    web.send(200, "text/html", configPage());
  });

  web.on("/logs", HTTP_GET, []() {
    if (!httpRequireAuthorized()) return;

    bool plain = web.hasArg("plain") && web.arg("plain") == "1";

    if (plain) {
      String out;
      out.reserve((size_t)logCount * 80U + 64U);
      uint16_t start = (uint16_t)((logHead + LOG_LINES_MAX - logCount) % LOG_LINES_MAX);
      for (uint16_t i = 0; i < logCount; i++) {
        uint16_t idx = (uint16_t)((start + i) % LOG_LINES_MAX);
        out += logLines[idx];
        out += "\n";
      }
      web.send(200, "text/plain", out);
      return;
    }

    String page;
    page.reserve(2000);
    page += "<!doctype html><html><head><meta charset='utf-8'>";
    page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    page += "<title>Logs</title></head><body>";
    page += "<h2>Logs</h2>";
    page += "<p><a href='/'>Back</a> | <a href='/logs?plain=1'>Plain</a></p>";
    page += "<pre style='white-space:pre-wrap'>";

    uint16_t start = (uint16_t)((logHead + LOG_LINES_MAX - logCount) % LOG_LINES_MAX);
    for (uint16_t i = 0; i < logCount; i++) {
      uint16_t idx = (uint16_t)((start + i) % LOG_LINES_MAX);
      page += htmlEscape(String(logLines[idx]));
      page += "\n";
    }

    page += "</pre>";
    page += "</body></html>";
    web.send(200, "text/html", page);
  });

  web.on("/control", HTTP_POST, []() {
    if (!httpRequireAuthorized()) return;
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
    // Disabling automation must always force the humidifier OFF immediately.
    if (!systemEnabled) relayWrite(false);

    String setpointStr = arg("setpoint");
    float newSetpoint;
    if (parseFloat(setpointStr, newSetpoint) && !isnan(newSetpoint)) {
      float clamped = clampSetpoint(newSetpoint);
      if (!isnan(clamped) && clamped != targetHumidity) {
        targetHumidity = clamped;
        changed = true;
      }
    }

    if (changed) saveRuntimeState();
    mqttPublishState(true);

    web.send(200, "text/html", configPage(changed ? "Applied." : "No changes."));
  });

  web.on("/update", HTTP_GET, []() {
    if (!httpRequireAuthorized()) return;
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
        if (!httpRequireAuthorized()) return;
        const bool ok = !Update.hasError();
        web.send(200, "text/plain", ok ? "OK\nRebooting..." : "FAIL\n");
        delay(300);
        if (ok) ESP.restart();
      },
      []() {
        if (!httpIsAuthorized()) return;
        HTTPUpload &upload = web.upload();
        if (upload.status == UPLOAD_FILE_START) {
          logf(LOG_INFO, "[WEB OTA] Start: %s, size=%u", upload.filename.c_str(), (unsigned)upload.totalSize);
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true)) {
            logf(LOG_INFO, "[WEB OTA] Success: %u bytes", (unsigned)upload.totalSize);
          } else {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          Update.end();
          logWriteLine(LOG_WARN, "[WEB OTA] Aborted");
        }
      });

  web.on("/save", HTTP_POST, []() {
    if (!httpRequireAuthorized()) return;
    auto arg = [&](const char *name) -> String {
      return web.hasArg(name) ? web.arg(name) : String("");
    };

    String wifiSsid = arg("wifi_ssid");
    String wifiPass = arg("wifi_pass");

    String webUser = arg("web_user");
    String webPass = arg("web_pass");
    String webPass2 = arg("web_pass2");

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
    String logLevelStr = arg("log_level");
    String hangSecStr = arg("hang_sec");
    String hangActStr = arg("hang_act");

    bool haDisc = web.hasArg("ha_disc");
    String haPrefix = arg("ha_prefix");
    String haName = arg("ha_name");

    wifiSsid.trim();
    webUser.trim();
    mqttHost.trim();
    baseTopic.trim();
    tHumIn.trim();
    tSetIn.trim();
    tEnIn.trim();

    haPrefix.trim();
    haName.trim();

    if (webPass.length() > 0 && webPass != webPass2) {
      web.send(400, "text/html", configPage("Web password mismatch (not saved)."));
      return;
    }

    if (wifiSsid.length() >= sizeof(config.wifiSsid)) wifiSsid = wifiSsid.substring(0, sizeof(config.wifiSsid) - 1);
    if (wifiPass.length() >= sizeof(config.wifiPass)) wifiPass = wifiPass.substring(0, sizeof(config.wifiPass) - 1);
    if (webUser.length() >= sizeof(config.webUser)) webUser = webUser.substring(0, sizeof(config.webUser) - 1);
    if (webPass.length() >= sizeof(config.webPass)) webPass = webPass.substring(0, sizeof(config.webPass) - 1);
    if (mqttHost.length() >= sizeof(config.mqttHost)) mqttHost = mqttHost.substring(0, sizeof(config.mqttHost) - 1);
    if (mqttUser.length() >= sizeof(config.mqttUser)) mqttUser = mqttUser.substring(0, sizeof(config.mqttUser) - 1);
    if (mqttPass.length() >= sizeof(config.mqttPass)) mqttPass = mqttPass.substring(0, sizeof(config.mqttPass) - 1);
    if (baseTopic.length() >= sizeof(config.baseTopic)) baseTopic = baseTopic.substring(0, sizeof(config.baseTopic) - 1);
    if (tHumIn.length() >= sizeof(config.topicHumidityIn)) tHumIn = tHumIn.substring(0, sizeof(config.topicHumidityIn) - 1);
    if (tSetIn.length() >= sizeof(config.topicSetpointIn)) tSetIn = tSetIn.substring(0, sizeof(config.topicSetpointIn) - 1);
    if (tEnIn.length() >= sizeof(config.topicEnableIn)) tEnIn = tEnIn.substring(0, sizeof(config.topicEnableIn) - 1);

    if (haPrefix.length() >= sizeof(config.haDiscoveryPrefix)) haPrefix = haPrefix.substring(0, sizeof(config.haDiscoveryPrefix) - 1);
    if (haName.length() >= sizeof(config.haDeviceName)) haName = haName.substring(0, sizeof(config.haDeviceName) - 1);

    strncpy(config.wifiSsid, wifiSsid.c_str(), sizeof(config.wifiSsid) - 1);
    strncpy(config.wifiPass, wifiPass.c_str(), sizeof(config.wifiPass) - 1);

    if (webUser.length() > 0) strncpy(config.webUser, webUser.c_str(), sizeof(config.webUser) - 1);
    if (webPass.length() > 0) strncpy(config.webPass, webPass.c_str(), sizeof(config.webPass) - 1);

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

    int lvl = logLevelStr.toInt();
    if (lvl < 0) lvl = 0;
    if (lvl > 3) lvl = 3;
    config.logLevel = (uint8_t)lvl;

    uint32_t hangSec = (uint32_t)hangSecStr.toInt();
    if (hangSec > 86400U) hangSec = 86400U;
    config.hangTimeoutSec = hangSec;

    int hangAct = hangActStr.toInt();
    if (hangAct != 2) hangAct = 1;
    config.hangAction = (uint8_t)hangAct;

    config.haDiscoveryEnabled = haDisc;
    if (haPrefix.length() == 0) haPrefix = "homeassistant";
    strncpy(config.haDiscoveryPrefix, haPrefix.c_str(), sizeof(config.haDiscoveryPrefix) - 1);
    strncpy(config.haDeviceName, haName.c_str(), sizeof(config.haDeviceName) - 1);

    saveConfig();

    web.send(200, "text/html", configPage("Saved. Rebooting..."));
    delay(500);
    ESP.restart();
  });

  web.onNotFound([]() {
    if (captivePortalActive) {
      // Captive portal: always redirect to /
      web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
      web.send(302, "text/plain", "");
      return;
    }

    if (!httpRequireAuthorized()) return;
    web.send(404, "text/plain", "Not found");
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

  logf(LOG_INFO, "[AP] SSID: %s", HUM_DEFAULT_AP_SSID);
  logf(LOG_INFO, "[AP] IP: %s", apIP.toString().c_str());
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

  logf(LOG_INFO, "[WiFi] Connected, IP: %s", WiFi.localIP().toString().c_str());
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

static void mqttPublishDiscovery() {
  if (!mqtt.connected()) return;

  const String did = deviceId();

  String prefix = String(config.haDiscoveryPrefix);
  prefix.trim();
  if (prefix.length() == 0) prefix = "homeassistant";
  while (prefix.endsWith("/")) prefix.remove(prefix.length() - 1);

  const String devName = (strlen(config.haDeviceName) > 0) ? String(config.haDeviceName) : did;

  const String availTopic = topicOf("status/online");
  const String stateEnabled = topicOf("state/enabled");
  const String stateSetpoint = topicOf("state/setpoint");
  const String stateHumidity = topicOf("state/humidity");
  const String stateHumidityAge = topicOf("state/humidity_age_ms");
  const String stateRelay = topicOf("state/relay");
  const String stateReason = topicOf("state/reason");

  const String cmdEnable = (strlen(config.topicEnableIn) > 0) ? String(config.topicEnableIn) : topicOf("cmd/enabled");
  const String cmdSetpoint = (strlen(config.topicSetpointIn) > 0) ? String(config.topicSetpointIn) : topicOf("cmd/setpoint");

    const String dev =
      String("\"dev\":{\"ids\":[\"") + jsonEscape(did) +
      String("\"],\"name\":\"") + jsonEscape(devName) +
      String("\",\"mdl\":\"ESP32 Humidifier\",\"mf\":\"Custom\",\"sw\":\"") +
      jsonEscape(String(HUM_DEVICE_NAME)) + String("\"}");

  auto pub = [&](const String &topic, const String &payload) {
    if (!mqtt.publish(topic.c_str(), payload.c_str(), true)) {
      logf(LOG_WARN, "[MQTT] Publish failed (len=%u): %s", (unsigned)payload.length(), topic.c_str());
    }
  };

  const String oldHumidifierTopic = prefix + String("/humidifier/") + did + String("/config");
  const String humidifierTopic = prefix + String("/humidifier/") + did + String("/humidifier/config");
  const String humiditySensorTopic = prefix + String("/sensor/") + did + String("/humidity/config");
  const String humidityAgeSensorTopic = prefix + String("/sensor/") + did + String("/humidity_age_ms/config");
  const String relayBinarySensorTopic = prefix + String("/binary_sensor/") + did + String("/relay/config");
  const String reasonSensorTopic = prefix + String("/sensor/") + did + String("/automation_reason/config");

  if (!config.haDiscoveryEnabled) {
    mqtt.publish(oldHumidifierTopic.c_str(), "", true);
    mqtt.publish(humidifierTopic.c_str(), "", true);
    mqtt.publish(humiditySensorTopic.c_str(), "", true);
    mqtt.publish(humidityAgeSensorTopic.c_str(), "", true);
    mqtt.publish(relayBinarySensorTopic.c_str(), "", true);
    mqtt.publish(reasonSensorTopic.c_str(), "", true);
    logf(LOG_INFO, "[MQTT] HA discovery disabled; cleared %s/* for %s", prefix.c_str(), did.c_str());
    return;
  }

  // Humidifier entity
  {
    // Backward-compat cleanup: remove old (incorrect) 3-segment topic if it exists
    mqtt.publish(oldHumidifierTopic.c_str(), "", true);

    String payload;
    payload.reserve(820);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName) + "\",";
    payload += String("\"unique_id\":\"") + jsonEscape(did + String("_humidifier")) + "\",";
    payload += String("\"availability_topic\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"payload_available\":\"1\",\"payload_not_available\":\"0\",";
    payload += String("\"command_topic\":\"") + jsonEscape(cmdEnable) + "\",";
    payload += String("\"state_topic\":\"") + jsonEscape(stateEnabled) + "\",";
    payload += "\"payload_on\":\"1\",\"payload_off\":\"0\",";
    payload += String("\"target_humidity_command_topic\":\"") + jsonEscape(cmdSetpoint) + "\",";
    payload += String("\"target_humidity_state_topic\":\"") + jsonEscape(stateSetpoint) + "\",";
    payload += String("\"current_humidity_topic\":\"") + jsonEscape(stateHumidity) + "\",";
    payload += String("\"min_humidity\":") + String((int)SETPOINT_MIN) + ",";
    payload += String("\"max_humidity\":") + String((int)SETPOINT_MAX) + ",";
    payload += "\"device_class\":\"humidifier\",";
    payload += dev;
    payload += "}";
    pub(humidifierTopic, payload);
  }

  // Current humidity sensor
  {
    const String topic = humiditySensorTopic;
    String payload;
    payload.reserve(520);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Humidity")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_humidity")) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(stateHumidity) + "\",";
    payload += String("\"unit_of_meas\":\"%\",\"dev_cla\":\"humidity\",");
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(topic, payload);
  }

  // Humidity age sensor (ms)
  {
    const String topic = humidityAgeSensorTopic;
    String payload;
    payload.reserve(540);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Humidity age")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_humidity_age_ms")) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(stateHumidityAge) + "\",";
    payload += String("\"unit_of_meas\":\"ms\",");
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(topic, payload);
  }

  // Relay binary sensor
  {
    const String topic = relayBinarySensorTopic;
    String payload;
    payload.reserve(540);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Relay")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_relay")) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(stateRelay) + "\",";
    payload += "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"power\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(topic, payload);
  }

  // Automation reason sensor
  {
    const String topic = reasonSensorTopic;
    String payload;
    payload.reserve(560);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Automation reason")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_automation_reason")) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(stateReason) + "\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(topic, payload);
  }

  logf(LOG_INFO, "[MQTT] Discovery published under %s/* for %s", prefix.c_str(), did.c_str());
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

  String r = automationReason();
  mqtt.publish(topicOf("state/reason").c_str(), r.c_str(), true);
}

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String t(topic);
  String p;
  p.reserve(length);
  for (unsigned int i = 0; i < length; i++) p += (char)payload[i];
  p.trim();

  if (config.logLevel >= LOG_DEBUG) {
    logf(LOG_DEBUG, "[MQTT] RX topic=%s payload='%s'", t.c_str(), p.c_str());
  }

  if (strlen(config.topicEnableIn) > 0 && t == String(config.topicEnableIn)) {
    if (config.logLevel >= LOG_INFO) {
      logf(LOG_INFO, "[MQTT] CMD enabled topic=%s payload='%s'", t.c_str(), p.c_str());
    }
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
    if (config.logLevel >= LOG_INFO) {
      logf(LOG_INFO, "[MQTT] CMD setpoint topic=%s payload='%s'", t.c_str(), p.c_str());
    }
    float v;
    if (parseFloat(p, v)) {
      v = clampSetpoint(v);
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
      if (config.logLevel >= LOG_DEBUG) {
        logf(LOG_DEBUG, "[HUM] Throttled (%lums < %lums), keep=%.2f", (unsigned long)(now - lastHumidityAcceptMs),
             (unsigned long)config.humidityMinIntervalMs, isnan(currentHumidity) ? -1.0f : currentHumidity);
      }
      return;
    }

    float v;
    if (parseFloat(p, v)) {
      currentHumidity = v;
      lastHumidityAcceptMs = now;
      lastHumiditySeenMs = now;
      if (config.logLevel >= LOG_DEBUG) {
        logf(LOG_DEBUG, "[HUM] Accepted: %.2f (samples=%u)", currentHumidity, (unsigned)humiditySamplesSinceMqttConnect);
      }
      mqttPublishState(true);
    } else {
      if (config.logLevel >= LOG_WARN) {
        logf(LOG_WARN, "[HUM] Parse failed for payload='%s'", p.c_str());
      }
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
  mqtt.setBufferSize(1024);

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

    logf(LOG_INFO, "[MQTT] Connected. sub hum='%s' set='%s' en='%s' humInt=%lus", config.topicHumidityIn, config.topicSetpointIn,
      config.topicEnableIn, (unsigned long)(config.humidityMinIntervalMs / 1000U));

  mqttPublishDiscovery();

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
    if (humiditySamplesSinceMqttConnect < 2) {
      if (config.logLevel >= LOG_DEBUG) {
        logf(LOG_DEBUG, "[CTRL] Want ON but waiting samples=%u", (unsigned)humiditySamplesSinceMqttConnect);
      }
      return;
    }
    relayWrite(true);
    if (config.logLevel >= LOG_INFO) {
      logf(LOG_INFO, "[CTRL] Relay ON (hum=%.2f low=%.2f target=%.2f)", currentHumidity, low, targetHumidity);
    }
    mqttPublishState(true);
  } else if (relayOn && currentHumidity > high) {
    relayWrite(false);
    if (config.logLevel >= LOG_INFO) {
      logf(LOG_INFO, "[CTRL] Relay OFF (hum=%.2f high=%.2f target=%.2f)", currentHumidity, high, targetHumidity);
    }
    mqttPublishState(true);
  }
}

void setup() {
  // Try to avoid relay glitches at boot: drive the default relay pin to a safe OFF
  // level as early as possible, before delays, Wi-Fi/MQTT, and config load.
  int bootRelayPin = HUM_DEFAULT_RELAY_PIN;
  bool bootRelayInverted = (HUM_DEFAULT_RELAY_INVERTED != 0);
  {
    Preferences bootPrefs;
    if (bootPrefs.begin("hum", true)) {
      bootRelayPin = bootPrefs.getInt("relayPin", HUM_DEFAULT_RELAY_PIN);
      bootRelayInverted = bootPrefs.getBool("relayInv", HUM_DEFAULT_RELAY_INVERTED != 0);
      bootPrefs.end();
    }
  }
  if (bootRelayPin >= 0 && bootRelayPin <= 39) {
    pinMode(bootRelayPin, OUTPUT);
    bool bootLevel = false; // OFF
    if (bootRelayInverted) bootLevel = !bootLevel;
    digitalWrite(bootRelayPin, bootLevel ? HIGH : LOW);
  }

  Serial.begin(115200);
  delay(50);

  loadConfig();

  // If user config uses a different pin than default, release the default pin.
  if (config.relayPin != bootRelayPin && bootRelayPin >= 0 && bootRelayPin <= 39) {
    pinMode(bootRelayPin, INPUT);
  }

  pinMode(config.relayPin, OUTPUT);
  relayWrite(false);

  logf(LOG_INFO, "Device: %s", deviceId().c_str());
  logf(LOG_INFO, "Relay pin: %d, inverted: %s", config.relayPin, config.relayInverted ? "yes" : "no");

  bool staOk = connectWiFiSta();
  if (!staOk) {
    logWriteLine(LOG_WARN, "[WiFi] STA failed or not configured -> starting captive portal");
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

  if (wifiStatus == WL_CONNECTED) {
    if (!mqttConnected) {
      if (mqttDisconnectedSinceMs == 0) mqttDisconnectedSinceMs = millis();
    } else {
      mqttDisconnectedSinceMs = 0;
    }
  } else {
    mqttDisconnectedSinceMs = 0;
  }

  static uint32_t lastControlMs = 0;
  uint32_t now = millis();
  if ((now - lastControlMs) >= 1000U) {
    lastControlMs = now;
    controlLoopTick();

    if (mqtt.connected()) {
      String r = automationReason();
      if (r != lastAutomationReason) {
        lastAutomationReason = r;
        mqtt.publish(topicOf("state/reason").c_str(), r.c_str(), true);
      }
    }

    if (mqtt.connected()) mqttPublishState(false);

    // Anti-hang watchdog
    if (wifiStatus == WL_CONNECTED && config.hangTimeoutSec > 0) {
      uint32_t timeoutMs = config.hangTimeoutSec * 1000U;
      bool hang = false;
      const char *hangWhy = nullptr;

      if (!mqtt.connected()) {
        if (mqttDisconnectedSinceMs > 0 && (now - mqttDisconnectedSinceMs) > timeoutMs) {
          hang = true;
          hangWhy = "mqtt_disconnected";
        }
      } else {
        if (lastHumiditySeenMs > 0 && (now - lastHumiditySeenMs) > timeoutMs) {
          hang = true;
          hangWhy = "no_humidity";
        }
      }

      if (hang && (lastHangActionMs == 0 || (now - lastHangActionMs) > timeoutMs)) {
        lastHangActionMs = now;
        logf(LOG_WARN, "[WATCHDOG] Hang detected (%s), action=%u", hangWhy ? hangWhy : "unknown", (unsigned)config.hangAction);
        if (config.hangAction == 2) {
          delay(50);
          ESP.restart();
        } else {
          mqtt.disconnect();
          lastMqttAttemptMs = 0;
          mqttBackoffMs = MQTT_RECONNECT_MIN_MS;
        }
      }
    }
  }
}
