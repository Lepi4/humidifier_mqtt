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

#ifndef BTN_DEVICE_NAME
#define BTN_DEVICE_NAME "button-sim-esp32"
#endif

#ifndef BTN_DEFAULT_RELAY1_PIN
#define BTN_DEFAULT_RELAY1_PIN 23
#endif

#ifndef BTN_DEFAULT_RELAY2_PIN
#define BTN_DEFAULT_RELAY2_PIN 22
#endif

#ifndef BTN_DEFAULT_RELAY3_PIN
#define BTN_DEFAULT_RELAY3_PIN 21
#endif

#ifndef BTN_DEFAULT_RELAY4_PIN
#define BTN_DEFAULT_RELAY4_PIN 19
#endif

#ifndef BTN_DEFAULT_REED_PIN
// Passive reed switch input (GPIO). Must support internal pull-up if using INPUT_PULLUP.
#define BTN_DEFAULT_REED_PIN 18
#endif

#ifndef BTN_DEFAULT_REED_ENABLED
#define BTN_DEFAULT_REED_ENABLED 0
#endif

#ifndef BTN_DEFAULT_REED_NC
// 0 = NO contact used, 1 = NC contact used (affects OPEN/CLOSED interpretation)
#define BTN_DEFAULT_REED_NC 1
#endif

#ifndef BTN_DEFAULT_RELAY_COUNT
#define BTN_DEFAULT_RELAY_COUNT 2
#endif

#ifndef BTN_DEFAULT_RELAY_INVERTED
// 1 = active LOW (ON -> LOW)
#define BTN_DEFAULT_RELAY_INVERTED 1
#endif

#ifndef BTN_DEFAULT_AP_SSID
#define BTN_DEFAULT_AP_SSID "ButtonSim-Setup"
#endif

#ifndef BTN_DEFAULT_AP_PASS
#define BTN_DEFAULT_AP_PASS ""
#endif

static constexpr uint16_t HTTP_PORT = 80;
static constexpr uint16_t DNS_PORT = 53;

static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
static constexpr uint32_t MQTT_RECONNECT_MIN_MS = 1000;
static constexpr uint32_t MQTT_RECONNECT_MAX_MS = 30000;

static constexpr uint32_t DEFAULT_PRESS_MS = 200;
static constexpr uint32_t PRESS_MS_MIN = 20;
static constexpr uint32_t PRESS_MS_MAX = 5000;

static constexpr uint8_t RELAY_MAX = 4;

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
  char topicEnableIn[129] = {0};
  char topicButton1In[129] = {0};
  char topicButton2In[129] = {0};

  char topicButton3In[129] = {0};
  char topicButton4In[129] = {0};

  int relay1Pin = BTN_DEFAULT_RELAY1_PIN;
  int relay2Pin = BTN_DEFAULT_RELAY2_PIN;

  int relay3Pin = BTN_DEFAULT_RELAY3_PIN;
  int relay4Pin = BTN_DEFAULT_RELAY4_PIN;

  uint8_t relayCount = BTN_DEFAULT_RELAY_COUNT;
  bool relayInverted = (BTN_DEFAULT_RELAY_INVERTED != 0);

  uint32_t pressMs = DEFAULT_PRESS_MS;

  // Reed switch sensor (independent)
  int reedPin = BTN_DEFAULT_REED_PIN;
  bool reedEnabled = (BTN_DEFAULT_REED_ENABLED != 0);
  bool reedNc = (BTN_DEFAULT_REED_NC != 0);

  uint8_t logLevel = 2; // 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG

  uint32_t hangTimeoutSec = 0; // 0 disables
  uint8_t hangAction = 1;      // 1=restart MQTT, 2=reboot

  bool haDiscoveryEnabled = false;
  char haDiscoveryPrefix[33] = "homeassistant";
  char haDeviceName[65] = {0};
};

static Preferences prefs;
static AppConfig config;

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

#ifndef BTN_OTA_PASSWORD
#define BTN_OTA_PASSWORD ""
#endif

static bool systemEnabled = true;
static bool relay1On = false;
static bool relay2On = false;
static bool relay3On = false;
static bool relay4On = false;

static bool reedDoorClosed = false;
static bool reedHasStable = false;
static bool reedEverPublished = false;
static uint32_t reedLastSampleMs = 0;
static uint32_t reedLastPublishMs = 0;
static bool reedLastSampleValue = false;
static uint8_t reedStableSamples = 0;

struct PressJob {
  bool active = false;
  uint32_t startMs = 0;
};

static PressJob press1;
static PressJob press2;
static PressJob press3;
static PressJob press4;

// presses are independent per-channel; no global queue/gap needed

static uint32_t lastMqttAttemptMs = 0;
static uint32_t mqttBackoffMs = MQTT_RECONNECT_MIN_MS;
static uint32_t lastStatePublishMs = 0;

static uint32_t mqttDisconnectedSinceMs = 0;
static uint32_t lastHangActionMs = 0;

static bool isValidGpio(int pin) {
  return pin >= 0 && pin <= 39;
}

static bool parseReedNc(const String &value, bool defaultValue) {
  String v = value;
  v.trim();
  v.toLowerCase();
  if (v == "nc" || v == "1" || v == "true" || v == "yes") return true;
  if (v == "no" || v == "0" || v == "false" || v == "off") return false;
  return defaultValue;
}

static bool reedConfigured() {
  return config.reedEnabled && isValidGpio(config.reedPin);
}

static bool reedReadDoorClosedRaw() {
  if (!isValidGpio(config.reedPin)) return false;
  // With INPUT_PULLUP: closed contact -> LOW, open contact -> HIGH
  const bool contactClosed = (digitalRead(config.reedPin) == LOW);
  // Interpret OPEN/CLOSED depending on which contact pair is used (NO/NC)
  return config.reedNc ? contactClosed : !contactClosed;
}

static void reedApplyPinMode() {
  if (!isValidGpio(config.reedPin)) return;
  if (config.reedEnabled) {
    pinMode(config.reedPin, INPUT_PULLUP);
  } else {
    pinMode(config.reedPin, INPUT);
  }
  reedHasStable = false;
  reedStableSamples = 0;
}

static uint8_t clampRelayCount(int v) {
  if (v < 1) return 1;
  if (v > (int)RELAY_MAX) return RELAY_MAX;
  return (uint8_t)v;
}

static uint32_t clampPressMs(uint32_t v) {
  if (v < PRESS_MS_MIN) return PRESS_MS_MIN;
  if (v > PRESS_MS_MAX) return PRESS_MS_MAX;
  return v;
}

static int getRelayPin(uint8_t index) {
  switch (index) {
    case 1: return config.relay1Pin;
    case 2: return config.relay2Pin;
    case 3: return config.relay3Pin;
    case 4: return config.relay4Pin;
    default: return -1;
  }
}

static void setRelayPin(uint8_t index, int pin) {
  switch (index) {
    case 1: config.relay1Pin = pin; break;
    case 2: config.relay2Pin = pin; break;
    case 3: config.relay3Pin = pin; break;
    case 4: config.relay4Pin = pin; break;
    default: break;
  }
}

static bool getRelayOn(uint8_t index) {
  switch (index) {
    case 1: return relay1On;
    case 2: return relay2On;
    case 3: return relay3On;
    case 4: return relay4On;
    default: return false;
  }
}

static void setRelayOn(uint8_t index, bool on) {
  switch (index) {
    case 1: relay1On = on; break;
    case 2: relay2On = on; break;
    case 3: relay3On = on; break;
    case 4: relay4On = on; break;
    default: break;
  }
}

static PressJob *getPressJob(uint8_t index) {
  switch (index) {
    case 1: return &press1;
    case 2: return &press2;
    case 3: return &press3;
    case 4: return &press4;
    default: return nullptr;
  }
}

static const char *getTopicButtonIn(uint8_t index) {
  switch (index) {
    case 1: return config.topicButton1In;
    case 2: return config.topicButton2In;
    case 3: return config.topicButton3In;
    case 4: return config.topicButton4In;
    default: return "";
  }
}

static void copyTopicButtonIn(uint8_t index, const String &value) {
  switch (index) {
    case 1: strncpy(config.topicButton1In, value.c_str(), sizeof(config.topicButton1In) - 1); break;
    case 2: strncpy(config.topicButton2In, value.c_str(), sizeof(config.topicButton2In) - 1); break;
    case 3: strncpy(config.topicButton3In, value.c_str(), sizeof(config.topicButton3In) - 1); break;
    case 4: strncpy(config.topicButton4In, value.c_str(), sizeof(config.topicButton4In) - 1); break;
    default: break;
  }
}

static bool parseBool(const String &value, bool defaultValue) {
  String v = value;
  v.trim();
  v.toLowerCase();
  if (v == "1" || v == "on" || v == "true" || v == "yes" || v == "press" || v == "pressed") return true;
  if (v == "0" || v == "off" || v == "false" || v == "no") return false;
  return defaultValue;
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
      case '\"': o += "&quot;"; break;
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
      case '\"': o += "\\\""; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default: o += c; break;
    }
  }
  return o;
}

static String deviceId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "%s-%04X", BTN_DEVICE_NAME, (unsigned)(mac & 0xFFFF));
  return String(buf);
}

static void setupOta() {
  if (otaActive) return;

  const String host = deviceId();
  ArduinoOTA.setHostname(host.c_str());

  static constexpr const char *OTA_PASSWORD = BTN_OTA_PASSWORD;
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

static String topicOf(const char *suffix) {
  String base = String(config.baseTopic);
  if (base.length() == 0) base = "buttons/" + deviceId();
  if (!base.endsWith("/")) base += "/";
  return base + suffix;
}

static void relayWriteRaw(int pin, bool on) {
  if (!isValidGpio(pin)) return;
  bool level = on;
  if (config.relayInverted) level = !level;
  digitalWrite(pin, level ? HIGH : LOW);
}

static void relayWrite(uint8_t index, bool on) {
  if (index < 1 || index > RELAY_MAX) return;
  setRelayOn(index, on);
  relayWriteRaw(getRelayPin(index), on);
}

static void allRelaysOff() {
  for (uint8_t i = 1; i <= RELAY_MAX; i++) {
    relayWrite(i, false);
    PressJob *job = getPressJob(i);
    if (job) job->active = false;
  }
}

static void saveRuntimeState() {
  prefs.begin("btn", false);
  prefs.putBool("sysEn", systemEnabled);
  prefs.end();
}

static void savePressMsOnly() {
  prefs.begin("btn", false);
  prefs.putULong("pressMs", config.pressMs);
  prefs.end();
}

static void saveReedOnly() {
  prefs.begin("btn", false);
  prefs.putBool("reedEn", config.reedEnabled);
  prefs.putInt("reedPin", config.reedPin);
  prefs.putBool("reedNc", config.reedNc);
  prefs.end();
}

static void saveConfig() {
  prefs.begin("btn", false);
  prefs.putString("wifiSsid", config.wifiSsid);
  prefs.putString("wifiPass", config.wifiPass);
  prefs.putString("webUser", config.webUser);
  prefs.putString("webPass", config.webPass);
  prefs.putString("mqttHost", config.mqttHost);
  prefs.putUShort("mqttPort", config.mqttPort);
  prefs.putString("mqttUser", config.mqttUser);
  prefs.putString("mqttPass", config.mqttPass);
  prefs.putString("baseTopic", config.baseTopic);
  prefs.putString("tEnIn", config.topicEnableIn);
  prefs.putString("tB1In", config.topicButton1In);
  prefs.putString("tB2In", config.topicButton2In);
  prefs.putString("tB3In", config.topicButton3In);
  prefs.putString("tB4In", config.topicButton4In);
  prefs.putInt("r1Pin", config.relay1Pin);
  prefs.putInt("r2Pin", config.relay2Pin);
  prefs.putInt("r3Pin", config.relay3Pin);
  prefs.putInt("r4Pin", config.relay4Pin);
  prefs.putUChar("rCnt", config.relayCount);
  prefs.putBool("rInv", config.relayInverted);
  prefs.putULong("pressMs", config.pressMs);
  prefs.putBool("reedEn", config.reedEnabled);
  prefs.putInt("reedPin", config.reedPin);
  prefs.putBool("reedNc", config.reedNc);
  prefs.putUChar("logLvl", config.logLevel);
  prefs.putULong("hangSec", config.hangTimeoutSec);
  prefs.putUChar("hangAct", config.hangAction);
  prefs.putBool("haDisc", config.haDiscoveryEnabled);
  prefs.putString("haPref", config.haDiscoveryPrefix);
  prefs.putString("haName", config.haDeviceName);
  prefs.putBool("sysEn", systemEnabled);
  prefs.end();
}

static void loadConfig() {
  prefs.begin("btn", true);

  String wifiSsid = prefs.getString("wifiSsid", "");
  String wifiPass = prefs.getString("wifiPass", "");

  String webUser = prefs.getString("webUser", "admin");
  String webPass = prefs.getString("webPass", "admin");

  String mqttHost = prefs.getString("mqttHost", "");
  uint16_t mqttPort = prefs.getUShort("mqttPort", 1883);
  String mqttUser = prefs.getString("mqttUser", "");
  String mqttPass = prefs.getString("mqttPass", "");

  String baseTopic = prefs.getString("baseTopic", (String("buttons/") + deviceId()).c_str());
  String baseForDefaults = baseTopic;
  if (baseForDefaults.length() == 0) baseForDefaults = "buttons/" + deviceId();
  if (!baseForDefaults.endsWith("/")) baseForDefaults += "/";

  String tEnIn = prefs.getString("tEnIn", (baseForDefaults + "cmd/enabled").c_str());
  String tB1In = prefs.getString("tB1In", (baseForDefaults + "cmd/button1").c_str());
  String tB2In = prefs.getString("tB2In", (baseForDefaults + "cmd/button2").c_str());
  String tB3In = prefs.getString("tB3In", (baseForDefaults + "cmd/button3").c_str());
  String tB4In = prefs.getString("tB4In", (baseForDefaults + "cmd/button4").c_str());

  int r1Pin = prefs.getInt("r1Pin", BTN_DEFAULT_RELAY1_PIN);
  int r2Pin = prefs.getInt("r2Pin", BTN_DEFAULT_RELAY2_PIN);
  int r3Pin = prefs.getInt("r3Pin", BTN_DEFAULT_RELAY3_PIN);
  int r4Pin = prefs.getInt("r4Pin", BTN_DEFAULT_RELAY4_PIN);
  uint8_t rCnt = prefs.getUChar("rCnt", (uint8_t)BTN_DEFAULT_RELAY_COUNT);
  bool rInv = prefs.getBool("rInv", BTN_DEFAULT_RELAY_INVERTED != 0);

  uint32_t pressMs = prefs.getULong("pressMs", DEFAULT_PRESS_MS);

  bool reedEn = prefs.getBool("reedEn", (BTN_DEFAULT_REED_ENABLED != 0));
  int reedPin = prefs.getInt("reedPin", BTN_DEFAULT_REED_PIN);
  bool reedNc = prefs.getBool("reedNc", (BTN_DEFAULT_REED_NC != 0));

  uint8_t logLvl = prefs.getUChar("logLvl", (uint8_t)LOG_INFO);
  uint32_t hangSec = prefs.getULong("hangSec", 0);
  uint8_t hangAct = prefs.getUChar("hangAct", 1);

  bool haDisc = prefs.getBool("haDisc", false);
  String haPref = prefs.getString("haPref", "homeassistant");
  String haName = prefs.getString("haName", "");

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
  strncpy(config.topicEnableIn, tEnIn.c_str(), sizeof(config.topicEnableIn) - 1);
  strncpy(config.topicButton1In, tB1In.c_str(), sizeof(config.topicButton1In) - 1);
  strncpy(config.topicButton2In, tB2In.c_str(), sizeof(config.topicButton2In) - 1);
  strncpy(config.topicButton3In, tB3In.c_str(), sizeof(config.topicButton3In) - 1);
  strncpy(config.topicButton4In, tB4In.c_str(), sizeof(config.topicButton4In) - 1);

  config.relay1Pin = r1Pin;
  config.relay2Pin = r2Pin;
  config.relay3Pin = r3Pin;
  config.relay4Pin = r4Pin;
  config.relayCount = clampRelayCount((int)rCnt);
  config.relayInverted = rInv;

  config.pressMs = clampPressMs(pressMs);

  config.reedEnabled = reedEn;
  config.reedPin = reedPin;
  config.reedNc = reedNc;

  if (logLvl > 3) logLvl = 3;
  config.logLevel = logLvl;
  config.hangTimeoutSec = hangSec;
  config.hangAction = (hangAct == 2) ? 2 : 1;

  config.haDiscoveryEnabled = haDisc;
  strncpy(config.haDiscoveryPrefix, haPref.c_str(), sizeof(config.haDiscoveryPrefix) - 1);
  strncpy(config.haDeviceName, haName.c_str(), sizeof(config.haDeviceName) - 1);

  systemEnabled = storedEnabled;
}

static bool httpIsAuthorized() {
  if (captivePortalActive) return true;
  return web.authenticate(config.webUser, config.webPass);
}

static bool httpRequireAuthorized() {
  if (httpIsAuthorized()) return true;
  web.requestAuthentication(BASIC_AUTH, "ButtonSim", "Authentication required");
  return false;
}

static void mqttPublishBool(const String &topic, bool value, bool retain = true) {
  mqtt.publish(topic.c_str(), value ? "1" : "0", retain);
}

static void mqttPublishU32(const String &topic, uint32_t value, bool retain = true) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)value);
  mqtt.publish(topic.c_str(), buf, retain);
}

static void mqttClearRetained(const String &topic) {
  mqtt.publish(topic.c_str(), (const uint8_t *)"", 0, true);
}

static void mqttCleanupLegacyGap() {
  if (!mqtt.connected()) return;
  // Older firmware versions could have published/auto-discovered gap_ms.
  mqttClearRetained(topicOf("state/gap_ms"));

  const String did = deviceId();
  String prefix = String(config.haDiscoveryPrefix);
  prefix.trim();
  if (prefix.length() == 0) prefix = "homeassistant";
  while (prefix.endsWith("/")) prefix.remove(prefix.length() - 1);
  mqttClearRetained(prefix + String("/number/") + did + String("/gap_ms/config"));
}

static void mqttPublishState(bool force) {
  uint32_t now = millis();
  if (!force && (now - lastStatePublishMs) < 60000U) return;
  lastStatePublishMs = now;

  mqttPublishBool(topicOf("state/enabled"), systemEnabled);
  for (uint8_t i = 1; i <= config.relayCount; i++) {
    mqtt.publish(topicOf((String("state/relay") + String(i)).c_str()).c_str(), getRelayOn(i) ? "ON" : "OFF", true);
  }

  // Remove retained states for channels that are not in use.
  for (uint8_t i = config.relayCount + 1; i <= RELAY_MAX; i++) {
    const String t = topicOf((String("state/relay") + String(i)).c_str());
    mqtt.publish(t.c_str(), (const uint8_t *)"", 0, true);
  }

  mqttPublishU32(topicOf("state/press_ms"), config.pressMs);
  mqttPublishU32(topicOf("state/relay_count"), config.relayCount);
}

static void mqttPublishReedState(bool force) {
  if (!mqtt.connected()) return;

  const String t = topicOf("state/reed");

  if (!reedConfigured()) {
    if (reedEverPublished) {
      mqtt.publish(t.c_str(), (const uint8_t *)"", 0, true);
      reedEverPublished = false;
    }
    return;
  }

  uint32_t now = millis();
  if (!force && (now - reedLastPublishMs) < 5000U) return;
  reedLastPublishMs = now;

  mqtt.publish(t.c_str(), reedDoorClosed ? "CLOSED" : "OPEN", true);
  reedEverPublished = true;
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
  const String cmdEnable = (strlen(config.topicEnableIn) > 0) ? String(config.topicEnableIn) : topicOf("cmd/enabled");
  const String cmdB1 = (strlen(config.topicButton1In) > 0) ? String(config.topicButton1In) : topicOf("cmd/button1");
  const String cmdB2 = (strlen(config.topicButton2In) > 0) ? String(config.topicButton2In) : topicOf("cmd/button2");
  const String cmdB3 = (strlen(config.topicButton3In) > 0) ? String(config.topicButton3In) : topicOf("cmd/button3");
  const String cmdB4 = (strlen(config.topicButton4In) > 0) ? String(config.topicButton4In) : topicOf("cmd/button4");

  const String dev =
    String("\"dev\":{\"ids\":[\"") + jsonEscape(did) +
    String("\"],\"name\":\"") + jsonEscape(devName) +
    String("\",\"mdl\":\"ESP32 Button Simulator\",\"mf\":\"Custom\",\"sw\":\"") +
    jsonEscape(String(BTN_DEVICE_NAME)) + String("\"}");

  auto pub = [&](const String &topic, const String &payload) {
    if (!mqtt.publish(topic.c_str(), payload.c_str(), true)) {
      logf(LOG_WARN, "[MQTT] Publish failed (len=%u): %s", (unsigned)payload.length(), topic.c_str());
    }
  };

  const String button1Topic = prefix + String("/button/") + did + String("/button1/config");
  const String button2Topic = prefix + String("/button/") + did + String("/button2/config");
  const String button3Topic = prefix + String("/button/") + did + String("/button3/config");
  const String button4Topic = prefix + String("/button/") + did + String("/button4/config");
  const String enabledSwitchTopic = prefix + String("/switch/") + did + String("/enabled/config");
  const String reedSensorTopic = prefix + String("/binary_sensor/") + did + String("/reed/config");
  const String pressMsNumberTopic = prefix + String("/number/") + did + String("/press_ms/config");

  // Always clear legacy discovery topic if present.
  mqttClearRetained(prefix + String("/number/") + did + String("/gap_ms/config"));

  if (!config.haDiscoveryEnabled) {
    mqtt.publish(button1Topic.c_str(), "", true);
    mqtt.publish(button2Topic.c_str(), "", true);
    mqtt.publish(button3Topic.c_str(), "", true);
    mqtt.publish(button4Topic.c_str(), "", true);
    mqtt.publish(enabledSwitchTopic.c_str(), "", true);
    mqtt.publish(reedSensorTopic.c_str(), "", true);
    mqtt.publish(pressMsNumberTopic.c_str(), "", true);
    logf(LOG_INFO, "[MQTT] HA discovery disabled; cleared %s/* for %s", prefix.c_str(), did.c_str());
    return;
  }

  // Enabled switch
  {
    String payload;
    payload.reserve(520);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Enabled")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_enabled")) + "\",";
    payload += String("\"cmd_t\":\"") + jsonEscape(cmdEnable) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(topicOf("state/enabled")) + "\",";
    payload += "\"pl_on\":\"1\",\"pl_off\":\"0\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(enabledSwitchTopic, payload);
  }

  // Button 1
  {
    String payload;
    payload.reserve(480);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Button 1")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_button1")) + "\",";
    payload += String("\"cmd_t\":\"") + jsonEscape(cmdB1) + "\",";
    payload += "\"pl_prs\":\"PRESS\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(button1Topic, payload);
  }

  // Button 2: publish only if relayCount allows, otherwise clear old config
  if (config.relayCount >= 2) {
    String payload;
    payload.reserve(480);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Button 2")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_button2")) + "\",";
    payload += String("\"cmd_t\":\"") + jsonEscape(cmdB2) + "\",";
    payload += "\"pl_prs\":\"PRESS\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(button2Topic, payload);
  } else {
    mqtt.publish(button2Topic.c_str(), "", true);
  }

  // Button 3/4: publish only if relayCount allows, otherwise clear old configs
  if (config.relayCount >= 3) {
    String payload;
    payload.reserve(480);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Button 3")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_button3")) + "\",";
    payload += String("\"cmd_t\":\"") + jsonEscape(cmdB3) + "\",";
    payload += "\"pl_prs\":\"PRESS\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(button3Topic, payload);
  } else {
    mqtt.publish(button3Topic.c_str(), "", true);
  }

  if (config.relayCount >= 4) {
    String payload;
    payload.reserve(480);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Button 4")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_button4")) + "\",";
    payload += String("\"cmd_t\":\"") + jsonEscape(cmdB4) + "\",";
    payload += "\"pl_prs\":\"PRESS\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(button4Topic, payload);
  } else {
    mqtt.publish(button4Topic.c_str(), "", true);
  }

  // Reed switch sensor (publish only if enabled and pin is valid; otherwise clear old config)
  if (reedConfigured()) {
    String payload;
    payload.reserve(520);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Reed")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_reed")) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(topicOf("state/reed")) + "\",";
    payload += "\"pl_on\":\"OPEN\",\"pl_off\":\"CLOSED\",";
    payload += "\"dev_cla\":\"door\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(reedSensorTopic, payload);
  } else {
    mqtt.publish(reedSensorTopic.c_str(), "", true);
  }

  // Press duration number (ms)
  {
    const String cmdPressMs = topicOf("cmd/press_ms");
    String payload;
    payload.reserve(560);
    payload += "{";
    payload += String("\"name\":\"") + jsonEscape(devName + String(" Press ms")) + "\",";
    payload += String("\"uniq_id\":\"") + jsonEscape(did + String("_press_ms")) + "\",";
    payload += String("\"cmd_t\":\"") + jsonEscape(cmdPressMs) + "\",";
    payload += String("\"stat_t\":\"") + jsonEscape(topicOf("state/press_ms")) + "\",";
    payload += String("\"min\":") + String(PRESS_MS_MIN) + ",";
    payload += String("\"max\":") + String(PRESS_MS_MAX) + ",";
    payload += "\"step\":10,";
    payload += "\"mode\":\"box\",";
    payload += "\"unit_of_meas\":\"ms\",";
    payload += String("\"avty_t\":\"") + jsonEscape(availTopic) + "\",";
    payload += "\"pl_avail\":\"1\",\"pl_not_avail\":\"0\",";
    payload += dev;
    payload += "}";
    pub(pressMsNumberTopic, payload);
  }

  logf(LOG_INFO, "[MQTT] Discovery published under %s/* for %s", prefix.c_str(), did.c_str());
}

static void requestPress(uint8_t index, const char *why) {
  if (!systemEnabled) {
    logf(LOG_WARN, "[PRESS] Ignored (disabled) btn=%u why=%s", (unsigned)index, why ? why : "?");
    return;
  }

  if (index < 1 || index > config.relayCount || index > RELAY_MAX) {
    logf(LOG_WARN, "[PRESS] Ignored (invalid btn=%u, relayCount=%u)", (unsigned)index, (unsigned)config.relayCount);
    return;
  }
  PressJob *job = getPressJob(index);
  if (!job) return;
  job->active = true;
  job->startMs = millis();
  relayWrite(index, true);

  if (mqtt.connected()) {
    const String evt = topicOf((String("event/button") + String(index)).c_str());
    mqtt.publish(evt.c_str(), "PRESS", false);
  }

  if (config.logLevel >= LOG_INFO) {
    logf(LOG_INFO, "[PRESS] btn=%u start (%lums) why=%s", (unsigned)index, (unsigned long)config.pressMs, why ? why : "?");
  }

  if (mqtt.connected()) mqttPublishState(true);
}

static void pressLoopTick() {
  const uint32_t now = millis();
  // Per-channel finish checks
  for (uint8_t i = 1; i <= config.relayCount && i <= RELAY_MAX; i++) {
    PressJob *job = getPressJob(i);
    if (!job) continue;
    if (job->active && (uint32_t)(now - job->startMs) >= config.pressMs) {
      job->active = false;
      relayWrite(i, false);
      if (config.logLevel >= LOG_INFO) {
        logf(LOG_INFO, "[PRESS] btn=%u end", (unsigned)i);
      }
      if (mqtt.connected()) mqttPublishState(true);
    }
  }
}

static String configPage(const String &notice = "") {
  String apMode = (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) ? "AP" : "STA";

  String page;
  page.reserve(4200);
  page += "<!doctype html><html><head><meta charset='utf-8'>";
  page += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  page += "<title>Button Simulator Setup</title></head><body>";
  page += "<h2>Button Simulator Setup</h2>";
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

  page += "<h3>Topics</h3>";
  page += "Base topic:<br><input name='base_topic' maxlength='128' value='" + htmlEscape(String(config.baseTopic)) + "'><br>";
  page += "Enable topic (subscribe):<br><input name='t_en_in' maxlength='128' value='" + htmlEscape(String(config.topicEnableIn)) + "'><br>";
  page += "Button 1 topic (subscribe):<br><input name='t_b1_in' maxlength='128' value='" + htmlEscape(String(config.topicButton1In)) + "'><br>";
  page += "Button 2 topic (subscribe):<br><input name='t_b2_in' maxlength='128' value='" + htmlEscape(String(config.topicButton2In)) + "'><br>";
  page += "Button 3 topic (subscribe):<br><input name='t_b3_in' maxlength='128' value='" + htmlEscape(String(config.topicButton3In)) + "'><br>";
  page += "Button 4 topic (subscribe):<br><input name='t_b4_in' maxlength='128' value='" + htmlEscape(String(config.topicButton4In)) + "'><br>";

  page += "<h3>Control</h3>";
  page += "Relay modules count (1-4):<br><input name='relay_count' type='number' min='1' max='4' value='" + String(config.relayCount) + "'><br>";
  page += "Relay 1 pin (GPIO):<br><input name='relay1_pin' type='number' min='0' max='39' value='" + String(config.relay1Pin) + "'><br>";
  page += "Relay 2 pin (GPIO):<br><input name='relay2_pin' type='number' min='0' max='39' value='" + String(config.relay2Pin) + "'><br>";
  page += "Relay 3 pin (GPIO):<br><input name='relay3_pin' type='number' min='0' max='39' value='" + String(config.relay3Pin) + "'><br>";
  page += "Relay 4 pin (GPIO):<br><input name='relay4_pin' type='number' min='0' max='39' value='" + String(config.relay4Pin) + "'><br>";
  page += "Relay inverted (1=ON->LOW):<br><input name='relay_inv' maxlength='5' value='" + String(config.relayInverted ? "1" : "0") + "'><br>";
  page += "Press duration (ms):<br><input name='press_ms' type='number' min='" + String(PRESS_MS_MIN) + "' max='" + String(PRESS_MS_MAX) + "' value='" + String(config.pressMs) + "'><br>";

  page += "<h3>Reed sensor</h3>";
  page += String("Activate reed sensor: <input type='checkbox' name='reed_en' value='1'") + (config.reedEnabled ? " checked" : "") + "><br>";
  page += "Reed pin (GPIO):<br><input name='reed_pin' type='number' min='0' max='39' value='" + String(config.reedPin) + "'><br>";
  page += "Reed contact type (COM+NO / COM+NC):<br><select name='reed_type'>";
  page += String("<option value='NC'") + (config.reedNc ? " selected" : "") + ">NC</option>";
  page += String("<option value='NO'") + (!config.reedNc ? " selected" : "") + ">NO</option>";
  page += "</select><br>";

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
  page += "Enabled: <select name='enabled'>";
  page += String("<option value='1'") + (systemEnabled ? " selected" : "") + ">ON</option>";
  page += String("<option value='0'") + (!systemEnabled ? " selected" : "") + ">OFF</option>";
  page += "</select><br>";
  page += "Reed sensor: <select name='reed_en'>";
  page += String("<option value='1'") + (config.reedEnabled ? " selected" : "") + ">ON</option>";
  page += String("<option value='0'") + (!config.reedEnabled ? " selected" : "") + ">OFF</option>";
  page += "</select><br>";
  page += "Press duration (ms):<br><input name='press_ms' type='number' min='" + String(PRESS_MS_MIN) + "' max='" + String(PRESS_MS_MAX) + "' value='" + String(config.pressMs) + "'><br>";
  page += "<p><button type='submit'>Apply</button></p>";
  page += "</form>";

  for (uint8_t i = 1; i <= config.relayCount && i <= RELAY_MAX; i++) {
    page += "<form method='POST' action='/press'>";
    page += "<input type='hidden' name='btn' value='" + String(i) + "'>";
    page += "<button type='submit'>Press Button " + String(i) + "</button>";
    page += "</form>";
  }

  page += "<hr>";
  page += "<h3>Status</h3>";
  page += "Enabled: <b>" + String(systemEnabled ? "YES" : "NO") + "</b><br>";
  for (uint8_t i = 1; i <= config.relayCount && i <= RELAY_MAX; i++) {
    page += "Relay" + String(i) + ": <b>" + String(getRelayOn(i) ? "ON" : "OFF") + "</b><br>";
  }
  page += "Press ms: <b>" + String(config.pressMs) + "</b><br>";
  page += "Relay count: <b>" + String(config.relayCount) + "</b><br>";
  page += "Reed: <b>" + String(config.reedEnabled ? (reedHasStable ? (reedDoorClosed ? "CLOSED" : "OPEN") : "(reading)") : "disabled") + "</b><br>";
  page += "WiFi IP: <b>" + WiFi.localIP().toString() + "</b><br>";
  page += "MQTT: <b>" + String(mqtt.connected() ? "connected" : "disconnected") + "</b><br>";

  page += "</body></html>";
  return page;
}

static void httpSetupHandlers();

static void startCaptivePortal() {
  WiFi.mode(WIFI_AP);
  if (strlen(BTN_DEFAULT_AP_PASS) == 0) {
    WiFi.softAP(BTN_DEFAULT_AP_SSID);
  } else {
    WiFi.softAP(BTN_DEFAULT_AP_SSID, BTN_DEFAULT_AP_PASS);
  }

  IPAddress apIP = WiFi.softAPIP();
  dns.start(DNS_PORT, "*", apIP);
  captivePortalActive = true;

  httpSetupHandlers();
  web.begin();

  logf(LOG_INFO, "[AP] SSID: %s", BTN_DEFAULT_AP_SSID);
  if (strlen(BTN_DEFAULT_AP_PASS) == 0) logWriteLine(LOG_INFO, "[AP] Open network (no password)");
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

  web.on("/press", HTTP_POST, []() {
    if (!httpRequireAuthorized()) return;

    uint8_t btn = 0;
    if (web.hasArg("btn")) btn = (uint8_t)web.arg("btn").toInt();

    if (btn >= 1 && btn <= config.relayCount && btn <= RELAY_MAX) {
      requestPress(btn, "web");
      if (mqtt.connected()) mqttPublishState(true);
      web.send(200, "text/html", configPage(String("Pressed button ") + String(btn) + "."));
    } else {
      web.send(400, "text/html", configPage("Invalid button."));
    }
  });

  web.on("/control", HTTP_POST, []() {
    if (!httpRequireAuthorized()) return;

    auto arg = [&](const char *name) -> String {
      return web.hasArg(name) ? web.arg(name) : String("");
    };

    bool changed = false;

    if (web.hasArg("reed_en")) {
      bool newReedEn = parseBool(arg("reed_en"), config.reedEnabled);
      if (newReedEn != config.reedEnabled) {
        config.reedEnabled = newReedEn;
        reedApplyPinMode();
        saveReedOnly();
        changed = true;
      }
    }

    String enabledStr = arg("enabled");
    bool newEnabled = parseBool(enabledStr, systemEnabled);
    if (newEnabled != systemEnabled) {
      systemEnabled = newEnabled;
      changed = true;
    }
    if (!systemEnabled) allRelaysOff();

    if (web.hasArg("press_ms")) {
      uint32_t newPress = (uint32_t)arg("press_ms").toInt();
      newPress = clampPressMs(newPress);
      if (newPress != config.pressMs) {
        config.pressMs = newPress;
        savePressMsOnly();
        changed = true;
      }
    }

    if (changed) saveRuntimeState();
    if (mqtt.connected()) mqttPublishState(true);
    if (mqtt.connected()) mqttPublishReedState(true);

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
    }
  );

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
    String tEnIn = arg("t_en_in");
    String tB1In = arg("t_b1_in");
    String tB2In = arg("t_b2_in");
    String tB3In = arg("t_b3_in");
    String tB4In = arg("t_b4_in");

    String relayCountStr = arg("relay_count");
    String relay1Pin = arg("relay1_pin");
    String relay2Pin = arg("relay2_pin");
    String relay3Pin = arg("relay3_pin");
    String relay4Pin = arg("relay4_pin");
    String relayInv = arg("relay_inv");
    String pressMs = arg("press_ms");

    bool reedEn = web.hasArg("reed_en");
    String reedPinStr = arg("reed_pin");
    String reedTypeStr = arg("reed_type");

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
    tEnIn.trim();
    tB1In.trim();
    tB2In.trim();
    tB3In.trim();
    tB4In.trim();

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
    if (tEnIn.length() >= sizeof(config.topicEnableIn)) tEnIn = tEnIn.substring(0, sizeof(config.topicEnableIn) - 1);
    if (tB1In.length() >= sizeof(config.topicButton1In)) tB1In = tB1In.substring(0, sizeof(config.topicButton1In) - 1);
    if (tB2In.length() >= sizeof(config.topicButton2In)) tB2In = tB2In.substring(0, sizeof(config.topicButton2In) - 1);
    if (tB3In.length() >= sizeof(config.topicButton3In)) tB3In = tB3In.substring(0, sizeof(config.topicButton3In) - 1);
    if (tB4In.length() >= sizeof(config.topicButton4In)) tB4In = tB4In.substring(0, sizeof(config.topicButton4In) - 1);

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
    strncpy(config.topicEnableIn, tEnIn.c_str(), sizeof(config.topicEnableIn) - 1);
    strncpy(config.topicButton1In, tB1In.c_str(), sizeof(config.topicButton1In) - 1);
    strncpy(config.topicButton2In, tB2In.c_str(), sizeof(config.topicButton2In) - 1);
    strncpy(config.topicButton3In, tB3In.c_str(), sizeof(config.topicButton3In) - 1);
    strncpy(config.topicButton4In, tB4In.c_str(), sizeof(config.topicButton4In) - 1);

    config.relayCount = clampRelayCount(relayCountStr.toInt());

    config.relay1Pin = relay1Pin.toInt();
    config.relay2Pin = relay2Pin.toInt();
    config.relay3Pin = relay3Pin.toInt();
    config.relay4Pin = relay4Pin.toInt();
    config.relayInverted = parseBool(relayInv, config.relayInverted);

    config.pressMs = clampPressMs((uint32_t)pressMs.toInt());

    config.reedEnabled = reedEn;
    config.reedPin = reedPinStr.toInt();
    config.reedNc = parseReedNc(reedTypeStr, config.reedNc);

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
      web.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
      web.send(302, "text/plain", "");
      return;
    }

    if (!httpRequireAuthorized()) return;
    web.send(404, "text/plain", "Not found");
  });
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
    bool newEnabled = parseBool(p, systemEnabled);
    if (newEnabled != systemEnabled) {
      systemEnabled = newEnabled;
      if (!systemEnabled) allRelaysOff();
      saveRuntimeState();
    } else if (!newEnabled) {
      allRelaysOff();
    }
    mqttPublishState(true);
    return;
  }

  // Set press duration via MQTT / Home Assistant Number
  {
    const String cmdPressMs = topicOf("cmd/press_ms");
    if (t == cmdPressMs) {
      uint32_t newPress = (uint32_t)p.toInt();
      newPress = clampPressMs(newPress);
      if (newPress != config.pressMs) {
        config.pressMs = newPress;
        savePressMsOnly();
        logf(LOG_INFO, "[MQTT] press_ms=%lu", (unsigned long)config.pressMs);
      }
      if (mqtt.connected()) mqttPublishState(true);
      return;
    }
  }

  for (uint8_t i = 1; i <= config.relayCount && i <= RELAY_MAX; i++) {
    const char *ti = getTopicButtonIn(i);
    if (ti[0] == '\0') continue;
    if (t == String(ti)) {
      if (parseBool(p, false)) {
        requestPress(i, "mqtt");
        mqttPublishState(true);
      }
      return;
    }
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

  mqtt.publish(willTopic.c_str(), "1", true);

  mqttCleanupLegacyGap();

  if (strlen(config.topicEnableIn) > 0) mqtt.subscribe(config.topicEnableIn);
  mqtt.subscribe(topicOf("cmd/press_ms").c_str());
  for (uint8_t i = 1; i <= config.relayCount && i <= RELAY_MAX; i++) {
    const char *ti = getTopicButtonIn(i);
    if (ti[0] != '\0') mqtt.subscribe(ti);
  }

  logf(LOG_INFO, "[MQTT] Connected. sub en='%s' press_ms='%s' relayCount=%u", config.topicEnableIn, topicOf("cmd/press_ms").c_str(),
    (unsigned)config.relayCount);

  mqttPublishDiscovery();
  mqttPublishState(true);
  mqttPublishReedState(true);

  return true;
}

// Arduino-ESP32 calls initVariant() before setup().
// Drive both relay GPIOs to a safe OFF state as early as possible.
extern "C" void initVariant() {
  const int pins[4] = {BTN_DEFAULT_RELAY1_PIN, BTN_DEFAULT_RELAY2_PIN, BTN_DEFAULT_RELAY3_PIN, BTN_DEFAULT_RELAY4_PIN};
  for (int i = 0; i < 4; i++) {
    const int pin = pins[i];
    if (!isValidGpio(pin)) continue;

    const bool inverted = (BTN_DEFAULT_RELAY_INVERTED != 0);
    bool level = false; // OFF
    if (inverted) level = !level;

    const gpio_num_t gpio = (gpio_num_t)pin;
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, level ? 1 : 0);
  }
}

static void driveBootPinsSafe(int &bootR1, int &bootR2, int &bootR3, int &bootR4, bool &bootInv) {
  bootR1 = BTN_DEFAULT_RELAY1_PIN;
  bootR2 = BTN_DEFAULT_RELAY2_PIN;
  bootR3 = BTN_DEFAULT_RELAY3_PIN;
  bootR4 = BTN_DEFAULT_RELAY4_PIN;
  bootInv = (BTN_DEFAULT_RELAY_INVERTED != 0);

  Preferences bootPrefs;
  if (bootPrefs.begin("btn", true)) {
    bootR1 = bootPrefs.getInt("r1Pin", BTN_DEFAULT_RELAY1_PIN);
    bootR2 = bootPrefs.getInt("r2Pin", BTN_DEFAULT_RELAY2_PIN);
    bootR3 = bootPrefs.getInt("r3Pin", BTN_DEFAULT_RELAY3_PIN);
    bootR4 = bootPrefs.getInt("r4Pin", BTN_DEFAULT_RELAY4_PIN);
    bootInv = bootPrefs.getBool("rInv", BTN_DEFAULT_RELAY_INVERTED != 0);
    bootPrefs.end();
  }

  auto safeOff = [&](int pin) {
    if (!isValidGpio(pin)) return;
    pinMode(pin, OUTPUT);
    bool level = false;
    if (bootInv) level = !level;
    digitalWrite(pin, level ? HIGH : LOW);
  };

  safeOff(bootR1);
  safeOff(bootR2);
  safeOff(bootR3);
  safeOff(bootR4);
}

void setup() {
  int bootR1, bootR2, bootR3, bootR4;
  bool bootInv;
  driveBootPinsSafe(bootR1, bootR2, bootR3, bootR4, bootInv);

  Serial.begin(115200);
  delay(50);

  loadConfig();

  reedApplyPinMode();

  // If user config uses different pins than boot pins, release the boot pins.
  auto releaseIfDifferent = [&](int bootPin, int cfgPin) {
    if (bootPin != cfgPin && isValidGpio(bootPin)) {
      pinMode(bootPin, INPUT);
    }
  };
  releaseIfDifferent(bootR1, config.relay1Pin);
  releaseIfDifferent(bootR2, config.relay2Pin);
  releaseIfDifferent(bootR3, config.relay3Pin);
  releaseIfDifferent(bootR4, config.relay4Pin);

  if (isValidGpio(config.relay1Pin)) pinMode(config.relay1Pin, OUTPUT);
  if (isValidGpio(config.relay2Pin)) pinMode(config.relay2Pin, OUTPUT);
  if (isValidGpio(config.relay3Pin)) pinMode(config.relay3Pin, OUTPUT);
  if (isValidGpio(config.relay4Pin)) pinMode(config.relay4Pin, OUTPUT);

  allRelaysOff();

  logf(LOG_INFO, "Device: %s", deviceId().c_str());
  logf(LOG_INFO, "Relay pins: r1=%d r2=%d r3=%d r4=%d count=%u inverted=%s", config.relay1Pin, config.relay2Pin, config.relay3Pin,
    config.relay4Pin, (unsigned)config.relayCount, config.relayInverted ? "yes" : "no");
  logf(LOG_INFO, "Press ms: %lu", (unsigned long)config.pressMs);
  logf(LOG_INFO, "Reed: enabled=%s pin=%d type=%s", config.reedEnabled ? "yes" : "no", config.reedPin, config.reedNc ? "NC" : "NO");

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
    allRelaysOff();
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
    allRelaysOff();
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

  pressLoopTick();

  // Reed switch polling with debounce
  {
    const uint32_t now = millis();
    if (reedConfigured() && (now - reedLastSampleMs) >= 20U) {
      reedLastSampleMs = now;
      const bool v = reedReadDoorClosedRaw();
      if (!reedHasStable) {
        reedLastSampleValue = v;
        reedStableSamples = 1;
        reedDoorClosed = v;
        reedHasStable = true;
        if (mqtt.connected()) mqttPublishReedState(true);
      } else {
        if (v == reedLastSampleValue) {
          if (reedStableSamples < 10) reedStableSamples++;
        } else {
          reedLastSampleValue = v;
          reedStableSamples = 1;
        }
        if (reedStableSamples >= 3 && v != reedDoorClosed) {
          reedDoorClosed = v;
          if (mqtt.connected()) mqttPublishReedState(true);
          if (config.logLevel >= LOG_INFO) {
            logf(LOG_INFO, "[REED] %s", reedDoorClosed ? "CLOSED" : "OPEN");
          }
        }
      }
    }

    // If reed is disabled: clear retained once and stop publishing
    if (!reedConfigured() && mqtt.connected()) {
      mqttPublishReedState(false);
    }
  }

  static uint32_t last1sTickMs = 0;
  uint32_t now = millis();
  if ((now - last1sTickMs) >= 1000U) {
    last1sTickMs = now;

    if (mqtt.connected()) {
      mqttPublishState(false);
      mqttPublishReedState(false);
    }

    // Anti-hang watchdog (only MQTT connectivity)
    if (wifiStatus == WL_CONNECTED && config.hangTimeoutSec > 0) {
      uint32_t timeoutMs = config.hangTimeoutSec * 1000U;
      bool hang = false;
      const char *hangWhy = nullptr;

      if (!mqtt.connected()) {
        if (mqttDisconnectedSinceMs > 0 && (now - mqttDisconnectedSinceMs) > timeoutMs) {
          hang = true;
          hangWhy = "mqtt_disconnected";
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
