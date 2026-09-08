
#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <LovyanGFX.hpp>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <math.h>

// ======================================================================
// Hardware profile
// Build flags in platformio.ini select exactly one supported platform.
// ======================================================================

#if defined(WIFIWATCH_HW_WT32)

  static constexpr const char* PLATFORM_NAME  = "WT32-ETH01";
  static constexpr const char* PLATFORM_SHORT = "WT32";
  static constexpr const char* DEFAULT_HOSTNAME = "wifiwatch";
  static constexpr const char* DEFAULT_MQTT_TOPIC = "wifiwatch";

  // LAN8720 RMII
  #define ETH_PHY_TYPE  ETH_PHY_LAN8720
  #define ETH_PHY_ADDR  1
  #define ETH_PHY_MDC   23
  #define ETH_PHY_MDIO  18
  #define ETH_PHY_POWER 16
  #define ETH_CLK_MODE  ETH_CLOCK_GPIO0_IN

  // Reference ST7789 wiring
  static constexpr int TFT_SCLK = 32;
  static constexpr int TFT_MOSI = 33;
  static constexpr int TFT_CS   = 17;
  static constexpr int TFT_DC   = 14;
  static constexpr int TFT_RST  = 4;
  #define TFT_SPI_HOST VSPI_HOST

#elif defined(WIFIWATCH_HW_WAVESHARE_S3_ETH)

  static constexpr const char* PLATFORM_NAME  = "Waveshare ESP32-S3-ETH";
  static constexpr const char* PLATFORM_SHORT = "Waveshare-S3-ETH";
  static constexpr const char* DEFAULT_HOSTNAME = "wifiwatch-waveshare";
  static constexpr const char* DEFAULT_MQTT_TOPIC = "wifiwatch-waveshare";

  // Onboard W5500 SPI Ethernet.
  // Waveshare schematic / board wiring:
  // RST=GPIO9, INT=GPIO10, MOSI=GPIO11, MISO=GPIO12,
  // SCLK=GPIO13, CS=GPIO14.
  #define ETH_PHY_TYPE      ETH_PHY_W5500
  #define ETH_PHY_ADDR      1
  static constexpr int ETH_PHY_RST_PIN  = 9;
  static constexpr int ETH_PHY_IRQ_PIN  = 10;
  static constexpr int ETH_PHY_MOSI_PIN = 11;
  static constexpr int ETH_PHY_MISO_PIN = 12;
  static constexpr int ETH_PHY_SCLK_PIN = 13;
  static constexpr int ETH_PHY_CS_PIN   = 14;

  // Reference ST7789 wiring on the normal header.
  // These pins are separate from the onboard W5500 SPI bus.
  static constexpr int TFT_RST  = 38;
  static constexpr int TFT_SCLK = 39;
  static constexpr int TFT_MOSI = 40;
  static constexpr int TFT_CS   = 41;
  static constexpr int TFT_DC   = 42;
  #define TFT_SPI_HOST SPI3_HOST

#else
  #error "Select WIFIWATCH_HW_WT32 or WIFIWATCH_HW_WAVESHARE_S3_ETH"
#endif

// ---------- Detection defaults ----------
static constexpr uint16_t DEFAULT_SCAN_INTERVAL_SEC = 30;
static constexpr uint16_t MIN_SCAN_INTERVAL_SEC = 15;
static constexpr uint16_t MAX_SCAN_INTERVAL_SEC = 300;

static constexpr uint32_t LEARN_WINDOW_SECONDS = 7200;      // ~2 h history
static constexpr uint32_t MIN_LEARNING_SECONDS = 600;       // ~10 min initial learning
static constexpr float STABLE_SEEN_RATIO = 0.70f;
static constexpr int STRONG_RSSI = -80;
static constexpr size_t SHORT_HISTORY_LEN = 5;
static constexpr size_t MAX_IGNORE = 40;
static constexpr size_t MAX_APS = 80;

// Persistent baseline
static constexpr uint16_t BASELINE_FORMAT_VERSION = 1;
static constexpr uint32_t BASELINE_SAVE_INTERVAL_MS = 30UL * 60UL * 1000UL;
static constexpr uint32_t BOOT_VALIDATION_MS = 5UL * 60UL * 1000UL;

// Composite outage score:
// 50% long-term stable-BSSID loss
// 30% abrupt drop vs. preceding short-term scans
// 20% loss of strong/stable BSSIDs
static constexpr float WEIGHT_BASELINE = 0.50f;
static constexpr float WEIGHT_DYNAMIC  = 0.30f;
static constexpr float WEIGHT_QUALITY  = 0.20f;

// ---------- Debug ----------
// 0 = serial diagnostics off, 1 = on
#define WIFIWATCH_DEBUG 0

#if WIFIWATCH_DEBUG
  #define DBG_BEGIN(...) Serial.begin(__VA_ARGS__)
  #define DBG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(...)
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 panel;
  lgfx::Bus_SPI bus;
public:
  LGFX() {
    auto c = bus.config();
    c.spi_host = TFT_SPI_HOST;
    c.spi_mode = 0;
    c.freq_write = 40000000;
    c.freq_read = 16000000;
    c.spi_3wire = true;
    c.use_lock = true;
    c.dma_channel = 1;
    c.pin_sclk = TFT_SCLK;
    c.pin_mosi = TFT_MOSI;
    c.pin_miso = -1;
    c.pin_dc = TFT_DC;
    bus.config(c);
    panel.setBus(&bus);

    auto q = panel.config();
    q.pin_cs = TFT_CS;
    q.pin_rst = TFT_RST;
    q.pin_busy = -1;
    q.panel_width = 240;
    q.panel_height = 240;
    q.memory_width = 240;
    q.memory_height = 240;
    q.readable = false;
    q.invert = true;
    q.rgb_order = false;
    q.bus_shared = false;
    panel.config(q);
    setPanel(&panel);
  }
};

LGFX display;
WebServer server(80);
WiFiClient netClient;
PubSubClient mqtt(netClient);
Preferences prefs;

struct CFG {
  String host;
  String user;
  String pass;
  String topic = DEFAULT_MQTT_TOPIC;
  uint16_t port = 1883;
  bool ha = true;
  String webUser = "admin";
  String webPass = "wifiwatch";
  uint16_t scanIntervalSec = DEFAULT_SCAN_INTERVAL_SEC;
  String language = "de";
  String hostname = DEFAULT_HOSTNAME;
  String alarmThreshold = "likely";
} cfg;

struct AP {
  String ssid;
  String bssid;
  int rssi;
  int ch;
  bool ignS;
  bool ignB;
};

struct Learned {
  float seen = 0.0f;
  float rssiSum = 0.0f;
  int lastRssi = -100;
  String ssid;
};

struct PersistEntry {
  uint8_t mac[6];
  float seen;
  float rssiSum;
  int16_t lastRssi;
  char ssid[33];
};

std::vector<AP> aps;
std::vector<String> ignSSIDs;
std::vector<String> ignBSSIDs;
std::vector<String> watchSSIDs;
std::vector<String> watchBSSIDs;
std::map<String, Learned> learned;
std::vector<float> recentSurvival;

bool eth = false;
bool lastMqttConnected = false;
uint32_t lastScan = 0;
uint32_t lastDur = 0;
uint32_t lastMq = 0;
float learnedScans = 0.0f;

bool baselineLoaded = false;
bool baselineEverSaved = false;
bool baselineDirty = false;
bool bootValidationActive = false;
uint32_t bootValidationStartedMs = 0;
uint32_t lastBaselineSaveMs = 0;
uint16_t persistedBaselineEntries = 0;

int total = 0;
int ignored = 0;
int neighbors = 0;
int uniqueSSIDs = 0;
int hidden = 0;
int channels[14] = {0};

int stableExpected = 0;
int stableVisible = 0;
int outageScore = 0;
int baselineLossScore = 0;
int dynamicDropScore = 0;
int qualityLossScore = 0;
float shortTermAverageSurvival = 1.0f;

bool alarmFlag = false;

// Own-WiFi watch status
int watchExpected = 0;
int watchVisible = 0;
int watchMissing = 0;
bool wifiAlarm = false;
uint8_t watchMissingStreak = 0;
uint8_t watchHealthyStreak = 0;

enum class DisplayState {
  Learning,
  VeryUnlikely,
  Unlikely,
  Possible,
  Likely,
  VeryLikely
};

DisplayState displayState = DisplayState::Learning;

// Neighborhood alarm confirmation.
// Level/score react immediately; the binary alarm requires two consecutive
// likely/very_likely scans. Clearing is also confirmed with two healthy scans.
uint8_t neighborhoodAlarmStreak = 0;
uint8_t neighborhoodHealthyStreak = 0;

// ---------- forward declarations ----------
void draw();
void scan();
void discovery();
void publishAll();
void saveBaseline();
void clearPersistentBaseline();
void startRelearn(bool hardReset = false);

// ---------- helpers ----------
String esc(String s) {
  s.replace("&","&amp;");
  s.replace("<","&lt;");
  s.replace(">","&gt;");
  s.replace("\"","&quot;");
  s.replace("'","&#39;");
  return s;
}

bool exact(const std::vector<String>& v, const String& s) {
  for (const auto& x : v) if (x == s) return true;
  return false;
}

bool ci(const std::vector<String>& v, const String& s) {
  for (const auto& x : v) if (x.equalsIgnoreCase(s)) return true;
  return false;
}

void parseLines(const String& s, std::vector<String>& v) {
  v.clear();
  int a = 0;
  while (a < (int)s.length()) {
    int e = s.indexOf('\n', a);
    if (e < 0) e = s.length();
    String x = s.substring(a, e);
    x.trim();
    if (x.length() && v.size() < MAX_IGNORE) v.push_back(x);
    a = e + 1;
  }
}

String joinLines(const std::vector<String>& v) {
  String s;
  for (const auto& x : v) {
    if (s.length()) s += '\n';
    s += x;
  }
  return s;
}

String devId() {
  char b[24];
  snprintf(b, sizeof(b), "wifiwatch_%08X",
           (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF));
  return String(b);
}

String baseTopic() {
  String t = cfg.topic;
  if (t.endsWith("/")) t.remove(t.length() - 1);
  return t;
}

String topicSystem(const String& s) {
  return baseTopic() + "/system/" + s;
}

String topicNeighborhood(const String& s) {
  return baseTopic() + "/neighborhood/" + s;
}

String topicWiFi(const String& s) {
  return baseTopic() + "/wifi/" + s;
}

String topicControl(const String& s) {
  return baseTopic() + "/control/" + s;
}

uint32_t scanIntervalMs() {
  return (uint32_t)cfg.scanIntervalSec * 1000UL;
}

uint32_t minLearningScans() {
  uint16_t sec = cfg.scanIntervalSec ? cfg.scanIntervalSec : 1;
  uint32_t n = MIN_LEARNING_SECONDS / sec;
  return (n < 5) ? 5 : n;
}

uint32_t targetLearnWindowScans() {
  uint16_t sec = cfg.scanIntervalSec ? cfg.scanIntervalSec : 1;
  uint32_t n = LEARN_WINDOW_SECONDS / sec;
  return (n < 20) ? 20 : n;
}

bool learnedEntryIgnored(const Learned& l, const String& bssid) {
  if (ci(ignBSSIDs, bssid)) return true;
  if (ci(watchBSSIDs, bssid)) return true;
  if (l.ssid.length() && exact(ignSSIDs, l.ssid)) return true;
  if (l.ssid.length() && exact(watchSSIDs, l.ssid)) return true;
  return false;
}

void loadCfg() {
  prefs.begin("wifiwatch", true);
  cfg.host = prefs.getString("mq_host", "");
  cfg.port = prefs.getUShort("mq_port", 1883);
  cfg.user = prefs.getString("mq_user", "");
  cfg.pass = prefs.getString("mq_pass", "");
  cfg.topic = prefs.getString("mq_topic", DEFAULT_MQTT_TOPIC);
  cfg.ha = prefs.getBool("ha_disc", true);
  cfg.webUser = prefs.getString("web_user", "admin");
  cfg.webPass = prefs.getString("web_pass", "wifiwatch");
  cfg.scanIntervalSec = prefs.getUShort("scan_sec", DEFAULT_SCAN_INTERVAL_SEC);
  cfg.language = prefs.getString("language", "de");
  if (cfg.language != "de" && cfg.language != "en") cfg.language = "de";
  cfg.hostname = prefs.getString("hostname", DEFAULT_HOSTNAME);
  if (!cfg.hostname.length()) cfg.hostname = DEFAULT_HOSTNAME;
  cfg.alarmThreshold = prefs.getString("alarm_thr", "likely");
  if (cfg.alarmThreshold != "very_unlikely" &&
      cfg.alarmThreshold != "unlikely" &&
      cfg.alarmThreshold != "possible" &&
      cfg.alarmThreshold != "likely" &&
      cfg.alarmThreshold != "very_likely") {
    cfg.alarmThreshold = "likely";
  }
  parseLines(prefs.getString("ign_ssid", ""), ignSSIDs);
  parseLines(prefs.getString("ign_bssid", ""), ignBSSIDs);
  parseLines(prefs.getString("watch_ssid", ""), watchSSIDs);
  parseLines(prefs.getString("watch_bssid", ""), watchBSSIDs);
  prefs.end();

  cfg.scanIntervalSec = constrain(cfg.scanIntervalSec,
                                  MIN_SCAN_INTERVAL_SEC,
                                  MAX_SCAN_INTERVAL_SEC);
}

void saveCfg() {
  prefs.begin("wifiwatch", false);
  prefs.putString("mq_host", cfg.host);
  prefs.putUShort("mq_port", cfg.port);
  prefs.putString("mq_user", cfg.user);
  prefs.putString("mq_pass", cfg.pass);
  prefs.putString("mq_topic", cfg.topic);
  prefs.putBool("ha_disc", cfg.ha);
  prefs.putString("web_user", cfg.webUser);
  prefs.putString("web_pass", cfg.webPass);
  prefs.putUShort("scan_sec", cfg.scanIntervalSec);
  prefs.putString("language", cfg.language);
  prefs.putString("hostname", cfg.hostname);
  prefs.putString("alarm_thr", cfg.alarmThreshold);
  prefs.putString("ign_ssid", joinLines(ignSSIDs));
  prefs.putString("ign_bssid", joinLines(ignBSSIDs));
  prefs.putString("watch_ssid", joinLines(watchSSIDs));
  prefs.putString("watch_bssid", joinLines(watchBSSIDs));
  prefs.end();
}

bool requireAuth() {
  if (server.authenticate(cfg.webUser.c_str(), cfg.webPass.c_str())) return true;
  server.requestAuthentication(BASIC_AUTH, "WiFiWatch", "Anmeldung erforderlich");
  return false;
}



String sanitizeHostname(String s) {
  s.toLowerCase();
  String out;
  bool lastDash = false;

  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    bool ok = (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') ||
              c == '-';

    if (!ok) continue;
    if (c == '-') {
      if (!out.length() || lastDash) continue;
      lastDash = true;
    } else {
      lastDash = false;
    }

    if (out.length() < 63) out += c;
  }

  while (out.endsWith("-")) out.remove(out.length() - 1);
  if (!out.length()) out = DEFAULT_HOSTNAME;
  return out;
}

String mdnsHost() {
  return sanitizeHostname(cfg.hostname) + ".local";
}

bool mdnsStarted = false;

void startMDNS() {
  if (mdnsStarted) MDNS.end();

  cfg.hostname = sanitizeHostname(cfg.hostname);

  if (MDNS.begin(cfg.hostname.c_str())) {
    mdnsStarted = true;
    MDNS.addService("http", "tcp", 80);
    DBG_PRINTF("[mDNS] http://%s.local/\n", cfg.hostname.c_str());
  } else {
    mdnsStarted = false;
    DBG_PRINTLN("[mDNS] start failed");
  }
}

bool isGerman() {
  return cfg.language == "de";
}

String mqttOutageLevel() {
  switch (displayState) {
    case DisplayState::Learning:      return "learning";
    case DisplayState::VeryUnlikely:  return "very_unlikely";
    case DisplayState::Unlikely:      return "unlikely";
    case DisplayState::Possible:      return "possible";
    case DisplayState::Likely:        return "likely";
    case DisplayState::VeryLikely:    return "very_likely";
  }
  return "learning";
}

String localizedOutageLevel() {
  if (isGerman()) {
    switch (displayState) {
      case DisplayState::Learning:      return "Lernphase";
      case DisplayState::VeryUnlikely:  return "Sehr unwahrscheinlich";
      case DisplayState::Unlikely:      return "Unwahrscheinlich";
      case DisplayState::Possible:      return "Möglich";
      case DisplayState::Likely:        return "Wahrscheinlich";
      case DisplayState::VeryLikely:    return "Sehr wahrscheinlich";
    }
  } else {
    switch (displayState) {
      case DisplayState::Learning:      return "Learning";
      case DisplayState::VeryUnlikely:  return "Very unlikely";
      case DisplayState::Unlikely:      return "Unlikely";
      case DisplayState::Possible:      return "Possible";
      case DisplayState::Likely:        return "Likely";
      case DisplayState::VeryLikely:    return "Very likely";
    }
  }
  return isGerman() ? "Lernphase" : "Learning";
}

int outageLevelRank(const String& level) {
  if (level == "very_unlikely") return 0;
  if (level == "unlikely") return 1;
  if (level == "possible") return 2;
  if (level == "likely") return 3;
  if (level == "very_likely") return 4;
  return 3; // default: likely
}

int displayStateRank() {
  switch (displayState) {
    case DisplayState::VeryUnlikely: return 0;
    case DisplayState::Unlikely:     return 1;
    case DisplayState::Possible:     return 2;
    case DisplayState::Likely:       return 3;
    case DisplayState::VeryLikely:   return 4;
    case DisplayState::Learning:     return -1;
  }
  return -1;
}

String localizedProbabilityName(const String& raw) {
  if (isGerman()) {
    if (raw == "very_unlikely") return "Sehr unwahrscheinlich";
    if (raw == "unlikely")      return "Unwahrscheinlich";
    if (raw == "possible")      return "Möglich";
    if (raw == "likely")        return "Wahrscheinlich";
    if (raw == "very_likely")   return "Sehr wahrscheinlich";
  } else {
    if (raw == "very_unlikely") return "Very unlikely";
    if (raw == "unlikely")      return "Unlikely";
    if (raw == "possible")      return "Possible";
    if (raw == "likely")        return "Likely";
    if (raw == "very_likely")   return "Very likely";
  }
  return raw;
}

String alarmThresholdDiscoveryExtra() {
  if (isGerman()) {
    return "\"options\":[\"Sehr unwahrscheinlich\",\"Unwahrscheinlich\","
           "\"Möglich\",\"Wahrscheinlich\",\"Sehr wahrscheinlich\"],"
           "\"value_template\":\"{% set m = {'very_unlikely':'Sehr unwahrscheinlich',"
           "'unlikely':'Unwahrscheinlich','possible':'Möglich','likely':'Wahrscheinlich',"
           "'very_likely':'Sehr wahrscheinlich'} %}{{ m.get(value, value) }}\","
           "\"command_template\":\"{% set m = {'Sehr unwahrscheinlich':'very_unlikely',"
           "'Unwahrscheinlich':'unlikely','Möglich':'possible','Wahrscheinlich':'likely',"
           "'Sehr wahrscheinlich':'very_likely'} %}{{ m.get(value, value) }}\","
           "\"icon\":\"mdi:alarm-light\"";
  }

  return "\"options\":[\"Very unlikely\",\"Unlikely\",\"Possible\",\"Likely\",\"Very likely\"],"
         "\"value_template\":\"{% set m = {'very_unlikely':'Very unlikely',"
         "'unlikely':'Unlikely','possible':'Possible','likely':'Likely',"
         "'very_likely':'Very likely'} %}{{ m.get(value, value) }}\","
         "\"command_template\":\"{% set m = {'Very unlikely':'very_unlikely',"
         "'Unlikely':'unlikely','Possible':'possible','Likely':'likely',"
         "'Very likely':'very_likely'} %}{{ m.get(value, value) }}\","
         "\"icon\":\"mdi:alarm-light\"";
}

String t(const char* de, const char* en) {
  return isGerman() ? String(de) : String(en);
}

String outageLevelDiscoveryExtra() {
  if (isGerman()) {
    return "\"icon\":\"mdi:transmission-tower-off\","
           "\"value_template\":\"{% set m = {'learning':'Lernphase',"
           "'very_unlikely':'Sehr unwahrscheinlich','unlikely':'Unwahrscheinlich',"
           "'possible':'Möglich','likely':'Wahrscheinlich',"
           "'very_likely':'Sehr wahrscheinlich'} %}{{ m.get(value, value) }}\"";
  }

  return "\"icon\":\"mdi:transmission-tower-off\","
         "\"value_template\":\"{% set m = {'learning':'Learning',"
         "'very_unlikely':'Very unlikely','unlikely':'Unlikely',"
         "'possible':'Possible','likely':'Likely',"
         "'very_likely':'Very likely'} %}{{ m.get(value, value) }}\"";
}

String baselineStatusDiscoveryExtra() {
  if (isGerman()) {
    return "\"icon\":\"mdi:database-check\","
           "\"value_template\":\"{% set m = {'learning':'Lernphase',"
           "'validating':'Validierung','valid':'Gültig'} %}{{ m.get(value, value) }}\"";
  }

  return "\"icon\":\"mdi:database-check\","
         "\"value_template\":\"{% set m = {'learning':'Learning',"
         "'validating':'Validation','valid':'Valid'} %}{{ m.get(value, value) }}\"";
}

String baselineStatusCode() {
  if (displayState == DisplayState::Learning)
    return "learning";

  if (bootValidationActive) {
    if (millis() - bootValidationStartedMs < BOOT_VALIDATION_MS)
      return "validating";
    bootValidationActive = false;
  }

  return "valid";
}

String localizedBaselineStatus() {
  String s = baselineStatusCode();
  if (s == "learning") return t("Lernphase","Learning");
  if (s == "validating") return t("Validierung","Validation");
  return t("Gültig","Valid");
}

bool parseBSSID(const String& s, uint8_t mac[6]) {
  unsigned int b[6];
  if (sscanf(s.c_str(), "%02X:%02X:%02X:%02X:%02X:%02X",
             &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
    return false;

  for (int i=0; i<6; i++) mac[i] = (uint8_t)b[i];
  return true;
}

String formatBSSID(const uint8_t mac[6]) {
  char b[18];
  snprintf(b, sizeof(b), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(b);
}

void loadBaseline() {
  learned.clear();
  learnedScans = 0.0f;
  baselineLoaded = false;
  baselineEverSaved = false;
  persistedBaselineEntries = 0;

  Preferences bp;
  if (!bp.begin("wwbaseline", true))
    return;

  uint16_t ver = bp.getUShort("ver", 0);
  uint16_t count = bp.getUShort("count", 0);
  float scans = bp.getFloat("scans", 0.0f);
  size_t bytes = bp.getBytesLength("entries");

  if (ver != BASELINE_FORMAT_VERSION ||
      count == 0 ||
      count > MAX_APS ||
      scans < 1.0f ||
      bytes != ((size_t)count * sizeof(PersistEntry))) {
    bp.end();
    return;
  }

  std::vector<PersistEntry> entries(count);
  size_t got = bp.getBytes("entries", entries.data(), bytes);
  bp.end();

  if (got != bytes)
    return;

  for (auto& p : entries) {
    p.ssid[sizeof(p.ssid)-1] = '\0';

    Learned l;
    l.seen = p.seen;
    l.rssiSum = p.rssiSum;
    l.lastRssi = p.lastRssi;
    l.ssid = String(p.ssid);

    if (l.seen > 0.01f)
      learned[formatBSSID(p.mac)] = l;
  }

  if (!learned.empty()) {
    learnedScans = scans;
    baselineLoaded = true;
    baselineEverSaved = true;
    persistedBaselineEntries = (uint16_t)learned.size();
    baselineDirty = false;
    bootValidationActive = true;
    bootValidationStartedMs = millis();

    DBG_PRINTF("[BASELINE] loaded: %u entries, %.1f scans\n",
               persistedBaselineEntries, learnedScans);
  }
}

void saveBaseline() {
  if (learned.empty() || learnedScans < 1.0f)
    return;

  std::vector<PersistEntry> entries;
  entries.reserve(MAX_APS);

  for (const auto& kv : learned) {
    if (entries.size() >= MAX_APS)
      break;

    if (kv.second.seen <= 0.01f)
      continue;

    PersistEntry p{};
    if (!parseBSSID(kv.first, p.mac))
      continue;

    p.seen = kv.second.seen;
    p.rssiSum = kv.second.rssiSum;
    p.lastRssi = (int16_t)kv.second.lastRssi;
    strncpy(p.ssid, kv.second.ssid.c_str(), sizeof(p.ssid)-1);
    p.ssid[sizeof(p.ssid)-1] = '\0';

    entries.push_back(p);
  }

  if (entries.empty())
    return;

  Preferences bp;
  if (!bp.begin("wwbaseline", false))
    return;

  bp.putUShort("ver", BASELINE_FORMAT_VERSION);
  bp.putUShort("count", (uint16_t)entries.size());
  bp.putFloat("scans", learnedScans);

  size_t expected = entries.size() * sizeof(PersistEntry);
  size_t written = bp.putBytes("entries", entries.data(), expected);
  bp.end();

  if (written == expected) {
    baselineDirty = false;
    baselineEverSaved = true;
    baselineLoaded = true;
    persistedBaselineEntries = (uint16_t)entries.size();
    lastBaselineSaveMs = millis();

    DBG_PRINTF("[BASELINE] saved: %u entries, %.1f scans\n",
               persistedBaselineEntries, learnedScans);
  }
}

void clearPersistentBaseline() {
  Preferences bp;
  if (bp.begin("wwbaseline", false)) {
    bp.clear();
    bp.end();
  }

  persistedBaselineEntries = 0;
  baselineEverSaved = false;
  baselineLoaded = false;
}

void resetBaselineRuntime() {
  learned.clear();
  learnedScans = 0.0f;
  recentSurvival.clear();

  stableExpected = 0;
  stableVisible = 0;
  outageScore = 0;
  baselineLossScore = 0;
  dynamicDropScore = 0;
  qualityLossScore = 0;
  shortTermAverageSurvival = 1.0f;

  displayState = DisplayState::Learning;
  alarmFlag = false;
  neighborhoodAlarmStreak = 0;
  neighborhoodHealthyStreak = 0;

  bootValidationActive = false;
  baselineDirty = false;
}

void startRelearn(bool hardReset) {
  // Both controls deliberately remove the old persistent model and start
  // a new learning phase. "Reset" is the explicit destructive wording.
  (void)hardReset;

  clearPersistentBaseline();
  resetBaselineRuntime();

  draw();
  publishAll();
}

void maybeSaveBaseline() {
  if (!baselineDirty)
    return;

  // During first learning, save immediately once enough scans exist.
  if (displayState == DisplayState::Learning &&
      learnedScans < (float)minLearningScans())
    return;

  // Let a loaded baseline validate for five minutes before overwriting it.
  if (bootValidationActive &&
      millis() - bootValidationStartedMs < BOOT_VALIDATION_MS)
    return;

  if (!baselineEverSaved ||
      millis() - lastBaselineSaveMs >= BASELINE_SAVE_INTERVAL_MS)
    saveBaseline();
}


bool isWatchedSSID(const String& ssid) {
  return ssid.length() && exact(watchSSIDs, ssid);
}

bool isWatchedBSSID(const String& bssid) {
  return ci(watchBSSIDs, bssid);
}

// ---------- MQTT / HA ----------
void pubTopic(const String& fullTopic, const String& value) {
  if (mqtt.connected()) mqtt.publish(fullTopic.c_str(), value.c_str(), true);
}

void discoveryPublish(const String& component,
                      const String& objectId,
                      const String& name,
                      const String& stateTopic,
                      const String& extra = "") {
  if (!mqtt.connected() || !cfg.ha) return;

  String uid = devId() + "_" + objectId;
  String discoveryTopic = "homeassistant/" + component + "/" + uid + "/config";

  String payload = "{\"name\":\"" + name + "\",\"unique_id\":\"" + uid + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"availability_topic\":\"" + topicSystem("status") + "\",";
  payload += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
  payload += "\"device\":{\"identifiers\":[\"" + devId() + "\"],";
  payload += "\"name\":\"WiFiWatch\",\"manufacturer\":\"DIY\",";
  payload += "\"model\":\"" + String(PLATFORM_NAME) + "\",\"sw_version\":\"0.4.8\",";
  payload += "\"configuration_url\":\"http://" + mdnsHost() + "/\"}";
  if (extra.length()) payload += "," + extra;
  payload += "}";

  mqtt.publish(discoveryTopic.c_str(), payload.c_str(), true);
}

void discoveryCommandOnly(const String& component,
                          const String& objectId,
                          const String& name,
                          const String& commandTopic,
                          const String& extra = "") {
  if (!mqtt.connected() || !cfg.ha) return;

  String uid = devId() + "_" + objectId;
  String discoveryTopic = "homeassistant/" + component + "/" + uid + "/config";

  String payload = "{\"name\":\"" + name + "\",\"unique_id\":\"" + uid + "\",";
  payload += "\"command_topic\":\"" + commandTopic + "\",";
  payload += "\"availability_topic\":\"" + topicSystem("status") + "\",";
  payload += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
  payload += "\"device\":{\"identifiers\":[\"" + devId() + "\"],";
  payload += "\"name\":\"WiFiWatch\",\"manufacturer\":\"DIY\",";
  payload += "\"model\":\"" + String(PLATFORM_NAME) + "\",\"sw_version\":\"0.4.8\",";
  payload += "\"configuration_url\":\"http://" + mdnsHost() + "/\"}";
  if (extra.length()) payload += "," + extra;
  payload += "}";

  mqtt.publish(discoveryTopic.c_str(), payload.c_str(), true);
}

void discoveryNumber(const String& objectId,
                     const String& name,
                     const String& stateTopic,
                     const String& commandTopic,
                     int minValue,
                     int maxValue,
                     int step) {
  if (!mqtt.connected() || !cfg.ha) return;

  String uid = devId() + "_" + objectId;
  String discoveryTopic = "homeassistant/number/" + uid + "/config";

  String payload = "{\"name\":\"" + name + "\",\"unique_id\":\"" + uid + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"command_topic\":\"" + commandTopic + "\",";
  payload += "\"min\":" + String(minValue) + ",";
  payload += "\"max\":" + String(maxValue) + ",";
  payload += "\"step\":" + String(step) + ",";
  payload += "\"mode\":\"box\",";
  payload += "\"unit_of_measurement\":\"s\",";
  payload += "\"availability_topic\":\"" + topicSystem("status") + "\",";
  payload += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
  payload += "\"device\":{\"identifiers\":[\"" + devId() + "\"],";
  payload += "\"name\":\"WiFiWatch\",\"manufacturer\":\"DIY\",";
  payload += "\"model\":\"" + String(PLATFORM_NAME) + "\",\"sw_version\":\"0.4.8\",";
  payload += "\"configuration_url\":\"http://" + mdnsHost() + "/\"}}";

  mqtt.publish(discoveryTopic.c_str(), payload.c_str(), true);
}

void discoverySelect(const String& objectId,
                     const String& name,
                     const String& stateTopic,
                     const String& commandTopic,
                     const String& extra) {
  if (!mqtt.connected() || !cfg.ha) return;

  String uid = devId() + "_" + objectId;
  String discoveryTopic = "homeassistant/select/" + uid + "/config";

  String payload = "{\"name\":\"" + name + "\",\"unique_id\":\"" + uid + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"command_topic\":\"" + commandTopic + "\",";
  payload += "\"availability_topic\":\"" + topicSystem("status") + "\",";
  payload += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
  payload += "\"device\":{\"identifiers\":[\"" + devId() + "\"],";
  payload += "\"name\":\"WiFiWatch\",\"manufacturer\":\"DIY\",";
  payload += "\"model\":\"" + String(PLATFORM_NAME) + "\",\"sw_version\":\"0.4.8\",";
  payload += "\"configuration_url\":\"http://" + mdnsHost() + "/\"}";
  if (extra.length()) payload += "," + extra;
  payload += "}";

  mqtt.publish(discoveryTopic.c_str(), payload.c_str(), true);
}

void removeLegacyDiscovery() {
  // Remove retained HA Discovery definitions from <=0.4.0.
  // This prevents duplicate/stale entities after the English topic cleanup.
  const char* sensorIds[] = {
    "outage_level",
    "outage_score",
    "bssids",
    "stable_visible",
    "stable_expected",
    "ssids",
    "scan_duration",
    "baseline_loss",
    "dynamic_drop",
    "quality_loss",
    "scan_interval"
  };

  for (const char* id : sensorIds) {
    String uid = devId() + "_" + id;
    String topic = "homeassistant/sensor/" + uid + "/config";
    mqtt.publish(topic.c_str(), "", true);
  }

  {
    String uid = devId() + "_alarm";
    String topic = "homeassistant/binary_sensor/" + uid + "/config";
    mqtt.publish(topic.c_str(), "", true);
  }

  {
    String uid = devId() + "_system_scan_interval";
    String topic = "homeassistant/sensor/" + uid + "/config";
    mqtt.publish(topic.c_str(), "", true);
  }
}

void discovery() {
  // English object IDs / labels and structured MQTT paths.
  discoveryPublish(
    "sensor",
    "neighborhood_outage_level",
    t("Ausfall-Einschätzung","Neighborhood Outage Level"),
    topicNeighborhood("outage_level"),
    outageLevelDiscoveryExtra()
  );

  discoveryPublish(
    "sensor",
    "neighborhood_outage_score",
    t("Ausfallwahrscheinlichkeit","Neighborhood Outage Score"),
    topicNeighborhood("outage_score"),
    "\"unit_of_measurement\":\"%\",\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "binary_sensor",
    "neighborhood_alarm",
    t("Nachbarschaft Stromausfall","Neighborhood Power Outage"),
    topicNeighborhood("alarm"),
    "\"payload_on\":\"1\",\"payload_off\":\"0\",\"device_class\":\"problem\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_bssid_count",
    t("Nachbar-BSSIDs","Neighborhood BSSID Count"),
    topicNeighborhood("bssid_count"),
    "\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_ssid_count",
    t("Nachbar-SSIDs","Neighborhood SSID Count"),
    topicNeighborhood("ssid_count"),
    "\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_stable_visible",
    t("Stabile BSSIDs sichtbar","Stable BSSIDs Visible"),
    topicNeighborhood("stable_visible"),
    "\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_stable_expected",
    t("Stabile BSSIDs erwartet","Stable BSSIDs Expected"),
    topicNeighborhood("stable_expected"),
    "\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_baseline_loss",
    t("Baseline-Verlust","Baseline Loss"),
    topicNeighborhood("baseline_loss_score"),
    "\"unit_of_measurement\":\"%\",\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_dynamic_drop",
    t("Dynamischer Einbruch","Dynamic Drop"),
    topicNeighborhood("dynamic_drop_score"),
    "\"unit_of_measurement\":\"%\",\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "neighborhood_quality_loss",
    t("Qualitätsverlust","Quality Loss"),
    topicNeighborhood("quality_loss_score"),
    "\"unit_of_measurement\":\"%\",\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "system_scan_duration",
    t("Scan-Dauer","Scan Duration"),
    topicSystem("scan_duration_ms"),
    "\"unit_of_measurement\":\"ms\",\"state_class\":\"measurement\""
  );

  discoveryNumber(
    "system_scan_interval",
    t("Scan-Intervall","Scan Interval"),
    topicSystem("scan_interval_sec"),
    topicControl("scan_interval_sec"),
    MIN_SCAN_INTERVAL_SEC,
    MAX_SCAN_INTERVAL_SEC,
    15
  );

  discoverySelect(
    "system_alarm_threshold",
    t("Alarm ab Wahrscheinlichkeit","Alarm Threshold"),
    topicSystem("alarm_threshold"),
    topicControl("alarm_threshold"),
    alarmThresholdDiscoveryExtra()
  );

  discoveryPublish(
    "sensor",
    "system_baseline_status",
    t("Baseline-Status","Baseline Status"),
    topicSystem("baseline_status"),
    baselineStatusDiscoveryExtra()
  );

  discoveryPublish(
    "sensor",
    "system_baseline_entries",
    t("Baseline BSSIDs","Baseline BSSIDs"),
    topicSystem("baseline_entries"),
    "\"state_class\":\"measurement\""
  );

  discoveryCommandOnly(
    "button",
    "baseline_relearn",
    t("Baseline neu lernen","Relearn Baseline"),
    topicControl("relearn"),
    "\"payload_press\":\"PRESS\",\"icon\":\"mdi:school\""
  );

  discoveryCommandOnly(
    "button",
    "baseline_reset",
    t("Baseline löschen","Reset Baseline"),
    topicControl("reset_baseline"),
    "\"payload_press\":\"PRESS\",\"icon\":\"mdi:database-remove\""
  );

  discoveryPublish(
    "binary_sensor",
    "wifi_alarm",
    t("Eigenes WLAN Alarm","Own WiFi Alarm"),
    topicWiFi("alarm"),
    "\"payload_on\":\"1\",\"payload_off\":\"0\",\"device_class\":\"problem\""
  );

  discoveryPublish(
    "sensor",
    "wifi_watch_expected",
    t("Eigene WLANs erwartet","Watched WiFi Expected"),
    topicWiFi("watch_expected"),
    "\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "wifi_watch_visible",
    t("Eigene WLANs sichtbar","Watched WiFi Visible"),
    topicWiFi("watch_visible"),
    "\"state_class\":\"measurement\""
  );

  discoveryPublish(
    "sensor",
    "wifi_watch_missing",
    t("Eigene WLANs fehlen","Watched WiFi Missing"),
    topicWiFi("watch_missing"),
    "\"state_class\":\"measurement\""
  );
}

void publishAll() {
  pubTopic(topicSystem("status"), "online");
  pubTopic(topicSystem("scan_duration_ms"), String(lastDur));
  pubTopic(topicSystem("scan_interval_sec"), String(cfg.scanIntervalSec));
  pubTopic(topicSystem("ignored_count"), String(ignored));
  pubTopic(topicSystem("learning_scans"), String((uint32_t)roundf(learnedScans)));
  pubTopic(topicSystem("baseline_status"), baselineStatusCode());
  pubTopic(topicSystem("baseline_entries"), String(learned.size()));
  pubTopic(topicSystem("alarm_threshold"), cfg.alarmThreshold);

  pubTopic(topicNeighborhood("bssid_count"), String(neighbors));
  pubTopic(topicNeighborhood("ssid_count"), String(uniqueSSIDs));
  pubTopic(topicNeighborhood("hidden_count"), String(hidden));
  pubTopic(topicNeighborhood("stable_visible"), String(stableVisible));
  pubTopic(topicNeighborhood("stable_expected"), String(stableExpected));
  pubTopic(topicNeighborhood("outage_score"), String(outageScore));
  pubTopic(topicNeighborhood("outage_level"), mqttOutageLevel());
  pubTopic(topicNeighborhood("alarm"), alarmFlag ? "1" : "0");
  pubTopic(topicNeighborhood("baseline_loss_score"), String(baselineLossScore));
  pubTopic(topicNeighborhood("dynamic_drop_score"), String(dynamicDropScore));
  pubTopic(topicNeighborhood("quality_loss_score"), String(qualityLossScore));

  pubTopic(topicWiFi("alarm"), wifiAlarm ? "1" : "0");
  pubTopic(topicWiFi("watch_expected"), String(watchExpected));
  pubTopic(topicWiFi("watch_visible"), String(watchVisible));
  pubTopic(topicWiFi("watch_missing"), String(watchMissing));

  for (int i = 1; i <= 13; i++) {
    pubTopic(topicNeighborhood("channel/" + String(i)), String(channels[i]));
  }
}

void mqttCallback(char* rawTopic, byte* payload, unsigned int length) {
  String commandTopic(rawTopic);
  String value;
  value.reserve(length);

  for (unsigned int i=0; i<length; i++)
    value += (char)payload[i];

  value.trim();

  if (commandTopic == topicControl("relearn")) {
    startRelearn(false);
    return;
  }

  if (commandTopic == topicControl("reset_baseline")) {
    startRelearn(true);
    return;
  }

  if (commandTopic == topicControl("alarm_threshold")) {
    if (value == "very_unlikely" ||
        value == "unlikely" ||
        value == "possible" ||
        value == "likely" ||
        value == "very_likely") {
      cfg.alarmThreshold = value;
      saveCfg();

      // Reset confirmation state when the threshold itself changes.
      neighborhoodAlarmStreak = 0;
      neighborhoodHealthyStreak = 0;
      alarmFlag = false;

      publishAll();
      draw();
    }
    return;
  }

  if (commandTopic == topicControl("scan_interval_sec")) {
    int sec = value.toInt();
    if (sec < MIN_SCAN_INTERVAL_SEC) sec = MIN_SCAN_INTERVAL_SEC;
    if (sec > MAX_SCAN_INTERVAL_SEC) sec = MAX_SCAN_INTERVAL_SEC;

    cfg.scanIntervalSec = (uint16_t)sec;
    saveCfg();

    recentSurvival.clear();
    lastScan = millis();

    publishAll();
    draw();
  }
}

void subscribeControls() {
  if (!mqtt.connected()) return;

  mqtt.subscribe(topicControl("relearn").c_str());
  mqtt.subscribe(topicControl("reset_baseline").c_str());
  mqtt.subscribe(topicControl("scan_interval_sec").c_str());
  mqtt.subscribe(topicControl("alarm_threshold").c_str());
}

void mqConnect() {
  if (!eth || cfg.host.isEmpty() || mqtt.connected() ||
      millis() - lastMq < 5000) return;

  lastMq = millis();
  String id = devId();

  bool ok = cfg.user.length()
    ? mqtt.connect(id.c_str(), cfg.user.c_str(), cfg.pass.c_str(),
                   topicSystem("status").c_str(), 0, true, "offline")
    : mqtt.connect(id.c_str(),
                   topicSystem("status").c_str(), 0, true, "offline");

  if (ok) {
    DBG_PRINTLN("[MQTT] connected");
    removeLegacyDiscovery();
    discovery();
    subscribeControls();
    publishAll();
    draw();
  } else {
    DBG_PRINTF("[MQTT] connect failed, rc=%d\n", mqtt.state());
  }
}

// ---------- dynamic learning / assessment ----------
void decayLearningIfNeeded() {
  uint32_t target = targetLearnWindowScans();

  if (learnedScans < (float)target) return;

  float factor = ((float)target - 1.0f) / (float)target;

  for (auto& kv : learned) {
    kv.second.seen *= factor;
    kv.second.rssiSum *= factor;
  }

  learnedScans *= factor;
}

void updateLearningFromCurrentScan() {
  decayLearningIfNeeded();

  for (const auto& a : aps) {
    if (a.ignS || a.ignB) continue;

    auto& l = learned[a.bssid];
    l.seen += 1.0f;
    l.rssiSum += (float)a.rssi;
    l.lastRssi = a.rssi;
    l.ssid = a.ssid;
  }

  learnedScans += 1.0f;
  baselineDirty = true;
}

void pushRecentSurvival(float survival) {
  if (recentSurvival.size() >= SHORT_HISTORY_LEN)
    recentSurvival.erase(recentSurvival.begin());

  recentSurvival.push_back(survival);
}

float previousShortAverage() {
  if (recentSurvival.empty()) return 1.0f;

  float sum = 0.0f;
  for (float v : recentSurvival) sum += v;
  return sum / (float)recentSurvival.size();
}

void assess() {
  std::set<String> visibleNow;
  for (const auto& a : aps)
    if (!a.ignS && !a.ignB)
      visibleNow.insert(a.bssid);

  stableExpected = 0;
  stableVisible = 0;

  float expectedWeight = 0.0f;
  float visibleWeight = 0.0f;

  float strongExpectedWeight = 0.0f;
  float strongVisibleWeight = 0.0f;

  // Evaluate against the PREVIOUS learned baseline.
  if (learnedScans >= 1.0f) {
    for (auto& kv : learned) {
      Learned& l = kv.second;

      if (learnedEntryIgnored(l, kv.first)) continue;

      float ratio = l.seen / learnedScans;
      if (ratio < STABLE_SEEN_RATIO) continue;

      int avgRssi = l.seen > 0.1f
        ? (int)roundf(l.rssiSum / l.seen)
        : -100;

      float weight = 1.0f;
      if (avgRssi > -70) weight = 1.5f;
      else if (avgRssi > STRONG_RSSI) weight = 1.25f;

      stableExpected++;
      expectedWeight += weight;

      bool visible = visibleNow.count(kv.first) > 0;
      if (visible) {
        stableVisible++;
        visibleWeight += weight;
      }

      // Quality factor deliberately focuses on reliably strong APs.
      if (avgRssi > STRONG_RSSI) {
        strongExpectedWeight += weight;
        if (visible) strongVisibleWeight += weight;
      }
    }
  }

  uint32_t minimumScans = minLearningScans();

  if (learnedScans < (float)minimumScans || expectedWeight < 1.0f) {
    outageScore = 0;
    baselineLossScore = 0;
    dynamicDropScore = 0;
    qualityLossScore = 0;
    shortTermAverageSurvival = 1.0f;
    displayState = DisplayState::Learning;
    alarmFlag = false;
    neighborhoodAlarmStreak = 0;
    neighborhoodHealthyStreak = 0;

    updateLearningFromCurrentScan();
    return;
  }

  float survival = constrain(visibleWeight / expectedWeight, 0.0f, 1.0f);
  baselineLossScore = constrain(
      (int)roundf((1.0f - survival) * 100.0f), 0, 100);

  // Compare current stable survival with the preceding 2-5 scans.
  // On the first evaluated scan there is intentionally no dynamic penalty yet.
  float suddenDrop = 0.0f;
  if (!recentSurvival.empty()) {
    shortTermAverageSurvival = previousShortAverage();
    suddenDrop = shortTermAverageSurvival - survival;
    if (suddenDrop < 0.0f) suddenDrop = 0.0f;
  } else {
    shortTermAverageSurvival = survival;
  }

  // A 50 percentage-point sudden drop already counts as 100% dynamic event.
  dynamicDropScore = constrain(
      (int)roundf((suddenDrop / 0.50f) * 100.0f), 0, 100);

  if (strongExpectedWeight > 0.01f) {
    float strongSurvival = constrain(
        strongVisibleWeight / strongExpectedWeight, 0.0f, 1.0f);
    qualityLossScore = constrain(
        (int)roundf((1.0f - strongSurvival) * 100.0f), 0, 100);
  } else {
    qualityLossScore = baselineLossScore;
  }

  float composite =
      WEIGHT_BASELINE * baselineLossScore +
      WEIGHT_DYNAMIC  * dynamicDropScore +
      WEIGHT_QUALITY  * qualityLossScore;

  outageScore = constrain((int)roundf(composite), 0, 100);

  if (outageScore < 15) {
    displayState = DisplayState::VeryUnlikely;
  }
  else if (outageScore < 30) {
    displayState = DisplayState::Unlikely;
  }
  else if (outageScore < 50) {
    displayState = DisplayState::Possible;
  }
  else if (outageScore < 75) {
    displayState = DisplayState::Likely;
  }
  else {
    displayState = DisplayState::VeryLikely;
  }

  // Score and level react immediately, but the binary alarm is deliberately
  // confirmed over two scans to suppress one-scan RF dropouts.
  bool alarmCandidate =
      displayState != DisplayState::Learning &&
      displayStateRank() >= outageLevelRank(cfg.alarmThreshold);

  if (alarmCandidate) {
    neighborhoodHealthyStreak = 0;
    if (neighborhoodAlarmStreak < 255) neighborhoodAlarmStreak++;
    if (neighborhoodAlarmStreak >= 2) alarmFlag = true;
  } else {
    neighborhoodAlarmStreak = 0;
    if (neighborhoodHealthyStreak < 255) neighborhoodHealthyStreak++;
    if (neighborhoodHealthyStreak >= 2) alarmFlag = false;
  }

  // Store current result for NEXT scan's dynamic comparison.
  pushRecentSurvival(survival);

  // Never teach a likely/very_likely event into the long-term baseline,
  // even during the first (not-yet-confirmed) alarm scan.
  if (!alarmCandidate)
    updateLearningFromCurrentScan();
}

// ---------- TFT ----------
String tftStatusText() {
  if (isGerman()) {
    switch (displayState) {
      case DisplayState::Learning:      return "LERNPHASE";
      case DisplayState::VeryUnlikely:  return "SEHR UNWAHRSCHEINLICH";
      case DisplayState::Unlikely:      return "UNWAHRSCHEINLICH";
      case DisplayState::Possible:      return "MOEGLICH";
      case DisplayState::Likely:        return "WAHRSCHEINLICH";
      case DisplayState::VeryLikely:    return "SEHR WAHRSCHEINLICH";
    }
  } else {
    switch (displayState) {
      case DisplayState::Learning:      return "LEARNING";
      case DisplayState::VeryUnlikely:  return "VERY UNLIKELY";
      case DisplayState::Unlikely:      return "UNLIKELY";
      case DisplayState::Possible:      return "POSSIBLE";
      case DisplayState::Likely:        return "LIKELY";
      case DisplayState::VeryLikely:    return "VERY LIKELY";
    }
  }
  return isGerman() ? "LERNPHASE" : "LEARNING";
}

uint32_t tftStatusColor() {
  switch (displayState) {
    case DisplayState::Learning:      return TFT_CYAN;
    case DisplayState::VeryUnlikely:  return TFT_GREEN;
    case DisplayState::Unlikely:      return TFT_GREEN;
    case DisplayState::Possible:      return TFT_YELLOW;
    case DisplayState::Likely:        return TFT_ORANGE;
    case DisplayState::VeryLikely:    return TFT_RED;
  }
  return TFT_CYAN;
}

void drawCenteredFit(const String& text, int y, int maxWidth, uint32_t color) {
  display.setTextColor(color, TFT_BLACK);
  int sizes[] = {4,3,2,1};

  for (int s : sizes) {
    display.setTextSize(s);
    int w = display.textWidth(text);
    if (w <= maxWidth) {
      display.setCursor((240 - w) / 2, y);
      display.print(text);
      return;
    }
  }
}

void draw() {
  display.fillScreen(TFT_BLACK);

  display.setTextColor(TFT_WHITE, TFT_BLACK);
  display.setTextSize(1);
  display.setCursor(8,6);
  display.print("WiFiWatch");
  display.setCursor(170,6);
  display.print("0.4.8");
  display.drawFastHLine(8,20,224,TFT_DARKGREY);

  display.setTextDatum(textdatum_t::top_center);
  display.setTextColor(TFT_WHITE,TFT_BLACK);
  display.setTextSize(2);
  display.drawString(isGerman() ? "NACHBARSCHAFT" : "NEIGHBORHOOD",120,30);

  String status = tftStatusText();
  uint32_t col = tftStatusColor();

  int y = 66;
  display.setTextDatum(textdatum_t::top_left);
  drawCenteredFit(status,y,220,col);

  display.setTextDatum(textdatum_t::top_left);
  display.setTextColor(TFT_WHITE,TFT_BLACK);
  display.setTextSize(1);

  display.setCursor(18,118);
  display.printf("Score       %3d %%",outageScore);

  display.setCursor(18,136);
  display.printf(isGerman() ? "Nachbarn    %2d / %-2d" : "Neighbors   %2d / %-2d",stableVisible,stableExpected);

  display.setCursor(18,154);
  display.printf(isGerman() ? "Dynamik     %3d %%" : "Dynamic     %3d %%",dynamicDropScore);

  display.drawFastHLine(8,180,224,TFT_DARKGREY);

  display.setCursor(18,194);
  display.print("LAN");
  display.setTextColor(eth ? TFT_GREEN : TFT_RED,TFT_BLACK);
  display.setCursor(58,194);
  display.print(eth ? "OK" : "DOWN");

  display.setTextColor(TFT_WHITE,TFT_BLACK);
  display.setCursor(126,194);
  display.print("MQTT");
  display.setTextColor(mqtt.connected() ? TFT_GREEN : TFT_YELLOW,TFT_BLACK);
  display.setCursor(180,194);
  display.print(mqtt.connected() ? "OK" : "OFF");

  display.setTextDatum(textdatum_t::top_left);
  display.setTextSize(1);

  display.setCursor(18,216);
  display.setTextColor(alarmFlag ? TFT_RED : TFT_GREEN,TFT_BLACK);
  display.print(alarmFlag ? (isGerman() ? "NETZ ALARM" : "POWER ALARM")
                          : (isGerman() ? "NETZ OK" : "POWER OK"));

  display.setCursor(126,216);
  display.setTextColor(wifiAlarm ? TFT_RED : TFT_GREEN,TFT_BLACK);
  display.print(wifiAlarm ? (isGerman() ? "WIFI ALARM" : "WIFI ALARM")
                          : (isGerman() ? "WIFI OK" : "WIFI OK"));

  display.setTextDatum(textdatum_t::top_left);
}


void assessWatchList() {
  std::set<String> visibleSSIDs;
  std::set<String> visibleBSSIDs;

  for (const auto& a : aps) {
    if (a.ssid.length()) visibleSSIDs.insert(a.ssid);
    visibleBSSIDs.insert(a.bssid);
  }

  watchExpected = (int)watchSSIDs.size() + (int)watchBSSIDs.size();
  watchVisible = 0;

  for (const auto& ssid : watchSSIDs)
    if (visibleSSIDs.count(ssid)) watchVisible++;

  for (const auto& bssid : watchBSSIDs)
    if (visibleBSSIDs.count(bssid)) watchVisible++;

  watchMissing = watchExpected - watchVisible;

  // Two consecutive scans are required to set/clear the alarm.
  // This avoids a single missed beacon scan causing an alarm.
  if (watchExpected == 0) {
    watchMissingStreak = 0;
    watchHealthyStreak = 0;
    wifiAlarm = false;
    return;
  }

  if (watchMissing > 0) {
    watchHealthyStreak = 0;
    if (watchMissingStreak < 255) watchMissingStreak++;
    if (watchMissingStreak >= 2) wifiAlarm = true;
  } else {
    watchMissingStreak = 0;
    if (watchHealthyStreak < 255) watchHealthyStreak++;
    if (watchHealthyStreak >= 2) wifiAlarm = false;
  }
}

// ---------- WiFi scan ----------
void scan() {
  DBG_PRINTLN("\n=== WiFi Scan ===");

  uint32_t st = millis();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false,true);
  delay(80);

  int n = WiFi.scanNetworks(false,true);
  lastDur = millis() - st;

  aps.clear();
  total = n > 0 ? n : 0;
  ignored = neighbors = uniqueSSIDs = hidden = 0;
  memset(channels,0,sizeof(channels));

  std::set<String> u;

  if (n > 0) {
    for (int i=0; i<n && aps.size()<MAX_APS; i++) {
      AP a{
        WiFi.SSID(i),
        WiFi.BSSIDstr(i),
        WiFi.RSSI(i),
        WiFi.channel(i),
        false,
        false
      };

      a.ignS = a.ssid.length() && (exact(ignSSIDs,a.ssid) || exact(watchSSIDs,a.ssid));
      a.ignB = ci(ignBSSIDs,a.bssid) || ci(watchBSSIDs,a.bssid);

      aps.push_back(a);

      if (a.ignS || a.ignB) {
        ignored++;
        continue;
      }

      neighbors++;

      if (a.ssid.length()) u.insert(a.ssid);
      else hidden++;

      if (a.ch >= 1 && a.ch <= 13)
        channels[a.ch]++;
    }
  }

  uniqueSSIDs = u.size();

  std::sort(aps.begin(),aps.end(),
            [](const AP& a,const AP& b){ return a.rssi > b.rssi; });

  WiFi.scanDelete();

  assessWatchList();
  assess();

  DBG_PRINTF("Found total : %d APs\n",total);
  DBG_PRINTF("Ignored     : %d\n",ignored);
  DBG_PRINTF("Neighbours  : %d BSSIDs\n",neighbors);
  DBG_PRINTF("Stable      : %d/%d\n",stableVisible,stableExpected);
  DBG_PRINTF("Baseline    : %d%%\n",baselineLossScore);
  DBG_PRINTF("Dynamic     : %d%%\n",dynamicDropScore);
  DBG_PRINTF("Quality     : %d%%\n",qualityLossScore);
  DBG_PRINTF("Outage      : %s (%d%%)\n",mqttOutageLevel().c_str(),outageScore);
  DBG_PRINTF("Alarm       : %d (candidate streak %u, healthy streak %u)\n",
             alarmFlag?1:0, neighborhoodAlarmStreak, neighborhoodHealthyStreak);
  DBG_PRINTF("WiFi watch  : %d/%d missing=%d alarm=%d\n",
             watchVisible,watchExpected,watchMissing,wifiAlarm?1:0);
  DBG_PRINTF("Learn scans : %.1f / %lu minimum\n",
             learnedScans,(unsigned long)minLearningScans());
  DBG_PRINTF("Duration    : %lu ms\n",(unsigned long)lastDur);

  publishAll();
  draw();
}

// ---------- Web ----------
String head() {
  String h = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{font-family:system-ui,Arial;background:#101214;color:#eee;margin:0}
.w{max-width:1100px;margin:auto;padding:18px}
.c{background:#1b1e21;border-radius:12px;padding:16px;margin:14px 0}
.hero{text-align:center;padding:24px}
.hero h2{font-size:2rem;margin:.2rem}
.sub{color:#aaa}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px}
.m{background:#272b2f;padding:12px;border-radius:9px}
.m b{display:block;font-size:1.7em}
.ok{color:#69d879}.warn{color:#ffd166}.bad{color:#ff6b6b}
.nav a,a.btn,button{background:#356bd7;color:#fff;border:0;border-radius:6px;padding:7px 10px;text-decoration:none;cursor:pointer;margin-right:6px}
input,select{width:100%;box-sizing:border-box;background:#111;color:#eee;border:1px solid #555;border-radius:5px;padding:7px;margin:4px 0 10px}
table{width:100%;border-collapse:collapse}
th,td{padding:7px;border-bottom:1px solid #333;text-align:left;font-size:.92em}
.ig{opacity:.42}.tiny{font-size:.8em;padding:5px!important}
.scoregrid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
@media(max-width:600px){.scoregrid{grid-template-columns:1fr}}
</style></head><body><div class="w">
<h1>ESP-WiFiWatch <span class="sub">v0.4.8</span></h1>
)HTML";
  h += "<div class='nav'><a class='btn' href='/'>Status</a><a class='btn' href='/config'>" +
       t("Konfiguration","Configuration") + "</a></div>";
  return h;
}

void redir(const String& x) {
  server.sendHeader("Location",x,true);
  server.send(303,"text/plain","");
}

void root() {
  String h = head();

  String cls = "ok";
  if (displayState == DisplayState::Possible) cls = "warn";
  if (displayState == DisplayState::Likely ||
      displayState == DisplayState::VeryLikely) cls = "bad";

  h += "<div class='c hero'><div class='sub'>" + t("Nachbarschaft","Neighborhood") + "</div>";
  h += "<h2 class='" + cls + "'>" + t("Stromausfall: ","Power outage: ") + esc(localizedOutageLevel()) + "</h2>";
  h += "<div>" + t("Alarmflag: ","Alarm flag: ") + "<b class='" +
       String(alarmFlag ? "bad" : "ok") + "'>" +
       String(alarmFlag ? t("AKTIV (1)","ACTIVE (1)") : t("INAKTIV (0)","INACTIVE (0)")) + "</b>";
  if (!alarmFlag && neighborhoodAlarmStreak == 1)
    h += " · <span class='warn'>" + t("Bestätigung 1/2","Confirmation 1/2") + "</span>";
  h += "</div>";

  if (displayState == DisplayState::Learning) {
    h += "<p class='sub'>" + t("Baseline wird gelernt: ","Learning baseline: ") +
         String((uint32_t)roundf(learnedScans)) + " / " +
         String(minLearningScans()) + t(" Mindest-Scans"," minimum scans") + "</p>";
  } else {
    h += "<p class='sub'>" + String(stableVisible) + t(" von "," of ") +
         String(stableExpected) +
         t(" stabilen Nachbar-BSSIDs sichtbar · Gesamt-Score ",
           " stable neighborhood BSSIDs visible · Total score ") +
         String(outageScore) + "%</p>";
  }
  h += "</div>";

  if (watchExpected == 0) {
    h += "<div class='c'><b>" + t("Eigenes WLAN","Own WiFi") + "</b> · ";
    h += "<span class='sub'>" + t("Nicht konfiguriert","Not configured") + "</span> ";
    h += "<a class='btn tiny' href='/config'>" + t("Konfigurieren","Configure") + "</a></div>";
  } else {
    h += "<div class='c'><h2>" + t("Eigenes WLAN","Own WiFi") + "</h2>";
    h += "<div class='g'>";
    h += "<div class='m'>" + t("WiFi Alarm","WiFi Alarm") + "<b class='" +
         String(wifiAlarm ? "bad" : "ok") + "'>" +
         String(wifiAlarm ? t("AKTIV","ACTIVE") : "OK") + "</b></div>";
    h += "<div class='m'>" + t("Erwartet","Expected") + "<b>" + String(watchExpected) + "</b></div>";
    h += "<div class='m'>" + t("Sichtbar","Visible") + "<b>" + String(watchVisible) + "</b></div>";
    h += "<div class='m'>" + t("Fehlend","Missing") + "<b>" + String(watchMissing) + "</b></div>";
    h += "</div></div>";
  }

  h += "<div class='c'><h2>" + t("Erkennungsfaktoren","Detection Factors") + "</h2>";
  h += "<p class='sub'>" + t("Binärer Alarm ab ","Binary alarm from ") +
       "<b>" + localizedProbabilityName(cfg.alarmThreshold) + "</b> · " +
       t("Bestätigung über 2 Scans","confirmed over 2 scans") + "</p>";
  h += "<div class='scoregrid'>";
  h += "<div class='m'>" + t("Baseline-Verlust","Baseline Loss") + "<b>" + String(baselineLossScore) +
       "%</b><span class='sub'>" + t("Gewicht 50 %","Weight 50 %") + "</span></div>";
  h += "<div class='m'>" + t("Abrupter Einbruch","Sudden Drop") + "<b>" + String(dynamicDropScore) +
       "%</b><span class='sub'>" + t("Gewicht 30 %","Weight 30 %") + "</span></div>";
  h += "<div class='m'>" + t("Starke stabile APs","Strong Stable APs") + "<b>" + String(qualityLossScore) +
       "%</b><span class='sub'>" + t("Gewicht 20 %","Weight 20 %") + "</span></div>";
  h += "</div></div>";

  h += "<div class='c g'>";
  h += "<div class='m'>" + t("Fremde BSSIDs","Neighborhood BSSIDs") + "<b>" + String(neighbors) + "</b></div>";
  h += "<div class='m'>" + t("Stabil sichtbar","Stable visible") + "<b>" + String(stableVisible) +
       " / " + String(stableExpected) + "</b></div>";
  h += "<div class='m'>SSIDs<b>" + String(uniqueSSIDs) + "</b></div>";
  h += "<div class='m'>Hidden<b>" + String(hidden) + "</b></div>";
  h += "<div class='m'>" + t("Ignoriert","Ignored") + "<b>" + String(ignored) + "</b></div>";
  h += "<div class='m'>" + t("Scan-Intervall","Scan interval") + "<b>" + String(cfg.scanIntervalSec) + " s</b></div>";
  h += "</div>";

  h += "<div class='c'><h2>" + t("Sichtbare Access Points","Visible Access Points") + "</h2>";
  h += "<table><tr><th>SSID</th><th>BSSID</th><th>RSSI</th><th>CH</th><th>" + t("Aktion","Action") + "</th></tr>";

  for (size_t i=0; i<aps.size(); i++) {
    auto& a = aps[i];
    bool ig = a.ignS || a.ignB;

    h += "<tr" + String(ig ? " class='ig'" : "") + "><td>" +
         esc(a.ssid.length() ? a.ssid : "(hidden)") + "</td>";
    h += "<td>" + a.bssid + "</td><td>" + String(a.rssi) +
         "</td><td>" + String(a.ch) + "</td><td>";

    bool watched = isWatchedSSID(a.ssid) || isWatchedBSSID(a.bssid);

    if (!watched) {
      if (a.ssid.length()) {
        h += "<a class='btn tiny' href='/watch/add?t=s&i=" +
             String(i) + "'>" + t("SSID überwachen","Watch SSID") + "</a>";
      }
      h += "<a class='btn tiny' href='/watch/add?t=b&i=" +
           String(i) + "'>" + t("BSSID überwachen","Watch BSSID") + "</a>";
    } else {
      h += "<span class='ok'>" + t("überwacht","watched") + "</span> ";
    }

    if (!ig) {
      if (a.ssid.length())
        h += "<a class='btn tiny' href='/ignore/add?t=s&i=" +
             String(i) + "'>" + t("SSID ignorieren","Ignore SSID") + "</a>";
      h += "<a class='btn tiny' href='/ignore/add?t=b&i=" +
           String(i) + "'>" + t("BSSID ignorieren","Ignore BSSID") + "</a>";
    } else if (!watched) {
      h += t("ignoriert","ignored");
    }

    h += "</td></tr>";
  }

  h += "</table></div>";

  h += "<div class='c sub'>" + String(PLATFORM_NAME) + " · " + mdnsHost() +
       " · IPv4 " + ETH.localIP().toString() +
       " · LAN <span class='" + String(eth ? "ok" : "bad") + "'><b>" +
       String(eth ? "OK" : "OFFLINE") + "</b></span>" +
       " · MQTT <span class='" + String(mqtt.connected() ? "ok" : "bad") + "'><b>" +
       String(mqtt.connected() ? "ONLINE" : "OFFLINE") + "</b></span>" +
       " · " + t("Scan ","Scan ") + String(lastDur) + " ms · " +
       t("Baseline ","Baseline ") + localizedBaselineStatus() + " · " +
       t("Lern-Scans ","Learning scans ") +
       String((uint32_t)roundf(learnedScans)) + "</div>";

  h += "</div></body></html>";
  server.send(200,"text/html; charset=utf-8",h);
}

void configPage() {
  if (!requireAuth()) return;

  String h = head();

  h += "<div class='c'><h2>" + t("Netzwerk","Network") + "</h2>";
  h += "<form method='post' action='/config/network'>";
  h += "<label>Hostname / mDNS</label>";
  h += "<input name='hostname' value='" + esc(cfg.hostname) + "'>";
  h += "<button>" + t("Hostname speichern & neu starten","Save hostname & restart") + "</button></form>";
  h += "<p class='sub'>" +
       t("Erreichbar als ","Available as ") +
       "<b>http://" + mdnsHost() + "/</b><br>" +
       t("Erlaubt sind Buchstaben, Zahlen und Bindestrich. Standard: wifiwatch. "
         "Nach einer Änderung startet das Gerät neu.",
         "Letters, numbers and hyphens are allowed. Default: wifiwatch. "
         "The device restarts after a change.") +
       "</p></div>";

  h += "<div class='c'><h2>" + t("Sprache","Language") + "</h2>";
  h += "<form method='post' action='/config/language'>";
  h += "<label>" + t("Webseite und Display","Website and display") + "</label>";
  h += "<select name='language'>";
  h += "<option value='de'" + String(cfg.language=="de" ? " selected" : "") + ">Deutsch</option>";
  h += "<option value='en'" + String(cfg.language=="en" ? " selected" : "") + ">English</option>";
  h += "</select><button>" + t("Sprache speichern","Save language") + "</button></form>";
  h += "<p class='sub'>" + t("MQTT-Pfade und Zustandswerte bleiben immer Englisch.",
                              "MQTT topics and state values always remain English.") + "</p></div>";

  h += "<div class='c'><h2>" + t("Erkennung","Detection") + "</h2>";
  h += "<form method='post' action='/config/detection'>";
  h += "<label>" + t("Scan-Intervall","Scan interval") + "</label>";
  h += "<select name='scan_sec'>";
  const int intervals[] = {15,30,60,120,300};
  for (int v : intervals) {
    h += "<option value='" + String(v) + "'" +
         String(cfg.scanIntervalSec == v ? " selected" : "") +
         ">" + String(v) + t(" Sekunden"," seconds") + "</option>";
  }
  h += "</select>";
  h += "<label>" + t("Alarm ab Wahrscheinlichkeit","Alarm from probability") + "</label>";
  h += "<select name='alarm_threshold'>";
  const char* alarmLevels[] = {"very_unlikely","unlikely","possible","likely","very_likely"};
  for (const char* raw : alarmLevels) {
    h += "<option value='" + String(raw) + "'" +
         String(cfg.alarmThreshold == raw ? " selected" : "") +
         ">" + localizedProbabilityName(raw) + "</option>";
  }
  h += "</select>";
  h += "<button>" + t("Erkennung speichern","Save detection settings") + "</button></form>";
  h += "<p class='sub'>" +
       t("Standard: 30 s. Die Lernphase dauert unabhängig vom Intervall ungefähr 10 Minuten. Der Langzeit-Horizont liegt bei ungefähr 2 Stunden.",
         "Default: 30 s. The learning phase lasts about 10 minutes regardless of interval. The long-term horizon is about 2 hours.") +
       "</p></div>";

  h += "<div class='c'><h2>Baseline</h2>";
  h += "<div class='g'>";
  h += "<div class='m'>" + t("Status","Status") + "<b>" + localizedBaselineStatus() + "</b></div>";
  h += "<div class='m'>" + t("Gelernte BSSIDs","Learned BSSIDs") + "<b>" + String(learned.size()) + "</b></div>";
  h += "<div class='m'>" + t("Lern-Scans","Learning scans") + "<b>" +
       String((uint32_t)roundf(learnedScans)) + "</b></div>";
  h += "<div class='m'>" + t("Persistent","Persistent") + "<b>" +
       String(baselineEverSaved ? t("JA","YES") : t("NEIN","NO")) + "</b></div>";
  h += "</div>";
  h += "<p class='sub'>" +
       t("Eine gespeicherte Baseline wird beim Boot sofort geladen. Danach läuft ungefähr 5 Minuten eine sanfte Validierung. "
         "Im Normalbetrieb passt sie sich weiter langsam an und wird spätestens alle 30 Minuten gespeichert. "
         "Bei aktivem Nachbarschaftsalarm wird nicht weiter angelernt.",
         "A saved baseline is loaded immediately at boot, followed by about 5 minutes of gentle validation. "
         "During normal operation it continues to adapt slowly and is saved at most every 30 minutes. "
         "Learning is paused while the neighborhood alarm is active.") +
       "</p>";
  h += "<form style='display:inline' method='post' action='/baseline/relearn'>";
  h += "<button>" + t("Lernphase neu starten","Relearn baseline") + "</button></form> ";
  h += "<form style='display:inline' method='post' action='/baseline/reset'>";
  h += "<button>" + t("Baseline löschen","Reset baseline") + "</button></form>";
  h += "</div>";

  h += "<div class='c'><h2>" + t("Web-Zugriffsschutz","Web Access Protection") + "</h2>";
  h += "<form method='post' action='/config/auth'>";
  h += "<label>" + t("Benutzer","User") + "</label><input name='user' value='" + esc(cfg.webUser) + "'>";
  h += "<label>" + t("Neues Kennwort","New password") + "</label><input type='password' name='pass' placeholder='" +
       t("leer = unverändert","blank = unchanged") + "'>";
  h += "<button>" + t("Web-Zugang speichern","Save web access") + "</button></form>";
  h += "<p class='sub'>" +
       t("Schützt Konfiguration, Ignorieren/Entfernen und Firmware-Update per HTTP Basic Auth. Keine Verschlüsselung.",
         "Protects configuration, ignore/remove actions and firmware update using HTTP Basic Auth. No encryption.") +
       "</p></div>";

  h += "<div class='c'><h2>MQTT</h2>";
  h += "<form method='post' action='/config/mqtt'>";
  h += "<label>" + t("Broker / IP","Broker / IP") + "</label><input name='host' value='" + esc(cfg.host) + "'>";
  h += "<label>Port</label><input name='port' value='" + String(cfg.port) + "'>";
  h += "<label>" + t("Benutzer","User") + "</label><input name='user' value='" + esc(cfg.user) + "'>";
  h += "<label>" + t("Passwort","Password") + "</label><input type='password' name='pass' value='" + esc(cfg.pass) + "'>";
  h += "<label>" + t("Basis-Topic","Base topic") + "</label><input name='topic' value='" + esc(cfg.topic) + "'>";
  h += "<label><input style='width:auto' type='checkbox' name='ha' value='1' " +
       String(cfg.ha ? "checked" : "") + "> Home Assistant MQTT Discovery</label><br>";
  h += "<button>" + t("Speichern & verbinden","Save & connect") + "</button></form></div>";

  h += "<div class='c'><h2>" + t("Watch List eigenes WLAN","Own WiFi Watch List") + "</h2>";
  h += "<p class='sub'>" +
       t("Überwachte SSIDs/BSSIDs werden automatisch aus der Nachbarschaftsanalyse ausgeschlossen. "
         "Ein fehlender Eintrag löst nach zwei aufeinanderfolgenden Scans den separaten WiFi-Alarm aus.",
         "Watched SSIDs/BSSIDs are automatically excluded from neighborhood analysis. "
         "A missing item triggers the separate WiFi alarm after two consecutive scans.") +
       "</p>";
  h += "<table><tr><th>" + t("Typ","Type") + "</th><th>" + t("Eintrag","Entry") + "</th><th></th></tr>";

  for (size_t i=0; i<watchSSIDs.size(); i++)
    h += "<tr><td>SSID</td><td>" + esc(watchSSIDs[i]) +
         "</td><td><a class='btn tiny' href='/watch/del?t=s&i=" +
         String(i) + "'>" + t("Entfernen","Remove") + "</a></td></tr>";

  for (size_t i=0; i<watchBSSIDs.size(); i++)
    h += "<tr><td>BSSID</td><td>" + watchBSSIDs[i] +
         "</td><td><a class='btn tiny' href='/watch/del?t=b&i=" +
         String(i) + "'>" + t("Entfernen","Remove") + "</a></td></tr>";

  h += "</table></div>";

  h += "<div class='c'><h2>" + t("Ignorierliste","Ignore List") + "</h2>";
  h += "<table><tr><th>" + t("Typ","Type") + "</th><th>" + t("Eintrag","Entry") + "</th><th></th></tr>";

  for (size_t i=0; i<ignSSIDs.size(); i++)
    h += "<tr><td>SSID</td><td>" + esc(ignSSIDs[i]) +
         "</td><td><a class='btn tiny' href='/ignore/del?t=s&i=" +
         String(i) + "'>" + t("Entfernen","Remove") + "</a></td></tr>";

  for (size_t i=0; i<ignBSSIDs.size(); i++)
    h += "<tr><td>BSSID</td><td>" + ignBSSIDs[i] +
         "</td><td><a class='btn tiny' href='/ignore/del?t=b&i=" +
         String(i) + "'>" + t("Entfernen","Remove") + "</a></td></tr>";

  h += "</table></div>";

  h += "<div class='c'><h2>" + t("Firmware Update","Firmware Update") + "</h2>";
  h += "<p class='sub'>" +
       t("Passende Plattform-Firmware aus ","Use the matching platform firmware from ") +
       "<code>firmware/</code>: <b>ESP-WiFiWatch-v0.4.8-" +
       String(PLATFORM_SHORT) + ".bin</b></p>";
  h += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  h += "<input type='file' name='firmware' accept='.bin,application/octet-stream'>";
  h += "<button>" + t("Firmware hochladen","Upload firmware") + "</button></form></div>";

  h += "</div></body></html>";
  server.send(200,"text/html; charset=utf-8",h);
}

void setupWeb() {
  server.on("/",HTTP_GET,root);
  server.on("/config",HTTP_GET,configPage);

  server.on("/config/network",HTTP_POST,[](){
    if (!requireAuth()) return;

    String newHost = sanitizeHostname(server.arg("hostname"));
    bool changed = newHost != cfg.hostname;
    cfg.hostname = newHost;
    saveCfg();

    if (changed) {
      server.send(200, "text/html; charset=utf-8",
        isGerman()
        ? "<html><body><h2>Hostname gespeichert</h2><p>WiFiWatch startet neu...</p></body></html>"
        : "<html><body><h2>Hostname saved</h2><p>WiFiWatch is restarting...</p></body></html>");
      delay(700);
      ESP.restart();
      return;
    }

    redir("/config");
  });

  server.on("/config/language",HTTP_POST,[](){
    if (!requireAuth()) return;

    String lang = server.arg("language");
    if (lang == "de" || lang == "en") {
      cfg.language = lang;
      saveCfg();
      draw();

      // Keep technical IDs/topics stable, but refresh visible HA names.
      if (mqtt.connected() && cfg.ha) {
        discovery();
      }
    }

    redir("/config");
  });

  server.on("/config/detection",HTTP_POST,[](){
    if (!requireAuth()) return;

    uint16_t sec = (uint16_t)server.arg("scan_sec").toInt();
    cfg.scanIntervalSec = constrain(sec,
                                    MIN_SCAN_INTERVAL_SEC,
                                    MAX_SCAN_INTERVAL_SEC);

    String threshold = server.arg("alarm_threshold");
    if (threshold == "very_unlikely" ||
        threshold == "unlikely" ||
        threshold == "possible" ||
        threshold == "likely" ||
        threshold == "very_likely") {
      if (threshold != cfg.alarmThreshold) {
        cfg.alarmThreshold = threshold;
        neighborhoodAlarmStreak = 0;
        neighborhoodHealthyStreak = 0;
        alarmFlag = false;
      }
    }

    saveCfg();

    // Keep the already learned baseline, but reset the short-term
    // dynamic history because its time scale just changed.
    recentSurvival.clear();
    lastScan = millis();

    publishAll();
    draw();
    redir("/config");
  });

  server.on("/baseline/relearn",HTTP_POST,[](){
    if (!requireAuth()) return;

    startRelearn(false);

    server.send(200, "text/html; charset=utf-8",
      isGerman()
      ? "<html><body><h2>Lernphase gestartet</h2><p>Die bisherige Baseline wurde verworfen.</p><p><a href='/config'>Zurück</a></p></body></html>"
      : "<html><body><h2>Learning phase started</h2><p>The previous baseline was discarded.</p><p><a href='/config'>Back</a></p></body></html>");
  });

  server.on("/baseline/reset",HTTP_POST,[](){
    if (!requireAuth()) return;

    startRelearn(true);

    server.send(200, "text/html; charset=utf-8",
      isGerman()
      ? "<html><body><h2>Baseline gelöscht</h2><p>Die persistente Baseline wurde gelöscht. Eine neue Lernphase läuft.</p><p><a href='/config'>Zurück</a></p></body></html>"
      : "<html><body><h2>Baseline reset</h2><p>The persistent baseline was deleted. A new learning phase is running.</p><p><a href='/config'>Back</a></p></body></html>");
  });

  server.on("/config/auth",HTTP_POST,[](){
    if (!requireAuth()) return;

    String u = server.arg("user");
    u.trim();
    String p = server.arg("pass");

    if (u.length()) cfg.webUser = u;
    if (p.length()) cfg.webPass = p;

    saveCfg();

    server.sendHeader("Connection","close");
    server.send(200,"text/html; charset=utf-8",
      isGerman()
      ? "<html><body><h2>Zugangsdaten gespeichert</h2><p>Beim nächsten geschützten Zugriff bitte neu anmelden.</p><p><a href='/'>Zur Statusseite</a></p></body></html>"
      : "<html><body><h2>Credentials saved</h2><p>Please sign in again on the next protected request.</p><p><a href='/'>Back to status</a></p></body></html>");
  });

  server.on("/config/mqtt",HTTP_POST,[](){
    if (!requireAuth()) return;

    cfg.host = server.arg("host");
    cfg.host.trim();

    cfg.port = (uint16_t)server.arg("port").toInt();
    if (!cfg.port) cfg.port = 1883;

    cfg.user = server.arg("user");
    cfg.pass = server.arg("pass");

    cfg.topic = server.arg("topic");
    cfg.topic.trim();
    if (cfg.topic.isEmpty()) cfg.topic = DEFAULT_MQTT_TOPIC;

    cfg.ha = server.hasArg("ha");

    saveCfg();

    mqtt.disconnect();
    mqtt.setServer(cfg.host.c_str(),cfg.port);
    lastMq = 0;

    redir("/config");
  });

  server.on("/watch/add",HTTP_GET,[](){
    if (!requireAuth()) return;

    int i = server.arg("i").toInt();

    if (i >= 0 && i < (int)aps.size()) {
      if (server.arg("t") == "s" &&
          aps[i].ssid.length() &&
          !exact(watchSSIDs,aps[i].ssid) &&
          watchSSIDs.size() < MAX_IGNORE) {
        watchSSIDs.push_back(aps[i].ssid);
      }

      if (server.arg("t") == "b" &&
          !ci(watchBSSIDs,aps[i].bssid) &&
          watchBSSIDs.size() < MAX_IGNORE) {
        watchBSSIDs.push_back(aps[i].bssid);
      }

      saveCfg();
      recentSurvival.clear();
      watchMissingStreak = watchHealthyStreak = 0;
      scan();
    }

    redir("/");
  });

  server.on("/watch/del",HTTP_GET,[](){
    if (!requireAuth()) return;

    int i = server.arg("i").toInt();

    if (server.arg("t") == "b") {
      if (i >= 0 && i < (int)watchBSSIDs.size())
        watchBSSIDs.erase(watchBSSIDs.begin()+i);
    } else {
      if (i >= 0 && i < (int)watchSSIDs.size())
        watchSSIDs.erase(watchSSIDs.begin()+i);
    }

    saveCfg();
    recentSurvival.clear();
    watchMissingStreak = watchHealthyStreak = 0;
    scan();
    redir("/config");
  });

  server.on("/ignore/add",HTTP_GET,[](){
    if (!requireAuth()) return;

    int i = server.arg("i").toInt();

    if (i >= 0 && i < (int)aps.size()) {
      if (server.arg("t") == "s" &&
          aps[i].ssid.length() &&
          !exact(ignSSIDs,aps[i].ssid) &&
          ignSSIDs.size() < MAX_IGNORE)
        ignSSIDs.push_back(aps[i].ssid);

      if (server.arg("t") == "b" &&
          !ci(ignBSSIDs,aps[i].bssid) &&
          ignBSSIDs.size() < MAX_IGNORE)
        ignBSSIDs.push_back(aps[i].bssid);

      saveCfg();

      // Avoid stale short-term comparisons after changing the population.
      recentSurvival.clear();
      scan();
    }

    redir("/");
  });

  server.on("/ignore/del",HTTP_GET,[](){
    if (!requireAuth()) return;

    int i = server.arg("i").toInt();

    if (server.arg("t") == "b") {
      if (i >= 0 && i < (int)ignBSSIDs.size())
        ignBSSIDs.erase(ignBSSIDs.begin()+i);
    } else {
      if (i >= 0 && i < (int)ignSSIDs.size())
        ignSSIDs.erase(ignSSIDs.begin()+i);
    }

    saveCfg();
    recentSurvival.clear();
    scan();
    redir("/config");
  });

  server.on("/update",HTTP_POST,
    [](){
      if (!requireAuth()) return;

      bool ok = !Update.hasError();

      server.sendHeader("Connection","close");
      server.send(200,"text/html; charset=utf-8",
        ok
        ? (isGerman()
           ? "<html><body><h2>Update erfolgreich</h2><p>WiFiWatch startet neu...</p></body></html>"
           : "<html><body><h2>Update successful</h2><p>WiFiWatch is restarting...</p></body></html>")
        : (isGerman()
           ? "<html><body><h2>Update fehlgeschlagen</h2><p>Siehe Debug-Ausgabe, falls aktiviert.</p></body></html>"
           : "<html><body><h2>Update failed</h2><p>See debug output if enabled.</p></body></html>"));

      if (ok) {
        delay(800);
        ESP.restart();
      }
    },
    [](){
      if (!server.authenticate(cfg.webUser.c_str(),cfg.webPass.c_str()))
        return;

      HTTPUpload& up = server.upload();

      if (up.status == UPLOAD_FILE_START) {
        DBG_PRINTF("[OTA] Start: %s\n",up.filename.c_str());
        if (mqtt.connected()) pubTopic(topicSystem("status"),"updating");

        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
#if WIFIWATCH_DEBUG
          Update.printError(Serial);
#endif
        }
      }
      else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf,up.currentSize) != up.currentSize) {
#if WIFIWATCH_DEBUG
          Update.printError(Serial);
#endif
        }
      }
      else if (up.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
          DBG_PRINTF("[OTA] Success: %u bytes\n",up.totalSize);
        } else {
#if WIFIWATCH_DEBUG
          Update.printError(Serial);
#endif
        }
      }
      else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        DBG_PRINTLN("[OTA] Aborted");
      }
    }
  );

  server.begin();
}

// ---------- Ethernet events ----------
void event(WiFiEvent_t e) {
  if (e == ARDUINO_EVENT_ETH_START) {
    ETH.setHostname(cfg.hostname.c_str());
    DBG_PRINTLN("[ETH] started");
  }
  else if (e == ARDUINO_EVENT_ETH_CONNECTED) {
    DBG_PRINTLN("[ETH] link connected");
  }
  else if (e == ARDUINO_EVENT_ETH_GOT_IP) {
    eth = true;
    startMDNS();
    DBG_PRINTF("[ETH] IP: %s\n",ETH.localIP().toString().c_str());
    draw();

    // Refresh HA configuration_url after network/mDNS is available.
    if (mqtt.connected() && cfg.ha) {
      discovery();
    }
  }
  else if (e == ARDUINO_EVENT_ETH_DISCONNECTED) {
    eth = false;
    if (mdnsStarted) {
      MDNS.end();
      mdnsStarted = false;
    }
    DBG_PRINTLN("[ETH] disconnected");
    draw();
  }
}

// ---------- setup / loop ----------
void setup() {
  DBG_BEGIN(115200);
  delay(400);
  DBG_PRINTF("\nESP-WiFiWatch v0.4.8 [%s]\n", PLATFORM_NAME);

  display.init();
  display.setRotation(0);
  display.fillScreen(TFT_BLACK);
  display.setTextColor(TFT_CYAN,TFT_BLACK);
  display.setTextSize(3);
  display.setTextDatum(textdatum_t::middle_center);
  display.drawString("LERNPHASE",120,120);
  display.setTextDatum(textdatum_t::top_left);

  loadCfg();
  loadBaseline();

  mqtt.setServer(cfg.host.c_str(),cfg.port);
  mqtt.setBufferSize(4096);
  mqtt.setCallback(mqttCallback);

  WiFi.onEvent(event);
#if defined(WIFIWATCH_HW_WT32)
  ETH.begin(ETH_PHY_TYPE,
            ETH_PHY_ADDR,
            ETH_PHY_MDC,
            ETH_PHY_MDIO,
            ETH_PHY_POWER,
            ETH_CLK_MODE);
#elif defined(WIFIWATCH_HW_WAVESHARE_S3_ETH)
  ETH.begin(ETH_PHY_TYPE,
            ETH_PHY_ADDR,
            ETH_PHY_CS_PIN,
            ETH_PHY_IRQ_PIN,
            ETH_PHY_RST_PIN,
            SPI2_HOST,
            ETH_PHY_SCLK_PIN,
            ETH_PHY_MISO_PIN,
            ETH_PHY_MOSI_PIN);
#endif

  setupWeb();

  delay(800);
  scan();
  lastScan = millis();
}

void loop() {
  server.handleClient();

  mqConnect();
  if (mqtt.connected()) mqtt.loop();

  bool mqttNow = mqtt.connected();
  if (mqttNow != lastMqttConnected) {
    lastMqttConnected = mqttNow;
    draw();
  }

  if (millis() - lastScan >= scanIntervalMs()) {
    lastScan = millis();
    scan();
  }

  if (bootValidationActive &&
      millis() - bootValidationStartedMs >= BOOT_VALIDATION_MS) {
    bootValidationActive = false;
    publishAll();
  }

  maybeSaveBaseline();

  delay(2);
}
