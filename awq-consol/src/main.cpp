#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TFT_eSPI.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <time.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>

// Create separate SPI instances
SPIClass DisplaySPI(FSPI);  // SPI2 for display
SPIClass LoRaSPI(HSPI);     // SPI3 for LoRa

// Display SPI pins
#define DISPLAY_SCK  12
#define DISPLAY_MISO 13
#define DISPLAY_MOSI 11
#define DISPLAY_CS   7
#define TFT_BL      15

// LoRa SPI pins
#define LORA_SCK    36
#define LORA_MISO   37
#define LORA_MOSI   35
#define LORA_SS     17
#define LORA_RST    16 //18
#define LORA_DIO0   18 //16

// Display colors
#define BACKGROUND TFT_BLACK
#define TEXT_COLOR TFT_WHITE
#define VALUE_COLOR TFT_GREEN
#define ERROR_COLOR TFT_RED
#define TIME_COLOR TFT_CYAN
#define PH_COLOR TFT_BLUE
#define EC_COLOR 0xFD20
#define DO_COLOR 0xA0FA
#define TDS_COLOR 0xFD20
#define TURBIDITY_COLOR 0xCCCC
#define BOD_COLOR 0xE8E4
#define COD_COLOR 0xACB7
#define SALINITY_COLOR 0x07FF
#define TEMP_COLOR 0xFD20
#define VOLTAGE_COLOR 0xACB7
#define RAW_COLOR 0xE8E4
#define HUMIDITY_COLOR TFT_CYAN
#define PRESSURE_COLOR TFT_MAGENTA
#define AIR_TEMP_COLOR TFT_ORANGE
#define TIDE_COLOR TFT_YELLOW

// Timing constants
#define DISPLAY_UPDATE_INTERVAL 2000
#define DEBUG_MSG_INTERVAL 30000
#define NTP_UPDATE_INTERVAL 3600000 // 1 hour
#define SERVER_UPDATE_INTERVAL 30000  // Send data every 30 seconds
#define BOD_UPDATE_INTERVAL 60000    // Check for BOD calculation every minute

// =========================================================
// EEPROM Configuration (Web AP feature)
// =========================================================
#define EEPROM_SIZE         512
#define ADDR_WIFI_SSID      10
#define ADDR_WIFI_PASS      42
#define ADDR_AP_PASS        74
#define ADDR_SERVER_HOST    106
#define ADDR_SERVER_PATH    170
#define ADDR_SERVER_PORT    234
#define ADDR_UPDATE_INTERVAL 298
#define MAX_CRED_LENGTH     32
#define MAX_SERVER_LENGTH   64

// AP identity
const char* ap_ssid   = "AWQ-Console";
String ap_password    = "";
String sta_ssid       = "";
String sta_password   = "";

// Server configuration (loaded from EEPROM; empty until configured via Web AP)
String serverHost              = "";
String serverPath              = "";
int    serverPort              = 0;
unsigned long serverUpdateInterval = SERVER_UPDATE_INTERVAL;
bool   serverConnected         = false;
unsigned long lastServerSend   = 0;
unsigned long lastServerAttempt = 0;
String lastServerStatus        = "Not configured";

// NTP Server settings
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const char* ntpServer3 = "id.pool.ntp.org";
const long  gmtOffset_sec      = 25200;  // GMT+7
const int   daylightOffset_sec = 0;

// Web server
WebServer server(80);

// WiFi configuration (loaded from EEPROM; hardcoded fallbacks)
String wifi_ssid_str  = "";
String wifi_pass_str  = "";
bool wifiConfigured   = false;

// NTP Time
bool timeSynchronized = false;
unsigned long lastNTPUpdate = 0;

// Debug settings
#define DEBUG_CRC true
#define DEBUG_PACKET_DATA true
#define DEBUG_LORA true

// Create display instance
TFT_eSPI tft = TFT_eSPI();

// Data structure for water quality sensor readings
struct WaterQualityData {
  // Sensor values (calculated)
  float pH;
  float ec;               // Electrical conductivity in µS/cm
  float do_level;         // Dissolved oxygen in mg/L
  float tds;              // Total dissolved solids in ppm
  float turbidity;        // Turbidity in NTU
  float temperature;      // Temperature in °C
  float salinity;         // Calculated salinity in ppt
  float tide_level;       // New: Tide level in cm
  
  // New BOD/COD fields
  float do_saturation;    // DO saturation in mg/L
  float k;                // Reaction rate constant
  float bod1;             // 1-day biochemical oxygen demand
  float bod5;             // 5-day biochemical oxygen demand
  float cod;              // Chemical oxygen demand
  
  // BME280 Environmental data
  float humidity;         // Humidity in %
  float pressure;         // Pressure in hPa
  float airTemperature;   // Air temperature in °C
  bool bmeReady;          // BME280 sensor status
  
  // Raw voltage values (if sent)
  float phVoltage;        // pH sensor voltage in mV
  float ecVoltage;        // EC sensor voltage in mV
  float doVoltage;        // DO sensor voltage in mV
  float tdsVoltage;       // TDS sensor voltage in V
  float turbidityVoltage; // Turbidity sensor voltage in V
  
  // Metadata
  int deviceId;
  int rssi;
  unsigned long timestamp;
  bool isValid;
  uint8_t sequence;
  bool hasRawData;        // Flag indicating if raw voltage data is available
  bool hasBME280;         // Flag indicating if BME280 data is available
  bool hasTideData;       // New: Flag indicating if tide data is available
};

// Packet counters for debugging
unsigned long receivedPacketsTotal = 0;
unsigned long crcErrors = 0;

// Global variables
WaterQualityData qualityData = {0};
unsigned long lastQualityDataTime = 0;
unsigned long startupTime = 0;
bool loraInitialized = false;
bool layoutInitialized = false;
unsigned long packetCounter = 0;
bool bodCalculated = false;
unsigned long lastBODCheck = 0;

// Function prototypes
void initializeDisplay();
void initializeDisplayLayout();
void initializeWiFi();
void checkWiFiConnection();
void setupWebServer();
void handleRoot();
void handleReadings();
void handleSaveConfig();
void handleResetConfig();
// Web AP handlers
void handleWiFiConfig();
void handleSaveSTA();
void handleSaveAP();
void handleSaveServer();
void handleOTAUpdate();
void handleOTAUpload();
String generatePasswordField(const String& id, const String& value, const String& label);
// EEPROM helpers
void readWiFiCredentials();
void readAPPassword();
void readServerConfig();
void saveWiFiCredentials(String ssid, String password, String ap_pass = "");
void saveServerConfig(String host, String path, int port, unsigned long interval);
void configureLoRa();
void updateDisplay();
uint16_t calculateCRC16(uint8_t *data, size_t length);
void flashReceivedIndicator(int x, int y, uint16_t color);
void processQualityPacket(int packetSize, int rssi);
void processQualityPacket26Bytes(int packetSize, int rssi);
void processQualityPacketWithBME280(int packetSize, int rssi);
void processQualityPacketWithRawData(int packetSize, int rssi);
void processQualityPacket38Bytes(int packetSize, int rssi);
void checkForPacketsWithTimeout();
void checkLoRaStatus();
void checkWiFiStatus();
void printPacketHex(uint8_t *packet, int length);
float calculateSalinity(float ec, float temperature);
void calculateBOD();
float calculateCOD(float bod5, float temperature);
void initializeNTP();
void updateNTPTime();
String getFormattedTime();
String getFormattedDateTime();
unsigned long getCurrentEpochTime();
void sendDataToServer();
void updateSensorDataDisplay();

// Calculate Chemical Oxygen Demand (COD) from BOD and temperature
float calculateCOD(float bod5, float temperature) {
  return 0.1011 + (6.7610 * bod5) - (0.0204 * temperature);
}

// Calculate BOD values - MODIFIED for immediate calculation
void calculateBOD() {
  if (!qualityData.isValid || qualityData.do_level <= 0) {
    Serial.println("BOD Calculation: Invalid data or DO level");
    return;
  }
  
  float currentDO = qualityData.do_level;
  float temp = qualityData.temperature > 0 ? qualityData.temperature : 25.0;
  
  // Calculate DO saturation based on temperature
  qualityData.do_saturation = 14.259 * exp(-0.022 * temp);
  
  // Use default k value
  qualityData.k = 0.4; // Fixed k value
  
  // Calculate BOD values using k and DO deficit
  float doDeficit = qualityData.do_saturation - currentDO;
  if (doDeficit < 0) doDeficit = 0; // No deficit if DO exceeds saturation
  
  // BOD calculation: BOD = DO deficit / k
  qualityData.bod1 = doDeficit / qualityData.k;
  qualityData.bod5 = qualityData.bod1 * 1.5; // Typical BOD5/BOD1 ratio
  
  // Calculate COD values using the specified formula
  qualityData.cod = calculateCOD(qualityData.bod5, temp);
  
  // Constrain to reasonable values
  qualityData.bod1 = constrain(qualityData.bod1, 0.1, 50.0);
  qualityData.bod5 = constrain(qualityData.bod5, 0.2, 75.0);
  qualityData.cod = constrain(qualityData.cod, 0.5, 100.0);
  
  bodCalculated = true;
  
  Serial.println("=== BOD and COD CALCULATED ===");
  Serial.print("  Temperature: "); 
  Serial.print(temp, 1); 
  Serial.println("°C");
  
  Serial.print("  DO Measured: "); 
  Serial.print(currentDO, 2); 
  Serial.println(" mg/L");
  
  Serial.print("  DO Saturation: "); 
  Serial.print(qualityData.do_saturation, 2); 
  Serial.println(" mg/L");
  
  Serial.print("  DO Deficit: "); 
  Serial.print(doDeficit, 2); 
  Serial.println(" mg/L");
  
  Serial.print("  BOD₁: "); 
  Serial.print(qualityData.bod1, 2); 
  Serial.println(" mg/L");
  
  Serial.print("  BOD₅: "); 
  Serial.print(qualityData.bod5, 2); 
  Serial.println(" mg/L");
  
  Serial.print("  COD: "); 
  Serial.print(qualityData.cod, 2); 
  Serial.println(" mg/L");
  Serial.println("==============================");
}

// Calculate salinity from EC and temperature (in ppt)
float calculateSalinity(float ec, float temperature) {
  if (ec <= 0) return 0;
  
  // Convert to salinity in ppt
  float salinity = ec * 0.64/1000;
  return salinity;
}

// Initialize NTP time client
void initializeNTP() {
  if (wifiConfigured) {
    Serial.println("Initializing NTP time client...");
    
    // Configure multiple NTP servers with timezone
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
    
    // Force immediate time update
    updateNTPTime();
  }
}

// Update NTP time
void updateNTPTime() {
  if (wifiConfigured && (millis() - lastNTPUpdate > NTP_UPDATE_INTERVAL || lastNTPUpdate == 0)) {
    Serial.println("Updating NTP time...");
    
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 5000)) { // 5 second timeout
      timeSynchronized = true;
      lastNTPUpdate = millis();
      Serial.println("NTP time synchronized: " + getFormattedDateTime());
    } else {
      Serial.println("NTP time update failed");
      timeSynchronized = false;
    }
  }
}

// Get current epoch time (with fallback)
unsigned long getCurrentEpochTime() {
  if (timeSynchronized) {
    time_t now;
    time(&now);
    return now;
  } else {
    return millis(); // Fallback to relative time
  }
}

// Get formatted time string
String getFormattedTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) {
    return "N/A";
  }
  
  char buffer[9];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
  return String(buffer);
}

// Get formatted date and time string
String getFormattedDateTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) {
    return "N/A";
  }
  
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

// Initialize display
void initializeDisplay() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
  DisplaySPI.begin(DISPLAY_SCK, DISPLAY_MISO, DISPLAY_MOSI, DISPLAY_CS);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(BACKGROUND);
  
  Serial.println("Display initialized");
}

// Initialize the static display layout
void initializeDisplayLayout() {
  tft.fillScreen(BACKGROUND);
  
  // Top title bar
  tft.fillRect(0, 0, tft.width(), 25, TFT_NAVY);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(5, 8);
  tft.print("Automatic Water Quality");
  
  // Status bar
  tft.fillRect(0, 25, tft.width(), 20, TFT_DARKGREY);
  tft.setTextSize(1);
  tft.setCursor(5, 30);
  tft.setTextColor(TEXT_COLOR);
  tft.print("LoRa:");
  tft.setCursor(80, 30);
  tft.print("Updated:");
  tft.setCursor(180, 30);
  tft.print("Pkts:");
  tft.setCursor(240, 30);
  tft.print("WiFi:");
  
  // Node 2 header
  tft.fillRect(0, 45, tft.width(), 20, TFT_NAVY);
  tft.setTextColor(TEXT_COLOR);
  tft.setCursor(5, 50);
  tft.print("WATER QUALITY SENSOR DATA");
  
  // Footer
  tft.fillRect(0, tft.height()-20, tft.width(), 20, TFT_NAVY);
  tft.setCursor(5, tft.height()-15);
  tft.print("Water Quality Monitoring System");
  
  layoutInitialized = true;
}

// =========================================================
// EEPROM helper functions
// =========================================================
void readWiFiCredentials() {
  char ssid_buf[MAX_CRED_LENGTH] = {0};
  char pass_buf[MAX_CRED_LENGTH] = {0};
  for (int i = 0; i < MAX_CRED_LENGTH; i++) {
    ssid_buf[i] = EEPROM.read(ADDR_WIFI_SSID + i);
    pass_buf[i] = EEPROM.read(ADDR_WIFI_PASS + i);
  }
  String s = String(ssid_buf); s.trim();
  String p = String(pass_buf); p.trim();
  if (s.length() > 0) { wifi_ssid_str = s; wifi_pass_str = p; }
}

void readAPPassword() {
  char buf[MAX_CRED_LENGTH] = {0};
  bool valid = false;
  for (int i = 0; i < MAX_CRED_LENGTH; i++) {
    buf[i] = EEPROM.read(ADDR_AP_PASS + i);
    if (buf[i] != 0 && buf[i] != 255) valid = true;
  }
  ap_password = String(buf); ap_password.trim();
  if (!valid || ap_password.isEmpty()) {
    ap_password = "12345678";
    for (int i = 0; i < (int)ap_password.length(); i++)
      EEPROM.write(ADDR_AP_PASS + i, ap_password[i]);
    EEPROM.commit();
  }
}

void readServerConfig() {
  char host_buf[MAX_SERVER_LENGTH] = {0};
  char path_buf[MAX_SERVER_LENGTH] = {0};
  for (int i = 0; i < MAX_SERVER_LENGTH; i++) {
    host_buf[i] = EEPROM.read(ADDR_SERVER_HOST + i);
    path_buf[i] = EEPROM.read(ADDR_SERVER_PATH + i);
  }
  String h = String(host_buf); h.trim();
  String p = String(path_buf); p.trim();
  int port = EEPROM.read(ADDR_SERVER_PORT) | (EEPROM.read(ADDR_SERVER_PORT + 1) << 8);

  // Read update interval
  unsigned long interval = 0;
  for (int i = 0; i < 4; i++)
    interval |= ((unsigned long)EEPROM.read(ADDR_UPDATE_INTERVAL + i) << (i * 8));
  if (interval != 0xFFFFFFFF && interval >= 5000 && interval <= 86400000)
    serverUpdateInterval = interval;

  // Only override hardcoded defaults if EEPROM values are valid
  if (h.length() > 0 && h.indexOf('.') != -1) serverHost = h;
  if (p.length() > 0 && p.charAt(0) == '/') serverPath = p;
  if (port > 0 && port <= 65535) serverPort = port;
}

void saveWiFiCredentials(String ssid, String password, String ap_pass) {
  for (int i = 0; i < MAX_CRED_LENGTH; i++) {
    EEPROM.write(ADDR_WIFI_SSID + i, 0);
    EEPROM.write(ADDR_WIFI_PASS + i, 0);
    if (!ap_pass.isEmpty()) EEPROM.write(ADDR_AP_PASS + i, 0);
  }
  for (int i = 0; i < (int)ssid.length() && i < MAX_CRED_LENGTH; i++)
    EEPROM.write(ADDR_WIFI_SSID + i, ssid[i]);
  for (int i = 0; i < (int)password.length() && i < MAX_CRED_LENGTH; i++)
    EEPROM.write(ADDR_WIFI_PASS + i, password[i]);
  if (!ap_pass.isEmpty())
    for (int i = 0; i < (int)ap_pass.length() && i < MAX_CRED_LENGTH; i++)
      EEPROM.write(ADDR_AP_PASS + i, ap_pass[i]);
  EEPROM.commit();
  wifi_ssid_str = ssid;
  wifi_pass_str = password;
}

void saveServerConfig(String host, String path, int port, unsigned long interval) {
  for (int i = 0; i < MAX_SERVER_LENGTH; i++) {
    EEPROM.write(ADDR_SERVER_HOST + i, 0);
    EEPROM.write(ADDR_SERVER_PATH + i, 0);
  }
  for (int i = 0; i < (int)host.length() && i < MAX_SERVER_LENGTH; i++)
    EEPROM.write(ADDR_SERVER_HOST + i, host[i]);
  for (int i = 0; i < (int)path.length() && i < MAX_SERVER_LENGTH; i++)
    EEPROM.write(ADDR_SERVER_PATH + i, path[i]);
  EEPROM.write(ADDR_SERVER_PORT,     port & 0xFF);
  EEPROM.write(ADDR_SERVER_PORT + 1, (port >> 8) & 0xFF);
  for (int i = 0; i < 4; i++)
    EEPROM.write(ADDR_UPDATE_INTERVAL + i, (interval >> (i * 8)) & 0xFF);
  EEPROM.commit();
  serverHost = host; serverPath = path;
  serverPort = port; serverUpdateInterval = interval;
  Serial.printf("Server config saved -> %s:%d%s  Interval:%lus\n",
                host.c_str(), port, path.c_str(), interval / 1000);
}

// =========================================================
// Initialize WiFi — uses EEPROM credentials (with hardcoded fallback)
// =========================================================
void initializeWiFi() {
  WiFi.mode(WIFI_AP_STA);

  // Start AP
  if (!WiFi.softAP(ap_ssid, ap_password.c_str()))
    WiFi.softAP(ap_ssid, "12345678");
  Serial.println("AP started: " + String(ap_ssid) + " | IP: " + WiFi.softAPIP().toString());

  // Connect STA
  if (wifi_ssid_str.length() > 0) {
    Serial.println("Connecting to WiFi: " + wifi_ssid_str);
    WiFi.begin(wifi_ssid_str.c_str(), wifi_pass_str.c_str());
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
      delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiConfigured = true;
      Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
    } else {
      Serial.println("\nWiFi connection failed.");
    }
  }
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 30000) return;
  lastCheck = millis();
  if (WiFi.status() != WL_CONNECTED) {
    wifiConfigured = false;
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) {
      delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiConfigured = true;
      Serial.println("\nWiFi reconnected!");
      if (!timeSynchronized) initializeNTP();
    }
  }
}

// =========================================================
// Web AP — Helper
// =========================================================
String generatePasswordField(const String& id, const String& value, const String& label) {
  String html  = "<label for='" + id + "'>" + label + ":</label>";
  html += "<div style='display:flex;align-items:center;'>";
  html += "<input type='password' id='" + id + "' name='" + id + "' value='" + value + "' style='flex-grow:1;'>";
  html += "<button type='button' onclick=\"togglePassword('" + id + "')\" style='margin-left:5px;'>Show</button>";
  html += "</div>";
  return html;
}

// =========================================================
// Web AP — Root page (live sensor readings)
// =========================================================
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Water Quality RX Monitor</title>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:700px;margin:0 auto;padding:20px;}";
  html += ".reading{background:#f0f0f0;padding:16px;margin:16px 0;border-radius:8px;}";
  html += "table{width:100%;border-collapse:collapse;}td,th{padding:8px;text-align:left;border:1px solid #ddd;}";
  html += ".button{background-color:#4CAF50;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;margin:5px;}";
  html += ".status{padding:10px;margin:10px 0;border-radius:4px;}.error{background:#f2dede;color:#a94442;}.info{background:#d9edf7;color:#31708f;}";
  html += "</style>";
  html += "<script>function upd(){fetch('/readings').then(r=>r.text()).then(d=>{document.getElementById('rd').innerHTML=d;}).catch(e=>console.log(e));} setInterval(upd,3000);window.onload=upd;</script>";
  html += "</head><body>";
  html += "<h1>Water Quality RX Monitor</h1>";
  html += "<p style='text-align:center;color:#666;font-size:0.9em;'>AP: " + String(ap_ssid) + " | STA: " + (wifiConfigured ? WiFi.localIP().toString() : "Disconnected") + "</p>";

  if (serverHost.isEmpty() || serverPath.isEmpty() || serverPort <= 0)
    html += "<div class='status error'><strong>Server not configured!</strong> Go to <a href='/wifi-config'>Configuration</a>.</div>";

  html += "<div id='rd'><div class='reading'><p>Loading data...</p></div></div>";
  html += "<div class='reading'><h2>Configuration</h2>";
  html += "<p><a href='/wifi-config' class='button'>WiFi, AP, Server &amp; OTA Settings</a></p></div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// =========================================================
// /readings — live JSON-formatted table (polled by root page)
// =========================================================
void handleReadings() {
  String html = "<div class='reading'><h3>Sensor Data</h3><table>";
  html += "<tr><td>Device ID:</td><td>" + String(qualityData.deviceId) + "</td></tr>";
  if (qualityData.isValid) {
    html += "<tr><td>Tide Level (cm):</td><td>" + (qualityData.hasTideData ? String(qualityData.tide_level, 1) : "N/A") + "</td></tr>";
    html += "<tr><td>pH:</td><td>" + String(qualityData.pH, 2) + "</td></tr>";
    html += "<tr><td>EC (µS/cm):</td><td>" + String(qualityData.ec, 0) + "</td></tr>";
    html += "<tr><td>DO (mg/L):</td><td>" + String(qualityData.do_level, 2) + "</td></tr>";
    html += "<tr><td>TDS (ppm):</td><td>" + String(qualityData.tds, 0) + "</td></tr>";
    html += "<tr><td>Temperature (°C):</td><td>" + String(qualityData.temperature, 2) + "</td></tr>";
    if (qualityData.hasRawData) {
      html += "<tr><th colspan='2' style='background:#ddd;'>Raw Voltages</th></tr>";
      html += "<tr><td>pH Voltage (mV):</td><td>" + String(qualityData.phVoltage, 0) + "</td></tr>";
      html += "<tr><td>EC Voltage (mV):</td><td>" + String(qualityData.ecVoltage, 0) + "</td></tr>";
      html += "<tr><td>DO Voltage (mV):</td><td>" + String(qualityData.doVoltage, 0) + "</td></tr>";
      html += "<tr><td>TDS Voltage (V):</td><td>" + String(qualityData.tdsVoltage, 3) + "</td></tr>";
    }
    html += "<tr><td>RSSI (dBm):</td><td>" + String(qualityData.rssi) + "</td></tr>";
    html += "<tr><td>Last Update:</td><td>" + String((millis() - lastQualityDataTime) / 1000) + "s ago</td></tr>";
  } else {
    html += "<tr><td colspan='2' style='color:red;'>No valid data received yet</td></tr>";
  }
  html += "</table></div>";

  html += "<div class='reading'><h3>Server Status</h3><table>";
  html += "<tr><td>Host:</td><td>" + (serverHost.isEmpty() ? "Not configured" : serverHost) + "</td></tr>";
  html += "<tr><td>Path:</td><td>" + (serverPath.isEmpty() ? "Not configured" : serverPath) + "</td></tr>";
  html += "<tr><td>Port:</td><td>" + (serverPort == 0 ? "Not configured" : String(serverPort)) + "</td></tr>";
  html += "<tr><td>Interval (s):</td><td>" + String(serverUpdateInterval / 1000) + "</td></tr>";
  html += "<tr><td>Status:</td><td>" + String(serverConnected ? "Connected" : "Disconnected") + "</td></tr>";
  html += "<tr><td>Last Status:</td><td>" + lastServerStatus + "</td></tr>";
  if (lastServerSend > 0)
    html += "<tr><td>Last Send:</td><td>" + String((millis() - lastServerSend) / 1000) + "s ago</td></tr>";
  html += "</table></div>";

  html += "<div class='reading'><h3>WiFi Status</h3><table>";
  html += "<tr><td>STA:</td><td>" + String(wifiConfigured ? "Connected" : "Disconnected") + "</td></tr>";
  if (wifiConfigured) {
    html += "<tr><td>IP:</td><td>" + WiFi.localIP().toString() + "</td></tr>";
    html += "<tr><td>RSSI:</td><td>" + String(WiFi.RSSI()) + " dBm</td></tr>";
  }
  html += "<tr><td>AP SSID:</td><td>" + String(ap_ssid) + "</td></tr>";
  html += "<tr><td>AP IP:</td><td>" + WiFi.softAPIP().toString() + "</td></tr>";
  html += "<tr><td>AP Clients:</td><td>" + String(WiFi.softAPgetStationNum()) + "</td></tr>";
  html += "</table></div>";
  server.send(200, "text/html", html);
}

// =========================================================
// Web AP — Configuration page (tabbed: STA / AP / Server / OTA)
// =========================================================
void handleWiFiConfig() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Configuration</title>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:600px;margin:0 auto;padding:20px;}";
  html += ".config-form{background:#f0f0f0;padding:20px;margin:20px 0;border-radius:8px;}";
  html += "input[type='text'],input[type='password'],input[type='number'],select{width:100%;padding:8px;margin:8px 0;box-sizing:border-box;}";
  html += ".button{background-color:#4CAF50;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;margin:5px;}";
  html += ".button.secondary{background-color:#2196F3;}.button.danger{background-color:#f44336;}.button.server{background-color:#9C27B0;}";
  html += ".status{padding:10px;margin:10px 0;border-radius:4px;}.success{background:#dff0d8;color:#3c763d;}";
  html += ".info{background:#d9edf7;color:#31708f;}.error{background:#f2dede;color:#a94442;}";
  html += ".tab{overflow:hidden;border:1px solid #ccc;background-color:#f1f1f1;}";
  html += ".tab button{background-color:inherit;float:left;border:none;outline:none;cursor:pointer;padding:10px 16px;transition:0.3s;}";
  html += ".tab button:hover{background-color:#ddd;}.tab button.active{background-color:#ccc;}";
  html += ".tabcontent{display:none;padding:6px 12px;border:1px solid #ccc;border-top:none;}";
  html += ".network-info{background:white;padding:15px;margin:10px 0;border-radius:5px;border:1px solid #ddd;}</style>";
  html += "<script>";
  html += "function togglePassword(id){var x=document.getElementById(id);if(x.type==='password'){x.type='text';}else{x.type='password';}}";
  html += "function openTab(evt,tabName){var i,tc,tb;tc=document.getElementsByClassName('tabcontent');for(i=0;i<tc.length;i++){tc[i].style.display='none';}tb=document.getElementsByClassName('tabbutton');for(i=0;i<tb.length;i++){tb[i].className=tb[i].className.replace(' active','');}document.getElementById(tabName).style.display='block';evt.currentTarget.className+=' active';}";
  html += "</script></head><body>";
  html += "<h1>Configuration</h1>";

  if      (server.hasArg("sta_saved"))    html += "<div class='status success'>WiFi configuration saved!</div>";
  else if (server.hasArg("ap_saved"))     html += "<div class='status success'>AP configuration saved!</div>";
  else if (server.hasArg("server_saved")) html += "<div class='status success'>Server configuration saved!</div>";

  html += "<div class='tab'>";
  html += "<button class='tabbutton active' onclick=\"openTab(event,'STA')\">Connect to WiFi</button>";
  html += "<button class='tabbutton' onclick=\"openTab(event,'AP')\">AP Settings</button>";
  html += "<button class='tabbutton' onclick=\"openTab(event,'SERVER')\">Server Settings</button>";
  html += "<button class='tabbutton' onclick=\"openTab(event,'OTA')\">Firmware Update</button>";
  html += "</div>";

  // STA tab
  html += "<div id='STA' class='tabcontent' style='display:block;'>";
  html += "<h2>WiFi Connection</h2><div class='network-info'><h3>Current Status</h3>";
  if (wifiConfigured) {
    html += "<p><strong>Connected to:</strong> " + wifi_ssid_str + "</p>";
    html += "<p><strong>IP:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>";
  } else {
    html += "<p><strong>Status:</strong> Not connected</p>";
    if (wifi_ssid_str.length() > 0)
      html += "<p><strong>Last SSID:</strong> " + wifi_ssid_str + "</p>";
  }
  html += "</div>";
  html += "<form action='/save-sta' method='post' class='config-form'>";
  html += "<h3>Configure WiFi</h3>";
  html += "<label>WiFi SSID:</label><input type='text' name='sta_ssid' value='" + wifi_ssid_str + "' required placeholder='Enter WiFi SSID'>";
  html += generatePasswordField("sta_password", wifi_pass_str, "WiFi Password");
  html += "<div style='text-align:center;'><input type='submit' value='Save &amp; Connect' class='button'></div>";
  html += "</form></div>";

  // AP tab
  html += "<div id='AP' class='tabcontent'>";
  html += "<form action='/save-ap' method='post' class='config-form'>";
  html += "<h3>Access Point (AP) Configuration</h3>";
  html += "<label>AP SSID:</label><input type='text' name='ap_ssid' value='" + String(ap_ssid) + "' required>";
  html += generatePasswordField("ap_password", ap_password, "AP Password");
  html += "<p><small>AP password must be at least 8 characters</small></p>";
  html += "<input type='submit' value='Save AP Configuration' class='button'>";
  html += "</form></div>";

  // Server tab
  html += "<div id='SERVER' class='tabcontent'>";
  html += "<form action='/save-server' method='post' class='config-form'>";
  html += "<h3>Server Configuration</h3>";
  html += "<label>Server Host:</label><input type='text' name='server_host' value='" + serverHost + "' required placeholder='e.g., www.example.com'>";
  html += "<label>Server Path:</label><input type='text' name='server_path' value='" + serverPath + "' required placeholder='e.g., /api/data.php'>";
  html += "<label>Server Port:</label><input type='number' name='server_port' value='" + String(serverPort == 0 ? 443 : serverPort) + "' min='1' max='65535' required>";
  html += "<p><small>Common ports: 80 (HTTP), 443 (HTTPS)</small></p>";
  html += "<label>Update Interval (seconds):</label><input type='number' name='update_interval' value='" + String(serverUpdateInterval / 1000) + "' min='5' max='86400' required>";
  html += "<div style='background:#f8e8f8;padding:10px;margin:10px 0;border-radius:5px;'>";
  html += "<p><strong>Current:</strong> " + (serverHost.isEmpty() ? "Not set" : serverHost + ":" + String(serverPort) + serverPath) + "</p>";
  html += "<p><strong>Interval:</strong> " + String(serverUpdateInterval / 1000) + " seconds</p>";
  html += "</div>";
  html += "<div style='text-align:center;'><input type='submit' value='Save Server Configuration' class='button server'></div>";
  html += "</form></div>";

  // OTA tab
  html += "<div id='OTA' class='tabcontent'>";
  html += "<div style='text-align:center;margin:20px 0;'>";
  html += "<h3>Firmware Update</h3><p>Update device firmware via .bin file</p>";
  html += "<a href='/ota-update' class='button danger'>Update Firmware</a>";
  html += "<div class='status info' style='margin-top:20px;'><p>Use only the correct .bin file for this device.</p></div>";
  html += "</div></div>";

  html += "<p><a href='/' class='button'>Return to Main Page</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSaveConfig()  { server.sendHeader("Location", "/"); server.send(303); }
void handleResetConfig() { server.sendHeader("Location", "/"); server.send(303); }

void handleSaveSTA() {
  if (server.hasArg("sta_ssid") && server.arg("sta_ssid").length() > 0) {
    saveWiFiCredentials(server.arg("sta_ssid"), server.arg("sta_password"), "");
    WiFi.disconnect();
    delay(500);
    initializeWiFi();
    server.sendHeader("Location", "/wifi-config?sta_saved=1");
    server.send(303);
    return;
  }
  server.sendHeader("Location", "/wifi-config");
  server.send(303);
}

void handleSaveAP() {
  if (server.hasArg("ap_ssid") && server.hasArg("ap_password")) {
    String newSSID = server.arg("ap_ssid");
    String newPass = server.arg("ap_password");
    if (newSSID.length() > 0 && newPass.length() >= 8) {
      saveWiFiCredentials("", "", newPass);
      WiFi.softAPdisconnect(true);
      delay(500);
      WiFi.softAP(newSSID.c_str(), newPass.c_str());
      server.sendHeader("Location", "/wifi-config?ap_saved=1");
      server.send(303);
      return;
    }
  }
  server.sendHeader("Location", "/wifi-config");
  server.send(303);
}

void handleSaveServer() {
  if (server.hasArg("server_host") && server.hasArg("server_path") &&
      server.hasArg("server_port") && server.hasArg("update_interval")) {
    String newHost     = server.arg("server_host");
    String newPath     = server.arg("server_path");
    int    newPort     = server.arg("server_port").toInt();
    unsigned long newInterval = server.arg("update_interval").toInt() * 1000;
    if (newHost.length() > 0 && newPath.length() > 0 && newPath.charAt(0) == '/' &&
        newPort > 0 && newPort <= 65535 && newInterval >= 5000 && newInterval <= 86400000) {
      saveServerConfig(newHost, newPath, newPort, newInterval);
      server.sendHeader("Location", "/wifi-config?server_saved=1");
      server.send(303);
      return;
    }
  }
  server.sendHeader("Location", "/wifi-config");
  server.send(303);
}

// =========================================================
// OTA Update handlers
// =========================================================
void handleOTAUpdate() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>Firmware Update</title>";
  html += "<style>body{font-family:Arial,sans-serif;max-width:600px;margin:0 auto;padding:20px;}";
  html += ".update-form{background:#f0f0f0;padding:20px;margin:20px 0;border-radius:8px;}";
  html += ".button{background-color:#4CAF50;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;margin:5px;}";
  html += ".button.danger{background-color:#f44336;}";
  html += ".status{padding:10px;margin:10px 0;border-radius:4px;}.warning{background:#fcf8e3;color:#8a6d3b;}.info{background:#d9edf7;color:#31708f;}";
  html += "#progress{width:100%;height:30px;background:#f0f0f0;border-radius:5px;overflow:hidden;margin:10px 0;}";
  html += "#progress-bar{height:100%;background:#4CAF50;width:0%;transition:width 0.3s;text-align:center;line-height:30px;color:white;}</style>";
  html += "<script>";
  html += "function uploadFirmware(){";
  html += "  var file=document.getElementById('firmware').files[0];";
  html += "  if(!file){alert('Please select a firmware file');return;}";
  html += "  if(!file.name.endsWith('.bin')){alert('Please select a .bin file');return;}";
  html += "  if(!confirm('Update firmware? Device will restart.'))return;";
  html += "  var formData=new FormData();";
  html += "  formData.append('file',file);";
  html += "  var xhr=new XMLHttpRequest();";
  html += "  document.getElementById('status').innerHTML='Uploading...';";
  html += "  document.getElementById('progress').style.display='block';";
  html += "  xhr.upload.addEventListener('progress',function(e){";
  html += "    if(e.lengthComputable){";
  html += "      var pct=(e.loaded/e.total)*100;";
  html += "      document.getElementById('progress-bar').style.width=pct+'%';";
  html += "      document.getElementById('progress-bar').innerHTML=Math.round(pct)+'%';";
  html += "    }";
  html += "  });";
  html += "  xhr.addEventListener('load',function(){";
  html += "    if(xhr.status===200){";
  html += "      document.getElementById('status').innerHTML='Update successful! Device restarting...';";
  html += "      document.getElementById('status').className='status info';";
  html += "      setTimeout(function(){window.location.href='/';},10000);";
  html += "    }else{";
  html += "      document.getElementById('status').innerHTML='Update failed: '+xhr.responseText;";
  html += "      document.getElementById('status').className='status warning';";
  html += "    }";
  html += "  });";
  html += "  xhr.addEventListener('error',function(){";
  html += "    document.getElementById('status').innerHTML='Upload error occurred';";
  html += "    document.getElementById('status').className='status warning';";
  html += "  });";
  html += "  xhr.open('POST','/ota-upload');";
  html += "  xhr.send(formData);";
  html += "}";
  html += "</script>";
  html += "</head><body>";
  html += "<h1>Firmware Update</h1>";
  html += "<div class='status warning'><strong>Warning:</strong> Only upload firmware from authorized technician. Do not disconnect power during update.</div>";
  html += "<div class='update-form'><h3>Upload New Firmware</h3>";
  html += "<input type='file' id='firmware' accept='.bin' style='margin:10px 0;'><br>";
  html += "<button onclick='uploadFirmware()' class='button danger'>Upload &amp; Update</button>";
  html += "<div id='progress' style='display:none;'><div id='progress-bar'>0%</div></div>";
  html += "<div id='status' class='status info' style='margin-top:10px;'>Select a .bin file and click Upload</div>";
  html += "</div>";
  html += "<p><a href='/' class='button'>Return to Main Page</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleOTAUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA Start: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    } else {
      if (Update.size() > 0)
        Serial.printf("OTA Progress: %d%%\r", (Update.progress() * 100) / Update.size());
      else
        Serial.printf("OTA Written: %u bytes\r", Update.progress());
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("\nOTA Success: %u bytes. Rebooting...\n", upload.totalSize);
      delay(500);
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
}

// =========================================================
// Web server setup — ENABLED with full Web AP routes
// =========================================================
void setupWebServer() {
  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/readings",   HTTP_GET,  handleReadings);
  server.on("/wifi-config",HTTP_GET,  handleWiFiConfig);
  server.on("/save-sta",   HTTP_POST, handleSaveSTA);
  server.on("/save-ap",    HTTP_POST, handleSaveAP);
  server.on("/save-server",HTTP_POST, handleSaveServer);
  server.on("/ota-update", HTTP_GET,  handleOTAUpdate);
  server.on("/ota-upload", HTTP_POST,
    []() { /* OTA upload handled via ESP.restart() in handleOTAUpload */ },
    handleOTAUpload
  );
  server.begin();
  Serial.println("Web server started. AP: " + String(ap_ssid) + " | IP: " + WiFi.softAPIP().toString());
  Serial.println("OTA available at: /ota-update");
}

// Configure LoRa parameters
void configureLoRa() {
  LoRa.setSpreadingFactor(7);          // Match transmitter
  LoRa.setSignalBandwidth(250E2);      // Match transmitter
  LoRa.setCodingRate4(5);              // Match transmitter
  LoRa.setSyncWord(0x91);              // Match transmitter
  LoRa.setPreambleLength(8);           // Match transmitter
  LoRa.setTxPower(17);                 // Match transmitter
  LoRa.enableCrc();
  LoRa.receive();
  
  if (DEBUG_LORA) {
    Serial.println("LoRa configured with parameters:");
    Serial.println("- Spreading Factor: 7");
    Serial.println("- Bandwidth: 125 kHz");
    Serial.println("- Coding Rate: 4/5");
    Serial.println("- Sync Word: 0x12");
    Serial.println("- Preamble: 8");
    Serial.println("- TX Power: 17 dBm");
    Serial.println("- CRC: Enabled");
  }
}

// CRC-16 calculation for packet integrity (MODBUS variant)
uint16_t calculateCRC16(uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  return crc;
}

// Print packet in hexadecimal for debugging
void printPacketHex(uint8_t *packet, int length) {
  Serial.println("Packet data (hex):");
  for (int i = 0; i < length; i++) {
    if (packet[i] < 0x10) Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
    if ((i + 1) % 8 == 0) Serial.println();
  }
  Serial.println();
}

// Flashes indicator when packet received
void flashReceivedIndicator(int x, int y, uint16_t color) {
  tft.fillCircle(x, y, 5, color);
  delay(50);
  tft.fillCircle(x, y, 5, BACKGROUND);
}

// Process water quality packet from Device ID 2 (calculated values) - 22 bytes
void processQualityPacket(int packetSize, int rssi) {
  Serial.print("Processing quality packet, ");
  Serial.print(packetSize);
  Serial.println(" bytes");
  
  if (packetSize != 22) {
    Serial.println("Error: Invalid packet size for water quality data");
    return;
  }
  
  // Read packet
  uint8_t packet[22];
  for (int i = 0; i < packetSize; i++) {
    packet[i] = LoRa.read();
  }
  
  if (DEBUG_PACKET_DATA) {
    printPacketHex(packet, packetSize);
  }
  
  // Verify CRC
  uint16_t receivedCRC = (packet[20] << 8) | packet[21];
  uint16_t calculatedCRC = calculateCRC16(packet, 20);
  
  if (DEBUG_CRC) {
    Serial.print("Received CRC: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(" | Calculated CRC: 0x");
    Serial.println(calculatedCRC, HEX);
  }
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("Error: CRC check failed for water quality packet");
    crcErrors++;
    return;
  }
  
  // Extract data
  uint8_t deviceId = packet[0];
  
  // pH (2 bytes) - divided by 100 to get pH value with 2 decimal places
  float pH = ((packet[1] << 8) | packet[2]) / 100.0;
  
  // EC (2 bytes) - divided by 100 to get mS/cm with 2 decimal places
  float ec = ((packet[3] << 8) | packet[4]) / 100.0 * 1000.0; // Convert to µS/cm
  
  // DO (2 bytes) - divided by 100 to get mg/L with 2 decimal places
  float doValue = ((packet[5] << 8) | packet[6]) / 100.0;
  
  // TDS (2 bytes) - raw ppm value
  float tds = ((packet[7] << 8) | packet[8]);
  
  // Turbidity (2 bytes) - NTU value
  float turbidity = ((packet[9] << 8) | packet[10]);
  
  // Temperature (2 bytes) - divided by 100 to get °C with 2 decimal places
  float temperature = ((int16_t)(packet[11] << 8) | packet[12]) / 100.0;
  
  // Sequence number (1 byte)
  uint8_t sequence = packet[13];
  
  // Update current data with proper timestamp
  qualityData.pH = pH;
  qualityData.ec = ec;
  qualityData.do_level = doValue;
  qualityData.tds = tds;
  qualityData.turbidity = turbidity;
  qualityData.temperature = temperature;
  qualityData.salinity = calculateSalinity(ec, temperature);
  
  qualityData.deviceId = deviceId;
  qualityData.rssi = rssi;
  qualityData.timestamp = getCurrentEpochTime();
  qualityData.sequence = sequence;
  qualityData.isValid = true;
  qualityData.hasRawData = false;
  qualityData.hasBME280 = false;
  qualityData.hasTideData = false;
  
  lastQualityDataTime = millis();
  receivedPacketsTotal++;
  
  Serial.println("Water quality data processed successfully:");
  Serial.println("  pH: " + String(qualityData.pH, 2));
  Serial.println("  EC: " + String(qualityData.ec, 2) + " µS/cm");
  Serial.println("  DO: " + String(qualityData.do_level, 2) + " mg/L");
  Serial.println("  TDS: " + String(qualityData.tds, 0) + " ppm");
  Serial.println("  Turbidity: " + String(qualityData.turbidity, 0) + " NTU");
  Serial.println("  Temperature: " + String(qualityData.temperature, 2) + " °C");
  Serial.println("  Salinity: " + String(qualityData.salinity, 2) + " ppt");
  Serial.println("  RSSI: " + String(qualityData.rssi) + " dBm");
  Serial.println("  Sequence: " + String(qualityData.sequence));
  if (timeSynchronized) {
    Serial.println("  Time: " + getFormattedDateTime());
  }
}

// Process 26-byte water quality packet from Device ID 2
void processQualityPacket26Bytes(int packetSize, int rssi) {
  Serial.print("Processing quality packet (26 bytes), ");
  Serial.print(packetSize);
  Serial.println(" bytes");
  
  if (packetSize != 26) {
    Serial.println("Error: Invalid packet size for 26-byte water quality data");
    return;
  }
  
  // Read packet
  uint8_t packet[26];
  for (int i = 0; i < packetSize; i++) {
    packet[i] = LoRa.read();
  }
  
  if (DEBUG_PACKET_DATA) {
    printPacketHex(packet, packetSize);
  }
  
  // Verify CRC - bytes 0-23 for data, bytes 24-25 for CRC
  uint16_t receivedCRC = (packet[24] << 8) | packet[25];
  uint16_t calculatedCRC = calculateCRC16(packet, 24);
  
  if (DEBUG_CRC) {
    Serial.print("Received CRC: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(" | Calculated CRC: 0x");
    Serial.println(calculatedCRC, HEX);
  }
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("Error: CRC check failed for 26-byte water quality packet");
    crcErrors++;
    return;
  }
  
  // Extract data from 26-byte format
  uint8_t deviceId = packet[0];
  
  // pH (2 bytes) + voltage (2 bytes)
  float pH = ((packet[1] << 8) | packet[2]) / 100.0f;
  float phVoltage = ((packet[3] << 8) | packet[4]);
  
  // EC (2 bytes) + voltage (2 bytes)
  float ec = ((packet[5] << 8) | packet[6]) / 100.0f * 1000.0f;
  float ecVoltage = ((packet[7] << 8) | packet[8]);
  
  // DO (2 bytes) + voltage (2 bytes)
  float doValue = ((packet[9] << 8) | packet[10]) / 100.0f;
  float doVoltage = ((packet[11] << 8) | packet[12]);
  
  // TDS (2 bytes) + voltage (2 bytes)
  float tds = ((packet[13] << 8) | packet[14]);
  float tdsVoltage = ((packet[15] << 8) | packet[16]) / 1000.0f;
  
  // Turbidity (2 bytes) + voltage (2 bytes)
  float turbidity = ((packet[17] << 8) | packet[18]) / 10.0f;
  float turbidityVoltage = ((packet[19] << 8) | packet[20]) / 1000.0f;
  
  // Temperature (2 bytes)
  float temperature = ((int16_t)(packet[21] << 8) | packet[22]) / 100.0f;
  
  // Update current data
  qualityData.pH = pH;
  qualityData.ec = ec;
  qualityData.do_level = doValue;
  qualityData.tds = tds;
  qualityData.turbidity = turbidity;
  qualityData.temperature = temperature;
  qualityData.salinity = calculateSalinity(ec, temperature);
  
  // Raw voltage values
  qualityData.phVoltage = phVoltage;
  qualityData.ecVoltage = ecVoltage;
  qualityData.doVoltage = doVoltage;
  qualityData.tdsVoltage = tdsVoltage;
  qualityData.turbidityVoltage = turbidityVoltage;
  
  qualityData.deviceId = deviceId;
  qualityData.rssi = rssi;
  qualityData.timestamp = getCurrentEpochTime();
  qualityData.sequence = packetCounter++;
  qualityData.isValid = true;
  qualityData.hasRawData = true;
  qualityData.hasBME280 = false;
  qualityData.hasTideData = false;
  
  lastQualityDataTime = millis();
  receivedPacketsTotal++;
  
  Serial.println("Water quality data (26-byte) processed successfully:");
  Serial.println("  pH: " + String(qualityData.pH, 2) + " (" + String(qualityData.phVoltage) + " mV)");
  Serial.println("  EC: " + String(qualityData.ec, 2) + " µS/cm (" + String(qualityData.ecVoltage) + " mV)");
  Serial.println("  DO: " + String(qualityData.do_level, 2) + " mg/L (" + String(qualityData.doVoltage) + " mV)");
  Serial.println("  TDS: " + String(qualityData.tds, 0) + " ppm (" + String(qualityData.tdsVoltage, 3) + " V)");
  Serial.println("  Turbidity: " + String(qualityData.turbidity, 1) + " NTU (" + String(qualityData.turbidityVoltage, 3) + " V)");
  Serial.println("  Temperature: " + String(qualityData.temperature, 2) + " °C");
  Serial.println("  Salinity: " + String(qualityData.salinity, 2) + " ppt");
  Serial.println("  RSSI: " + String(qualityData.rssi) + " dBm");
  if (timeSynchronized) {
    Serial.println("  Time: " + getFormattedDateTime());
  }
}

// Process water quality packet with BME280 data (35 bytes)
void processQualityPacketWithBME280(int packetSize, int rssi) {
  Serial.print("Processing quality packet with BME280, ");
  Serial.print(packetSize);
  Serial.println(" bytes");
  
  if (packetSize != 35) {
    Serial.println("Error: Invalid packet size for BME280 water quality data");
    return;
  }
  
  // Read packet
  uint8_t packet[35];
  for (int i = 0; i < packetSize; i++) {
    packet[i] = LoRa.read();
  }
  
  if (DEBUG_PACKET_DATA) {
    printPacketHex(packet, packetSize);
  }
  
  // Verify CRC - bytes 0-32 for data, bytes 33-34 for CRC
  uint16_t receivedCRC = (packet[33] << 8) | packet[34];
  uint16_t calculatedCRC = calculateCRC16(packet, 33);
  
  if (DEBUG_CRC) {
    Serial.print("Received CRC: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(" | Calculated CRC: 0x");
    Serial.println(calculatedCRC, HEX);
  }
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("Error: CRC check failed for BME280 water quality packet");
    crcErrors++;
    return;
  }
  
  // Extract data from BME280 packet format
  uint8_t deviceId = packet[0];
  
  // pH data (bytes 1-4)
  float pH = ((packet[1] << 8) | packet[2]) / 100.0f;
  float phVoltage = ((packet[3] << 8) | packet[4]);
  
  // EC data (bytes 5-8)
  float ec = ((packet[5] << 8) | packet[6]) / 100.0f * 1000.0f;
  float ecVoltage = ((packet[7] << 8) | packet[8]);
  
  // DO data (bytes 9-12)
  float doValue = ((packet[9] << 8) | packet[10]) / 100.0f;
  float doVoltage = ((packet[11] << 8) | packet[12]);
  
  // TDS data (bytes 13-16)
  float tds = ((packet[13] << 8) | packet[14]);
  float tdsVoltage = ((packet[15] << 8) | packet[16]) / 1000.0f;
  
  // Turbidity data (bytes 17-20)
  float turbidity = ((packet[17] << 8) | packet[18]) / 10.0f;
  float turbidityVoltage = ((packet[19] << 8) | packet[20]) / 1000.0f;
  
  // Temperature (bytes 21-22)
  float temperature = ((int16_t)(packet[21] << 8) | packet[22]) / 100.0f;
  
  // BME280 data (bytes 24-32)
  float humidity = ((packet[24] << 8) | packet[25]) / 100.0f;
  
  // Pressure - 4 bytes
  uint32_t pressureRaw = ((uint32_t)packet[26] << 24) | ((uint32_t)packet[27] << 16) | 
                        ((uint32_t)packet[28] << 8) | packet[29];
  float pressure = pressureRaw / 100000.0f;
  
  // Air temperature
  float airTemperature = ((int16_t)(packet[30] << 8) | packet[31]) / 100.0f;
  
  // BME280 status (byte 32)
  bool bmeReady = packet[32] == 0x01;
  
  // Update current data
  qualityData.pH = pH;
  qualityData.ec = ec;
  qualityData.do_level = doValue;
  qualityData.tds = tds;
  qualityData.turbidity = turbidity;
  qualityData.temperature = temperature;
  qualityData.salinity = calculateSalinity(ec, temperature);
  
  // BME280 environmental data
  qualityData.humidity = humidity;
  qualityData.pressure = pressure;
  qualityData.airTemperature = airTemperature;
  qualityData.bmeReady = bmeReady;
  
  // Raw voltage values
  qualityData.phVoltage = phVoltage;
  qualityData.ecVoltage = ecVoltage;
  qualityData.doVoltage = doVoltage;
  qualityData.tdsVoltage = tdsVoltage;
  qualityData.turbidityVoltage = turbidityVoltage;
  
  qualityData.deviceId = deviceId;
  qualityData.rssi = rssi;
  qualityData.timestamp = getCurrentEpochTime();
  qualityData.sequence = packetCounter++;
  qualityData.isValid = true;
  qualityData.hasRawData = true;
  qualityData.hasBME280 = true;
  qualityData.hasTideData = false;
  
  lastQualityDataTime = millis();
  receivedPacketsTotal++;
  
  Serial.println("Water quality data (BME280) processed successfully:");
  Serial.println("  pH: " + String(qualityData.pH, 2) + " (" + String(qualityData.phVoltage) + " mV)");
  Serial.println("  EC: " + String(qualityData.ec, 2) + " µS/cm (" + String(qualityData.ecVoltage) + " mV)");
  Serial.println("  DO: " + String(qualityData.do_level, 2) + " mg/L (" + String(qualityData.doVoltage) + " mV)");
  Serial.println("  TDS: " + String(qualityData.tds, 0) + " ppm (" + String(qualityData.tdsVoltage, 3) + " V)");
  Serial.println("  Turbidity: " + String(qualityData.turbidity, 1) + " NTU (" + String(qualityData.turbidityVoltage, 3) + " V)");
  Serial.println("  Water Temperature: " + String(qualityData.temperature, 2) + " °C");
  Serial.println("  Air Temperature: " + String(qualityData.airTemperature, 2) + " °C");
  Serial.println("  Humidity: " + String(qualityData.humidity, 1) + " %");
  Serial.println("  Pressure: " + String(qualityData.pressure, 5) + " hPa");
  Serial.println("  Salinity: " + String(qualityData.salinity, 2) + " ppt");
  Serial.println("  RSSI: " + String(qualityData.rssi) + " dBm");
  Serial.println("  Sequence: " + String(qualityData.sequence));
  if (timeSynchronized) {
    Serial.println("  Time: " + getFormattedDateTime());
  }
}

// Process water quality packet with raw data but no BME280 (32 bytes)
void processQualityPacketWithRawData(int packetSize, int rssi) {
  Serial.print("Processing quality packet with raw data, ");
  Serial.print(packetSize);
  Serial.println(" bytes");
  
  if (packetSize != 32) {
    Serial.println("Error: Invalid packet size for raw data water quality data");
    return;
  }
  
  // Read packet
  uint8_t packet[32];
  for (int i = 0; i < packetSize; i++) {
    packet[i] = LoRa.read();
  }
  
  if (DEBUG_PACKET_DATA) {
    printPacketHex(packet, packetSize);
  }
  
  // Verify CRC - bytes 0-29 for data, bytes 30-31 for CRC
  uint16_t receivedCRC = (packet[30] << 8) | packet[31];
  uint16_t calculatedCRC = calculateCRC16(packet, 30);
  
  if (DEBUG_CRC) {
    Serial.print("Received CRC: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(" | Calculated CRC: 0x");
    Serial.println(calculatedCRC, HEX);
  }
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("Error: CRC check failed for raw data water quality packet");
    crcErrors++;
    return;
  }
  
  // Extract data from raw data packet format
  uint8_t deviceId = packet[0];
  
  // pH data (bytes 1-4)
  float pH = ((packet[1] << 8) | packet[2]) / 100.0f;
  float phVoltage = ((packet[3] << 8) | packet[4]);
  
  // EC data (bytes 5-8)
  float ec = ((packet[5] << 8) | packet[6]) / 100.0f * 1000.0f;
  float ecVoltage = ((packet[7] << 8) | packet[8]);
  
  // DO data (bytes 9-12)
  float doValue = ((packet[9] << 8) | packet[10]) / 100.0f;
  float doVoltage = ((packet[11] << 8) | packet[12]);
  
  // TDS data (bytes 13-16)
  float tds = ((packet[13] << 8) | packet[14]);
  float tdsVoltage = ((packet[15] << 8) | packet[16]) / 1000.0f;
  
  // Turbidity data (bytes 17-20)
  float turbidity = ((packet[17] << 8) | packet[18]) / 10.0f;
  float turbidityVoltage = ((packet[19] << 8) | packet[20]) / 1000.0f;
  
  // Temperature (bytes 21-22)
  float temperature = ((int16_t)(packet[21] << 8) | packet[22]) / 100.0f;
  
  // Sequence number (byte 24)
  uint8_t sequence = packet[24];
  
  // Update current data
  qualityData.pH = pH;
  qualityData.ec = ec;
  qualityData.do_level = doValue;
  qualityData.tds = tds;
  qualityData.turbidity = turbidity;
  qualityData.temperature = temperature;
  qualityData.salinity = calculateSalinity(ec, temperature);
  
  // Raw voltage values
  qualityData.phVoltage = phVoltage;
  qualityData.ecVoltage = ecVoltage;
  qualityData.doVoltage = doVoltage;
  qualityData.tdsVoltage = tdsVoltage;
  qualityData.turbidityVoltage = turbidityVoltage;
  
  qualityData.deviceId = deviceId;
  qualityData.rssi = rssi;
  qualityData.timestamp = getCurrentEpochTime();
  qualityData.sequence = sequence;
  qualityData.isValid = true;
  qualityData.hasRawData = true;
  qualityData.hasBME280 = false;
  qualityData.hasTideData = false;
  
  lastQualityDataTime = millis();
  receivedPacketsTotal++;
  
  Serial.println("Water quality data (raw data) processed successfully:");
  Serial.println("  pH: " + String(qualityData.pH, 2) + " (" + String(qualityData.phVoltage) + " mV)");
  Serial.println("  EC: " + String(qualityData.ec, 2) + " µS/cm (" + String(qualityData.ecVoltage) + " mV)");
  Serial.println("  DO: " + String(qualityData.do_level, 2) + " mg/L (" + String(qualityData.doVoltage) + " mV)");
  Serial.println("  TDS: " + String(qualityData.tds, 0) + " ppm (" + String(qualityData.tdsVoltage, 3) + " V)");
  Serial.println("  Turbidity: " + String(qualityData.turbidity, 1) + " NTU (" + String(qualityData.turbidityVoltage, 3) + " V)");
  Serial.println("  Temperature: " + String(qualityData.temperature, 2) + " °C");
  Serial.println("  Salinity: " + String(qualityData.salinity, 2) + " ppt");
  Serial.println("  RSSI: " + String(qualityData.rssi) + " dBm");
  Serial.println("  Sequence: " + String(qualityData.sequence));
  if (timeSynchronized) {
    Serial.println("  Time: " + getFormattedDateTime());
  }
}

// Process 38-byte water quality packet with BME280 and Tide data
void processQualityPacket38Bytes(int packetSize, int rssi) {
  Serial.print("Processing quality packet (38 bytes), ");
  Serial.print(packetSize);
  Serial.println(" bytes");
  
  if (packetSize != 38) {
    Serial.println("Error: Invalid packet size for 38-byte water quality data");
    return;
  }
  
  // Read packet
  uint8_t packet[38];
  for (int i = 0; i < packetSize; i++) {
    packet[i] = LoRa.read();
  }
  
  if (DEBUG_PACKET_DATA) {
    printPacketHex(packet, packetSize);
  }
  
  // Verify CRC - bytes 0-35 for data, bytes 36-37 for CRC
  uint16_t receivedCRC = (packet[36] << 8) | packet[37];
  uint16_t calculatedCRC = calculateCRC16(packet, 36);
  
  if (DEBUG_CRC) {
    Serial.print("Received CRC: 0x");
    Serial.print(receivedCRC, HEX);
    Serial.print(" | Calculated CRC: 0x");
    Serial.println(calculatedCRC, HEX);
  }
  
  if (receivedCRC != calculatedCRC) {
    Serial.println("Error: CRC check failed for 38-byte water quality packet");
    crcErrors++;
    return;
  }
  
  // Extract data from 38-byte format
  uint8_t deviceId = packet[0];
  
  // pH data (bytes 1-4)
  float pH = ((packet[1] << 8) | packet[2]) / 100.0f;
  float phVoltage = ((packet[3] << 8) | packet[4]);
  
  // EC data (bytes 5-8)
  float ec = ((packet[5] << 8) | packet[6]) / 100.0f * 1000.0f;
  float ecVoltage = ((packet[7] << 8) | packet[8]);
  
  // DO data (bytes 9-12)
  float doValue = ((packet[9] << 8) | packet[10]) / 100.0f;
  float doVoltage = ((packet[11] << 8) | packet[12]);
  
  // TDS data (bytes 13-16)
  float tds = ((packet[13] << 8) | packet[14]);
  float tdsVoltage = ((packet[15] << 8) | packet[16]) / 1000.0f;
  
  // Turbidity data (bytes 17-20)
  float turbidity = ((packet[17] << 8) | packet[18]) / 10.0f;
  float turbidityVoltage = ((packet[19] << 8) | packet[20]) / 1000.0f;
  
  // Temperature (bytes 21-22)
  float temperature = ((int16_t)(packet[21] << 8) | packet[22]) / 100.0f;
  
  // BME280 data (bytes 24-31)
  float humidity = ((packet[24] << 8) | packet[25]) / 100.0f;
  
  // Pressure - 4 bytes
  uint32_t pressureRaw = ((uint32_t)packet[26] << 24) | ((uint32_t)packet[27] << 16) | 
                        ((uint32_t)packet[28] << 8) | packet[29];
  float pressure = pressureRaw / 100000.0f;
  
  // Air temperature
  float airTemperature = ((int16_t)(packet[30] << 8) | packet[31]) / 100.0f;
  
  // Tide data (bytes 32-33)
  float tide_level = ((int16_t)(packet[32] << 8) | packet[33]) / 10.0f;
  
  // Status bytes (bytes 34-35)
  bool bmeReady = packet[34] == 0x01;
  bool tempReady = packet[35] == 0x01;
  
  // Update current data
  qualityData.pH = pH;
  qualityData.ec = ec;
  qualityData.do_level = doValue;
  qualityData.tds = tds;
  qualityData.turbidity = turbidity;
  qualityData.temperature = temperature;
  qualityData.salinity = calculateSalinity(ec, temperature);
  qualityData.tide_level = tide_level;
  
  // BME280 environmental data
  qualityData.humidity = humidity;
  qualityData.pressure = pressure;
  qualityData.airTemperature = airTemperature;
  qualityData.bmeReady = bmeReady;
  
  // Raw voltage values
  qualityData.phVoltage = phVoltage;
  qualityData.ecVoltage = ecVoltage;
  qualityData.doVoltage = doVoltage;
  qualityData.tdsVoltage = tdsVoltage;
  qualityData.turbidityVoltage = turbidityVoltage;
  
  qualityData.deviceId = deviceId;
  qualityData.rssi = rssi;
  qualityData.timestamp = getCurrentEpochTime();
  qualityData.sequence = packetCounter++;
  qualityData.isValid = true;
  qualityData.hasRawData = true;
  qualityData.hasBME280 = true;
  qualityData.hasTideData = true;
  
  lastQualityDataTime = millis();
  receivedPacketsTotal++;
  
  Serial.println("Water quality data (38-byte) processed successfully:");
  Serial.println("  pH: " + String(qualityData.pH, 2) + " (" + String(qualityData.phVoltage) + " mV)");
  Serial.println("  EC: " + String(qualityData.ec, 2) + " µS/cm (" + String(qualityData.ecVoltage) + " mV)");
  Serial.println("  DO: " + String(qualityData.do_level, 2) + " mg/L (" + String(qualityData.doVoltage) + " mV)");
  Serial.println("  TDS: " + String(qualityData.tds, 0) + " ppm (" + String(qualityData.tdsVoltage, 3) + " V)");
  Serial.println("  Turbidity: " + String(qualityData.turbidity, 1) + " NTU (" + String(qualityData.turbidityVoltage, 3) + " V)");
  Serial.println("  Water Temperature: " + String(qualityData.temperature, 2) + " °C");
  Serial.println("  Air Temperature: " + String(qualityData.airTemperature, 2) + " °C");
  Serial.println("  Humidity: " + String(qualityData.humidity, 1) + " %");
  Serial.println("  Pressure: " + String(qualityData.pressure, 5) + " hPa");
  Serial.println("  Salinity: " + String(qualityData.salinity, 2) + " ppt");
  Serial.println("  Tide Level: " + String(qualityData.tide_level, 1) + " cm");
  Serial.println("  RSSI: " + String(qualityData.rssi) + " dBm");
  Serial.println("  Sequence: " + String(qualityData.sequence));
  if (timeSynchronized) {
    Serial.println("  Time: " + getFormattedDateTime());
  }
}

// Check for incoming LoRa packets with timeout
void checkForPacketsWithTimeout() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    int rssi = LoRa.packetRssi();
    uint8_t deviceId = LoRa.peek();
    
    Serial.print("Packet received from Device ID: ");
    Serial.print(deviceId);
    Serial.print(", Size: ");
    Serial.print(packetSize);
    Serial.print(" bytes, RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");
    
    // Flash indicator for packet received
    flashReceivedIndicator(50, 35, TFT_BLUE);
    
    // Process packet based on size (only Device ID 2)
    if (deviceId == 2) {
      // Determine packet format based on size
      if (packetSize == 22) {
        processQualityPacket(packetSize, rssi);
      } else if (packetSize == 26) {
        processQualityPacket26Bytes(packetSize, rssi);
      } else if (packetSize == 35) {
        processQualityPacketWithBME280(packetSize, rssi);
      } else if (packetSize == 32) {
        processQualityPacketWithRawData(packetSize, rssi);
      } else if (packetSize == 38) {
        processQualityPacket38Bytes(packetSize, rssi);
      } else {
        Serial.println("Error: Unknown packet size for Device ID 2: " + String(packetSize));
      }
    } else {
      Serial.println("Error: Unknown Device ID: " + String(deviceId));
    }
  }
}

// Check LoRa module status
void checkLoRaStatus() {
  static unsigned long lastStatusCheck = 0;
  
  if (millis() - lastStatusCheck > 5000) {
    if (loraInitialized) {
      Serial.println("LoRa module is active and listening");
    } else {
      Serial.println("LoRa module initialization failed");
    }
    lastStatusCheck = millis();
  }
}

// Check WiFi status and attempt reconnection if needed
void checkWiFiStatus() {
  static unsigned long lastWiFiReconnect = 0;
  
  if (millis() - lastWiFiReconnect > 30000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, attempting to reconnect...");
      wifiConfigured = false;
      initializeWiFi();
      if (wifiConfigured) {
        initializeNTP();
      }
    } else if (!wifiConfigured) {
      wifiConfigured = true;
      Serial.println("WiFi reconnected! IP: " + WiFi.localIP().toString());
      initializeNTP();
    }
    lastWiFiReconnect = millis();
  }
}

// Update the main display
void updateDisplay() {
  static unsigned long lastDisplayUpdate = 0;
  
  if (millis() - lastDisplayUpdate < DISPLAY_UPDATE_INTERVAL) {
    return;
  }
  
  // Update status bar
  tft.fillRect(40, 30, 35, 8, TFT_DARKGREY);
  tft.setCursor(40, 30);
  tft.setTextColor(loraInitialized ? TFT_GREEN : TFT_RED);
  tft.print(loraInitialized ? "OK" : "ERR");
  
  tft.fillRect(125, 30, 50, 8, TFT_DARKGREY);
  tft.setCursor(125, 30);
  tft.setTextColor(TIME_COLOR);
  tft.print(getFormattedTime());
  
  tft.fillRect(210, 30, 25, 8, TFT_DARKGREY);
  tft.setCursor(210, 30);
  tft.setTextColor(TEXT_COLOR);
  tft.print(receivedPacketsTotal);
  
  tft.fillRect(270, 30, 35, 8, TFT_DARKGREY);
  tft.setCursor(270, 30);
  tft.setTextColor(wifiConfigured ? TFT_GREEN : TFT_RED);
  tft.print(wifiConfigured ? "OK" : "OFF");
  
  // Update sensor data display
  updateSensorDataDisplay();
  
  lastDisplayUpdate = millis();
}

// Update sensor data display
void updateSensorDataDisplay() {
    int startY = 70;
    
    // Clear previous data area
    tft.fillRect(0, startY, tft.width(), tft.height() - startY - 20, BACKGROUND);
    
    // First row: pH, EC, DO, TDS, Turbidity, Salinity
    tft.setTextSize(1.5);
    
    // pH - Column 1
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(5, startY);
    tft.print("pH:");
    tft.setTextSize(2.7);
    tft.setTextColor(PH_COLOR);
    tft.setCursor(5, startY + 12);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.pH, 2));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    
    // EC - Column 2
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(75, startY);
    tft.print("EC:");
    tft.setTextSize(2);
    tft.setTextColor(EC_COLOR);
    tft.setCursor(75, startY + 12);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.ec, 0));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(75, startY + 30);
    tft.print("uS/cm");
    
    // DO - Column 3
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(160, startY);
    tft.print("DO:");
    tft.setTextSize(2);
    tft.setTextColor(DO_COLOR);
    tft.setCursor(160, startY + 12);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.do_level, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(160, startY + 30);
    tft.print("mg/L");
    
    // TDS - Column 4
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(210, startY);
    tft.print("TDS:");
    tft.setTextSize(2);
    tft.setTextColor(TDS_COLOR);
    tft.setCursor(210, startY + 12);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.tds, 0));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(211, startY + 30);
    tft.print("ppm");
    
    // Turbidity - Column 5
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(280, startY);
    tft.print("Turb:");
    tft.setTextSize(2);
    tft.setTextColor(TURBIDITY_COLOR);
    tft.setCursor(280, startY + 12);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.turbidity, 0));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(280, startY + 30);
    tft.print("NTU");
    
    // Salinity - Column 6
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(340, startY);
    tft.print("Sal:");
    tft.setTextSize(2);
    tft.setTextColor(SALINITY_COLOR);
    tft.setCursor(340, startY + 12);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.salinity, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(340, startY + 30);
    tft.print("ppt");
    
    // Second row: BOD5, COD, Water Temperature, Air Temperature, Humidity, Pressure, Tide
    // BOD5 - Column 1
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(5, startY + 50);
    tft.print("BOD5:");
    tft.setTextSize(2);
    tft.setTextColor(BOD_COLOR);
    tft.setCursor(5, startY + 62);
    if (bodCalculated && qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.bod5, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(5, startY + 80);
    tft.print("mg/L");
    
    // COD - Column 2
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(70, startY + 50);
    tft.print("COD:");
    tft.setTextSize(2);
    tft.setTextColor(COD_COLOR);
    tft.setCursor(70, startY + 62);
    if (bodCalculated && qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.cod, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(70, startY + 80);
    tft.print("mg/L");
    
    // Water Temperature - Column 3
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(140, startY + 50);
    tft.print("Water T:");
    tft.setTextSize(2);
    tft.setTextColor(TEMP_COLOR);
    tft.setCursor(140, startY + 62);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.temperature, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(140, startY + 80);
    tft.print("C");
    
    // Air Temperature - Column 4
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(210, startY + 50);
    tft.print("Air T:");
    tft.setTextSize(2);
    tft.setTextColor(AIR_TEMP_COLOR);
    tft.setCursor(210, startY + 62);
    if (qualityData.hasBME280 && qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.airTemperature, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(210, startY + 80);
    tft.print("C");
    
    // Humidity - Column 5
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(280, startY + 50);
    tft.print("Humid:");
    tft.setTextSize(2);
    tft.setTextColor(HUMIDITY_COLOR);
    tft.setCursor(280, startY + 62);
    if (qualityData.hasBME280 && qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.humidity, 1));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(280, startY + 80);
    tft.print("%");
    
    // Pressure - Column 6
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(345, startY + 50);
    tft.print("Press:");
    tft.setTextSize(2);
    tft.setTextColor(PRESSURE_COLOR);
    tft.setCursor(345, startY + 62);
    if (qualityData.hasBME280 && qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.pressure, 3));
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    tft.setTextSize(1);
    tft.setCursor(345, startY + 80);
    tft.print("hPa");
    
    // Third row: Tide Level
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(5, startY + 95);
    tft.print("Tide Level:");
    tft.setTextSize(2);
    tft.setTextColor(TIDE_COLOR);
    tft.setCursor(80, startY + 95);
    if (qualityData.hasTideData && qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      tft.print(String(qualityData.tide_level, 1) + " cm");
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("N/A");
    }
    
    // Device info and RSSI
    tft.setTextSize(1);
    tft.setTextColor(TEXT_COLOR);
    tft.setCursor(5, startY + 115);
    if (qualityData.isValid && (millis() - lastQualityDataTime < 60000)) {
      String bmeStatus = qualityData.hasBME280 ? "BME:OK" : "BME:NO";
      String rawStatus = qualityData.hasRawData ? "RAW:OK" : "RAW:NO";
      String tideStatus = qualityData.hasTideData ? "TIDE:OK" : "TIDE:NO";
      String bodStatus = bodCalculated ? "BOD:OK" : "BOD:NO";
      
      tft.print("RSSI: " + String(qualityData.rssi) + " dBm | Dev: " + 
                String(qualityData.deviceId) + " | " + bmeStatus + " | " + rawStatus + " | " + tideStatus + " | " + bodStatus);
      
      // Last update time
      tft.setCursor(370, startY + 115);
      if (timeSynchronized) {
        tft.print("Updated: " + getFormattedTime());
      } else {
        unsigned long secondsAgo = (millis() - lastQualityDataTime) / 1000;
        tft.print("Updated: " + String(secondsAgo) + "s ago");
      }
    } else {
      tft.setTextColor(ERROR_COLOR);
      tft.print("No recent data from sensor");
    }
}

// Send data to server as JSON
void sendDataToServer() {
  static unsigned long lastServerUpdate = 0;

  if (!wifiConfigured) {
    lastServerStatus = "WiFi not connected";
    return;
  }
  if (serverHost.isEmpty() || serverPath.isEmpty() || serverPort <= 0) {
    lastServerStatus = "Server not configured";
    return;
  }
  if (millis() - lastServerUpdate < serverUpdateInterval) return;

  bool hasRecentQualityData = qualityData.isValid && (millis() - lastQualityDataTime < 120000);
  if (!hasRecentQualityData) {
    Serial.println("No recent data from sensor, skipping server update");
    return;
  }

  lastServerUpdate = millis();
  Serial.println("Preparing to send JSON data to server...");

  // Build JSON — calculated values + raw voltages
  DynamicJsonDocument doc(512);
  doc["device_id"]        = qualityData.deviceId;
  doc["tide_level"]       = String(qualityData.tide_level, 1).toFloat();
  doc["ph"]               = String(qualityData.pH, 2).toFloat();
  doc["ec"]               = String(qualityData.ec, 2).toFloat();
  doc["do_level"]         = String(qualityData.do_level, 2).toFloat();
  doc["tds"]              = qualityData.tds;
  doc["temperature"]      = String(qualityData.temperature, 2).toFloat();
  // Raw voltage data (sent as 0 if packet type does not include raw data)
  doc["ph_voltage"]       = qualityData.hasRawData ? qualityData.phVoltage   : 0;
  doc["ec_voltage"]       = qualityData.hasRawData ? qualityData.ecVoltage   : 0;
  doc["do_voltage"]       = qualityData.hasRawData ? qualityData.doVoltage   : 0;
  doc["tds_voltage"]      = qualityData.hasRawData ? qualityData.tdsVoltage  : 0;
  doc["timestamp"]        = getCurrentEpochTime();

  String jsonString;
  serializeJson(doc, jsonString);
  Serial.println("JSON: " + jsonString);

  bool useSSL = (serverPort == 443);
  String url  = (useSSL ? "https://" : "http://") + serverHost + ":" + String(serverPort) + serverPath;
  Serial.println("URL: " + url);

  WiFiClient        plainClient;
  WiFiClientSecure  secureClient;
  HTTPClient        http;
  bool begun = false;

  if (useSSL) {
    secureClient.setInsecure();
    begun = http.begin(secureClient, url);
  } else {
    begun = http.begin(plainClient, url);
  }

  if (!begun) {
    lastServerStatus = "Connection failed";
    serverConnected  = false;
    http.end();
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.setTimeout(10000);
  int httpCode = http.POST(jsonString);
  Serial.printf("HTTP Response: %d\n", httpCode);

  if (httpCode == 200 || httpCode == 201) {
    lastServerStatus = "Success (HTTP " + String(httpCode) + ")";
    serverConnected  = true;
    lastServerSend   = millis();
    Serial.println("Server update SUCCESS");
  } else if (httpCode < 0) {
    lastServerStatus = "Error: " + http.errorToString(httpCode);
    serverConnected  = false;
  } else {
    lastServerStatus = "Server error: HTTP " + String(httpCode);
    serverConnected  = false;
  }

  http.end();
  Serial.println("========================================");
}

// Setup function
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Water Quality Monitoring Receiver ===");
  Serial.println("Initializing system...");
  
  startupTime = millis();

  // EEPROM — load WiFi / AP / server config (overrides hardcoded defaults if valid)
  EEPROM.begin(EEPROM_SIZE);
  readAPPassword();
  readWiFiCredentials();
  readServerConfig();
  
  // Initialize display first
  initializeDisplay();
  initializeDisplayLayout();
  
  // Initialize WiFi
  initializeWiFi();
  
  // Initialize NTP time if WiFi is connected
  if (wifiConfigured) {
    initializeNTP();
  }
  
  // Initialize LoRa with separate SPI
  LoRaSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(LoRaSPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  Serial.println("Starting LoRa receiver...");
  
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa initialization failed!");
    loraInitialized = false;
  } else {
    Serial.println("LoRa initialized successfully!");
    configureLoRa();
    loraInitialized = true;
  }
  
  // Setup web server (disabled)
  setupWebServer();
  
  Serial.println("System initialization complete");
  Serial.println("Waiting for LoRa packets...");
}

// Main loop
void loop() {
  // Handle web AP clients
  server.handleClient();

  // Reconnect WiFi if needed
  checkWiFiConnection();
  
  // Check for incoming LoRa packets
  checkForPacketsWithTimeout();
  
  // Update display periodically
  updateDisplay();
  
  // Check LoRa and WiFi status
  checkLoRaStatus();
  checkWiFiStatus();
  
  // Update NTP time if needed
  updateNTPTime();
  
  // Send data to server if connected
  sendDataToServer();
  
  // Check for BOD calculation periodically
  if (millis() - lastBODCheck > BOD_UPDATE_INTERVAL) {
    if (qualityData.isValid && qualityData.do_level > 0 && !bodCalculated) {
      calculateBOD();
    }
    lastBODCheck = millis();
  }
  
  // Debug information
  static unsigned long lastDebugMsg = 0;
  if (millis() - lastDebugMsg > DEBUG_MSG_INTERVAL) {
    Serial.println("=== System Status ===");
    Serial.println("Uptime: " + String((millis() - startupTime) / 1000) + "s");
    Serial.println("LoRa: " + String(loraInitialized ? "OK" : "ERROR"));
    Serial.println("WiFi: " + String(wifiConfigured ? "OK" : "DISCONNECTED"));
    Serial.println("NTP: " + String(timeSynchronized ? "SYNC" : "NO SYNC"));
    Serial.println("Packets - Total: " + String(receivedPacketsTotal));
    Serial.println("CRC Errors: " + String(crcErrors));
    Serial.println("Last Quality: " + String((millis() - lastQualityDataTime) / 1000) + "s ago");
    
    // BOD calculation status
    if (bodCalculated) {
      Serial.println("BOD/COD: Calculated");
      Serial.println("  BOD1: " + String(qualityData.bod1, 2) + " mg/L");
      Serial.println("  BOD5: " + String(qualityData.bod5, 2) + " mg/L");
      Serial.println("  COD: " + String(qualityData.cod, 2) + " mg/L");
    } else {
      Serial.println("BOD/COD: Not calculated yet");
    }
    
    // Tide data status
    if (qualityData.hasTideData) {
      Serial.println("Tide Level: " + String(qualityData.tide_level, 1) + " cm");
    } else {
      Serial.println("Tide Level: No data available");
    }
    
    Serial.println("====================");
    lastDebugMsg = millis();
  }
  
  delay(10);
}