#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ThingSpeak.h>
#include <Adafruit_SHT31.h>
#include <PubSubClient.h>
#include <Adafruit_BMP085.h>
#include <BH1750.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <math.h>
#include "SSD1306Wire.h"
#include "secrets.h"


// =========================
// USER SETTINGS
// =========================
const char* NTP_SERVER = "0.au.pool.ntp.org";
const char* TZ_INFO    = "AEST-10AEDT,M10.1.0,M4.1.0/3";

#define SHT31_ADDRESS_PRIMARY 0x44
#define SHT31_ADDRESS_SECONDARY 0x45

#define I2C_SDA D2
#define I2C_SCL D1
#define OLED_ADDRESS 0x3C

const unsigned long SENSOR_READ_INTERVAL_MS    = 30000UL;    // 30 sec
const unsigned long SENSOR_UPLOAD_INTERVAL_MS  = 600000UL;   // 10 min
const unsigned long FORECAST_REFRESH_MS        = 1800000UL;  // 30 min
const unsigned long PAGE_ROTATE_MS             = 6000UL;     // 5 sec

const int PRESSURE_HISTORY_SIZE = 18; // 18 x 10 minutes = 3 hours
const float PRESSURE_TREND_THRESHOLD_HPA = 0.15f;

// =========================
// GLOBALS
// =========================
WiFiClient tsClient;

WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);

unsigned long lastMqttConnectAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 10000UL;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
bool sht31Found = false;
uint8_t sht31Address = SHT31_ADDRESS_PRIMARY;
uint8_t sht31FailCount = 0;
const uint8_t SHT31_REINIT_AFTER_FAILS = 3;
Adafruit_BMP085 bmp;
BH1750 lightMeter;
SSD1306Wire display(OLED_ADDRESS, I2C_SDA, I2C_SCL);

bool bmpFound = false;
bool bh1750Found = false;

unsigned long lastSensorRead = 0;
unsigned long lastSensorUpload = 0;
unsigned long lastForecastFetch = 0;
unsigned long lastPageChange = 0;
uint8_t pageIndex = 0;   // 0=clock, 1=sensors, 2=today, 3=3-day

float currentTempC = NAN;
float currentHumidity = NAN;
float currentPressureHpa = NAN;
float currentLux = NAN;

// Pressure trend tracking
float pressureHistory[PRESSURE_HISTORY_SIZE];
int pressureHistoryCount = 0;
int pressureHistoryHead = 0; // next write index

float pressureWeatherReferenceHpa = NAN;
bool pressureWeatherFallbackValid = false;

// boot status
String pressureSeedStatus = "Pressure seed: none";

struct ForecastDay {
  String date;
  int weatherCode;
  float tMax;
  float tMin;
};

ForecastDay forecast[4];   // 0=today, 1..3=next three days
bool forecastValid = false;

// =========================
// WIFI / TIME
// =========================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}

void initTime() {
  configTime(TZ_INFO, NTP_SERVER);

  Serial.print("Waiting for NTP time");
  time_t now = time(nullptr);
  int retries = 0;

  while (now < 100000 && retries < 30) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    retries++;
  }

  Serial.println();
  if (now >= 100000) {
    Serial.println("NTP time acquired");
  } else {
    Serial.println("NTP time not acquired");
  }
}

String getTime12hString() {
  time_t now = time(nullptr);
  if (now < 100000) return "--:--";

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  int hour = timeinfo.tm_hour;
  String ampm = (hour >= 12) ? "PM" : "AM";
  hour = hour % 12;
  if (hour == 0) hour = 12;

  String minStr = (timeinfo.tm_min < 10) ? "0" + String(timeinfo.tm_min) : String(timeinfo.tm_min);
  return String(hour) + ":" + minStr + " " + ampm;
}

String getDayString() {
  time_t now = time(nullptr);
  if (now < 100000) return "---";

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  const char* names[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  return String(names[timeinfo.tm_wday]);
}

String getDateString() {
  time_t now = time(nullptr);
  if (now < 100000) return "-- --- ----";

  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                          "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

  char buf[20];
  snprintf(buf, sizeof(buf), "%d %s %d",
           timeinfo.tm_mday,
           months[timeinfo.tm_mon],
           timeinfo.tm_year + 1900);
  return String(buf);
}

String fullDayNameFromISO(const String& isoDate) {
  if (isoDate.length() < 10) return "---";

  int y = isoDate.substring(0, 4).toInt();
  int m = isoDate.substring(5, 7).toInt();
  int d = isoDate.substring(8, 10).toInt();

  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = m - 1;
  t.tm_mday = d;
  t.tm_hour = 12;
  mktime(&t);

  const char* names[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
  if (t.tm_wday < 0 || t.tm_wday > 6) return "---";
  return String(names[t.tm_wday]);
}

String shortDayNameFromISO(const String& isoDate) {
  if (isoDate.length() < 10) return "---";

  int y = isoDate.substring(0, 4).toInt();
  int m = isoDate.substring(5, 7).toInt();
  int d = isoDate.substring(8, 10).toInt();

  struct tm t = {};
  t.tm_year = y - 1900;
  t.tm_mon = m - 1;
  t.tm_mday = d;
  t.tm_hour = 12;
  mktime(&t);

  const char* names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
  if (t.tm_wday < 0 || t.tm_wday > 6) return "---";
  return String(names[t.tm_wday]);
}

// =========================
// ICONS / HELPERS
// =========================
String degC(int value) {
  return String(value) + "\xB0" + "°C";
}

String degC1(float value) {
  return String(value, 1) + "\xB0" + "°C";
}

void drawBoldString(int x, int y, const String& s) {
  display.drawString(x, y, s);
  display.drawString(x + 1, y, s);
}

void drawCenteredBoldString(int cx, int y, const String& s) {
  int x = cx - display.getStringWidth(s) / 2;
  drawBoldString(x, y, s);
}

void drawSunIcon(int x, int y) {
  display.drawCircle(x, y, 5);
  for (int i = 0; i < 8; i++) {
    float a = i * 3.14159f / 4.0f;
    int x1 = x + (int)(7 * cos(a));
    int y1 = y + (int)(7 * sin(a));
    int x2 = x + (int)(10 * cos(a));
    int y2 = y + (int)(10 * sin(a));
    display.drawLine(x1, y1, x2, y2);
  }
}

void drawCloudIcon(int x, int y) {
  display.fillCircle(x - 5, y + 1, 4);
  display.fillCircle(x,     y - 2, 5);
  display.fillCircle(x + 6, y + 1, 4);
  display.fillRect(x - 9, y + 1, 18, 6);
}

void drawRainIcon(int x, int y) {
  drawCloudIcon(x, y);
  display.drawLine(x - 4, y + 10, x - 6, y + 14);
  display.drawLine(x + 1, y + 10, x - 1, y + 14);
  display.drawLine(x + 6, y + 10, x + 4, y + 14);
}

void drawPartlyCloudyIcon(int x, int y) {
  display.drawCircle(x - 5, y - 3, 4);
  for (int i = 0; i < 8; i++) {
    float a = i * 3.14159f / 4.0f;
    int x1 = (x - 5) + (int)(6 * cos(a));
    int y1 = (y - 3) + (int)(6 * sin(a));
    int x2 = (x - 5) + (int)(8 * cos(a));
    int y2 = (y - 3) + (int)(8 * sin(a));
    display.drawLine(x1, y1, x2, y2);
  }
  drawCloudIcon(x + 2, y + 1);
}

void drawWeatherIcon(int weatherCode, int x, int y) {
  if (weatherCode == 0) {
    drawSunIcon(x, y);
  } else if (weatherCode == 1 || weatherCode == 2) {
    drawPartlyCloudyIcon(x, y);
  } else if (weatherCode == 3 || weatherCode == 45 || weatherCode == 48) {
    drawCloudIcon(x, y);
  } else if ((weatherCode >= 51 && weatherCode <= 67) ||
             (weatherCode >= 80 && weatherCode <= 82)) {
    drawRainIcon(x, y);
  } else {
    drawCloudIcon(x, y);
  }
}

String weatherText(int weatherCode) {
  if (weatherCode == 0) return "Sunny";
  if (weatherCode == 1) return "Mostly clear";
  if (weatherCode == 2) return "Partly cloudy";
  if (weatherCode == 3) return "Cloudy";
  if (weatherCode == 45 || weatherCode == 48) return "Fog";
  if (weatherCode >= 51 && weatherCode <= 67) return "Rain";
  if (weatherCode >= 71 && weatherCode <= 77) return "Snow";
  if (weatherCode >= 80 && weatherCode <= 82) return "Showers";
  return "Weather";
}

// =========================
// PRESSURE HISTORY / TREND
// =========================
void clearPressureHistory() {
  pressureHistoryCount = 0;
  pressureHistoryHead = 0;
  for (int i = 0; i < PRESSURE_HISTORY_SIZE; i++) {
    pressureHistory[i] = NAN;
  }
}

void pushPressureHistory(float p) {
  if (isnan(p)) return;

  pressureHistory[pressureHistoryHead] = p;
  pressureHistoryHead = (pressureHistoryHead + 1) % PRESSURE_HISTORY_SIZE;

  if (pressureHistoryCount < PRESSURE_HISTORY_SIZE) {
    pressureHistoryCount++;
  }
}

float getOldestPressureHistory() {
  if (pressureHistoryCount < PRESSURE_HISTORY_SIZE) return NAN;
  return pressureHistory[pressureHistoryHead];
}

bool seedPressureHistoryFromThingSpeak() {
  if (strlen(THINGSPEAK_READ_API_KEY) == 0) {
    Serial.println("ThingSpeak read key missing, cannot seed pressure history");
    return false;
  }

  String url = "https://api.thingspeak.com/channels/" + String(THINGSPEAK_CHANNEL_ID) +
               "/fields/4.json?api_key=" + String(THINGSPEAK_READ_API_KEY) +
               "&results=" + String(PRESSURE_HISTORY_SIZE);

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  if (!http.begin(secureClient, url)) {
    Serial.println("ThingSpeak seed begin failed");
    return false;
  }

  int httpCode = http.GET();
  Serial.print("ThingSpeak seed HTTP code: ");
  Serial.println(httpCode);

  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("ThingSpeak seed JSON error: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray feeds = doc["feeds"].as<JsonArray>();
  if (feeds.isNull() || feeds.size() == 0) {
    Serial.println("ThingSpeak seed: no feeds");
    return false;
  }

  clearPressureHistory();

  int validCount = 0;
  for (JsonVariant feed : feeds) {
    if (!feed["field4"].isNull()) {
      const char* s = feed["field4"];
      if (s != nullptr && strlen(s) > 0) {
        float p = atof(s);
        if (p > 800.0f && p < 1200.0f) {
          pushPressureHistory(p);
          validCount++;
        }
      }
    }
  }

  Serial.print("ThingSpeak seed valid pressure samples: ");
  Serial.println(validCount);

  return pressureHistoryCount >= 2;
}

bool seedPressureReferenceFromWeatherService() {
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(FORECAST_LAT, 4) +
               "&longitude=" + String(FORECAST_LON, 4) +
               "&hourly=surface_pressure&past_hours=4&forecast_hours=1&timezone=auto";

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  if (!http.begin(secureClient, url)) {
    Serial.println("Weather fallback begin failed");
    return false;
  }

  int httpCode = http.GET();
  Serial.print("Weather fallback HTTP code: ");
  Serial.println(httpCode);

  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("Weather fallback JSON error: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray arr = doc["hourly"]["surface_pressure"].as<JsonArray>();
  if (arr.isNull() || arr.size() < 4) {
    Serial.println("Weather fallback insufficient pressure data");
    return false;
  }

  pressureWeatherReferenceHpa = arr[0].as<float>();
  pressureWeatherFallbackValid = !isnan(pressureWeatherReferenceHpa);

  Serial.print("Weather fallback reference pressure: ");
  Serial.println(pressureWeatherReferenceHpa);

  return pressureWeatherFallbackValid;
}

String pressureTrendSymbol() {
  if (!isnan(currentPressureHpa)) {
    float reference = getOldestPressureHistory();
    if (!isnan(reference)) {
      float delta = currentPressureHpa - reference;
      if (delta > PRESSURE_TREND_THRESHOLD_HPA) return "^";
      if (delta < -PRESSURE_TREND_THRESHOLD_HPA) return "v";
      return "-";
    }

    if (pressureWeatherFallbackValid && !isnan(pressureWeatherReferenceHpa)) {
      float delta = currentPressureHpa - pressureWeatherReferenceHpa;
      if (delta > PRESSURE_TREND_THRESHOLD_HPA) return "^";
      if (delta < -PRESSURE_TREND_THRESHOLD_HPA) return "v";
      return "-";
    }
  }

  return "?";
}

// =========================
// SENSORS
// =========================
void initSensors() {
  Wire.begin(I2C_SDA, I2C_SCL);

  if (sht31.begin(SHT31_ADDRESS_PRIMARY)) {
    sht31Found = true;
    sht31Address = SHT31_ADDRESS_PRIMARY;
    Serial.println("SHT31D found at 0x44");
  } else if (sht31.begin(SHT31_ADDRESS_SECONDARY)) {
    sht31Found = true;
    sht31Address = SHT31_ADDRESS_SECONDARY;
    Serial.println("SHT31D found at 0x45");
  } else {
    sht31Found = false;
    Serial.println("SHT31D not found");
  }

  if (bmp.begin()) {
    bmpFound = true;
    Serial.println("BMP180 found");
  } else {
    Serial.println("BMP180 not found");
  }

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    bh1750Found = true;
    Serial.println("BH1750 found");
  } else {
    Serial.println("BH1750 not found");
  }
}

void readLocalSensors() {
  if (sht31Found) {
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      currentTempC = t;
      currentHumidity = h;
      sht31FailCount = 0;
    } else {
      sht31FailCount++;
      Serial.print("SHT31D read failed, count: ");
      Serial.println(sht31FailCount);

      if (sht31FailCount >= SHT31_REINIT_AFTER_FAILS) {
        Serial.println("Reinitialising SHT31D");
        sht31.begin(sht31Address);
        sht31FailCount = 0;
      }
    }
  } else {
    currentTempC = NAN;
    currentHumidity = NAN;
  }

  if (bmpFound) {
    currentPressureHpa = bmp.readPressure() / 100.0f;
  } else {
    currentPressureHpa = NAN;
  }

  if (bh1750Found) {
    currentLux = lightMeter.readLightLevel();
  } else {
    currentLux = NAN;
  }

  Serial.println("Local readings:");
  Serial.print("Temp : "); Serial.println(currentTempC);
  Serial.print("Hum  : "); Serial.println(currentHumidity);
  Serial.print("Light: "); Serial.println(currentLux);
  Serial.print("Pres : "); Serial.println(currentPressureHpa);
  Serial.print("3h ref: "); Serial.println(getOldestPressureHistory());
  Serial.print("WX ref: "); Serial.println(pressureWeatherReferenceHpa);
  Serial.print("Trend: "); Serial.println(pressureTrendSymbol());
}

// =========================
// THINGSPEAK
// =========================
void uploadThingSpeak() {
  if (!isnan(currentTempC))       ThingSpeak.setField(1, currentTempC);
  if (!isnan(currentHumidity))    ThingSpeak.setField(2, currentHumidity);
  if (!isnan(currentLux))         ThingSpeak.setField(3, currentLux);
  if (!isnan(currentPressureHpa)) ThingSpeak.setField(4, currentPressureHpa);

  ThingSpeak.setStatus(String("RSSI=") + WiFi.RSSI() + " dBm");

  int rc = ThingSpeak.writeFields(THINGSPEAK_CHANNEL_ID, THINGSPEAK_WRITE_API_KEY);
  Serial.print("ThingSpeak response: ");
  Serial.println(rc);
}

// =========================
// FORECAST
// =========================
bool fetchForecast() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(FORECAST_LAT, 4) +
               "&longitude=" + String(FORECAST_LON, 4) +
               "&daily=weather_code,temperature_2m_max,temperature_2m_min"
               "&forecast_days=4&timezone=auto";

  WiFiClientSecure secureClient;
  secureClient.setInsecure();

  HTTPClient http;
  if (!http.begin(secureClient, url)) {
    Serial.println("Forecast HTTP begin failed");
    return false;
  }

  int httpCode = http.GET();
  Serial.print("Forecast HTTP code: ");
  Serial.println(httpCode);

  if (httpCode != HTTP_CODE_OK) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("Forecast JSON error: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonArray times = doc["daily"]["time"].as<JsonArray>();
  JsonArray codes = doc["daily"]["weather_code"].as<JsonArray>();
  JsonArray tmaxs = doc["daily"]["temperature_2m_max"].as<JsonArray>();
  JsonArray tmins = doc["daily"]["temperature_2m_min"].as<JsonArray>();

  if (times.size() < 4 || codes.size() < 4 || tmaxs.size() < 4 || tmins.size() < 4) {
    Serial.println("Forecast arrays too small");
    return false;
  }

  for (int i = 0; i < 4; i++) {
    forecast[i].date = String((const char*)times[i]);
    forecast[i].weatherCode = codes[i].as<int>();
    forecast[i].tMax = tmaxs[i].as<float>();
    forecast[i].tMin = tmins[i].as<float>();
  }

  forecastValid = true;
  Serial.println("Forecast updated");
  return true;
}

// =========================
// DISPLAY PAGES
// =========================
void initDisplay() {
  display.init();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 12, "Booting...");
  display.display();
}

void showPressureSeedStatus() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 4, "Boot complete");

  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 20, "Pressure");

  display.setFont(ArialMT_Plain_10);
  display.drawString(64, 40, pressureSeedStatus);
  display.display();
}

void drawClockPage() {
  display.clear();

  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 0, getDayString());

  display.setFont(ArialMT_Plain_24);
  display.drawString(64, 18, getTime12hString());

  display.setFont(ArialMT_Plain_16);
  display.drawString(64, 48, getDateString());

  display.display();
}

void drawSensorsPage() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  {
    // Temperature in the first quadrant 0-63, 0-31
    String s = "Temperature";
    display.setFont(ArialMT_Plain_10);
    drawCenteredBoldString(32, 8, s);
    s = isnan(currentTempC) ? "--" : degC1(currentTempC);
    display.setFont(ArialMT_Plain_16);
    drawCenteredBoldString(32, 24, s);
  }

  {
    // Humidity in the second quadrant
    String s = "Humidity";
    display.setFont(ArialMT_Plain_10);
    drawCenteredBoldString(96, 8, s);
    s = isnan(currentHumidity) ? "--" : String(currentHumidity, 1) + "%";
    display.setFont(ArialMT_Plain_16);
    drawCenteredBoldString(96, 24, s);
  }

  {
    // Barometric Pressure
    String s = "Pressure";
    display.setFont(ArialMT_Plain_10);
    drawCenteredBoldString(32, 40, s);
    s = isnan(currentPressureHpa) ? "--" : String(currentPressureHpa, 0) + "hPa " + pressureTrendSymbol();
    display.setFont(ArialMT_Plain_16);
    drawCenteredBoldString(32, 56, s);
  }

  {
    // Light sensor
    String s = "Light";
    display.setFont(ArialMT_Plain_10);
    drawCenteredBoldString(96, 40, s);
    s = isnan(currentLux) ? "--" : String(currentLux, 0) + "lx";
    drawCenteredBoldString(96, 56, s);
  }

  display.display();
}

void drawTodayForecastPage() {
  display.clear();

  if (!forecastValid) {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 18, "No forecast");
    display.display();
    return;
  }

  // Blue zone: icon in top-left corner
  drawWeatherIcon(forecast[0].weatherCode, 12, 14);

  // Blue zone: weather and day left-justified to the right of icon
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(28, 2, weatherText(forecast[0].weatherCode));

  display.setFont(ArialMT_Plain_16);
  display.drawString(28, 14, fullDayNameFromISO(forecast[0].date));

  // Blue zone: temp/humidity at bottom of blue section
  display.drawString(4, 30, isnan(currentTempC) ? "--" : degC1(currentTempC));
  display.drawString(76, 30, isnan(currentHumidity) ? "--" : String(currentHumidity, 0) + "%");

  // Yellow zone
  display.drawString(4, 48, "L " + degC((int)round(forecast[0].tMin)));
  display.drawString(94, 48, "H " + degC((int)round(forecast[0].tMax)));

  display.display();
}

void drawThreeDayForecastPage() {
  display.clear();

  if (!forecastValid) {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_16);
    display.drawString(64, 18, "No forecast");
    display.display();
    return;
  }

  const int colX[3] = {21, 64, 107};

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);

  for (int i = 0; i < 3; i++) {
    int f = i + 1;
    String day = shortDayNameFromISO(forecast[f].date);

    drawCenteredBoldString(colX[i], 2, day);
    drawWeatherIcon(forecast[f].weatherCode, colX[i], 20);
  }

  display.setFont(ArialMT_Plain_16);
  for (int i = 0; i < 3; i++) {
    int f = i + 1;
    String temps = String((int)round(forecast[f].tMin)) + "/" + String((int)round(forecast[f].tMax));
    int x = colX[i] - display.getStringWidth(temps) / 2;
    display.drawString(x, 48, temps);
  }

  display.display();
}

void updateDisplay() {
  switch (pageIndex) {
    case 0:
      drawClockPage();
      break;
    case 1:
      drawSensorsPage();
      break;
    case 2:
      drawTodayForecastPage();
      break;
    default:
      drawThreeDayForecastPage();
      break;
  }
}

// =========================
// MQTT
// =========================
String mqttTopic(const char* suffix) {
  return String(MQTT_BASE_TOPIC) + "/" + suffix;
}

bool mqttConfigured() {
  return strlen(MQTT_HOST) > 0;
}

void mqttPublish(const char* suffix, const String& payload, bool retained = false) {
  if (!mqttConfigured() || !mqttClient.connected()) return;
  String topic = mqttTopic(suffix);
  mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

void mqttPublishFloat(const char* suffix, float value, uint8_t decimals, bool retained = false) {
  if (isnan(value)) return;
  mqttPublish(suffix, String(value, decimals), retained);
}

void publishMqttReadings() {
  mqttPublishFloat("sensor/temperature", currentTempC, 2, true);
  mqttPublishFloat("sensor/humidity", currentHumidity, 2, true);
  mqttPublishFloat("sensor/pressure", currentPressureHpa, 2, true);
  mqttPublishFloat("sensor/lux", currentLux, 1, true);

  mqttPublish("status/rssi", String(WiFi.RSSI()), true);
  mqttPublish("status/ip", WiFi.localIP().toString(), true);
  mqttPublish("status/page", String(pageIndex), true);
  mqttPublish("status/uptime_ms", String(millis()), false);
}

void mqttPublishRaw(const String& topic, const String& payload, bool retained = false) {
  if (!mqttConfigured() || !mqttClient.connected()) return;
  mqttClient.publish(topic.c_str(), payload.c_str(), retained);
}

String discoveryDeviceJson() {
  return "\"dev\":{\"ids\":[\"weatherstation_esp8266\"],\"name\":\"Weather Station\",\"mf\":\"Dave\",\"mdl\":\"ESP8266 NodeMCU\"}";
}

void publishDiscoverySensor(
  const char* objectId,
  const char* name,
  const char* stateTopic,
  const char* unit,
  const char* deviceClass,
  const char* stateClass
) {
  String topic = String("homeassistant/sensor/weatherstation_") + objectId + "/config";

  String payload = "{";
  payload += "\"name\":\"";
  payload += name;
  payload += "\",\"uniq_id\":\"weatherstation_";
  payload += objectId;
  payload += "\",\"stat_t\":\"";
  payload += stateTopic;
  payload += "\"";

  if (strlen(unit) > 0) {
    payload += ",\"unit_of_meas\":\"";
    payload += unit;
    payload += "\"";
  }

  if (strlen(deviceClass) > 0) {
    payload += ",\"dev_cla\":\"";
    payload += deviceClass;
    payload += "\"";
  }

  if (strlen(stateClass) > 0) {
    payload += ",\"stat_cla\":\"";
    payload += stateClass;
    payload += "\"";
  }

  payload += ",\"avty_t\":\"weatherstation/status/state\"";
  payload += ",\"pl_avail\":\"online\"";
  payload += ",\"pl_not_avail\":\"offline\"";
  payload += ",";
  payload += discoveryDeviceJson();
  payload += "}";

  mqttPublishRaw(topic, payload, true);
}

void publishHomeAssistantDiscovery() {
  publishDiscoverySensor(
    "temperature",
    "Temperature",
    "weatherstation/sensor/temperature",
    "°C",
    "temperature",
    "measurement"
  );

  publishDiscoverySensor(
    "humidity",
    "Humidity",
    "weatherstation/sensor/humidity",
    "%",
    "humidity",
    "measurement"
  );

  publishDiscoverySensor(
    "pressure",
    "Pressure",
    "weatherstation/sensor/pressure",
    "hPa",
    "atmospheric_pressure",
    "measurement"
  );

  publishDiscoverySensor(
    "lux",
    "Light",
    "weatherstation/sensor/lux",
    "lx",
    "illuminance",
    "measurement"
  );

  publishDiscoverySensor(
    "rssi",
    "WiFi RSSI",
    "weatherstation/status/rssi",
    "dBm",
    "signal_strength",
    "measurement"
  );
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;

  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  msg.trim();

  if (t == mqttTopic("cmd/reboot")) {
    if (msg == "1" || msg == "ON" || msg == "reboot") {
      mqttPublish("status/state", "rebooting", true);
      delay(250);
      ESP.restart();
    }
  }

  if (t == mqttTopic("cmd/force_read")) {
    if (msg == "1" || msg == "ON" || msg == "read") {
      readLocalSensors();
      publishMqttReadings();
      updateDisplay();
    }
  }

  if (t == mqttTopic("cmd/force_upload")) {
    if (msg == "1" || msg == "ON" || msg == "upload") {
      uploadThingSpeak();
    }
  }

  if (t == mqttTopic("cmd/page")) {
    int requestedPage = msg.toInt();
    if (requestedPage >= 0 && requestedPage <= 3) {
      pageIndex = requestedPage;
      mqttPublish("status/page", String(pageIndex), true);
      updateDisplay();
    }
  }
}

void connectMQTT() {
  if (!mqttConfigured()) return;
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastMqttConnectAttempt < MQTT_RECONNECT_INTERVAL_MS) return;
  lastMqttConnectAttempt = now;

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(768);

  Serial.print("Connecting to MQTT ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  bool connected = false;

  if (strlen(MQTT_USER) > 0) {
    connected = mqttClient.connect(
      MQTT_CLIENT_ID,
      MQTT_USER,
      MQTT_PASSWORD,
      mqttTopic("status/state").c_str(),
      0,
      true,
      "offline"
    );
  } else {
    connected = mqttClient.connect(
      MQTT_CLIENT_ID,
      mqttTopic("status/state").c_str(),
      0,
      true,
      "offline"
    );
  }

  if (connected) {
    Serial.println("MQTT connected");

    mqttPublish("status/state", "online", true);
    mqttPublish("status/ip", WiFi.localIP().toString(), true);
    mqttPublish("status/rssi", String(WiFi.RSSI()), true);
    mqttPublish("status/ota", "ready", true);

    mqttClient.subscribe(mqttTopic("cmd/reboot").c_str());
    mqttClient.subscribe(mqttTopic("cmd/force_read").c_str());
    mqttClient.subscribe(mqttTopic("cmd/force_upload").c_str());
    mqttClient.subscribe(mqttTopic("cmd/page").c_str());

    publishHomeAssistantDiscovery();
    publishMqttReadings();
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// =========================
// OTA
// =========================
void initOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA start");
    mqttPublish("status/ota", "starting", true);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA end");
    mqttPublish("status/ota", "complete", true);
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = (progress * 100U) / total;
    Serial.printf("OTA progress: %u%%\n", pct);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA error: %u\n", error);
    mqttPublish("status/ota", String("error ") + String(error), true);
  });

  ArduinoOTA.begin();
  Serial.println("OTA ready");
  mqttPublish("status/ota", "ready", true);
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);

  clearPressureHistory();

  initDisplay();
  initSensors();
  connectWiFi();
  initTime();
  ThingSpeak.begin(tsClient);
  connectMQTT();
  initOTA();

  readLocalSensors();
  fetchForecast();

  if (seedPressureHistoryFromThingSpeak()) {
    pressureSeedStatus = "History loaded";
  } else if (seedPressureReferenceFromWeatherService()) {
    pressureSeedStatus = "Weather fallback";
  } else {
    pressureSeedStatus = "Warm-up only";
    if (!isnan(currentPressureHpa)) {
      pushPressureHistory(currentPressureHpa);
    }
  }

  showPressureSeedStatus();
  delay(1800);

  updateDisplay();

  lastSensorRead = millis();
  lastSensorUpload = millis() - SENSOR_UPLOAD_INTERVAL_MS + 5000UL;
  lastForecastFetch = millis();
  lastPageChange = millis();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    initTime();
  }

  connectMQTT();
  mqttClient.loop();
  ArduinoOTA.handle();

  unsigned long now = millis();

  if (now - lastPageChange >= PAGE_ROTATE_MS) {
    lastPageChange = now;
    pageIndex = (pageIndex + 1) % 4;
    updateDisplay();
  }

  if (now - lastSensorRead >= SENSOR_READ_INTERVAL_MS) {
    lastSensorRead = now;
    readLocalSensors();
    publishMqttReadings();
    updateDisplay();
  }

  if (now - lastForecastFetch >= FORECAST_REFRESH_MS) {
    lastForecastFetch = now;
    fetchForecast();
    updateDisplay();
  }

  if (now - lastSensorUpload >= SENSOR_UPLOAD_INTERVAL_MS) {
    lastSensorUpload = now;

    pushPressureHistory(currentPressureHpa);

    uploadThingSpeak();
    updateDisplay();
  }
}