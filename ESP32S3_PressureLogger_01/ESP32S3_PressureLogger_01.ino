/**
 * ESP32-S3 Pressure Sensor - MicroSD Data Logger
 *
 * Target MCU : Freenove ESP32-S3 WROOM Camera Module
 * Sensor (Digital) : WNK Series (WNK21 / WD19 / WNK811) I2C, 3.3V
 * Sensor (Analog)  : Analog pressure sensor, 0.5V~4.5V output (reads within 3.3V)
 * Storage    : MicroSD via SDMMC 1-bit (CMD/D0/CLK)  →  LOG0001.csv, LOG0002.csv ...
 * Time       : Single NTP sync on boot, then millis()-based relative time
 *              (Replace with DS3231 library when RTC module is connected)
 *
 * ──────────────────────────────────────────────────────────────
 *  Pin Wiring  (Freenove ESP32-S3 WROOM, camera not used)
 * ──────────────────────────────────────────────────────────────
 *  Function      ESP32-S3 GPIO   → Target
 *  I2C SDA       GPIO 21         → Digital Sensor SDA
 *  I2C SCL       GPIO 20         → Digital Sensor SCL  (ESP32-S3 has no GPIO22)
 *  ADC IN        GPIO 4          → Analog Sensor VOUT  (ADC1 CH3, 0~3.3V)
 *  SD CMD        GPIO 38         → MicroSD CMD
 *  SD D0         GPIO 40         → MicroSD DATA
 *  SD CLK        GPIO 39         → MicroSD CLK
 *  3.3V                          → Sensor VCC, SD VCC
 *  GND                           → Sensor GND, SD GND
 * ──────────────────────────────────────────────────────────────
 *
 * Sensor selection (SENSOR_TYPE):
 *   0 = I2C digital sensor only  (WNK)
 *       CSV: timestamp_ms, datetime, pressure_mbar, pressure_avg_mbar, temperature_c, temperature_avg_c
 *   1 = ADC analog sensor only
 *       CSV: timestamp_ms, datetime, adc_pressure_mbar, adc_pressure_avg_mbar
 *
 * Sampling interval guide:
 *   SAMPLE_INTERVAL_MS = 100   → 10  Hz  (immediate write per sample, recommended)
 *   SAMPLE_INTERVAL_MS =  10   → 100 Hz  (buffering required, use BUF_FLUSH_COUNT)
 *   SAMPLE_INTERVAL_MS =   2   → 500 Hz  (PSRAM buffer required)
 *
 * Required libraries:
 *   - SD_MMC (ESP32 built-in)
 *   - Wire (Arduino built-in)  [SENSOR_TYPE=0 only]
 *   - WiFi (ESP32 built-in) - for NTP sync; set USE_NTP 0 if not needed
 */

#include <Wire.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include "time.h"
#include <Adafruit_NeoPixel.h>

// ==================== User Settings ====================

// Use NTP (1=enabled, 0=millis()-based relative time only)
#define USE_NTP            1

// Select sensor type (choose one):
//   0 = I2C digital sensor only  (WNK Series)
//   1 = ADC analog sensor only
#define SENSOR_TYPE        1

// WiFi credentials (required only when USE_NTP=1)
const char* WIFI_SSID     = "KT_GiGA_9748";
const char* WIFI_PASSWORD = "9cf0bkd529";

// Sampling interval (ms)
//   100ms = 10Hz  → stable immediate write recommended
//    10ms = 100Hz → enable BUF_FLUSH_COUNT
#define SAMPLE_INTERVAL_MS   10

// SD write buffering
//   1  = flush every sample (slow, stable)
//  10+ = flush every N samples (use for high-speed sampling)
// #define BUF_FLUSH_COUNT       1
#define BUF_FLUSH_COUNT       10

// Max rows per file (auto-rotate to next file when exceeded)
// 0 = unlimited
// #define MAX_ROWS_PER_FILE  10000
#define MAX_ROWS_PER_FILE  360000

// ==================== I2C Pins / Sensor Settings ====================
#define I2C_SDA_PIN        21       // ESP32-S3 valid pin (GPIO 0~21, 26~48)
#define I2C_SCL_PIN        20       // GPIO 22 does not exist on ESP32-S3 → use GPIO 20
#define I2C_CLOCK_FREQ     400000   // 400kHz Fast mode

#define SENSOR_I2C_ADDR    0x6D
#define REG_PRESSURE       0x06
#define REG_TEMPERATURE    0x09

#define PRESSURE_RANGE     2000.0f  // kPa
#define KPA_TO_MBAR        10.0f
#define PRESSURE_OFFSET    70.0f    // mbar zero offset

// ==================== ADC Pressure Sensor Settings ====================
// (active only when USE_ADC_SENSOR=1)
//
// Sensor output: 0.5V = 0 kPa,  4.5V = ADC_SENSOR_RANGE_KPA
// Circuit currently outputs within 3.3V, so ADC reads 0~3.3V.
// Use GPIO 1~10 (ADC1) to avoid conflict with WiFi (ADC2 = GPIO 11~20).
//
#define ADC_PRESSURE_PIN      1         // GPIO1 (ADC1 CH3)
#define ADC_SENSOR_RANGE_KPA  2000.0f   // Full-scale pressure (kPa)
#define ADC_SENSOR_VMIN       0.5f      // Voltage at 0 kPa (V)
#define ADC_SENSOR_VMAX       4.5f      // Voltage at full scale (V)
#define ADC_VREF              3.3f      // ESP32-S3 ADC reference (V)
#define ADC_BITS              12        // ADC resolution (12-bit → 0~4095)

// ==================== SD Card Pins (SDMMC 1-bit) ====================
#define SD_CMD_PIN        38
#define SD_D0_PIN         40
#define SD_CLK_PIN        39

// ==================== Moving Average Filter ====================
#define FILTER_SIZE        10

typedef struct
{
  float   buffer[FILTER_SIZE];
  uint8_t index;
  uint8_t count;
  float   sum;
} MovingAvgFilter_t;

// ==================== RGB LED (WS2812B - GPIO48) ====================
#define RGB_LED_PIN       48
#define RGB_LED_COUNT      1

Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

enum LedState {
  LED_BOOTING,              // 흰색  점등          - 초기화 중
  LED_WIFI_CONNECTING,      // 파란색 빠른 깜빡임   - WiFi/NTP 연결 중
  LED_LOGGING,              // 초록색 느린 깜빡임   - 정상 로깅 중
  LED_SD_ERROR,             // 빨간색 빠른 깜빡임   - SD 카드 오류
  LED_SENSOR_ERROR,         // 마젠타 빠른 깜빡임   - 센서 미감지
  LED_SENSOR_DISCONNECTED   // 빨간색 점등          - 센서 연결 끊김
};

static LedState ledState = LED_BOOTING;

void ledSetState(LedState s) { ledState = s; }

void ledUpdate()
{
  static unsigned long lastToggle = 0;
  static bool          blinkOn    = true;
  static LedState      lastState  = LED_BOOTING;
  unsigned long now = millis();

  // 상태 전환 시 깜빡임 위상 리셋
  if (ledState != lastState)
  {
    lastState  = ledState;
    blinkOn    = true;
    lastToggle = now;
  }

  uint32_t      color    = 0;
  unsigned long interval = 500;
  bool          blink    = false;

  switch (ledState)
  {
    case LED_BOOTING:
      color = rgbLed.Color(30, 30, 30);                              break; // 흰색  점등
    case LED_WIFI_CONNECTING:
      color = rgbLed.Color(0, 0, 60);  blink = true; interval = 300; break; // 파란색 300ms
    case LED_LOGGING:
      color = rgbLed.Color(0, 60, 0);  blink = true; interval = 1000; break; // 초록색 1s
    case LED_SD_ERROR:
      color = rgbLed.Color(60, 0, 0);  blink = true; interval = 150;  break; // 빨간색 150ms
    case LED_SENSOR_ERROR:
      color = rgbLed.Color(60, 0, 60); blink = true; interval = 150;  break; // 마젠타 150ms
    case LED_SENSOR_DISCONNECTED:
      color = rgbLed.Color(60, 0, 0);                                 break; // 빨간색 점등
  }

  if (blink)
  {
    if (now - lastToggle >= interval) { lastToggle = now; blinkOn = !blinkOn; }
    rgbLed.setPixelColor(0, blinkOn ? color : 0);
  }
  else
  {
    rgbLed.setPixelColor(0, color);
  }
  rgbLed.show();
}

MovingAvgFilter_t pressureFilter;
#if SENSOR_TYPE == 0
MovingAvgFilter_t temperatureFilter;
#endif

void filterInit(MovingAvgFilter_t *f)
{
  for (int i = 0; i < FILTER_SIZE; i++)
  {
    f->buffer[i] = 0.0f;
  }
  f->index = 0;
  f->count = 0;
  f->sum   = 0.0f;
}

float filterUpdate(MovingAvgFilter_t *f, float v)
{
  if (f->count >= FILTER_SIZE)
  {
    f->sum -= f->buffer[f->index];
  }
  else
  {
    f->count++;
  }
  f->buffer[f->index] = v;
  f->sum += v;
  if (++f->index >= FILTER_SIZE)
  {
    f->index = 0;
  }
  return f->sum / (float)f->count;
}

// ==================== Sensor Read ====================
bool readSensor24bit(uint8_t reg, int32_t *out)
{
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }
  if (Wire.requestFrom((uint8_t)SENSOR_I2C_ADDR, (uint8_t)3) != 3)
  {
    return false;
  }

  uint32_t raw = ((uint32_t)Wire.read() << 16)
               | ((uint32_t)Wire.read() <<  8)
               |  (uint32_t)Wire.read();

  *out = (raw & 0x800000) ? (int32_t)raw - 16777216 : (int32_t)raw;
  return true;
}

float calcPressure(int32_t raw)
{
  float adc = 3.3f * (float)raw / 8388608.0f;
  // return (PRESSURE_RANGE * (adc - 0.5f) / 2.0f * KPA_TO_MBAR) + PRESSURE_OFFSET;
  
  adc = (PRESSURE_RANGE * (adc - 0.5f) / 2.0f * KPA_TO_MBAR) + PRESSURE_OFFSET;
  if (adc <= 0.0f)
  {
    adc = 0;
  }

  return adc;
}

float calcTemperature(int32_t raw)
{
  return 25.0f + (float)raw / 65536.0f;
}

// ==================== ADC Sensor Read ====================
#if SENSOR_TYPE == 1
// Returns raw ADC count (0~4095, 12-bit).
// Pressure conversion formula TBD.
int readAdcRaw()
{
  return analogRead(ADC_PRESSURE_PIN);
}
#endif

// ==================== Time ====================
static bool    ntpSynced    = false;
static int64_t epochOffsetS = 0;    // NTP epoch (s) - millis()/1000 offset

void syncNTP()
{
#if USE_NTP
  ledSetState(LED_WIFI_CONNECTING);
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry++ < 40)
  {
    // 500ms 대기하는 동안 LED 깜빡임 유지
    for (int i = 0; i < 50; i++) { ledUpdate(); delay(10); }
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(" Connected");
    configTime(9 * 3600, 0, "pool.ntp.org", "time.nist.gov"); // UTC+9 (KST)
    struct tm ti;
    if (getLocalTime(&ti, 5000))
    {
      ntpSynced = true;
      time_t now;
      time(&now);
      epochOffsetS = (int64_t)now - (int64_t)(millis() / 1000);
      Serial.printf("NTP sync OK: %04d-%02d-%02d %02d:%02d:%02d\n",
        ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday,
        ti.tm_hour, ti.tm_min, ti.tm_sec);
    }
    else
    {
      Serial.println("NTP sync failed → using millis()-based time");
    }
    // WiFi not needed during logging; disconnect to save power
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  else
  {
    Serial.println("\nWiFi failed → using millis()-based time");
    WiFi.mode(WIFI_OFF);
  }
#endif
}

// Build "YYYY-MM-DD HH:MM:SS.mmm" string
void getTimeString(char *buf, size_t len)
{
  unsigned long ms = millis();
  if (ntpSynced)
  {
    int64_t    epochMs  = epochOffsetS * 1000LL + (int64_t)ms;
    time_t     epochSec = (time_t)(epochMs / 1000);
    int        msec     = (int)(epochMs % 1000);
    struct tm *ti       = localtime(&epochSec);
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
      ti->tm_year+1900, ti->tm_mon+1, ti->tm_mday,
      ti->tm_hour, ti->tm_min, ti->tm_sec, msec);
  }
  else
  {
    // Without NTP: elapsed time format  HH:MM:SS.mmm
    unsigned long s = ms / 1000;
    unsigned long m = s  / 60;
    unsigned long h = m  / 60;
    snprintf(buf, len, "%02lu:%02lu:%02lu.%03lu",
      h, m % 60, s % 60, ms % 1000);
  }
}

// ==================== SD Card / File Management ====================
static File     logFile;
static char     logFileName[32];
static bool     sdReady      = false;
static bool     sensorReady  = false;
static uint32_t rowCount  = 0;
static uint32_t fileIndex = 0;
static uint32_t flushCnt  = 0;

// Find the next unused file index on SD
uint32_t findNextFileIndex()
{
  for (uint32_t i = 1; i <= 9999; i++)
  {
    char name[32];
    snprintf(name, sizeof(name), "/LOG%04u.csv", i);
    if (!SD_MMC.exists(name))
    {
      return i;
    }
  }
  return 9999;
}

bool openNewLogFile()
{
  if (logFile)
  {
    logFile.close();
  }

  fileIndex = findNextFileIndex();
  snprintf(logFileName, sizeof(logFileName), "/LOG%04u.csv", fileIndex);

  logFile = SD_MMC.open(logFileName, FILE_WRITE);
  if (!logFile)
  {
    Serial.printf("Failed to open file: %s\n", logFileName);
    return false;
  }

  // CSV header
#if SENSOR_TYPE == 1
  logFile.println("timestamp_ms,datetime,adc_raw,adc_raw_avg");
#else
  logFile.println("timestamp_ms,datetime,pressure_mbar,pressure_avg_mbar,temperature_c,temperature_avg_c");
#endif
  logFile.flush();
  rowCount = 0;

  Serial.printf("New file created: %s\n", logFileName);
  return true;
}

// ==================== Setup ====================
void setup()
{
  Serial.begin(115200);

  // RGB LED 초기화
  rgbLed.begin();
  rgbLed.clear();
  rgbLed.show();
  ledSetState(LED_BOOTING);
  ledUpdate();

  delay(500);

  Serial.println("=========================================");
  Serial.println("  WNK Pressure Sensor MicroSD Data Logger");
  Serial.printf ("  Sample interval: %d ms  (%.1f Hz)\n",
                 SAMPLE_INTERVAL_MS, 1000.0f / SAMPLE_INTERVAL_MS);
  Serial.printf ("  Flush interval: every %d sample(s)\n", BUF_FLUSH_COUNT);
  Serial.println("=========================================");

  filterInit(&pressureFilter);

#if SENSOR_TYPE == 1
  // ADC analog sensor init
  filterInit(&pressureFilter);
  analogSetAttenuation(ADC_11db);   // Input range 0~3.3V (11dB attenuation)
  pinMode(ADC_PRESSURE_PIN, INPUT);
  sensorReady = true;
  Serial.printf("ADC sensor enabled  GPIO: %d\n", ADC_PRESSURE_PIN);
#else
  // I2C digital sensor init
  filterInit(&temperatureFilter);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);

  // Sensor connection check
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  if (Wire.endTransmission() == 0)
  {
    sensorReady = true;
    Serial.printf("I2C sensor detected  Address: 0x%02X\n", SENSOR_I2C_ADDR);
  }
  else
  {
    sensorReady = false;
    ledSetState(LED_SENSOR_ERROR);
    Serial.println("[WARNING] Sensor not found! Check wiring. Sensor reads will be skipped.");
  }
#endif

  // NTP time sync (when USE_NTP=1)
  syncNTP();

  // SD card init (SDMMC 1-bit: CMD/D0/CLK)
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true))  // true = 1-bit mode
  {
    Serial.println("[ERROR] SD card initialization failed!");
    Serial.println("  → Check CMD/D0/CLK wiring and card format (FAT32)");
    sdReady = false;
  }
  else
  {
    uint64_t cardSize = SD_MMC.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("SD card ready  Size: %llu MB\n", cardSize);
    sdReady = openNewLogFile();
  }

  // 초기화 완료 후 최종 LED 상태 결정
  if (!sdReady)
    ledSetState(LED_SD_ERROR);
  else if (!sensorReady)
    ledSetState(LED_SENSOR_ERROR);
  else
    ledSetState(LED_LOGGING);

  Serial.println();
#if SENSOR_TYPE == 1
  Serial.println("timestamp_ms | datetime                 | ADC_raw   | ADC_raw_avg");
  Serial.println("-------------|--------------------------|-----------|------------");
#else
  Serial.println("timestamp_ms | datetime                 | P(mbar)   | P_avg     | T(°C)  | T_avg");
  Serial.println("-------------|--------------------------|-----------|-----------|--------|-------");
#endif
}

// ==================== Loop ====================
void loop()
{
  static unsigned long lastSample = 0;
  unsigned long now = millis();

  ledUpdate();  // LED 상태는 항상 갱신

  if (now - lastSample < (unsigned long)SAMPLE_INTERVAL_MS)
  {
    return;
  }
  lastSample = now;

  if (!sensorReady)
  {
    return;
  }

#if SENSOR_TYPE == 1
  int   adcRaw    = readAdcRaw();
  float adcRawAvg = filterUpdate(&pressureFilter, (float)adcRaw);
#else
  int32_t rawP = 0, rawT = 0;
  bool pOk = readSensor24bit(REG_PRESSURE,    &rawP);
  bool tOk = readSensor24bit(REG_TEMPERATURE, &rawT);

  if (!pOk || !tOk)
  {
    sensorReady = false;  // 연결 끊김으로 간주, 이후 루프에서 I2C 시도 중단
    ledSetState(LED_SENSOR_DISCONNECTED);
    Serial.printf("[%lu ms] Read error -", now);
    if (!pOk) Serial.print(" Pressure");
    if (!tOk) Serial.print(" Temperature");
    Serial.println(" (sensor disconnected, reads halted)");
    return;
  }

  float pressure    = calcPressure(rawP);
  float temperature = calcTemperature(rawT);
  float pressureAvg = filterUpdate(&pressureFilter,    pressure);
  float tempAvg     = filterUpdate(&temperatureFilter, temperature);
#endif

  char timeStr[32];
  getTimeString(timeStr, sizeof(timeStr));

  // Serial output
#if SENSOR_TYPE == 1
  Serial.printf("%11lu | %-24s | %9d | %11.1f\n",
    now, timeStr, adcRaw, adcRawAvg);
#else
  Serial.printf("%11lu | %-24s | %9.2f | %9.2f | %6.2f | %6.2f\n",
    now, timeStr, pressure, pressureAvg, temperature, tempAvg);
#endif

  // SD card write
  if (sdReady && logFile)
  {
    // Rotate to next file when row limit is reached
    if (MAX_ROWS_PER_FILE > 0 && rowCount >= (uint32_t)MAX_ROWS_PER_FILE)
    {
      logFile.flush();
      logFile.close();
      Serial.printf("[File rotate] %s done (%u rows)\n", logFileName, rowCount);
      if (!openNewLogFile())
      {
        sdReady = false;
        Serial.println("[ERROR] Failed to create new file - logging stopped");
        return;
      }
    }

    // Write one CSV row
#if SENSOR_TYPE == 1
    logFile.printf("%lu,%s,%d,%.1f\n",
      now, timeStr, adcRaw, adcRawAvg);
#else
    logFile.printf("%lu,%s,%.2f,%.2f,%.2f,%.2f\n",
      now, timeStr, pressure, pressureAvg, temperature, tempAvg);
#endif
    rowCount++;

    // Flush every BUF_FLUSH_COUNT samples
    if (++flushCnt >= (uint32_t)BUF_FLUSH_COUNT)
    {
      logFile.flush();
      flushCnt = 0;
    }
  }
}
