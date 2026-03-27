/**
 * ESP32-S3 Digital Pressure Sensor - MicroSD Data Logger
 *
 * Target MCU : Freenove ESP32-S3 WROOM Camera Module
 * Sensor     : WNK Series (WNK21 / WD19 / WNK811) I2C, 3.3V
 * Storage    : MicroSD via SDMMC 1-bit (CMD/D0/CLK)  →  LOG0001.csv, LOG0002.csv ...
 * Time       : Single NTP sync on boot, then millis()-based relative time
 *              (Replace with DS3231 library when RTC module is connected)
 *
 * ──────────────────────────────────────────────────────────────
 *  Pin Wiring  (Freenove ESP32-S3 WROOM, camera not used)
 * ──────────────────────────────────────────────────────────────
 *  Function      ESP32-S3 GPIO   → Target
 *  I2C SDA       GPIO 21         → Sensor SDA
 *  I2C SCL       GPIO 20         → Sensor SCL  (ESP32-S3 has no GPIO22)
 *  SD CMD        GPIO 38         → MicroSD CMD
 *  SD D0         GPIO 40         → MicroSD DATA
 *  SD CLK        GPIO 39         → MicroSD CLK
 *  3.3V                          → Sensor VCC, SD VCC
 *  GND                           → Sensor GND, SD GND
 * ──────────────────────────────────────────────────────────────
 *
 * CSV output format:
 *   timestamp_ms, datetime, pressure_mbar, pressure_avg, temp_c, temp_avg
 *
 * Sampling interval guide:
 *   SAMPLE_INTERVAL_MS = 100   → 10  Hz  (immediate write per sample, recommended)
 *   SAMPLE_INTERVAL_MS =  10   → 100 Hz  (buffering required, use BUF_FLUSH_COUNT)
 *   SAMPLE_INTERVAL_MS =   2   → 500 Hz  (PSRAM buffer required)
 *
 * Required libraries:
 *   - SD_MMC (ESP32 built-in)
 *   - Wire (Arduino built-in)
 *   - WiFi (ESP32 built-in) - for NTP sync; set USE_NTP 0 if not needed
 */

#include <Wire.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include "time.h"

// ==================== User Settings ====================

// Use NTP (1=enabled, 0=millis()-based relative time only)
#define USE_NTP            1

// WiFi credentials (required only when USE_NTP=1)
const char* WIFI_SSID     = "KT_GiGA_9748";
const char* WIFI_PASSWORD = "9cf0bkd529";

// Sampling interval (ms)
//   100ms = 10Hz  → stable immediate write recommended
//    10ms = 100Hz → enable BUF_FLUSH_COUNT
#define SAMPLE_INTERVAL_MS   100

// SD write buffering
//   1  = flush every sample (slow, stable)
//  10+ = flush every N samples (use for high-speed sampling)
#define BUF_FLUSH_COUNT       1

// Max rows per file (auto-rotate to next file when exceeded)
// 0 = unlimited
#define MAX_ROWS_PER_FILE  10000

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

MovingAvgFilter_t pressureFilter;
MovingAvgFilter_t temperatureFilter;

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
  return (PRESSURE_RANGE * (adc - 0.5f) / 2.0f * KPA_TO_MBAR) + PRESSURE_OFFSET;
}

float calcTemperature(int32_t raw)
{
  return 25.0f + (float)raw / 65536.0f;
}

// ==================== Time ====================
static bool    ntpSynced    = false;
static int64_t epochOffsetS = 0;    // NTP epoch (s) - millis()/1000 offset

void syncNTP()
{
#if USE_NTP
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry++ < 40)
  {
    delay(500);
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
  logFile.println("timestamp_ms,datetime,pressure_mbar,pressure_avg_mbar,temperature_c,temperature_avg_c");
  logFile.flush();
  rowCount = 0;

  Serial.printf("New file created: %s\n", logFileName);
  return true;
}

// ==================== Setup ====================
void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("=========================================");
  Serial.println("  WNK Pressure Sensor MicroSD Data Logger");
  Serial.printf ("  Sample interval: %d ms  (%.1f Hz)\n",
                 SAMPLE_INTERVAL_MS, 1000.0f / SAMPLE_INTERVAL_MS);
  Serial.printf ("  Flush interval: every %d sample(s)\n", BUF_FLUSH_COUNT);
  Serial.println("=========================================");

  // I2C init
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);

  filterInit(&pressureFilter);
  filterInit(&temperatureFilter);

  // Sensor connection check
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  if (Wire.endTransmission() == 0)
  {
    sensorReady = true;
    Serial.printf("Sensor detected  Address: 0x%02X\n", SENSOR_I2C_ADDR);
  }
  else
  {
    sensorReady = false;
    Serial.println("[WARNING] Sensor not found! Check wiring. Sensor reads will be skipped.");
  }

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

  Serial.println();
  Serial.println("timestamp_ms | datetime                 | P(mbar)   | P_avg     | T(°C)  | T_avg");
  Serial.println("-------------|--------------------------|-----------|-----------|--------|-------");
}

// ==================== Loop ====================
void loop()
{
  static unsigned long lastSample = 0;
  unsigned long now = millis();

  if (now - lastSample < (unsigned long)SAMPLE_INTERVAL_MS)
  {
    return;
  }
  lastSample = now;

  if (!sensorReady)
  {
    return;
  }

  int32_t rawP = 0, rawT = 0;
  bool pOk = readSensor24bit(REG_PRESSURE,    &rawP);
  bool tOk = readSensor24bit(REG_TEMPERATURE, &rawT);

  if (!pOk || !tOk)
  {
    sensorReady = false;  // 연결 끊김으로 간주, 이후 루프에서 I2C 시도 중단
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

  char timeStr[32];
  getTimeString(timeStr, sizeof(timeStr));

  // Serial output
  Serial.printf("%11lu | %-24s | %9.2f | %9.2f | %6.2f | %6.2f\n",
    now, timeStr, pressure, pressureAvg, temperature, tempAvg);

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
    logFile.printf("%lu,%s,%.2f,%.2f,%.2f,%.2f\n",
      now, timeStr, pressure, pressureAvg, temperature, tempAvg);
    rowCount++;

    // Flush every BUF_FLUSH_COUNT samples
    if (++flushCnt >= (uint32_t)BUF_FLUSH_COUNT)
    {
      logFile.flush();
      flushCnt = 0;
    }
  }
}
