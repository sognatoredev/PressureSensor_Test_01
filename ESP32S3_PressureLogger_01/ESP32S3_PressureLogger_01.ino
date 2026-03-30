/**
 * ESP32-S3 Pressure / Vibration Sensor - MicroSD Data Logger
 *
 * Target MCU : Freenove ESP32-S3 WROOM Camera Module
 * Sensor (Digital) : WNK Series (WNK21 / WD19 / WNK811) I2C, 3.3V
 * Sensor (Analog)  : Analog pressure sensor, 0.5V~4.5V output (reads within 3.3V)
 * Sensor (Piezo)   : Piezoelectric vibration via MAX4466EXK+ preamp → WAV file
 * Storage    : MicroSD via SDMMC 1-bit (CMD/D0/CLK)
 * Time       : Single NTP sync on boot, then millis()-based relative time
 *
 * ──────────────────────────────────────────────────────────────
 *  Pin Wiring  (Freenove ESP32-S3 WROOM, camera not used)
 * ──────────────────────────────────────────────────────────────
 *  Function      ESP32-S3 GPIO   → Target
 *  I2C SDA       GPIO 21         → Digital Sensor SDA
 *  I2C SCL       GPIO 20         → Digital Sensor SCL
 *  ADC IN        GPIO 4          → Analog Pressure Sensor VOUT  (ADC1 CH3)
 *  ADC IN        GPIO 1          → MAX4466 OUT (Piezo preamp)   (ADC1 CH0)
 *  TRIGGER       GPIO 0          → BOOT button (built-in, active LOW)
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
 *   1 = ADC analog pressure sensor only
 *       CSV: timestamp_ms, datetime, adc_raw, adc_raw_avg
 *   2 = Piezo vibration WAV recorder (triggered by BOOT button)
 *       Press BOOT button → record WAV_RECORD_SECONDS sec → save VIBxxxx.wav
 *       8000 Hz, 16-bit PCM Mono
 *       Circuit: Piezo → MAX4466EXK+ (GAIN open = 100x) → GPIO1
 *
 * Tested on Arduino ESP32 board package v3.3.0 (ESP-IDF v5.x)
 *
 * Required libraries:
 *   - SD_MMC, Wire, WiFi (ESP32 built-in)
 *   - Adafruit NeoPixel
 *   - esp_timer.h (ESP-IDF built-in) [SENSOR_TYPE=2]
 */

#include <Wire.h>
#include <SD_MMC.h>
#include <WiFi.h>
#include "time.h"
#include <Adafruit_NeoPixel.h>
#if SENSOR_TYPE == 2
#include "esp_timer.h"    // High-resolution timer (task context, analogRead safe)
#endif

// ==================== User Settings ====================

#define USE_NTP            1

// Select sensor type:
//   0 = I2C digital sensor  (WNK Series)
//   1 = ADC analog pressure sensor
//   2 = Piezo vibration WAV recorder (button-triggered)
#define SENSOR_TYPE        2

const char* WIFI_SSID     = "KT_GiGA_9748";
const char* WIFI_PASSWORD = "9cf0bkd529";

// Sampling interval (ms) — SENSOR_TYPE 0/1 only
#define SAMPLE_INTERVAL_MS   10
#define BUF_FLUSH_COUNT      10
#define MAX_ROWS_PER_FILE    360000

// ==================== I2C / WNK Sensor Settings ====================
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        20
#define I2C_CLOCK_FREQ     400000
#define SENSOR_I2C_ADDR    0x6D
#define REG_PRESSURE       0x06
#define REG_TEMPERATURE    0x09
#define PRESSURE_RANGE     2000.0f
#define KPA_TO_MBAR        10.0f
#define PRESSURE_OFFSET    70.0f

// ==================== ADC Pressure Sensor Settings (SENSOR_TYPE 1) ====================
#define ADC_PRESSURE_PIN      4         // GPIO4  (ADC1 CH3) — pressure sensor

// ==================== Piezo WAV Recorder Settings (SENSOR_TYPE 2) ====================
//
//  Circuit : Piezo → MAX4466EXK+ → GPIO1 (ADC1 CH0)
//  Trigger : BOOT button (GPIO0, active LOW, built-in pull-up)
//
//  Operation:
//    1. Boot  → LED 흰색(대기)
//    2. BOOT 버튼 누름 → LED 초록 깜빡 → WAV_RECORD_SECONDS 초 녹음
//    3. 저장 완료 → LED 흰색(대기)
//
//  Sampling note:
//    esp_timer callback runs in a high-priority task (not ISR),
//    so analogRead() is safe to call directly.
//    Timing jitter < ~10 μs, acceptable for 8 kHz audio.
//
#define WAV_ADC_PIN         1           // GPIO1 = ADC1 CH0  (MAX4466 output)
#define WAV_TRIGGER_PIN     0           // GPIO0 = BOOT button (active LOW)
#define WAV_SAMPLE_RATE     8000        // Hz
#define WAV_RECORD_SECONDS  3           // Seconds per triggered recording
#define WAV_BUF_SAMPLES     4096        // Samples per double-buffer half (~512 ms)

// ==================== SD Card Pins (SDMMC 1-bit) ====================
#define SD_CMD_PIN        38
#define SD_D0_PIN         40
#define SD_CLK_PIN        39

// ==================== Moving Average Filter ====================
#define FILTER_SIZE        10

typedef struct {
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
  LED_BOOTING,              // 흰색 점등          — 부팅 완료 / WAV 대기
  LED_WIFI_CONNECTING,      // 파란색 빠른 깜빡임 — WiFi/NTP 연결 중
  LED_LOGGING,              // 초록색 깜빡임       — 정상 로깅 / WAV 녹음 중
  LED_SD_ERROR,             // 빨간색 빠른 깜빡임 — SD 카드 오류
  LED_SENSOR_ERROR,         // 마젠타 빠른 깜빡임 — 센서 미감지
  LED_SENSOR_DISCONNECTED   // 빨간색 점등         — 센서 연결 끊김
};

static LedState ledState = LED_BOOTING;
void ledSetState(LedState s) { ledState = s; }

void ledUpdate()
{
  static unsigned long lastToggle = 0;
  static bool          blinkOn    = true;
  static LedState      lastState  = LED_BOOTING;
  unsigned long t = millis();

  if (ledState != lastState) { lastState = ledState; blinkOn = true; lastToggle = t; }

  uint32_t      color    = 0;
  unsigned long interval = 500;
  bool          blink    = false;

  switch (ledState) {
    case LED_BOOTING:
      color = rgbLed.Color(30,30,30);                                break;
    case LED_WIFI_CONNECTING:
      color = rgbLed.Color(0,0,60);   blink=true; interval=300;     break;
    case LED_LOGGING:
      color = rgbLed.Color(0,60,0);   blink=true; interval=500;     break;
    case LED_SD_ERROR:
      color = rgbLed.Color(60,0,0);   blink=true; interval=150;     break;
    case LED_SENSOR_ERROR:
      color = rgbLed.Color(60,0,60);  blink=true; interval=150;     break;
    case LED_SENSOR_DISCONNECTED:
      color = rgbLed.Color(60,0,0);                                  break;
  }

  if (blink) {
    if (t - lastToggle >= interval) { lastToggle = t; blinkOn = !blinkOn; }
    rgbLed.setPixelColor(0, blinkOn ? color : 0);
  } else {
    rgbLed.setPixelColor(0, color);
  }
  rgbLed.show();
}

// ==================== Moving Average Filter ====================
MovingAvgFilter_t pressureFilter;
#if SENSOR_TYPE == 0
MovingAvgFilter_t temperatureFilter;
#endif

void filterInit(MovingAvgFilter_t *f)
{
  for (int i = 0; i < FILTER_SIZE; i++) f->buffer[i] = 0.0f;
  f->index = 0; f->count = 0; f->sum = 0.0f;
}

float filterUpdate(MovingAvgFilter_t *f, float v)
{
  if (f->count >= FILTER_SIZE) f->sum -= f->buffer[f->index]; else f->count++;
  f->buffer[f->index] = v;
  f->sum += v;
  if (++f->index >= FILTER_SIZE) f->index = 0;
  return f->sum / (float)f->count;
}

// ==================== I2C Sensor Read (SENSOR_TYPE 0) ====================
bool readSensor24bit(uint8_t reg, int32_t *out)
{
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)SENSOR_I2C_ADDR, (uint8_t)3) != 3) return false;
  uint32_t raw = ((uint32_t)Wire.read()<<16)|((uint32_t)Wire.read()<<8)|(uint32_t)Wire.read();
  *out = (raw & 0x800000) ? (int32_t)raw - 16777216 : (int32_t)raw;
  return true;
}

float calcPressure(int32_t raw)
{
  float v = 3.3f * (float)raw / 8388608.0f;
  v = (PRESSURE_RANGE * (v - 0.5f) / 2.0f * KPA_TO_MBAR) + PRESSURE_OFFSET;
  return (v < 0.0f) ? 0.0f : v;
}

float calcTemperature(int32_t raw) { return 25.0f + (float)raw / 65536.0f; }

// ==================== ADC Sensor Read (SENSOR_TYPE 1) ====================
#if SENSOR_TYPE == 1
int readAdcRaw() { return analogRead(ADC_PRESSURE_PIN); }
#endif

// ==================== Piezo WAV Recorder (SENSOR_TYPE 2) ====================
#if SENSOR_TYPE == 2

// ── WAV State Machine ──────────────────────────────────────────────────────
enum WavState { WAV_STANDBY, WAV_RECORDING, WAV_SAVING };
static WavState wavState = WAV_STANDBY;

// ── Double buffer (4096 × 2 × 2 = 16 KB in internal SRAM) ───────────────
static int16_t           wavBuf[2][WAV_BUF_SAMPLES];
static volatile uint8_t  wavFillBuf  = 0;    // buffer currently being filled (0/1)
static volatile uint16_t wavFillPos  = 0;    // write position inside fill buffer
static volatile uint8_t  wavFlushBuf = 0;    // buffer index ready for SD write
static volatile bool     wavFlushReq = false;// true = one full buffer pending write
static volatile uint32_t wavISRCount = 0;    // total samples captured so far

// ── File tracking ─────────────────────────────────────────────────────────
static File     wavFile;
static char     wavFileName[32];
static uint32_t wavFileIndex = 0;
static uint32_t wavDataBytes = 0;   // PCM bytes written to current file

// ── esp_timer handle (replaces hw_timer; callback runs in task, not ISR) ──
static esp_timer_handle_t wavEspTimer = NULL;

// Timer callback — runs in esp_timer task context (NOT an ISR)
// analogRead() is safe here: no ISR mutex restrictions.
static void wavTimerCb(void *arg)
{
  int raw = analogRead(WAV_ADC_PIN);              // GPIO1, 12-bit 0~4095
  int16_t sample = (int16_t)((raw - 2048) << 4); // center + scale → 16-bit signed

  wavBuf[wavFillBuf][wavFillPos] = sample;
  wavISRCount++;

  if (++wavFillPos >= WAV_BUF_SAMPLES)
  {
    wavFlushBuf = wavFillBuf;
    wavFlushReq = true;
    wavFillBuf ^= 1;   // switch to other buffer
    wavFillPos  = 0;
  }
}

// Write 44-byte RIFF/WAV header (dataSize=0 as placeholder on open)
void writeWavHeader(File &f, uint32_t dataSize)
{
  const uint16_t numCh    = 1;
  const uint16_t bits     = 16;
  const uint32_t sr       = WAV_SAMPLE_RATE;
  const uint32_t byteRate = sr * numCh * bits / 8;
  const uint16_t align    = (uint16_t)(numCh * bits / 8);
  const uint32_t fmtSize  = 16;
  const uint16_t audioFmt = 1;
  uint32_t       riffSize = 36 + dataSize;

  f.write((const uint8_t*)"RIFF", 4);
  f.write((const uint8_t*)&riffSize,  4);
  f.write((const uint8_t*)"WAVE",     4);
  f.write((const uint8_t*)"fmt ",     4);
  f.write((const uint8_t*)&fmtSize,   4);
  f.write((const uint8_t*)&audioFmt,  2);
  f.write((const uint8_t*)&numCh,     2);
  f.write((const uint8_t*)&sr,        4);
  f.write((const uint8_t*)&byteRate,  4);
  f.write((const uint8_t*)&align,     2);
  f.write((const uint8_t*)&bits,      2);
  f.write((const uint8_t*)"data",     4);
  f.write((const uint8_t*)&dataSize,  4);
}

uint32_t findNextWavFileIndex()
{
  for (uint32_t i = 1; i <= 9999; i++) {
    char name[32];
    snprintf(name, sizeof(name), "/VIB%04u.wav", i);
    if (!SD_MMC.exists(name)) return i;
  }
  return 9999;
}

bool openNewWavFile()
{
  if (wavFile) wavFile.close();
  wavFileIndex = findNextWavFileIndex();
  snprintf(wavFileName, sizeof(wavFileName), "/VIB%04u.wav", wavFileIndex);
  wavFile = SD_MMC.open(wavFileName, FILE_WRITE);
  if (!wavFile) { Serial.printf("[WAV] Cannot open: %s\n", wavFileName); return false; }
  wavDataBytes = 0;
  writeWavHeader(wavFile, 0);   // placeholder; sizes updated on close
  wavFile.flush();
  Serial.printf("[WAV] File: %s\n", wavFileName);
  return true;
}

// Seek back and update RIFF/data chunk sizes, then close
void closeWavFile()
{
  if (!wavFile) return;
  uint32_t riffSize = 36 + wavDataBytes;
  wavFile.seek(4);   wavFile.write((const uint8_t*)&riffSize,    4);
  wavFile.seek(40);  wavFile.write((const uint8_t*)&wavDataBytes, 4);
  wavFile.flush();
  wavFile.close();
  float sec = (float)wavDataBytes / (float)(WAV_SAMPLE_RATE * 2);
  Serial.printf("[WAV] Saved %s  (%.2f sec, %u bytes)\n", wavFileName, sec, wavDataBytes);
}

// Start triggered recording
void wavStartRecording(bool sd)
{
  if (!sd) { Serial.println("[WAV] SD not ready"); return; }
  if (!openNewWavFile()) return;

  wavFillBuf  = 0;
  wavFillPos  = 0;
  wavFlushReq = false;
  wavISRCount = 0;
  wavState    = WAV_RECORDING;
  ledSetState(LED_LOGGING);

  // Start 8000 Hz periodic callback (125 μs interval)
  esp_timer_start_periodic(wavEspTimer, 125);

  Serial.printf("[WAV] Recording %d sec → %s\n", WAV_RECORD_SECONDS, wavFileName);
}

// Stop timer, flush remaining data, close file
void wavStopRecording(bool sd)
{
  esp_timer_stop(wavEspTimer);   // stop callback
  wavState = WAV_SAVING;

  if (!sd || !wavFile) { wavState = WAV_STANDBY; return; }

  const uint32_t maxBytes = (uint32_t)WAV_RECORD_SECONDS * WAV_SAMPLE_RATE * 2;

  // Flush any full buffer that completed just before stop
  if (wavFlushReq)
  {
    uint32_t need  = (wavDataBytes < maxBytes) ? maxBytes - wavDataBytes : 0;
    uint32_t bytes = min((uint32_t)(WAV_BUF_SAMPLES * 2), need);
    if (bytes) { wavFile.write((const uint8_t*)wavBuf[wavFlushBuf], bytes); wavDataBytes += bytes; }
    wavFlushReq = false;
  }

  // Flush partial (incomplete) buffer up to target length
  {
    uint32_t need  = (wavDataBytes < maxBytes) ? maxBytes - wavDataBytes : 0;
    uint32_t avail = (uint32_t)wavFillPos * 2;
    uint32_t bytes = min(avail, need);
    if (bytes) { wavFile.write((const uint8_t*)wavBuf[wavFillBuf], bytes); wavDataBytes += bytes; }
  }

  closeWavFile();
  wavState = WAV_STANDBY;
  ledSetState(LED_BOOTING);   // white = standby
  Serial.println("[WAV] Standby. Press BOOT button to record.");
}

#endif  // SENSOR_TYPE == 2

// ==================== Time ====================
static bool    ntpSynced    = false;
static int64_t epochOffsetS = 0;

void syncNTP()
{
#if USE_NTP
  ledSetState(LED_WIFI_CONNECTING);
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry++ < 40) {
    for (int i = 0; i < 50; i++) { ledUpdate(); delay(10); }
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" Connected");
    configTime(9*3600, 0, "pool.ntp.org", "time.nist.gov");
    struct tm ti;
    if (getLocalTime(&ti, 5000)) {
      ntpSynced = true;
      time_t now; time(&now);
      epochOffsetS = (int64_t)now - (int64_t)(millis()/1000);
      Serial.printf("NTP OK: %04d-%02d-%02d %02d:%02d:%02d\n",
        ti.tm_year+1900, ti.tm_mon+1, ti.tm_mday,
        ti.tm_hour, ti.tm_min, ti.tm_sec);
    } else { Serial.println("NTP failed → millis-based time"); }
    WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  } else {
    Serial.println("\nWiFi failed → millis-based time");
    WiFi.mode(WIFI_OFF);
  }
#endif
}

void getTimeString(char *buf, size_t len)
{
  unsigned long ms = millis();
  if (ntpSynced) {
    int64_t    em  = epochOffsetS*1000LL + (int64_t)ms;
    time_t     es  = (time_t)(em/1000);
    int        msec = (int)(em%1000);
    struct tm *ti  = localtime(&es);
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
      ti->tm_year+1900, ti->tm_mon+1, ti->tm_mday,
      ti->tm_hour, ti->tm_min, ti->tm_sec, msec);
  } else {
    unsigned long s=ms/1000, m=s/60, h=m/60;
    snprintf(buf, len, "%02lu:%02lu:%02lu.%03lu", h, m%60, s%60, ms%1000);
  }
}

// ==================== SD Card / CSV File Management ====================
static File     logFile;
static char     logFileName[32];
static bool     sdReady     = false;
static bool     sensorReady = false;
static uint32_t rowCount    = 0;
static uint32_t fileIndex   = 0;
static uint32_t flushCnt    = 0;

uint32_t findNextFileIndex()
{
  for (uint32_t i = 1; i <= 9999; i++) {
    char name[32]; snprintf(name, sizeof(name), "/LOG%04u.csv", i);
    if (!SD_MMC.exists(name)) return i;
  }
  return 9999;
}

bool openNewLogFile()
{
  if (logFile) logFile.close();
  fileIndex = findNextFileIndex();
  snprintf(logFileName, sizeof(logFileName), "/LOG%04u.csv", fileIndex);
  logFile = SD_MMC.open(logFileName, FILE_WRITE);
  if (!logFile) { Serial.printf("Failed to open: %s\n", logFileName); return false; }
#if SENSOR_TYPE == 1
  logFile.println("timestamp_ms,datetime,adc_raw,adc_raw_avg");
#else
  logFile.println("timestamp_ms,datetime,pressure_mbar,pressure_avg_mbar,temperature_c,temperature_avg_c");
#endif
  logFile.flush(); rowCount = 0;
  Serial.printf("New file: %s\n", logFileName);
  return true;
}

// ==================== Setup ====================
void setup()
{
  Serial.begin(115200);
  rgbLed.begin(); rgbLed.clear(); rgbLed.show();
  ledSetState(LED_BOOTING); ledUpdate();
  delay(500);

  Serial.println("=========================================");
#if SENSOR_TYPE == 2
  Serial.println("  Piezo Vibration WAV Recorder");
  Serial.printf ("  %d Hz · 16-bit PCM Mono · %d sec/file\n",
    WAV_SAMPLE_RATE, WAV_RECORD_SECONDS);
  Serial.println("  Press BOOT button (GPIO0) to record");
#else
  Serial.println("  WNK/ADC Pressure Sensor Data Logger");
  Serial.printf ("  %d ms (%0.f Hz) · flush every %d samples\n",
    SAMPLE_INTERVAL_MS, 1000.0f/SAMPLE_INTERVAL_MS, BUF_FLUSH_COUNT);
#endif
  Serial.println("=========================================");

  // ── Sensor init ─────────────────────────────────────────────────────────
#if SENSOR_TYPE == 0
  filterInit(&pressureFilter);
  filterInit(&temperatureFilter);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  if (Wire.endTransmission() == 0) {
    sensorReady = true;
    Serial.printf("I2C sensor  0x%02X\n", SENSOR_I2C_ADDR);
  } else {
    ledSetState(LED_SENSOR_ERROR);
    Serial.println("[WARN] Sensor not found!");
  }

#elif SENSOR_TYPE == 1
  filterInit(&pressureFilter);
  analogSetAttenuation(ADC_11db);   // 0~3.3V range for all ADC pins
  pinMode(ADC_PRESSURE_PIN, INPUT);
  sensorReady = true;
  Serial.printf("ADC pressure sensor  GPIO%d\n", ADC_PRESSURE_PIN);

#elif SENSOR_TYPE == 2
  // ADC: use Arduino analogRead (attenuation covers 0~3.3V)
  analogSetPinAttenuation(WAV_ADC_PIN, ADC_11db);
  pinMode(WAV_ADC_PIN, INPUT);

  // BOOT button — hardware pull-up on Freenove board, active LOW
  pinMode(WAV_TRIGGER_PIN, INPUT_PULLUP);

  // Create esp_timer (NOT started yet; started on button press)
  // dispatch_method = ESP_TIMER_TASK → callback runs in task context,
  // so analogRead() is fully safe (no ISR mutex restrictions).
  esp_timer_create_args_t cfg = {
    .callback        = wavTimerCb,
    .arg             = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name            = "wavADC",
    .skip_unhandled_events = true
  };
  esp_timer_create(&cfg, &wavEspTimer);

  sensorReady = true;
  Serial.printf("Piezo WAV  ADC=GPIO%d  Trigger=GPIO%d (BOOT)\n",
    WAV_ADC_PIN, WAV_TRIGGER_PIN);
#endif

  // ── NTP time sync ────────────────────────────────────────────────────────
  syncNTP();

  // ── SD card init ─────────────────────────────────────────────────────────
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[ERROR] SD init failed! Check wiring / FAT32.");
    sdReady = false;
  } else {
    uint64_t sz = SD_MMC.cardSize() / (1024ULL*1024ULL);
    Serial.printf("SD ready  %llu MB\n", sz);
#if SENSOR_TYPE == 2
    sdReady = true;   // WAV files opened on-demand when recording starts
#else
    sdReady = openNewLogFile();
#endif
  }

  // ── Final LED ────────────────────────────────────────────────────────────
  if      (!sdReady)     ledSetState(LED_SD_ERROR);
  else if (!sensorReady) ledSetState(LED_SENSOR_ERROR);
  else                   ledSetState(LED_BOOTING);  // type 2: white = standby

  Serial.println();
#if SENSOR_TYPE == 2
  Serial.println(">>> Standby. Press BOOT button to record.");
#elif SENSOR_TYPE == 1
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
  ledUpdate();
  unsigned long now = millis();   // 공통 타임스탬프 (SENSOR_TYPE 0/1/2 모두 사용)

  // ── SENSOR_TYPE 2: WAV triggered recorder ─────────────────────────────
#if SENSOR_TYPE == 2
  if (!sensorReady) return;

  // Button detection (GPIO0 active LOW, 50 ms debounce)
  static bool          lastBtn     = HIGH;
  static unsigned long lastBtnTime = 0;
  bool btn = (bool)digitalRead(WAV_TRIGGER_PIN);

  if (btn != lastBtn && (now - lastBtnTime) > 50)
  {
    lastBtnTime = now;
    lastBtn     = btn;
    if (btn == LOW && wavState == WAV_STANDBY)   // falling edge = pressed
      wavStartRecording(sdReady);
  }

  // Flush full buffers to SD while recording
  if (wavState == WAV_RECORDING)
  {
    if (wavFlushReq && wavFile)
    {
      uint8_t bi = wavFlushBuf;
      wavFile.write((const uint8_t*)wavBuf[bi], WAV_BUF_SAMPLES * 2);
      wavDataBytes += (uint32_t)(WAV_BUF_SAMPLES * 2);
      wavFlushReq = false;
    }

    // Progress print every 500 ms
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 500)
    {
      lastPrint = now;
      float elapsed = (float)wavISRCount / (float)WAV_SAMPLE_RATE;
      Serial.printf("  %.1f / %d sec  (%.1f sec left)\n",
        elapsed, WAV_RECORD_SECONDS,
        max(0.0f, (float)WAV_RECORD_SECONDS - elapsed));
    }

    // Stop when enough samples captured
    if (wavISRCount >= (uint32_t)WAV_RECORD_SECONDS * WAV_SAMPLE_RATE)
      wavStopRecording(sdReady);
  }

  return;   // skip CSV logic below
#endif

  // ── SENSOR_TYPE 0/1: CSV sampling ─────────────────────────────────────
  static unsigned long lastSample = 0;
  if (now - lastSample < (unsigned long)SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  if (!sensorReady) return;

#if SENSOR_TYPE == 1
  int   adcRaw    = readAdcRaw();
  float adcRawAvg = filterUpdate(&pressureFilter, (float)adcRaw);
#elif SENSOR_TYPE == 0
  int32_t rawP = 0, rawT = 0;
  bool pOk = readSensor24bit(REG_PRESSURE,    &rawP);
  bool tOk = readSensor24bit(REG_TEMPERATURE, &rawT);
  if (!pOk || !tOk) {
    sensorReady = false;
    ledSetState(LED_SENSOR_DISCONNECTED);
    Serial.printf("[%lu ms] Read error -%s%s (halted)\n", now,
      pOk ? "" : " Pressure", tOk ? "" : " Temperature");
    return;
  }
  float pressure    = calcPressure(rawP);
  float temperature = calcTemperature(rawT);
  float pressureAvg = filterUpdate(&pressureFilter,    pressure);
  float tempAvg     = filterUpdate(&temperatureFilter, temperature);
#endif

  char timeStr[32];
  getTimeString(timeStr, sizeof(timeStr));

#if SENSOR_TYPE == 1
  Serial.printf("%11lu | %-24s | %9d | %11.1f\n", now, timeStr, adcRaw, adcRawAvg);
#elif SENSOR_TYPE == 0
  Serial.printf("%11lu | %-24s | %9.2f | %9.2f | %6.2f | %6.2f\n",
    now, timeStr, pressure, pressureAvg, temperature, tempAvg);
#endif

  if (sdReady && logFile)
  {
    if (MAX_ROWS_PER_FILE > 0 && rowCount >= (uint32_t)MAX_ROWS_PER_FILE) {
      logFile.flush(); logFile.close();
      Serial.printf("[Rotate] %s (%u rows)\n", logFileName, rowCount);
      if (!openNewLogFile()) { sdReady = false; return; }
    }
#if SENSOR_TYPE == 1
    logFile.printf("%lu,%s,%d,%.1f\n", now, timeStr, adcRaw, adcRawAvg);
#elif SENSOR_TYPE == 0
    logFile.printf("%lu,%s,%.2f,%.2f,%.2f,%.2f\n",
      now, timeStr, pressure, pressureAvg, temperature, tempAvg);
#endif
    rowCount++;
    if (++flushCnt >= (uint32_t)BUF_FLUSH_COUNT) { logFile.flush(); flushCnt = 0; }
  }
}
