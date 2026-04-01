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
 *  I2C SDA       GPIO 21         → Digital Sensor SDA / NAU88 SDA (Type 0/3)
 *  I2C SCL       GPIO 20         → Digital Sensor SCL / NAU88 SCL (Type 0/3)
 *  ADC IN        GPIO 4          → Analog Pressure Sensor VOUT  (ADC1 CH3, Type 1)
 *  ADC IN        GPIO 1          → MAX4466 OUT (Piezo preamp)   (ADC1 CH0, Type 2)
 *  I2S MCLK      GPIO 8          → NAU88 MCLK  (256×fs, Type 3)
 *  I2S BCLK      GPIO 5          → NAU88 BCLK  (Type 3)
 *  I2S LRCLK     GPIO 6          → NAU88 LRC   (Type 3)
 *  I2S DIN       GPIO 7          ← NAU88 ADCOUT (Type 3)
 *  TRIGGER       GPIO 0          → BOOT button (built-in, active LOW, Type 2/3)
 *  SD CMD        GPIO 38         → MicroSD CMD
 *  SD D0         GPIO 40         → MicroSD DATA
 *  SD CLK        GPIO 39         → MicroSD CLK
 *  3.3V                          → Sensor VCC, SD VCC
 *  GND                           → Sensor GND, SD GND
 * ──────────────────────────────────────────────────────────────
 *
 * Sensor selection (SENSOR_TYPE in config.h):
 *   0 = I2C digital sensor only  (WNK)
 *   1 = ADC analog pressure sensor only
 *   2 = Piezo vibration WAV recorder (ADC, triggered by BOOT button)
 *   3 = Piezo vibration WAV recorder (NAU88C10YG codec via I2S, triggered by BOOT button)
 *
 * Required libraries:
 *   - SD_MMC, Wire, WiFi (ESP32 built-in)
 *   - Adafruit NeoPixel
 *   - esp_timer.h (ESP-IDF built-in) [SENSOR_TYPE=2]
 *   - driver/i2s.h  (ESP-IDF built-in) [SENSOR_TYPE=3]
 */

#include "config.h"
#include "led_indicator.h"
#include "filter.h"
#include "time_manager.h"
#include "sd_logger.h"

#if SENSOR_TYPE == 0
  #include <Wire.h>
  #include "sensor_i2c.h"
#elif SENSOR_TYPE == 1
  #include "sensor_adc.h"
#elif SENSOR_TYPE == 2
  #include "wav_recorder.h"
#elif SENSOR_TYPE == 3
  #include "wav_recorder_codec.h"
#endif

static bool sensorReady = false;

#if SENSOR_TYPE == 0
static MovingAvgFilter_t pressureFilter;
static MovingAvgFilter_t temperatureFilter;
#elif SENSOR_TYPE == 1
static MovingAvgFilter_t pressureFilter;
#endif  // SENSOR_TYPE 2/3: 필터 불필요 (코덱/ADC 녹음기가 자체 처리)

// ==================== Setup ====================
void setup()
{
  Serial.begin(115200);
  ledBegin();
  ledSetState(LED_BOOTING);
  ledUpdate();
  delay(500);

  Serial.println("=========================================");
#if SENSOR_TYPE == 2
  Serial.println("  Piezo Vibration WAV Recorder (ADC)");
  Serial.printf ("  %d Hz · 16-bit PCM Mono · %d sec/file\n",
    WAV_SAMPLE_RATE, WAV_RECORD_SECONDS);
  Serial.println("  Press BOOT button (GPIO0) to record");
#elif SENSOR_TYPE == 3
  Serial.println("  Piezo Vibration WAV Recorder (NAU88C10YG Codec / I2S)");
  Serial.printf ("  %d Hz · 16-bit PCM Mono · %d sec/file\n",
    NAU88_SAMPLE_RATE, NAU88_RECORD_SECONDS);
  Serial.println("  Press BOOT button (GPIO0) to record");
#else
  Serial.println("  WNK/ADC Pressure Sensor Data Logger");
  Serial.printf ("  %d ms (%0.f Hz) · flush every %d samples\n",
    SAMPLE_INTERVAL_MS, 1000.0f / SAMPLE_INTERVAL_MS, BUF_FLUSH_COUNT);
#endif
  Serial.println("=========================================");

  // ── Sensor init ──────────────────────────────────────────────────────────
#if SENSOR_TYPE == 0
  filterInit(&pressureFilter);
  filterInit(&temperatureFilter);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  
  if (Wire.endTransmission() == 0)
  {
    sensorReady = true;
    Serial.printf("I2C sensor  0x%02X\n", SENSOR_I2C_ADDR);
  }
  else
  {
    ledSetState(LED_SENSOR_ERROR);
    Serial.println("[WARN] Sensor not found!");
  }

#elif SENSOR_TYPE == 1
  filterInit(&pressureFilter);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PRESSURE_PIN, INPUT);
  sensorReady = true;
  Serial.printf("ADC pressure sensor  GPIO%d\n", ADC_PRESSURE_PIN);

#elif SENSOR_TYPE == 2
  wavRecorderInit();
  sensorReady = true;
#elif SENSOR_TYPE == 3
  codecRecorderInit();
  sensorReady = true;
#endif

  // ── NTP time sync ─────────────────────────────────────────────────────────
  syncNTP();

  // ── SD card init ──────────────────────────────────────────────────────────
  if (sdInit())
  {
    #if SENSOR_TYPE == 2 || SENSOR_TYPE == 3
        // WAV 파일은 녹음 시작 시 on-demand 생성
    #else
        if (!openNewLogFile()) sdReady = false;
    #endif
  }

  // ── Final LED ─────────────────────────────────────────────────────────────
  if      (!sdReady)     ledSetState(LED_SD_ERROR);
  else if (!sensorReady) ledSetState(LED_SENSOR_ERROR);
  else                   ledSetState(LED_BOOTING);

  Serial.println();
#if SENSOR_TYPE == 2 || SENSOR_TYPE == 3
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
  unsigned long now = millis();

  // ── SENSOR_TYPE 2/3: WAV triggered recorder ───────────────────────────
#if SENSOR_TYPE == 2 || SENSOR_TYPE == 3
  if (!sensorReady)
  {
    return;
  }

#if SENSOR_TYPE == 2
  wavLoop(sdReady, now);
#else
  codecLoop(sdReady, now);
#endif
  return;
#endif

  // ── SENSOR_TYPE 0/1: CSV sampling ─────────────────────────────────────
  static unsigned long lastSample = 0;
  if (now - lastSample < (unsigned long)SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  if (!sensorReady) return;

  char timeStr[32];
  getTimeString(timeStr, sizeof(timeStr));

#if SENSOR_TYPE == 1
  int   adcRaw    = readAdcRaw();
  float adcRawAvg = filterUpdate(&pressureFilter, (float)adcRaw);

  Serial.printf("%11lu | %-24s | %9d | %11.1f\n", now, timeStr, adcRaw, adcRawAvg);
  sdWriteADC(now, timeStr, adcRaw, adcRawAvg);

#elif SENSOR_TYPE == 0
  int32_t rawP = 0, rawT = 0;
  bool pOk = readSensor24bit(REG_PRESSURE,    &rawP);
  bool tOk = readSensor24bit(REG_TEMPERATURE, &rawT);
  
  if (!pOk || !tOk)
  {
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

  Serial.printf("%11lu | %-24s | %9.2f | %9.2f | %6.2f | %6.2f\n",
    now, timeStr, pressure, pressureAvg, temperature, tempAvg);
  sdWriteI2C(now, timeStr, pressure, pressureAvg, temperature, tempAvg);
#endif
}
