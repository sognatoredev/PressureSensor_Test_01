#pragma once

// ==================== User Settings ====================
#define USE_NTP            1

// Select sensor type:
//   0 = I2C digital sensor  (WNK Series)
//   1 = ADC analog pressure sensor
//   2 = Piezo vibration WAV recorder (button-triggered)
#define SENSOR_TYPE        2

#define WIFI_SSID     "KT_GiGA_9748"
#define WIFI_PASSWORD "9cf0bkd529"

// Sampling interval (ms) — SENSOR_TYPE 0/1 only
#define SAMPLE_INTERVAL_MS   10
#define BUF_FLUSH_COUNT      10
#define MAX_ROWS_PER_FILE    360000

// ==================== I2C / WNK Sensor ====================
#define I2C_SDA_PIN        21
#define I2C_SCL_PIN        20
#define I2C_CLOCK_FREQ     400000
#define SENSOR_I2C_ADDR    0x6D
#define REG_PRESSURE       0x06
#define REG_TEMPERATURE    0x09
#define PRESSURE_RANGE     2000.0f
#define KPA_TO_MBAR        10.0f
#define PRESSURE_OFFSET    70.0f

// ==================== ADC Pressure Sensor (SENSOR_TYPE 1) ====================
#define ADC_PRESSURE_PIN   4   // GPIO4 (ADC1 CH3)

// ==================== Piezo WAV Recorder (SENSOR_TYPE 2) ====================
#define WAV_ADC_PIN         1  // GPIO1 = ADC1 CH0 (MAX4466 output)
#define WAV_TRIGGER_PIN     0  // GPIO0 = BOOT button (active LOW)
#define WAV_SAMPLE_RATE     8000
#define WAV_RECORD_SECONDS  3
#define WAV_BUF_SAMPLES     4096

// ==================== SD Card Pins (SDMMC 1-bit) ====================
#define SD_CMD_PIN        38
#define SD_D0_PIN         40
#define SD_CLK_PIN        39

// ==================== Moving Average Filter ====================
#define FILTER_SIZE        10

// ==================== RGB LED (WS2812B - GPIO48) ====================
#define RGB_LED_PIN       48
#define RGB_LED_COUNT      1
