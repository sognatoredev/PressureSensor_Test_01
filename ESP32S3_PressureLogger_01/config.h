#pragma once

// ==================== User Settings ====================
#define USE_NTP            1

// Select sensor type:
//   0 = I2C digital sensor  (WNK Series)
//   1 = ADC analog pressure sensor
//   2 = Piezo vibration WAV recorder (ADC, button-triggered)
//   3 = Piezo vibration WAV recorder (NAU88C10YG codec via I2S, button-triggered)
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
// analogRead() on ESP32-S3 takes ~30-100 µs → safe upper limit ≈ 10 kHz
// 8000 Hz → timer period = 125 µs, well within analogRead margin
#define WAV_SAMPLE_RATE     8000
#define WAV_RECORD_SECONDS  3
#define WAV_BUF_SAMPLES     1024

// ==================== NAU88C10YG Codec WAV Recorder (SENSOR_TYPE 3) ====================
// I2C control interface (shares Wire with SENSOR_TYPE 0)
#define NAU88_I2C_ADDR       0x1A  // 7-bit I2C address

// I2S audio interface (ESP32-S3 is I2S master; codec is slave)
// 핀 선정 기준 (후보: 1, 2, 14, 41, 42, 45, 46, 47)
//   MCLK/BCLK/LRCLK : 출력 전용 일반 GPIO 사용 (strapping 핀 45 제외)
//   DIN             : GPIO 46 — strapping 핀이나 부트 후 입력으로 안전
//   GPIO 45         : VDD_SPI strapping → 클록 출력 시 부트 전압 오인식 위험, 미사용
#define NAU88_I2S_PORT       0     // I2S port number (I2S_NUM_0)
#define NAU88_I2S_MCLK_PIN   14    // Master Clock output → NAU88 MCLK  (256 × fs = 2.048 MHz)
#define NAU88_I2S_BCLK_PIN   2     // Bit Clock output   → NAU88 BCLK
#define NAU88_I2S_LRCLK_PIN  1     // LR Clock output    → NAU88 LRC / WS
#define NAU88_I2S_DIN_PIN    46    // Data input         ← NAU88 ADCOUT  (GPIO46: 입력 전용 strapping)

#define NAU88_TRIGGER_PIN    0     // BOOT button (active LOW) — same as SENSOR_TYPE 2
// #define NAU88_SAMPLE_RATE    44100 // 44.1 kHz (MCLK = 256 × 44100 = 11.2896 MHz, APLL)
#define NAU88_SAMPLE_RATE    8000 // 44.1 kHz (MCLK = 256 × 44100 = 11.2896 MHz, APLL)
#define NAU88_RECORD_SECONDS 3     // recording length per file
#define NAU88_BUF_SAMPLES    4096  // double-buffer size (samples per half)

// ==================== LTE (SIMCOM7000) ====================
// LTE_PASSTHROUGH=1: AT 커맨드 터미널 모드 (Serial Monitor ↔ SIM7000 직결)
//   Serial Monitor 설정: 115200 baud, "Both NL & CR" 또는 "Newline"
//   이 모드에서는 lteInit() 등 자동 시퀀스를 건너뛰고 패스스루만 동작합니다.
#define LTE_PASSTHROUGH     0
// 사용 중인 핀: 0,1,2,14,20,21,38,39,40,46,48 → 17/18 미사용 확인
#define USE_LTE             1
#define LTE_TX_PIN          17    // ESP32 TX → SIMCOM7000 RXD
#define LTE_RX_PIN          18    // ESP32 RX ← SIMCOM7000 TXD
#define LTE_PWRKEY_PIN      4     // ESP32 GPIO4 → SIMCOM7000 PWRKEY (active LOW pulse)
#define LTE_BAUD            115200
#define LTE_APN             "em"
#define LTE_SERVER_HOST     "dev.neverlosewater.com"
#define LTE_SERVER_PORT     49152
#define LTE_SERIAL_NO       "SM2-V3A-6000"
#define LTE_GET_PATH        "/m2/device_setting?serial_no=" LTE_SERIAL_NO

// ==================== SD Card Pins (SDMMC 1-bit) ====================
#define SD_CMD_PIN        38
#define SD_D0_PIN         40
#define SD_CLK_PIN        39

// ==================== Moving Average Filter ====================
#define FILTER_SIZE        10

// ==================== RGB LED (WS2812B - GPIO48) ====================
#define RGB_LED_PIN       48
#define RGB_LED_COUNT      1
