# ESP32S3 Pressure / Vibration Sensor Data Logger

ESP32-S3를 이용한 압력·진동 센서 데이터 MicroSD 로거.
`config.h`의 `SENSOR_TYPE`으로 동작 모드를 선택합니다.

---

## 센서 타입 선택

```cpp
// config.h
#define SENSOR_TYPE  3
```

| 값 | 모드 | 설명 |
|----|------|------|
| 0 | I2C 디지털 압력 센서 | WNK Series — CSV 기록 |
| 1 | ADC 아날로그 압력 센서 | GPIO1 ADC — CSV 기록 |
| 2 | 압전 WAV 녹음 (ADC) | MAX4466 + esp_timer, 44.1kHz, 버튼 트리거 |
| 3 | 압전 WAV 녹음 (코덱 I2S) | NAU88C10YG, 8kHz, 버튼 트리거 |

---

## 하드웨어

- **MCU**: Freenove ESP32-S3 WROOM Camera Module
- **센서 (디지털)**: WNK Series (WNK21 / WD19 / WNK811) — I2C, 3.3V
- **센서 (아날로그)**: 아날로그 압력 센서 — 0.5V~4.5V (3.3V 이하)
- **센서 (압전)**: 압전 소자 → MAX4466(SENSOR_TYPE 2) 또는 NAU88C10YG(SENSOR_TYPE 3)
- **저장 장치**: MicroSD — SDMMC 1-bit 모드

---

## 핀 배선

### 공통

| 기능 | GPIO | 연결 대상 |
|------|------|----------|
| SD CMD | 38 | MicroSD CMD |
| SD D0 | 40 | MicroSD DATA |
| SD CLK | 39 | MicroSD CLK |
| RGB LED | 48 | WS2812B (내장) |
| BOOT 버튼 | 0 | 녹음 트리거 (SENSOR_TYPE 2/3) |

### SENSOR_TYPE 0/3 — I2C

| 기능 | GPIO | 연결 대상 |
|------|------|----------|
| I2C SDA | 21 | 센서 SDA / NAU88 SDA |
| I2C SCL | 20 | 센서 SCL / NAU88 SCL |

### SENSOR_TYPE 1/2 — ADC

| 기능 | GPIO | 연결 대상 |
|------|------|----------|
| ADC IN | 1 | 센서 VOUT / MAX4466 OUT (ADC1 CH0) |

### SENSOR_TYPE 3 — I2S (NAU88C10YG)

| 기능 | GPIO | 방향 | 설명 |
|------|------|------|------|
| MCLK | 14 | → 코덱 | Master Clock (256 × fs) |
| BCLK | 2  | → 코덱 | Bit Clock |
| LRCLK | 1 | → 코덱 | LR / Word Select |
| DIN | 46  | ← 코덱 | ADCOUT (입력) |

> ADC 입력은 WiFi 충돌 방지를 위해 ADC1 (GPIO 1~10)을 사용합니다.

---

## 주요 설정 (`config.h`)

### SENSOR_TYPE 0/1 — 압력 센서

```cpp
#define SAMPLE_INTERVAL_MS  10    // 샘플링 주기 (ms)
#define BUF_FLUSH_COUNT     10    // N 샘플마다 SD flush
#define MAX_ROWS_PER_FILE   360000 // 행 초과 시 다음 파일로 전환
```

### SENSOR_TYPE 2 — ADC WAV 녹음

```cpp
#define WAV_SAMPLE_RATE     44100  // 샘플레이트 (ADC 타이머, 실제 ~45.5kHz)
#define WAV_RECORD_SECONDS  3      // 녹음 길이
#define WAV_BUF_SAMPLES     4096   // 더블 버퍼 크기
```

> esp_timer 정수 µs 한계로 정확히 44100Hz 구현 불가. 22µs 주기 사용 시 실제 ~45,454Hz.

### SENSOR_TYPE 3 — 코덱 I2S WAV 녹음

```cpp
#define NAU88_SAMPLE_RATE    8000  // 샘플레이트 (Hz)
#define NAU88_RECORD_SECONDS 3     // 녹음 길이
#define NAU88_BUF_SAMPLES    4096  // 더블 버퍼 크기
```

### NTP

```cpp
#define USE_NTP       1
#define WIFI_SSID     "your_ssid"
#define WIFI_PASSWORD "your_password"
```

---

## SENSOR_TYPE 2 — ADC WAV 녹음 동작

### 신호 처리 흐름

```
GPIO1 ADC
  └→ Biquad HPF 200Hz + Biquad LPF 2000Hz + TPDF 디더
       └→ 더블 버퍼 → SD (VIB0001.wav …)
            └→ normalizeWavFile() : 피크 탐색 → -1dBFS 게인 → 재기록
                 └→ analyzeFFT()  : Hamming 윈도우 → 제로패딩 → FFT → 피크 주파수
```

### 소프트웨어 필터 (2차 Butterworth @ 44100Hz)

| 필터 | 차단 주파수 | 역할 |
|------|-----------|------|
| Biquad HPF | 200 Hz | DC 및 저주파 진동 제거 |
| Biquad LPF | 2000 Hz | 고주파 화이트노이즈 제거 |
| TPDF 디더링 | — | int16 재양자화 시 왜곡 분산 |

### FFT 설정

| 항목 | 값 |
|------|----|
| FFT 크기 | 262144 (2^18, PSRAM ~2MB) |
| 캡처 샘플 수 | 44100 × 3 = 132300 |
| 주파수 분해능 | 44100 / 262144 ≈ 0.168 Hz/bin |
| 전기 노이즈 제거 | 59~61Hz 빈 제로 (전원 험 제외) |

### 파일

- 파일명: `VIB0001.wav`, `VIB0002.wav`, …
- 포맷: 16-bit PCM, Mono, 44100Hz
- 3초 녹음 시 약 264KB

---

## SENSOR_TYPE 3 — 코덱 I2S WAV 녹음 동작

### 신호 처리 흐름

```
압전소자 → NAU88C10YG ADC → I2S → ESP32-S3
  ├─ [원본] → rawBuf (PSRAM) ──────────────────→ analyzeFFT()  ← 원 데이터 FFT
  └─ [필터] IIR HPF 10Hz + IIR LPF 5kHz + IIR LPF 2kHz
               └→ 더블 버퍼 → SD (COD0001.wav …)
                    └→ normalizeWavFile() : -1dBFS 정규화 → 재기록
```

### NAU88C10YG 코덱 설정

| 항목 | 설정값 | 설명 |
|------|--------|------|
| PGA Gain | 0dB (R45=0x010) | 클리핑 방지, 필요 시 최대 +35.25dB |
| ADC Boost | OFF (R47=0x000) | 필요 시 +20dB 활성 가능 |
| ADC Volume | 최대 (R15=0x1FF) | 디지털 볼륨 최대 |
| 내장 HPF | 활성 (R14=0x100) | 코덱 하드웨어 DC 제거 |
| I2S 포맷 | I2S Standard, 16bit | R04=0x010 |
| MCLK | 256 × fs | APLL 사용 시 정확한 44.1kHz 계열 생성 가능 |

### 소프트웨어 필터 (녹음 경로 전용)

| 필터 | 차단 주파수 | 계수 |
|------|-----------|------|
| IIR HPF | ~10 Hz | α = 0.9986 |
| IIR LPF | ~5000 Hz | α = 0.5412 |
| IIR LPF | ~2000 Hz | α = 0.222 |

> 코덱 내장 HPF가 활성화되어 있으므로 소프트웨어 DC 제거는 비활성(주석 처리).

### FFT 설정

| 항목 | 값 |
|------|----|
| FFT 크기 | 32768 |
| 입력 데이터 | rawBuf (필터 미적용 코덱 원 데이터) |
| 전기 노이즈 제거 | 59~61Hz 빈 제로 (전원 험 제외) |
| 윈도우 | Hamming |

### 파일

- 파일명: `COD0001.wav`, `COD0002.wav`, …
- 포맷: 16-bit PCM, Mono, 8000Hz
- 3초 녹음 시 약 48KB

---

## CSV 출력 형식 (SENSOR_TYPE 0/1)

### SENSOR_TYPE 0 (I2C 디지털 센서)

```
timestamp_ms, datetime, pressure_mbar, pressure_avg_mbar, temperature_c, temperature_avg_c
```

### SENSOR_TYPE 1 (ADC 아날로그 센서)

```
timestamp_ms, datetime, adc_pressure_mbar, adc_pressure_avg_mbar
```

- `datetime`: NTP 동기화 시 `YYYY-MM-DD HH:MM:SS.mmm` (KST, UTC+9)
- 파일명: `LOG0001.csv`, `LOG0002.csv`, …

---

## RGB LED 상태 표시

| 상태 | 색상 | 패턴 | 설명 |
|------|------|------|------|
| 초기화 중 | 흰색 | 점등 | `setup()` 실행 중 |
| WiFi/NTP 연결 중 | 파란색 | 300ms 깜빡임 | NTP 동기화 대기 |
| 정상 로깅 | 초록색 | 1s 깜빡임 | SD 저장 정상 |
| WAV 녹음 중 | 초록색 | 점등 | 버튼 트리거 후 녹음 중 |
| SD 카드 오류 | 빨간색 | 150ms 깜빡임 | SD 초기화/파일 오류 |
| 센서 미감지 | 마젠타 | 150ms 깜빡임 | I2C 응답 없음 |
| 센서 연결 끊김 | 빨간색 | 점등 | 로깅 중 읽기 오류 |

---

## 라이브러리

| 라이브러리 | 용도 |
|-----------|------|
| SD_MMC (ESP32 내장) | MicroSD 읽기/쓰기 |
| Wire (Arduino 내장) | I2C (SENSOR_TYPE 0/3) |
| driver/i2s (ESP-IDF) | I2S (SENSOR_TYPE 3) |
| WiFi (ESP32 내장) | NTP 동기화 |
| Adafruit NeoPixel | RGB LED (WS2812B) |
| arduinoFFT | FFT 주파수 분석 (SENSOR_TYPE 2/3) |

> Arduino IDE → `Sketch → Include Library → Manage Libraries`에서 **Adafruit NeoPixel**, **arduinoFFT** 설치 필요.

---

## 시리얼 출력 예시 (SENSOR_TYPE 3)

```
[FFT] Source   : RAW (codec PCM, no SW filter)
[FFT] Signal   : 24000 samples (3.000 sec)
[FFT] Zero-pad : 24000 → 32768
[FFT] Freq Resolution : 0.244 Hz/bin
[FFT] Peak Freq : 312.50 Hz
─────────────────────────────────
```
