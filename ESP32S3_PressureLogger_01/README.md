# ESP32S3 Pressure Sensor Data Logger

ESP32-S3를 이용한 압력 센서 데이터 MicroSD 로거.
I2C 디지털 센서(WNK Series) 또는 아날로그 ADC 센서 중 하나를 선택하여 사용합니다.

---

## 하드웨어

- **MCU**: Freenove ESP32-S3 WROOM Camera Module
- **센서 (디지털)**: WNK Series (WNK21 / WD19 / WNK811) — I2C, 3.3V
- **센서 (아날로그)**: 아날로그 압력 센서 — 출력 0.5V~4.5V (회로 내 3.3V 이하로 사용)
- **저장 장치**: MicroSD — SDMMC 1-bit 모드

---

## 핀 배선

| 기능 | ESP32-S3 GPIO | 연결 대상 |
|------|--------------|----------|
| I2C SDA | GPIO 21 | 디지털 센서 SDA |
| I2C SCL | GPIO 20 | 디지털 센서 SCL (GPIO 22 없음) |
| ADC IN | GPIO 1 | 아날로그 센서 VOUT (ADC1 CH0) |
| SD CMD | GPIO 38 | MicroSD CMD |
| SD D0 | GPIO 40 | MicroSD DATA |
| SD CLK | GPIO 39 | MicroSD CLK |
| RGB LED | GPIO 48 | WS2812B (내장) |
| 3.3V | — | 센서 VCC, SD VCC |
| GND | — | 센서 GND, SD GND |

> ADC 입력은 WiFi와 충돌을 피하기 위해 ADC1 (GPIO 1~10) 을 사용합니다.

---

## 주요 설정 (`ESP32S3_PressureLogger_01.ino` 상단)

### 센서 선택

```cpp
// 0 = I2C 디지털 센서 (WNK Series)
// 1 = ADC 아날로그 센서
#define SENSOR_TYPE  0
```

### NTP 시간 동기화

```cpp
#define USE_NTP  1   // 1=활성화, 0=millis() 기반 상대 시간

const char* WIFI_SSID     = "your_ssid";
const char* WIFI_PASSWORD = "your_password";
```

> NTP 동기화 후 WiFi는 자동으로 꺼집니다 (절전).

### 샘플링 속도

```cpp
#define SAMPLE_INTERVAL_MS  10   // 10ms = 100Hz
```

| 설정값 | 샘플링 속도 | 비고 |
|--------|-----------|------|
| 100 ms | 10 Hz | 즉시 쓰기, 안정적 |
| 10 ms | 100 Hz | BUF_FLUSH_COUNT 활용 권장 |
| 2 ms | 500 Hz | PSRAM 버퍼 필요 |

### SD 쓰기 버퍼

```cpp
#define BUF_FLUSH_COUNT  10   // N 샘플마다 flush
```

### 파일 자동 교체

```cpp
#define MAX_ROWS_PER_FILE  360000   // 행 수 초과 시 다음 파일로 자동 전환 (0=무제한)
```

---

## CSV 출력 형식

### SENSOR_TYPE 0 (I2C 디지털 센서)

```
timestamp_ms, datetime, pressure_mbar, pressure_avg_mbar, temperature_c, temperature_avg_c
```

### SENSOR_TYPE 1 (ADC 아날로그 센서)

```
timestamp_ms, datetime, adc_pressure_mbar, adc_pressure_avg_mbar
```

- `datetime`: NTP 동기화 시 `YYYY-MM-DD HH:MM:SS.mmm` (KST, UTC+9), 미동기화 시 `HH:MM:SS.mmm`
- 파일명: `LOG0001.csv`, `LOG0002.csv`, ... (자동 증가)
- 이동 평균 필터 크기: 10샘플

---

## I2C 센서 파라미터

| 항목 | 값 |
|------|---|
| I2C 주소 | `0x6D` |
| 압력 레지스터 | `0x06` |
| 온도 레지스터 | `0x09` |
| 압력 풀스케일 | 2000 kPa |
| 영점 오프셋 | 70 mbar |
| I2C 클럭 | 400 kHz (Fast mode) |

---

## RGB LED 상태 표시

내장 WS2812B LED (GPIO 48) 로 디바이스 동작 상태를 색상과 패턴으로 표시합니다.

| 상태 | 색상 | 패턴 | 설명 |
|------|------|------|------|
| 초기화 중 | 흰색 | 점등 | `setup()` 실행 중 |
| WiFi/NTP 연결 중 | 파란색 | 300ms 깜빡임 | WiFi 연결 및 NTP 동기화 대기 |
| 정상 로깅 | 초록색 | 1s 깜빡임 | SD 카드 저장 정상 동작 중 |
| SD 카드 오류 | 빨간색 | 150ms 빠른 깜빡임 | SD 초기화 실패 또는 파일 생성 오류 |
| 센서 미감지 | 마젠타 | 150ms 빠른 깜빡임 | 부팅 시 I2C 센서 응답 없음 |
| 센서 연결 끊김 | 빨간색 | 점등 | 로깅 중 센서 읽기 오류 발생 |

> SD 오류와 센서 오류가 동시에 발생한 경우 SD 오류(빨간색 깜빡임)가 우선 표시됩니다.

---

## 라이브러리

| 라이브러리 | 용도 |
|-----------|------|
| SD_MMC (ESP32 내장) | MicroSD 읽기/쓰기 |
| Wire (Arduino 내장) | I2C 통신 (SENSOR_TYPE=0 전용) |
| WiFi (ESP32 내장) | NTP 동기화 (USE_NTP=1 시) |
| Adafruit NeoPixel | RGB LED 제어 (WS2812B) |

> **Adafruit NeoPixel** 설치: Arduino IDE → `Sketch → Include Library → Manage Libraries` → "Adafruit NeoPixel" 검색 후 설치
