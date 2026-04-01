/**
 * wav_recorder_codec.cpp  —  SENSOR_TYPE 3  (기본 기능 확인용)
 *
 * 신호 경로:
 *   압전소자 → NAU88C10YG(I2S slave) ──ADCOUT──→ ESP32-S3(I2S master RX) → SD WAV
 *
 * I2S 핀 (ESP32-S3 master):
 *   MCLK = GPIO 8  → NAU88 MCLK
 *   BCLK = GPIO 5  → NAU88 BCLK
 *   LRCLK= GPIO 6  → NAU88 FS/WS
 *   DIN  = GPIO 7  ← NAU88 ADCOUT
 *
 * I2C 핀 (코덱 레지스터):
 *   SDA = GPIO 21, SCL = GPIO 20
 */

#include "wav_recorder_codec.h"

#if SENSOR_TYPE == 3

#include <Arduino.h>
#include <Wire.h>
#include <SD_MMC.h>
#include <driver/i2s.h>
#include "led_indicator.h"

// ── 상태 머신 ──────────────────────────────────────────────────────────────
enum CodecWavState { CODEC_STANDBY, CODEC_RECORDING, CODEC_SAVING };
static CodecWavState codecState = CODEC_STANDBY;

// ── 단일 DMA 읽기 버퍼 (스택 할당) ───────────────────────────────────────
#define RX_BUF_SAMPLES 64
static int16_t rxBuf[RX_BUF_SAMPLES];

// ── SD 기록용 더블 버퍼 (SRAM, 4096 × 2 × 2 = 16 KB) ────────────────────
static int16_t  codecBuf[2][NAU88_BUF_SAMPLES];
static uint8_t  codecFillBuf  = 0;
static uint32_t codecFillPos  = 0;
static uint8_t  codecFlushBuf = 0;
static bool     codecFlushReq = false;

// ── 녹음 통계 ─────────────────────────────────────────────────────────────
static uint32_t codecSampleCount = 0;
static int16_t  codecPcmMin     = 0;
static int16_t  codecPcmMax     = 0;
static uint32_t codecNonZero    = 0;   // 0이 아닌 샘플 수

// ── 파일 관리 ──────────────────────────────────────────────────────────────
static File     codecFile;
static char     codecFileName[32];
static uint32_t codecDataBytes = 0;

// ─────────────────────────────────────────────────────────────────────────
// NAU88C10YG I2C 레지스터 쓰기
//   Byte1 = (reg << 1) | (val >> 8)
//   Byte2 = val & 0xFF
// ─────────────────────────────────────────────────────────────────────────
static void nau88WriteReg(uint8_t reg, uint16_t val)
{
  Wire.beginTransmission(NAU88_I2C_ADDR);
  Wire.write((uint8_t)((reg << 1) | ((val >> 8) & 0x01)));
  Wire.write((uint8_t)(val & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err != 0)
    Serial.printf("[CODEC] I2C err reg=0x%02X err=%d\n", reg, err);
}

// ─────────────────────────────────────────────────────────────────────────
// NAU88C10YG 초기화
//   레지스터 비트 맵 (WM8510 / NAU8810 호환):
//   R1  [8:7]=VMIDSEL, [6]=BUFIOEN, [4]=BIASEN
//   R2  [3]=ADCENL
//   R3  [1]=LMIXEN
//   R4  [7]=MS(0=slave), [4:3]=WL(00=16bit), [2:1]=FMT(10=I2S)
//   R6  [8:6]=MCLKDIV(000=÷1), [2]=CLKSEL(0=MCLK)
//   R14 [8]=HPFEN
//   R15 [8]=LADCVU, [7:0]=LADCVOL
//   R44 [7]=LMICP(MICP→PGA), [6]=LPGAUPDATE, [5:0]=LPGAVOL
//   R47 [0]=LMIXEN
//   R49 [8]=PGABOOSTL(+20dB)
// ─────────────────────────────────────────────────────────────────────────
static bool nau88Init()
{
  Wire.beginTransmission(NAU88_I2C_ADDR);
  if (Wire.endTransmission() != 0)
  {
    Serial.printf("[CODEC] NAU88C10YG not found at 0x%02X\n", NAU88_I2C_ADDR);
    return false;
  }
  Serial.printf("[CODEC] NAU88C10YG found at 0x%02X\n", NAU88_I2C_ADDR);

  // 소프트웨어 리셋
  nau88WriteReg(0x00, 0x000);
  delay(50);

  // Power1: VMIDSEL=01(75kΩ), BUFIOEN=1, BIASEN=1  → 0x0D0
  nau88WriteReg(0x01, 0x0D0);
  delay(300);  // VMID 커패시터 충전 대기

  // Power2: ADCENL=1  → 0x008
  nau88WriteReg(0x02, 0x008);

  // Power3: LMIXEN=1  → 0x002
  nau88WriteReg(0x03, 0x002);

  // Audio Interface: I2S, 16-bit, slave  → 0x004
  //   BCLK = 8000 × 16 × 2 = 256 kHz (ESP32 16-bit 모드와 일치)
  nau88WriteReg(0x04, 0x004);

  // Clock: MCLKDIV=÷1, CLKSEL=MCLK  → 0x000
  //   SYSCLK = MCLK = 2.048 MHz  →  fs = 2.048 MHz / 256 = 8 kHz
  nau88WriteReg(0x06, 0x000);

  // ADC Control: HPFEN=1 (하드웨어 HPF, DC 제거)  → 0x100
  nau88WriteReg(0x0E, 0x100);

  // Left ADC Volume: 0 dB + 업데이트  → 0x1FF
  nau88WriteReg(0x0F, 0x1FF);

  // Left PGA: LMICP=1(MICP→PGA) + LPGAUPDATE=1 + LPGAVOL=16(0dB)  → 0x0D0
  //   gain_dB = -12 + LPGAVOL × 0.75  (16 → 0 dB)
  nau88WriteReg(0x2C, 0x0D0);

  // Left Input Mixer: LMIXEN=1  → 0x001
  nau88WriteReg(0x2F, 0x001);

  // Left ADC Boost: PGABOOSTL=1 (+20 dB)  → 0x100
  nau88WriteReg(0x31, 0x100);

  Serial.println("[CODEC] NAU88C10YG init OK");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────
// I2S 마스터 RX 초기화
//   use_apll=false: APB(80 MHz) 분주 사용
//     → APLL 잠금 실패로 MCLK 무출력되는 문제 회피
// ─────────────────────────────────────────────────────────────────────────
static bool codecI2SInit()
{
  i2s_config_t cfg = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate          = NAU88_SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = 256,
    .use_apll             = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0
  };

  i2s_pin_config_t pins = {
    .mck_io_num   = NAU88_I2S_MCLK_PIN,
    .bck_io_num   = NAU88_I2S_BCLK_PIN,
    .ws_io_num    = NAU88_I2S_LRCLK_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = NAU88_I2S_DIN_PIN
  };

  esp_err_t err = i2s_driver_install((i2s_port_t)NAU88_I2S_PORT, &cfg, 0, NULL);
  if (err != ESP_OK)
  {
    Serial.printf("[CODEC] I2S install failed: 0x%X\n", err);
    return false;
  }

  err = i2s_set_pin((i2s_port_t)NAU88_I2S_PORT, &pins);
  if (err != ESP_OK)
  {
    Serial.printf("[CODEC] I2S pin failed: 0x%X\n", err);
    return false;
  }

  Serial.printf("[CODEC] I2S OK  SR=%dHz  BCLK=%dHz  MCLK_GPIO=%d\n",
    NAU88_SAMPLE_RATE,
    NAU88_SAMPLE_RATE * 16 * 2,
    NAU88_I2S_MCLK_PIN);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────
// WAV 헤더 쓰기 (44바이트)
// ─────────────────────────────────────────────────────────────────────────
static void writeWavHeader(File &f, uint32_t dataBytes)
{
  const uint16_t ch       = 1;
  const uint16_t bits     = 16;
  const uint32_t sr       = NAU88_SAMPLE_RATE;
  const uint32_t byteRate = sr * ch * bits / 8;
  const uint16_t align    = ch * bits / 8;
  const uint32_t fmtSize  = 16;
  const uint16_t fmt      = 1;  // PCM
  uint32_t       riffSize = 36 + dataBytes;

  f.write((const uint8_t*)"RIFF",    4);
  f.write((const uint8_t*)&riffSize, 4);
  f.write((const uint8_t*)"WAVE",    4);
  f.write((const uint8_t*)"fmt ",    4);
  f.write((const uint8_t*)&fmtSize,  4);
  f.write((const uint8_t*)&fmt,      2);
  f.write((const uint8_t*)&ch,       2);
  f.write((const uint8_t*)&sr,       4);
  f.write((const uint8_t*)&byteRate, 4);
  f.write((const uint8_t*)&align,    2);
  f.write((const uint8_t*)&bits,     2);
  f.write((const uint8_t*)"data",    4);
  f.write((const uint8_t*)&dataBytes,4);
}

// ─────────────────────────────────────────────────────────────────────────
// 녹음 시작
// ─────────────────────────────────────────────────────────────────────────
static void codecStartRecording(bool sd)
{
  if (!sd) { Serial.println("[CODEC] SD not ready"); return; }

  // 파일 열기
  char name[32];
  for (uint32_t i = 1; i <= 9999; i++)
  {
    snprintf(name, sizeof(name), "/COD%04u.wav", i);
    if (!SD_MMC.exists(name)) break;
  }
  strncpy(codecFileName, name, sizeof(codecFileName));

  codecFile = SD_MMC.open(codecFileName, FILE_WRITE);
  if (!codecFile) { Serial.printf("[CODEC] Cannot open %s\n", codecFileName); return; }

  codecDataBytes   = 0;
  codecFillBuf     = 0;
  codecFillPos     = 0;
  codecFlushReq    = false;
  codecSampleCount = 0;
  codecPcmMin      = 0;
  codecPcmMax      = 0;
  codecNonZero     = 0;

  writeWavHeader(codecFile, 0);  // 크기는 종료 시 업데이트
  codecFile.flush();

  i2s_zero_dma_buffer((i2s_port_t)NAU88_I2S_PORT);

  codecState = CODEC_RECORDING;
  ledSetState(LED_LOGGING);
  Serial.printf("[CODEC] REC start → %s  (%d sec)\n",
    codecFileName, NAU88_RECORD_SECONDS);
}

// ─────────────────────────────────────────────────────────────────────────
// 녹음 종료 및 WAV 저장
// ─────────────────────────────────────────────────────────────────────────
static void codecStopRecording(bool sd)
{
  codecState = CODEC_SAVING;

  if (!sd || !codecFile) { codecState = CODEC_STANDBY; return; }

  const uint32_t maxBytes = (uint32_t)NAU88_RECORD_SECONDS * NAU88_SAMPLE_RATE * 2;

  // 완료된 더블 버퍼 플러시
  if (codecFlushReq)
  {
    uint32_t need  = maxBytes - min(codecDataBytes, maxBytes);
    uint32_t bytes = min((uint32_t)(NAU88_BUF_SAMPLES * 2), need);
    if (bytes) { codecFile.write((const uint8_t*)codecBuf[codecFlushBuf], bytes); codecDataBytes += bytes; }
    codecFlushReq = false;
  }

  // 부분 채워진 버퍼 플러시
  {
    uint32_t need  = maxBytes - min(codecDataBytes, maxBytes);
    uint32_t avail = codecFillPos * 2;
    uint32_t bytes = min(avail, need);
    if (bytes) { codecFile.write((const uint8_t*)codecBuf[codecFillBuf], bytes); codecDataBytes += bytes; }
  }

  // WAV 헤더 크기 업데이트
  uint32_t riffSize = 36 + codecDataBytes;
  codecFile.seek(4);  codecFile.write((const uint8_t*)&riffSize,    4);
  codecFile.seek(40); codecFile.write((const uint8_t*)&codecDataBytes, 4);
  codecFile.flush();
  codecFile.close();

  float sec = (float)codecDataBytes / (float)(NAU88_SAMPLE_RATE * 2);
  Serial.println("─────────────────────────────────");
  Serial.printf("[CODEC] Saved  : %s\n", codecFileName);
  Serial.printf("[CODEC] Size   : %u bytes  (%.2f sec)\n", codecDataBytes, sec);
  Serial.printf("[CODEC] Samples: %u  NonZero: %u (%.1f%%)\n",
    codecSampleCount, codecNonZero,
    codecSampleCount ? (100.0f * codecNonZero / codecSampleCount) : 0.0f);
  Serial.printf("[CODEC] PCM    : min=%d  max=%d\n", codecPcmMin, codecPcmMax);
  if (codecNonZero == 0)
    Serial.println("[CODEC] !! ALL ZERO — ADCOUT not driven. Check MCLK/wiring.");
  Serial.println("─────────────────────────────────");

  codecState = CODEC_STANDBY;
  ledSetState(LED_BOOTING);
  Serial.println("[CODEC] Standby. Press BOOT to record.");
}

// ─────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────
void codecRecorderInit()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);

  if (!nau88Init())  { Serial.println("[CODEC] NAU88 init failed!"); return; }
  if (!codecI2SInit()) { Serial.println("[CODEC] I2S init failed!");  return; }

  pinMode(NAU88_TRIGGER_PIN, INPUT_PULLUP);
  Serial.printf("[CODEC] Ready  Trigger=GPIO%d\n", NAU88_TRIGGER_PIN);
}

void codecLoop(bool sd, unsigned long now)
{
  // ── 버튼 감지 (BOOT, 50 ms 디바운스) ────────────────────────────────────
  static bool          lastBtn     = HIGH;
  static unsigned long lastBtnTime = 0;
  bool btn = (bool)digitalRead(NAU88_TRIGGER_PIN);

  if (btn != lastBtn && (now - lastBtnTime) > 50)
  {
    lastBtnTime = now;
    lastBtn     = btn;
    if (btn == LOW && codecState == CODEC_STANDBY)
      codecStartRecording(sd);
  }

  // ── I2S 읽기 및 버퍼 채우기 ──────────────────────────────────────────────
  if (codecState == CODEC_RECORDING)
  {
    size_t bytesRead = 0;
    i2s_read((i2s_port_t)NAU88_I2S_PORT,
             rxBuf, sizeof(rxBuf), &bytesRead, pdMS_TO_TICKS(100));

    uint32_t n = bytesRead / sizeof(int16_t);

    // ── 초반 원시값 덤프 (처음 3샘플만) ─────────────────────────────────
    if (codecSampleCount < 8 && n > 0)
    {
      for (uint32_t d = 0; d < min(n, (uint32_t)3); d++)
        Serial.printf("  [I2S] pcm=%-6d  0x%04X\n",
          rxBuf[d], (uint16_t)rxBuf[d]);
    }

    for (uint32_t i = 0; i < n; i++)
    {
      int16_t s = rxBuf[i];

      // 통계
      if (s != 0) codecNonZero++;
      if (s < codecPcmMin) codecPcmMin = s;
      if (s > codecPcmMax) codecPcmMax = s;

      // 더블 버퍼 채우기
      codecBuf[codecFillBuf][codecFillPos] = s;
      codecSampleCount++;

      if (++codecFillPos >= NAU88_BUF_SAMPLES)
      {
        codecFlushBuf = codecFillBuf;
        codecFlushReq = true;
        codecFillBuf ^= 1;
        codecFillPos  = 0;
      }
    }

    // SD 기록
    if (codecFlushReq && codecFile)
    {
      codecFile.write((const uint8_t*)codecBuf[codecFlushBuf], NAU88_BUF_SAMPLES * 2);
      codecDataBytes += NAU88_BUF_SAMPLES * 2;
      codecFlushReq = false;
    }

    // 진행 출력 (500 ms 간격)
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 500)
    {
      lastPrint = now;
      float elapsed = (float)codecSampleCount / (float)NAU88_SAMPLE_RATE;
      Serial.printf("  %.1f / %d sec  nonzero=%u  min=%d  max=%d\n",
        elapsed, NAU88_RECORD_SECONDS,
        codecNonZero, codecPcmMin, codecPcmMax);
    }

    // 목표 샘플 수 달성 → 종료
    if (codecSampleCount >= (uint32_t)NAU88_RECORD_SECONDS * NAU88_SAMPLE_RATE)
      codecStopRecording(sd);
  }
}

#endif // SENSOR_TYPE == 3
