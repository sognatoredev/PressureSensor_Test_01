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
#include <arduinoFFT.h>

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

// ── FFT 버퍼 (PSRAM 동적 할당) ────────────────────────────────────────────
#define CODEC_FFT_SIZE         32768
#define CODEC_FFT_CAPTURE_SIZE (NAU88_RECORD_SECONDS * NAU88_SAMPLE_RATE)

static float*   fftReal  = nullptr;
static float*   fftImag  = nullptr;

// ── 원 데이터 버퍼 (필터 적용 전 I2S 원본, PSRAM) ──────────────────────────
static int16_t* rawBuf    = nullptr;  // PSRAM: CODEC_FFT_CAPTURE_SIZE × 2 bytes
static uint32_t rawBufPos = 0;

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

  // R01 Power Management 1
  // [수정] NAU88C10 실제 비트 레이아웃 기준으로 수정
  //   bit8=DCBUFEN(1), bit3=ABIASEN(1,필수!), bit2=IOBUFEN(1), bit1:0=REFIMP=01(80kΩ)
  //   0x010D = 0b1_0000_1101
  // [원본 오류] WM8510 레이아웃 혼용: VMIDSEL/BUFIOEN/BIASEN 비트 위치 오해 → ABIASEN=0으로 아날로그 전체 비활성
  //   nau88WriteReg(0x01, 0x0D0);  // WRONG: ABIASEN=0, REFIMP=00 → 아날로그 섹션 비활성화
  nau88WriteReg(0x01, 0x010D);
  delay(300);  // VMID 커패시터 충전 대기

  // R02 Power Management 2
  // [수정] BSTEN=1(D4), PGAEN=1(D2), ADCEN=1(D0) → 0x015
  // [원본 오류] bit3=1(0x008)은 NAU88C10에서 reserved(must be 0); ADCEN/PGAEN/BSTEN 모두 0으로 신호 경로 미활성
  //   nau88WriteReg(0x02, 0x008);  // WRONG: ADCEN=0, PGAEN=0, BSTEN=0 → ADC 신호 경로 전체 비활성
  nau88WriteReg(0x02, 0x015);

  // R03 Power Management 3
  // [수정] ADC 전용 모드에서 불필요 → 생략 (기본값 0x000)
  // [원본 오류] bit1=reserved(must be 0); 의미 있는 비트 없음
  //   nau88WriteReg(0x03, 0x002);  // WRONG: reserved 비트 설정, 기능 없음

  // R04 Audio Interface
  // [수정] WLEN=00(16bit), AIFMT=10(I2S standard) → 0x010
  // [원본 오류] 0x004 = AIFMT=00(Right-Justified); ESP32 I2S 표준 포맷과 불일치 → 비트 정렬 오류
  //   nau88WriteReg(0x04, 0x004);  // WRONG: Right-Justified 포맷, I2S 표준이 아님
  nau88WriteReg(0x04, 0x010);

  // R06 Clock Control
  // CLKM=0(MCLK), MCLKDIV=000(÷1), Slave mode → SYSCLK=2.048MHz, fs=8kHz
  // nau88WriteReg(0x06, 0x000);  // 변경 없음
  nau88WriteReg(0x06, 0x018);  // 변경 없음

  // R07 ADC/DAC Sampling Rate
  // SMPLR[2:0]=110(6) → 44.1kHz 필터 계수 선택 (WM8510/NAU8810 호환 테이블)
  // MCLK=11.2896MHz(APLL), 256×44100=11,289,600Hz → SYSCLK=fs×256
  // nau88WriteReg(0x07, 0x006);
  nau88WriteReg(0x07, 0x005);

  // R14 ADC Control
  // HPFEN=0: HPF 비활성화, DC 포함 전체 통과 (진동 측정 목적)
  // nau88WriteReg(0x0E, 0x000);  // 변경 없음
  nau88WriteReg(0x0E, 0x100);  // 변경 없음

  // R15 Left ADC Volume
  // LADCVU=1(update), LADCVOL=0xFF(최대) → 0x1FF
  nau88WriteReg(0x0F, 0x1FF);  // 변경 없음
  // nau88WriteReg(0x0F, 0x0FF);  // 변경 없음

  // R44 MIC/PGA Input Selection
  // [수정] NMICPGA=1(D1), PMICPGA=1(D0): 차동 MIC 입력 → PGA 연결 → 0x003
  // [원본 오류] 0x0FF는 D7:D2 = undefined 비트 설정; WM8510의 LMICP/LPGAUPDATE/LPGAVOL 비트 혼용
  //   nau88WriteReg(0x2C, 0x0FF);  // WRONG: undefined 비트 설정, WM8510 비트 레이아웃 혼용
  nau88WriteReg(0x2C, 0x003);

  // R45 PGA Gain
  //   gain_dB = -12 + PGAGAIN × 0.75
  //   0x03F(63) = +35.25dB  ← 클리핑 발생, 너무 높음
  //   0x020(32) = +12.0 dB
  //   0x018(24) =  +6.0 dB
  //   0x010(16) =   0.0 dB  ← 압전 직결 시 시작점으로 권장
  // [클리핑 대응] 0x03F → 0x010 (0dB)로 낮춤; 신호 크기 보고 조정 필요
  nau88WriteReg(0x2D, 0x010);  // 0dB; 클리핑 시 낮추고, 신호 작으면 0x018~0x020으로 올릴 것
  // nau88WriteReg(0x2D, 0x03F);  // 0dB; 클리핑 시 낮추고, 신호 작으면 0x018~0x020으로 올릴 것

  // R47 ADC Boost (Boost Stage)
  // PGABST=1(D8): +20dB 부스트
  // [클리핑 대응] 부스트 비활성(0x000)으로 변경 — 0dB PGA + 20dB 부스트도 클리핑 가능성 높음
    // nau88WriteReg(0x2F, 0x100);  // +20dB 부스트 활성 — 게인 과다로 클리핑
  nau88WriteReg(0x2F, 0x000);  // 부스트 OFF; 신호 작으면 0x100으로 되돌릴 것
  // nau88WriteReg(0x2F, 0x100);  // 부스트 OFF; 신호 작으면 0x100으로 되돌릴 것

  // R49 Output Control
  // [수정] TSEN=1(D1): 열 차단 보호 활성 → 0x002
  // [원본 오류] 0x100 = bit8=reserved(must be 0); ADC Boost는 R47(0x2F)에 있으며 여기는 무관
  //   nau88WriteReg(0x31, 0x100);  // WRONG: reserved 비트 설정, ADC Boost와 무관한 레지스터
  nau88WriteReg(0x31, 0x002);

  Serial.println("[CODEC] NAU88C10YG init OK");
  return true;
}

// ─────────────────────────────────────────────────────────────────────────
// I2S 마스터 RX 초기화
//   use_apll=true : APLL로 MCLK=11.2896MHz (=256×44100) 정확히 생성
//   fixed_mclk    : APLL 타겟 주파수 명시 → 44.1kHz 계열 정수비 실현
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
    .dma_buf_count        = 8,
    .dma_buf_len          = 256,
    .use_apll             = true,
    .tx_desc_auto_clear   = false,
    // .fixed_mclk           = 11289600  // 256 × 44100 Hz
    .fixed_mclk           = 2048000   // 256 × 8000 Hz
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
  rawBufPos        = 0;

  writeWavHeader(codecFile, 0);  // 크기는 종료 시 업데이트
  codecFile.flush();

  i2s_zero_dma_buffer((i2s_port_t)NAU88_I2S_PORT);

  codecState = CODEC_RECORDING;
  ledSetState(LED_LOGGING);
  Serial.printf("[CODEC] REC start → %s  (%d sec)\n",
    codecFileName, NAU88_RECORD_SECONDS);
}

// ─────────────────────────────────────────────────────────────────────────
// WAV 정규화: 2-pass (피크 탐색 → 게인 적용 → 재기록)
// ─────────────────────────────────────────────────────────────────────────
static void writeWavHeader(File &f, uint32_t dataBytes);  // 전방 선언

static void normalizeWavFile()
{
  if (!fftReal) { Serial.println("[NORM] Buffer not ready"); return; }

  File f = SD_MMC.open(codecFileName, FILE_READ);
  if (!f) { Serial.printf("[NORM] Open failed: %s\n", codecFileName); return; }
  f.seek(44);

  uint32_t count = 0;
  int16_t  peak  = 0;
  int16_t  s;
  while (f.read((uint8_t*)&s, 2) == 2 && count < (uint32_t)CODEC_FFT_CAPTURE_SIZE)
  {
    fftReal[count++] = (float)s;
    int16_t a = (s < 0) ? -s : s;
    if (a > peak) peak = a;
  }
  f.close();

  if (peak < 64) { Serial.println("[NORM] Signal too weak, skip normalization"); return; }

  const float TARGET = 29204.0f;  // -1 dBFS
  float gain = TARGET / (float)peak;
  Serial.printf("[NORM] Peak: %d  Gain: %.3f (%+.1f dB)\n",
    peak, gain, 20.0f * log10f(gain));

  for (uint32_t i = 0; i < count; i++)
    fftReal[i] *= gain;

  File fw = SD_MMC.open(codecFileName, FILE_WRITE);
  if (!fw) { Serial.println("[NORM] Rewrite open failed"); return; }

  writeWavHeader(fw, count * 2);

  int16_t normBuf[256];
  uint32_t written = 0;
  while (written < count)
  {
    uint32_t chunk = min((uint32_t)256, count - written);
    for (uint32_t i = 0; i < chunk; i++)
      normBuf[i] = (int16_t)constrain((long)fftReal[written + i], -32768L, 32767L);
    fw.write((const uint8_t*)normBuf, chunk * 2);
    written += chunk;
  }
  fw.flush();
  fw.close();
  Serial.printf("[NORM] Done: %u samples normalized\n", count);
}

// ─────────────────────────────────────────────────────────────────────────
// FFT 피크 주파수 분석
// ─────────────────────────────────────────────────────────────────────────
static void analyzeFFT()
{
  if (!fftReal || !fftImag) { Serial.println("[FFT] Buffer not ready, skipping."); return; }
  if (!rawBuf || rawBufPos == 0) { Serial.println("[FFT] Raw buffer empty, skipping."); return; }

  // 원 데이터(필터 미적용)를 직접 fftReal로 복사 — SD 파일 재읽기 불필요
  uint32_t captureSize = min(rawBufPos, (uint32_t)CODEC_FFT_CAPTURE_SIZE);
  // float mean = 0.0f;  // DC 제거용 — 코덱 내장 HPF(R14) 활성 시 불필요, 필요 시 주석 해제
  for (uint32_t i = 0; i < captureSize; i++)
  {
    fftReal[i] = (float)rawBuf[i];
    // mean += fftReal[i];
  }
  // mean /= (float)captureSize;

  // Hamming 윈도우 (DC 제거 비활성 — 코덱 HPF가 대신 처리)
  for (uint32_t i = 0; i < captureSize; i++)
  {
    // fftReal[i] -= mean;  // DC 제거 — 코덱 내장 HPF 미사용 시 주석 해제
    fftReal[i] *= 0.54f - 0.46f * cosf(TWO_PI * i / (float)(captureSize - 1));
  }

  // 제로패딩
  memset(&fftReal[captureSize], 0,
    (CODEC_FFT_SIZE - captureSize) * sizeof(float));
  memset(fftImag, 0, CODEC_FFT_SIZE * sizeof(float));

  ArduinoFFT<float> FFT(fftReal, fftImag, CODEC_FFT_SIZE, (float)NAU88_SAMPLE_RATE);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // 전기 노이즈 대역(59~61 Hz) 제거 — peak 탐색에서 제외
  {
    uint32_t binLow  = (uint32_t)(59.0f * CODEC_FFT_SIZE / NAU88_SAMPLE_RATE);
    uint32_t binHigh = (uint32_t)(61.0f * CODEC_FFT_SIZE / NAU88_SAMPLE_RATE) + 1;
    for (uint32_t b = binLow; b <= binHigh; b++) fftReal[b] = 0.0f;
  }

  float peakFreq = FFT.majorPeak();

  Serial.println("─────────────────────────────────");
  Serial.printf("[FFT] Source   : RAW (codec PCM, no SW filter)\n");
  Serial.printf("[FFT] Signal   : %u samples (%.3f sec)\n",
    captureSize, (float)captureSize / NAU88_SAMPLE_RATE);
  Serial.printf("[FFT] Zero-pad : %u → %u\n", captureSize, CODEC_FFT_SIZE);
  Serial.printf("[FFT] Freq Resolution : %.3f Hz/bin\n",
    (float)NAU88_SAMPLE_RATE / (float)CODEC_FFT_SIZE);
  // Serial.printf("[FFT] DC Offset removed : %.1f\n", mean);  // 코덱 HPF 활성 시 불필요
  Serial.printf("[FFT] Peak Freq : %.2f Hz\n", peakFreq);
  Serial.println("─────────────────────────────────");
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

  normalizeWavFile();   // 피크 탐색 → 게인 적용 → 재기록
  analyzeFFT();         // 정규화된 파일로 FFT 주파수 분석

  codecState = CODEC_STANDBY;
  ledSetState(LED_BOOTING);
  Serial.println("[CODEC] Standby. Press BOOT to record.");
}

// ─────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────
void codecRecorderInit()
{
  // FFT 버퍼를 PSRAM에 동적 할당
  fftReal = (float*)heap_caps_malloc(CODEC_FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  fftImag = (float*)heap_caps_malloc(CODEC_FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fftReal || !fftImag)
    Serial.println("[FFT] PSRAM alloc failed!");
  else
    Serial.printf("[FFT] PSRAM alloc OK: %u KB\n",
      (uint32_t)(CODEC_FFT_SIZE * sizeof(float) * 2 / 1024));

  // 원 데이터 버퍼 PSRAM 할당 (필터 적용 전 I2S 원본)
  rawBuf = (int16_t*)heap_caps_malloc(CODEC_FFT_CAPTURE_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rawBuf)
    Serial.println("[RAW] PSRAM alloc failed!");
  else
    Serial.printf("[RAW] PSRAM alloc OK: %u KB\n",
      (uint32_t)(CODEC_FFT_CAPTURE_SIZE * sizeof(int16_t) / 1024));

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

      // ── 원 데이터 저장 (필터 적용 전, FFT 전용) ───────────────────────────
      if (rawBuf && rawBufPos < (uint32_t)CODEC_FFT_CAPTURE_SIZE)
        rawBuf[rawBufPos++] = s;

      // ── IIR High-Pass (DC/저주파 제거, fc ≈ 10 Hz @ 44.1kHz) ──
      static float hpPrev = 0.0f;
      static float hpOut  = 0.0f;
      const float  hpA    = 0.9986f;  // 1 - 2π×10/44100
      hpOut  = hpA * (hpOut + (float)s - hpPrev);
      hpPrev = (float)s;
      s = (int16_t)hpOut;

      // ── IIR Low-Pass (고주파 노이즈 제거, fc ≈ 5000 Hz @ 44.1kHz) ──
      static float lpState = 0.0f;
      const float  lpA     = 0.5412f;  // 1 - 2π×5000/44100 (근사)
      lpState = lpState * lpA + (float)s * (1.0f - lpA);
      s = (int16_t)lpState;

      // ── 소프트웨어 IIR 저역통과 필터 (fc ≈ 2000 Hz @ 44100 Hz) ──
      // alpha = 2π×fc / (2π×fc + fs) = 2π×2000 / (2π×2000 + 44100) ≈ 0.222
      static float sflpState = 0.0f;
      const float  lpAlpha = 0.222f;
      // LPF 적용
      sflpState = sflpState + lpAlpha * ((float)s - sflpState);
      s = (int16_t)sflpState;

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
