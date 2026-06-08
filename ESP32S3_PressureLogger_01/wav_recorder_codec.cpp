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
#include "web_server.h"
#include "deep_sleep.h"
#include "kiss_fft.h"

// ── 강도 분석 알고리즘 상수 (Python analysis.py / 검증된 KissFFT 구현과 동일) ──
#define MAX_ANALYSIS_FREQ_HZ  3000  // Python: tfa_data[0:3000]
#define SKIP_LOW_FREQ_HZ      50    // Python: tfa_data[:50] = 0
#define ENERGY_HALF_WIN       10    // Python: avg[idx-10 : idx+10]
#define ANALYSIS_MAX_SEC      3     // 강도 분석은 시작부터 최대 3초 구간만 사용
                                    //  → 녹음 길이 무관 동일 구간 분석(일관성) + 대용량 파일 read 오류 방지

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
static uint32_t codecClipCount  = 0;   // int16 레일(±32767) 근접 샘플 수
#define CODEC_CLIP_MARGIN  64           // 레일 ±64 이내면 포화로 간주

// ── 오프라인 FFT 버퍼 (PSRAM 동적 할당) ──────────────────────────────────
//   CODEC_FFT_SIZE = 32768 (2^15): 제로패딩 후 FFT 포인트 수
//   fftReal / fftImag 각 128 KB → 합계 256 KB PSRAM
//   CODEC_FFT_CAPTURE_SIZE: rawBuf에 저장하는 최대 샘플 수
//     가변 길이 녹음 대응 — FFT_SIZE와 동일하게 설정 (32768 samples @ 8kHz = 4.096초)
//     녹음이 4.096초 이상이면 앞 구간만 FFT 분석 (실제 WAV 저장은 전체 길이)
#define CODEC_FFT_SIZE         32768
#define CODEC_FFT_CAPTURE_SIZE CODEC_FFT_SIZE   // 최대 FFT_SIZE 샘플 저장 (64 KB PSRAM)

static float*   fftReal = nullptr;  // PSRAM: 실수부 (128 KB)
static float*   fftImag = nullptr;  // PSRAM: 허수부 (128 KB)

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
  codecClipCount   = 0;
  rawBufPos        = 0;

  writeWavHeader(codecFile, 0);  // 크기는 종료 시 업데이트
  codecFile.flush();

  i2s_zero_dma_buffer((i2s_port_t)NAU88_I2S_PORT);

  codecState = CODEC_RECORDING;
  ledSetState(LED_LOGGING);
  webSetRecording(true);   // 웹 UI → STOP 버튼으로 전환
  Serial.printf("[CODEC] REC start → %s\n", codecFileName);
}

// ─────────────────────────────────────────────────────────────────────────
// WAV 정규화: 2-pass (피크 탐색 → 게인 적용 → 재기록)
// ─────────────────────────────────────────────────────────────────────────
static void writeWavHeader(File &f, uint32_t dataBytes);  // 전방 선언

static void normalizeWavFile()
{
  if (!fftReal) { Serial.println("[NORM] Buffer not ready"); return; }
  float* normF = fftReal;

  File f = SD_MMC.open(codecFileName, FILE_READ);
  if (!f) { Serial.printf("[NORM] Open failed: %s\n", codecFileName); return; }
  f.seek(44);

  uint32_t count = 0;
  int16_t  peak  = 0;
  int16_t  s;
  while (f.read((uint8_t*)&s, 2) == 2 && count < (uint32_t)CODEC_FFT_CAPTURE_SIZE)
  {
    normF[count++] = (float)s;
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
    normF[i] *= gain;

  File fw = SD_MMC.open(codecFileName, FILE_WRITE);
  if (!fw) { Serial.println("[NORM] Rewrite open failed"); return; }

  writeWavHeader(fw, count * 2);

  int16_t normBuf[256];
  uint32_t written = 0;
  while (written < count)
  {
    uint32_t chunk = min((uint32_t)256, count - written);
    for (uint32_t i = 0; i < chunk; i++)
      normBuf[i] = (int16_t)constrain((long)normF[written + i], -32768L, 32767L);
    fw.write((const uint8_t*)normBuf, chunk * 2);
    written += chunk;
  }
  fw.flush();
  fw.close();
  Serial.printf("[NORM] Done: %u samples normalized\n", count);
}

// ── 헬퍼: |x| 평균 기준 표준편차 (Python: np.std(abs(slice))) ───────────────
static float stdAbsFloat(const float *arr, int n)
{
  float sum = 0.0f;
  for (int i = 0; i < n; i++) sum += fabsf(arr[i]);
  float mean = sum / (float)n;
  float s = 0.0f;
  for (int i = 0; i < n; i++)
  {
    float v = fabsf(arr[i]) - mean;
    s += v * v;
  }
  return sqrtf(s / (float)n);
}

// ── 헬퍼: WAV PCM16 모노 샘플 읽기 → float [-1.0, 1.0] ───────────────────────
// 본 레코더 WAV는 항상 8000Hz·16bit·모노·헤더 44바이트 고정 포맷
static bool wavReadSamples(File &f, uint32_t startFrame, uint32_t count, float *out)
{
  if (!f.seek(44 + (size_t)startFrame * 2)) return false;

  uint8_t  buf[512];
  uint32_t done = 0;
  while (done < count)
  {
    uint32_t toRead = min((uint32_t)sizeof(buf), (count - done) * 2);
    int got = f.read(buf, toRead);
    if (got <= 0) return false;

    int frames = got / 2;
    for (int j = 0; j < frames && done < count; j++, done++)
    {
      int16_t s = (int16_t)((uint16_t)buf[j*2] | ((uint16_t)buf[j*2 + 1] << 8));
      out[done] = (float)s / 32768.0f;
    }
  }
  return true;
}

// ── 강도 분석 (KissFFT · Python analysis.py 와 동일 알고리즘 / SENSOR_TYPE 2와 동일) ──
// 핵심: fftSize = 샘플레이트(8000) → 1 bin = 1 Hz → 정수 Hz 신호가 bin 중심에
//       정확히 정렬되어 스펙트럼 누설(leakage) 제거 → 반복 측정 강도값 안정
//
//   STEP1  표준편차 최소 1초 윈도우 탐색 (가장 안정적인 정상 진동 구간 선택)
//   STEP2  KissFFT (윈도우 함수 없음 = scipy.fftpack.fft 와 동일)
//   STEP3  저주파 제거 (0 ~ 49 Hz = 0)
//   STEP4  피크 주파수 탐색 (50 ~ 3000 Hz)
//   STEP5  wave_energy = mean(peak ±10 bins)  ← 최종 강도값
//
// ※ 반드시 정규화(normalizeWavFile) 이전에 호출 — 원본 신호 기준 강도 측정
// 반환값: wave_energy (캘리브레이션 기준 저장 및 보정 강도 계산에 사용)
static float analyzeFFT(uint32_t totalSamples)
{
  const int sr       = NAU88_SAMPLE_RATE;               // 8000
  const int sec_0_1  = sr / 10;                         // 800  (0.1초)
  const int sec_1    = sr;                              // 8000 (1초 = fftSize)
  const int fftSize  = sec_1;

  // 분석 구간: 시작부터 최대 ANALYSIS_MAX_SEC(3초)까지만 (파일이 길어도 동일 구간)
  uint32_t  analyzeSamples = min(totalSamples, (uint32_t)(ANALYSIS_MAX_SEC * sr));
  const int totalFr  = (int)analyzeSamples;
  const int time_l   = totalFr / sec_1;                 // 정수 초 길이 (≤ 3)
  const int i_time   = (time_l - 1) * 10 - 1;           // 윈도우 스캔 횟수 (3초→19회)

  const int skipBins = SKIP_LOW_FREQ_HZ;                // 50
  const int numBins  = min(MAX_ANALYSIS_FREQ_HZ, fftSize / 2);  // 3000

  if (i_time <= 0)
  {
    Serial.println("[FFT] Audio too short (need >= 2 sec) — skip analysis.");
    return 0.0f;
  }

  File f = SD_MMC.open(codecFileName, FILE_READ);
  if (!f)
  {
    Serial.printf("[FFT] Cannot open %s\n", codecFileName);
    return 0.0f;
  }

  // ── STEP1: 표준편차 최소 윈도우 탐색 ──────────────────────────────────────
  float *tmpBuf = (float*)ps_malloc((size_t)sec_1 * sizeof(float));
  if (!tmpBuf)  tmpBuf = (float*)malloc((size_t)sec_1 * sizeof(float));
  if (!tmpBuf) { Serial.println("[FFT] tmpBuf alloc fail"); f.close(); return 0.0f; }

  float minStd  = 1e30f;
  int   bestIdx = 0;
  for (int i = 0; i < i_time; i++)
  {
    uint32_t startFrame = (uint32_t)(i + 1) * sec_0_1;
    if (!wavReadSamples(f, startFrame, (uint32_t)sec_1, tmpBuf))
    {
      Serial.printf("[FFT] read error (i=%d)\n", i);
      free(tmpBuf); f.close(); return 0.0f;
    }
    float s = stdAbsFloat(tmpBuf, sec_1);
    if (s < minStd) { minStd = s; bestIdx = i; }
  }
  free(tmpBuf);

  const int a = bestIdx + 1;

  // ── STEP2: KissFFT (fftSize = 8000, 윈도우 함수 없음) ─────────────────────
  float        *audioBuf = (float*)ps_malloc((size_t)fftSize * sizeof(float));
  kiss_fft_cpx *fftIn    = (kiss_fft_cpx*)ps_malloc((size_t)fftSize * sizeof(kiss_fft_cpx));
  kiss_fft_cpx *fftOut   = (kiss_fft_cpx*)ps_malloc((size_t)fftSize * sizeof(kiss_fft_cpx));
  float        *mag      = (float*)ps_malloc((size_t)fftSize * sizeof(float));

  if (!audioBuf) audioBuf = (float*)malloc((size_t)fftSize * sizeof(float));
  if (!fftIn)    fftIn    = (kiss_fft_cpx*)malloc((size_t)fftSize * sizeof(kiss_fft_cpx));
  if (!fftOut)   fftOut   = (kiss_fft_cpx*)malloc((size_t)fftSize * sizeof(kiss_fft_cpx));
  if (!mag)      mag      = (float*)malloc((size_t)fftSize * sizeof(float));

  if (!audioBuf || !fftIn || !fftOut || !mag)
  {
    Serial.println("[FFT] FFT buffer alloc fail");
    free(audioBuf); free(fftIn); free(fftOut); free(mag);
    f.close(); return 0.0f;
  }

  const uint32_t fftStart = (uint32_t)a * sec_0_1;
  if (!wavReadSamples(f, fftStart, (uint32_t)fftSize, audioBuf))
  {
    Serial.println("[FFT] FFT window read fail");
    free(audioBuf); free(fftIn); free(fftOut); free(mag);
    f.close(); return 0.0f;
  }
  f.close();

  for (int i = 0; i < fftSize; i++) { fftIn[i].r = audioBuf[i]; fftIn[i].i = 0.0f; }
  free(audioBuf);

  // cfg는 PSRAM에 배치 (twiddle 테이블 ~64KB)
  size_t cfgLen = 0;
  kiss_fft_alloc(fftSize, 0, NULL, &cfgLen);           // 필요 크기 산출
  void *cfgMem = ps_malloc(cfgLen);
  if (!cfgMem) cfgMem = malloc(cfgLen);
  kiss_fft_cfg cfg = kiss_fft_alloc(fftSize, 0, cfgMem, &cfgLen);
  if (!cfg)
  {
    Serial.println("[FFT] KissFFT cfg alloc fail");
    free(cfgMem); free(fftIn); free(fftOut); free(mag);
    return 0.0f;
  }

  kiss_fft(cfg, fftIn, fftOut);
  free(cfgMem);
  free(fftIn);

  // 크기 스펙트럼: |FFT[k]| = sqrt(re²+im²)  (Python: abs(fft(...)))
  for (int i = 0; i < fftSize; i++)
    mag[i] = sqrtf(fftOut[i].r * fftOut[i].r + fftOut[i].i * fftOut[i].i);
  free(fftOut);

  // ── STEP3: 저주파 제거 (Python: tfa_data3000[:50] = 0) ───────────────────
  for (int i = 0; i < skipBins; i++) mag[i] = 0.0f;

  // ── STEP4: 피크 탐색 (Python: idx = argmax(tfa_data3000)) ────────────────
  int peakBin = skipBins;
  for (int i = skipBins + 1; i < numBins; i++)
    if (mag[i] > mag[peakBin]) peakBin = i;
  const float peakFreqHz = (float)peakBin;   // 1 bin = 1 Hz

  // ── STEP5: wave_energy = 피크 ±10 bins 평균 (Python: wave_energy) ────────
  const int eStart = max(0, peakBin - ENERGY_HALF_WIN);
  const int eStop  = min(numBins, peakBin + ENERGY_HALF_WIN);
  float eSum = 0.0f;
  for (int i = eStart; i < eStop; i++) eSum += mag[i];
  const float waveEnergy = eSum / (float)(eStop - eStart);

  // ── (보조) 표준편차 — Python: np.std(tfa_data) ───────────────────────────
  float magSum = 0.0f;
  for (int i = 0; i < numBins; i++) magSum += mag[i];
  const float magMean = magSum / (float)numBins;
  float varSum = 0.0f;
  for (int i = 0; i < numBins; i++) { float d = mag[i] - magMean; varSum += d * d; }
  const float stdDev = sqrtf(varSum / (float)numBins);

  free(mag);

  // ── 결과 출력 ─────────────────────────────────────────────────────────────
  Serial.println("---------------------------------");
  Serial.printf("[FFT] Total      : %.2f sec (%u samples)\n",
    (float)totalSamples / sr, totalSamples);
  Serial.printf("[FFT] Analyze    : first %.2f sec (%d samples)\n",
    (float)totalFr / sr, totalFr);
  Serial.printf("[FFT] Window     : a=%d  std=%.6f  start=%.3f sec  (scan %d)\n",
    a, minStd, (float)(a * sec_0_1) / sr, i_time);
  Serial.printf("[FFT] FFT size   : %d  (res 1.000 Hz/bin)\n", fftSize);
  Serial.printf("[FFT] Peak Freq  : %.0f Hz\n", peakFreqHz);
  Serial.printf("[FFT] Intensity  : %.6f  (mean +-10 bins)\n", waveEnergy);
  Serial.printf("[FFT] Std Dev    : %.6f\n", stdDev);
  Serial.println("---------------------------------");

  return waveEnergy;
}

// ─────────────────────────────────────────────────────────────────────────
// 녹음 종료 및 WAV 저장
// ─────────────────────────────────────────────────────────────────────────
static void codecStopRecording(bool sd)
{
  codecState = CODEC_SAVING;
  webSetRecording(false);  // 웹 UI → REC 버튼으로 복귀

  if (!sd || !codecFile) { codecState = CODEC_STANDBY; return; }

  // 완료된 더블 버퍼 플러시 (가변 길이: maxBytes 상한 제거)
  if (codecFlushReq)
  {
    codecFile.write((const uint8_t*)codecBuf[codecFlushBuf], NAU88_BUF_SAMPLES * 2);
    codecDataBytes += (uint32_t)(NAU88_BUF_SAMPLES * 2);
    codecFlushReq = false;
  }

  // 부분 채워진 버퍼 플러시
  if (codecFillPos > 0)
  {
    codecFile.write((const uint8_t*)codecBuf[codecFillBuf], codecFillPos * 2);
    codecDataBytes += (uint32_t)(codecFillPos * 2);
  }

  // WAV 헤더 크기 업데이트
  uint32_t riffSize = 36 + codecDataBytes;
  codecFile.seek(4);  codecFile.write((const uint8_t*)&riffSize,    4);
  codecFile.seek(40); codecFile.write((const uint8_t*)&codecDataBytes, 4);
  codecFile.flush();
  codecFile.close();

  float sec = (float)codecDataBytes / (float)(NAU88_SAMPLE_RATE * 2);
  Serial.println("---------------------------------");
  Serial.printf("[CODEC] Saved  : %s\n", codecFileName);
  Serial.printf("[CODEC] Size   : %u bytes  (%.2f sec)\n", codecDataBytes, sec);
  Serial.printf("[CODEC] Samples: %u  NonZero: %u (%.1f%%)\n",
    codecSampleCount, codecNonZero,
    codecSampleCount ? (100.0f * codecNonZero / codecSampleCount) : 0.0f);
  Serial.printf("[CODEC] PCM    : min=%d  max=%d  (16bit ±32768)\n", codecPcmMin, codecPcmMax);
  {
    float clipPct = codecSampleCount ? (100.0f * (float)codecClipCount / (float)codecSampleCount) : 0.0f;
    Serial.printf("[CODEC] Clipping: %.2f%%  (%u / %u samples near rail +/-%d)\n",
      clipPct, codecClipCount, codecSampleCount, CODEC_CLIP_MARGIN);
    if (clipPct >= 1.0f)
      Serial.println("[CODEC] !! Saturation -- reduce input gain (intensity unreliable)");
    else if (codecPcmMin <= -32000 || codecPcmMax >= 32000)
      Serial.println("[CODEC] ! Near full-scale -- insufficient headroom (reduce gain)");
  }
  if (codecNonZero == 0)
    Serial.println("[CODEC] !! ALL ZERO — ADCOUT not driven. Check MCLK/wiring.");
  Serial.println("---------------------------------");

  // ★ 강도 분석은 정규화 이전(원본 신호 기준)에 수행
  uint32_t totalSamples = codecDataBytes / 2;
  analyzeFFT(totalSamples);   // KissFFT 주파수/강도 분석 (Intensity 출력)

  normalizeWavFile();   // 피크 탐색 → 게인 적용 → 재기록 (재생 음량 정규화)

  codecState = CODEC_STANDBY;
  ledSetState(LED_BOOTING);
  Serial.println("[CODEC] Standby. Press REC in browser to start recording.");
}

// ─────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────
void codecRecorderInit()
{
  // ── 정규화(normalizeWavFile) 전용 버퍼 PSRAM 할당 ───────────────────────
  //   강도 분석(analyzeFFT)은 KissFFT 자체 버퍼를 on-demand 할당하므로
  //   여기 fftReal/fftImag는 normalizeWavFile 피크 탐색 버퍼로만 사용
  fftReal = (float*)heap_caps_malloc(CODEC_FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  fftImag = (float*)heap_caps_malloc(CODEC_FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fftReal || !fftImag)
    Serial.println("[FFT] PSRAM alloc failed!");
  else
    Serial.printf("[FFT] PSRAM buffer OK: %u KB\n",
      (uint32_t)(CODEC_FFT_SIZE * sizeof(float) * 2 / 1024));

  // 원 데이터 버퍼 PSRAM 할당 (필터 적용 전 I2S 원본)
  rawBuf = (int16_t*)heap_caps_malloc(CODEC_FFT_CAPTURE_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rawBuf)
    Serial.println("[RAW] PSRAM alloc failed!");
  else
    Serial.printf("[RAW] PSRAM alloc OK: %u KB\n",
      (uint32_t)(CODEC_FFT_CAPTURE_SIZE * sizeof(int16_t) / 1024));

  // I2S 드라이버를 먼저 설치 — 코덱 미연결 상태에서도 크래시 없이 동작
  if (!codecI2SInit()) { Serial.println("[CODEC] I2S init failed!"); return; }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_FREQ);

  // 코덱 초기화 실패는 치명적이지 않음 — 미연결 시 I2S는 0/노이즈 수신
  if (!nau88Init()) {
    Serial.println("[CODEC] NAU88 not found - I2S will receive zeros/noise");
  }

  pinMode(NAU88_TRIGGER_PIN, INPUT_PULLUP);
  Serial.printf("[CODEC] Ready  Trigger=GPIO%d\n", NAU88_TRIGGER_PIN);
}

void codecLoop(bool sd, unsigned long now)
{
  // ── 웹 REC 버튼 ──────────────────────────────────────────────────────────
  if (codecState == CODEC_STANDBY && webRecordConsumed())
    codecStartRecording(sd);

  // ── 웹 STOP 버튼 ─────────────────────────────────────────────────────────
  if (codecState == CODEC_RECORDING && webStopConsumed())
    codecStopRecording(sd);

  // ── 비상 물리 버튼 (방수 기구물에서는 정상 미사용, 디버그 전용) ──────────
#if 0
  static bool          lastBtn     = HIGH;
  static unsigned long lastBtnTime = 0;
  bool btn = (bool)digitalRead(NAU88_TRIGGER_PIN);
  if (btn != lastBtn && (now - lastBtnTime) > 50)
  {
    lastBtnTime = now;
    lastBtn     = btn;
    if (btn == LOW && codecState == CODEC_STANDBY)   codecStartRecording(sd);
    if (btn == LOW && codecState == CODEC_RECORDING) codecStopRecording(sd);
  }
#endif

  // ── I2S 읽기 및 버퍼 채우기 ──────────────────────────────────────────────
  if (codecState == CODEC_RECORDING)
  {
    size_t bytesRead = 0;
    i2s_read((i2s_port_t)NAU88_I2S_PORT,
             rxBuf, sizeof(rxBuf), &bytesRead, pdMS_TO_TICKS(100));

    uint32_t n = bytesRead / sizeof(int16_t);

    // 초반 원시값 덤프 (처음 3샘플만)
    if (codecSampleCount < 8 && n > 0)
    {
      for (uint32_t d = 0; d < min(n, (uint32_t)3); d++)
        Serial.printf("  [I2S] pcm=%-6d  0x%04X\n",
          rxBuf[d], (uint16_t)rxBuf[d]);
    }

    for (uint32_t i = 0; i < n; i++)
    {
      int16_t s = rxBuf[i];

      if (rawBuf && rawBufPos < (uint32_t)CODEC_FFT_CAPTURE_SIZE)
        rawBuf[rawBufPos++] = s;

      if (s != 0) codecNonZero++;
      if (s < codecPcmMin) codecPcmMin = s;
      if (s > codecPcmMax) codecPcmMax = s;
      if (s >= (32767 - CODEC_CLIP_MARGIN) || s <= (-32768 + CODEC_CLIP_MARGIN)) codecClipCount++;

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

    // 진행 출력 + 웹 경과 시간 갱신 (500 ms 간격)
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 500)
    {
      lastPrint = now;
      float    elapsed    = (float)codecSampleCount / (float)NAU88_SAMPLE_RATE;
      uint32_t elapsedSec = (uint32_t)elapsed;
      webSetElapsed(elapsedSec);

      // SD 여유 공간 체크 (5초마다)
      static unsigned long lastSpaceCheck = 0;
      if (now - lastSpaceCheck >= 5000)
      {
        lastSpaceCheck = now;
        uint64_t freeMB = (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL);
        if (freeMB < (uint64_t)NAU88_SD_RESERVE_MB)
        {
          Serial.printf("[CODEC] SD free space low (%llu MB) — auto stop\n", freeMB);
          codecStopRecording(sd);
          return;
        }
      }

      Serial.printf("  %.1f sec  nonzero=%u  min=%d  max=%d\n",
        elapsed, codecNonZero, codecPcmMin, codecPcmMax);
    }

    // 안전 최대 시간 초과 → 자동 종료
    if (codecSampleCount >= (uint32_t)NAU88_MAX_RECORD_SEC * NAU88_SAMPLE_RATE)
    {
      Serial.printf("[CODEC] Max record time (%d sec) reached — auto stop\n", NAU88_MAX_RECORD_SEC);
      codecStopRecording(sd);
    }
  }

  // ── DeepSleep: SLEEP_IDLE_SEC 동안 대기 상태 유지 시 슬립 진입 ────────────
#if USE_DEEPSLEEP
  {
    static bool          inStandby  = false;
    static unsigned long standbyMs  = 0;

    if (codecState == CODEC_STANDBY)
    {
      if (!inStandby) { inStandby = true; standbyMs = now; }

      if ((now - standbyMs) >= (unsigned long)SLEEP_IDLE_SEC * 1000UL)
      {
        Serial.printf("[SLEEP] Standby for %d sec — attempting DeepSleep\n", SLEEP_IDLE_SEC);
        deepSleepEnter();
        standbyMs = now;  // 리드스위치 활성으로 반환된 경우 타이머 리셋
      }
    }
    else
    {
      inStandby = false;  // 녹음/저장 중이면 타이머 리셋
    }
  }
#endif
}

#endif // SENSOR_TYPE == 3
