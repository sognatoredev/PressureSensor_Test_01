#include "wav_recorder.h"

#if SENSOR_TYPE == 2

#include <Arduino.h>
#include <SD_MMC.h>
#include <stdarg.h>
#include "esp_timer.h"
#include "esp_wifi.h"
#include "led_indicator.h"
#include "sd_logger.h"
#include "web_server.h"
#include "deep_sleep.h"
#include "kiss_fft.h"

// ── 강도 분석 알고리즘 상수 (Python analysis.py / 검증된 KissFFT 구현과 동일) ──
#define MAX_ANALYSIS_FREQ_HZ  3000  // Python: tfa_data[0:3000]
#define SKIP_LOW_FREQ_HZ      50    // Python: tfa_data[:50] = 0
#define ENERGY_HALF_WIN       10    // Python: avg[idx-10 : idx+10]
#define ANALYSIS_MAX_SEC      3     // 강도 분석은 시작부터 최대 3초 구간만 사용
                                    //  → 녹음 길이 무관 동일 구간 분석(일관성) + 대용량 파일 read 오류 방지

// Serial 출력과 SD 로그 파일에 동시 기록
static void wavLog(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void wavLog(const char* fmt, ...)
{
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  int n = (int)strlen(buf);
  while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r')) buf[--n] = '\0';
  sdDebugLine(buf);
}

// ── FFT Analysis ──────────────────────────────────────────────────────────
// FFT_SIZE = 32768 (2^15): 최대 32768 샘플 분석, 초과분은 앞 구간만 사용
// 주파수 해상도 = WAV_SAMPLE_RATE / FFT_SIZE = 8000 / 32768 ≈ 0.244 Hz/bin
#define FFT_SIZE   32768   // 2^15 (PSRAM: 32768×4×2 = 256 KB)

static float* fftReal = nullptr;
static float* fftImag = nullptr;

// ── Biquad IIR 필터 (Direct Form II Transposed) ────────────────────────────
// fs = 8000 Hz 기준으로 재계산 (MEMS 마이크용)
//
// [변경 이력]
//   이전: 44100Hz 기준 계수 사용 → 실제 fc: HP≈36Hz, LP≈362Hz (설계 의도 대비 5~6배 저하)
//   현재: 8000Hz 기준으로 재계산
//     HP  80Hz @ 8000Hz (입바람·저주파 핸들링 노이즈 제거, 음성 대역 유지)
//     LP 3600Hz @ 8000Hz (나이퀴스트 4000Hz 직전 앨리어싱 방지)
//
// HP 80Hz, Q=0.7071 @ fs=8000Hz:
//   K = tan(π×80/8000) = tan(0.031416) = 0.031462
//   norm = 1 + K/Q + K² = 1 + 0.044499 + 0.000989 = 1.045488
//   b0 =  1/norm           =  0.95652
//   b1 = -2/norm           = -1.91303
//   b2 =  1/norm           =  0.95652
//   a1 = 2(K²-1)/norm      = -1.91115  [ 2×(0.000989-1)/1.045488 ]
//   a2 = (1-K/Q+K²)/norm   =  0.91454  [ (1-0.044499+0.000989)/1.045488 ]
typedef struct {
  float b0, b1, b2;
  float a1, a2;
  float w1, w2;
} Biquad_t;

static inline float biquadProcess(Biquad_t* f, float x)
{
  float y = f->b0 * x + f->w1;
  f->w1   = f->b1 * x - f->a1 * y + f->w2;
  f->w2   = f->b2 * x - f->a2 * y;
  return y;
}

// HP 80Hz @ 8000Hz, Q=0.7071 (2차 Butterworth 하이패스)
static Biquad_t hpFilter = {
   0.95652f, -1.91303f,  0.95652f,   // b0, b1, b2
  -1.91115f,  0.91454f,              // a1, a2
   0.0f, 0.0f                        // w1, w2 (상태)
};

// LP 3600Hz @ 8000Hz: 나이퀴스트(4kHz)에 매우 가까우므로 1차 RC 필터 사용
// α = 1 - exp(-2π × fc / fs) = 1 - exp(-2π×3600/8000) = 1 - exp(-2.827) ≈ 0.9406
// y[n] = α×x[n] + (1-α)×y[n-1]  →  IIR 1차 LPF
static float lpAlpha  = 0.9406f;   // LP 3600Hz @ 8000Hz
static float lpState  = 0.0f;      // 이전 출력값

// ── TPDF 디더링 (삼각형 PDF, LCG 난수) ────────────────────────────────────
static uint32_t rngState = 0xDEADBEEFu;

static inline float tpdfDither()
{
  rngState = rngState * 1664525u + 1013904223u;
  int d1 = (int)(rngState >> 31);
  rngState = rngState * 1664525u + 1013904223u;
  int d2 = (int)(rngState >> 31);
  return (float)(d1 - d2);
}

// ── WAV State Machine ─────────────────────────────────────────────────────
enum WavState { WAV_STANDBY, WAV_RECORDING, WAV_SAVING };
static WavState wavState = WAV_STANDBY;

// ── Double buffer (WAV_BUF_SAMPLES × 2 × 2 = 4 KB in internal SRAM) ───────
static int16_t           wavBuf[2][WAV_BUF_SAMPLES];
static volatile uint8_t  wavFillBuf  = 0;
static volatile uint16_t wavFillPos  = 0;
static volatile uint8_t  wavFlushBuf = 0;
static volatile bool     wavFlushReq = false;
static volatile uint32_t wavISRCount = 0;

// ── 포화(클리핑) 진단 통계 (녹음 중 누적, 종료 시 출력) ────────────────────
static volatile int      wavRawMin    = 4095;   // ADC 원본 최소 (0~4095)
static volatile int      wavRawMax    = 0;       // ADC 원본 최대
static volatile uint32_t wavClipCount = 0;       // ADC 레일(0/4095) 근접 샘플 수
#define WAV_CLIP_MARGIN  4                        // 레일 ±4 이내면 포화로 간주

// ── File tracking ──────────────────────────────────────────────────────────
static File     wavFile;
static char     wavFileName[32];
static uint32_t wavDataBytes = 0;

// ── 녹음 시작 시각 (경과 시간 계산용) ─────────────────────────────────────
static unsigned long wavStartMs = 0;

// ── esp_timer handle ───────────────────────────────────────────────────────
static esp_timer_handle_t wavEspTimer = NULL;

// Timer callback — runs in esp_timer task context (NOT an ISR)
static void wavTimerCb(void *arg)
{
  int raw = analogRead(WAV_ADC_PIN);

  // ── 포화 진단: ADC 원본 min/max + 레일 근접 카운트 ──────────────────────
  if (raw < wavRawMin) wavRawMin = raw;
  if (raw > wavRawMax) wavRawMax = raw;
  if (raw <= WAV_CLIP_MARGIN || raw >= (4095 - WAV_CLIP_MARGIN)) wavClipCount++;

  // 12bit ADC 중심값(2048) 기준 부호 있는 값으로 변환, 4비트 좌쉬프트로 16bit 스케일
  float sample = (float)((raw - 2048) << 4);

  // 1. HP 80Hz — DC 및 저주파 입바람/핸들링 노이즈 제거 (MEMS 마이크용)
  sample = biquadProcess(&hpFilter, sample);

  // 2. LP 3600Hz — 나이퀴스트 근방 앨리어싱 방지 (1차 IIR RC 필터)
  lpState = lpAlpha * sample + (1.0f - lpAlpha) * lpState;
  sample  = lpState;

  // 3. TPDF 디더링 — int16 재양자화 시 왜곡 분산
  sample += tpdfDither();

  int16_t pcm = (int16_t)constrain((long)sample, -32768L, 32767L);

  wavBuf[wavFillBuf][wavFillPos] = pcm;
  wavISRCount++;

  if (++wavFillPos >= WAV_BUF_SAMPLES)
  {
    wavFlushBuf = wavFillBuf;
    wavFlushReq = true;
    wavFillBuf ^= 1;
    wavFillPos  = 0;
  }
}

// 전방 선언
static void writeWavHeader(File &f, uint32_t dataSize);

// ── 정규화: WAV 파일 2-pass (Peak → Gain → 재기록) ───────────────────────
static void normalizeWavFile(uint32_t sampleCount)
{
  if (!fftReal) { wavLog("[NORM] Buffer not ready\n"); return; }

  File f = SD_MMC.open(wavFileName, FILE_READ);
  if (!f) { wavLog("[NORM] Open failed: %s\n", wavFileName); return; }
  f.seek(44);

  // Pass 1: 피크 탐색 (최대 FFT_SIZE 샘플, 초과분은 건너뜀)
  uint32_t readCount = min(sampleCount, (uint32_t)FFT_SIZE);
  uint32_t count = 0;
  int16_t  peak  = 0;
  int16_t  s;
  while (f.read((uint8_t*)&s, 2) == 2 && count < readCount)
  {
    fftReal[count++] = (float)s;
    int16_t a = (s < 0) ? -s : s;
    if (a > peak) peak = a;
  }
  f.close();

  if (peak < 64)
  {
    wavLog("[NORM] Signal too weak, skip normalization\n");
    return;
  }

  const float TARGET = 29204.0f;  // -1 dBFS
  float gain = TARGET / (float)peak;
  wavLog("[NORM] Peak: %d  Gain: %.3f (%+.1f dB)\n",
    peak, gain, 20.0f * log10f(gain));

  for (uint32_t i = 0; i < count; i++)
    fftReal[i] *= gain;

  File fw = SD_MMC.open(wavFileName, FILE_WRITE);
  if (!fw) { wavLog("[NORM] Rewrite open failed\n"); return; }

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
  wavLog("[NORM] Done: %u samples normalized\n", count);
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

// ── 강도 분석 (KissFFT · Python analysis.py 와 동일 알고리즘) ────────────────
// 핵심: fftSize = 샘플레이트(8000) → 1 bin = 1 Hz → 정수 Hz 신호가 bin 중심에
//       정확히 정렬되어 스펙트럼 누설(leakage) 제거 → 반복 측정 강도값 안정
//
//   STEP1  표준편차 최소 1초 윈도우 탐색 (가장 안정적인 정상 진동 구간 선택)
//   STEP2  KissFFT (윈도우 함수 없음 = scipy.fftpack.fft 와 동일)
//   STEP3  저주파 제거 (0 ~ 49 Hz = 0)
//   STEP4  피크 주파수 탐색 (50 ~ 3000 Hz)
//   STEP5  wave_energy = mean(peak ±10 bins)  ← 최종 강도값
//
// 반환값: wave_energy (캘리브레이션 기준 저장 및 보정 강도 계산에 사용)
static float analyzeFFT(uint32_t totalSamples)
{
  const int sr       = WAV_SAMPLE_RATE;                 // 8000
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
    wavLog("[FFT] Audio too short (need >= 2 sec) — skip analysis.\n");
    return 0.0f;
  }

  File f = SD_MMC.open(wavFileName, FILE_READ);
  if (!f)
  {
    wavLog("[FFT] Cannot open %s\n", wavFileName);
    return 0.0f;
  }

  // ── STEP1: 표준편차 최소 윈도우 탐색 ──────────────────────────────────────
  float *tmpBuf = (float*)ps_malloc((size_t)sec_1 * sizeof(float));
  if (!tmpBuf)  tmpBuf = (float*)malloc((size_t)sec_1 * sizeof(float));
  if (!tmpBuf) { wavLog("[FFT] tmpBuf alloc fail\n"); f.close(); return 0.0f; }

  float minStd  = 1e30f;
  int   bestIdx = 0;
  for (int i = 0; i < i_time; i++)
  {
    uint32_t startFrame = (uint32_t)(i + 1) * sec_0_1;
    if (!wavReadSamples(f, startFrame, (uint32_t)sec_1, tmpBuf))
    {
      wavLog("[FFT] read error (i=%d)\n", i);
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
    wavLog("[FFT] FFT buffer alloc fail\n");
    free(audioBuf); free(fftIn); free(fftOut); free(mag);
    f.close(); return 0.0f;
  }

  const uint32_t fftStart = (uint32_t)a * sec_0_1;
  if (!wavReadSamples(f, fftStart, (uint32_t)fftSize, audioBuf))
  {
    wavLog("[FFT] FFT window read fail\n");
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
    wavLog("[FFT] KissFFT cfg alloc fail\n");
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
  wavLog("---------------------------------\n");
  wavLog("[FFT] Total      : %.2f sec (%u samples)\n",
    (float)totalSamples / sr, totalSamples);
  wavLog("[FFT] Analyze    : first %.2f sec (%d samples)\n",
    (float)totalFr / sr, totalFr);
  wavLog("[FFT] Window     : a=%d  std=%.6f  start=%.3f sec  (scan %d)\n",
    a, minStd, (float)(a * sec_0_1) / sr, i_time);
  wavLog("[FFT] FFT size   : %d  (res 1.000 Hz/bin)\n", fftSize);
  wavLog("[FFT] Peak Freq  : %.0f Hz\n", peakFreqHz);
  wavLog("[FFT] Intensity  : %.6f  (mean +-10 bins)\n", waveEnergy);
  wavLog("[FFT] Std Dev    : %.6f\n", stdDev);
  wavLog("---------------------------------\n");

  return waveEnergy;
}

// ── WAV Header ────────────────────────────────────────────────────────────
static void writeWavHeader(File &f, uint32_t dataSize)
{
  const uint16_t numCh    = 1;
  const uint16_t bits     = 16;
  const uint32_t sr       = WAV_SAMPLE_RATE;
  const uint32_t byteRate = sr * numCh * bits / 8;
  const uint16_t align    = (uint16_t)(numCh * bits / 8);
  const uint32_t fmtSize  = 16;
  const uint16_t audioFmt = 1;
  uint32_t       riffSize = 36 + dataSize;

  f.write((const uint8_t*)"RIFF",      4);
  f.write((const uint8_t*)&riffSize,   4);
  f.write((const uint8_t*)"WAVE",      4);
  f.write((const uint8_t*)"fmt ",      4);
  f.write((const uint8_t*)&fmtSize,    4);
  f.write((const uint8_t*)&audioFmt,   2);
  f.write((const uint8_t*)&numCh,      2);
  f.write((const uint8_t*)&sr,         4);
  f.write((const uint8_t*)&byteRate,   4);
  f.write((const uint8_t*)&align,      2);
  f.write((const uint8_t*)&bits,       2);
  f.write((const uint8_t*)"data",      4);
  f.write((const uint8_t*)&dataSize,   4);
}

// ── File Index ────────────────────────────────────────────────────────────
static uint32_t findNextWavFileIndex()
{
  char name[32];
  for (uint32_t i = 1; i <= 9999; i++)
  {
    snprintf(name, sizeof(name), "/VIB%04u.wav", i);
    if (!SD_MMC.exists(name)) return i;
  }
  return 9999;
}

static bool openNewWavFile()
{
  if (wavFile) wavFile.close();

  uint32_t idx = findNextWavFileIndex();
  snprintf(wavFileName, sizeof(wavFileName), "/VIB%04u.wav", idx);
  wavFile = SD_MMC.open(wavFileName, FILE_WRITE);

  if (!wavFile)
  {
    wavLog("[WAV] Cannot open: %s\n", wavFileName);
    return false;
  }

  wavDataBytes = 0;
  writeWavHeader(wavFile, 0);  // placeholder: 종료 시 업데이트
  wavFile.flush();
  wavLog("[WAV] File: %s\n", wavFileName);
  return true;
}

static void closeWavFile()
{
  if (!wavFile) return;

  uint32_t riffSize = 36 + wavDataBytes;
  wavFile.seek(4);  wavFile.write((const uint8_t*)&riffSize,    4);
  wavFile.seek(40); wavFile.write((const uint8_t*)&wavDataBytes, 4);
  wavFile.flush();
  wavFile.close();

  float sec = (float)wavDataBytes / (float)(WAV_SAMPLE_RATE * 2);
  wavLog("[WAV] Saved %s  (%.2f sec, %u bytes)\n", wavFileName, sec, wavDataBytes);
}

// ── Start / Stop ──────────────────────────────────────────────────────────
static void wavStartRecording(bool sd)
{
  if (!sd) { Serial.println("[WAV] SD not ready"); return; }
  if (!openNewWavFile()) return;

  sdDebugOpen(wavFileName);

  // WiFi는 계속 ON 유지 (방수 기구물 — 물리 버튼 접근 불가, 웹 전용 제어)
  // ADC1 (GPIO1)은 ESP32-S3에서 WiFi와 별도 하드웨어 블록 → 간섭 최소
  // TX Power(10dBm) + WIFI_PS_MIN_MODEM은 webServerBegin()에서 전역 설정 — 여기서 재설정 불필요

  wavFillBuf  = 0;
  wavFillPos  = 0;
  wavFlushReq = false;
  wavISRCount = 0;
  wavStartMs  = millis();

  // 포화 진단 통계 초기화
  wavRawMin    = 4095;
  wavRawMax    = 0;
  wavClipCount = 0;

  // 필터 상태 초기화
  hpFilter.w1 = hpFilter.w2 = 0.0f;
  lpState = 0.0f;
  rngState = (uint32_t)esp_timer_get_time();

  wavState = WAV_RECORDING;
  ledSetState(LED_LOGGING);
  webSetRecording(true);  // 웹 UI → STOP 버튼으로 전환

  esp_timer_start_periodic(wavEspTimer, 1000000UL / WAV_SAMPLE_RATE);
  wavLog("[WAV] Recording started -> %s  (press STOP to finish)\n", wavFileName);
}

static void wavStopRecording(bool sd)
{
  esp_timer_stop(wavEspTimer);
  wavState = WAV_SAVING;
  ledSetState(LED_BOOTING);

  webSetRecording(false);   // 웹 UI → REC 버튼으로 복귀
  // 녹음 종료 후 대기 상태로 복귀 — TX/PS 설정은 webServerBegin()에서 설정한 값 유지
  // (10 dBm + WIFI_PS_MIN_MODEM → WIFI_PS_NONE으로 되돌리지 않음)

  if (!sd || !wavFile)
  {
    wavState = WAV_STANDBY;
    sdDebugClose();
    return;
  }

  // 완료된 더블 버퍼 플러시
  if (wavFlushReq)
  {
    wavFile.write((const uint8_t*)wavBuf[wavFlushBuf], WAV_BUF_SAMPLES * 2);
    wavDataBytes += (uint32_t)(WAV_BUF_SAMPLES * 2);
    wavFlushReq = false;
  }

  // 부분 채워진 버퍼 플러시
  if (wavFillPos > 0)
  {
    wavFile.write((const uint8_t*)wavBuf[wavFillBuf], wavFillPos * 2);
    wavDataBytes += (uint32_t)(wavFillPos * 2);
  }

  closeWavFile();

  uint32_t totalSamples = wavDataBytes / 2;
  float    totalSec     = (float)wavDataBytes / (float)(WAV_SAMPLE_RATE * 2);

  wavLog("[WAV] Total: %.2f sec (%u samples)\n", totalSec, totalSamples);

  // ── 포화(클리핑) 진단 출력 ───────────────────────────────────────────────
  {
    uint32_t totalAdc  = wavISRCount;
    float    clipPct   = totalAdc ? (100.0f * (float)wavClipCount / (float)totalAdc) : 0.0f;
    int      pcmMin    = (wavRawMin - 2048) << 4;   // ADC→PCM 환산 (참고)
    int      pcmMax    = (wavRawMax - 2048) << 4;
    wavLog("[ADC] Raw range : min=%d  max=%d  (valid 0~4095)\n", wavRawMin, wavRawMax);
    wavLog("[ADC] PCM range : min=%d  max=%d  (16bit ±32768)\n", pcmMin, pcmMax);
    wavLog("[ADC] Clipping  : %.2f%%  (%u / %u samples near rail +/-%d)\n",
      clipPct, wavClipCount, totalAdc, WAV_CLIP_MARGIN);
    if (clipPct >= 1.0f)
      wavLog("[ADC] !! Saturation -- reduce preamp gain (intensity unreliable)\n");
    else if (wavRawMin <= 50 || wavRawMax >= 4045)
      wavLog("[ADC] ! Near full-scale -- insufficient headroom (reduce gain)\n");
  }

  analyzeFFT(totalSamples);   // KissFFT 주파수/강도 분석 (Intensity 출력)

#if WAV_NORMALIZE
  normalizeWavFile(totalSamples);
#endif

  wavLog("[WAV] Standby. Press REC in browser to start recording.\n");
  sdDebugClose();
  wavState = WAV_STANDBY;
}

// ── Public API ────────────────────────────────────────────────────────────
void wavRecorderInit()
{
  fftReal = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  fftImag = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!fftReal || !fftImag)
    Serial.println("[FFT] PSRAM alloc failed!");
  else
    Serial.printf("[FFT] PSRAM alloc OK: %u KB\n",
      (uint32_t)(FFT_SIZE * sizeof(float) * 2 / 1024));

  analogSetPinAttenuation(WAV_ADC_PIN, ADC_11db);
  pinMode(WAV_ADC_PIN,     INPUT);
  // 물리 버튼 핀은 비상 디버그용으로만 초기화
  pinMode(WAV_TRIGGER_PIN, INPUT_PULLUP);

  esp_timer_create_args_t cfg = {
    .callback              = wavTimerCb,
    .arg                   = NULL,
    .dispatch_method       = ESP_TIMER_TASK,
    .name                  = "wavADC",
    .skip_unhandled_events = true
  };
  esp_timer_create(&cfg, &wavEspTimer);

  Serial.printf("[WAV] MEMS Mic  ADC=GPIO%d  SR=%dHz  MaxRec=%dsec\n",
    WAV_ADC_PIN, WAV_SAMPLE_RATE, WAV_MAX_RECORD_SEC);
  Serial.println("[WAV] Filter: HP 80Hz + LP 3600Hz @ 8000Hz (MEMS mic)");
}

void wavLoop(bool sd, unsigned long now)
{
  // ── SD 재삽입 감지 (대기 상태에서 3초마다 재마운트 시도) ──────────────
  if (wavState == WAV_STANDBY && !sdReady)
  {
    static unsigned long lastRemount = 0;
    if (now - lastRemount >= 3000)
    {
      lastRemount = now;
      Serial.println("[SD] Not ready — attempting remount...");
      if (sdRemount())
      {
        sdReady = true;
        ledSetState(LED_IDLE);
      }
    }
  }

  // ── 웹 REC 버튼 ──────────────────────────────────────────────────────────
  if (wavState == WAV_STANDBY && webRecordConsumed())
    wavStartRecording(sd);

  // ── 웹 STOP 버튼 ─────────────────────────────────────────────────────────
  if (wavState == WAV_RECORDING && webStopConsumed())
    wavStopRecording(sd);

  // ── 비상 물리 버튼 (방수 기구물에서는 정상 미사용, 디버그 전용) ──────────
#if 0
  static bool          lastBtn     = HIGH;
  static unsigned long lastBtnTime = 0;
  bool btn = (bool)digitalRead(WAV_TRIGGER_PIN);
  if (btn != lastBtn && (now - lastBtnTime) > 50)
  {
    lastBtnTime = now;
    lastBtn     = btn;
    if (btn == LOW && wavState == WAV_STANDBY)   wavStartRecording(sd);
    if (btn == LOW && wavState == WAV_RECORDING) wavStopRecording(sd);
  }
#endif

  // ── 녹음 중 처리 ─────────────────────────────────────────────────────────
  if (wavState == WAV_RECORDING)
  {
    // 더블버퍼 → SD 플러시
    if (wavFlushReq && wavFile)
    {
      uint8_t bi = wavFlushBuf;
      wavFile.write((const uint8_t*)wavBuf[bi], WAV_BUF_SAMPLES * 2);
      wavDataBytes += (uint32_t)(WAV_BUF_SAMPLES * 2);
      wavFlushReq = false;
    }

    // 경과 시간 계산 및 웹 UI 갱신 (500ms마다)
    static unsigned long lastPrint = 0;
    if (now - lastPrint >= 500)
    {
      lastPrint = now;
      float    elapsed    = (float)wavISRCount / (float)WAV_SAMPLE_RATE;
      uint32_t elapsedSec = (uint32_t)elapsed;
      webSetElapsed(elapsedSec);  // 웹 페이지 경과 시간 갱신

      // SD 여유 공간 체크 (5초마다)
      static unsigned long lastSpaceCheck = 0;
      if (now - lastSpaceCheck >= 5000)
      {
        lastSpaceCheck = now;
        uint64_t freeMB = (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL);
        if (freeMB < (uint64_t)WAV_SD_RESERVE_MB)
        {
          wavLog("[WAV] SD free space low (%llu MB) — auto stop\n", freeMB);
          wavStopRecording(sd);
          return;
        }
      }

      // WAV 헤더 주기적 갱신 (5초마다) — 비정상 종료(전원차단/크래시) 대비
      // closeWavFile()이 호출되지 않더라도 마지막 갱신 시점까지 데이터 복구 가능
      static unsigned long lastHeaderUpdate = 0;
      if (wavFile && (now - lastHeaderUpdate >= 5000))
      {
        lastHeaderUpdate = now;
        uint32_t riffSize = 36 + wavDataBytes;
        wavFile.seek(4);  wavFile.write((const uint8_t*)&riffSize,    4);
        wavFile.seek(40); wavFile.write((const uint8_t*)&wavDataBytes, 4);
        wavFile.seek(wavFile.size());  // 파일 끝으로 복귀 (다음 write 위치 유지)
      }

      wavLog("  %.1f sec  (%u bytes)  SD free: %llu MB\n",
        elapsed,
        wavDataBytes,
        (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL));
    }

    // 안전 최대 시간 초과 → 자동 종료
    if (wavISRCount >= (uint32_t)WAV_MAX_RECORD_SEC * WAV_SAMPLE_RATE)
    {
      wavLog("[WAV] Max record time (%d sec) reached — auto stop\n", WAV_MAX_RECORD_SEC);
      wavStopRecording(sd);
    }
  }

  // ── DeepSleep: SLEEP_IDLE_SEC 동안 대기 상태 유지 시 슬립 진입 ────────────
#if USE_DEEPSLEEP
  {
    static bool          inStandby  = false;
    static unsigned long standbyMs  = 0;

    if (wavState == WAV_STANDBY)
    {
      if (!inStandby) { inStandby = true; standbyMs = now; }

      if ((now - standbyMs) >= (unsigned long)SLEEP_IDLE_SEC * 1000UL)
      {
        wavLog("[SLEEP] Standby for %d sec — attempting DeepSleep\n", SLEEP_IDLE_SEC);
        deepSleepEnter();
        // deepSleepEnter()가 반환된 경우(리드스위치 활성) → 타이머 리셋 후 재시도
        standbyMs = now;
      }
    }
    else
    {
      inStandby = false;  // 녹음/저장 중이면 타이머 리셋
    }
  }
#endif
}

#endif // SENSOR_TYPE == 2
