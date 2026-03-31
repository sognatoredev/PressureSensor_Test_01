#include "wav_recorder.h"

#if SENSOR_TYPE == 2

#include <Arduino.h>
#include <SD_MMC.h>
#include "esp_timer.h"
#include "led_indicator.h"
#include <arduinoFFT.h>

// ── FFT Analysis ──────────────────────────────────────────────────────────
// 실제 샘플: 24000 (3sec × 8kHz), 2의 거듭제곱으로 제로패딩 → 32768 (2^15)
#define FFT_SIZE          32768  // 2^15, zero-padded, freq resolution ~0.244 Hz
#define FFT_CAPTURE_SIZE  (WAV_RECORD_SECONDS * WAV_SAMPLE_RATE)  // 24000 samples

// FFT 연산용: PSRAM 동적 할당
// analyzeFFT()는 SD 읽기/FFT 모두 Core 1(loop)에서 실행 → 크로스 코어 캐시 문제 없음
static float* fftReal = nullptr;
static float* fftImag = nullptr;

// ── WAV State Machine ─────────────────────────────────────────────────────
enum WavState { WAV_STANDBY, WAV_RECORDING, WAV_SAVING };
static WavState wavState = WAV_STANDBY;

// ── Double buffer (4096 × 2 × 2 = 16 KB in internal SRAM) ────────────────
static int16_t           wavBuf[2][WAV_BUF_SAMPLES];
static volatile uint8_t  wavFillBuf  = 0;
static volatile uint16_t wavFillPos  = 0;
static volatile uint8_t  wavFlushBuf = 0;
static volatile bool     wavFlushReq = false;
static volatile uint32_t wavISRCount = 0;

// ── File tracking ──────────────────────────────────────────────────────────
static File     wavFile;
static char     wavFileName[32];
static uint32_t wavDataBytes = 0;

// ── esp_timer handle ───────────────────────────────────────────────────────
static esp_timer_handle_t wavEspTimer = NULL;

// Timer callback — runs in esp_timer task context (NOT an ISR)
static void wavTimerCb(void *arg)
{
  int raw = analogRead(WAV_ADC_PIN);
  int16_t sample = (int16_t)((raw - 2048) << 4);

  wavBuf[wavFillBuf][wavFillPos] = sample;
  wavISRCount++;

  if (++wavFillPos >= WAV_BUF_SAMPLES)
  {
    wavFlushBuf = wavFillBuf;
    wavFlushReq = true;
    wavFillBuf ^= 1;
    wavFillPos  = 0;
  }
}

// ── FFT Peak Frequency Analysis ───────────────────────────────────────────
// SD 파일에서 직접 읽기 → Core 1에서만 접근 → 코어 간 캐시 문제 없음
// 윈도우는 실제 신호 구간(FFT_CAPTURE_SIZE)에만 수동 적용 후 제로패딩
static void analyzeFFT()
{
  if (!fftReal || !fftImag)
  {
    Serial.println("[FFT] Buffer not ready, skipping.");
    return;
  }

  // WAV 파일 열기 (헤더 44바이트 스킵 후 PCM 샘플 읽기)
  File f = SD_MMC.open(wavFileName, FILE_READ);
  if (!f)
  {
    Serial.printf("[FFT] Cannot open %s\n", wavFileName);
    return;
  }
  f.seek(44);

  // SD → fftReal (int16_t → float), DC 평균 동시 계산
  float mean = 0.0f;
  for (uint32_t i = 0; i < FFT_CAPTURE_SIZE; i++)
  {
    int16_t s = 0;
    f.read((uint8_t*)&s, 2);
    fftReal[i] = (float)s;
    mean += fftReal[i];
  }
  f.close();
  mean /= (float)FFT_CAPTURE_SIZE;

  // DC 제거 + Hamming 윈도우를 실제 신호 구간에만 적용
  for (uint32_t i = 0; i < FFT_CAPTURE_SIZE; i++)
  {
    fftReal[i] -= mean;
    fftReal[i] *= 0.54f - 0.46f * cosf(TWO_PI * i / (float)(FFT_CAPTURE_SIZE - 1));
  }

  // 제로패딩 (FFT_CAPTURE_SIZE ~ FFT_SIZE-1)
  memset(&fftReal[FFT_CAPTURE_SIZE], 0,
    (FFT_SIZE - FFT_CAPTURE_SIZE) * sizeof(float));
  memset(fftImag, 0, FFT_SIZE * sizeof(float));

  // FFT 계산 (windowing은 이미 수동 적용했으므로 호출 안 함)
  ArduinoFFT<float> FFT(fftReal, fftImag, FFT_SIZE, (float)WAV_SAMPLE_RATE);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // majorPeak(): 내장 이차보간으로 bin 경계 사이 정확한 주파수 계산
  float peakFreq = FFT.majorPeak();

  Serial.println("─────────────────────────────────");
  Serial.printf("[FFT] Signal   : %u samples (%.3f sec)\n",
    FFT_CAPTURE_SIZE, (float)FFT_CAPTURE_SIZE / WAV_SAMPLE_RATE);
  Serial.printf("[FFT] Zero-pad : %u → %u\n", FFT_CAPTURE_SIZE, FFT_SIZE);
  Serial.printf("[FFT] Freq Resolution : %.3f Hz/bin\n",
    (float)WAV_SAMPLE_RATE / (float)FFT_SIZE);
  Serial.printf("[FFT] DC Offset removed : %.1f\n", mean);
  Serial.printf("[FFT] Peak Freq : %.2f Hz\n", peakFreq);
  Serial.println("─────────────────────────────────");
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

// ── File Open / Close ─────────────────────────────────────────────────────
static uint32_t findNextWavFileIndex()
{
  char name[32];

  for (uint32_t i = 1; i <= 9999; i++)
  {
    snprintf(name, sizeof(name), "/VIB%04u.wav", i);
    if (!SD_MMC.exists(name))
    {
      return i;
    }
  }
  return 9999;
}

static bool openNewWavFile()
{
  uint32_t idx = 0;

  if(wavFile)
  {
    wavFile.close();
  }

  idx = findNextWavFileIndex();
  snprintf(wavFileName, sizeof(wavFileName), "/VIB%04u.wav", idx);
  wavFile = SD_MMC.open(wavFileName, FILE_WRITE);
  
  if (!wavFile)
  {
    Serial.printf("[WAV] Cannot open: %s\n", wavFileName);
    return false;
  }

  wavDataBytes = 0;
  writeWavHeader(wavFile, 0);   // placeholder; sizes updated on close
  wavFile.flush();
  Serial.printf("[WAV] File: %s\n", wavFileName);
  return true;
}

static void closeWavFile()
{
  int32_t riffSize = 0;
  float sec = 0.0;

  if (!wavFile)
  {
    return;
  }
  
  riffSize = 36 + wavDataBytes;

  wavFile.seek(4);  wavFile.write((const uint8_t*)&riffSize,    4);
  wavFile.seek(40); wavFile.write((const uint8_t*)&wavDataBytes, 4);
  wavFile.flush();
  wavFile.close();

  sec = (float)wavDataBytes / (float)(WAV_SAMPLE_RATE * 2);
  Serial.printf("[WAV] Saved %s  (%.2f sec, %u bytes)\n", wavFileName, sec, wavDataBytes);
}

// ── Start / Stop ──────────────────────────────────────────────────────────
static void wavStartRecording(bool sd)
{
  if (!sd) { Serial.println("[WAV] SD not ready"); return; }
  if (!openNewWavFile()) return;

  wavFillBuf  = 0;
  wavFillPos  = 0;
  wavFlushReq = false;
  wavISRCount = 0;
  wavState    = WAV_RECORDING;
  ledSetState(LED_LOGGING);

  esp_timer_start_periodic(wavEspTimer, 125);   // 8000 Hz = 125 μs
  Serial.printf("[WAV] Recording %d sec → %s\n", WAV_RECORD_SECONDS, wavFileName);
}

static void wavStopRecording(bool sd)
{
  esp_timer_stop(wavEspTimer);
  wavState = WAV_SAVING;

  if (!sd || !wavFile)
  {
    wavState = WAV_STANDBY;
    return;
  }

  const uint32_t maxBytes = (uint32_t)WAV_RECORD_SECONDS * WAV_SAMPLE_RATE * 2;

  // Flush full buffer that completed just before stop
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
  analyzeFFT();               // WAV 저장 완료 후 FFT 주파수 분석
  wavState = WAV_STANDBY;
  ledSetState(LED_BOOTING);   // white = standby
  Serial.println("[WAV] Standby. Press BOOT button to record.");
}

// ── Public API ────────────────────────────────────────────────────────────
void wavRecorderInit()
{
  // FFT 버퍼를 PSRAM에 동적 할당 (Core 1에서만 접근하므로 캐시 문제 없음)
  fftReal = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  fftImag = (float*)heap_caps_malloc(FFT_SIZE * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!fftReal || !fftImag)
    Serial.println("[FFT] PSRAM alloc failed!");
  else
    Serial.printf("[FFT] PSRAM alloc OK: %u KB\n",
      (uint32_t)(FFT_SIZE * sizeof(float) * 2 / 1024));

  analogSetPinAttenuation(WAV_ADC_PIN, ADC_11db);
  pinMode(WAV_ADC_PIN,     INPUT);
  pinMode(WAV_TRIGGER_PIN, INPUT_PULLUP);

  esp_timer_create_args_t cfg =
  {
    .callback             = wavTimerCb,
    .arg                  = NULL,
    .dispatch_method      = ESP_TIMER_TASK,
    .name                 = "wavADC",
    .skip_unhandled_events = true
  };
  esp_timer_create(&cfg, &wavEspTimer);

  Serial.printf("Piezo WAV  ADC=GPIO%d  Trigger=GPIO%d (BOOT)\n",
    WAV_ADC_PIN, WAV_TRIGGER_PIN);
}

void wavLoop(bool sd, unsigned long now)
{
  // Button detection (GPIO0 active LOW, 50 ms debounce)
  static bool          lastBtn     = HIGH;
  static unsigned long lastBtnTime = 0;
  bool btn = (bool)digitalRead(WAV_TRIGGER_PIN);

  if (btn != lastBtn && (now - lastBtnTime) > 50)
  {
    lastBtnTime = now;
    lastBtn     = btn;
    if (btn == LOW && wavState == WAV_STANDBY)
      wavStartRecording(sd);
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
      wavStopRecording(sd);
  }
}

#endif // SENSOR_TYPE == 2
