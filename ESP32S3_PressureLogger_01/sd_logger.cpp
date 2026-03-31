#include "sd_logger.h"
#include <SD_MMC.h>

bool sdReady = false;

static File     logFile;
static char     logFileName[32];
static uint32_t rowCount  = 0;
static uint32_t flushCnt  = 0;

static uint32_t findNextFileIndex()
{
  for (uint32_t i = 1; i <= 9999; i++) {
    char name[32];
    snprintf(name, sizeof(name), "/LOG%04u.csv", i);
    if (!SD_MMC.exists(name)) return i;
  }
  return 9999;
}

bool openNewLogFile()
{
  if (logFile) logFile.close();
  uint32_t idx = findNextFileIndex();
  snprintf(logFileName, sizeof(logFileName), "/LOG%04u.csv", idx);
  logFile = SD_MMC.open(logFileName, FILE_WRITE);
  if (!logFile) { Serial.printf("Failed to open: %s\n", logFileName); return false; }

#if SENSOR_TYPE == 1
  logFile.println("timestamp_ms,datetime,adc_raw,adc_raw_avg");
#else
  logFile.println("timestamp_ms,datetime,pressure_mbar,pressure_avg_mbar,temperature_c,temperature_avg_c");
#endif

  logFile.flush();
  rowCount = 0;
  Serial.printf("New file: %s\n", logFileName);
  return true;
}

bool sdInit()
{
  SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("[ERROR] SD init failed! Check wiring / FAT32.");
    sdReady = false;
    return false;
  }
  uint64_t sz = SD_MMC.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("SD ready  %llu MB\n", sz);
  sdReady = true;
  return true;
}

// ── 파일 교체가 필요한 경우 체크 및 처리 ──────────────────────────────────
static bool rotateIfNeeded()
{
  if (MAX_ROWS_PER_FILE > 0 && rowCount >= (uint32_t)MAX_ROWS_PER_FILE) {
    logFile.flush(); logFile.close();
    Serial.printf("[Rotate] %s (%u rows)\n", logFileName, rowCount);
    if (!openNewLogFile()) { sdReady = false; return false; }
  }
  return true;
}

static void afterWrite()
{
  rowCount++;
  if (++flushCnt >= (uint32_t)BUF_FLUSH_COUNT) { logFile.flush(); flushCnt = 0; }
}

#if SENSOR_TYPE == 1
void sdWriteADC(unsigned long ts, const char *dt, int raw, float rawAvg)
{
  if (!sdReady || !logFile) return;
  if (!rotateIfNeeded()) return;
  logFile.printf("%lu,%s,%d,%.1f\n", ts, dt, raw, rawAvg);
  afterWrite();
}
#endif

#if SENSOR_TYPE == 0
void sdWriteI2C(unsigned long ts, const char *dt,
                float pressure, float pressureAvg,
                float temp,     float tempAvg)
{
  if (!sdReady || !logFile) return;
  if (!rotateIfNeeded()) return;
  logFile.printf("%lu,%s,%.2f,%.2f,%.2f,%.2f\n",
    ts, dt, pressure, pressureAvg, temp, tempAvg);
  afterWrite();
}
#endif
