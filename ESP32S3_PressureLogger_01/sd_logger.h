#pragma once
#include <stdint.h>
#include "config.h"

extern bool sdReady;

// SD_MMC 초기화 (핀 설정, begin)
bool sdInit();

// 새 CSV 파일 열기 (LOG0001.csv, LOG0002.csv, ...)
bool openNewLogFile();

#if SENSOR_TYPE == 1
// ADC 압력센서 데이터 한 행 기록
void sdWriteADC(unsigned long ts, const char *dt, int raw, float rawAvg);
#endif

#if SENSOR_TYPE == 0
// I2C WNK 센서 데이터 한 행 기록
void sdWriteI2C(unsigned long ts, const char *dt,
                float pressure, float pressureAvg,
                float temp,     float tempAvg);
#endif
