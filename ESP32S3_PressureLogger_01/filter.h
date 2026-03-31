#pragma once
#include <stdint.h>
#include "config.h"

typedef struct {
  float   buffer[FILTER_SIZE];
  uint8_t index;
  uint8_t count;
  float   sum;
} MovingAvgFilter_t;

void  filterInit(MovingAvgFilter_t *f);
float filterUpdate(MovingAvgFilter_t *f, float v);
