#include "filter.h"

void filterInit(MovingAvgFilter_t *f)
{
  for (int i = 0; i < FILTER_SIZE; i++) f->buffer[i] = 0.0f;
  f->index = 0; f->count = 0; f->sum = 0.0f;
}

float filterUpdate(MovingAvgFilter_t *f, float v)
{
  if (f->count >= FILTER_SIZE) f->sum -= f->buffer[f->index]; else f->count++;
  f->buffer[f->index] = v;
  f->sum += v;
  if (++f->index >= FILTER_SIZE) f->index = 0;
  return f->sum / (float)f->count;
}
