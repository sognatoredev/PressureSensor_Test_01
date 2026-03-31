#pragma once
#include "config.h"

#if SENSOR_TYPE == 0
#include <stdint.h>
#include <stdbool.h>

bool  readSensor24bit(uint8_t reg, int32_t *out);
float calcPressure(int32_t raw);
float calcTemperature(int32_t raw);

#endif // SENSOR_TYPE == 0
