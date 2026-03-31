#include "sensor_i2c.h"

#if SENSOR_TYPE == 0
#include <Wire.h>

bool readSensor24bit(uint8_t reg, int32_t *out)
{
  Wire.beginTransmission(SENSOR_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)SENSOR_I2C_ADDR, (uint8_t)3) != 3) return false;
  uint32_t raw = ((uint32_t)Wire.read() << 16)
               | ((uint32_t)Wire.read() << 8)
               |  (uint32_t)Wire.read();
  *out = (raw & 0x800000) ? (int32_t)raw - 16777216 : (int32_t)raw;
  return true;
}

float calcPressure(int32_t raw)
{
  float v = 3.3f * (float)raw / 8388608.0f;
  v = (PRESSURE_RANGE * (v - 0.5f) / 2.0f * KPA_TO_MBAR) + PRESSURE_OFFSET;
  return (v < 0.0f) ? 0.0f : v;
}

float calcTemperature(int32_t raw) { return 25.0f + (float)raw / 65536.0f; }

#endif // SENSOR_TYPE == 0
