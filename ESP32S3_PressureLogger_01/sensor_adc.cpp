#include "sensor_adc.h"

#if SENSOR_TYPE == 1
#include <Arduino.h>

int readAdcRaw()
{
    return analogRead(ADC_PRESSURE_PIN);
}

#endif // SENSOR_TYPE == 1
