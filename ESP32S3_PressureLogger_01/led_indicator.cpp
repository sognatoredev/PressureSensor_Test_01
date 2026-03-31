#include "led_indicator.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
static LedState ledState = LED_BOOTING;

void ledBegin()
{
  rgbLed.begin();
  rgbLed.clear();
  rgbLed.show();
}

void ledSetState(LedState s)
{
  ledState = s;
}

void ledUpdate()
{
  static unsigned long lastToggle = 0;
  static bool          blinkOn    = true;
  static LedState      lastState  = LED_BOOTING;
  unsigned long t = millis();

  if (ledState != lastState) { lastState = ledState; blinkOn = true; lastToggle = t; }

  uint32_t      color    = 0;
  unsigned long interval = 500;
  bool          blink    = false;

  switch (ledState)
  {
    case LED_BOOTING:
      color = rgbLed.Color(30, 30, 30);                              break;
    case LED_WIFI_CONNECTING:
      color = rgbLed.Color(0, 0, 60);   blink=true; interval=300;   break;
    case LED_LOGGING:
      color = rgbLed.Color(0, 60, 0);   blink=true; interval=500;   break;
    case LED_SD_ERROR:
      color = rgbLed.Color(60, 0, 0);   blink=true; interval=150;   break;
    case LED_SENSOR_ERROR:
      color = rgbLed.Color(60, 0, 60);  blink=true; interval=150;   break;
    case LED_SENSOR_DISCONNECTED:
      color = rgbLed.Color(60, 0, 0);                                break;
  }

  if (blink)
  {
    if (t - lastToggle >= interval) { lastToggle = t; blinkOn = !blinkOn; }
    rgbLed.setPixelColor(0, blinkOn ? color : 0);
  }
  else
  {
    rgbLed.setPixelColor(0, color);
  }
  rgbLed.show();
}
