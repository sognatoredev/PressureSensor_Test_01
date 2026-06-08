#include "led_indicator.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel rgbLed(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
static LedState ledState = LED_BOOTING;

// 무드등: HSV 색상환을 순환하는 주기 (ms). 값이 클수록 천천히 변함
#define MOOD_CYCLE_MS   8000
// 무드등 최대 밝기 (0~255). WS2812B는 저밝기에서 색감이 더 좋음
#define MOOD_BRIGHTNESS   50

// HSV → RGB 변환 후 NeoPixel Color 반환
// h: 0~359, s: 0~255, v: 0~255
static uint32_t hsvToColor(uint16_t h, uint8_t s, uint8_t v)
{
  uint8_t r, g, b;
  if (s == 0) { r = g = b = v; }
  else
  {
    uint8_t  region  = h / 60;
    uint16_t rem     = (h % 60) * 255 / 60;
    uint8_t  p = (uint16_t)v * (255 - s) / 255;
    uint8_t  q = (uint16_t)v * (255 - ((uint16_t)s * rem / 255)) / 255;
    uint8_t  t_val = (uint16_t)v * (255 - ((uint16_t)s * (255 - rem) / 255)) / 255;
    switch (region)
    {
      case 0: r=v;     g=t_val; b=p;     break;
      case 1: r=q;     g=v;     b=p;     break;
      case 2: r=p;     g=v;     b=t_val; break;
      case 3: r=p;     g=q;     b=v;     break;
      case 4: r=t_val; g=p;     b=v;     break;
      default:r=v;     g=p;     b=q;     break;
    }
  }
  return rgbLed.Color(r, g, b);
}

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

void ledOff()
{
  rgbLed.clear();
  rgbLed.show();
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

    case LED_IDLE:
    {
      // 색상환(0~359°)을 MOOD_CYCLE_MS 주기로 부드럽게 순환
      uint16_t hue = (uint16_t)((t % MOOD_CYCLE_MS) * 360UL / MOOD_CYCLE_MS);
      color = hsvToColor(hue, 255, MOOD_BRIGHTNESS);
      rgbLed.setPixelColor(0, color);
      rgbLed.show();
      return;
    }

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

    case LED_SLEEPING:
      // 주황색 느린 깜빡임 (1.5 s 주기) — DeepSleep 직전 시각적 알림
      color = rgbLed.Color(80, 30, 0);   blink=true; interval=1500; break;
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
