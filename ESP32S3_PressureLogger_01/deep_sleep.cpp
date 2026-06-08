#include "deep_sleep.h"

#if USE_DEEPSLEEP

#include <Arduino.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "led_indicator.h"

// ─────────────────────────────────────────────────────────────────────────────
// deepSleepInit
//   ∙ GPIO3: 내부 풀다운 활성, 슬립 중 RTC 도메인에서 유지
//   ∙ EXT0 Wake-Up: GPIO3 = SLEEP_WAKE_LEVEL (HIGH) 에서 깨어남
//   ∙ 이 함수는 부팅(초기 + DeepSleep 재기상)마다 반드시 호출해야 함
// ─────────────────────────────────────────────────────────────────────────────
void deepSleepInit()
{
  // Arduino 레벨: 일반 동작 중 풀다운 설정
  pinMode(SLEEP_TRIGGER_PIN, INPUT_PULLDOWN);

  // RTC 레벨: DeepSleep 중에도 풀다운이 유지되도록 RTC GPIO API로 명시 설정
  // (ESP32-S3 GPIO0~21은 RTC GPIO로 사용 가능)
  rtc_gpio_init((gpio_num_t)SLEEP_TRIGGER_PIN);
  rtc_gpio_set_direction((gpio_num_t)SLEEP_TRIGGER_PIN, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_en((gpio_num_t)SLEEP_TRIGGER_PIN);
  rtc_gpio_pullup_dis((gpio_num_t)SLEEP_TRIGGER_PIN);

  // Wake-Up 소스 등록: EXT0, GPIO3 = HIGH
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_TRIGGER_PIN, SLEEP_WAKE_LEVEL);

  Serial.printf("[SLEEP] Init  GPIO%d (pull-down)  wake-on=%s\n",
    SLEEP_TRIGGER_PIN, SLEEP_WAKE_LEVEL ? "HIGH" : "LOW");
}

// ─────────────────────────────────────────────────────────────────────────────
// deepSleepIsWakeup
//   이번 부팅이 EXT0(리드스위치 트리거)로 인한 Wake-Up인지 확인
// ─────────────────────────────────────────────────────────────────────────────
bool deepSleepIsWakeup()
{
  return esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0;
}

// ─────────────────────────────────────────────────────────────────────────────
// deepSleepReedActive
//   리드스위치가 현재 닫혀 있는지 확인 (자석 있음 = GPIO3 = Wake 레벨)
// ─────────────────────────────────────────────────────────────────────────────
bool deepSleepReedActive()
{
  return (digitalRead(SLEEP_TRIGGER_PIN) == SLEEP_WAKE_LEVEL);
}

// ─────────────────────────────────────────────────────────────────────────────
// deepSleepEnter
//   DeepSleep 진입 시퀀스:
//     1. 리드스위치 활성 여부 확인 (HIGH이면 즉시 재기상 → 진입 취소)
//     2. LED 슬립 표시 (주황 깜빡임 3초)
//     3. LED 소등 (WS2812B 전류 절감)
//     4. esp_deep_sleep_start() — 이후 반환 없음
//
//   소비 전류 (참고):
//     ESP32-S3 DeepSleep: ~7 µA
//     RTC 풀다운 유지:    ~73 µA (3.3V / 45kΩ) — 스위치 OPEN 시 0 µA
//     WS2812B OFF:         ~1 µA (데이터 LOW 유지)
// ─────────────────────────────────────────────────────────────────────────────
void deepSleepEnter()
{
  // 리드스위치가 활성 상태(HIGH)이면 슬립하지 않음
  // → 진입 즉시 재기상(wake) 될 수 있으므로 자석이 떼어진 후에만 진입
  if (deepSleepReedActive())
  {
    Serial.printf("[SLEEP] Reed GPIO%d = %s  →  sleep aborted (remove magnet first)\n",
      SLEEP_TRIGGER_PIN, SLEEP_WAKE_LEVEL ? "HIGH" : "LOW");
    return;
  }

  Serial.printf("[SLEEP] Entering DeepSleep  wake-pin=GPIO%d  level=%s\n",
    SLEEP_TRIGGER_PIN, SLEEP_WAKE_LEVEL ? "HIGH" : "LOW");
  Serial.println("[SLEEP] >>> Apply magnet to wake up <<<");
  Serial.flush();

  // LED: 슬립 표시 (~3 초)
  ledSetState(LED_SLEEPING);
  for (int i = 0; i < 20; i++) { ledUpdate(); delay(150); }

  // LED 완전 소등 (슬립 중 불필요한 전류 방지)
  ledOff();
  delay(50);

  // DeepSleep 진입 — 이후 코드 실행되지 않음
  esp_deep_sleep_start();
}

#endif // USE_DEEPSLEEP
