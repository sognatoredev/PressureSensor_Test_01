#pragma once

enum LedState
{
  LED_BOOTING,              // 흰색 점등          — 부팅 중
  LED_IDLE,                 // 무드등 색상 순환    — 정상 대기 / WAV 대기
  LED_WIFI_CONNECTING,      // 파란색 빠른 깜빡임 — WiFi/NTP 연결 중
  LED_LOGGING,              // 초록색 깜빡임       — 정상 로깅 / WAV 녹음 중
  LED_SD_ERROR,             // 빨간색 빠른 깜빡임 — SD 카드 오류
  LED_SENSOR_ERROR,         // 마젠타 빠른 깜빡임 — 센서 미감지
  LED_SENSOR_DISCONNECTED,  // 빨간색 점등         — 센서 연결 끊김
  LED_SLEEPING              // 주황색 천천히 깜빡임 — DeepSleep 진입 중
};

void ledBegin();
void ledSetState(LedState s);
void ledUpdate();
void ledOff();              // LED 즉시 소등 (DeepSleep 직전 호출)
