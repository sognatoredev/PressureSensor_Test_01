#pragma once
#include "config.h"

#if SENSOR_TYPE == 2

// WAV 녹음기 초기화 (esp_timer 생성, ADC/버튼 핀 설정)
void wavRecorderInit();

// 메인 루프에서 매 루프마다 호출: 버튼 감지 → 녹음 시작/진행/종료
void wavLoop(bool sd, unsigned long now);

#endif // SENSOR_TYPE == 2
