#pragma once
#include "config.h"

#if SENSOR_TYPE == 3

// NAU88C10YG 코덱 WAV 레코더 초기화
//   - Wire(I2C)로 코덱 레지스터 초기화
//   - I2S 마스터 RX 드라이버 설치
//   - FFT 버퍼 PSRAM 할당
void codecRecorderInit();

// 메인 루프에서 매 루프마다 호출:
//   버튼 감지 → 녹음 시작 / I2S 버퍼 플러시 / 종료 → 정규화 + FFT
void codecLoop(bool sd, unsigned long now);

#endif // SENSOR_TYPE == 3
