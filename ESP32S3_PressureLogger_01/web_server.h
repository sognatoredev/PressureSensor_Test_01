#pragma once
#include "config.h"

#if SENSOR_TYPE == 2 || SENSOR_TYPE == 3

// WiFi 접속 + mDNS + 웹서버 시작
void webServerBegin();

// 웹서버 중지 + WiFi OFF (녹음 전 호출 → ADC 간섭 방지)
void webServerStop();

// 메인 루프에서 호출 (클라이언트 요청 처리)
void webServerLoop();

// HTML 녹음 버튼이 눌렸는지 확인 (플래그 소비 — 한 번만 true 반환)
bool webRecordConsumed();

#endif
