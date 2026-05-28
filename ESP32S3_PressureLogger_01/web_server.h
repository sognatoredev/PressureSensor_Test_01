#pragma once
#include <stdint.h>
#include "config.h"

#if SENSOR_TYPE == 2 || SENSOR_TYPE == 3

// WiFi 접속 + mDNS + 웹서버 시작
void webServerBegin();

// 웹서버 중지 + WiFi OFF
// ※ 가변 길이 녹음 모드에서는 녹음 중에도 WiFi를 유지하므로 호출하지 않음
void webServerStop();

// 메인 루프에서 호출 (클라이언트 요청 처리)
void webServerLoop();

// HTML REC 버튼이 눌렸는지 확인 (플래그 소비 — 한 번만 true 반환)
bool webRecordConsumed();

// HTML STOP 버튼이 눌렸는지 확인 (플래그 소비 — 한 번만 true 반환)
bool webStopConsumed();

// 녹음 상태를 웹 UI에 반영 (true = 녹음 중 → STOP 버튼, false = 대기 → REC 버튼)
void webSetRecording(bool isRecording);

// 현재 녹음 경과 시간(초)을 웹 UI 표시용으로 설정
void webSetElapsed(uint32_t seconds);

#endif
