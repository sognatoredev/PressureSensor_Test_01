#pragma once
#include "config.h"

#if USE_LTE

/**
 * lte_simcom.h — SIMCOM7000 LTE Cat.M1 드라이버
 *
 * 사용 핀:
 *   LTE_TX_PIN (GPIO17) : ESP32 TX → SIMCOM7000 RXD
 *   LTE_RX_PIN (GPIO18) : ESP32 RX ← SIMCOM7000 TXD
 *
 * 사용 UART: HardwareSerial UART1 (Serial1)
 */

// 모뎀 초기화 및 네트워크 등록 (setup()에서 1회 호출)
//   반환: true = 모뎀 응답 + SIM 정상 + 네트워크 등록 완료
bool lteInit();

// HTTP GET 요청
//   path      : URL 경로 (예: "/")
//   respBuf   : 응답 본문 저장 버퍼 (NULL 가능)
//   respBufLen: 버퍼 크기
//   반환: true = HTTP 2xx 성공
bool lteHttpGet(const char* path, char* respBuf, int respBufLen);

// ── AT 패스스루 (LTE_PASSTHROUGH=1 전용) ────────────────────────────────────
// ltePassthroughBegin() : setup()에서 1회 호출 — UART 초기화만 수행
// ltePassthroughLoop()  : loop()에서 반복 호출 — Serial ↔ SIM7000 브리지
void ltePassthroughBegin();
void ltePassthroughLoop();

#endif // USE_LTE
