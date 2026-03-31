#pragma once
#include <stddef.h>

// WiFi 연결 후 NTP 동기화, 이후 WiFi 종료
void syncNTP();

// 현재 시간 문자열 반환 (NTP 동기화 여부에 따라 절대/상대 시간)
void getTimeString(char *buf, size_t len);
