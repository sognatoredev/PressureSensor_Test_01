#pragma once
#include "config.h"

#if USE_DEEPSLEEP

/**
 * deepSleepInit()
 *   GPIO3을 내부 풀다운으로 설정하고 EXT0 Wake-Up 소스를 등록합니다.
 *   setup() 초반에 한 번 호출하십시오 — DeepSleep에서 깨어난 재부팅 시에도 반드시 호출해야 합니다.
 *
 * deepSleepIsWakeup()
 *   이번 부팅이 EXT0(리드스위치) Wake-Up으로 인한 것이면 true를 반환합니다.
 *   setup() 내 deepSleepInit() 호출 후 언제든 사용 가능합니다.
 *
 * deepSleepReedActive()
 *   현재 GPIO3이 Wake-Up 레벨(HIGH)이면 true를 반환합니다.
 *   리드스위치가 닫혀 있는(자석 있음) 상태를 의미합니다.
 *
 * deepSleepEnter()
 *   DeepSleep에 진입합니다 — 정상적으로 호출되면 반환되지 않습니다.
 *   리드스위치가 활성 중(GPIO3 = HIGH)이면 즉시 재기상을 방지하기 위해
 *   진입하지 않고 반환합니다.
 */
void deepSleepInit();
bool deepSleepIsWakeup();
bool deepSleepReedActive();
void deepSleepEnter();

#endif // USE_DEEPSLEEP
