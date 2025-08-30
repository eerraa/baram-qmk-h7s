#ifndef POLLING_RATE_H
#define POLLING_RATE_H

#include "eeprom.h" // eeprom.h를 포함하여 함수 선언을 가져옵니다.

// 폴링레이트 모드 정의
#define POLLING_RATE_8K 0
#define POLLING_RATE_4K 1
#define POLLING_RATE_1K 2

// --- 함수 선언 (프로토타입) ---
void polling_rate_init(void);
void polling_rate_set(uint8_t mode);
uint8_t polling_rate_get(void);
uint8_t get_bInterval_for_current_mode(void);


#endif /* POLLING_RATE_H */