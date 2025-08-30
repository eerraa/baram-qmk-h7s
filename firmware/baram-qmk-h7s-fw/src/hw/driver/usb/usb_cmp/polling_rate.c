#include "polling_rate.h"
#include "port.h" // EECONFIG_USER_POLLING_RATE를 포함

// 현재 적용될 폴링레이트 모드를 저장할 전역 변수
static uint8_t current_polling_rate_mode = POLLING_RATE_8K;


// EEPROM에서 모드를 읽어오는 함수
void polling_rate_init(void) {
    uint8_t mode;
    // (uint32_t)로 명시적 타입 캐스팅을 하여 컴파일 경고를 제거합니다.
    uint32_t addr = (uint32_t)EECONFIG_USER_POLLING_RATE;

    if (eepromReadByte(addr, &mode) == true)
    {
        if (mode > POLLING_RATE_1K) { // 유효하지 않은 값이면 기본값으로 초기화
            mode = POLLING_RATE_8K;
            eepromWriteByte(addr, mode);
        }
    }
    else // EEPROM 읽기 실패 시 (초기 상태 등)
    {
        mode = POLLING_RATE_8K;
        eepromWriteByte(addr, mode);
    }
    current_polling_rate_mode = mode;
}

// EEPROM에 모드를 저장하는 함수
void polling_rate_set(uint8_t mode) {
    if (mode <= POLLING_RATE_1K) {
        logPrintf("[DEBUG] 2. polling_rate_set() called with mode %d.\n", mode);
        uint32_t addr = (uint32_t)EECONFIG_USER_POLLING_RATE;
        eepromWriteByte(addr, mode);
        logPrintf("[DEBUG] 3. eepromWriteByte() called.\n");
        current_polling_rate_mode = mode;
    }
}

// 현재 모드를 가져오는 함수
uint8_t polling_rate_get(void) {
    return current_polling_rate_mode;
}

// bInterval 값을 계산하는 함수
uint8_t get_bInterval_for_current_mode(void) {
    switch (current_polling_rate_mode) {
        case POLLING_RATE_4K:
            return 2; // 4KHz
        case POLLING_RATE_1K:
            return 4; // 1KHz
        case POLLING_RATE_8K:
        default:
            return 1; // 8KHz
    }
}