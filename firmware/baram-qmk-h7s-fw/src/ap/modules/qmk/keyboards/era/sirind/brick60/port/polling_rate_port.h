#pragma once
#include "quantum.h" // QK_USER를 사용하기 위해 포함

// VIA JSON에 정의된 value_id
enum {
    id_qmk_polling_rate_pending = 1,
    id_qmk_polling_rate_apply   = 2,
    id_qmk_polling_rate_status_kc = 3,
};

// 커스텀 상태 키코드 enum 정의
enum via_custom_keycodes {
    VIA_KEYMAP_CUSTOM_START = QK_USER,
    ST_OPTIMAL,
    ST_STABILIZING,
    ST_READY,
    VIA_KEYMAP_CUSTOM_END
};

#define IS_VIA_KEYMAP_CUSTOM_KEYCODE(kc) (kc >= VIA_KEYMAP_CUSTOM_START && kc <= VIA_KEYMAP_CUSTOM_END)


void polling_rate_port_init(void);
void via_qmk_polling_rate_command(uint8_t *data, uint8_t length);