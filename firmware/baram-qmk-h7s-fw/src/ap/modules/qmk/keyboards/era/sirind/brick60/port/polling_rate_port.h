#pragma once
#include "quantum.h"

// VIA JSON에 정의된 value_id (단순화된 버전)
enum {
    id_qmk_polling_rate_pending = 1,
    id_qmk_polling_rate_apply   = 2,
};

void polling_rate_port_init(void);
void via_qmk_polling_rate_command(uint8_t *data, uint8_t length);