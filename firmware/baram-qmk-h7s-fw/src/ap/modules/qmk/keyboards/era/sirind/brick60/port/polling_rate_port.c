#include "polling_rate_port.h"
#include "polling_rate.h"
#include "reset.h"
#include "usbd_hid.h" // usbHidGetStabilityCounter() 함수를 사용하기 위해 포함

static uint8_t pending_polling_rate_mode;

void polling_rate_port_init(void) {
    pending_polling_rate_mode = polling_rate_get();
}

static void via_qmk_polling_rate_get_value(uint8_t *data) {
    uint8_t *value_id   = &data[0];
    uint8_t *value_data = &data[1]; 

    switch (*value_id) {
        case id_qmk_polling_rate_pending:
            value_data[0] = pending_polling_rate_mode;
            break;

        case id_qmk_polling_rate_apply:
            value_data[0] = 0; 
            break;
    }
}

static void via_qmk_polling_rate_set_value(uint8_t *data) {
    uint8_t *value_id   = &data[0];
    uint8_t *value_data = &data[1];

    switch (*value_id) {
        case id_qmk_polling_rate_pending:
            if (value_data[0] <= POLLING_RATE_1K) {
                pending_polling_rate_mode = value_data[0];
            }
            break;

        case id_qmk_polling_rate_apply:
            if (value_data[0] == 1) { 
                uint8_t current_mode = polling_rate_get();

                if (pending_polling_rate_mode != current_mode) {
                    // 수동 복구 시나리오: 현재보다 높은 폴링레이트를 선택했지만 아직 불안정할 경우 적용 안 함
                    if (pending_polling_rate_mode < current_mode && usbHidGetStabilityCounter() < STABILITY_THRESHOLD) {
                        // 아무것도 하지 않음. 사용자는 CLI를 통해 상태를 확인해야 함.
                    } else {
                        // 설정 변경 및 재부팅
                        polling_rate_set(pending_polling_rate_mode);
                        resetToReset();
                    }
                }
            }
            break;
    }
}

// via_qmk_polling_rate_command 함수는 변경 없음
void via_qmk_polling_rate_command(uint8_t *data, uint8_t length) {
    uint8_t *command_id        = &data[0];
    uint8_t *value_id_and_data = &data[2];

    switch (*command_id) {
        case id_custom_set_value:
            via_qmk_polling_rate_set_value(value_id_and_data);
            break;
        case id_custom_get_value:
            via_qmk_polling_rate_get_value(value_id_and_data);
            break;
        case id_custom_save:
            break;
        default:
            *command_id = id_unhandled;
            break;
    }
}