#include "polling_rate_port.h"
#include "polling_rate.h"
#include "reset.h"
#include "usbd_hid.h" // usbHidGetStabilityCounter() 함수를 사용하기 위해 포함

// 사용자가 드롭다운에서 선택한 설정을 임시 저장하는 변수
static uint8_t pending_polling_rate_mode;

// 키보드 초기화 시 호출되어야 할 함수
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
            
        case id_qmk_polling_rate_status_kc:
            {
                uint16_t status_keycode;
                uint8_t current_mode = polling_rate_get();

                if (current_mode == POLLING_RATE_8K) {
                    status_keycode = ST_OPTIMAL;
                } else {
                    if (usbHidGetStabilityCounter() >= STABILITY_THRESHOLD) {
                        status_keycode = ST_READY;
                    } else {
                        status_keycode = ST_STABILIZING;
                    }
                }
                // Keycode는 16비트 값이므로 2바이트로 전송 (Big Endian)
                value_data[0] = (status_keycode >> 8) & 0xFF; // 상위 바이트
                value_data[1] = status_keycode & 0xFF;        // 하위 바이트
            }
            break;
    }
}

static void via_qmk_polling_rate_set_value(uint8_t *data) {
    uint8_t *value_id   = &data[0];
    uint8_t *value_data = &data[1];

    switch (*value_id) {
        case id_qmk_polling_rate_pending:
            pending_polling_rate_mode = value_data[0];
            break;

        case id_qmk_polling_rate_apply:
            if (value_data[0] == 1) { 
                if (pending_polling_rate_mode != polling_rate_get()) {
                    if (pending_polling_rate_mode < polling_rate_get() && usbHidGetStabilityCounter() < STABILITY_THRESHOLD) {
                        // 아무것도 하지 않음
                    } else {
                        polling_rate_set(pending_polling_rate_mode);
                        resetToReset();
                    }
                }
            }
            break;

        case id_qmk_polling_rate_status_kc:
            // 이 컨트롤은 읽기 전용이므로 set 요청을 무시합니다.
            break;
    }
}

// 메인 라우터 함수 (via_port.c에서 호출)
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