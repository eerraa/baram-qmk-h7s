#pragma once

#include "quantum.h"

// led_port.c와 via_port.c에서 공유할 enum 정의
enum
{
    LED_TYPE_CAPS = 0,
};

void led_init_ports(void);
void via_qmk_led_command(uint8_t led_type, uint8_t *data, uint8_t length);