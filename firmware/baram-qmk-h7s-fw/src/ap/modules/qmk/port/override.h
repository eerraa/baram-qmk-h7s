#pragma once
#include <stdbool.h>

// led_port.c가 이 값을 쓰고, rgblight.c가 이 값을 읽음
extern volatile bool rgblight_override_enable;