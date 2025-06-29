#include "led_port.h"
#include "color.h"
#include "eeconfig.h"
#include "led.h" // led_set() 함수를 사용하기 위해 추가
#include "override.h" // 전역 오버라이드 플래그 사용하기 위해 추가
#include "rgblight.h" // rgblight_mode_noeeprom()을 호출하기 위해 추가


#define LED_TYPE_MAX_CH       1
#define CAPS_LED_COUNT        30


typedef union
{
  uint32_t raw;

  struct PACKED
  {
    uint8_t enable : 2;
    uint8_t mode   : 6;
    HSV     hsv;
  };
} led_config_t;

_Static_assert(sizeof(led_config_t) == sizeof(uint32_t), "EECONFIG out of spec.");

enum via_qmk_led_value {
    id_qmk_led_enable       = 1,
    id_qmk_led_brightness   = 2,
    id_qmk_led_color        = 3,
};


static void via_qmk_led_get_value(uint8_t led_type, uint8_t *data);
static void via_qmk_led_set_value(uint8_t led_type, uint8_t *data);
static void via_qmk_led_save(uint8_t led_type);

// static led_t leds_temp_for_via = {0};
static led_config_t led_config[LED_TYPE_MAX_CH];


EECONFIG_DEBOUNCE_HELPER(led_caps,   EECONFIG_USER_LED_CAPS,   led_config[LED_TYPE_CAPS]);


void usbHidSetStatusLed(uint8_t led_bits)
{
  // 받은 LED 상태를 QMK의 공식 LED 설정 함수에 전달합니다.
  led_set(led_bits);
}

void led_init_ports(void)
{
  eeconfig_init_led_caps();
  if (led_config[LED_TYPE_CAPS].mode != 1)
  {
    led_config[LED_TYPE_CAPS].mode = 1;
    led_config[LED_TYPE_CAPS].enable = true;
    led_config[LED_TYPE_CAPS].hsv    = (HSV){HSV_GREEN};
    eeconfig_flush_led_caps(true);
  }
}

void led_update_ports(led_t led_state)
{
    // 이 함수가 호출되기 전, 오버라이드 플래그의 이전 상태를 저장합니다.
    bool was_overridden = rgblight_override_enable;

    // 현재 Caps Lock 상태에 따라 오버라이드 플래그의 새 상태를 결정합니다.
    if (led_config[LED_TYPE_CAPS].enable && led_state.caps_lock) {
        rgblight_override_enable = true;
    } else {
        rgblight_override_enable = false;
    }


    // 이제 결정된 상태에 따라 행동합니다.
    if (rgblight_override_enable) {
        // --- 경우 1: Caps Lock이 켜져 있음 ---
        // led_port.c가 그리기를 독점합니다.
        
        uint32_t led_color;
        RGB      rgb_color;

        rgb_color = hsv_to_rgb(led_config[LED_TYPE_CAPS].hsv);
        led_color = WS2812_COLOR(rgb_color.r, rgb_color.g, rgb_color.b);
        
        for (int i = 0; i < CAPS_LED_COUNT; i++) {
            ws2812SetColor(i, led_color);
        }
        ws2812Refresh();

    } else {
        // --- 경우 2: Caps Lock이 꺼져 있음 ---
        // led_port.c는 절대 LED를 그리지 않습니다.

        // 방금 막 Caps Lock이 꺼졌는지(상태가 전환되었는지) 확인합니다.
        if (was_overridden == true) {
            // 네, 방금 꺼졌습니다.
            // RGBLIGHT 시스템에게 제어권을 넘겨주고, 원래 효과를 복원하라고 명령합니다.
            if (rgblight_is_enabled()) {
                rgblight_mode_noeeprom(rgblight_get_mode());
            }
        }
        // (만약 원래부터 꺼져 있었다면, 아무것도 하지 않고 rgblight가 계속 작동하도록 둡니다.)
    }
}

void via_qmk_led_command(uint8_t led_type, uint8_t *data, uint8_t length)
{
  // data = [ command_id, channel_id, value_id, value_data ]
  uint8_t *command_id        = &(data[0]);
  uint8_t *value_id_and_data = &(data[2]);

  switch (*command_id)
  {
    case id_custom_set_value:
      {
        via_qmk_led_set_value(led_type, value_id_and_data);
        break;
      }
    case id_custom_get_value:
      {
        via_qmk_led_get_value(led_type, value_id_and_data);
        break;
      }
    case id_custom_save:
      {
        via_qmk_led_save(led_type);
        break;
      }
    default:
      {
        *command_id = id_unhandled;
        break;
      }
  }
}

void via_qmk_led_get_value(uint8_t led_type, uint8_t *data)
{
  // data = [ value_id, value_data ]
  uint8_t *value_id   = &(data[0]);
  uint8_t *value_data = &(data[1]);
  switch (*value_id)
  {
    case id_qmk_led_enable:
      {
        value_data[0] = led_config[led_type].enable;
        break;
      }    
    case id_qmk_led_brightness:
      {
        value_data[0] = led_config[led_type].hsv.v;
        break;
      }
    case id_qmk_led_color:
      {
        value_data[0] = led_config[led_type].hsv.h;
        value_data[1] = led_config[led_type].hsv.s;
        break;
      }
  }
}

void via_qmk_led_set_value(uint8_t led_type, uint8_t *data)
{
  // data = [ value_id, value_data ]
  uint8_t *value_id   = &(data[0]);
  uint8_t *value_data = &(data[1]);
  switch (*value_id)
  {
    case id_qmk_led_enable:
      {
        led_config[led_type].enable = value_data[0];
        break;
      }
    case id_qmk_led_brightness:
      {
        led_config[led_type].hsv.v = value_data[0];
        break;
      }
    case id_qmk_led_color:
      {
        led_config[led_type].hsv.h = value_data[0];
        led_config[led_type].hsv.s = value_data[1];
        break;
      }
  }
  
  led_set(host_keyboard_led_state().raw);
}

void via_qmk_led_save(uint8_t led_type)
{
  if (led_type == LED_TYPE_CAPS)
  {
    eeconfig_flush_led_caps(true);
  }  
}
