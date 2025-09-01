#pragma once

#include <stdbool.h>
#include "usbd_def.h"

typedef enum
{
  BOOT_MODE_USB_HS_8K = 0,
  BOOT_MODE_USB_HS_4K,
  BOOT_MODE_USB_FS_1K,
} boot_mode_t;

void            boot_mode_init(void);
boot_mode_t     boot_mode_get(void);
bool            boot_mode_set(boot_mode_t mode);
USBD_SpeedTypeDef boot_mode_apply(void);

