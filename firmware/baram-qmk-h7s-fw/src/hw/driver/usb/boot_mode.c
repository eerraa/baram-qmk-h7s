#include "boot_mode.h"
#include "polling_rate.h"
#include "eeprom.h"
#include "port.h"
#include "log.h"

static boot_mode_t current_boot_mode = BOOT_MODE_USB_HS_8K;

void boot_mode_init(void)
{
  uint8_t mode;
  uint32_t addr = (uint32_t)EECONFIG_USER_BOOT_MODE;

  if (eepromReadByte(addr, &mode) == true)
  {
    if (mode > BOOT_MODE_USB_FS_1K)
    {
      mode = BOOT_MODE_USB_HS_8K;
      eepromWriteByte(addr, mode);
    }
  }
  else
  {
    mode = BOOT_MODE_USB_HS_8K;
    eepromWriteByte(addr, mode);
  }

  current_boot_mode = (boot_mode_t)mode;
}

boot_mode_t boot_mode_get(void)
{
  return current_boot_mode;
}

bool boot_mode_set(boot_mode_t mode)
{
  if (mode > BOOT_MODE_USB_FS_1K)
  {
    return false;
  }

  uint32_t addr = (uint32_t)EECONFIG_USER_BOOT_MODE;

  if (eepromWriteByte(addr, (uint8_t)mode) != true)
  {
    return false;
  }

  current_boot_mode = mode;
  return true;
}

USBD_SpeedTypeDef boot_mode_apply(void)
{
  switch (current_boot_mode)
  {
    case BOOT_MODE_USB_HS_4K:
      polling_rate_set(POLLING_RATE_4K);
      return USBD_SPEED_HIGH;

    case BOOT_MODE_USB_FS_1K:
      polling_rate_set(POLLING_RATE_1K);
      return USBD_SPEED_FULL;

    case BOOT_MODE_USB_HS_8K:
    default:
      polling_rate_set(POLLING_RATE_8K);
      return USBD_SPEED_HIGH;
  }
}

