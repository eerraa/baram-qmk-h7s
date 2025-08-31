#include "ap.h"
#include "qmk/qmk.h"
#include "usb/usb_cmp/polling_rate.h" // polling_rate_get(); 사용을 위함
#include "usb/usb_hid/usbd_hid.h" // usbHidGetStabilityCounter(); 게터와 STABILITY_THRESHOLD 사용을 위함


void cliUpdate(void);


void apInit(void)
{  
  cliOpen(HW_UART_CH_CLI, 115200);  
  qmkInit();

  logBoot(false);
}

void apMain(void)
{
  uint32_t pre_time;
  bool is_led_on = true;


  ledOn(_DEF_LED1);

  pre_time = millis();
  while(1)
  {
    if (is_led_on && millis()-pre_time >= 500)
    {
      is_led_on = false;
      ledOff(_DEF_LED1);
    }

    cliUpdate();
    qmkUpdate();
  }
}

void cliUpdate(void)
{
  static uint8_t cli_ch = HW_UART_CH_CLI; 
  static uint32_t cli_status_pre_time = 0;

  if (usbIsOpen() && usbGetType() == USB_CON_CLI)
  {
    cli_ch = HW_UART_CH_USB;
  }
  else
  {
    cli_ch = HW_UART_CH_CLI;
  }
  if (cli_ch != cliGetPort())
  {
    if (cli_ch == HW_UART_CH_USB)
      logPrintf("\nCLI To USB\n");
    else
      logPrintf("\nCLI To UART\n");
    cliOpen(cli_ch, 0);
  }

  cliMain();
  
  // 'usbhid status on'이 실행되었고, 사용자가 타이핑 중이 아니며, 1초마다 출력
  if (usbHidIsCliStatusEnabled() && cliAvailable() == 0 && millis() - cli_status_pre_time >= 1000)
  {
    cli_status_pre_time = millis();
    
    // --- 실시간 로깅 출력 강화 ---
    uint8_t config_mode = polling_rate_get();
    const char *config_str;
    switch(config_mode)
    {
      case POLLING_RATE_8K: config_str = "8K"; break;
      case POLLING_RATE_4K: config_str = "4K"; break;
      case POLLING_RATE_1K: config_str = "1K"; break;
      default:              config_str = "??"; break;
    }
    
    uint32_t current_rate = usbHidGetActualRate();
    uint32_t stab_cnt     = usbHidGetStabilityCounter();
    uint32_t sof_dt_ms    = usbHidGetSofDtMs();   // [V1.8.2] SOF delta 시간(ms)

    // [V1.8.1] SOF dT(ms) 추가
    cliPrintf("Rate[C:%s A:%lu] Stab[%lu] SOF dT:%lums\r\n",
              config_str,
              (unsigned long)current_rate,
              (unsigned long)stab_cnt,
              (unsigned long)sof_dt_ms);
  }
}

void cliLoopIdle(void)
{
  qmkUpdate();
}