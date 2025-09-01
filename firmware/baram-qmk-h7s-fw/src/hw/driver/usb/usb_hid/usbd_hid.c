/** 
  ******************************************************************************
  * @file    usbd_hid.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                HID Class  Description
  *          ===================================================================
  *           This module manages the HID class V1.11 following the "Device Class Definition
  *           for Human Interface Devices (HID) Version 1.11 Jun 27, 2001".
  *           This driver implements the following aspects of the specification:
  *             - The Boot Interface Subclass
  *             - The Mouse protocol
  *             - Usage Page : Generic Desktop
  *             - Usage : Joystick
  *             - Collection : Application
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  */


#include "usbd_hid.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"

#include "cli.h"
#include "log.h"
#include "keys.h"
#include "qbuffer.h"
#include "report.h"

#include "reset.h"        // resetToReset() 함수를 사용하기 위함
#include "polling_rate.h" // polling_rate_init() 함수를 사용하기 위함
#include "micros.h"       // [V1.5.0] 새로 만든 64비트 시간 함수를 사용하기 위함

#if HW_USB_LOG == 1
#define logDebug(...)                              \
  {                                                \
    if (HW_LOG_CH == HW_UART_CH_USB) logDisable(); \
    logPrintf(__VA_ARGS__);                        \
    if (HW_LOG_CH == HW_UART_CH_USB) logEnable();  \
  }
#else
#define logDebug(...)
#endif


#define HID_KEYBOARD_REPORT_SIZE (HW_KEYS_PRESS_MAX + 2U)
#define KEY_TIME_LOG_MAX         32

//-- 함수 프로토타입 선언
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_HID_SOF(USBD_HandleTypeDef *pdev);

#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
static uint8_t *USBD_HID_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length);
#endif


static void cliCmd(cli_args_t *args);
static bool usbHidUpdateWakeUp(USBD_HandleTypeDef *pdev);

// [V1.1.0] 최적화: 타이머 콜백 내부 로직을 별도 함수로 분리
static void usbHidProcessAutoStability(void);
static void usbHidProcessReportQueue(void);

// [V1.1.0] 최적화: `usbhid rate` 디버깅 관련 변수를 구조체로 통합
typedef struct
{
  // Rate 측정 관련
  uint32_t data_in_cnt;
  uint32_t data_in_rate;
  uint32_t poll_rate_measure_cnt;

  // [V1.8.1] 첫 1초 프라임 여부
  bool     window_primed; // [V1.8.1] 첫 1초 집계만 하고 출력 금지

  // --- 전송 지연(latency) 집계 ---
  bool     rate_time_req;
  uint64_t rate_time_pre;
  uint32_t rate_time_us;
  uint32_t rate_time_sum;
  uint32_t rate_time_min_check;
  uint32_t rate_time_max_check;
  uint32_t rate_time_avg;
  uint32_t rate_time_max;
  uint32_t rate_time_min;
  
  // [V1.8.2] 유효 지연 샘플 수(평균 분모)
  uint32_t rate_valid_cnt;

  // [V1.8.2] SendReport 시점에 다음 폴링까지 남은 시간(위상 보정용)
  uint32_t rate_phase_wait_us;

  // [V1.8.1] 동일-틱/무효 샘플 카운팅
  uint32_t same_tick_cnt;        // 더는 드랍 근거로 쓰지 않음(0 유지)
  uint32_t invalid_samples_cnt;  // <5us 같은 비정상치만 카운트

  // [V1.8.1] 기대 폴링 간격(진단용)
  uint32_t expected_interval_us;

  // [V1.8.1] Δt_in(연속 DataIn 간격) 통계 -------------------------------
  uint64_t din_prev_time_us;        // 직전 DataIn 완료 시각(us)
  uint32_t din_interval_us;         // 가장 최근 Δt_in
  uint32_t din_interval_sum;        // 윈도우 합
  uint32_t din_interval_min_check;  // 윈도우 최소
  uint32_t din_interval_max_check;  // 윈도우 최대
  uint32_t din_interval_avg;        // 출력용 평균
  uint32_t din_interval_min;        // 출력용 최소
  uint32_t din_interval_max;        // 출력용 최대
  uint32_t din_interval_cnt;        // 윈도우 유효 샘플 수
  uint16_t din_his_buf[100];        // Δt_in 히스토그램(10us/bin, 0~990us)
  uint32_t din_invalid_cnt;         // 비정상(예: <20us) 샘플 수
  bool     din_discard_next;        // [V1.8.2] 윈도우 경계 직후 첫 Δt_in은 버림

  // SOF/Timer 동기화 측정 관련 (버스 상태 모니터링 전용)
  uint32_t sof_cnt;
  uint32_t timer_cnt;
  uint32_t sof_interval_us;
  uint32_t sof_interval_max_us;

  // Histogram (전송 지연)
  uint16_t rate_his_buf[100];
} usb_hid_rate_debug_t;

static usb_hid_rate_debug_t rate_debug; // 디버깅 구조체 인스턴스
static void usbHidResetDebugCounters(void);
static void usbHidMeasurePollRate(void);
static void usbHidMeasureRateTime(void);

// [V1.8.1] 동일-틱 가드를 위한 tick id
static volatile uint32_t g_usb_tick_id = 0;     // 8kHz 콜백마다 증가
static uint32_t          rate_pre_tick_id = 0;  // 전송 시작 시점의 tick 스냅샷
static uint32_t          key_pre_tick_id  = 0;  // 키 로깅 시작 시점의 tick 스냅샷

//-- 외부 공개 함수
bool usbHidIsCliStatusEnabled(void);
uint32_t usbHidGetActualRate(void);

//-- 구조체 선언
typedef struct
{
  uint8_t  buf[HID_KEYBOARD_REPORT_SIZE];
} report_info_t;

typedef struct
{
  uint8_t  buf[32];
} via_report_info_t;

typedef struct
{
  uint8_t len;
  uint8_t buf[HID_EXK_EP_SIZE];
} exk_report_info_t;


// 안정성 측정 및 공용 변수들
static uint32_t sof_1s_cnt = 0;       // 1초간 실제 폴링레이트 측정용 카운터
static uint32_t last_sof_time_ms = 0; // SOF 수신 시간을 기록할 변수
static uint32_t instability_counter = 0;
static uint32_t stability_counter = 0; // 모듈 내부에서만 사용
static uint64_t timer_sof_start_time = 0; // [V1.8.0] 64-bit: SOF-Timer 간격 측정을 위한 시작 시각(us)

// CLI 및 기타 변수들
static USBD_SetupReqTypedef ep0_req;
static uint8_t ep0_req_buf[USB_MAX_EP0_SIZE];

static qbuffer_t             via_report_q;
static via_report_info_t     via_report_q_buf[128];
static uint32_t              via_report_pre_time;
static uint32_t              via_report_time = 20;
__ALIGN_BEGIN static uint8_t via_hid_usb_report[32] __ALIGN_END;
static void (*via_hid_receive_func)(uint8_t *data, uint8_t length) = NULL;

static qbuffer_t              report_q;
static report_info_t          report_buf[128];
__ALIGN_BEGIN  static uint8_t hid_buf[HID_KEYBOARD_REPORT_SIZE] __ALIGN_END = {0,};

static qbuffer_t              report_exk_q;
static exk_report_info_t      report_exk_buf[128];
__ALIGN_BEGIN  static uint8_t hid_buf_exk[HID_EXK_EP_SIZE] __ALIGN_END = {0,};

static bool     key_time_req = false;
static uint64_t key_time_pre;                // [V1.8.1] 64-bit로 승격
static uint32_t key_time_end;
static uint32_t key_time_idx = 0;
static uint32_t key_time_cnt = 0;
static uint32_t key_time_log[KEY_TIME_LOG_MAX];
static bool     key_time_raw_req = false;
static uint32_t key_time_raw_pre;
static uint32_t key_time_raw_log[KEY_TIME_LOG_MAX];
static uint32_t key_time_pre_log[KEY_TIME_LOG_MAX];


// 전역 변수 선언
bool cli_status_enabled = false;
uint32_t actual_polling_rate = 0;
#define INSTABILITY_THRESHOLD 3 // 3초 연속 성능 저하 시 강등

USBD_ClassTypeDef USBD_HID =
{
  USBD_HID_Init,
  USBD_HID_DeInit,
  USBD_HID_Setup,
  NULL,                 /* EP0_TxSent */
  USBD_HID_EP0_RxReady, /* EP0_RxReady */
  USBD_HID_DataIn,      /* DataIn */
  USBD_HID_DataOut,     /* DataOut */
  USBD_HID_SOF,         /* SOF */
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_HID_GetHSCfgDesc,
  USBD_HID_GetFSCfgDesc,
  USBD_HID_GetOtherSpeedCfgDesc,
  USBD_HID_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
  USBD_HID_GetUsrStrDescriptor,
#endif
};

#ifndef USE_USBD_COMPOSITE
/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_HID_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09,                                               /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION,                        /* bDescriptorType: Configuration */
  USB_HID_CONFIG_DESC_SIZ,                            /* wTotalLength: Bytes returned */
  0x00,
  0x03,                                               /* bNumInterfaces: 3 interface */
  0x01,                                               /* bConfigurationValue: Configuration value */
  0x00,                                               /* iConfiguration: Index of string descriptor
                                                         describing the configuration */
#if (USBD_SELF_POWERED == 1U)
  0xE0,                                               /* bmAttributes: Bus Powered according to user configuration */
#else
  0xA0,                                               /* bmAttributes: Bus Powered according to user configuration */
#endif /* USBD_SELF_POWERED */
  USBD_MAX_POWER,                                     /* MaxPower (mA) */

  /************** Descriptor of Keyboard interface ****************/
  /* 09 */
  0x09,                                               /* bLength: Interface Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: Interface descriptor type */
  0x00,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints */
  0x03,                                               /* bInterfaceClass: HID */
  0x01,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x01,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0,                                                  /* iInterface: Index of string descriptor */
  /******************** Descriptor of Keyboard HID ********************/
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_KEYBOARD_REPORT_DESC_SIZE,                      /* wItemLength: Total length of Report descriptor */
  0x00,
  /******************** Descriptor of Keyboard endpoint ********************/
  /* 27 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/

  HID_EPIN_ADDR,                                      /* bEndpointAddress: Endpoint Address (IN) */
  0x03,                                               /* bmAttributes: Interrupt endpoint */
  HID_EPIN_SIZE,                                      /* wMaxPacketSize: */
  0x00,
  HID_HS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 34 */


  /*---------------------------------------------------------------------------*/
  /* VIA interface descriptor */
  0x09,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: */
  0x01,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x02,                                               /* bNumEndpoints: Two endpoints used */
  0x03,                                               /* bInterfaceClass: HID */
  0x00,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00,                                               /* iInterface */

  /******************** Descriptor of VIA ********************/
  /* 43 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_KEYBOARD_VIA_REPORT_DESC_SIZE,                  /* wItemLength: Total length of Report descriptor */
  0x00,

  /******************** Descriptor of VIA endpoint ********************/
  /* 52 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/
  HID_VIA_EP_IN,                                      /* bEndpointAddress: Endpoint Address (IN) */
  USBD_EP_TYPE_INTR,                                  /* bmAttributes: Interrupt endpoint */
  HID_VIA_EP_SIZE,                                    /* wMaxPacketSize: */
  0x00,
  4,                                                  /* bInterval: Polling Interval */

  /* 59 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/
  HID_VIA_EP_OUT,                                     /* bEndpointAddress: Endpoint Address (OUT) */
  USBD_EP_TYPE_INTR,                                  /* bmAttributes: Interrupt endpoint */
  HID_VIA_EP_SIZE,                                    /* wMaxPacketSize: */
  0x00,
  4,                                                  /* bInterval: Polling Interval */
  /* 66 */


  /*---------------------------------------------------------------------------*/
  /* EXK interface descriptor */
  0x09,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_INTERFACE,                            /* bDescriptorType: */
  0x02,                                               /* bInterfaceNumber: Number of Interface */
  0x00,                                               /* bAlternateSetting: Alternate setting */
  0x01,                                               /* bNumEndpoints: One endpoint used */
  0x03,                                               /* bInterfaceClass: HID */
  0x01,                                               /* bInterfaceSubClass : 1=BOOT, 0=no boot */
  0x00,                                               /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
  0x00,                                               /* iInterface */

  /******************** Descriptor of EXK ********************/
  /* 75 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_EXK_REPORT_DESC_SIZE,                           /* wItemLength: Total length of Report descriptor */
  0x00,

  /******************** Descriptor of EXK endpoint ********************/
  /* 84 */
  0x07,                                               /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT,                             /* bDescriptorType:*/
  HID_EXK_EP_IN,                                      /* bEndpointAddress: Endpoint Address (IN) */
  USBD_EP_TYPE_INTR,                                  /* bmAttributes: Interrupt endpoint */
  HID_EXK_EP_SIZE,                                    /* wMaxPacketSize: */
  0x00,
  HID_HS_BINTERVAL,                                   /* bInterval: Polling Interval */
  /* 91 */

};
#endif /* USE_USBD_COMPOSITE  */

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END =
{
  /* 18 */
  0x09,                                               /* bLength: HID Descriptor size */
  HID_DESCRIPTOR_TYPE,                                /* bDescriptorType: HID */
  0x11,                                               /* bcdHID: HID Class Spec release number */
  0x01,
  0x00,                                               /* bCountryCode: Hardware target country */
  0x01,                                               /* bNumDescriptors: Number of HID class descriptors to follow */
  0x22,                                               /* bDescriptorType */
  HID_KEYBOARD_REPORT_DESC_SIZE,                      /* wItemLength: Total length of Report descriptor */
  0x00,
};

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */

__ALIGN_BEGIN static uint8_t HID_KEYBOARD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] __ALIGN_END =
{
  0x05, 0x01,                         // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,                         // USAGE (Keyboard)
  0xa1, 0x01,                         // COLLECTION (Application)
  0x05, 0x07,                         //   USAGE_PAGE (Keyboard)
  0x19, 0xe0,                         //   USAGE_MINIMUM (Keyboard LeftControl)
  0x29, 0xe7,                         //   USAGE_MAXIMUM (Keyboard Right GUI)
  0x15, 0x00,                         //   LOGICAL_MINIMUM (0)
  0x25, 0x01,                         //   LOGICAL_MAXIMUM (1)
  0x75, 0x01,                         //   REPORT_SIZE (1)
  0x95, 0x08,                         //   REPORT_COUNT (8)
  0x81, 0x02,                         //   INPUT (Data,Var,Abs)
  0x95, 0x01,                         //   REPORT_COUNT (1)
  0x75, 0x08,                         //   REPORT_SIZE (8)
  0x81, 0x03,                         //   INPUT (Cnst,Var,Abs)
  0x95, 0x05,                         //   REPORT_COUNT (5)
  0x75, 0x01,                         //   REPORT_SIZE (1)
  0x05, 0x08,                         //   USAGE_PAGE (LEDs)
  0x19, 0x01,                         //   USAGE_MINIMUM (Num Lock)
  0x29, 0x05,                         //   USAGE_MAXIMUM (Kana)
  0x91, 0x02,                         //   OUTPUT (Data,Var,Abs)
  0x95, 0x01,                         //   REPORT_COUNT (1)
  0x75, 0x03,                         //   REPORT_SIZE (3)
  0x91, 0x03,                         //   OUTPUT (Cnst,Var,Abs)
  0x95, HW_KEYS_PRESS_MAX,            //   REPORT_COUNT (6)
  0x75, 0x08,                         //   REPORT_SIZE (8)
  0x15, 0x00,                         //   LOGICAL_MINIMUM (0)
  0x26, 0xFF, 0x00,                   //   LOGICAL_MAXIMUM (255)
  0x05, 0x07,                         //   USAGE_PAGE (Keyboard)
  0x19, 0x00,                         //   USAGE_MINIMUM (Reserved (no event indicated))
  0x29, 0xFF,                         //   USAGE_MAXIMUM (Keyboard Application)
  0x81, 0x00,                         //   INPUT (Data,Ary,Abs)
  0xc0                                // END_COLLECTION
};

__ALIGN_BEGIN static uint8_t HID_VIA_ReportDesc[HID_KEYBOARD_VIA_REPORT_DESC_SIZE] __ALIGN_END =
{
  //
  0x06, 0x60, 0xFF, // Usage Page (Vendor Defined)
  0x09, 0x61,       // Usage (Vendor Defined)
  0xA1, 0x01,       // Collection (Application)
  // Data to host
  0x09, 0x62,       //   Usage (Vendor Defined)
  0x15, 0x00,       //   Logical Minimum (0)
  0x26, 0xFF, 0x00, //   Logical Maximum (255)
  0x95, 32,         //   Report Count
  0x75, 0x08,       //   Report Size (8)
  0x81, 0x02,       //   Input (Data, Variable, Absolute)
  // Data from host
  0x09, 0x63,       //   Usage (Vendor Defined)
  0x15, 0x00,       //   Logical Minimum (0)
  0x26, 0xFF, 0x00, //   Logical Maximum (255)
  0x95, 32,         //   Report Count
  0x75, 0x08,       //   Report Size (8)
  0x91, 0x02,       //   Output (Data, Variable, Absolute)
  0xC0              // End Collection
};

__ALIGN_BEGIN static uint8_t HID_EXK_ReportDesc[HID_EXK_REPORT_DESC_SIZE] __ALIGN_END =
{
  //
  0x05, 0x01,               // Usage Page (Generic Desktop)
  0x09, 0x80,               // Usage (System Control)
  0xA1, 0x01,               // Collection (Application)
  0x85, REPORT_ID_SYSTEM,   //   Report ID
  0x19, 0x01,               //   Usage Minimum (Pointer)
  0x2A, 0xB7, 0x00,         //   Usage Maximum (System Display LCD Autoscale)
  0x15, 0x01,               //   Logical Minimum
  0x26, 0xB7, 0x00,         //   Logical Maximum
  0x95, 0x01,               //   Report Count (1)
  0x75, 0x10,               //   Report Size (16)
  0x81, 0x00,               //   Input (Data, Array, Absolute)
  0xC0,                     // End Collection

  0x05, 0x0C,               // Usage Page (Consumer)
  0x09, 0x01,               // Usage (Consumer Control)
  0xA1, 0x01,               // Collection (Application)
  0x85, REPORT_ID_CONSUMER, //   Report ID
  0x19, 0x01,               //   Usage Minimum (Consumer Control)
  0x2A, 0xA0, 0x02,         //   Usage Maximum (AC Desktop Show All Applications)
  0x15, 0x01,               //   Logical Minimum
  0x26, 0xA0, 0x02,         //   Logical Maximum
  0x95, 0x01,               //   Report Count (1)
  0x75, 0x10,               //   Report Size (16)
  0x81, 0x00,               //   Input (Data, Array, Absolute)
  0xC0                      // End Collection
};

static USBD_HID_HandleTypeDef *p_hhid = NULL;
static uint8_t HIDInEpAdd = HID_EPIN_ADDR;
extern USBD_HandleTypeDef USBD_Device;
// static TIM_HandleTypeDef htim2; // [V1.5.0] 삭제: micros.c에서 중앙 관리

// [V1.5.0] 8kHz 콜백으로 호출될 함수 프로토타입
static void usbHidTimerCallback(void);

// [V1.8.1] 기대 폴링 간격 계산 유틸
static uint32_t usbHidExpectedIntervalUs(void)
{
  USBD_HandleTypeDef *pdev = &USBD_Device;
  uint8_t b = pdev->ep_in[HIDInEpAdd & 0x0FU].bInterval;
  if (b == 0) b = 1;
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    // HS: 125us * 2^(bInterval-1)
    return 125U * (1U << (b - 1U));
  }
  else
  {
    // FS: bInterval * 1ms
    return (uint32_t)b * 1000U;
  }
}

/**
  * @brief  USBD_HID_Init
  *         Initialize the HID interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

  USBD_HID_HandleTypeDef *hhid;

  hhid = (USBD_HID_HandleTypeDef *)USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (hhid == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  p_hhid = hhid;

  pdev->pClassDataCmsit[pdev->classId] = (void *)hhid;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];


#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */
  pdev->ep_in[HIDInEpAdd & 0x0FU].bInterval = pdev->dev_speed == USBD_SPEED_HIGH ? HID_HS_BINTERVAL:HID_FS_BINTERVAL;

  /* Open EP IN */
  (void)USBD_LL_OpenEP(pdev, HIDInEpAdd, USBD_EP_TYPE_INTR, HID_EPIN_SIZE);
  pdev->ep_in[HIDInEpAdd & 0x0FU].is_used = 1U;


  // VIA EP
  //
  pdev->ep_in[HID_VIA_EP_IN & 0x0FU].bInterval = pdev->dev_speed == USBD_SPEED_HIGH ? HID_HS_BINTERVAL:HID_FS_BINTERVAL;
  (void)USBD_LL_OpenEP(pdev, HID_VIA_EP_IN, USBD_EP_TYPE_INTR, HID_VIA_EP_SIZE);
  pdev->ep_in[HID_VIA_EP_IN & 0x0FU].is_used = 1U;

  // [V1.8.4] FIX: OUT 엔드포인트 메타데이터는 ep_out[]에 설정되어야 함
  pdev->ep_out[HID_VIA_EP_OUT & 0x0FU].bInterval = pdev->dev_speed == USBD_SPEED_HIGH ? HID_HS_BINTERVAL:HID_FS_BINTERVAL; // [V1.8.4]
  (void)USBD_LL_OpenEP(pdev, HID_VIA_EP_OUT, USBD_EP_TYPE_INTR, HID_VIA_EP_SIZE);
  pdev->ep_out[HID_VIA_EP_OUT & 0x0FU].is_used = 1U;       

  // EXK EP
  //
  pdev->ep_in[HID_EXK_EP_IN & 0x0FU].bInterval = pdev->dev_speed == USBD_SPEED_HIGH ? HID_HS_BINTERVAL:HID_FS_BINTERVAL;
  (void)USBD_LL_OpenEP(pdev, HID_EXK_EP_IN, USBD_EP_TYPE_INTR, HID_EXK_EP_SIZE);
  pdev->ep_in[HID_EXK_EP_IN & 0x0FU].is_used = 1U;


  hhid->state = USBD_HID_IDLE;

  /* Prepare Out endpoint to receive next packet */
  (void)USBD_LL_PrepareReceive(pdev, HID_VIA_EP_OUT, via_hid_usb_report, 32);


  static bool is_first = true;
  if (is_first)
  {
    is_first = false;

    //polling_rate_init(); 주석처리함, hwInit() 함수에서 이미 호출됨

    qbufferCreateBySize(&report_q, (uint8_t *)report_buf, sizeof(report_info_t), 128);
    qbufferCreateBySize(&via_report_q, (uint8_t *)via_report_q_buf, sizeof(via_report_info_t), 128);
    qbufferCreateBySize(&report_exk_q, (uint8_t *)report_exk_buf, sizeof(report_info_t), 128);

    logPrintf("[OK] USB Hid\n");
    logPrintf("     Keyboard\n");
    cliAdd("usbhid", cliCmd);

    // usbHidInitTimer(); // [V1.5.0] 삭제: hwInit()에서 microsInit()이 호출됨
    microsSetCallback(usbHidTimerCallback); // [V1.5.0] 8kHz 콜백 함수 등록
  }

  // [V1.8.1] 기대 폴링 간격(진단용) 초기화
  rate_debug.expected_interval_us = usbHidExpectedIntervalUs();

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_DeInit
  *         DeInitialize the HID layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  HIDInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_INTR, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Close HID EPs */
  (void)USBD_LL_CloseEP(pdev, HIDInEpAdd);
  pdev->ep_in[HIDInEpAdd & 0x0FU].is_used = 0U;
  pdev->ep_in[HIDInEpAdd & 0x0FU].bInterval = 0U;

  /* Free allocated memory */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_Setup
  *         Handle the HID specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  USBD_StatusTypeDef ret = USBD_OK;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  logDebug("HID_SETUP %d\n", pdev->classId);
  logDebug("  req->bmRequest : 0x%X\n", req->bmRequest);
  logDebug("  req->bRequest  : 0x%X\n", req->bRequest);
  logDebug("       wIndex    : 0x%X\n", req->wIndex);
  logDebug("       wLength   : 0x%X %d\n", req->wLength, req->wLength);

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS :
      switch (req->bRequest)
      {
        case USBD_HID_REQ_SET_PROTOCOL:
          logDebug("  USBD_HID_REQ_SET_PROTOCOL  : 0x%X, 0x%d\n", req->wValue, req->wLength);
          hhid->Protocol = (uint8_t)(req->wValue);
          break;

        case USBD_HID_REQ_GET_PROTOCOL:
          logDebug("  USBD_HID_REQ_GET_PROTOCOL  : 0x%X, 0x%d\n", req->wValue, req->wLength);
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
          break;

        case USBD_HID_REQ_SET_IDLE:
          logDebug("  USBD_HID_REQ_SET_IDLE  : 0x%X, 0x%d\n", req->wValue, req->wLength);
          hhid->IdleState = (uint8_t)(req->wValue >> 8);
          break;

        case USBD_HID_REQ_GET_IDLE:
          logDebug("  USBD_HID_REQ_GET_IDLE  : 0x%X, 0x%d\n", req->wValue, req->wLength);
          (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
          break;

        case USBD_HID_REQ_SET_REPORT:
          logDebug("  USBD_HID_REQ_SET_REPORT  : 0x%X, 0x%d\n", req->wValue, req->wLength);
          ep0_req = *req;
          USBD_CtlPrepareRx(pdev, ep0_req_buf, req->wLength);
          break;

        default:
          logDebug("  ERROR  : 0x%X\n", req->wValue);
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          logDebug("  USB_REQ_GET_DESCRIPTOR  : 0x%X\n", req->wValue);
          if ((req->wValue >> 8) == HID_REPORT_DESC)
          {
            switch(req->wIndex)
            {
              case 1:
                len = MIN(HID_KEYBOARD_VIA_REPORT_DESC_SIZE, req->wLength);
                pbuf = HID_VIA_ReportDesc;
                break;

              case 2:
                len = MIN(HID_EXK_REPORT_DESC_SIZE, req->wLength);
                pbuf = HID_EXK_ReportDesc;
                break;

              default:
                len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
                pbuf = HID_KEYBOARD_ReportDesc;
              break;
            }
          }
          else if ((req->wValue >> 8) == HID_DESCRIPTOR_TYPE)
          {
            pbuf = USBD_HID_Desc;
            len = MIN(USB_HID_DESC_SIZ, req->wLength);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
          }
          (void)USBD_CtlSendData(pdev, pbuf, len);
          break;

        case USB_REQ_GET_INTERFACE :
          logDebug("  USB_REQ_GET_INTERFACE  : 0x%X\n", req->wValue);
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          logDebug("  USB_REQ_SET_INTERFACE  : 0x%X\n", req->wValue);
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            hhid->AltSetting = (uint8_t)(req->wValue);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          logDebug("  USB_REQ_CLEAR_FEATURE  : 0x%X\n", req->wValue);
          break;

        default:
          logDebug("  ERROR  : 0x%X\n", req->wValue);
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

/**
  * @brief  USBD_HID_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev) // [V1.8.4] 선언과 일치
{
  logDebug("USBD_HID_EP0_RxReady()\n");
  logDebug("  req->bmRequest : 0x%X\n", ep0_req.bmRequest);
  logDebug("  req->bRequest  : 0x%X\n", ep0_req.bRequest);
  logDebug("  %d \n", ep0_req.wLength);
  for (int i=0; i<ep0_req.wLength; i++)
  {
    logDebug("  %d : 0x%02X\n", i, ep0_req_buf[i]);
  }

  if (ep0_req.bRequest == USBD_HID_REQ_SET_REPORT)
  {
    uint8_t led_bits = ep0_req_buf[0];

    usbHidSetStatusLed(led_bits);
  }
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_HID_SendReport
  *         Send HID Report
  * @param  buff: pointer to report
  * @retval status
  */
bool USBD_HID_SendReport(uint8_t *report, uint16_t len)
{
  USBD_HandleTypeDef *pdev = &USBD_Device;
  bool ret = false;

  if (p_hhid == NULL)
  {
    return false;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (p_hhid->state == USBD_HID_IDLE)
    {
      ret = true;
      p_hhid->state = USBD_HID_BUSY;
      (void)USBD_LL_Transmit(pdev, HID_EPIN_ADDR, report, len);
    }
  }

  return ret;
}

/**
  * @brief  USBD_HID_SendReportEXK
  *         Send HID Report
  * @param  buff: pointer to report
  * @retval status
  */
bool USBD_HID_SendReportEXK(uint8_t *report, uint16_t len)
{
  USBD_HandleTypeDef *pdev = &USBD_Device;
  bool ret = false;

  if (p_hhid == NULL)
  {
    return false;
  }

  if (pdev->dev_state == USBD_STATE_CONFIGURED)
  {
    if (p_hhid->state == USBD_HID_IDLE)
    {
      ret = true;
      p_hhid->state = USBD_HID_BUSY;
      (void)USBD_LL_Transmit(pdev, HID_EXK_EP_IN, report, len);
    }
  }

  return ret;
}

/**
  * @brief  USBD_HID_GetPollingInterval
  *         return polling interval from endpoint descriptor
  * @param  pdev: device instance
  * @retval polling interval
  */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev)
{
  uint32_t polling_interval;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Sets the data transfer polling interval for high speed transfers.
     Values between 1..16 are allowed. Values correspond to interval
     of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL */
    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
  }
  else   /* LOW and FULL-speed endpoints */
  {
    /* Sets the data transfer polling interval for low and full
    speed transfers */
    polling_interval =  HID_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
uint8_t *USBD_HID_GetUsrStrDescriptor(struct _USBD_HandleTypeDef *pdev, uint8_t index,  uint16_t *length)
{
  logPrintf("USBD_HID_GetUsrStrDescriptor() %d\n", index);
  return USBD_HID_ProductStrDescriptor(pdev->dev_speed, length);
}
#endif

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_HID_GetFSCfgDesc
  *         return FS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_GetHSCfgDesc
  *         return HS configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_HS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
  * @brief  USBD_HID_GetOtherSpeedCfgDesc
  *         return other speed configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpDesc = USBD_GetEpDesc(USBD_HID_CfgDesc, HID_EPIN_ADDR);

  if (pEpDesc != NULL)
  {
    pEpDesc->bInterval = HID_FS_BINTERVAL;
  }

  *length = (uint16_t)sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */

/**
  * @brief  USBD_HID_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  UNUSED(epnum);
  /* Ensure that the FIFO is empty before a new transfer, this condition could
  be caused by  a new transfer before the end of the previous transfer */
  ((USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId])->state = USBD_HID_IDLE;

  if (epnum != (HID_EPIN_ADDR & 0x0F))
  {
    return (uint8_t)USBD_OK;
  }

  rate_debug.data_in_cnt++;

  usbHidMeasureRateTime();
  
  // Δt_in 집계
  uint64_t now_us = micros64();
  if (rate_debug.din_prev_time_us != 0 && !rate_debug.din_discard_next)
  {
    uint32_t dt = (uint32_t)(now_us - rate_debug.din_prev_time_us);

    // [CHG] 하한 20us 유지 + 창 경계 가짜 대형값 방지(상한 200ms)
    if (dt >= 20U && dt <= 200000U)
    {
      rate_debug.din_interval_us  = dt;
      rate_debug.din_interval_sum += dt;
      rate_debug.din_interval_cnt++;

      if (rate_debug.din_interval_min_check == 0 || dt < rate_debug.din_interval_min_check)
        rate_debug.din_interval_min_check = dt;
      if (dt > rate_debug.din_interval_max_check)
        rate_debug.din_interval_max_check = dt;

      uint32_t idx = constrain(dt/10U, 0U, 99U);
      if (rate_debug.din_his_buf[idx] < 0xFFFF) rate_debug.din_his_buf[idx]++;
    }
    else
    {
      rate_debug.din_invalid_cnt++;
    }
  }
  else
  {
    // [V1.8.2] 창 경계 이후 첫 샘플은 버리고 다음부터 정상 집계
    rate_debug.din_discard_next = false;
  }

  rate_debug.din_prev_time_us = now_us;
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (hhid == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Get the received data length */
  uint32_t rx_size;
  rx_size = USBD_LL_GetRxDataSize(pdev, epnum);

  if (via_hid_receive_func != NULL)
  {
    via_hid_receive_func(via_hid_usb_report, rx_size);
  }

  #if 0
  USBD_LL_Transmit(pdev, HID_VIA_EP_OUT, via_hid_usb_report, sizeof(via_hid_usb_report));
  USBD_LL_PrepareReceive(pdev, HID_VIA_EP_OUT, via_hid_usb_report, sizeof(via_hid_usb_report));
  #else
  via_report_info_t info;
  memcpy(info.buf, via_hid_usb_report, sizeof(via_hid_usb_report));
  qbufferWrite(&via_report_q, (uint8_t *)&info, 1);
  via_report_pre_time = millis();
  #endif
  return (uint8_t)USBD_OK;
}

// [V1.8.0] SOF 통계를 SOF ISR에서만 집계
uint8_t USBD_HID_SOF(USBD_HandleTypeDef *pdev)
{
  (void)pdev;
  last_sof_time_ms = millis();
  uint64_t now_us = micros64();
  if (timer_sof_start_time != 0) {
    uint32_t dt = (uint32_t)(now_us - timer_sof_start_time);
    rate_debug.sof_interval_us = dt;
    if (dt > rate_debug.sof_interval_max_us) {
      rate_debug.sof_interval_max_us = dt;
    }
  }
  timer_sof_start_time = now_us;
  rate_debug.sof_cnt++;
  sof_1s_cnt++;
  return (uint8_t)USBD_OK;
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_HID_DeviceQualifierDesc);

  return USBD_HID_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */


bool usbHidUpdateWakeUp(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  bool ret = false;

  if (pdev->dev_state == USBD_STATE_SUSPENDED)
  {
    logPrintf("[  ] USB WakeUp\n");

    __HAL_PCD_UNGATE_PHYCLOCK((hpcd));
    HAL_PCD_ActivateRemoteWakeup(hpcd);
    delay(10);
    HAL_PCD_DeActivateRemoteWakeup(hpcd);
    ret = true;
  }

  return ret;
}

bool usbHidSetViaReceiveFunc(void (*func)(uint8_t *, uint8_t))
{
  via_hid_receive_func = func;
  return true;
}

static inline void hid_mark_send_timestamp(void) // [V1.8.2] 추가
{
  uint64_t now = micros64();
  rate_debug.rate_time_pre = now;
  rate_debug.rate_time_req = true;

  // [V1.8.2] 마지막 SOF로부터 경과시간 → 다음 폴링까지 남은 시간(위상 보정)
  // [V1.8.3] 참고용으로만 유지. 평균/히스토그램 계산에는 더 이상 사용하지 않음.
  uint32_t mod = rate_debug.expected_interval_us;
  if (mod > 0)
  {
    uint32_t since_sof = (uint32_t)(now - timer_sof_start_time);
    rate_debug.rate_phase_wait_us = (mod - (since_sof % mod)) % mod;
  }
  else
  {
    rate_debug.rate_phase_wait_us = 0;
  }
}

bool usbHidSendReport(uint8_t *p_data, uint16_t length)
{
  report_info_t report_info;

  if (length > HID_KEYBOARD_REPORT_SIZE)
    return false;

  if (!USBD_is_suspended())
  {
    memcpy(hid_buf, p_data, length);
    if (USBD_HID_SendReport((uint8_t *)hid_buf, HID_KEYBOARD_REPORT_SIZE))
    {
      hid_mark_send_timestamp();     // [V1.8.2]
      key_time_pre = micros64();
      key_time_req = true;
    }
    else
    {
    // 큐잉
      memcpy(report_info.buf, p_data, length);
      qbufferWrite(&report_q, (uint8_t *)&report_info, 1);
    }
  }
  else
  {
    usbHidUpdateWakeUp(&USBD_Device);
  }

  return true;
}

bool usbHidSendReportEXK(uint8_t *p_data, uint16_t length)
{
  exk_report_info_t report_info;

  if (length > HID_EXK_EP_SIZE)
    return false;

  if (!USBD_is_suspended())
  {
    memcpy(hid_buf_exk, p_data, length);
    if (!USBD_HID_SendReportEXK((uint8_t *)hid_buf_exk, length))
    {
      report_info.len = length;
      memcpy(report_info.buf, p_data, length);
      qbufferWrite(&report_exk_q, (uint8_t *)&report_info, 1);
    }
  }
  else
  {
    usbHidUpdateWakeUp(&USBD_Device);
  }

  return true;
}

void usbHidMeasurePollRate(void)
{
  // [V1.8.1] 1초 윈도우는 8kHz 틱 카운트로 근사(HS 기준)
  if (rate_debug.poll_rate_measure_cnt >= 8000)
  {
    rate_debug.poll_rate_measure_cnt = 0;

    if (!rate_debug.window_primed) {
      // [V1.8.1] 프라임 단계: 카운터 리셋만 하고 출력 금지
      rate_debug.window_primed = true;

      // 1초 프라임: 집계만 하고 출력 금지
      rate_debug.data_in_cnt = 0;

      rate_debug.rate_time_sum       = 0;
      rate_debug.rate_time_min_check = 0xFFFFFFFFU;
      rate_debug.rate_time_max_check = 0;
      rate_debug.rate_valid_cnt      = 0;      // [V1.8.2]

      rate_debug.din_interval_sum        = 0;
      rate_debug.din_interval_cnt        = 0;
      rate_debug.din_interval_min_check  = 0;
      rate_debug.din_interval_max_check  = 0;
      rate_debug.din_discard_next        = true;   // [V1.8.2] 프라임 직후도 첫 샘플 버림
    } else {
      // 1초 창 집계 확정
      rate_debug.data_in_rate  = rate_debug.data_in_cnt;
      rate_debug.rate_time_min = (rate_debug.rate_valid_cnt > 0) ? rate_debug.rate_time_min_check : 0;
      rate_debug.rate_time_max = rate_debug.rate_time_max_check;
      rate_debug.rate_time_avg = (rate_debug.rate_valid_cnt > 0) ? (rate_debug.rate_time_sum / rate_debug.rate_valid_cnt) : 0;

      rate_debug.din_interval_avg = (rate_debug.din_interval_cnt > 0) ? (rate_debug.din_interval_sum / rate_debug.din_interval_cnt) : 0;
      rate_debug.din_interval_min = (rate_debug.din_interval_min_check == 0) ? 0 : rate_debug.din_interval_min_check;
      rate_debug.din_interval_max = rate_debug.din_interval_max_check;

      // 다음 창 준비
      rate_debug.data_in_cnt = 0;

      rate_debug.rate_time_sum       = 0;
      rate_debug.rate_time_min_check = 0xFFFFFFFFU;
      rate_debug.rate_time_max_check = 0;
      rate_debug.rate_valid_cnt      = 0;      // [V1.8.2]

      rate_debug.din_interval_sum        = 0;
      rate_debug.din_interval_cnt        = 0;
      rate_debug.din_interval_min_check  = 0;
      rate_debug.din_interval_max_check  = 0;
      rate_debug.din_discard_next        = true;   // [V1.8.2] 창 경계 첫 샘플 버림
    }
  }
  rate_debug.poll_rate_measure_cnt++;
}

void usbHidMeasureRateTime(void)
{
  // -------- 전송 지연(latency) 집계 --------
  if (rate_debug.rate_time_req)
  {
    uint64_t now_us = micros64();
    uint32_t dt = (uint32_t)(now_us - rate_debug.rate_time_pre);

    // [CHG] 동일-틱 가드 제거, 아주 짧은 비정상치만 드랍
    if (dt < 5U)  // 링버퍼/인터럽트 중첩 등으로 생길 수 있는 비정상치
    {
      rate_debug.invalid_samples_cnt++;
    }
    else
    {
      // [V1.8.3] 변경: 위상 보정 제거, 전송 지연을 '엔드-투-엔드(폴링 대기 포함)'
      //             로 집계하여 avg/hist가 key_time_log와 일치하도록 함
      uint32_t end2end = dt;
      rate_debug.rate_time_us  = end2end;
      rate_debug.rate_time_sum += end2end;
      if (rate_debug.rate_time_min_check > end2end) rate_debug.rate_time_min_check = end2end;
      if (rate_debug.rate_time_max_check < end2end) rate_debug.rate_time_max_check = end2end;

      uint32_t idx = constrain(end2end/10U, 0U, 99U);
      if (rate_debug.rate_his_buf[idx] < 0xFFFF) rate_debug.rate_his_buf[idx]++;

      rate_debug.rate_valid_cnt++;   // [V1.8.3] 분모: 유효 샘플 수
    }

    rate_debug.rate_time_req = false;
  }

  // --- 키 로깅(참고용) : 동일-틱 드랍 제거, 짧은 값만 거름 ---
  if (key_time_req)
  {
    uint64_t now2 = micros64();
    uint32_t dt2 = (uint32_t)(now2 - key_time_pre);
    key_time_req = false;

    if (dt2 < 5U)
    {
      rate_debug.invalid_samples_cnt++;
    }
    else
    {
      key_time_end = dt2;
      key_time_log[key_time_idx] = key_time_end;
      
      if (key_time_raw_req)
      {
        key_time_raw_req = false;
        key_time_raw_log[key_time_idx] = (uint32_t)(now2 - key_time_raw_pre);
        key_time_pre_log[key_time_idx] = (uint32_t)(key_time_pre - key_time_raw_pre);
      }
      else
      {
        key_time_raw_log[key_time_idx] = key_time_end;
      }

      key_time_idx = (key_time_idx + 1) % KEY_TIME_LOG_MAX;
      if (key_time_cnt < KEY_TIME_LOG_MAX)
      {
        key_time_cnt++;
      }
    }
  }
}

bool usbHidGetRateInfo(usb_hid_rate_info_t *p_info)
{
  p_info->freq_hz = rate_debug.data_in_rate;
  p_info->time_max = rate_debug.rate_time_max;
  p_info->time_min = rate_debug.rate_time_min;
  return true;
}

bool usbHidSetTimeLog(uint16_t index, uint32_t time_us)
{
  key_time_raw_pre = time_us;
  key_time_raw_req = true;
  return true;
}

__weak void usbHidSetStatusLed(uint8_t led_bits)
{

}

// [V1.5.0] 아래 함수들을 모두 삭제합니다.
/*
void usbHidInitTimer(void) { ... }
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) { ... }
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) { ... }
void TIM2_IRQHandler(void) { ... }
*/

// [V1.1.0] 리팩토링: 자동 안정성 검사 로직 분리
void usbHidProcessAutoStability(void)
{
  static uint32_t last_check_time = 0;
  if (millis() - last_check_time >= 1000)
  {
      last_check_time = millis();
      actual_polling_rate = sof_1s_cnt;
      sof_1s_cnt = 0;

      uint8_t current_mode = polling_rate_get();
      uint32_t actual_rate = usbHidGetActualRate();

      const uint32_t TARGET_RATE_8K = 7000;
      const uint32_t TARGET_RATE_4K = 3500;

      if (millis() - last_sof_time_ms > 500)
      {
          if (current_mode == POLLING_RATE_8K) {
              polling_rate_set(POLLING_RATE_4K);
              resetToReset();
          } else if (current_mode == POLLING_RATE_4K) {
              polling_rate_set(POLLING_RATE_1K);
              resetToReset();
          }
          return;
      }

      bool is_unstable = false;
      if ((current_mode == POLLING_RATE_8K && actual_rate > 0 && actual_rate < TARGET_RATE_8K) ||
          (current_mode == POLLING_RATE_4K && actual_rate > 0 && actual_rate < TARGET_RATE_4K)) {
          is_unstable = true;
      }

      if (is_unstable) {
          instability_counter++;
          stability_counter = 0;
          if (instability_counter >= INSTABILITY_THRESHOLD) {
              if (current_mode == POLLING_RATE_8K) {
                  polling_rate_set(POLLING_RATE_4K);
                  resetToReset();
              } else if (current_mode == POLLING_RATE_4K) {
                  polling_rate_set(POLLING_RATE_1K);
                  resetToReset();
              }
              instability_counter = 0;
          }
      } else {
          instability_counter = 0;
          if (current_mode != POLLING_RATE_8K) {
              if (stability_counter < STABILITY_THRESHOLD) {
                  stability_counter++;
              }
          } else {
            stability_counter = 0;
          }
      }
  }
}

// [V1.1.0] 리팩토링: 리포트 큐 처리 로직 분리
void usbHidProcessReportQueue(void)
{
  // Main HID Report Queue
  if (qbufferAvailable(&report_q) > 0 && p_hhid->state == USBD_HID_IDLE)
  {
    qbufferRead(&report_q, (uint8_t *)hid_buf, 1);
    USBD_HID_SendReport((uint8_t *)hid_buf, HID_KEYBOARD_REPORT_SIZE);

    hid_mark_send_timestamp();     // [V1.8.2]
    key_time_pre = micros64();
    key_time_req = true;
  }

  // EXK Report Queue
  if (qbufferAvailable(&report_exk_q) > 0)
  {
    if (p_hhid->state == USBD_HID_IDLE)
    {
      exk_report_info_t report_info;
      qbufferRead(&report_exk_q, (uint8_t *)&report_info, 1);
      memcpy(hid_buf_exk, report_info.buf, report_info.len);
      USBD_HID_SendReportEXK((uint8_t *)hid_buf_exk, report_info.len);
    }
  }

  // VIA Report Queue
  if (qbufferAvailable(&via_report_q) && (millis()-via_report_pre_time) >= via_report_time)
  {
    USBD_HandleTypeDef *pdev = &USBD_Device;
    qbufferRead(&via_report_q, (uint8_t *)via_hid_usb_report, 1);
    // [V1.8.4] FIX: OUT EP로의 Transmit 시도는 논리적 오류.
    // VIA 응답(IN 송신)은 via.c의 raw_hid_send()가 단일 경로로 담당한다.
    // 따라서 여기서는 OUT EP의 수신만 무한 준비(재-ARM)한다.
    USBD_LL_PrepareReceive(pdev, HID_VIA_EP_OUT, via_hid_usb_report, sizeof(via_hid_usb_report));  // [V1.8.4]
  }
}


// [V1.5.0] HAL_TIM_PWM_PulseFinishedCallback 함수를 아래 함수로 대체합니다.
// 이 함수는 이제 micros.c의 콜백을 통해 125us 마다 호출됩니다.
static void usbHidTimerCallback(void)
{
  // [V1.8.1] 동일-틱 가드를 위한 틱 증가
  g_usb_tick_id++;

  // 1. 자동 안정성 모드 로직 실행
  usbHidProcessAutoStability();

  // 2. 모든 리포트 큐 처리
  usbHidProcessReportQueue();

  // 3. 'usbhid rate' 디버깅을 위한 통계 업데이트
  usbHidMeasurePollRate();
  // SOF 관련 카운트/간격은 SOF ISR에서만 갱신한다.
  rate_debug.timer_cnt++;
}


// [V1.8.2] 수정 `usbhid rate` 테스트 시작 시 모든 관련 통계 변수를 초기화하는 함수
void usbHidResetDebugCounters(void)
{
  memset(&rate_debug, 0, sizeof(usb_hid_rate_debug_t));
  rate_debug.rate_time_min_check   = 0xFFFFFFFFU;
  rate_debug.window_primed         = false;
  rate_debug.expected_interval_us  = usbHidExpectedIntervalUs();

  // Δt_in 관련(경계 보정)
  rate_debug.din_prev_time_us      = 0;
  rate_debug.din_discard_next      = true;     // [V1.8.2] 리셋 후 첫 샘플 버림

  // 지연 샘플 분모
  rate_debug.rate_valid_cnt        = 0;        // [V1.8.2]
}

// Getter 함수 구현부
bool usbHidIsCliStatusEnabled(void)
{
  return cli_status_enabled;
}

uint32_t usbHidGetActualRate(void)
{
  return actual_polling_rate;
}

uint32_t usbHidGetStabilityCounter(void)
{
  return stability_counter;
}

// [V1.8.1] 얇은 getter 추가: SOF dT(ms) - ap.c 실시간 로그에서 사용
uint32_t usbHidGetSofDtMs(void)
{
  return millis() - last_sof_time_ms;        // last_sof_time_ms는 SOF ISR에서 갱신
}

#ifdef _USE_HW_CLI
void cliCmd(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    cliPrintf("USB HID Interface for QMK\r\n");
    ret = true;
  }

  if (args->argc >= 1 && args->isStr(0, "status") == true)
  {
    if (args->argc == 2 && args->isStr(1, "on"))
    {
      cli_status_enabled = true;
      cliPrintf("Real-time status logging: ON\r\n");
    }
    else if (args->argc == 2 && args->isStr(1, "off"))
    {
      cli_status_enabled = false;
      cliPrintf("Real-time status logging: OFF\r\n");
    }
    else
    {
      uint8_t config_mode = polling_rate_get();
      const char *config_str;
      switch(config_mode)
      {
        case POLLING_RATE_8K: config_str = "8000 Hz"; break;
        case POLLING_RATE_4K: config_str = "4000 Hz"; break;
        case POLLING_RATE_1K: config_str = "1000 Hz"; break;
        default:              config_str = "Unknown"; break;
      }
      uint32_t actual_rate = usbHidGetActualRate();
      uint32_t sof_delta = millis() - last_sof_time_ms;

      cliPrintf("--- Polling Rate Status ---\r\n");
      cliPrintf(" Config : %s\r\n", config_str);
      cliPrintf(" Actual : %d Hz\r\n", actual_rate);
      cliPrintf(" SOF dT : %d ms\r\n", sof_delta);
      cliPrintf("---------------------------\r\n");
      cliPrintf(" Instability Cnt : %d / %d\r\n", instability_counter, INSTABILITY_THRESHOLD);
      cliPrintf(" Stability Cnt   : %d / %d\r\n", stability_counter, STABILITY_THRESHOLD);
      cliPrintf("---------------------------\r\n");
    }
    ret = true;
  }

  if (args->argc >= 1 && args->isStr(0, "rate") == true)
  {
    uint32_t pre_time;
    uint32_t pre_time_key;
    uint32_t key_send_cnt = 0;

    // [V1.8.1] 프라임 로직에 맞게 첫 1초는 출력 금지(카운터/통계 초기화만 수행)
    usbHidResetDebugCounters();

    pre_time = millis();
    pre_time_key = millis();
    while(cliKeepLoop())
    {
      if (millis()-pre_time_key >= 2 && key_send_cnt < 50)
      {
        uint8_t buf[HID_KEYBOARD_REPORT_SIZE];
        memset(buf, 0, HID_KEYBOARD_REPORT_SIZE);
        pre_time_key = millis();
        usbHidSendReport(buf, HID_KEYBOARD_REPORT_SIZE);
        key_send_cnt++;
      }
      if (millis()-pre_time >= 1000)
      {
        pre_time = millis();

        if (rate_debug.window_primed && rate_debug.data_in_rate > 0) // [V1.8.2] 빈 윈도우 차단
        {
          // [V1.8.1] fixed: single cliPrintf with matching argument count (ASCII: dt_in)
          cliPrintf(
            "hid rate %d Hz, avg %4d us, max %4d us, min %d us, "
            "sof_max_dt %d us, expected %d us, "
            "dt_in avg %4d us (min %d, max %d), "
            "drop(same/invalid) %d/%d, in_invalid %d\r\n",
            rate_debug.data_in_rate,
            rate_debug.rate_time_avg,
            rate_debug.rate_time_max,
            rate_debug.rate_time_min,
            rate_debug.sof_interval_max_us,
            rate_debug.expected_interval_us,
            rate_debug.din_interval_avg,
            rate_debug.din_interval_min,
            rate_debug.din_interval_max,
            rate_debug.same_tick_cnt,
            rate_debug.invalid_samples_cnt,
            rate_debug.din_invalid_cnt
          );

          for (int i=0; i<10; i++)
          {
            cliPrintf("%d us\r\n",key_time_log[i]);
          }
          cliPrintf("sof/tim cnt : %d/%d\r\n", rate_debug.sof_cnt, rate_debug.timer_cnt);
        }

        // 다음 1초 측정을 위해 초당 카운터만 리셋
        rate_debug.timer_cnt = 0;
        key_send_cnt = 0;
        rate_debug.sof_cnt = 0;
        rate_debug.sof_interval_max_us = 0; // 1초마다 최대값도 리셋
        // [V1.8.1] 드롭 카운터는 누적 유지(진단용), 필요시 여기서 리셋 가능
        // rate_debug.same_tick_cnt = 0;
        // rate_debug.invalid_samples_cnt = 0;
      }
    }

    if (args->argc == 2 && args->isStr(1, "his"))
    {
      for (int i=0; i<100; i++)
      {
        cliPrintf("%d %d\r\n", i, rate_debug.rate_his_buf[i]);
      }
    }
    else if (args->argc == 2 && args->isStr(1, "inhis")) // [V1.8.1] Δt_in 전용 히스토그램
    {
      for (int i=0; i<100; i++)
      {
        cliPrintf("%d %d\r\n", i, rate_debug.din_his_buf[i]);
      }
    }
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "log") && args->isStr(1, "clear"))
  {
    key_time_idx = 0;
    key_time_cnt = 0;
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "log") == true)
  {
    uint16_t index;
    uint16_t time_max[3] = {0, 0, 0};
    uint16_t time_min[3] = {0xFFFF, 0xFFFF, 0xFFFF};
    uint16_t time_sum[3] = {0, 0, 0};
    for (int i = 0; i < key_time_cnt; i++)
    {
      if (key_time_cnt == KEY_TIME_LOG_MAX)
        index = (key_time_idx + i) % KEY_TIME_LOG_MAX;
      else
        index = i;
      cliPrintf("%2d: %3d us, raw : %3d us, %d\r\n", i, key_time_log[index], key_time_raw_log[index], key_time_pre_log[index]);
      for (int j=0; j<3; j++)
      {
        uint16_t data;
        if (j == 0) data = key_time_log[index];
        else if (j == 1) data = key_time_raw_log[index];
        else data = key_time_pre_log[index];
        time_sum[j] += data;
        if (data > time_max[j]) time_max[j] = data;
        if (data < time_min[j]) time_min[j] = data;
      }
    }
    cliPrintf("\r\n");
    if (key_time_cnt > 0)
    {
      cliPrintf("avg : %3d us %3d us %3d us\r\n", time_sum[0] / key_time_cnt, time_sum[1] / key_time_cnt, time_sum[2] / key_time_cnt);
      cliPrintf("max : %3d us %3d us %3d us\r\n", time_max[0], time_max[1], time_max[2]);
      cliPrintf("min : %3d us %3d us %3d us\r\n", time_min[0], time_min[1], time_min[2]);
    }
    ret = true;
  }

  // [V1.6.3] RCC 레지스터 확인용 명령어 수정 (H7RSxx 시리즈에 맞게)
  if (args->argc == 2 && args->isStr(0, "check") && args->isStr(1, "reg"))
  {
    // APBCFGR 레지스터의 전체 값을 16진수로 출력
    uint32_t apbcfgr_val = RCC->APBCFGR;
    cliPrintf("--- RCC Register Check ---\r\n");
    cliPrintf(" RCC->APBCFGR (raw hex): 0x%08X\r\n", apbcfgr_val);

    // PPRE1 비트 필드(APB1 Prescaler)만 추출
    uint32_t ppre1_bits = (apbcfgr_val & RCC_APBCFGR_PPRE1_Msk) >> RCC_APBCFGR_PPRE1_Pos;
    cliPrintf(" PPRE1 field (bits 14-12): 0b");
    cliPrintf("%d", (ppre1_bits >> 2) & 1);
    cliPrintf("%d", (ppre1_bits >> 1) & 1);
    cliPrintf("%d\r\n", (ppre1_bits >> 0) & 1);

    cliPrintf(" Interpretation: ");
    switch(ppre1_bits)
    {
      case 0b000: case 0b001: case 0b010: case 0b011:
        cliPrintf("HCLK not divided (/1)\r\n");
        cliPrintf(" -> Timer Clock is PCLK1\r\n");
        break;
      case 0b100:
        cliPrintf("HCLK divided by 2 (/2)\r\n");
        cliPrintf(" -> Timer Clock is PCLK1 * 2\r\n");
        break;
      case 0b101:
        cliPrintf("HCLK divided by 4 (/4)\r\n");
        cliPrintf(" -> Timer Clock is PCLK1 * 2\r\n");
        break;
      case 0b110:
        cliPrintf("HCLK divided by 8 (/8)\r\n");
        cliPrintf(" -> Timer Clock is PCLK1 * 2\r\n");
        break;
      case 0b111:
        cliPrintf("HCLK divided by 16 (/16)\r\n");
        cliPrintf(" -> Timer Clock is PCLK1 * 2\r\n");
        break;
      default:
        cliPrintf("Unknown value\r\n");
        break;
    }
    cliPrintf("--------------------------\r\n");

    ret = true;
  }

  if (ret == false)
  {
    // [V1.8.1] Updated help: prime window, separated histograms, dt_in stats
    cliPrintf("usbhid info\r\n");
    cliPrintf("usbhid status [on|off]\r\n");
    cliPrintf("  - Enable/disable real-time status logging.\r\n");                        // [V1.8.1]

    cliPrintf("usbhid rate\r\n");
    cliPrintf("  - 1-second windows; the FIRST second is a prime window (no print).\r\n"); // [V1.8.1]
    cliPrintf("  - Prints: rate Hz, latency avg/max/min, sof_max_dt, expected_interval_us,\r\n"); // [V1.8.1]
    cliPrintf("            dt_in avg/min/max, drop(same/invalid), in_invalid.\r\n");       // [V1.8.1]
    cliPrintf("    * latency: SendReport -> IN complete (end-to-end, includes polling wait)\r\n");    // [V1.8.3]
    cliPrintf("    * dt_in  : consecutive IN-complete deltas (EP polling interval)\r\n");  // [V1.8.1]

    cliPrintf("usbhid rate his\r\n");
    cliPrintf("  - Latency histogram (end-to-end), 100 bins, 10us/bin, 0..990us.\r\n");    // [V1.8.3]

    cliPrintf("usbhid rate inhis\r\n");
    cliPrintf("  - dt_in histogram (IN-to-IN), 100 bins, 10us/bin, 0..990us.\r\n");        // [V1.8.1]

    cliPrintf("usbhid log\r\n");
    cliPrintf("  - Shows last %d key-time samples; same-tick/<20us samples are excluded.\r\n", KEY_TIME_LOG_MAX); // [V1.8.1]

    cliPrintf("usbhid log clear\r\n");
    cliPrintf("usbhid check reg\r\n");
  }
}
#endif