#include "micros.h"
#include "bsp.h" // TIM2_IRQHandler 를 위해 bsp.h 또는 관련 헤더 포함


#ifdef _USE_HW_MICROS

static TIM_HandleTypeDef  TimHandle;
static void (*micros_cb)(void) = NULL;
volatile uint64_t high_res_timestamp_us = 0;


bool microsInit(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  TimHandle.Instance = TIM2;

  uint32_t pclk1_freq = HAL_RCC_GetPCLK1Freq();
  uint32_t timer_clock = pclk1_freq;

  // [V1.6.2] Corrected for STM32H7RSxx series (uses APBCFGR register)
  if ((RCC->APBCFGR & RCC_APBCFGR_PPRE1_2) != 0)
  {
    timer_clock *= 2;
  }

  // 1MHz 카운터 클럭 생성 (1tick = 1us)
  uint32_t prescaler_value = (timer_clock / 1000000) - 1;

  TimHandle.Init.Prescaler      = prescaler_value;
  TimHandle.Init.CounterMode    = TIM_COUNTERMODE_UP;
  TimHandle.Init.Period         = 125 - 1; 
  TimHandle.Init.ClockDivision  = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  return true;
}

uint32_t micros(void)
{
  return TimHandle.Instance->CNT;
}

uint64_t micros64(void)
{
  uint64_t high_part;
  uint32_t low_part;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();

  high_part = high_res_timestamp_us;
  low_part = TimHandle.Instance->CNT;

  // 인터럽트가 방금 발생하여 high_part가 갱신되었는지 확인 (레이스 컨디션 방지)
  // 만약 업데이트 플래그(UIF)가 세트되어 있고, low_part가 매우 작은 값이라면
  // high_part를 읽은 직후에 오버플로우가 발생한 것이므로, high_part를 다시 읽음
  if (((TimHandle.Instance->SR & TIM_SR_UIF) != 0) && (low_part < (TimHandle.Init.Period / 2)))
  {
    high_part = high_res_timestamp_us;
  }

  __set_PRIMASK(primask);

  return high_part + low_part;
}

void microsSetCallback(void (*p_func)(void))
{
  micros_cb = p_func;
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    high_res_timestamp_us += 125;

    if (micros_cb != NULL)
    {
      micros_cb();
    }
  }
}

#endif