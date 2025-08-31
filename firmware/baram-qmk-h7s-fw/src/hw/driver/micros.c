// hw/driver/micros.c

#include "micros.h"
#include "bsp.h" 

#ifdef _USE_HW_MICROS

static TIM_HandleTypeDef  TimHandle;
static void (*micros_cb)(void) = NULL;
volatile uint64_t high_res_timestamp_us = 0;


bool microsInit(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();

  TimHandle.Instance = TIM2;

  // [V1.7.4] Revert to the 300MHz timer clock calculation, which is proven correct by the 4kHz result of V1.6.4.
  // The goal is to generate a precise 8kHz interrupt from this 300MHz source.
  uint32_t pclk1_freq = HAL_RCC_GetPCLK1Freq(); // Should be 150MHz
  uint32_t timer_clock = pclk1_freq;

  // The timer clock doubles if the PCLK divider is not 1.
  if ((RCC->APBCFGR & RCC_APBCFGR_PPRE1_2) != 0)
  {
    timer_clock *= 2; // timer_clock is now correctly 300MHz
  }
  
  // Calculate prescaler for a 1MHz counter clock (1 tick = 1us)
  uint32_t prescaler_value = (timer_clock / 1000000) - 1; // (300,000,000 / 1,000,000) - 1 = 299

  TimHandle.Init.Prescaler      = prescaler_value;
  TimHandle.Init.CounterMode    = TIM_COUNTERMODE_UP;
  TimHandle.Init.Period         = 125 - 1; // 125us overflow for a precise 8kHz interrupt
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

/**
  * @brief  [V1.7.4] Get a 64-bit microsecond timestamp that is safe from timer overflow
  *         and immune to race conditions between the ISR and the main thread.
  * @retval Current microsecond timestamp
  */
uint64_t micros64(void)
{
    uint64_t high_part_1, high_part_2;
    uint32_t low_part;

    do {
        high_part_1 = high_res_timestamp_us;
        low_part = TimHandle.Instance->CNT;
        high_part_2 = high_res_timestamp_us;
    } while (high_part_1 != high_part_2);

    return high_part_2 + low_part;
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
    // The ISR now correctly fires every 125us, so we add 125.
    high_res_timestamp_us += 125;

    if (micros_cb != NULL)
    {
      micros_cb();
    }
  }
}

#endif