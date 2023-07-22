#include <assert.h>
#include <stm32h7xx_hal.h>
#include "pcanpro_timestamp.h"

void pcan_timestamp_init( void )
{
  //HAL_GetTick must not return 0 to use LED events in early stage.
  //Let's increment the value here by one.
  HAL_IncTick();
}

uint32_t pcan_timestamp_millis( void )
{
  return HAL_GetTick();
}

uint32_t pcan_timestamp_us( void )
{
  return TIM2->CNT;
}
