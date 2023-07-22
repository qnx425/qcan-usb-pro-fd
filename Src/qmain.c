#include <stdio.h>
#include "stm32h7xx_hal.h"
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "usb_device.h"

#include "main.h"

extern void SystemClock_Config(void);

int __io_putchar( int ch ) {
	extern UART_HandleTypeDef  huart3;

	HAL_UART_Transmit( &huart3, (const uint8_t *)&ch, 1, 100 );
	return 0;
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM2_Init();

#if 0
  pcan_led_init();
  pcan_led_set_mode( LED_STAT, LED_MODE_BLINK_FAST, 0xFFFFFFFF );
#endif

  pcan_can_init();
  pcan_protocol_init();
  pcan_usb_device_init();
  
  for(;;)
  {
    pcan_usb_device_poll();
    pcan_protocol_poll();
#if 0
    pcan_led_poll();
#endif
  }
}
