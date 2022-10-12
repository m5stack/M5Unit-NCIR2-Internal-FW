/**
  ******************************************************************************
  * File Name          : encoder.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include <stdlib.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/

uint8_t led_num;
uint32_t *color_buf = NULL;
void color_set_single(uint32_t color);

void restart(void) {
  for (uint8_t i = 0; i < 113; i++) {
    delay_600ns();
  }
}

void sk6812_init(uint8_t num) {
  color_buf = (uint32_t *)calloc(num, sizeof(uint32_t));
  led_num = num;
}

void neopixel_set_color(uint8_t num, uint32_t color) {
	uint8_t rled = (color >> 24) & 0xff;
	uint8_t gled = (color >> 16) & 0xff;
	uint8_t bled = (color >> 8)  & 0xff;
	color_buf[num] = gled << 16 | rled << 8 | bled;
}

void neopixel_show(void) {
  __disable_irq();
  color_set_single(color_buf[0]);
  __enable_irq();
  restart();
}

void color_set_single(uint32_t color) {
  for (uint8_t i = 0; i < 24; i++) {
    if (color & (1 << (23 - i))) {
      out_bit_high();
    }
    else {
      out_bit_low();
    }
  }
}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
