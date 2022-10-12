/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_gpio.h"
#include "i2c_ex.h"
#include "flash.h"
#include "mlx90614.h"
#include "ws2812.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 2
#define I2C_ADDRESS 0x5A
#define HIGH_TEMP_ALARM_TIMES 5
#define LOW_TEMP_ALARM_TIMES 5
#define NORMAL_TEMP_TIMES 5
#define TEMP_ALARM_FILTER_TIMEOUT NORMAL_TEMP_TIMES*3
#define BUTTON_FILTER 500
#define BUTTON_FILTER_TIMEROUT BUTTON_FILTER*3
#define FLASH_DATA_SIZE 36
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint8_t eeData[4] = {0};
uint8_t i2c_address[1] = {0};
uint8_t flash_data[FLASH_DATA_SIZE] = {0};
volatile uint8_t fm_version = FIRMWARE_VERSION;

volatile int16_t temperature = 0;
volatile int16_t temperature_soc = 0;
volatile uint16_t emissivity = 0;
volatile uint8_t emissivity_set_flag = 0;

/*-----------------------------------------*/
//high and low alarm enable flag
volatile uint16_t high_temp_alarm_flag = 0;
volatile uint16_t low_temp_alarm_flag= 0;
volatile uint8_t last_alarm_flag = 0;
/*-----------------------------------------*/
//high and low alarm value flag
volatile int16_t high_temp_alarm_value = 0;
volatile int16_t low_temp_alarm_value= 0;
/*-----------------------------------------*/
//high and low alarm fliter
volatile uint16_t high_temp_alarm_filter = 0;
volatile uint16_t normal_high_temp_alarm_filter = 0;
volatile uint16_t low_temp_alarm_filter = 0;
volatile uint16_t normal_low_temp_alarm_filter = 0;
/*-----------------------------------------*/
//buzz pwm control
volatile uint8_t high_buzz_flag = 0;
volatile uint8_t low_buzz_flag = 0;
volatile uint8_t low_pwm_start_flag = 0;
volatile uint8_t low_pwm_stop_flag = 0;
volatile uint8_t high_pwm_start_flag = 0;
volatile uint8_t high_pwm_stop_flag = 0;
/*-----------------------------------------*/
//buzz alarm control
volatile uint16_t high_buzz_freq = 0;
volatile uint16_t high_buzz_inter = 0;
volatile uint8_t high_buzz_duty = 0;
volatile uint16_t low_buzz_freq = 0;
volatile uint16_t low_buzz_inter = 0;
volatile uint8_t low_buzz_duty = 0;
/*-----------------------------------------*/
//buzz mannual control
volatile uint16_t buzz_freq = 0;
volatile uint8_t buzz_duty = 0;
volatile uint8_t buzz_manual_flag = 0;
volatile uint8_t buzz_enable = 0;


/*-----------------------------------------*/
//led mannual control
volatile uint32_t manual_led_rgb = 0;
/*-----------------------------------------*/
//led alarm control
// volatile uint32_t high_temp_led_rgb = 0;
// volatile uint32_t low_temp_led_rgb = 0;
uint32_t temp_led_rgb[2] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
// static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void user_delaynus_tim(uint32_t ns)
{
  uint16_t differ = 0xffff - ns/4 - 5;
  __HAL_TIM_SetCounter(&htim14, differ);
  HAL_TIM_Base_Start(&htim14);

  while( differ<0xffff-5) {
    differ = __HAL_TIM_GetCounter(&htim14);
  }
  HAL_TIM_Base_Stop(&htim14);
}

void user_tim3_init(uint32_t period, uint32_t pulse)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = period;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

void user_tim16_init(uint32_t period)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = period;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

void user_tim17_init(uint32_t period)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 48000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = period;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

void user_i2c_init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = i2c_address[0]<<1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void cover_data_to_flash(void)
{
  flash_data[0] = I2C_ADDRESS;
  flash_data[1] = 0;
  flash_data[2] = low_temp_alarm_value & 0xff;
  flash_data[3] = (low_temp_alarm_value >> 8) & 0xff;
  flash_data[4] = high_temp_alarm_value & 0xff;
  flash_data[5] = (high_temp_alarm_value >> 8) & 0xff;
  flash_data[6] = temp_led_rgb[0] & 0xff;
  flash_data[7] = (temp_led_rgb[0] >> 8) & 0xff;
  flash_data[8] = (temp_led_rgb[0] >> 16) & 0xff;
  flash_data[9] = (temp_led_rgb[0] >> 24) & 0xff;
  flash_data[10] = temp_led_rgb[1] & 0xff;
  flash_data[11] = (temp_led_rgb[1] >> 8) & 0xff;
  flash_data[12] = (temp_led_rgb[1] >> 16) & 0xff;
  flash_data[13] = (temp_led_rgb[1] >> 24) & 0xff;
  flash_data[14] = low_buzz_freq & 0xff;
  flash_data[15] = (low_buzz_freq >> 8) & 0xff;
  flash_data[16] = low_buzz_inter & 0xff;
  flash_data[17] = (low_buzz_inter >> 8) & 0xff;
  flash_data[18] = low_buzz_duty;
  flash_data[19] = 0;    
  flash_data[20] = high_buzz_freq & 0xff;
  flash_data[21] = (high_buzz_freq >> 8) & 0xff;
  flash_data[22] = high_buzz_inter & 0xff;
  flash_data[23] = (high_buzz_inter >> 8) & 0xff;
  flash_data[24] = high_buzz_duty;
  flash_data[25] = 0;  
  flash_data[26] = buzz_freq & 0xff;
  flash_data[27] = (buzz_freq >> 8) & 0xff;
  flash_data[28] = buzz_duty;
  flash_data[29] = 0;  
  flash_data[30] = manual_led_rgb & 0xff;
  flash_data[31] = (manual_led_rgb >> 8) & 0xff;
  flash_data[32] = (manual_led_rgb >> 16) & 0xff;
  flash_data[33] = (manual_led_rgb >> 24) & 0xff;  
  flash_data[34] = emissivity;
  flash_data[35] = (emissivity >> 8) & 0xff;     
}

void init_flash_data(void) 
{   
  if (!(readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE))) {
    i2c_address[0] = I2C_ADDRESS;
    cover_data_to_flash();     
    writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
  } else {
    i2c_address[0] = flash_data[0];
    low_temp_alarm_value = flash_data[2] | (flash_data[3] << 8);
    high_temp_alarm_value = flash_data[4] | (flash_data[5] << 8);
    temp_led_rgb[0] = flash_data[6] | (flash_data[7] << 8) | (flash_data[8] << 16) | (flash_data[9] << 24);
    temp_led_rgb[1] = flash_data[10] | (flash_data[11] << 8) | (flash_data[12] << 16) | (flash_data[13] << 24);
    low_buzz_freq = flash_data[14] | (flash_data[15] << 8);
    low_buzz_inter = flash_data[16] | (flash_data[17] << 8);
    low_buzz_duty = flash_data[18];
    high_buzz_freq = flash_data[20] | (flash_data[21] << 8);
    high_buzz_inter = flash_data[22] | (flash_data[23] << 8); 
    high_buzz_duty = flash_data[24];   
    buzz_freq = flash_data[26] | (flash_data[27] << 8);
    buzz_duty = flash_data[28];  
    manual_led_rgb = flash_data[30] | (flash_data[31] << 8) | (flash_data[32] << 16) | (flash_data[33] << 24);
    // emissivity = flash_data[34] | (flash_data[35] << 8);
  }
}

void set_pwm(uint16_t freq, uint8_t duty)
{
  if (freq > 10000)
    return;
  uint32_t period, pulse;
  period = 1000000 / freq;
  pulse = (uint32_t)((float)duty/255.0f*period);
  __HAL_TIM_SET_AUTORELOAD(&htim3, period);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse);  
  // user_tim3_init(period, pulse);
}

uint8_t read_button_status(void)
{
  uint8_t button_status = 0; 
  uint8_t last_button_status = 0;
  uint16_t counter = 0;

  last_button_status = HAL_GPIO_ReadPin(GPIOA,BTN_Pin);
  for (uint16_t i = 0; i < BUTTON_FILTER_TIMEROUT; i++) {  
    button_status = HAL_GPIO_ReadPin(GPIOA,BTN_Pin);
    if (button_status == last_button_status) {
      counter++;
    }
    else {
      last_button_status = button_status;
      counter = 0;
    }
    if (counter >= BUTTON_FILTER) {
      return button_status;
    }
  }
  //TODO: Just for debug
  return 1;  
}

int16_t get_temperature(void)
{
  uint32_t temp;
  uint16_t result = eeData[0] | (eeData[1] << 8);
  temp = result * 2 - 27315;
  return (int16_t)temp;
}

void set_led(uint32_t color)
{
  neopixel_set_color(0, color);
  neopixel_show();
}

void i2c1_receive_callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t rx_buf[16];
  uint8_t tx_buf[16];
  uint8_t rx_mark[16] = {0};
  if (len == 1 && (rx_data[0] == 0 || rx_data[0] == 0x01))
  {
    tx_buf[0] = temperature & 0xff;
    tx_buf[1] = (temperature >> 8) & 0xff;
    i2c1_set_send_data(tx_buf, 2);
  }
  else if (len == 1 && (rx_data[0] == 0x10 || rx_data[0] == 0x11))
  {
    i2c1_set_send_data((uint8_t *)&emissivity, 2);
  }
  else if (len > 1 && (rx_data[0] >= 0x10) && (rx_data[0] <= 0x11))
  {
    if (len == 3) {
      emissivity = rx_data[1] | (rx_data[2] << 8);
      emissivity_set_flag = 1;
    }
  }
  else if (len == 1 && ((rx_data[0] >= 0x20) &&(rx_data[0] <= 0x24)))
  {
    if (rx_data[0] == 0x20 || rx_data[0] == 0x21) {
      i2c1_set_send_data((uint8_t *)&low_temp_alarm_value, 2);
    } else if (rx_data[0] == 0x22 || rx_data[0] == 0x23) {
      i2c1_set_send_data((uint8_t *)&high_temp_alarm_value, 2);
    }
  }
  else if (len > 1 && ((rx_data[0] >= 0x20) &&(rx_data[0] <= 0x24)))
  {
    if (len <= 6) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x20+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x20+i] = 1;     
      }
      if (rx_mark[0] && rx_mark[1]) {
        low_temp_alarm_value = (rx_buf[0] | (rx_buf[1] << 8));
        low_temp_alarm_flag = 1;
      }
      if (rx_mark[2] && rx_mark[3]) {
        high_temp_alarm_value = (rx_buf[2] | (rx_buf[3] << 8));
        high_temp_alarm_flag = 1;
      }
    }
  }
  else if (len == 1 && ((rx_data[0] >= 0x30) &&(rx_data[0] <= 0x35)))
  {
    if (rx_data[0] == 0x30 || rx_data[0] == 0x31 || rx_data[0] == 0x32) {
      tx_buf[0] = (temp_led_rgb[0] >> 24) & 0xff;
      tx_buf[1] = (temp_led_rgb[0] >> 16) & 0xff;
      tx_buf[2] = (temp_led_rgb[0] >> 8) & 0xff;       
    } else if (rx_data[0] == 0x33 || rx_data[0] == 0x34 || rx_data[0] == 0x35) {
      tx_buf[0] = (temp_led_rgb[1] >> 24) & 0xff;
      tx_buf[1] = (temp_led_rgb[1] >> 16) & 0xff;
      tx_buf[2] = (temp_led_rgb[1] >> 8) & 0xff;       
    }
    i2c1_set_send_data(tx_buf, 3);
  }
  else if (len > 1 && ((rx_data[0] >= 0x30) &&(rx_data[0] <= 0x35)))
  {
    if (len <= 7) {
      for (int i = 0; i < (len - 1) / 3; i++) {
        temp_led_rgb[((rx_data[0]+i*3 - 0x30)/3)] = (rx_data[1+i*3] << 24) | (rx_data[2+i*3] << 16) | (rx_data[3+i*3] << 8); 
      }      
    }
  }
  else if (len == 1 && ((rx_data[0] >= 0x40) &&(rx_data[0] <= 0x49)))
  {
    if (rx_data[0] == 0x40 || rx_data[0] == 0x41) {
      i2c1_set_send_data((uint8_t *)&low_buzz_freq, 2);
    } else if(rx_data[0] == 0x42 || rx_data[0] == 0x43) {
      i2c1_set_send_data((uint8_t *)&low_buzz_inter, 2);
    } else if(rx_data[0] == 0x44) {
      i2c1_set_send_data((uint8_t *)&low_buzz_duty, 1);
    } else if (rx_data[0] == 0x45 || rx_data[0] == 0x46) {
      i2c1_set_send_data((uint8_t *)&high_buzz_freq, 2);
    } else if(rx_data[0] == 0x47 || rx_data[0] == 0x48) {
      i2c1_set_send_data((uint8_t *)&high_buzz_inter, 2);
    } else if(rx_data[0] == 0x49) {
      i2c1_set_send_data((uint8_t *)&high_buzz_duty, 1);
    } 
  }
  else if (len > 1 && ((rx_data[0] >= 0x40) &&(rx_data[0] <= 0x49)))
  {
    if (len <= 11) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x40+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x40+i] = 1;     
      }
      if (rx_mark[0] && rx_mark[1]) {
        low_buzz_freq = (rx_buf[0] | (rx_buf[1] << 8));
      }
      if (rx_mark[2] && rx_mark[3]) {
        low_buzz_inter = (rx_buf[2] | (rx_buf[3] << 8));
        // user_tim17_init(low_buzz_inter);
      }
      if (rx_mark[4]) {
        low_buzz_duty = rx_buf[4];
      }
      if (rx_mark[5] && rx_mark[6]) {
        high_buzz_freq = (rx_buf[5] | (rx_buf[6] << 8));
      }
      if (rx_mark[7] && rx_mark[8]) {
        high_buzz_inter = (rx_buf[7] | (rx_buf[8] << 8));
        // user_tim16_init(high_buzz_inter);
      }
      if (rx_mark[9]) {
        high_buzz_duty = rx_buf[9];
      }
    }
  }  
  else if (len == 1 && ((rx_data[0] >= 0x50) &&(rx_data[0] <= 0x53)))
  {
    if (rx_data[0] == 0x50 || rx_data[0] == 0x51) {
      i2c1_set_send_data((uint8_t *)&buzz_freq, 2);
    } else if (rx_data[0] == 0x52) {
      i2c1_set_send_data((uint8_t *)&buzz_duty, 1);
    } else {
      i2c1_set_send_data((uint8_t *)&buzz_enable, 1);
    }
  }
  else if (len > 1 && ((rx_data[0] >= 0x50) &&(rx_data[0] <= 0x53)))
  {
    if (len <= 5) {
      for(int i = 0; i < len - 1; i++) {
        rx_buf[rx_data[0]-0x50+i] = rx_data[1+i];
        rx_mark[rx_data[0]-0x50+i] = 1;     
      }
      if (rx_mark[0] && rx_mark[1]) {
        buzz_freq = (rx_buf[0] | (rx_buf[1] << 8));
      }
      if (rx_mark[2]) {
        buzz_duty = rx_buf[2];
      }
      if (rx_mark[3]) {
        
        buzz_enable = rx_buf[3];
        if (buzz_enable) {
          set_pwm(buzz_freq, buzz_duty); 
          HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
          buzz_manual_flag = 1;
        } else {
          HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
          buzz_manual_flag = 0;
        }
      }
    }
  }
	else if (len > 1 && ((rx_data[0] >= 0x60) & (rx_data[0] <= 0x62))) 
	{
    if (len == 4) {
      manual_led_rgb = (rx_data[1] << 24) | (rx_data[2] << 16) | (rx_data[3] << 8);
      neopixel_set_color(0, manual_led_rgb);
      neopixel_show(); 
    }
  }
	else if (len == 1 && ((rx_data[0] >= 0x60) & (rx_data[0] <= 0x62))) 
	{
    tx_buf[0] = (manual_led_rgb >> 24) & 0xff;
    tx_buf[1] = (manual_led_rgb >> 16) & 0xff;
    tx_buf[2] = (manual_led_rgb >> 8) & 0xff;     
    i2c1_set_send_data(tx_buf, 3);
  }     
	else if (len == 1 && (rx_data[0] == 0x70)) 
	{
    tx_buf[0] = read_button_status(); 
    i2c1_set_send_data(tx_buf, 1);
  }     
	else if (len > 1 && (rx_data[0] == 0x80)) 
	{
    if (len == 2 && rx_data[1] == 1) {
      if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
        cover_data_to_flash();
        writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
      } 
    }
  }
  else if (len == 1 && (rx_data[0] == 0x90 || rx_data[0] == 0x91))
  {
    i2c1_set_send_data((uint8_t *)&temperature_soc, 2);
  }     
  else if (len == 1 && (rx_data[0] == 0xFE))
  {
    i2c1_set_send_data((uint8_t *)&fm_version, 1);
  }
  else if (len > 1 && (rx_data[0] == 0xFF))
  {
    if (len == 2) {
      if (rx_data[1] < 128) {
        if (readPackedMessageFromFlash(flash_data, FLASH_DATA_SIZE)) {
          i2c_address[0] = rx_data[1];
          flash_data[0] = i2c_address[0];
          flash_data[1] = 0;
          writeMessageToFlash(flash_data , FLASH_DATA_SIZE);
          user_i2c_init();
        } 
      }
    }     
  }
  else if (len == 1 && (rx_data[0] == 0xFF))
  {
    i2c1_set_send_data(i2c_address, 1);    
  }    
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==htim16.Instance) {

//	} else if(htim->Instance==htim17.Instance) {

//  }
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  // i2c_address_read_from_flash();
  init_flash_data();
  sk6812_init(TOTAL_RGB); 
  // gpio_i2c_init();
  HAL_Delay(500);
  mlx90614_get_emissivity(&emissivity);
  if (low_buzz_duty)
    low_temp_alarm_flag = 1;
  if (high_buzz_duty)
    high_temp_alarm_flag = 1;
  user_i2c_init();  
  HAL_I2C_EnableListen_IT(&hi2c1);

  // emissivity = 0xf332;
  // user_tim3_init(250, 127);
  // user_tim16_init(50);
  // user_tim17_init(50);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (emissivity_set_flag) {
      mlx90614_set_emissivity(emissivity);
      emissivity_set_flag = 0;
    }
    if (!high_temp_alarm_flag && !low_temp_alarm_flag) {
      if (mlx90614_get_data(eeData))
        temperature = get_temperature();
      if (mlx90614_get_soc_data(eeData))
        temperature_soc = get_temperature();
    }

    if (high_temp_alarm_flag) {
      if (mlx90614_get_data(eeData))
        temperature = get_temperature();
      if (mlx90614_get_soc_data(eeData))
        temperature_soc = get_temperature();
      if (temperature > high_temp_alarm_value) {
        high_temp_alarm_filter++;
        normal_high_temp_alarm_filter = 0;
      }
      else {
        normal_high_temp_alarm_filter++;
        high_temp_alarm_filter = 0;
      }
      if (high_temp_alarm_filter >= HIGH_TEMP_ALARM_TIMES) {
        last_alarm_flag = 1;
        set_led(temp_led_rgb[1]);
        // set_led(0x11000000);
				set_pwm(high_buzz_freq, high_buzz_duty);
				HAL_TIM_Base_Stop_IT(&htim16);
				if((high_buzz_inter > 0) && (high_buzz_inter <= 5000))
					__HAL_TIM_SET_AUTORELOAD(&htim16, high_buzz_inter);
        // __HAL_TIM_SET_AUTORELOAD(&htim3, 250);
        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 127);
        HAL_TIM_Base_Start_IT(&htim16);
        high_pwm_start_flag = 1;
        high_temp_alarm_filter = 0;
      }
      if (normal_high_temp_alarm_filter >= NORMAL_TEMP_TIMES) {
        // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
        HAL_TIM_Base_Stop_IT(&htim16);
        high_pwm_start_flag = 0;
        normal_high_temp_alarm_filter = 0;
      }
    }
    if (low_temp_alarm_flag) {
      if (mlx90614_get_data(eeData))
        temperature = get_temperature();
      if (mlx90614_get_soc_data(eeData))
        temperature_soc = get_temperature();
      if (temperature < low_temp_alarm_value) {
        low_temp_alarm_filter++;
        normal_low_temp_alarm_filter = 0;
      }
      else {
        normal_low_temp_alarm_filter++;
        low_temp_alarm_filter = 0;
      }
      if (low_temp_alarm_filter >= LOW_TEMP_ALARM_TIMES) {
        last_alarm_flag = 1;
        set_led(temp_led_rgb[0]);
        // set_led(0x00001100);
				set_pwm(low_buzz_freq, low_buzz_duty);
				HAL_TIM_Base_Stop_IT(&htim17);
				if ((low_buzz_inter > 0) && (low_buzz_inter <= 5000))
					__HAL_TIM_SET_AUTORELOAD(&htim17, low_buzz_inter);
        HAL_TIM_Base_Start_IT(&htim17);
        low_pwm_start_flag = 1;
        low_temp_alarm_filter = 0;
      }
      if (normal_low_temp_alarm_filter >= NORMAL_TEMP_TIMES) {
        // HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
        HAL_TIM_Base_Stop_IT(&htim17);
        low_pwm_start_flag = 0;
        normal_low_temp_alarm_filter = 0;
      }
    }
    if(low_pwm_start_flag) {
      if (low_buzz_flag) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
      } else {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      }
    } else if(high_pwm_start_flag) {
      if (high_buzz_flag) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
      } else {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
      }
    } else {
      if (last_alarm_flag) {
        set_led(manual_led_rgb);
        if (!buzz_manual_flag) {
          HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
        }
        last_alarm_flag = 0;        
      }
    }   
    // HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0x5A<<1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 250;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 2-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 48000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 200;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 48000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 200;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SCL1_Pin|SDA1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCL1_Pin SDA1_Pin */
  GPIO_InitStruct.Pin = SCL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SDA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

