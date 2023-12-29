/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define ADC_CHANNELS 5
#define ADC_DMA_BUFFER_SIZE 5

void ADC_Start_Conversion();
void ADC_DMA_Init();

void Flash_Write_Data (uint32_t, uint16_t);
uint16_t Flash_Read_Data (uint32_t);

void lcd_write_string(char*, uint8_t);
void lcd_write_bars(uint8_t, uint8_t);
void lcd_write_selection(uint8_t, uint8_t);

uint8_t diff(uint8_t, uint8_t);

void updateTDA(uint8_t, uint8_t, char*, uint8_t, uint8_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MonoMuteLed_Pin GPIO_PIN_4
#define MonoMuteLed_GPIO_Port GPIOA
#define StereoMuteLed_Pin GPIO_PIN_5
#define StereoMuteLed_GPIO_Port GPIOA
#define AudioInput3_Pin GPIO_PIN_6
#define AudioInput3_GPIO_Port GPIOA
#define AudioInput2_Pin GPIO_PIN_7
#define AudioInput2_GPIO_Port GPIOA
#define AudioInput1_Pin GPIO_PIN_4
#define AudioInput1_GPIO_Port GPIOC
#define AudioInput0_Pin GPIO_PIN_5
#define AudioInput0_GPIO_Port GPIOC
#define StereoMonoSwitch_Pin GPIO_PIN_0
#define StereoMonoSwitch_GPIO_Port GPIOB
#define MonoLed_Pin GPIO_PIN_1
#define MonoLed_GPIO_Port GPIOB
#define StereoLed_Pin GPIO_PIN_2
#define StereoLed_GPIO_Port GPIOB
#define STSLED_Pin GPIO_PIN_11
#define STSLED_GPIO_Port GPIOC
#define ERRLED_Pin GPIO_PIN_12
#define ERRLED_GPIO_Port GPIOC
#define StereoMonoAnalogSwitch_Pin GPIO_PIN_3
#define StereoMonoAnalogSwitch_GPIO_Port GPIOB
#define MuteRelais_Pin GPIO_PIN_4
#define MuteRelais_GPIO_Port GPIOB
#define MuteTPA_Pin GPIO_PIN_5
#define MuteTPA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define TDA7439DS 0x88

#define VolumeAddress 0b00000010
#define InputSelectAddress 0b00000000
#define GainAddress 0b00000001
#define BassAddress 0b00000011
#define MidRangeAddress 0b00000100
#define TrebleAddress 0b00000101
#define AttenuateLAddress 0b00000111
#define AttenuateRAddress 0b00000110

#define row1	0x80
#define row2	0xc0
#define row3	0x94
#define row4	0xd4


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
