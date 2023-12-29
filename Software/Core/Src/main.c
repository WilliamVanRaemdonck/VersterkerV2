/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "string.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t adcData[ADC_CHANNELS][ADC_DMA_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//	Global vars
uint8_t voltage = 1;
uint8_t counter = 0;
uint8_t first = 1;
uint8_t shift = 0;
uint8_t startup = 10;
static uint8_t prevVolume;
uint8_t databuffer = 0;

uint16_t volume = 1;

uint32_t input = 255;

bool monoMode = false;


//	TDA data
uint8_t steps[15] = {
		0b00000000,
		0b00000001,
		0b00000010,
		0b00000011,
		0b00000100,

		0b00000101,
		0b00000110,
		0b00000111,
		0b00001110,
		0b00001101,

		0b00001100,
		0b00001011,
		0b00001010,
		0b00001001,
		0b00001000};


//	START UP DISPLAY
//	--------------------
//	   50W VERSTERKER
//
//       William VR
//

char* muxTable[4] = {
		" [X]  [_]  [_]  [_] ",
		" [_]  [X]  [_]  [_] ",
		" [_]  [_]  [X]  [_] ",
		" [_]  [_]  [_]  [X] "
};

char* VersterkerValuesString[6] = {
		"Volume:             ",
		"Gain:               ",
		"Bass:               ",
		"Midrange:           ",
		"Treble:             ",
		"Input:              "
};

//1 VOLUME:					address: 0x80
//2 ??????_______________	address: 0xc0
//3 INPUT:					address: 0x94
//4 [] [] [?] []			address: 0xd4

//	struct amp valias
typedef struct{
	uint8_t inputSelect;
	uint8_t gain;
	uint8_t bass;
	uint8_t midrange;
	uint8_t treble;
	uint8_t attenuateL;
	uint8_t attenuateR;
}versterkerValues;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	input = (uint8_t)__HAL_TIM_GET_COUNTER(htim);

	volume = (uint8_t)((255 - input) / 6.375);

	if(volume == 0){
		volume = 1;
	}

	if (volume != prevVolume ) { //&& startup == 0
		// Display volume on LCD
		//lcd_write_string(VersterkerValuesString[0], row1);
		//lcd_write_bars(input, row2);

		// Update volume on TDA chip via I2C
		databuffer = volume;
		HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, VolumeAddress, I2C_MEMADD_SIZE_8BIT, &databuffer, 1, HAL_MAX_DELAY);
	}

	// Update the previous volume
	prevVolume = volume;
}

//	printf function
int _write(int file, char *ptr, int len) {
	for(int i = 0; i < len; i++){
		if(ptr[i]=='\n'){
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r", 1, HAL_MAX_DELAY);
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);
	}
	return len;
}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	//	leds on
	HAL_GPIO_WritePin(StereoMuteLed_GPIO_Port, StereoMuteLed_Pin, 1);
	HAL_GPIO_WritePin(MonoMuteLed_GPIO_Port, MonoMuteLed_Pin, 1);

	//	read from flash
	//volume = Flash_Read_Data(0x08001000);	//0

	versterkerValues amp;
	versterkerValues ampPrev;

	amp.inputSelect = 0;
	amp.gain = 0;
	amp.bass = 0;
	amp.midrange = 0;
	amp.treble = 0;
	amp.attenuateL = 0;
	amp.attenuateR = 0;

	// 	Initialize ADC channel(s)
	HAL_ADCEx_Calibration_Start(&hadc);

	// init DMA
	ADC_DMA_Init();

	//start up delay
	HAL_Delay(1000);

	//	Init display
	lcd_init();

	lcd_write_string(VersterkerValuesString[5], row3);
	lcd_write_selection(amp.inputSelect, row4);

	//	Encoder
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

	//	Toggle relais MUTE off
	HAL_GPIO_WritePin(MuteTPA_GPIO_Port, MuteTPA_Pin, 0);
	HAL_GPIO_WritePin(MuteRelais_GPIO_Port, MuteRelais_Pin, 1);

	//	leds off
	HAL_GPIO_WritePin(StereoMuteLed_GPIO_Port, StereoMuteLed_Pin, 0);
	HAL_GPIO_WritePin(MonoMuteLed_GPIO_Port, MonoMuteLed_Pin, 0);

	//init TDA
	databuffer = volume;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, VolumeAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.inputSelect;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, GainAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.gain;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, GainAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.bass;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, BassAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.midrange;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, MidRangeAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.treble;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, TrebleAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.attenuateL;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, AttenuateLAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
	databuffer = amp.attenuateR;
	HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, AttenuateRAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// read in values
		/*
		 * Audio Source Select
		 * 4 ADC values
		 * S/M switch
		 */
		//Audio select switch
		if(HAL_GPIO_ReadPin(AudioInput0_GPIO_Port, AudioInput0_Pin)){
			amp.inputSelect = 0;
		}

		if(HAL_GPIO_ReadPin(AudioInput1_GPIO_Port, AudioInput1_Pin)){
			amp.inputSelect = 1;
		}

		if(HAL_GPIO_ReadPin(AudioInput2_GPIO_Port, AudioInput2_Pin)){
			amp.inputSelect = 2;
		}

		if(HAL_GPIO_ReadPin(AudioInput3_GPIO_Port, AudioInput3_Pin)) {
			amp.inputSelect = 3;
		}

		//Stereo Mono Switch
		if(HAL_GPIO_ReadPin(StereoMonoSwitch_GPIO_Port, StereoMonoSwitch_Pin)){
			monoMode = false;
			HAL_GPIO_WritePin(StereoMonoAnalogSwitch_GPIO_Port, StereoMonoAnalogSwitch_Pin, 0);
		}
		else{
			monoMode = true;
			HAL_GPIO_WritePin(StereoMonoAnalogSwitch_GPIO_Port, StereoMonoAnalogSwitch_Pin, 1);
		}

		// ADC - Values
		ADC_Start_Conversion();

		amp.gain = adcData[0][0];
		amp.bass = adcData[0][1];
		amp.midrange = adcData[0][2];
		amp.treble = adcData[0][3];

		//printf("%d %d %d %d\n", amp.gain, amp.bass, amp.midrange, amp.treble);

		//update Leds
		if(monoMode){
			HAL_GPIO_WritePin(MonoLed_GPIO_Port, MonoLed_Pin, 1);
			HAL_GPIO_WritePin(StereoLed_GPIO_Port, StereoLed_Pin, 0);
		}
		else{
			HAL_GPIO_WritePin(MonoLed_GPIO_Port, MonoLed_Pin, 0);
			HAL_GPIO_WritePin(StereoLed_GPIO_Port, StereoLed_Pin, 1);
		}

		// update TDA IC & update display
		/*
		 * I2C
		 * 8 registers
		 */

		// Display volume on LCD
		lcd_write_string(VersterkerValuesString[0], row1);
		lcd_write_bars(input, row2);

		if(amp.inputSelect != ampPrev.inputSelect){
			//Dislay
			lcd_write_string(VersterkerValuesString[5], row3);
			lcd_write_selection(amp.inputSelect, row4);

			//printf("input = %d\n", amp.inputSelect);

			//TDA
			databuffer = amp.inputSelect;
			HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, InputSelectAddress, I2C_MEMADD_SIZE_8BIT, &databuffer, 1, HAL_MAX_DELAY);

			//delay
			HAL_Delay(10);
		}

		if(amp.gain != ampPrev.gain){
			databuffer = (amp.gain >> 4);
			printf("%d \n", amp.gain);
			HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, GainAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
		}

		updateTDA(amp.bass, ampPrev.bass, VersterkerValuesString[2], 1, BassAddress);
		updateTDA(amp.midrange, ampPrev.midrange, VersterkerValuesString[3], 1, MidRangeAddress);
		updateTDA(amp.treble, ampPrev.treble, VersterkerValuesString[4], 1, TrebleAddress);

		if(amp.attenuateL != ampPrev.attenuateL){
			databuffer = amp.attenuateL;
			HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, AttenuateLAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
		}

		if(amp.attenuateR != ampPrev.attenuateR){
			databuffer = amp.attenuateR;
			HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, AttenuateRAddress, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
		}

		//	update prev values
		ampPrev.inputSelect = amp.inputSelect;
		ampPrev.gain 		= amp.gain;
		ampPrev.bass 		= amp.bass;
		ampPrev.midrange 	= amp.midrange;
		ampPrev.treble 		= amp.treble;
		ampPrev.attenuateL 	= amp.attenuateL;
		ampPrev.attenuateR 	= amp.attenuateR;

		//	Toggle Status pin
		HAL_GPIO_TogglePin(STSLED_GPIO_Port, STSLED_Pin);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00101D7C;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 30000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7937;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MonoMuteLed_Pin|StereoMuteLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MonoLed_Pin|StereoLed_Pin|StereoMonoAnalogSwitch_Pin|MuteRelais_Pin
                          |MuteTPA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STSLED_Pin|ERRLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MonoMuteLed_Pin StereoMuteLed_Pin */
  GPIO_InitStruct.Pin = MonoMuteLed_Pin|StereoMuteLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AudioInput3_Pin AudioInput2_Pin */
  GPIO_InitStruct.Pin = AudioInput3_Pin|AudioInput2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : AudioInput1_Pin AudioInput0_Pin */
  GPIO_InitStruct.Pin = AudioInput1_Pin|AudioInput0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : StereoMonoSwitch_Pin */
  GPIO_InitStruct.Pin = StereoMonoSwitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(StereoMonoSwitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MonoLed_Pin StereoLed_Pin StereoMonoAnalogSwitch_Pin MuteRelais_Pin
                           MuteTPA_Pin */
  GPIO_InitStruct.Pin = MonoLed_Pin|StereoLed_Pin|StereoMonoAnalogSwitch_Pin|MuteRelais_Pin
                          |MuteTPA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STSLED_Pin ERRLED_Pin */
  GPIO_InitStruct.Pin = STSLED_Pin|ERRLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TDA

void updateTDA(uint8_t input, uint8_t prevInput, char* text, uint8_t index, uint8_t adres){
	//printf("%s\t%d\t%d\tdiff= %d\n", text, input, prevInput, diff(input, prevInput));
	if(input != prevInput && diff(input, prevInput) > 1){
		//Dislay
		/*
		if(index == 1){
			lcd_write_string(text, row1);
			lcd_write_bars(input, row2);
		}
		else{
			lcd_write_string(text, row3);
			lcd_write_bars(input, row4);
		}*/

		//Reset timer
		//HAL_TIM_Base_Start_IT(&htim6);

		//printf
		//printf("%s new = %d  prev = %d\n", text, input, prevInput);

		//TDA
		databuffer = input;
		shift = ((databuffer >> 4) & 0b00001111);
		databuffer = steps[shift] ;
		HAL_I2C_Mem_Write(&hi2c1, TDA7439DS, adres, I2C_MEMADD_SIZE_8BIT,  &databuffer, 1, HAL_MAX_DELAY);
		HAL_Delay(10);
	}



}






//	FLASH
void Flash_Write_Data(uint32_t address, uint16_t volume)
{
	// Unlock the Flash
	HAL_FLASH_Unlock();
	// Program the Flash
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, volume) != HAL_OK) {
		printf("Error writing to Flash memory. Error code: %lX\n", (unsigned long)HAL_FLASH_GetError());
	}

	// Lock the Flash
	HAL_FLASH_Lock();
}

uint16_t Flash_Read_Data(uint32_t Address){

	__IO uint16_t read_data = *(__IO uint32_t *)Address;
	return (uint16_t)read_data;
}

//	DISPLAY
void lcd_write_string(char* intput, uint8_t row){
	lcd_send_cmd(row);
	lcd_send_string(intput);
}

void lcd_write_bars(uint8_t input, uint8_t row){
	uint8_t bars;
	if(input != 0){
		bars = input / 12; // 0 - 255 => 0 - 20 => / 12.75 ~ 12
		if(bars > 20)
			bars = 20;
	}
	else{
		bars = 0;
	}

	uint8_t blanks = 20 - bars;
	char barStr[20];

	//assemble string
	strcpy(barStr, "");
	for(uint8_t i = 0; i < bars; i++){
		strcat(barStr, "Y");
	}
	for(uint8_t i = 0; i < blanks; i++){
		strcat(barStr, "_");
	}

	//printf("%d %d\n", bars, blanks);

	lcd_send_cmd(row);
	lcd_send_string(barStr);
}

void lcd_write_selection(uint8_t input, uint8_t row){
	char* muxStr;
	muxStr = muxTable[input];

	lcd_send_cmd(row);
	lcd_send_string(muxStr);
}

//	TIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		lcd_write_string(VersterkerValuesString[0], row1);
		lcd_write_bars(input, row2);
		HAL_TIM_Base_Stop_IT(&htim6);
	}
}
//	DIFF
uint8_t diff(uint8_t a, uint8_t b){
	uint8_t diff;
	if( a>b )
		diff=a-b;
	else
		diff=b-a;
	return diff;
}


//	ADC
void ADC_DMA_Init() {
	// Enable clock for ADC and GPIO
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure ADC pins as analog inputs (IN0 to IN3)
	// tmp GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3;
	GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1 | GPIO_MODER_MODER2 | GPIO_MODER_MODER3 | GPIO_MODER_MODER4  ;

	// Initialize the ADC
	ADC1->CR |= ADC_CR_ADEN; // Enable the ADC
	while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait for ADC to be ready

	// Configure ADC settings
	ADC1->CFGR1 |= ADC_CFGR1_CONT; // Continuous conversion mode
	//ADC1->CFGR1 |= ADC_CFGR1_RES_0; // 12-bit resolution
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN; // Enable DMA mode

	// Configure the sequence and channels (IN0 to IN3)
	// tmp ADC1->CHSELR |= ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL3;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2 | ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4  ;

	// Initialize the DMA
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable DMA1 clock
	DMA1_Channel1->CCR = 0;
	DMA1_Channel1->CCR |= DMA_CCR_MINC; // Memory increment mode
	DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0; // 16-bit memory size
	DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0; // 16-bit peripheral size
	DMA1_Channel1->CCR |= DMA_CCR_CIRC; // Circular mode
	DMA1_Channel1->CNDTR = ADC_DMA_BUFFER_SIZE; // Number of data items to transfer
	DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR); // Source: ADC data register
	DMA1_Channel1->CMAR = (uint32_t)(adcData[0]); // Destination: Memory buffer for channel 0

	// Enable DMA channel
	// tmp DMA1_Channel1->CCR |= DMA_CCR_EN;
	DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR); // Source: ADC data register
	DMA1_Channel1->CMAR = (uint32_t)(adcData[0]); // Destination: Memory buffer for channel 0
	DMA1_Channel1->CCR |= DMA_CCR_EN; // Enable DMA channel

}

void ADC_Start_Conversion() {
	// Start the ADC conversion
	ADC1->CR |= ADC_CR_ADSTART;
}



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
		HAL_GPIO_TogglePin(ERRLED_GPIO_Port, ERRLED_Pin);
		HAL_Delay(500);
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
