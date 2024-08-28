/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "pal.h"
#include "display.h"
#include "cpu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESC_PWM 0
#define PAL 1

#define F_TO_I(f) (uint32_t)(80000 * (f + 1))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
PAL_t hpal1;
GPIO_TypeDef *cia_row_ports[16];
uint16_t cia_row_pins[16];
GPIO_TypeDef *cia_col_ports[16];
uint16_t cia_col_pins[16];
uint8_t c64_irq = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_DAC1_Init();
	MX_TIM4_Init();
	MX_TIM15_Init();
	/* USER CODE BEGIN 2 */

#if ESC_PWM
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	/*
	 // calibration
	 TIM2->CCR1 = 160000;
	 while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
	 ;
	 */
	while (1)
	{
		TIM2->CCR1 = 80000;
		HAL_Delay(2000);
		while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 1)
			;
		// 0.13 - 0.20
		float speed = 0.1f;
		/*
		 if (speed < 0.175f)
		 {
		 for (float f = 0.175f; f > speed + 0.01f; f -= 0.002f)
		 {
		 TIM2->CCR1 = F_TO_I(f);
		 HAL_Delay(100);
		 }
		 }
		 */
		TIM2->CCR1 = F_TO_I(speed);
		HAL_Delay(5000);
		/*
		 // sweep
		 float start = 0.15f;
		 float end = 0.20f;
		 for (float f = start; f <= end + 0.001; f += 0.01f)
		 {
		 TIM2->CCR1 = F_TO_I(f);
		 HAL_Delay(2500);
		 }
		 */
	}
#endif

#if PAL
	hpal1.hdac = &hdac1;
	hpal1.dac_channel = DAC_CHANNEL_1;
	hpal1.htim = &htim4;
	PAL_Init(&hpal1);
	for (uint32_t y = 0; y < PAL_FRAME_LINE_COUNT; y++)
	{
		for (uint32_t x = 0; x < PAL_FRAME_LINE_LEN; x++)
		{
			hpal1.frame_buffer[y * PAL_FRAME_LINE_LEN + x] = PAL_BLACK;
		}
	}
	PAL_Start(&hpal1);

	HAL_TIM_Base_Start_IT(&htim15);

	cia_row_ports[0] = KB_Row0_GPIO_Port;
	cia_row_pins[0] = KB_Row0_Pin;
	cia_row_ports[1] = KB_Row1_GPIO_Port;
	cia_row_pins[1] = KB_Row1_Pin;
	cia_row_ports[2] = KB_Row2_GPIO_Port;
	cia_row_pins[2] = KB_Row2_Pin;
	cia_row_ports[3] = KB_Row3_GPIO_Port;
	cia_row_pins[3] = KB_Row3_Pin;
	cia_row_ports[4] = KB_Row4_GPIO_Port;
	cia_row_pins[4] = KB_Row4_Pin;
	cia_row_ports[5] = KB_Row5_GPIO_Port;
	cia_row_pins[5] = KB_Row5_Pin;
	cia_row_ports[6] = KB_Row6_GPIO_Port;
	cia_row_pins[6] = KB_Row6_Pin;
	cia_row_ports[7] = KB_Row7_GPIO_Port;
	cia_row_pins[7] = KB_Row7_Pin;

	cia_col_ports[0] = KB_Col0_GPIO_Port;
	cia_col_pins[0] = KB_Col0_Pin;
	cia_col_ports[1] = KB_Col1_GPIO_Port;
	cia_col_pins[1] = KB_Col1_Pin;
	cia_col_ports[2] = KB_Col2_GPIO_Port;
	cia_col_pins[2] = KB_Col2_Pin;
	cia_col_ports[3] = KB_Col3_GPIO_Port;
	cia_col_pins[3] = KB_Col3_Pin;
	cia_col_ports[4] = KB_Col4_GPIO_Port;
	cia_col_pins[4] = KB_Col4_Pin;
	cia_col_ports[5] = KB_Col5_GPIO_Port;
	cia_col_pins[5] = KB_Col5_Pin;
	cia_col_ports[6] = KB_Col6_GPIO_Port;
	cia_col_pins[6] = KB_Col6_Pin;
	cia_col_ports[7] = KB_Col7_GPIO_Port;
	cia_col_pins[7] = KB_Col7_Pin;

	reset();
	ram[0xff0] = ram[0xff1] = ram[0xff2] = 0;
	while (1)
	{
		if (c64_irq)
		{
			irq();
			c64_irq = 0;
		}
		if (pc == 0xe544)
		{
			display_clear();
		}
		if (pc == 0xe5cd)
		{
			// siehe 0xEB35
			uint8_t kb_index = (uint8_t)mem_read(0xc6);
			if (kb_index < (uint8_t)mem_read(0x0289))
			{
				int c = 0; // getchar();
				if (c > 0)
				{
					if (c == '\n')
						c = '\r';
					mem_write(0x0277 + kb_index, c);
					mem_write(0xc6, kb_index + 1);
				}
			}
		}
		if (pc == 0xe716)
		{
			if (a == '\r')
				; // putchar('\n');
			else if (a == 0x1d)
				; // putchar(' ');
			else if ((uint8_t)a != 0x93)
				; // putchar(a);
		}
		if (HAL_GPIO_ReadPin(KB_Restore_GPIO_Port, KB_Restore_Pin) == GPIO_PIN_RESET)
		{
			nmi();
		}
		exec_ins();
	}
#endif
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitTypeDef RCC_OscInitStruct = {
		0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {
		0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = {
		0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {
		0 };
	TIM_MasterConfigTypeDef sMasterConfig = {
		0 };
	TIM_OC_InitTypeDef sConfigOC = {
		0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1600000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {
		0 };
	TIM_MasterConfigTypeDef sMasterConfig = {
		0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 21;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM15 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM15_Init(void)
{

	/* USER CODE BEGIN TIM15_Init 0 */

	/* USER CODE END TIM15_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {
		0 };
	TIM_MasterConfigTypeDef sMasterConfig = {
		0 };
	TIM_OC_InitTypeDef sConfigOC = {
		0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {
		0 };

	/* USER CODE BEGIN TIM15_Init 1 */

	/* USER CODE END TIM15_Init 1 */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 26666 - 1;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 50;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM15_Init 2 */

	/* USER CODE END TIM15_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {
		0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | KB_Col0_Pin | KB_Col2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(KB_Col1_GPIO_Port, KB_Col1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, KB_Col6_Pin | KB_Col7_Pin | KB_Col3_Pin | KB_Col5_Pin
		| KB_Col4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin KB_Col0_Pin KB_Col2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | KB_Col0_Pin | KB_Col2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : KB_Row4_Pin KB_Row7_Pin KB_Row0_Pin */
	GPIO_InitStruct.Pin = KB_Row4_Pin | KB_Row7_Pin | KB_Row0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : KB_Col1_Pin */
	GPIO_InitStruct.Pin = KB_Col1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(KB_Col1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : KB_Col6_Pin KB_Col7_Pin KB_Col3_Pin KB_Col5_Pin
	 KB_Col4_Pin */
	GPIO_InitStruct.Pin = KB_Col6_Pin | KB_Col7_Pin | KB_Col3_Pin | KB_Col5_Pin
		| KB_Col4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : KB_Row1_Pin KB_Row3_Pin KB_Restore_Pin */
	GPIO_InitStruct.Pin = KB_Row1_Pin | KB_Row3_Pin | KB_Restore_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : KB_Row2_Pin KB_Row6_Pin KB_Row5_Pin */
	GPIO_InitStruct.Pin = KB_Row2_Pin | KB_Row6_Pin | KB_Row5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	// HAL_UART_Transmit(&huart2, &ch, sizeof(uint8_t), 10);

	return ch;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	PAL_IntHalfCplt(&hpal1);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	PAL_IntCplt(&hpal1);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM15)
	{
		c64_irq = 1;
	}
}

uint8_t gpio_read_row(uint8_t pin)
{
	return HAL_GPIO_ReadPin(cia_row_ports[pin], cia_row_pins[pin]) == GPIO_PIN_SET ? 1 : 0;
}

void gpio_write_col(uint8_t pin, uint8_t state)
{
	HAL_GPIO_WritePin(cia_col_ports[pin], cia_col_pins[pin], state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void display_set_pixel(uint16_t x, uint16_t y, bool state)
{
	// centering
	x += 27;
	y += 45;
	// neccessary sacrifices
	x /= 2;
	y += 4;
	if (x < PAL_FRAME_LINE_LEN && y < PAL_FRAME_LINE_COUNT)
	{
		if (state)
		{
			hpal1.frame_buffer[y * PAL_FRAME_LINE_LEN + x] = PAL_WHITE;
		}
		else
		{
			hpal1.frame_buffer[y * PAL_FRAME_LINE_LEN + x] = PAL_BLACK;
		}
	}
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
