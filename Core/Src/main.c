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

#define PAL_OFF 0
#define PAL_ZERO 23
#define PAL_BLACK 39
#define PAL_WHITE 77
#define PAL_TIME_CORRECTION 0.954
#define PAL_DMA_WRITE_A(next_state, next_offset) memcpy(pal_dma + 0, pal_buffer[next_state] + (pal_line_len * (next_offset)), 116);
#define PAL_DMA_WRITE_B() memcpy(pal_dma + 116, pal_buffer[pal_state] + (pal_line_len * pal_offset) + 116, 117);
#define PAL_LINE_LEN 233
#define PAL_W 189
#define PAL_H 610
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t pal_line_len = 0;
uint8_t pal_state = 0;
uint8_t pal_offset = 0;
uint16_t pal_line_counter = 0;
uint8_t pal_dma[625 * PAL_LINE_LEN];
uint8_t frame_buffer[PAL_W * PAL_H];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM4_Init(void);
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
	double line_period_us = 64.0;
	double line_sync_us = 4.7;
	double front_porch_us = 1.65;
	double short_pulse_us = 2.35;
	double long_pulse_us = 27.3;
	double sample_time_us = (double)TIM4->ARR / (80.0 * PAL_TIME_CORRECTION);
	uint32_t one_us_sa = (uint32_t)(1.0 / sample_time_us + 0.5);
	uint32_t line_period_sa = (uint32_t)(line_period_us / sample_time_us + 0.5);
	uint32_t line_sync_sa = (uint32_t)(line_sync_us / sample_time_us + 0.5);
	uint32_t front_porch_sa = (uint32_t)(front_porch_us / sample_time_us + 0.5);
	uint32_t short_pulse_sa = (uint32_t)(short_pulse_us / sample_time_us + 0.5);
	uint32_t long_pulse_sa = (uint32_t)(long_pulse_us / sample_time_us + 0.5);
	pal_line_len = line_period_sa;

	// clear all buffers
	for (uint32_t i = 0; i < 5; i++)
	{
		for (uint32_t j = 0; j < PAL_BUFFER_SIZE; j++)
		{
			pal_buffer[i][j] = PAL_ZERO;
		}
	}
	// sync
	for (uint32_t i1 = 0; i1 < line_period_sa * 5; i1++)
	{
		uint32_t i = i1 % (line_period_sa / 2);
		if (i1 * 2 < line_period_sa * 5)
		{
			pal_buffer[0][i1] = i <= long_pulse_sa ? PAL_OFF : PAL_ZERO;
		}
		else
		{
			pal_buffer[0][i1] = i <= short_pulse_sa ? PAL_OFF : PAL_ZERO;
		}
	}
	for (uint32_t i1 = 0; i1 < line_period_sa * 7; i1++)
	{
		uint32_t i = i1 % (line_period_sa / 2);
		if (i1 * 2 < line_period_sa * 5)
		{
			pal_buffer[2][i1] = i <= short_pulse_sa ? PAL_OFF : PAL_ZERO;
		}
		else if (i1 < line_period_sa * 5)
		{
			pal_buffer[2][i1] = i <= long_pulse_sa ? PAL_OFF : PAL_ZERO;
		}
		else
		{
			pal_buffer[2][i1] = i <= short_pulse_sa ? PAL_OFF : PAL_ZERO;
		}
	}
	for (uint32_t i1 = 0; i1 < line_period_sa * 3; i1++)
	{
		uint32_t i = i1 % (line_period_sa / 2);
		pal_buffer[4][i1] = i <= short_pulse_sa ? PAL_OFF : PAL_ZERO;
	}
	// frames
	for (uint32_t i = 0; i < line_period_sa; i++)
	{

	}
	for (uint32_t y = 0; y < PAL_H; y++)
	{
		for (uint32_t x = 0; x < line_period_sa; x++)
		{
			// TODO: field blanking interval -> PAL_ZERO (not black) for first 25 lines of both fields
			if (i < line_sync_sa)
			{
				pal_buffer[1][i] = PAL_OFF;
				pal_buffer[3][i] = PAL_OFF;
			}
			else if (i < 2 * line_sync_sa)
			{
				pal_buffer[1][i] = PAL_ZERO;
				pal_buffer[3][i] = PAL_ZERO;
			}
			else if (i < line_period_sa - one_us_sa - front_porch_sa)
			{
				pal_buffer[1][i] = PAL_BLACK + (int)(0.15 * 127 * (sin((i - 2 * line_sync_sa) * 0.15) + 1));
				if (pal_buffer[1][i] < PAL_BLACK)
				{
					pal_buffer[1][i] = PAL_BLACK;
				}
				if (pal_buffer[1][i] > PAL_WHITE)
				{
					pal_buffer[1][i] = PAL_WHITE;
				}

				pal_buffer[3][i] = PAL_BLACK + (int)(0.15 * 127 * (cos((i - 2 * line_sync_sa) * 0.15) + 1));
				if (pal_buffer[3][i] < PAL_BLACK)
				{
					pal_buffer[3][i] = PAL_BLACK;
				}
				if (pal_buffer[3][i] > PAL_WHITE)
				{
					pal_buffer[3][i] = PAL_WHITE;
				}
			}
			else if (i < line_period_sa - front_porch_sa)
			{
				pal_buffer[1][i] = PAL_BLACK;
				pal_buffer[3][i] = PAL_BLACK;
			}
			else
			{
				pal_buffer[1][i] = PAL_ZERO;
				pal_buffer[3][i] = PAL_ZERO;
			}
		}

		pal_state = 0;
		pal_offset = 0;
		PAL_DMA_WRITE_A(0, 0);
		PAL_DMA_WRITE_B();
		HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)pal_dma, pal_line_len, DAC_ALIGN_8B_R);
		HAL_TIM_Base_Start(&htim4);

		exec_ins();

		/*

		 int64_t t_f_cpu_start = 0;
		 int64_t t_last_irq = 0;

		 void handle_uart_io()
		 {
		 if (ram[0xff0] != 0)
		 {
		 if (pc == 0xe5cd)
		 {
		 // siehe 0xEB35
		 uint8_t kb_index = (uint8_t)mem_read(0xc6);
		 if (kb_index < (uint8_t)mem_read(0x0289))
		 {
		 int c = getchar();
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
		 putchar('\n');
		 else if (a == 0x1d)
		 putchar(' ');
		 else if ((uint8_t)a != 0x93)
		 putchar(a);
		 }
		 }
		 }

		 void loop_f_cpu_check()
		 {
		 if (pc == 0xc000)
		 {
		 t_f_cpu_start = esp_timer_get_time();
		 }
		 if (pc == 0xfce2 && t_f_cpu_start != 0)
		 {
		 int32_t t_duration = esp_timer_get_time() - t_f_cpu_start;
		 float f_cpu = 328714 / (t_duration / 1000000.0);
		 printf("F_CPU = %i Hz\n", (int)f_cpu);
		 t_f_cpu_start = 0;
		 }
		 }

		 void c64_run(void *parameters)
		 {
		 reset();
		 ram[0xff0] = ram[0xff1] = ram[0xff2] = 0;
		 while (1)
		 {
		 for (uint32_t i = 0; i < 10000; i++)
		 {
		 if (esp_timer_get_time() - t_last_irq > 16666)
		 {
		 irq();
		 t_last_irq = esp_timer_get_time();
		 }
		 if (!gpio_get_level(42))
		 nmi();
		 if (pc == 0xe544)
		 display_clear();

		 handle_uart_io();
		 loop_f_cpu_check();
		 exec_ins();
		 }
		 vTaskDelay(10 / portTICK_PERIOD_MS);
		 }
		 }

		 void app_main()
		 {
		 ESP_ERROR_CHECK(esp_task_wdt_init(30, false));

		 uart_config_t uart_config = {
		 .baud_rate = 115200,
		 .data_bits = UART_DATA_8_BITS,
		 .parity = UART_PARITY_DISABLE,
		 .stop_bits = UART_STOP_BITS_1,
		 .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		 };
		 ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0));
		 ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

		 gpio_config_t io_conf;
		 io_conf.mode = GPIO_MODE_INPUT;
		 io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
		 io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
		 io_conf.intr_type = GPIO_INTR_DISABLE;
		 io_conf.pin_bit_mask = (1ULL << 42) | (1ULL << 17);
		 ESP_ERROR_CHECK(gpio_config(&io_conf));

		 display_init();

		 xTaskCreate(c64_run, "c64_run", 5000, NULL, 2, NULL);
		 }

		 */
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
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

		/*Configure GPIO pin : B1_Pin */
		GPIO_InitStruct.Pin = B1_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

		/*Configure GPIO pin : LD2_Pin */
		GPIO_InitStruct.Pin = LD2_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

		/* USER CODE BEGIN MX_GPIO_Init_2 */
		/* USER CODE END MX_GPIO_Init_2 */
	}

	/* USER CODE BEGIN 4 */
	void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
	{
		if (pal_state == 0)
		{
			if (pal_offset + 1 >= 5)
			{
				PAL_DMA_WRITE_A(1, 0);
			}
			else
			{
				PAL_DMA_WRITE_A(0, pal_offset + 1);
			}
		}
		else if (pal_state == 1)
		{
			if (pal_line_counter + 1 >= 310)
			{
				PAL_DMA_WRITE_A(2, 0);
			}
		}
		else if (pal_state == 2)
		{
			if (pal_offset + 1 >= 7)
			{
				PAL_DMA_WRITE_A(3, 0);
			}
			else
			{
				PAL_DMA_WRITE_A(2, pal_offset + 1);
			}
		}
		else if (pal_state == 3)
		{
			if (pal_line_counter + 1 >= 622)
			{
				PAL_DMA_WRITE_A(4, 0);
			}
		}
		else if (pal_state == 4)
		{
			if (pal_offset + 1 >= 3)
			{
				PAL_DMA_WRITE_A(0, 0);
			}
			else
			{
				PAL_DMA_WRITE_A(4, pal_offset + 1);
			}
		}
	}

	void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
	{
		pal_line_counter++;
		if (pal_state == 0)
		{
			pal_offset++;
			if (pal_offset >= 5)
			{
				pal_state = 1;
				pal_offset = 0;
			}
			PAL_DMA_WRITE_B();
		}
		else if (pal_state == 1)
		{
			if (pal_line_counter >= 310)
			{
				pal_state = 2;
				pal_offset = 0;
				PAL_DMA_WRITE_B();
			}
		}
		else if (pal_state == 2)
		{
			pal_offset++;
			if (pal_offset >= 7)
			{
				pal_state = 3;
				pal_offset = 0;
			}
			PAL_DMA_WRITE_B();
		}
		else if (pal_state == 3)
		{
			if (pal_line_counter >= 622)
			{
				pal_state = 4;
				pal_offset = 0;
				PAL_DMA_WRITE_B();
			}
		}
		else if (pal_state == 4)
		{
			pal_offset++;
			if (pal_offset >= 3)
			{
				pal_state = 0;
				pal_offset = 0;
				pal_line_counter = 0;
			}
			PAL_DMA_WRITE_B();
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
