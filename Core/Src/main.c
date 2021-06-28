/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "lp_15hz.h"
#include "lp_envelop.h"
#include "usbd_cdc_if.h"
#include "find_abi.h"
#include "User_Structs.h"
#include "ILI9341_GFX.h"
#include "ILI9341_STM32_Driver.h"
#include "Display.h"
#include <string.h>
#include <stdlib.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LENGTH 2500
#define BLOCK_SIZE 16
#define COEFF_SIZE 32
#define LIMP_NUMBER 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t bt_flag;
uint16_t bt_flag2;

volatile uint16_t adc[8];
uint16_t (*adc_ptr)[8];
uint16_t _adc = 0;
char temp[12];
char result[12];
char toESP[21];
uint16_t _limp = 0;
int map_pos;
float32_t adc_filterred;
float32_t fir_state[COEFF_SIZE + BLOCK_SIZE -1];
float32_t *pPulsepre;
float32_t *pPulsefil;
//float32_t *pS;
/*
 * variable for dc removal
 */
//static float32_t w, w_d1;
static float32_t pulse_noDC[4];

/*
 * variable for ABI calculation
 */
NIBP_Struct NIBP;
FIR_filter_Struct pulse_filter;
envelop_filter_Struct envelop_filter;
float32_t limp_bp[4];
float32_t abi;
/*
 * State machine
 */
typedef enum
{
	READY,
	PUMPING,
	DEFLATING,
	CALCULATING,
	DISPLAY_DATA,
	HOLD
} STATE;
STATE state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  /* FPU settings ------------------------------------------------------------*/
    #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
      SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
    #endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  LCD_Layout();
//  HAL_TIM_Base_Start_IT(&htim2); //50 sample/s
  HAL_TIM_Base_Start_IT(&htim5); // 0.1s
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 8);


  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

//  LCD_Layout();


  pulse_filter.blockSize = BLOCK_SIZE;
  pulse_filter.num_taps = COEFF_SIZE;
  pulse_filter.coeff = &fir_coeff[0];
  pulse_filter.state = &fir_state[0];
  pulse_filter.num_block = MAX_LENGTH/BLOCK_SIZE;

  envelop_filter.blockSize = 128;
  envelop_filter.num_taps = 128;
  envelop_filter.coeff = &Envelop[0];
//
  arm_fir_init_f32(&pulse_filter.S, pulse_filter.num_taps, pulse_filter.coeff, &pulse_filter.state[0], pulse_filter.blockSize);
  arm_fir_init_f32(&envelop_filter.S, envelop_filter.num_taps, envelop_filter.coeff, &envelop_filter.state[0], envelop_filter.blockSize);
//  uint16_t measure_count = 0;
//  start(adc, limp_bp, &pulse_filter, &envelop_filter, &NIBP);
  state = READY;
  NIBP_Struct* pNIBP = &NIBP;
  pPulsepre = (float32_t*)&NIBP.Limp[_limp].pulse_prefilterred[0];
  pPulsefil = (float32_t*)&NIBP.pulse_filterred[0];
//  uint16_t aaa;
//  uint16_t* www;
//  www = &adc[4];
//  HAL_UART_Transmit(&huart4, "start\n\r", 6, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*********************************************************************************************
		 * STATE MACHINE
********************************************************************************************/

	  switch (state)
	  {
		case READY:
			HAL_TIM_Base_Start_IT(&htim2); //50 sample/s
			ILI9341_Draw_Text("   READY  ", 30, 200, BLUE, 2, BLACK);
			recording = 0;
		//	for(uint8_t i; i <4; i++)
		//	{
		//		GPIOD->BSRR = GPIO_PIN_8 << i;
		//	}
			GPIOD->BSRR = GPIO_PIN_8 << 16U;
			GPIOD->BSRR = GPIO_PIN_9 << 16U;
			GPIOD->BSRR = GPIO_PIN_10 << 16U;
			GPIOD->BSRR = GPIO_PIN_11 << 16U;
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = 0;
			break;
		case PUMPING:
			recording = 0;
			ILI9341_Draw_Text("  PUMPING  ", 30, 200, BLUE, 2, BLACK);
		//	for(uint8_t i; i <4; i++)
		//	{
		//		GPIOD->BSRR = GPIO_PIN_8 << i;
		//	}
			GPIOD->BSRR = GPIO_PIN_8;
//						GPIOD->BSRR = GPIO_PIN_9;
//						GPIOD->BSRR = GPIO_PIN_10;
//						GPIOD->BSRR = GPIO_PIN_11;
			htim3.Instance->CCR1 = 100;
			htim3.Instance->CCR2 = 100;
			htim3.Instance->CCR3 = 100;
			htim3.Instance->CCR4 = 100;
			for(uint8_t i = 0; i <LIMP_NUMBER; i++)
			{
				if (adc[i + 4] >3200 )
				{
//					GPIOD->BSRR = (uint32_t)GPIO_PIN_11 << 16;
//						GPIOD->BSRR = GPIO_PIN_15;
//						GPIOD->BSRR = GPIO_PIN_14 << 16U;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 >> i, 0);
				}
				else if(adc[i + 4] < 3000)
				{
//						GPIOD->BSRR = GPIO_PIN_11;
//						GPIOD->BSRR = GPIO_PIN_14;
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 >> i, 1);
				}
//				if(adc[4] > 2900)	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
			}
			if((GPIOD->ODR & (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_9 | GPIO_PIN_8 )) == 0)
			{
				state = DEFLATING;
				ILI9341_Draw_Text("  DEFLATING  ", 30, 200, BLUE, 2, BLACK);
			}
			break;
		case DEFLATING:

			recording = 1;
			GPIOD->BSRR = GPIO_PIN_8 << 16U;
			GPIOD->BSRR = GPIO_PIN_9 << 16U;
			GPIOD->BSRR = GPIO_PIN_10 << 16U;
			GPIOD->BSRR = GPIO_PIN_11 << 16U;
			htim3.Instance->CCR1 = 90;
			htim3.Instance->CCR2 = 90;
			htim3.Instance->CCR3 = 90;
			htim3.Instance->CCR4 = 90;
//
			if(adc[4] < 1700)
			{
				htim3.Instance->CCR1 = 0;
				htim3.Instance->CCR2 = 0;
				htim3.Instance->CCR3 = 0;
				htim3.Instance->CCR4 = 0;
				ILI9341_Draw_Text("CALCULATING", 30, 200, BLUE, 2, BLACK);
				state = CALCULATING;

			}
			break;
		case CALCULATING:


			/*
			 * wait for main loop done task
			 */
			HAL_TIM_Base_Stop_IT(&htim2); //50 sample/s
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = 0;

		  for(uint8_t _limp = 0; _limp <LIMP_NUMBER; _limp++)
		  {
				for (int i = 0; i < pulse_filter.num_block; i++)
				{
					arm_fir_f32(&pulse_filter.S, (float32_t*)(&NIBP.Limp[_limp].pulse_prefilterred[0]) + (i*BLOCK_SIZE) , &NIBP.pulse_filterred[0] + (i*BLOCK_SIZE), pulse_filter.blockSize);
					HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
//		  	  		GPIOD->BSRR = GPIO_PIN_15;
				}
//			  for (int i = 0; i < 2500; i++) {
////				  dtostrf
//		  		sprintf(temp, "%f\n", NIBP.pulse_filterred[i]);
//		  		HAL_UART_Transmit(&huart4, (uint8_t*)&temp[0], sizeof(temp), 10);
////		  		HAL_UART_Transmit(huart, pData, Size, Timeout)
//		  		HAL_Delay(1);
//				  adc_filterred = NIBP.pulse_filterred[i];
//
//			  }

			  find_peak(pNIBP, _limp); //
			  map_pos = find_MAP((float32_t*)(&NIBP.pulse_filterred), &NIBP.Limp[_limp].pressure[0], pNIBP, _limp);//
			  find_envelop(pNIBP);

//			  HAL_UART_Transmit(&huart4, "peak\n", 6, 10);
//			  for (int i = 0; i < 2500; i++) {
//	//				  dtostrf
//				sprintf(temp, "%f\n", NIBP.peak[i]);
//				HAL_UART_Transmit(&huart4, (uint8_t*)&temp[0], sizeof(temp), 10);
//	//		  		HAL_UART_Transmit(huart, pData, Size, Timeout)
//				HAL_Delay(1);
//				  adc_filterred = NIBP.pulse_filterred[i];
//
//			  }
			  limp_bp[_limp] = find_SYS(pNIBP, _limp);
			  sprintf(&result[0], "%6d", (int)limp_bp[_limp]);
//		  	  MAP_pos = 0;
			  for(int i = 0; i < MAX_LENGTH; i++)
			  {
				  NIBP.pulse_filterred[i] = 0;
				  NIBP.S_peak[i] = 0;
				  NIBP.peak[i] = 0;
				  NIBP.peak_envelop[i] = 0;
				  NIBP.peak_step[i] = 0;
			  }

//				sprintf(&result[0], "%6d", (int)map_pos);
//				ILI9341_Draw_Text(&result[0], 350, 220 + _limp*20, RED, 1, BLACK);

		  }
		  state = DISPLAY_DATA;
		  abi = find_abi(&limp_bp[0]);
		  sprintf(&toESP[0], "%.0f %.0f %.0f %.0f %.2f", (double)limp_bp[0], (double)limp_bp[1], (double)limp_bp[2], (double)limp_bp[3], (double)abi);
		  HAL_UART_Transmit(&huart4, &toESP[0], 20, 10);
		  HAL_TIM_Base_Start_IT(&htim2); //50 sample/s
//		  ILI9341_Fill_Screen(BLACK);
			//state = READY;
			break;
		case DISPLAY_DATA:
			for(uint8_t _limp = 0; _limp <LIMP_NUMBER; _limp++)
			{
				sprintf(&result[0], "%6d", (int)limp_bp[_limp]);

				HAL_Delay(200);

			}
			sprintf(&result[0], "%6d", (int)limp_bp[0]);
			ILI9341_Draw_Text("Limp0", 0, 220, YELLOW, 1, BLACK);
			ILI9341_Draw_Text(&result[0], 20, 220, YELLOW, 1, BLACK);

			sprintf(&result[0], "%6d", (int)limp_bp[1]);
			ILI9341_Draw_Text("Limp1", 80, 220, ORANGE, 1, BLACK);
			ILI9341_Draw_Text(&result[0], 100, 220, ORANGE, 1, BLACK);

			sprintf(&result[0], "%6d", (int)limp_bp[2]);
			ILI9341_Draw_Text("Limp2", 0, 240, PINK, 1, BLACK);
			ILI9341_Draw_Text(&result[0], 20, 240, PINK, 1, BLACK);

			sprintf(&result[0], "%6d", (int)limp_bp[3]);
			ILI9341_Draw_Text("Limp3", 80, 240, GREEN, 1, BLACK);
			ILI9341_Draw_Text(&result[0], 100, 240, GREEN, 1, BLACK);
			HAL_Delay(200);
			break;
		case HOLD:
			GPIOD->BSRR = GPIO_PIN_8 << 16U;
			GPIOD->BSRR = GPIO_PIN_9 << 16U;
			GPIOD->BSRR = GPIO_PIN_10 << 16U;
			GPIOD->BSRR = GPIO_PIN_11 << 16U;
			htim3.Instance->CCR1 = 100;
			htim3.Instance->CCR2 = 100;
			htim3.Instance->CCR3 = 100;
			htim3.Instance->CCR4 = 100;
			break;
	  }
/*********************************************************************************************
	  		 END STATE MACHINE
********************************************************************************************/


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == hadc1.Instance)
	{

		//w = adc[1] + 0.95*w_d1;
//		LCD_Update_adc(adc);


//		for (uint8_t i = 0; i <1; i++)
//		{
//			pulse_noDC[i] = adc[i] - 1000; //- w;
			if (state == DEFLATING && _adc < MAX_LENGTH)
			{
//				NIBP.Limp[i].pulse_prefilterred[_adc] = adc[i];
//				NIBP.Limp[i].pressure[_adc] = adc[i+4];
				NIBP.Limp[0].pulse_prefilterred[_adc] = (float32_t)(adc[0] - 1600);
				NIBP.Limp[0].pressure[_adc] = adc[4];
				NIBP.Limp[1].pulse_prefilterred[_adc] = (float32_t)(adc[1] - 1600);
				NIBP.Limp[1].pressure[_adc] = adc[5];
				NIBP.Limp[2].pulse_prefilterred[_adc] = (float32_t)(adc[2] - 1600);
				NIBP.Limp[2].pressure[_adc] = adc[6];
				NIBP.Limp[3].pulse_prefilterred[_adc] = (float32_t)(adc[3] - 1600);
				NIBP.Limp[3].pressure[_adc] = adc[7];
				_adc++;
			}
			if(_adc == MAX_LENGTH - 2)
			{
				state = CALCULATING;
			}
//		}

//		_adc++;
	}

}

/*
 * button interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0)
	{
		HAL_TIM_Base_Start(&htim4); /*timer for button debounce*/
		bt_flag = 1;
		_adc = 0;
		if (state == READY) state = PUMPING;
		if (state == DISPLAY_DATA) state = READY;
	}
//	if (GPIO_Pin == GPIO_PIN_1)
//	{
//		HAL_TIM_Base_Start(&htim4); /*timer for button debounce*/
//		bt_flag = 1;
//		_adc = 0;
//		if (state == READY) state = PUMPING;
//		else if (state == PUMPING) state = HOLD;
//		else if ( state == HOLD) state = PUMPING;
//	}
}
/*
 * Button debounce
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim4.Instance)
	{
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
		{
			HAL_TIM_Base_Stop(&htim4);
			bt_flag = 0;
		}
	}
	if (htim->Instance == htim5.Instance)
	{

		/*********************************************************
		 * LCD DISPLAY
		 ********************************************************/
		for(int i = 0; i < 4; i++)
		{
			sprintf(&temp[0], "%6d", adc[i]);
		//	ILI9341_Draw_Text("Test", 0, 240, RED, 1, BLACK);
			ILI9341_Draw_Text(&temp[0], 150, 40 + i*40, RED, 1, BLACK);
		}
		for(int i = 0; i < 4; i++)
		{
			sprintf(&temp[0], "%6d", adc[i + 4]);
		//	ILI9341_Draw_Text("Test", 0, 240, RED, 1, BLACK);
			ILI9341_Draw_Text(&temp[0], 150, 20 + i*40, RED, 1, BLACK);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
