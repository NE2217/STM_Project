/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>
#include "DS1302.h"
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
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void setPWM(uint16_t pwm_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
////void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
////{
////      if(htim == &htim1)
////      {
////         HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
////      }
////}

//int Dim(int count_LED, int on, int off)
//{
//	for (int i=0; i<100; i++ )
//		{
//		if (count_LED>=off)
//		{
//			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//			count_LED=0;
//		}
//		else if(count_LED==on)
//		{
//			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//			count_LED++;
//		}
//		else
//		{
//			count_LED++;
//		}
//		HAL_Delay(1);
//		}
//		return 0;
//}
#define RX_BUF_SIZE			8u
#define Last						4u
uint16_t A,B,C, P1, P2, P3, P4 = 1;
uint8_t Rx=0, Tx=0;
uint8_t RxBuf[RX_BUF_SIZE]={0}, TxBuf[RX_BUF_SIZE]={0};
uint8_t shift=0;
uint8_t Tim[8]={0};
uint8_t Init_Tim[8]={1,		// Control
								23, 	// Year
								4, 		// Month
								17,		// Data
								10, 	// Hour
								16, 	// Min
								2, 		// Sec
								1};		// Day
bool ready=0;

uint16_t Num[10]={63, 6, 91, 79, 102, 109, 125, 7, 127, 111};
uint16_t Buf[4]={0, 0, 0, 0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)	
{
	if (! ready) 
	{	
		RxBuf[shift] = Rx;	
		shift++;
		HAL_UART_Receive_IT(&huart1, &Rx, 1);
		if (shift == RX_BUF_SIZE)
		{
			shift=0;
			if (RxBuf[RX_BUF_SIZE-1]==255)
			{
				ready=true;
			}	
		}
	}	
}


void GPIO_Write(uint16_t N) //заполнение массива двузначного числа
{
//	Buf[0] =	~Num[N/10];
//	Buf[1] =	~Num[N%10]+128;
	Buf[0] =	Num[N/1000];
	Buf[1] =	Num[(N/100)%10]+128;
	Buf[2] =	Num[(N/10)%10];
	Buf[3] =	Num[N%10]+128;
}

void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
		
	if(A >= Last) A=0;

	GPIOA->BSRR=65535 << 16u;//сбросить все
	GPIOB->BSRR=65535;//установить все		
	GPIOA->BSRR=(uint32_t) Buf[A];
	GPIOB->BSRR=(uint32_t) (1024<<A) << 16u ;

	A++;
	B++;
		
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#include "stm32f10x.h"                  // Device header
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
DS1302_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
HAL_TIM_Base_Start_IT(&htim1);	
//__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, htim1.Init.Period/2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	HAL_TIM_Base_Start_IT(&htim1);
	//#include <LID_func.h>
	uint8_t count_LED=0, on=1, off=10;
	bool up=1;
	uint16_t T=0;	
	uint16_t step = 9999;
	
	HAL_UART_Receive_IT(&huart1, &Rx, 1);
	
  while (1)
  {
		DS1302_ReadTimeBurst(Tim);
		GPIO_Write(Tim[4]*100+Tim[5]);
		
		if (ready)
		{
			for (int i=0; i<7; i++)
			{
			Init_Tim[i+1]=RxBuf[i];
				RxBuf[i]=0;
			}	
			Init_Tim[0]=1;
			RxBuf[8]=0;
			DS1302_WriteTime(Init_Tim);
		}
		/*	
	GPIOA->BSRR=65535 << 16u;//сбросить все
	GPIOB->BSRR=65535;//установить все		
	GPIOA->BSRR=(uint32_t) Num[6];
	GPIOB->BSRR=(uint32_t) (1024<<0) << 16u ;
	HAL_Delay(500);	
		
		GPIOA->BSRR=65535 << 16u;//сбросить все
	GPIOB->BSRR=65535;//установить все		
	GPIOA->BSRR=(uint32_t) Num[6];
	GPIOB->BSRR=(uint32_t) (1024<<1) << 16u ;
	HAL_Delay(500);
		
		GPIOA->BSRR=65535 << 16u;//сбросить все
	GPIOB->BSRR=65535;//установить все		
	GPIOA->BSRR=(uint32_t) Num[6];
	GPIOB->BSRR=(uint32_t) (1024<<2) << 16u ;
	HAL_Delay(500);
		
		GPIOA->BSRR=65535 << 16u;//сбросить все
	GPIOB->BSRR=65535;//установить все		
	GPIOA->BSRR=(uint32_t) Num[6];
	GPIOB->BSRR=(uint32_t) (1024<<3) << 16u ;
	HAL_Delay(500);
	*/
		//A=5/3;
		//B=7/3;
		//C=7%3;
		
		//TIM1_UP_IRQn
		//HAL_TIMEx_BreakCallback
		//GPIOA->BSRR=255;
		//HAL_GPIO_WritePin((GPIO_TypeDef *)GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
				
		//GPIO_Write(GPIOA,255,false);
		//GPIO_Write(GPIOA,102,true);
		
		if (ready) 
		{
			for(uint8_t i = 0; i < 9; i++)
			{
				TxBuf[i]=RxBuf[i];
				RxBuf[i]=0;
			}
			HAL_UART_Transmit_IT(&huart1, TxBuf, 9);
			ready=0;
		}
		
//GPIO_Write(GPIOA, 76, GPIO_PIN_SET);		
		
		//GPIOA->BSRR=(uint32_t)255 << 16u;//установить все
		//GPIOA->BSRR=255;//сбросить все 				
	
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 25-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, T_CLK_Pin|T_DAT_Pin|T_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Pos_A_Pin|Pos_B_Pin|Pos_C_Pin|Pos_D_Pin
                          |Pos_E_Pin|Pos_F_Pin|Pos_G_Pin|Point_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Ind_1_Pin|Ind_2_Pin|Ind_3_Pin|Ind_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : T_CLK_Pin T_DAT_Pin T_RST_Pin */
  GPIO_InitStruct.Pin = T_CLK_Pin|T_DAT_Pin|T_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Pos_A_Pin Pos_B_Pin Pos_C_Pin Pos_D_Pin
                           Pos_E_Pin Pos_F_Pin Pos_G_Pin Point_Pin */
  GPIO_InitStruct.Pin = Pos_A_Pin|Pos_B_Pin|Pos_C_Pin|Pos_D_Pin
                          |Pos_E_Pin|Pos_F_Pin|Pos_G_Pin|Point_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Ind_1_Pin Ind_2_Pin Ind_3_Pin Ind_4_Pin */
  GPIO_InitStruct.Pin = Ind_1_Pin|Ind_2_Pin|Ind_3_Pin|Ind_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
