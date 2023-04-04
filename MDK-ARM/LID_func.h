//#include "stm32f1xx_hal.h"

//#define LED_Pin GPIO_PIN_13
//#define LED_GPIO_Port GPIOC

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