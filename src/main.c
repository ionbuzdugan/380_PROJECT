/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
			

int main(void)
{
  HAL_Init();
  BSP_LED_Init(LED2);
  while(1){
    BSP_LED_Toggle(LED2);
    HAL_Delay(1000);
  }
}
