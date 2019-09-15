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

uint16_t p1 = GPIO_PIN_7;
uint16_t p2 = GPIO_PIN_6;
uint16_t p3 = GPIO_PIN_10;
uint16_t p4 = GPIO_PIN_8;

void run_motor(float d){
  HAL_GPIO_WritePin(GPIOA,p1,1);
  HAL_GPIO_WritePin(GPIOA,p2,0);
  HAL_GPIO_WritePin(GPIOA,p3,0);
  HAL_GPIO_WritePin(GPIOA,p4,0);

  HAL_Delay(d);

  HAL_GPIO_WritePin(GPIOA,p1,0);
  HAL_GPIO_WritePin(GPIOA,p2,1);

  HAL_Delay(d);

  HAL_GPIO_WritePin(GPIOA,p2,0);
  HAL_GPIO_WritePin(GPIOA,p3,1);

  HAL_Delay(d);

  HAL_GPIO_WritePin(GPIOA,p3,0);
  HAL_GPIO_WritePin(GPIOA,p4,1);

  HAL_Delay(d);
}

void init_pins(){
  GPIO_InitTypeDef GPIO_InitStruct;
  
  HAL_Init();
  BSP_LED_Init(LED2);
  __GPIOA_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int main(void)
{
  init_pins();
  while(10){
    run_motor(10);
  }
}
