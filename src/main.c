/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_usart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "structures.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;
GPIO_TypeDef *PIN_GPIO[12] = {
                              GPIOA,
                              GPIOB,
                              GPIOB,
                              GPIOB,
                              GPIOB,
                              GPIOA,
                              GPIOA,
                              GPIOC,
                              GPIOB,
                              GPIOA,
                              GPIOA,
                              GPIOA
};

uint16_t PIN_NUM[12] = {
                    GPIO_PIN_10,
                    GPIO_PIN_3,
                    GPIO_PIN_5,
                    GPIO_PIN_4,
                    GPIO_PIN_10,
                    GPIO_PIN_8,
                    GPIO_PIN_9,
                    GPIO_PIN_7,
                    GPIO_PIN_6,
                    GPIO_PIN_7,
                    GPIO_PIN_6,
                    GPIO_PIN_5
};

int PIN_MOTORS[6][4] =  {
                          {0,1,2,3},
                          {4,5,6,7},
                          {8,9,10,11},
                          {1,0,3,2},
                          {5,4,7,6},
                          {9,8,11,10}
};

int speeds[6] = {0,0,0,0,0,0};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MotorCmd_Init(MotorCmdFrame *m){
  int i;
  for(i=0;i<6;i++){
    m->payload[i] = 0;
  }
  m->new = false;
}

void SendString (char *msg){
  while(HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),1000) != HAL_OK)
  {}
}

void Connect_Serial(void){
  uint8_t *flagByte = malloc(sizeof(*flagByte));
  do{
    HAL_UART_Receive(&huart2,flagByte,sizeof(*flagByte),1000);
  }while(*flagByte != LSB_CONNECT);
  free(flagByte);
  SendString("ACK");
  SendString("ACK");
  SendString("ACK");
}

uint8_t Listen_For_Command(int timeout){
  uint8_t *flagByte = malloc(sizeof(*flagByte));
  HAL_UART_Receive(&huart2,flagByte,sizeof(*flagByte),timeout);
  uint8_t ret = *flagByte;
  free(flagByte);
  return ret;
}

void Stop_Motor (int motorNum){
  int i;
  for(i = 0; i<4; i++){
    HAL_GPIO_WritePin(PIN_GPIO[PIN_MOTORS[motorNum][i]],PIN_NUM[PIN_MOTORS[motorNum][i]],0);
  }
}

void Run_Motor(int motorNum){
  int i;
  int j;
  for(i = 0; i<4; i++){
    for(j=0; j<4; j++){
      if(j==i){
        HAL_GPIO_WritePin(PIN_GPIO[PIN_MOTORS[motorNum][j]],PIN_NUM[PIN_MOTORS[motorNum][j]],1);
      }
      else{
        HAL_GPIO_WritePin(PIN_GPIO[PIN_MOTORS[motorNum][j]],PIN_NUM[PIN_MOTORS[motorNum][j]],0);
      }
    }
    HAL_Delay(MOTOR_DELAY);
  }
}

void Run_Motor_Rev(int motorNum){
  int i;
  int j;
  for(i = 3; i>=0; i--){
    for(j=0; j<4; j++){
      if(j==i){
        HAL_GPIO_WritePin(PIN_GPIO[PIN_MOTORS[motorNum][j]],PIN_NUM[PIN_MOTORS[motorNum][j]],1);
      }
      else{
        HAL_GPIO_WritePin(PIN_GPIO[PIN_MOTORS[motorNum][j]],PIN_NUM[PIN_MOTORS[motorNum][j]],0);
      }
    }
    HAL_Delay(MOTOR_DELAY);
  }
}

void Spin_Motors (void){
  int i;
  int j;
  // Loop only for max cycles
  for (i = 0; i<MAX_CYCLES_PER_STEP; i++){
    // Run each motor every cycle
    for (j = 0; j<6; j++){
      if (speeds[j] == 0){
          Stop_Motor(j);
      }
      else{
        if ((i+1)%abs(speeds[j]) == 0){
          if (speeds[j] > 0){
            Run_Motor(j);
          }
          else{
            Run_Motor_Rev(j);
          }
        }
      }
    }
  }
}

uint8_t MotorMsgParse (MotorCmdFrame *motorCmd){
  int i;
  motorCmd->new = false;
  // Convert speeds to cycles/step
  // 0 means no speed
  // 1-100 == 1-20 (same in negatives)
  for (i = 0; i<6; i++){
    speeds[i] = 0;
  }
  // Iterate through motors
  for (i = 0; i<6; i++){
    // Get motor value for each motor (-20 -> 20 scale)
    if (motorCmd->payload[i] == 0){
      speeds[i] = 0;
    }
    else{
      speeds[i] = (MAX_CYCLES_PER_STEP-1)*(1-(abs(motorCmd->payload[i])-1)/(MAX_MOTOR_CMD-1)) + 1;
      speeds[i] = speeds[i] * abs(motorCmd->payload[i]) / motorCmd->payload[i]; //preserve sign
    }
  }
  Spin_Motors();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Variables
  MotorCmdFrame *motorCmd = malloc(sizeof(*motorCmd));
  MotorCmd_Init(motorCmd);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED2);
  UART_HandleTypeDef *retStat;
  Connect_Serial();
  uint8_t msgId;
  while (1)
  {
    msgId = Listen_For_Command(1);
    // Get and process command
    if (msgId == LSB_MOTOR){
//      do{
        HAL_UART_Receive(&huart2,motorCmd,sizeof(*motorCmd),1);
//      }while(motorCmd->new == false);
    }
    MotorMsgParse(motorCmd);
    msgId = 0x00;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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

  GPIO_InitStruct.Pin = GPIO_PIN_2 |GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  //Init motor pins
  // GPIO A
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // GPIO B
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // GPIO C
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
