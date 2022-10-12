/**
  ******************************************************************************
  * @file    I2S/I2S_CCA01M1_ComIT/Src/I2S_CCA01M1_ComIT_main.c
  * @author  RF Application Team
  * @brief   This sample code shows how to use SPI HAL API to transmit
  *          and receive a data buffer with a communication process based on
  *          Interrupt transfer.
  *          The communication is done using 2 Boards.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "I2S_CCA01M1_ComIT_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
EXTI_HandleTypeDef HEXTI_InitStructure;
I2S_HandleTypeDef hi2s;

/* Private function prototypes -----------------------------------------------*/
static void EXTI10_IRQHandler_Config(void);

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    Error_Handler();
  }

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  /* Configure External line 10 (connected to PA.10 pin) in interrupt mode */
  EXTI10_IRQHandler_Config();

  if(Init_AudioOut_Device() != 0)
  {
    Error_Handler(); 
  }
  
  /* Start Audio Streaming*/
  if(Start_AudioOut_Device() != 0)
  {
    Error_Handler(); 
  }
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  Configures EXTI line 10 (connected to PA.10 pin) in interrupt mode
  * @param  None
  * @retval None
  */
static void EXTI10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  EXTI_ConfigTypeDef EXTI_Config_InitStructure;
  
  /* Enable GPIOC clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Configure PA.10 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  EXTI_Config_InitStructure.Line =    EXTI_LINE_PA10;
  EXTI_Config_InitStructure.Trigger = EXTI_TRIGGER_RISING_EDGE;
  EXTI_Config_InitStructure.Type =    EXTI_TYPE_EDGE;
   
  HAL_EXTI_SetConfigLine(&HEXTI_InitStructure, &EXTI_Config_InitStructure);
  HAL_EXTI_RegisterCallback(&HEXTI_InitStructure, HAL_EXTI_COMMON_CB_ID, Example_EXTI_Callback);
  HAL_EXTI_Cmd(&HEXTI_InitStructure , ENABLE);
  
  HAL_EXTI_ClearPending(&HEXTI_InitStructure);
  
  /* Enable and set line 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(GPIOA_IRQn, IRQ_HIGH_PRIORITY);
  HAL_NVIC_EnableIRQ(GPIOA_IRQn);
  
  /* Configure NVIC for SysTick_IRQn */
  HAL_NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}   
                                                 
/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void Example_EXTI_Callback(uint32_t Line)
{
    Switch_Demo();
}
                                 
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  printf("\n\r** Error_Handler **\n\r\n\r");
  /* User can add his own implementation to report the HAL error return state */
  while(1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



