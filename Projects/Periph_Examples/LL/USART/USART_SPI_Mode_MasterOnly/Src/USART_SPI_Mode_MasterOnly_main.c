/**
******************************************************************************
* @file    LL/USART/USART_Tx_IT_Init/Src/USART_SPI_Mode_MasterOnly_main.c
* @author  RF Application Team
* @brief   This example describes how to send bytes over USART IP using
*          the USART LL API.
*          Peripheral initialization done using LL unitary services functions.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "USART_SPI_Mode_MasterOnly_main.h"

/* Private includes */

/* Private typedef */

/* Private define */

/* Private macro */

/* Private variables */
__IO uint8_t ubButtonPress = 0;
__IO uint8_t ubSend = 0;
const uint8_t aStringToSend[] = "**** SPI_IT communication **** SPI_IT communication **** SPI_IT communication ****";
uint8_t ubSizeToSend = sizeof(aStringToSend);

/* Private function prototypes */
static void LL_Init(void);
static void MX_USART_Init(void);
void WaitForUserButtonPress(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_Init();
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of LEDs */
  BSP_LED_Init(BSP_LED3);
  
/* Initialize buttons as interrupt source */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* Initialize all configured peripherals */
  MX_USART_Init();
  
 /**   
  *   To test this SPI Master application was been used the LL/SPI/SPI_IT_Slave_Init with the SPI configured 
  *   as SPI1 and the data size set to 8bit.
  *   It is mandatory add to the slave application the follows define in the main.h file:
  *   #define CONFIG_DATASIZE_8BIT  1
  *   #define USE_SPI1_PINS         1
  *   
  *   |    MASTER        |      SLAVE      |
  *   | USART Pin/Signal | SPI Pin/Signal  | 
  *   |   PB8 / CK       |   PA13 / SCK    |
  *   |   PA8 / RX       |   PA14 / MISO   |
  *   |   PA9 / TX       |   PB14 / MOSI   |
  *   |   PA1 / TX       |   PA11 / CS     |
  *   
  */
  
  /* Wait for User push-button (PUSH1) press to start transfer */
  WaitForUserButtonPress();
  
  /* Infinite loop */
  while (1)
  {
  }
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
* @brief  Wait for User push-button (PUSH1) press to start transfer.
* @param  None
* @retval None
*/
/*  */
void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
    BSP_LED_Toggle(BSP_LED3);
    LL_mDelay(LED_BLINK_FAST);
  }
}

/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_USART_ClockInitTypeDef USART_ClockInitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock enable */
  LL_EnableClock_USART();
  
  GPIO_InitStruct.Pin = USART1_CK_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = USART1_CK_AF;
  LL_GPIO_Init(USART1_CK_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_TX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = USART1_TX_AF;
  LL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_RX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = USART1_RX_AF;
  LL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);
  
  /* CS */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_GPIO_SetOutputPin(GPIO_PORT_MASTER_CS, GPIO_PIN_SPI_MASTER_CS);
  GPIO_InitStruct.Pin = GPIO_PIN_SPI_MASTER_CS;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIO_PORT_MASTER_CS, &GPIO_InitStruct);
  
  /* Enable the IO pull-down configuration for USART TX and RX pins */
  LL_PWR_EnablePDA_USART_TX();//not work mm1mm
  LL_PWR_EnablePDA_USART_RX();
  
  LL_GPIO_ResetOutputPin(USART1_TX_PORT, USART1_TX_PIN);
  LL_GPIO_ResetOutputPin(USART1_RX_PORT, USART1_RX_PIN);
    
  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USART1_IRQn);
  
  USART_ClockInitStruct.ClockOutput = LL_USART_CLOCK_ENABLE;
  USART_ClockInitStruct.ClockPhase = LL_USART_PHASE_2EDGE;
  USART_ClockInitStruct.ClockPolarity = LL_USART_POLARITY_HIGH;
  USART_ClockInitStruct.LastBitClockPulse = LL_USART_LASTCLKPULSE_OUTPUT; /* USART_CR2_LBCL */
  LL_USART_ClockInit(USART1, &USART_ClockInitStruct);
  
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 1000000;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(USART1, &USART_InitStruct);
  
  LL_USART_SetTransferBitOrder(USART1, LL_USART_BITORDER_MSBFIRST);
  
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigSyncMode(USART1);
  LL_USART_Enable(USART1);
  
  /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  
  /* Enable Error interrupts */
  LL_USART_EnableIT_ERROR(USART1);
}



/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT Functions                                     */
/******************************************************************************/
/**
* @brief  Function to manage User push-button (PUSH1)
* @param  None
* @retval None
*/
void UserButton_Callback(void)
{
  /* Start transfer only if not already ongoing */
  if (ubSend == 0)
  {
    LL_GPIO_ResetOutputPin(GPIO_PORT_MASTER_CS, GPIO_PIN_SPI_MASTER_CS);
    
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]); 
    
    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART1); 
  }
}

/**
* @brief  Function called for achieving next TX Byte sending
* @param  None
* @retval None
*/
void USART_TXEmpty_Callback(void)
{
  if(ubSend == (ubSizeToSend - 1))
  {
    /* Disable TXE interrupt */
    LL_USART_DisableIT_TXE(USART1);
    
    /* Enable TC interrupt */
    LL_USART_EnableIT_TC(USART1);
  }
  
  /* Fill TDR with a new char */
  LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]);
}

/**
* @brief  Function called at completion of last byte transmission
* @param  None
* @retval None
*/
void USART_CharTransmitComplete_Callback(void)
{
  if(ubSend == sizeof(aStringToSend))
  {
    ubSend = 0;
    
    /* Disable TC interrupt */
    LL_USART_DisableIT_TC(USART1);
    
    LL_GPIO_SetOutputPin(GPIO_PORT_MASTER_CS, GPIO_PIN_SPI_MASTER_CS);
    
    BSP_LED_On(BSP_LED3); 
  }
}

/**
* @brief  Function called in case of error detected in USART IT Handler
* @param  None
* @retval None
*/
void Error_Callback(void)
{
  __IO uint32_t isr_reg;
  
  /* Disable USARTx_IRQn */
  NVIC_DisableIRQ(USART1_IRQn);
  
  /* Error handling example :
  - Read USART ISR register to identify flag that leads to IT raising
  - Perform corresponding error handling treatment according to flag
  */
  isr_reg = LL_USART_ReadReg(USART1, ISR);
  if (isr_reg & LL_USART_ISR_NE)
  {
    /* case Noise Error flag is raised : ... */
    Error_Handler();
  }
  else
  {
    /* Unexpected IT source */
    Error_Handler();
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
    BSP_LED_Toggle(BSP_LED3);
    LL_mDelay(LED_BLINK_ERROR);
  }
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
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


