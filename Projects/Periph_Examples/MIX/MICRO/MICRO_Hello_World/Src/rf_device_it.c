/**
******************************************************************************
* @file    rf_device_it.c 
* @author  RF Application Team
* @version V1.0.0
* @date    19-November-2020
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2020 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "rf_device_it.h"
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "system_BlueNRG_LP.h"
#endif
#include "hal_miscutil.h" 

/** @addtogroup StdPeriph_Examples
* @{
*/



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  */
NOSTACK_FUNCTION(NORETURN_FUNCTION(void NMI_IRQHandler(void)))
{
  HAL_CrashHandler(__get_MSP(), NMI_SIGNATURE);  
  /* Go to infinite loop when NMI exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles Hard Fault exception.
*/
NOSTACK_FUNCTION(NORETURN_FUNCTION(void HardFault_IRQHandler(void)))
{
  HAL_CrashHandler(__get_MSP(), HARD_FAULT_SIGNATURE);  
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles SysTick Handler.
*/
void SysTick_IRQHandler(void)
{
}


/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/
/**
* @brief  This function handles UART interrupt request.
* @param  None
* @retval None
*/

void USART1_IRQHandler(void)
{  

}


/**
* @brief  This function handles DMA Handler.
*/
void DMA_IRQHandler(void)
{
  
}


/**
* @brief  This function handles GPIO interrupt request.
* @param  None
* @retval None
*/

void GPIOA_IRQHandler(void)
{

  
}

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/ 

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/



