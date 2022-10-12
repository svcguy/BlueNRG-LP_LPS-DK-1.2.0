/**
******************************************************************************
* @file    BlueNRGLP_it.c 
* @author  AMS RF Application Team
* @version V1.0.0
* @date    25-March-2019
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
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "rf_device_it.h"

#include "system_BlueNRG_LP.h"
#include "bluenrg_lp_evb_button.h"
#include "rf_driver_hal_vtimer.h"
#include "rf_driver_ll_rtc.h"
#include "rf_driver_ll_lpuart.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples
* @{
*/



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern volatile uint8_t receivedChar;

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
//  HAL_CrashHandler(__get_MSP(), NMI_SIGNATURE);  
  /* Go to infinite loop when NMI exception occurs */
  while (1)
  {}
}

/**
* @brief  This function handles Hard Fault exception.
*/
NOSTACK_FUNCTION(NORETURN_FUNCTION(void HardFault_IRQHandler(void)))
{
//  HAL_CrashHandler(__get_MSP(), HARD_FAULT_SIGNATURE);  
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
  BSP_COM_IRQHandler();
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
  if (BSP_PB_GetITPendingBit(BSP_PUSH1)) {
    BSP_PB_ClearITPendingBit(BSP_PUSH1);
  } 
}

void BLE_WKUP_IRQHandler(void)
{
  HAL_VTIMER_WakeUpCallback();
}

/**
* @brief  This function handles CPU WAKEUP interrupt request.
* @param  None
* @retval None
*/
void CPU_WKUP_IRQHandler(void)
{
  HAL_VTIMER_TimeoutCallback();
}

/**
* @brief  This function handles RTC interrupt request.
* @param  None
* @retval None
*/
void RTC_IRQHandler(void)
{
  if(LL_RTC_IsActiveFlag_WUT(RTC)) {
    LL_RTC_ClearFlag_WUT(RTC);
  }
}

/**
* @brief  This function handles LPUART interrupt request.
* @param  None
* @retval None
*/
void LPUART1_IRQHandler(void)
{
  if(LL_LPUART_IsActiveFlag_RXNE_RXFNE(LPUART1) && LL_LPUART_IsEnabledIT_RXNE(LPUART1))
    receivedChar = LL_LPUART_ReceiveData8(LPUART1);
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
