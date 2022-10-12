/**
  ******************************************************************************
  * @file    LL/I2C/I2C_IT/Src/rf_device_it.c
  * @author  RF Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "rf_device_it.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
 


/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_IRQHandler(void)
{
  while (1)
  {
  }
}

#define DEBOUNCE_CNT  350
volatile uint32_t debounce_count = 0;

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/
void SysTick_IRQHandler(void)
{
  debounce_count++;  
}

/******************************************************************************/
/* Peripheral Interrupt Handlers                                              */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

#ifdef SLAVE_BOARD

/**
  * Brief   This function handles I2Cx (Slave) event interrupt request.
  * Param   None
  * Retval  None
  */
void I2Cx_IRQHandler(void)
{
  /* Check ADDR flag value in ISR register */
  if(LL_I2C_IsActiveFlag_ADDR(I2Cx))
  {
    /* Verify the Address Match with the OWN Slave address */
    if(LL_I2C_GetAddressMatchCode(I2Cx) == SLAVE_OWN_ADDRESS)
    {
      /* Verify the transfer direction, a read direction, Slave enters transmitter mode */
      if(LL_I2C_GetTransferDirection(I2Cx) == LL_I2C_DIRECTION_READ)
      {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2Cx);

        /* Enable Transmit Interrupt */
        LL_I2C_EnableIT_TX(I2Cx);
      }
      else
      {
        /* Clear ADDR flag value in ISR register */
        LL_I2C_ClearFlag_ADDR(I2Cx);

        /* Call Error function */
        Error_Callback();
      }
    }
    else
    {
      /* Clear ADDR flag value in ISR register */
      LL_I2C_ClearFlag_ADDR(I2Cx);
        
      /* Call Error function */
      Error_Callback();
    }
  }
  /* Check NACK flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_NACK(I2Cx))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_NACK(I2Cx);
  }
  /* Check TXIS flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_TXIS(I2Cx))
  {
    /* Call function Slave Ready to Transmit Callback */
    Slave_Ready_To_Transmit_Callback();
  }
  /* Check STOP flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_STOP(I2Cx))
  {
    /* Clear STOP flag value in ISR register */
    LL_I2C_ClearFlag_STOP(I2Cx);
    
    /* Check TXE flag value in ISR register */
    if(!LL_I2C_IsActiveFlag_TXE(I2Cx))
    {
      /* Flush the TXDR register */
      LL_I2C_ClearFlag_TXE(I2Cx);
    }

    /* Call function Slave Complete Callback */
    Slave_Complete_Callback();
  }
  /* Check TXE flag value in ISR register */
  else if(!LL_I2C_IsActiveFlag_TXE(I2Cx))
  {
    /* Do nothing */
    /* This Flag will be set by hardware when the TXDR register is empty */
    /* If needed, use LL_I2C_ClearFlag_TXE() interface to flush the TXDR register  */
  }
  else
  {
    /* Call Error function */
    Error_Callback();
  }
}


#else /* MASTER_BOARD */

/**
  * Brief   This function handles I2Cx (Master) interrupt request.
  * Param   None
  * Retval  None
  */
void I2Cx_IRQHandler(void)
{
  /* Check RXNE flag value in ISR register */
  if(LL_I2C_IsActiveFlag_RXNE(I2Cx))
  {
    /* Call function Master Reception Callback */
    Master_Reception_Callback();
  }
  /* Check STOP flag value in ISR register */
  else if(LL_I2C_IsActiveFlag_STOP(I2Cx))
  {
    /* End of Transfer */
    LL_I2C_ClearFlag_STOP(I2Cx);

    /* Call function Master Complete Callback */
    Master_Complete_Callback();
  }
  else
  {
    /* Call Error function */
    Error_Callback();
  }
}


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
* @brief  This function handles GPIOA interrupt request.
* @param  None
* @retval None
*/
void GPIOA_IRQHandler(void)
{
  USER_BUTTON_IRQHandler();
}

/**
* @brief  This function handles user button interrupt request.
* @param  None
* @retval None
*/
void USER_BUTTON_IRQHandler(void)
{
  static uint32_t debounce_last = 0;
  
  if (LL_EXTI_IsInterruptPending(USER_BUTTON_EXTI_LINE) != RESET)
  {
    LL_EXTI_ClearInterrupt(USER_BUTTON_EXTI_LINE);
    
    if ( (debounce_count - debounce_last) >= DEBOUNCE_CNT )
    {
      /* Add the SW no bounce */
      debounce_last = debounce_count;
      /* Handle user button press in dedicated function */
      UserButton_Callback();
    }
  }
}



#endif /* SLAVE_BOARD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



