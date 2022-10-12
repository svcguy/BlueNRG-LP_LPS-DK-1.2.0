/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BLE_SerialPort_Master_Slave_main.c
* Author             : RF Application Team 
* Version            : V1.0.0
* Date               : 12-June-2019
* Description        : NUCLEO-L152RE network coprocessor main file for SerialPort Master & Slave demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
 * BlueNRG-LP SerialPort Master and Slave demo \see BLE_SerialPort_Master_Slave_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "hal_types.h"
#include "hci.h"
#include "gp_timer.h"
#include "hal.h"
#include "osal.h"
#include "hci_const.h"
#include "bluenrg_lp_types.h"
#include "bluenrg_lp_gap.h"
#include "bluenrg_lp_aci.h"
#include "bluenrg_lp_hci_le.h"
#include "SDK_EVAL_Config.h"
#include "serial_port.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BLE_NEW_CHAT_VERSION_STRING "1.0.0" 

#ifndef DEBUG
#define DEBUG 1
#endif

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemInit_NWK(void);

/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  uint8_t ret;
  
  SystemInit_NWK();
  
  /* BlueNRG-LP stack init */
  ret = BlueNRG_Stack_Initialization();
  if (ret != BLE_STATUS_SUCCESS) {
    printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  printf("BlueNRG-LP BLE SerialPort Master & Slave Application (version: %s)\r\n", BLE_NEW_CHAT_VERSION_STRING); 

  /* Init SerialPort Device */
  ret = SerialPort_DeviceInit();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("SerialPort_DeviceInit()--> Failed 0x%02x\r\n", ret);
    while(1);
  }
  
  PRINTF("BLE Stack Initialized & Device Configured\r\n");

  while(1) {
    /* BlueNRG-LP stack tick */
    BTLE_StackTick();

    /* Application tick */
    APP_Tick();
  }
}


/* Event used to notify the Host that a hardware failure has occurred in the Controller. 
   See bluenrg_lp_events.h. */
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}

/**
  * This event is generated to report firmware error informations.
  * FW_Error_Type possible values: 
  * Values:
  - 0x01: L2CAP recombination failure
  - 0x02: GATT unexpected response
  - 0x03: GATT unexpected request
    After this event with error type (0x01, 0x02, 0x3) it is recommended to disconnect. 
*/
void aci_hal_fw_error_event(uint8_t FW_Error_Type,
                            uint8_t Data_Length,
                            uint8_t Data[])
{
  if (FW_Error_Type <= 0x03)
  {
    uint16_t connHandle;
    
    /* Data field is the connection handle where error has occurred */
    connHandle = LE_TO_HOST_16(Data);
    
    aci_gap_terminate(connHandle, BLE_ERROR_TERMINATED_REMOTE_USER); 
  }
}

void SystemInit_NWK(void)
{
  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  
  /* Configure the BlueNRG-LP pins - RESET pin */
  Sdk_Eval_Reset_Pin_Init();
  
#ifdef SPI_INTERFACE
  /* Init SPI interface */
  SdkEvalSpiInit();
#else

#ifdef UART_INTERFACE
  DTM_IO_Config();
#endif
#endif

  /* Configure I/O communication channel:
       It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
       where user received data should be processed */
  SdkEval_IO_Config(Process_InputData);
  
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 48000000  32000000
  *            HCLK(Hz)                       = 48000000  32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 12 6
  *            PLLDIV                         = 4  3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
#if (HCLK_32MHZ == 1)
  /* Enable ACC64 access and set FLASH latency */
  
#ifdef STM32L152xE
  LL_FLASH_Enable64bitAccess();
#endif
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  LL_RCC_HSI_SetCalibTrimming(0x10);

  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
  /* Main PLL configuration and activation */
#ifdef STM32L476xx
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
#endif
#ifdef STM32L152xE
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_12, LL_RCC_PLL_DIV_4); // LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);
#endif
  
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1) 
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) 
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  
#ifdef STM32L476xx
  SysTick_Config(80000000/1000);
  LL_SetSystemCoreClock(80000000);
#endif
#ifdef STM32L152xE
  SysTick_Config(48000000/1000);   // 32000000/1000
  LL_SetSystemCoreClock(48000000); // 32000000
#endif
#endif
  
  
#if (SYSCLK_MSI == 1)
  /* Set FLASH latency */ 
#ifdef STM32L152xE
  LL_FLASH_Enable64bitAccess();
#endif
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Set MSI Range */
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  
  /* Enable MSI if not already activated*/
  if (LL_RCC_MSI_IsReady() == 0)
  {
    /* MSI configuration and activation */
    LL_RCC_MSI_Enable();
    while(LL_RCC_MSI_IsReady() != 1)
    {
    };
  }
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  RCC_HCLKConfig(RCC_CFGR_HPRE_DIV1);

  SysTick_Config(4000000/1000);
  LL_SetSystemCoreClock(4000000);
#endif
  
}



#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
