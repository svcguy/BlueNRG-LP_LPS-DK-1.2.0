
/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* File Name          : Micro_PowerSave_main.c
* Author             : RF Application Team
* Version            : V1.0.0
* Date               : 25-March-2019
* Description        : Code demonstrating Power Save with BlueNRG-LP
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"

#include "bluenrg_lp_evb_com.h"
#include "bluenrg_lp_evb_led.h"
#include "rf_driver_ll_rcc.h"
#include "rf_driver_ll_rtc.h"
#include "rf_driver_ll_lpuart.h"
#include "hal_miscutil.h"
#include "rf_driver_hal_power_manager.h"
#include "rf_driver_hal_vtimer.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples BlueNRG-LP Peripheral Examples
  * @{
  */


/** @addtogroup Micro_Examples Micro Examples
  * @{
  */

/** @addtogroup Micro_PowerManager  Micro Power Manager Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#if defined CONFIG_HW_LS_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        1000

#elif defined CONFIG_HW_LS_XTAL

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* High Speed start up time */
#define HS_STARTUP_TIME 328 // 800 us 

#define WAKEUP_TIMEOUT 5000


#if defined CONFIG_DEVICE_BLUENRG_LPS
#define WAKEUP_UART_RX_PIN WAKEUP_PB0
#endif

#if defined CONFIG_DEVICE_BLUENRG_LP
#define WAKEUP_UART_RX_PIN WAKEUP_PA8
#endif

/* Public variables ----------------------------------------------------------*/  
volatile uint8_t receivedChar=0;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
static VTIMER_HandleType timerHandle;

/* Private function prototypes -----------------------------------------------*/
void help(void);
void PrintNegotiatedLevel(uint8_t stopLevel);
void PrintWakeupSource(uint32_t wakeupSources);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Display the help functions.
  * @param  None
  * @retval None
  */
void help(void)
{
  printf("POWER MANAGER help commands:\r\n");
  printf("s:   SHUTDOWN LEVEL : the only wakeup source is a low pulse on the RSTN pad\r\n");
  printf("t:   POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART RX pin/timeout 5 sec (VTIMER)/button PUSH1 (PA10)\r\n");
  printf("z:   POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART RX pin/timeout 5 sec (RTC)/button PUSH1 (PA10)\r\n");
  printf("u:   POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on LPUART/button PUSH1 (PA10)\r\n");
  printf("n:   POWER_SAVE_LEVEL_NOTIMER : wake on UART RX pin/button PUSH1 (PA10)\r\n");
  printf("c:   POWER_SAVE_LEVEL_CPU_HALT : wake on button PUSH1 (PA10)\r\n");
  printf("l:   Toggle led LED1\r\n");
  printf("p:   Print Hello World message\r\n");
  printf("r:   Reset the BlueNRG-LP\r\n");
  printf("?:   Display this help menu\r\n");
  printf("\r\n> ");
}

/**
  * @brief  Display the Stop Level negotiated.
  * @param  stopLevel negotiated Stop level
  * @retval None
  */
void PrintNegotiatedLevel(uint8_t stopLevel)
{
  printf("Power save level negotiated: ");
  switch (stopLevel)
  { 
  case POWER_SAVE_LEVEL_RUNNING:
    printf ("RUNNING\r\n");
    break;
  case POWER_SAVE_LEVEL_CPU_HALT:
    printf ("CPU_HALT\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_WITH_TIMER:
    printf ("STOP_WITH_TIMER\r\n");
    break;
  case POWER_SAVE_LEVEL_STOP_NOTIMER:
    printf ("STOP_NOTIMER\r\n");
    break;
  }
}

/**
  * @brief  Display the Wakeup Source.
  * @param  wakeupSource Wakeup Sources
  * @retval None
  */
void PrintWakeupSource(uint32_t wakeupSources)
{  
  uint8_t wakeup_reasons[][30] = {"WAKEUP_PB0", "WAKEUP_PB1", "WAKEUP_PB2", "WAKEUP_PB3",  "WAKEUP_PB4",
                                "WAKEUP_PB5", "WAKEUP_PB6", "WAKEUP_PB7", "WAKEUP_PA8",  "WAKEUP_PA9",
                                "WAKEUP_PA10", "WAKEUP_PA11", "WAKEUP_BLE", "WAKEUP_BLE_HOST_TIMER",
                                "WAKEUP_LPU", "WAKEUP_RTC",
                                "WAKEUP_PA0", "WAKEUP_PA1", "WAKEUP_PA2", "WAKEUP_PA3",
                                "WAKEUP_PB12", "WAKEUP_PB13", "WAKEUP_PB14", "WAKEUP_PB15"};
                                    
                                
  if (wakeupSources != 0) {
    printf("Wakeup Sources : ");
    for (int i=0; i<32; i++) {
      if ((wakeupSources >> i) & 1) {
        printf("%s ", wakeup_reasons[i]);
      }
    }
    printf("\r\n");
  }
}

void RTC_WakeupInit(void)
{
  /* Enable Peripheral Clock */
  LL_APB0_EnableClock(LL_APB0_PERIPH_RTC);
  for (volatile int i=0; i<0xFFFF; i++);
  
  /* Disable the write protection for RTC registers */
  LL_RTC_DisableWriteProtection(RTC);
  
  /* Init mode setup */
  LL_RTC_EnableInitMode(RTC);
  
  /* Wait till the Init mode is active */
  while(LL_RTC_IsActiveFlag_INIT(RTC) == RESET);
  
  /* Configure Hour Format */
  LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
  
  /* Output disabled */
  LL_RTC_SetAlarmOutEvent(RTC, LL_RTC_ALARMOUT_DISABLE);
  
  /* Output polarity */
  LL_RTC_SetOutputPolarity(RTC, LL_RTC_OUTPUTPOLARITY_PIN_HIGH);
  
  /* Set Synchronous prescaler factor */
  LL_RTC_SetSynchPrescaler(RTC, 31);
  
  /* Set Asynchronous prescaler factor */
  LL_RTC_SetAsynchPrescaler(RTC, 0);
  
  /* Exit Initialization mode */
  LL_RTC_DisableInitMode(RTC);

  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
}

void SetRTC_WakeupTimeout(uint32_t time)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);

  /* Disable Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* In case of interrupt mode is used, the interrupt source must disabled */
  LL_RTC_DisableIT_WUT(RTC);
  
  /* Wait till RTC WUTWF flag is set  */
  while(LL_RTC_IsActiveFlag_WUTW(RTC) == 0);
  
  /* Clear PWR wake up Flag */
  LL_PWR_ClearWakeupSource(LL_PWR_EWS_INT);
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);
  
  /* Configure the Wake-up Timer counter */
  LL_RTC_WAKEUP_SetAutoReload(RTC, time);
  
  /* Configure the clock source */
  LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
  
  /* Configure the Interrupt in the RTC_CR register */
  LL_RTC_EnableIT_WUT(RTC);
  
  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Enable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);
  
  /* Configure NVIC for RTC */
  NVIC_SetPriority(RTC_IRQn, IRQ_LOW_PRIORITY);
  NVIC_EnableIRQ(RTC_IRQn);    
  
  /* Clear RTC Wake Up timer Flag */
  LL_RTC_ClearFlag_WUT(RTC);

}

void DisableRTC_WakeupTimeout(void)
{
  /* Disable write protection */
  LL_RTC_DisableWriteProtection(RTC);

  /* Enable the Wake-up Timer */
  LL_RTC_WAKEUP_Disable(RTC);
  
  /* Enable write protection */
  LL_RTC_EnableWriteProtection(RTC);

}

#if defined CONFIG_DEVICE_BLUENRG_LPS
void Configure_LPUART(void)
{
  /* Reset LPUART */
  LL_APB1_ForceReset(LL_APB1_PERIPH_LPUART);
  LL_APB1_ReleaseReset(LL_APB1_PERIPH_LPUART);
  
  /* Enable LPUART clock */
  LL_APB1_EnableClock(LL_APB1_PERIPH_LPUART);
  
  /* Enable GPIOB clock */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  
  /* Configure TX Pin as : Alternate function, High Speed, PushPull, No-Pull */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_4, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_4, LL_GPIO_AF_0);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_4, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_4, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_4, LL_GPIO_PULL_NO);
  
  /* Configure RX Pin as : Alternate function, High Speed, PushPull, No-Pull */
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_5, LL_GPIO_AF_0);
  LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
  
  /* NVIC Configuration for LPUART1 interrupts */
  NVIC_SetPriority(LPUART1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(LPUART1_IRQn);

  /* Set LPUART Clock source */
  if(LL_RCC_LSE_IsEnabled())
  {
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUCLKSEL_LSE);
  }
  
  /* TX/RX direction */
  LL_LPUART_SetTransferDirection(LPUART1, LL_LPUART_DIRECTION_TX_RX);
  
  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_LPUART_ConfigCharacter(LPUART1, LL_LPUART_DATAWIDTH_8B, LL_LPUART_PARITY_NONE, LL_LPUART_STOPBITS_1);
  
  /* Set Baudrate to 9600 using LSE frequency set to HSI_VALUE */
  LL_LPUART_SetBaudRate(LPUART1, LL_LPUART_PRESCALER_DIV1, 9600); 
  
  /* Enable LPURART RX not empty Interrupt */
  LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);  
  
  /* Enable LPUART */
  LL_LPUART_Enable(LPUART1);
}

void Disable_LPUART(void)
{
  /* Disable LPUART clock */
  LL_APB1_DisableClock(LL_APB1_PERIPH_LPUART);
}
#endif

/**
  * @brief  Timeout callback.
  * @param  Timer handle
  * @retval None
  */
void TimeoutCallback(void *timerHandle)
{
  /* Add app code to execute @ Stop timeout */
  printf("Vtimer Timeout!!!!!\r\n");
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t data, ret_val, tmp;
  PartInfoType partInfo; 
  WakeupSourceConfig_TypeDef wakeupIO;
  uint32_t wakeupSources;
  PowerSaveLevels stopLevel;
  HAL_VTIMER_InitType VTIMER_InitStruct;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_DIRECT_HSE, BLE_SYSCLK_16M) != SUCCESS) {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Configure IOs for pwer save modes */
  BSP_IO_Init();
  
  /* Init the UART peripheral */
  BSP_COM_Init(BSP_COM_RxDataUserCb);
  
  /* Init LED1 */
  BSP_LED_Init(BSP_LED1);
  
  /* Init BUTTON 1 */
  BSP_PB_Init(BSP_PUSH1, BUTTON_MODE_EXTI);
  
  /* VTimer module Init */
  VTIMER_InitStruct.XTAL_StartupTime = HS_STARTUP_TIME;
  VTIMER_InitStruct.EnableInitialCalibration = INITIAL_CALIBRATION;
  VTIMER_InitStruct. PeriodicCalibrationInterval = CALIBRATION_INTERVAL;
  HAL_VTIMER_Init(&VTIMER_InitStruct);
  timerHandle.callback = TimeoutCallback;

  /* RTC Wakeup Peripheral Init */
  RTC_WakeupInit();
    
  printf("Power Manager FW demo!\r\nDigit ? for help command\r\n");
  
  while(1) {
    /* To run the VTIMER state machine */
    HAL_VTIMER_Tick();
    
    if (BSP_COM_Read(&data)) {
      switch(data)
      {
      case 's':   
        { 
          /* SHUTDOWN LEVEL : the only wakeup source is a low pulse on the RSTN pad */
          printf("Enable Power Save Request : SHUTDOWN\r\n");
          while(BSP_COM_UARTBusy());
          if (HAL_PWR_MNGR_ShutdownRequest(TRUE) != SUCCESS)
            printf("ERORR during the SHUTDOWN Request!\r\n");
        }
        break;
      case 't':
        {
          /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART RX pin/timeout 5 sec (VTIMER)/button PUSH1 (PA10) */
          printf("Enable Power Save Request : STOP_WITH_TIMER (VTIMER)\r\n");
          while(BSP_COM_UARTBusy());          
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = WAKEUP_UART_RX_PIN;
          wakeupIO.RTC_enable = 0;
          wakeupIO.LPU_enable = 0;
          ret_val = HAL_VTIMER_StartTimerMs(&timerHandle, WAKEUP_TIMEOUT);
          if (ret_val != SUCCESS) {
            printf("HAL_VTIMER_StartTimerMs() error 0x%02x\r\n", ret_val);
            while(1);
          }
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
        }
        break;
      case 'z':
        {
          /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on UART RX pin/timeout 5 sec (RTC)/button PUSH1 (PA10) */
          printf("Enable Power Save Request : STOP_WITH_TIMER (RTC)\r\n");
          while(BSP_COM_UARTBusy());          
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = WAKEUP_UART_RX_PIN;
          wakeupIO.RTC_enable = 1;    
          wakeupIO.LPU_enable = 0;
          SetRTC_WakeupTimeout(WAKEUP_TIMEOUT);
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
          DisableRTC_WakeupTimeout();
        }
        break;
      case 'u':
        {
          /* POWER_SAVE_LEVEL_STOP_WITH_TIMER : wake on LPUART/button PUSH1 (PA10) */
#if defined CONFIG_DEVICE_BLUENRG_LPS
          Configure_LPUART();
          printf("Enable Power Save Request : STOP_WITH_TIMER (LPUART)\r\n");
          while(BSP_COM_UARTBusy());          
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = 0;
          wakeupIO.RTC_enable = 0;         
          wakeupIO.LPU_enable = 1;          
          LL_PWR_EnablePDB(LL_PWR_PUPD_IO0);
          LL_PWR_EnablePUA(LL_PWR_PUPD_IO1);
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_WITH_TIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
          Disable_LPUART();
          if (wakeupSources == WAKEUP_LPU)
            printf("Wakeup Character: %c\r\n", receivedChar);
#else
          printf("Feature not supported!!!\r\n");
#endif
        }
        break;
      case 'n':
        {
          /* POWER_SAVE_LEVEL_NOTIMER : wake on UART RX pin/button PUSH1 (PA10) */
          printf("Enable Power Save Request : STOP_NOTIMER\r\n");
          while(BSP_COM_UARTBusy());
          wakeupIO.IO_Mask_High_polarity = WAKEUP_PA10;
          wakeupIO.IO_Mask_Low_polarity = WAKEUP_UART_RX_PIN;
          wakeupIO.RTC_enable = 0;    
          wakeupIO.LPU_enable = 0;
          ret_val = HAL_PWR_MNGR_Request(POWER_SAVE_LEVEL_STOP_NOTIMER, wakeupIO, &stopLevel);
          if (ret_val != SUCCESS)
            printf("Error during clock config 0x%2x\r\n", ret_val);
          PrintNegotiatedLevel(stopLevel);
          if (stopLevel >= POWER_SAVE_LEVEL_STOP_WITH_TIMER) {
            wakeupSources = HAL_PWR_MNGR_WakeupSource();
            PrintWakeupSource(wakeupSources);
          }
        }
        break;
      case 'c':
        {
          /* POWER_SAVE_LEVEL_CPU_HALT : wake on button PUSH1 (PA10) */
          printf("Enable Power Save Request : CPU_HALT\r\n");
          while(BSP_COM_UARTBusy());          
          __WFI();
          printf("Exit from WFI cortex instruction with interrupt\r\n");
          /* Empty the USART buffer */
          while(BSP_COM_Read(&tmp));
        }
        break;
      case 'l':
        {
          /* Toggle led LED1 */
          BSP_LED_Toggle(BSP_LED1);
        }
        break;
      case 'p':
        {
          /* Print Hello World */
          HAL_GetPartInfo(&partInfo);
#if defined CONFIG_DEVICE_BLUENRG_LPS
          printf("Hello World: BlueNRG-LPS (%d.%d) is here!\r\n",
                 partInfo.die_major,
                 partInfo.die_cut);
#endif
#if defined CONFIG_DEVICE_BLUENRG_LP
          printf("Hello World: BlueNRG-LP (%d.%d) is here!\r\n",
                 partInfo.die_major,
                 partInfo.die_cut);
#endif
        }
        break;
      case 'r':
        {
          /* Reset BlueNRG-LP */
          NVIC_SystemReset();
        }
        break;
      case '?':
        {
          /* Help command */
          help();
        }
        break;
      default:
        printf("UNKNWON COMMAND! Press ? for command list\r\n");
      }
    }
  }  
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

#endif


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
