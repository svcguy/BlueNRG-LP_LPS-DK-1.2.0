
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : LPUART_TxRx_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the LPUART functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  LPUART_TxRx/LPUART_TxRx_main.c
 * @brief This example describes how to configure LPUART peripheral in Asynchronous mode
 * for being able to receive on RX line using the BLUENRG_LP LPUART LL API.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\LPUART\\LPUART_TxRx\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\LPUART_TxRx.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\LPUART\\LPUART_TxRx\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\LPUART_TxRx.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\LPUART\\LPUART_TxRx\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB011V1
- \c STEVAL-IDB011V2
- \c STEVAL-IDB012V1



* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB01xV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name | Description                                                                |
| JP2         |                                                                            |
----------------------------------------------------------------------------------------------------------
| USB         | USB supply power                                                            |
| BAT         | The supply voltage must be provided through battery pins.                   |


@endtable

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB01xV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | It provides the voltage to the BlueNRG-LP circuit. It must be fitted. It can be used for current measurements of the BlueNRG-LP device.                                |          
| JP2         | It is a switch between two power domains. BAT position: to provide power from battery holder; USB position: to provide power from USB connector.                       |
| JP3         | It connects the BLE_SWCLK pin of the BlueNRG-LP with the SWCLK pin of the USB_CMSISDAP. It must be fitted.                                                             |          
| JP4         | It connects the BLE_SWDIO pin of the BlueNRG-LP with the SWDIO pin of the USB_CMSISDAP. It must be fitted.                                                             |
| JP5         | It connects the BLE_RSTN pin of the BlueNRG-LP with the rest of the board (the USB_CMSISDAP and RESET push button). It must be fitted.                                 |


@endtable 

* \section Pin_settings Pin settings
@table
|  PIN name  | STEVAL-IDB011V{1|2} |   STEVAL-IDB012V1  |
--------------------------------------------------------
|     A1     |       Not Used      |      Not Used      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       Not Used      |      Not Used      |
|     A9     |       Not Used      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |        DL3         |
|     B3     |       Not Used      |      Not Used      |
|     B4     |      LPUART1 TX     |     LPUART1 TX     |
|     B5     |      LPUART1 RX     |     LPUART1 RX     |
|     B7     |       Not Used      |      Not Used      |
|     B8     |         DL2         |        N.A.        |
|     B9     |       Not Used      |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      Not Used      |
|     B15    |         N.A.        |      Not Used      |
|     GND    |       Not Used      |      Not Used      |
|     RST    |       Not Used      |      Not Used      |
|    VBAT    |       Not Used      |      Not Used      |
@endtable 

* \section Serial_IO Serial I/O
  The application will listen for keys typed and it will send back in the serial port.
  In other words everything typed in serial port will be send back.
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 9600             | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Start bits      | 1                | bit       |
| Stop bits       | 1                | bit       |
| HW flow control | None             | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |                                           STEVAL-IDB011V1                                          |                                           STEVAL-IDB011V2                                          |                                           STEVAL-IDB012V1                                          |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                              Not Used                                              |                                              Not Used                                              |                                              Not Used                                              |
|     DL2    |  ON: specific value is received (s or S) - OFF: specific value is received - Slow blinking: error  |  ON: specific value is received (s or S) - OFF: specific value is received - Slow blinking: error  |                                              Not Used                                              |
|     DL3    |                                              Not Used                                              |                                              Not Used                                              |  ON: specific value is received (s or S) - OFF: specific value is received - Slow blinking: error  |
|     DL4    |                                              Not Used                                              |                                              Not Used                                              |                                              Not Used                                              |
|     U5     |                                              Not Used                                              |                                              Not Used                                              |                                              Not Used                                              |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

This example shows how to configure GPIO and LPUART peripherals to allow characters received on LPUART RX pin. 
This example is based on the LPUART LL API. 
The peripheral initialization is done using LL unitary services functions for optimization purpose (performance and size).


Example execution:
On first  character reception by the LPUART from PC Com port (ex: using HyperTerminal)
received character value is checked :
- On a specific value 'S' or 's', LED2 is turned On.
- If different from 'S' or 's', LED2 is turned Off.

If OVR error is raised check that the serial communication SW on PC (as HyperTerminal or TeraTerm) chosen must send the char during the writing on the terminal.

In case of errors, LED2 is slowly blinking (1 sec period).

BlueNRG_LP-EVB Set-up
 - Connect BlueNRG_LP board LPUART1 TX pin PB.04 to PC COM port RX signal (FTDI pin 5 Yellow RX)
 - Connect BlueNRG_LP board LPUART1 RX pin PB.05 to PC COM port TX signal (FTDI pin 4 Orange TX)
 - Connect BlueNRG_LP board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 9600 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control
**/
   

/* Includes ------------------------------------------------------------------*/
#include "LPUART_TxRx_main.h"

/** @addtogroup RF_DRIVER_LL_Examples
* @{
*/

/** @addtogroup LPUART_TxRx
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/**
* @brief Variables used for charcater reception from PC Com port
*/
__IO uint8_t      newCharReceived = 0;
__IO uint32_t     ubReceivedChar;

/**
* @brief Text string printed on PC Com port to inform MCU will enter in Stop 0 Mode
*/
uint8_t aTextInfo[] = "\r\nLPUART Example.\n\rEnter any character.\r\nOn a specific value ('S' or 's'), LED2 is turned On and program ends.\r\nIf different from 'S' or 's', LED2 is turned Off.\r\n";

/* Private function prototypes -----------------------------------------------*/
void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);
void Configure_LPUART1(void);
void PrintInfo(void);

/* Private functions ---------------------------------------------------------*/

/**
* @brief  Main program
* @param  None
* @retval None
*/
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_32M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
  /* Initialize LED2 */
  LED_Init();
  
  /* LED off */
  LED_Off();
  
  /* Configure LPUART1 (LPUART IP configuration and related GPIO initialization) */
  Configure_LPUART1();
  
  /* Start main program loop :
  - Wait for any character received on LPUART RX line
  */
  
  /* Send Text Information on LPUART TX to PC Com port */
  PrintInfo();
  
  /* Infinite loop */
  while (1)
  {
    if(newCharReceived == 1)
    {
      /* Check if received value is corresponding to specific one : S or s */
      if((ubReceivedChar == 'S') || (ubReceivedChar == 's'))
      {
        /* Turn LED2 on : Expected character has been received */
        LED_On();
      }
      else
      {
        /* Turn LED2 off */
        LED_Off();
      }
      
      /* Echo received character on TX */
      LL_LPUART_TransmitData8(LPUART1, ubReceivedChar);
  
      /* Wait for TC flag to be raised for last char */
      while (!LL_LPUART_IsActiveFlag_TC(LPUART1))
      {
      }
      
      newCharReceived = 0;
    }
  }
}

/**
* @brief  This function configures LPUART1.
* @note   This function is used to :
*         -1- Enable GPIO clock and configures the LPUART1 pins.
*         -2- NVIC Configuration for LPUART1 interrupts.
*         -3- Enable the LPUART1 peripheral clock and clock source.
*         -4- Configure LPUART1 functional parameters.
*         -5- Enable LPUART1.
* @note   Peripheral configuration is minimal configuration from reset values.
*         Thus, some useless LL unitary functions calls below are provided as
*         commented examples - setting is default configuration from reset.
* @param  None
* @retval None
*/
void Configure_LPUART1(void)
{
  /* (1) Enable GPIO clock and configures the LPUART1 pins *******************/
  
  /* Enable the peripheral clock of GPIOB */
  LL_EnableClock_LPUART();
  LL_EnableClock_LPUART_TX_PORT();
  LL_EnableClock_LPUART_RX_PORT();
  
  /* Configure TX Pin as : Alternate function, High Speed, PushPull, No-Pull */
  LL_GPIO_SetPinMode(LPUART_TX_PORT, LPUART_TX_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_LPUART_TX();
  LL_GPIO_SetPinSpeed(LPUART_TX_PORT, LPUART_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(LPUART_TX_PORT, LPUART_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(LPUART_TX_PORT, LPUART_TX_PIN, LL_GPIO_PULL_NO);
  
  /* Configure RX Pin as : Alternate function, High Speed, PushPull, No-Pull */
  LL_GPIO_SetPinMode(LPUART_RX_PORT, LPUART_RX_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_LPUART_RX();
  LL_GPIO_SetPinSpeed(LPUART_RX_PORT, LPUART_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(LPUART_RX_PORT, LPUART_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(LPUART_RX_PORT, LPUART_RX_PIN, LL_GPIO_PULL_NO);
  
  /* (2) NVIC Configuration for LPUART1 interrupts */
  /*  - Set priority for LPUART1_IRQn */
  /*  - Enable LPUART1_IRQn           */
  NVIC_SetPriority(LPUART1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(LPUART1_IRQn);
  
  /* (4) Configure LPUART1 functional parameters ********************************/
  
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* Set LPUART Clock source */
  if(LL_RCC_LSE_IsEnabled())
  {
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUCLKSEL_LSE);
  }
  else
  {
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUCLKSEL_16M);
  }
#endif
  
  /* TX/RX direction */
  LL_LPUART_SetTransferDirection(LPUART1, LL_LPUART_DIRECTION_TX_RX);
  
  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_LPUART_ConfigCharacter(LPUART1, LL_LPUART_DATAWIDTH_8B, LL_LPUART_PARITY_NONE, LL_LPUART_STOPBITS_1);
  
  /* No Hardware Flow control */
  /* Reset value is LL_USART_HWCONTROL_NONE */
  // LL_LPUART_SetHWFlowCtrl(LPUART1, LL_USART_HWCONTROL_NONE);
  
  /* Set Baudrate to 9600 using HSI frequency set to HSI_VALUE */
  LL_LPUART_SetBaudRate(LPUART1, LL_LPUART_PRESCALER_DIV1, 9600); 
  
  LL_LPUART_EnableIT_RXNE_RXFNE(LPUART1);  
  
  /* (5) Enable LPUART1 **********************************************************/
  LL_LPUART_Enable(LPUART1);
}

/**
* @brief  Send Txt information message on LPUART Tx line (to PC Com port).
* @param  None
* @retval None
*/
void PrintInfo(void)
{
  uint32_t index = 0;
  
  /* Send characters one per one, until last char to be sent */
  for (index = 0; index < sizeof(aTextInfo); index++)
  {
    /* Wait for TXE flag to be raised */
    while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
    {
    }
    /* Write character in Transmit Data register.
    TXE flag is cleared by writing data in TDR register */
    LL_LPUART_TransmitData8(LPUART1, aTextInfo[index]);
  }
  
  /* Wait for TC flag to be raised for last char */
  while (!LL_LPUART_IsActiveFlag_TC(LPUART1))
  {
  }
}

/**
* @brief  Initialize LED2.
* @param  None
* @retval None
*/
void LED_Init(void)
{
  /* Enable the LED2 Clock */
  LEDx_GPIO_CLK_ENABLE();
  
  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(LEDx_GPIO_PORT, LEDx_PIN, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(LEDx_GPIO_PORT, LEDx_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(LEDx_GPIO_PORT, LEDx_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(LEDx_GPIO_PORT, LEDx_PIN, LL_GPIO_PULL_NO);
  
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LEDx_GPIO_PORT, LEDx_PIN);
}

/**
* @brief  Turn-on LED2.
* @param  None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LEDx_GPIO_PORT, LEDx_PIN);
}

/**
* @brief  Turn-off LED2.
* @param  None
* @retval None
*/
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LEDx_GPIO_PORT, LEDx_PIN);
}

/**
* @brief  Set LED2 to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LEDx_GPIO_PORT, LEDx_PIN);  
    LL_mDelay(Period);
  }
}

/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/

/**
* @brief  Function called from LPUART IRQ Handler when RXNE flag is set
*         Function is in charge of reading character received on LPUART RX line.
* @param  None
* @retval None
*/
void LPUART_CharReception_Callback(void)
{
  /* Read Received character. RXNE flag is cleared by reading of RDR register */
  ubReceivedChar = LL_LPUART_ReceiveData8(LPUART1);
  
  newCharReceived = 1;
}

/**
* @brief  Function called in case of error detected in LPUART IT Handler
* @param  None
* @retval None
*/
void Error_Callback(void)
{
  /* Disable LPUART1_IRQn */
  NVIC_DisableIRQ(LPUART1_IRQn);
  
  /* Unexpected event : Set LED to Blinking mode to indicate error occurs */
  LED_Blinking(LED_BLINK_ERROR);
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
  ex: printf("Wrong parameters value: file %s on line %d", file, line) */
  
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



