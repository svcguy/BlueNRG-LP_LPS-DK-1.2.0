
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : USART_Tx_IT_Init_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the USART functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  USART_Tx_IT_Init/USART_Tx_IT_Init_main.c
 * @brief This example shows how to configure GPIO and USART peripheral to send characters
 * asynchronously to HyperTerminal (PC) in Interrupt mode.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_Tx_IT_Init\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_Tx_IT_Init.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_Tx_IT_Init\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\USART_Tx_IT_Init.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\USART\\USART_Tx_IT_Init\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c USART_Tx_IT_Init - Release configuration


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
|     A1     |        SPI CS       |       SPI CS       |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       SPI MOSI      |      Not Used      |
|     A9     |       SPI MISO      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Not Used      |
|     B4     |       Not Used      |        DL2         |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       SPI CLK       |        N.A.        |
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



* \section LEDs_description LEDs description
@table
|  LED name  |                     STEVAL-IDB011V1                    |                     STEVAL-IDB011V2                    |                     STEVAL-IDB012V1                    |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                        Not Used                        |                        Not Used                        |                        Not Used                        |
|     DL2    |                        Not Used                        |                        Not Used                        |   ON: last byte is transmitted - Slow blinking: error  |
|     DL3    |   ON: last byte is transmitted - Slow blinking: error  |   ON: last byte is transmitted - Slow blinking: error  |                        Not Used                        |
|     DL4    |                        Not Used                        |                        Not Used                        |                        Not Used                        |
|     U5     |                        Not Used                        |                        Not Used                        |                        Not Used                        |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |             STEVAL-IDB011V1            |             STEVAL-IDB011V2            |             STEVAL-IDB012V1            |
------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |   new transmission of complete buffer  |   new transmission of complete buffer  |   new transmission of complete buffer  |
|      PUSH2     |                Not Used                |                Not Used                |                Not Used                |
|      RESET     |            Reset BlueNRG-LP            |            Reset BlueNRG-LP            |            Reset BlueNRG-LP            |

@endtable

* \section Usage Usage

This example shows how to configure GPIO and USART peripheral to send characters synchronously to the LL SPI SPI_IT_Slave_Init in Interrupt mode.
This example is based on BLUENRG_LP USART LL API. 

In case of errors, LED3 is blinking (1sec period).

To test this SPI Master application was been used the LL/SPI/SPI_IT_Slave_Init Example with the SPI configured  as SPI1 and the data size set to 8bit.
On press on push button , USART TX Empty interrupt is enabled.
It is mandatory add to the slave application the follows define in the main.h file:
#define CONFIG_DATASIZE_8BIT  1
#define USE_SPI1_PINS         1


Connect Masrer and Slave :
- SPI CLK  (USART CK) with SPI Slave SCK
- SPI MISO (USART RX) with SPI Slave MISO
- SPI MOSI (USART TX) with SPI Slave MOSI
- SPI CS (Output pin) with SPI Slave CS

| MASTER STEVAL-IDB011V1 | SLAVE STEVAL-IDB011V1 |
|   USART Pin/Signal     |     SPI Pin/Signal    | 
|        PB8 / CK        |       PA13 / SCK      |
|        PA8 / RX        |       PA14 / MISO     |
|        PA9 / TX        |       PB14 / MOSI     |
|        PA1 / CS        |       PA11 / CS       |

  When the SPI slave shows on COM the waiting message for receiving data, press the PUSH1 button of the master board. The master application doesn't show meaning messsage on COM. LED3 is turned on in case of the transmission is completed. The master COM is used to virtualize the SPI. The SPI Slave show a success message in case of well received message.
  The aRxBuffer contains the received string also for Master and Slave application.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button


**/
   
/* Includes ------------------------------------------------------------------*/
#include "USART_Tx_IT_Init_main.h"

/* Private includes */

/* Private typedef */

/* Private define */

/* Private macro */

/* Private variables */
__IO uint8_t ubButtonPress = 0;
__IO uint8_t ubSend = 0;
const uint8_t aStringToSend[] = "USART LL API Example : TX in IT mode\r\nConfiguration UART\r\n\r\n** Test successfully. ** \n\r\n\r";
uint8_t ubSizeToSend = sizeof(aStringToSend);

/* Private function prototypes */
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_USART_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);

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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART_Init();

  /* Set LED2 Off */
  LED_Off();
  
  /* Start transfer only if not already ongoing */
  if (ubSend == 0)
  {
    /* Start USART transmission : Will initiate TXE interrupt after TDR register is empty */
    LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]); 

    /* Enable TXE interrupt */
    LL_USART_EnableIT_TXE(USART1); 
  }
  
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_EnableClock_USART();
  
  GPIO_InitStruct.Pin = USART1_TX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = USART1_TX_AF;
  LL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = USART1_RX_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = USART1_RX_AF;
  LL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USART1_IRQn);

  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  
    /* Polling USART initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  { 
  }

  /* Enable Error interrupts */
  LL_USART_EnableIT_ERROR(USART1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

  LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG);

  /* GPIO Ports Clock Enable */
  LED2_GPIO_CLK_ENABLE();
  USER_BUTTON_GPIO_CLK_ENABLE();
  USER_BUTTON_SYSCFG_CLK_ENABLE();
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
  
  /* Configure GPIO for BUTTON */
  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_PULL_NO);
  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, LL_GPIO_MODE_INPUT);
 
  /* Enable a rising trigger External line 10 Interrupt */
  EXTI_InitStruct.Line = LL_EXTI_LINE_PA10;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Type = LL_EXTI_TYPE_EDGE;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_EDGE;
  LL_EXTI_Init(&EXTI_InitStruct);

  /* Configure NVIC for USER_BUTTON_EXTI_IRQn */
  NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);

  /* Configure NVIC for SysTick_IRQn */
  NVIC_SetPriority(SysTick_IRQn, IRQ_LOW_PRIORITY);
}

/**
  * @brief  Turn-on LED2.
  * @param  None
  * @retval None
  */
void LED_On(void)
{
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
  * @brief  Turn-off LED2.
  * @param  None
  * @retval None
  */
void LED_Off(void)
{
  /* Turn LED2 off */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
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
  /* Toggle LED2 in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);  
    LL_mDelay(Period);
  }
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
    
    /* Turn LED2 on/off at end of transfer : Tx sequence completed successfully */
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN); 
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
    LED_Blinking(LED_BLINK_FAST);
  }
  else
  {
    /* Unexpected IT source : Set LED to Blinking mode to indicate error occurs */
    LED_Blinking(LED_BLINK_ERROR);
  }
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


