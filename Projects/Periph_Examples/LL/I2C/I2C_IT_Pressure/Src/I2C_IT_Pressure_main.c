
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : I2C_IT_Pressure_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the I2C IT functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  I2C_IT_Pressure/I2C_IT_Pressure_main.c
 * @brief How to handle the reception of one data byte from the pressure sensor on board
 *        to the device via I2C in interrupt mode.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\I2C\\I2C_IT_Pressure\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\I2C_IT_Pressure.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\I2C\\I2C_IT_Pressure\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\I2C_IT_Pressure.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\I2C\\I2C_IT_Pressure\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Master configuration


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
|     A1     |       I2C1 SDA      |      USART TX      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Not Used      |
|     B4     |       Not Used      |        DL2         |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      I2C1 SDA      |
|     B8     |         DL2         |        N.A.        |
|     B9     |       Not Used      |        N.A.        |
|     A0     |     N.A. (I2C1 SCL) |      I2C1 SCL      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      Not Used      |
|     B15    |         N.A.        |      Not Used      |
|     GND    |       Not Used      |      Not Used      |
|     RST    |       Not Used      |      Not Used      |
|    VBAT    |       Not Used      |      Not Used      |
@endtable 

* \section Serial_IO Serial I/O
@table
{SERIAL_IO_TABLE}
@endtable

* \section LEDs_description LEDs description
@table
|  LED name  |                                        STEVAL-IDB011V1                                       |                                        STEVAL-IDB011V2                                       |                                        STEVAL-IDB012V1                                       |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |
|     DL2    |  ON: communication is OK - Fast blinking: wait for user-button press - Slow blinking: error  |  ON: communication is OK - Fast blinking: wait for user-button press - Slow blinking: error  |  ON: communication is OK - Fast blinking: wait for user-button press - Slow blinking: error  |
|     DL3    |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |
|     DL4    |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |
|     U5     |                                           Not Used                                           |                                           Not Used                                           |                                           Not Used                                           |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |             STEVAL-IDB011V1            |             STEVAL-IDB011V2            |             STEVAL-IDB012V1            |
------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |  Start of the communication by Master  |  Start of the communication by Master  |  Start of the communication by Master  |
|      PUSH2     |                Not Used                |                Not Used                |                Not Used                |
|      RESET     |            Reset BlueNRG-LP            |            Reset BlueNRG-LP            |            Reset BlueNRG-LP            |

@endtable

* \section Usage Usage


How to handle the reception of one data byte from an I2C slave pressure sensor. 
The device operate in interrupt mode. 
The peripheral is initialized with LL unitary service functions to optimize for performance and size.
The I2Cx Peripheral is configured in Master mode with EXTI (Fast Mode @400kHz).

Example execution:
LED2 blinks quickly to wait for user-button press.
Press the User push-button (PUSH1) to initiate a write request.
This action will generate an I2C start condition with the pressure sensor address and a write bit condition.
I2C2 IRQ Handler routine is then checking when TC interrupt occurs. Now is initiate a read request.
When byte is received on I2Cx of the device, an RXNE interrupt occurs.
When RXDR register is read.

LED2 is On :
if the sequence is completed and if data is well received.

If an error occurs, LED2 is blinking slowly.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect board USART1 TX pin to PC COM port RX signal
- Connect board USART1 RX pin to PC COM port TX signal
- Connect board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 115200 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control 

**/
   

/* Includes ------------------------------------------------------------------*/
#include "I2C_IT_Pressure_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/**
* @brief I2C devices settings
*/

/* Private macro -------------------------------------------------------------*/
#define READ_CR2(__FIELDS__)  READ_BIT(I2Cx->CR2, __FIELDS__) 
#define CHECK_CR2(__FIELDS__)  ((READ_CR2(__FIELDS__) == __FIELDS__) ? 1UL : 0UL)

/* Private variables ---------------------------------------------------------*/


uint8_t aReceiveBuffer[0xF] = {0};
__IO uint8_t ubReceiveIndex      = 0;
__IO uint8_t ubButtonPress       = 0;
/* Configure the SDA setup, hold time and the SCL high, low period 
* For Fast-mode     kHz, PRESC | 0h | SCLDEL | SDADEL | SCLH | SCLL 
*                          1h  | 0h |    3h  |   2h   |  03h |  09h 
* timing = __LL_I2C_CONVERT_TIMINGS(0x01, 0x03, 0x02, 0x03, 0x09);
*/ 

/* I2C TIMING in Fast Mode */
__IO uint32_t timing = __LL_I2C_CONVERT_TIMINGS(0x01, 0x03, 0x02, 0x03, 0x09);

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2Cx_Init(void);
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
void LED_On(void);
void LED_Off(void);
void LED_Blinking(uint32_t Period);
void WaitForUserButtonPress(void);
void Handle_I2C_Master(void);

/* Private user code ---------------------------------------------------------*/

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
  
  /* Initialization of COM port */
  BSP_COM_Init(Process_InputData);
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2Cx_Init();
  
  /* Set LED2 Off */
  LED_Off();
  
  /* Wait for User push-button (PUSH1) press to start transfer */
  WaitForUserButtonPress();
  
  /* Handle I2Cx events (Master) */
  Handle_I2C_Master();
  
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
* @brief I2Cx Initialization Function
* @param None
* @retval None
*/
static void MX_I2Cx_Init(void)
{  
  LL_I2C_InitTypeDef I2C_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Enable the peripheral clock of GPIO */
  LL_I2Cx_SCL_EnableClock();
  LL_I2Cx_SDA_EnableClock();
  
  GPIO_InitStruct.Pin = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = I2Cx_SCL_AF;
  LL_GPIO_Init(I2Cx_SCL_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = I2Cx_SDA_AF;
  LL_GPIO_Init(I2Cx_SDA_PORT, &GPIO_InitStruct);
  
  /* Peripheral clock enable */
  LL_I2Cx_EnableClock();
  
  /* Configure Event IT:
  *  - Set priority for I2Cx_IRQn
  *  - Enable I2Cx_IRQn
  */
  NVIC_SetPriority(I2Cx_IRQn, IRQ_HIGH_PRIORITY);  
  NVIC_EnableIRQ(I2Cx_IRQn);
  
  /* I2C Initialization */
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.Timing = timing;
  I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  I2C_InitStruct.DigitalFilter = 2;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2Cx, &I2C_InitStruct);
  LL_I2C_EnableAutoEndMode(I2Cx);
  LL_I2C_SetOwnAddress2(I2Cx, 0, LL_I2C_OWNADDRESS2_NOMASK);
  LL_I2C_DisableOwnAddress2(I2Cx);
  LL_I2C_DisableGeneralCall(I2Cx);
  LL_I2C_EnableClockStretching(I2Cx);
  LL_I2C_EnableIT_RX(I2Cx);
  LL_I2C_EnableIT_TX(I2Cx);
  LL_I2C_EnableIT_NACK(I2Cx);
  LL_I2C_EnableIT_ERR(I2Cx);
  LL_I2C_EnableIT_STOP(I2Cx);
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
  
  /* Turn Off Led2 */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  
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
  /* Turn LED2 on */
  LL_GPIO_ResetOutputPin(LED2_GPIO_PORT, LED2_PIN);
  
  /* Toggle IO in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN); 
    LL_mDelay(Period);
  }
}
/******************************************************************************/
/*   IRQ HANDLER TREATMENT Functions                                          */
/******************************************************************************/

/**
* @brief  Wait for User push-button (PUSH1) press to start transfer.
* @param  None 
* @retval None
*/
void WaitForUserButtonPress(void)
{
  printf("Wait for User push-button (PUSH1) press or enter 'c'/'C' character.\n\r");
  while (ubButtonPress == 0)
  {
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
    LL_mDelay(LED_BLINK_FAST);
  }
  /* Turn LED2 off */
  LED_Off();
}

/**
* @brief  Function to manage User push-button (PUSH1)
* @param  None
* @retval None
*/
void UserButton_Callback(void)
{
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main program */
  ubButtonPress = 1;
}

/**
* @brief  This Function handle Master events to perform a reception process
* @note  This function is composed in one step :
*        -1- Initiate a Start condition to the Slave device.
* @param  None
* @retval None
*/
void Handle_I2C_Master(void)
{
  /* Initiate a Start condition to the Slave device ***********************/
  
  /* Master Generate Start condition for a read request:
  *  - to the Slave with a 7-Bit SLAVE_OWN_ADDRESS
  *  - with a auto stop condition generation when receive 1 byte
  */
  LL_I2C_HandleTransfer(I2Cx,0xB9U , LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  
}

/**
* @brief  Function called from I2C IRQ Handler when RXNE flag is set
*         Function is in charge of reading byte received on I2C lines.
* @param  None
* @retval None
*/
void Master_Reception_Callback(void)
{
  /* Read character in Receive Data register.
  RXNE flag is cleared by reading data in RXDR register */
  aReceiveBuffer[ubReceiveIndex] = LL_I2C_ReceiveData8(I2Cx);
  printf("Master_Reception_Callback : 0x%X \r\n",aReceiveBuffer[ubReceiveIndex]);
  ubReceiveIndex++;
}

/**
* @brief  Function called from I2C IRQ Handler when STOP flag is set
*         Function is in charge of checking data received,
*         LED2 is On if data are correct.
* @param  None
* @retval None
*/
void Master_WriteTXE_Callback(void)
{
  printf("Master_WriteTXE_Callback\n\r");
  /* Get the received byte from the RX FIFO */
  LL_I2C_TransmitData8(BSP_I2C, (0x0F | 0x80)); // 0w0F WHO_AM_I |0x80 auto-increment
}

/**
* @brief  Function called from I2C IRQ Handler when STOP flag is set
*         Function is in charge of checking data received
* @param  None
* @retval None
*/
void Master_Complete_Callback(void)
{
  
  if( READ_CR2(I2C_CR2_RD_WRN) == 0 )
  {
    //  0w0F WHO_AM_I |0x80 auto-increment transfer complete
    
    /* Initialize the handle transfer */
    LL_I2C_HandleTransfer(BSP_I2C, 0xB9U, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
  }
  else
  {
    /* Read Received character.
    RXNE flag is cleared by reading of RXDR register */
    printf("Master_Complete_Callback\n\r");
    if(aReceiveBuffer[ubReceiveIndex-1] == 0xB1)  // who I am = 0xB1
    {
      /* Turn LED2 On:
      *  - Expected byte has been received
      *  - Master Rx sequence completed successfully
      */
      LED_On();
      printf("Master Rx sequence completed successfully\n\r");
      printf("** Test successfully. ** \n\r\n\r");
    }
    else
    {
      /* Call Error function */
      Error_Callback();
    }
  }
}

/**
* @brief  Function called in case of error detected in I2C IT Handler
* @param  None
* @retval None
*/
void Error_Callback(void)
{
  /* Disable I2Cx_IRQn */
  NVIC_DisableIRQ(I2Cx_IRQn);
  
  Error_Handler();
}

void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
  if(Nb_bytes>0)
  {
    if(data_buffer[0] == 'c' || data_buffer[0] == 'C' )
    {
      UserButton_Callback();
    }
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


