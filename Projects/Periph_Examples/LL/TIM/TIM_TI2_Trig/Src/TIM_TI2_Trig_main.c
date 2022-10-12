
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_TI2_Trig_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the TIM functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  TIM_TI2_Trig/TIM_TI2_Trig_main.c
 * @brief Configuration of a timer to generate a positive pulse in 
 * Output Compare mode with a length of tPULSE and after a delay of tDELAY. 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_TI2_Trig\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_TI2_Trig.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_TI2_Trig\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_TI2_Trig.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_TI2_Trig\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  PIN name  | STEVAL-IDB011V{1|2} |
------------------------------------
|     A1     |       Not Used      |
|     A11    |       Not Used      |
|     A12    |       Not Used      |    
|     A13    |       Not Used      |  
|     A14    |       Not Used      |
|     A15    |       Not Used      |
|     A4     |       TIM1 CH1      |
|     A5     |       TIM1 CH2      |
|     A6     |       Not Used      |
|     A7     |       Not Used      |
|     A8     |       USART TX      |
|     A9     |       USART RX      |
|     B0     |       Not Used      |    
|     B14    |       Not Used      |
|     B2     |       Not Used      |
|     B3     |       Not Used      |
|     B4     |       Not Used      |  
|     B5     |       Not Used      |
|     B7     |       Not Used      |
|     B8     |         DL2         |
|     B9     |       Not Used      |
|     A0     |         N.A.        |
|     A10    |         N.A.        |
|     B1     |         N.A.        |
|     B6     |         N.A.        |
|     B15    |         N.A.        |
|     GND    |       Not Used      |
|     RST    |       Not Used      |
|    VBAT    |       Not Used      |
@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable


* \section LEDs_description LEDs description
@table
|  LED name  |                       STEVAL-IDB011V1                      |                       STEVAL-IDB011V2                      |
-----------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                          Not Used                          |                          Not Used                          |
|     DL2    |  On: signal generation on TIM1 CH2 - Slow blinking: error  |  On: signal generation on TIM1 CH2 - Slow blinking: error  |
|     DL3    |                          Not Used                          |                          Not Used                          |
|     DL4    |                          Not Used                          |                          Not Used                          |
|     U5     |                          Not Used                          |                          Not Used                          |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |    STEVAL-IDB011V1   |    STEVAL-IDB011V2   |
-----------------------------------------------------------------
|      PUSH1     |  Start TIMx counter  |  Start TIMx counter  |
|      PUSH2     |       Not Used       |       Not Used       |
|      RESET     |   Reset BlueNRG-LP   |   Reset BlueNRG-LP   |

@endtable

* \section Usage Usage
Configuration of a timer to generate a positive pulse in Output Compare mode with a length of tPULSE and after a delay of tDELAY. 
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes.

TIM1 generates a positive pulse of 3 s after a delay of 2 s. 
TIM1 counter start is controlled through the slave mode controller. 
TI2 signals is selected as trigger input meaning that TIM1 counter starts when a rising edge is detected on TI2.
  
BlueNRG_LP-EVB Set-up
Connecting LED2 pin to TIM1 TI2 (TIM1_CH2) allows to trigger TIM1 counter by pressing the User push-button.
Connect the following pins to an oscilloscope to monitor the different waveforms:
- LED2 :     PB.8 
- TIM1_CH1 : PA.4
- TIM1_CH2 : PA.5
Connect USART1 TX/RX to respectively RX and TX pins of PC UART (could be done through a USB to UART adapter) :
- Connect BlueNRG_LP board USART1 TX pin to PC COM port RX signal
- Connect BlueNRG_LP board USART1 RX pin to PC COM port TX signal
- Connect BlueNRG_LP board GND to PC COM port GND signal

Launch serial communication SW on PC (as HyperTerminal or TeraTerm) with proper configuration :
- 115200 bauds
- 8 bits data
- 1 start bit
- 1 stop bit
- no parity
- no HW flow control 
**/
   
/* Includes ------------------------------------------------------------------*/
#include "TIM_TI2_Trig_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void LL_Init(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
static void MX_GPIO_Init(void);
void WaitForUserButtonPress(void);

/* Private user code ---------------------------------------------------------*/
__IO uint8_t ubButtonPress = 0;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, RADIO_SYSCLK_NONE) != SUCCESS)
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

  /* LED off */
  LED_Off();

  LL_GPIO_SetPinMode(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(TIMx_CH1_PORT, TIMx_CH1_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_TIMx_CH1();
  
  LL_GPIO_SetPinMode(TIMx_CH2_PORT, TIMx_CH2_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TIMx_CH2_PORT, TIMx_CH2_PIN, LL_GPIO_PULL_DOWN);
  LL_GPIO_SetPinSpeed(TIMx_CH2_PORT, TIMx_CH2_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_TIMx_CH2();
   
  /* Peripheral clock enable */
  LL_EnableClock_TIMx();
  
  /*******************************/
  /* Input trigger configuration */
  /*******************************/
  /* Configure CH2 to detect rising edges on the TI2 input CC2S=01 (TIMx_CCMR1) */
  LL_TIM_IC_SetActiveInput(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  
  /* Set rising edge polarity CC2P=0,CC2NP=0 (TIMx_CCER) */
  LL_TIM_IC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING );
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* Configure the timer in external clock mode 1 SMS=111 (TIMx_SMCR) */
  LL_TIM_SetSlaveMode(TIMx, LL_TIM_SLAVEMODE_COMBINED_RESETTRIGGER);
#endif
  /* Set TI2 as the trigger input source TS=110 (TIMx_SMCR) */
  LL_TIM_SetTriggerInput(TIMx, LL_TIM_TS_TI2FP2);
  
  /* Enable counter. Note that the counter will stop automatically at the     */
  /* next update event (UEV).                                                 */
  LL_TIM_EnableCounter(TIMx);
  
  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Select counter mode: counting up */
  LL_TIM_SetCounterMode(TIMx, LL_TIM_COUNTERMODE_UP);
  
  /* Set the one pulse mode: generate only 1 pulse */
  LL_TIM_SetOnePulseMode(TIMx, LL_TIM_ONEPULSEMODE_SINGLE);
  
  /* Set the TIM2 prescaler to get counter clock frequency at 2 kHz */ 
  LL_TIM_SetPrescaler(TIMx, __LL_TIM_CALC_PSC(LL_TIM_GetPeriphClock(TIMx), 2000));
  
  /* Set the capture/compare register to get a pulse delay of 2s (2000000 us)*/
  LL_TIM_OC_SetCompareCH1(TIMx, __LL_TIM_CALC_DELAY(LL_TIM_GetPeriphClock(TIMx), LL_TIM_GetPrescaler(TIMx), 2000000));
  
  /* Set the autoreload register to get a pulse length of 3s (3000000 us)*/
  LL_TIM_SetAutoReload(TIMx, __LL_TIM_CALC_PULSE(LL_TIM_GetPeriphClock(TIMx), LL_TIM_GetPrescaler(TIMx), 2000000, 3000000));
  
  /* Set output channel 1 in PWM2 mode */
  LL_TIM_OC_SetMode(TIMx,  LL_TIM_CHANNEL_CH1,  LL_TIM_OCMODE_PWM2);
  
  /* Configure output channel 1 configuration */
  LL_TIM_OC_ConfigOutput(TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH | LL_TIM_OCIDLESTATE_LOW);
  
  /**************************/
  /* Start pulse generation */
  /**************************/
  /* Enable channel 1 */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH1);
  
  /* Enable auto-reload register preload */
  LL_TIM_EnableARRPreload(TIMx);
  
  /* Enable TIMx outputs */
  LL_TIM_EnableAllOutputs(TIMx);
  
  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIMx);

  printf("Connecting LED pin to TIM TI2 (TIMx_CH2) allows to trigger TIMx counter by pressing the User push-button.");
  printf("TIMx generates a positive pulse of 3 s after a delay of 2 s. \n\r");
  
  /* Infinite loop */
  while (1)
  {
      /* Wait for User push-button (PUSH1) press to start transfer */
      WaitForUserButtonPress();
      /* LED on */
      LED_On();
      LL_mDelay(50);
      /* LED off */
      LED_Off();
  }
}

static void LL_Init(void)
{
  /* System interrupt init */
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

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
    ;
  }
  ubButtonPress = 0;
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
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOA);
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  LL_APB0_EnableClock(LL_APB0_PERIPH_SYSCFG);
  
  /* Configure GPIO for LED */
  LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_PIN);
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
   
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


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
/**
  * @brief  User button interrupt processing
  * @note   TIMx counter is enabled every time the user button is presssed. 
  * @param  None
  * @retval None
  */
void UserButton_Callback(void)
{
  /* Update User push-button (PUSH1) variable : to be checked in waiting loop in main function */
  ubButtonPress = 1;
}

/**
* @brief  Turn-on LED.
* @param  None
* @retval None
*/
void LED_On(void)
{
  /* Turn LED on */
  LL_GPIO_ResetOutputPin(LED_GPIO_PORT, LED_PIN);
}

/**
* @brief  Turn-off LED.
* @param  None
* @retval None
*/
void LED_Off(void)
{
  /* Turn LED off */
  LL_GPIO_SetOutputPin(LED_GPIO_PORT, LED_PIN);
}

/**
* @brief  Set LED to Blinking mode for an infinite loop (toggle period based on value provided as input parameter).
* @param  Period : Period of time (in ms) between each toggling of LED
*   This parameter can be user defined values. Pre-defined values used in that example are :
*     @arg LED_BLINK_FAST : Fast Blinking
*     @arg LED_BLINK_SLOW : Slow Blinking
*     @arg LED_BLINK_ERROR : Error specific Blinking
* @retval None
*/
void LED_Blinking(uint32_t Period)
{
  /* Toggle LED in an infinite loop */
  while (1)
  {
    LL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    LL_mDelay(Period);
  }
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/


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



