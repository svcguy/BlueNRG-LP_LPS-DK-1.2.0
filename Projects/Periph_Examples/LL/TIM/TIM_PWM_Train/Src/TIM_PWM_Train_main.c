
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_PWM_Train_main.c
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
 * @file TIM_PWM_Train/TIM_PWM_Train_main.c
 * @brief Configuration of a timer to generate a fixed number of pulses in Output Compare mode 
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_PWM_Train\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_PWM_Train.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_PWM_Train\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_PWM_Train.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\TIM\\TIM_PWM_Train\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A1     |       TIM1 CH4      |      Not Used      |
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
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      TIM2 CH4      |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      Not Used      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Not Used      |        N.A.        |
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
|  LED name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------
|     DL1    |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Not Used      |      Not Used      |      Not Used      |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |
|     U5     |      Not Used      |      Not Used      |      Not Used      |

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

Configuration of a timer to generate a fixed number of pulses in Output Compare mode. 
This example is based on the BLUENRG_LP TIM LL API. 
The peripheral initialization uses LL unitary service functions for optimization purposes (performance and size).

TIM1CLK is fixed to 64 MHz, the TIM1 Prescaler is set to have TIM1 counter clock = 10 Hz.
The TIM1 auto-reload is set to generate PWM signal at 5 Hz.
The channel 1 duty cycle is set to 25%.
The update event (UEV) is generated after that the upcounting is repeated for the number of times programmed in the repetition counter register (TIMx_RCR + 1). 
The waveform generated is a train of 3 (TIMx_RCR + 1).

The PWM waveform can be displayed using an oscilloscope. 

  
BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms:      
- TIMx_CH4 
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
#include "TIM_PWM_Train_main.h"

/** @addtogroup TIM_PWM_Trains
  * @{
  */

/** @addtogroup TIM_PWM_Train
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void Configure_TIMPWMOutput(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configures the timer to generate a PWM signal on the OC1 output with only a fixed number of cycles.
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void  Configure_TIMPWMOutput(void)
{
  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_EnableClock_TIMx_CH4();
  
  /* GPIO TIMx_CH1 configuration */
  LL_GPIO_SetPinMode(TIMx_CH4_PORT, TIMx_CH4_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinPull(TIMx_CH4_PORT, TIMx_CH4_PIN, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinSpeed(TIMx_CH4_PORT, TIMx_CH4_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetAFPin_TIMx_CH4(); 
  
  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  LL_EnableClock_TIMx();
  
  /* Set the pre-scaler value to have TIMx counter clock equal to 10 Hz */
  LL_TIM_SetPrescaler(TIMx, __LL_TIM_CALC_PSC(LL_TIM_GetPeriphClock(TIMx), 10));

  /* Set the auto-reload value to have a counter frequency of 5 Hz */
  /* TIMxCLK = LL_TIM_GetPeriphClock(TIMx) / (APB prescaler & multiplier)               */
  LL_TIM_SetAutoReload( TIMx, __LL_TIM_CALC_ARR(LL_TIM_GetPeriphClock(TIMx), LL_TIM_GetPrescaler(TIMx), 5));
  
  /* Set output mode */
  LL_TIM_OC_SetMode(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);
  
  /* Set output channel polarity */
  /* Reset value is LL_TIM_OCPOLARITY_HIGH */
  LL_TIM_OC_SetPolarity(TIMx, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);
  
  /* Set compare value to half of the counter period (50% duty cycle ) */
  LL_TIM_OC_SetCompareCH4(TIMx, ((LL_TIM_GetAutoReload(TIMx) + 1 ) * 0.50));
  
  /* Repetition counter to update the timer registers only after a given number of cycles of the counter */
  LL_TIM_SetRepetitionCounter(TIMx, 0x2);   // 3 cycles = 3 - 1 = 2 = TIMx_RCR
  
  /* Set the one pulse mode : Counter stops counting at the next update event */
  LL_TIM_SetOnePulseMode(TIMx, LL_TIM_ONEPULSEMODE_SINGLE);
  
  /* To be sure that the pulse will be repeate N times an update event must occur only */
  /* when the repetition counter is reloaded with the content of TIMx_RCR register */
   
  /* Enable output channel */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  
  /* Enable the outputs (set the MOE bit in TIMx_BDTR register). */
  LL_TIM_EnableAllOutputs(TIMx);

  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIMx);  

  /* Enable counter */
  LL_TIM_EnableCounter(TIMx); 
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }

  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Configure the timer in output compare mode */
  Configure_TIMPWMOutput();
  
  /* Infinite loop */
  while (1)
  {
  }
}

/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/

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



