
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_PWMOutput_main.c
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
 * @file  TIM_PWMOutput/TIM_PWMOutput_main.c
 * @brief Configuration of the TIM peripheral in PWM (pulse width modulation) mode.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_PWMOutput\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_PWMOutput.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_PWMOutput\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_PWMOutput.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_PWMOutput\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A11    |       TIM1 CH6      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       TIM1 CH1      |        N.A.        |
|     A5     |       TIM1 CH2      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       Not Used      |      Not Used      |
|     A9     |       Not Used      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       TIM1_CH5      |      Not Used      |
|     B2     |       TIM1 CH3      |      Not Used      |
|     B3     |       Not Used      |      TIM17 CH1     |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      TIM17 BKIN    |
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

Configuration of the TIM peripheral in PWM (pulse width modulation) mode.

TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%
TIM1 Channel5 duty cycle = (TIM1_CCR5/ TIM1_ARR + 1)* 100 = 75%
TIM1 Channel6 duty cycle = (TIM1_CCR6/ TIM1_ARR + 1)* 100 = 62.5%

The PWM waveforms can be displayed using an oscilloscope.

The duty cycles values mentioned above are theoretical (obtained when the system clock frequency is exactly 64 MHz).
They might be slightly different depending on system clock frequency precision.

BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms:
- TIMx_CH1
- TIMx_CH2 
- TIMx_CH3 
- TIMx_CH4
- TIMx_CH5 
- TIMx_CH6
Connect the TIM1 break to the GND. To generate a break event, switch this pin level from 0V to 3.3V.  
- TIM1_BKIN 

**/


/* Includes ------------------------------------------------------------------*/
#include "TIM_PWMOutput_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIM_Init(void);

/* Private user code ---------------------------------------------------------*/

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
  HAL_Init();
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM_Init();
  
  /* Start PWM signals generation */
  /* Start channel 1 */
  if (HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
#ifdef TIMx_CH2_PIN
  /* Start channel 2 */
  if (HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
#endif
#ifdef TIMx_CH3_PIN
  /* Start channel 3 */
  if (HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
#endif
#ifdef TIMx_CH4_PIN
  /* Start channel 4 */
  if (HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_4) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
#endif
#ifdef TIMx_CH5_PIN  
  /* Start channel 5 */
  if (HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_5) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
#endif 
#ifdef TIMx_CH6_PIN
  /* Start channel 6 */
  if (HAL_TIM_PWM_Start(&htimx, TIM_CHANNEL_6) != HAL_OK)
  {
    /* PWM generation Error */
    Error_Handler();
  }
#endif 
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief TIM1 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  htimx.Instance = TIMx;
  htimx.Init.Prescaler = PRESCALER_VALUE;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = PERIOD_VALUE;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;  
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.Pulse = PULSE1_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
#ifdef TIMx_CH2_PIN
  sConfigOC.Pulse = PULSE2_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
#endif
#ifdef TIMx_CH3_PIN
  sConfigOC.Pulse = PULSE3_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
#endif
#ifdef TIMx_CH4_PIN
  sConfigOC.Pulse = PULSE4_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
#endif
#ifdef TIMx_CH5_PIN
  sConfigOC.Pulse = PULSE5_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
#endif 
#ifdef TIMx_CH6_PIN
  sConfigOC.Pulse = PULSE6_VALUE;
  if (HAL_TIM_PWM_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
#endif
    
  TIM_BreakDeadTimeConfigTypeDef TIM_BDTRInitStruct = {0};
  
  /* Configure the Break and Dead Time feature of the TIMx */
  TIM_BDTRInitStruct.OffStateRunMode = TIM_OSSR_ENABLE;
  TIM_BDTRInitStruct.OffStateIDLEMode = TIM_OSSI_ENABLE;
  TIM_BDTRInitStruct.LockLevel = TIM_LOCKLEVEL_1;
  TIM_BDTRInitStruct.DeadTime = __HAL_TIM_CALC_DEADTIME(HAL_TIM_GetPeriphClock(TIMx), 0, 100);
  TIM_BDTRInitStruct.BreakState = TIM_BREAK_ENABLE;
  TIM_BDTRInitStruct.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  TIM_BDTRInitStruct.BreakFilter = TIM_BREAK_FILTER_FDIV1;
#ifdef TIM_BDTR_BKBID
  TIM_BDTRInitStruct.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
#endif /* TIM_BDTR_BKBID */
#ifdef TIM_BREAK2_DISABLE
  TIM_BDTRInitStruct.Break2State = TIM_BREAK2_DISABLE;
#endif /* TIM_BREAK2_DISABLE */
#ifdef TIM_BREAK2POLARITY_HIGH
  TIM_BDTRInitStruct.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
#endif /* TIM_BREAK2POLARITY_HIGH */  
#ifdef TIM_BREAK2_FILTER_FDIV1  
  TIM_BDTRInitStruct.Break2Filter = TIM_BREAK2_FILTER_FDIV1;
#endif /* TIM_BREAK2_FILTER_FDIV1 */
  TIM_BDTRInitStruct.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htimx, &TIM_BDTRInitStruct);
  
  
  HAL_TIM_MspPostInit(&htimx);
}   

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  while(1) 
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
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
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


