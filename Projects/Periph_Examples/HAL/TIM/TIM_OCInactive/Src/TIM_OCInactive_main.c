
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_OCInactive_main.c
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
 * @file  TIM_OCInactive/TIM_OCInactive_main.c
 * @brief Configuration of the TIM peripheral in Output Compare Inactive mode with the corresponding Interrupt requests for each channel.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_OCInactive\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OCInactive.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_OCInactive\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_OCInactive.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_OCInactive\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A11    |       Output 1      |      Not Used      |
|     A12    |       Output 2      |        N.A.        |
|     A13    |       Output 3      |        N.A.        |
|     A14    |       Output 4      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       Not Used      |        N.A.        |
|     A5     |       Not Used      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       Not Used      |      Not Used      |
|     A9     |       Not Used      |      Not Used      |
|     B0     |       Not Used      |      Not Used      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Output 1      |
|     B3     |       Not Used      |      Output 2      |
|     B4     |       Not Used      |      Not Used      |
|     B5     |       Not Used      |      Output 4      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |       Not Used      |        N.A.        |
|     B9     |       Not Used      |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      Output 3      |
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

Configuration of the TIM peripheral in Output Compare Inactive mode with the corresponding Interrupt requests for each channel.

The TIM1 frequency is set to 64MHz, and the objective is to get TIM1 
counter clock at 10 kHz so the Prescaler is computed as following:
Prescaler = (TIM1CLK /TIM1 counter clock) - 1

The TIM1 CCR1 register value is equal to 10000:
TIM1_CH1 delay = CCR1_Val/TIM1 counter clock  = 1s
so the TIM1 Channel 1 generates a signal with a delay equal to 1s.

The TIM1 CCR2 register value is equal to 5000:
TIM1_CH2 delay = CCR2_Val/TIM1 counter clock = 500 ms
so the TIM1 Channel 2 generates a signal with a delay equal to 500 ms.

The TIM1 CCR3 register value is equal to 2500:
TIM1_CH3 delay = CCR3_Val/TIM1 counter clock = 250 ms
so the TIM1 Channel 3 generates a signal with a delay equal to 250 ms.

The TIM1 CCR4 register value is equal to 1250:
TIM1_CH4 delay = CCR4_Val/TIM1 counter clock = 125 ms
so the TIM1 Channel 4 generates a signal with a delay equal to 125 ms.

While the counter is lower than the Output compare registers values, which determines the Output delay, the PA11, PA12, PA13 and PA14 pin are turned ON.  

When the counter value reaches the Output compare registers values, the Output Compare interrupts are generated and, in the handler routine, these pins are turned OFF.

BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms (start the captur on rising edge detection):
- GPIO_CH1 : PA.11
- GPIO_CH2 : PA.12
- GPIO_CH3 : PA.13
- GPIO_CH4 : PA.14

BlueNRG_LP-EVB Set-up
Connect the following pins to an oscilloscope to monitor the different waveforms (start the captur on rising edge detection):
- GPIO_CH1 : PB.2
- GPIO_CH2 : PB.3
- GPIO_CH3 : PB.6
- GPIO_CH4 : PB.5

**/
   

/* Includes ------------------------------------------------------------------*/
#include "TIM_OCInactive_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_TIMx_Init(void);

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
  HAL_Init();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIMx_Init();
  
  /* Set GPIO Pins as reference */ 
  HAL_GPIO_WritePin(GPIO_OUT_PORT_CH1_2_3_4, GPIO_OUT_CH1|GPIO_OUT_CH2|GPIO_OUT_CH3|GPIO_OUT_CH4, GPIO_PIN_SET);
  
  /* Start signals generation */ 
  /* Start channel 1 in Output compare mode */
  if(HAL_TIM_OC_Start_IT(&htimx, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 2 in Output compare mode */
  if(HAL_TIM_OC_Start_IT(&htimx, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 3 in Output compare mode */
  if(HAL_TIM_OC_Start_IT(&htimx, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 4 in Output compare mode */
  if(HAL_TIM_OC_Start_IT(&htimx, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief TIMx Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIMx_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htimx.Instance = TIMx;
  htimx.Init.Prescaler = PRESCALER_VALUE;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = 65535;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = PULSE1_VALUE;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PULSE2_VALUE;
  if (HAL_TIM_OC_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PULSE3_VALUE;
  if (HAL_TIM_OC_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = PULSE4_VALUE;
  if (HAL_TIM_OC_ConfigChannel(&htimx, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htimx);
}   

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  EnableClock_GPIOx();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT_PORT_CH1_2_3_4, GPIO_OUT_CH1|GPIO_OUT_CH2|GPIO_OUT_CH3|GPIO_OUT_CH4, GPIO_PIN_RESET);

  /*Configure GPIO pins */
  GPIO_InitStruct.Pin = GPIO_OUT_CH1|GPIO_OUT_CH2|GPIO_OUT_CH3|GPIO_OUT_CH4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO_OUT_PORT_CH1_2_3_4, &GPIO_InitStruct);
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    /* Reset GPIO Pin PB2 after 1s */
    HAL_GPIO_WritePin(GPIO_OUT_PORT_CH1_2_3_4, GPIO_OUT_CH1, GPIO_PIN_RESET);
  }
  else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Reset GPIO Pin PB3 after 500ms */
    HAL_GPIO_WritePin(GPIO_OUT_PORT_CH1_2_3_4, GPIO_OUT_CH2, GPIO_PIN_RESET);
  }
  else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    /* Reset GPIO Pin PB4 after 250ms */
    HAL_GPIO_WritePin(GPIO_OUT_PORT_CH1_2_3_4, GPIO_OUT_CH3, GPIO_PIN_RESET);
  }
  else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    /* Reset GPIO Pin PB5 after 125ms */    
    HAL_GPIO_WritePin(GPIO_OUT_PORT_CH1_2_3_4, GPIO_OUT_CH4, GPIO_PIN_RESET);
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
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


