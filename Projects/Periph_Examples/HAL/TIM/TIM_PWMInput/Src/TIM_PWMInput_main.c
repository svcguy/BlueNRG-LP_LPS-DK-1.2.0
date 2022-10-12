
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : TIM_PWMInput_main.c
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
 * @file TIM_PWMInput/TIM_PWMInput_main.c
 * @brief How to use the TIM peripheral to measure the frequency and duty cycle of an external signal.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_PWMInput\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_PWMInput.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_PWMInput\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\TIM_PWMInput.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\TIM\\TIM_PWMInput\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|     A1     |       Not Used      |      USART TX      |
|     A11    |       Not Used      |      Not Used      |
|     A12    |       Not Used      |        N.A.        |
|     A13    |       Not Used      |        N.A.        |
|     A14    |       Not Used      |        N.A.        |
|     A15    |       Not Used      |        N.A.        |
|     A4     |       TIM1 CH1      |        N.A.        |
|     A5     |       TIM1 CH2      |        N.A.        |
|     A6     |       Not Used      |        N.A.        |
|     A7     |       Not Used      |        N.A.        |
|     A8     |       USART TX      |      Not Used      |
|     A9     |       USART RX      |      Not Used      |
|     B0     |       Not Used      |      USART RX      |
|     B14    |       Not Used      |      Not Used      |
|     B2     |       Not Used      |      Not Used      |
|     B3     |       Not Used      |      Not Used      |
|     B4     |       Not Used      |        DL2         |
|     B5     |       Not Used      |      TIM2 CH2      |
|     B7     |       Not Used      |      Not Used      |
|     B8     |         DL2         |        N.A.        |
|     B9     |         DL3         |        N.A.        |
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
|  LED name  |              STEVAL-IDB011V1             |              STEVAL-IDB011V2             |              STEVAL-IDB012V1             |
--------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |                 Not Used                 |                 Not Used                 |                 Not Used                 |
|     DL2    |  toggle: during the Input Capture value  |  toggle: during the Input Capture value  |  toggle: during the Input Capture value  |
|     DL3    |           Slow blinking: error           |           Slow blinking: error           |           Slow blinking: error           |
|     DL4    |                 Not Used                 |                 Not Used                 |                 Not Used                 |
|     U5     |                 Not Used                 |                 Not Used                 |                 Not Used                 |

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

How to use the TIM peripheral to measure the frequency and duty cycle of an external signal.

The TIM1CLK frequency is set to 64MHz, the Prescaler is 0 so the counter clock is 64MHz.

TIM1 is configured in PWM Input Mode: the external signal is connected to TIM1 Channel2 used as input pin.
To measure the frequency and the duty cycle, we use the TIM1 CC2 interrupt request, so in the timer IRQ routine (via call to function HAL_TIM_IC_CaptureCallback() ), the frequency and the duty cycle of the external signal are computed.

"uwFrequency" variable contains the external signal frequency:
TIM1 counter clock = 64MHz,
uwFrequency = TIM1 counter clock / TIM1_CCR2 in Hz, 

"uwDutyCycle" variable contains the external signal duty cycle:
uwDutyCycle = (TIM1_CCR1*100)/(TIM1_CCR2) in %.

The minimum frequency value to measure is (TIM1 counter clock / CCR MAX)
                                         = (64 MHz)/ 65535

In case of error, LED3 is turned ON.

BlueNRG_LP-EVB Set-up
Connect the external signal to (you can use the TIM_PWMOutput project as externel signal to measure into a second board)
- TIMx CH2 pin
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
#include "TIM_PWMInput_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htimx;

/* Captured Value */
__IO uint32_t            uwIC2Value = 0;
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
/* Duty Cycle Value */
__IO uint32_t            uwDutyCycle = 0;
#endif
/* Frequency Value */
__IO uint32_t            uwFrequency = 0;

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
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  /* Configure LED2 and LED3 */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIMx_Init();

  /* ---------------------------------------------------------------------------
  TIMx configuration: PWM Input mode

  External Signal Frequency = TIMx counter clock / TIMx_CCR2 in Hz.

  External Signal DutyCycle = (TIMx_CCR1*100)/(TIMx_CCR2) in %.
  --------------------------------------------------------------------------- */

  /*   Start the Input Capture in interrupt mode                              */
  if (HAL_TIM_IC_Start_IT(&htimx, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /*     Start the Input Capture in interrupt mode                            */
  if (HAL_TIM_IC_Start_IT(&htimx, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
#endif
  /* Infinite loop */
  while (1)
  {
    HAL_Delay(500);
    printf("uwFrequency = %d\n\r", uwFrequency);
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)    
    printf("uwDutyCycle = %d\n\r", uwDutyCycle);
#endif
    printf("\n\r");
  }
}

/**
  * @brief TIMx Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIMx_Init(void)
{
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htimx.Instance = TIMx;
  htimx.Init.Prescaler = 0;
  htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
  htimx.Init.Period = 0xFFFF;
  htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htimx) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htimx, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
///*****************/
//
//  /* Reset the MMS Bits */
//  htimx.Instance->CR2 &= ~TIM_CR2_MMS;
//  /* Select the TRGO source */
//  htimx.Instance->CR2 |=  TIM_TRGO_RESET;
//
//  /* Reset the MSM Bit */
//  htimx.Instance->SMCR &= ~TIM_SMCR_MSM;
//  /* Set or Reset the MSM Bit */
//  htimx.Instance->SMCR |= TIM_MASTERSLAVEMODE_DISABLE;
//
///******************************/
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htimx, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
#endif
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htimx, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim : TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Get the Input Capture value */
    uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    if (uwIC2Value != 0)
    {
      /* LED2 toggle during the Input Capture value */
      BSP_LED_Toggle(BSP_LED2);
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
      /* Duty cycle computation */
      uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;
#endif
      /* uwFrequency computation */
      uwFrequency =  (uint32_t) ( HAL_TIM_GetPeriphClock(htimx.Instance) / uwIC2Value );
    }
    else
    {
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
      uwDutyCycle = 0;
#endif
      uwFrequency = 0;
    }
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* Turn LED3 on */
  BSP_LED_On(BSP_LED3);
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


