
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : CRC_PolynomialUpdate_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the CRC functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  CRC_PolynomialUpdate/CRC_PolynomialUpdate_main.c 
 * @brief How to configure the CRC CRC operation with Polynomial update
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\CRC\\CRC_PolynomialUpdate\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\CRC_PolynomialUpdate.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\CRC\\CRC_PolynomialUpdate\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\CRC_PolynomialUpdate.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\MIX\\CRC\\CRC_PolynomialUpdate\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
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
|  LED name  |     STEVAL-IDB011V1    |     STEVAL-IDB011V2    |     STEVAL-IDB012V1    |
--------------------------------------------------------------------------------------------
|     DL1    |        Not Used        |        Not Used        |        Not Used        |
|     DL2    |   ON: right CRC value  |   ON: right CRC value  |   ON: right CRC value  |
|     DL3    |        ON: error       |        ON: error       |        ON: error       |
|     DL4    |        Not Used        |        Not Used        |        Not Used        |
|     U5     |        Not Used        |        Not Used        |        Not Used        |

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

How to use the CRC peripheral through the BLUENRG_LP CRC HAL and LL API.
The LL API is used for performance improvement. 
The CRC calculation unit computes the 8-bit CRC code of a given buffer of 32-bit data words, based on a user-defined generating polynomial. 
In this example, the polynomial is first set manually to 0x9B (X^8 + X^7 + X^4 + X^3 + X + 1).
In a second step, the generating polynomial value and length are updated and set to 0x1021  (X^16 + X^12 + X^5 + 1) for a new CRC calculation.
These updates are performed using the CRC LL API.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The CRC peripheral configuration is ensured by HAL_CRC_Init() function.
The latter is calling HAL_CRC_MspInit() function which core is implementing the configuration of the needed CRC resources according to the used hardware (CLOCK). 
You can update HAL_CRC_Init() input parameters to change the CRC configuration.

For each computation, calculated CRC code is stored in uwCRCValue variable.
Once calculated, the CRC value (uwCRCValue) is compared to the CRC expected value (uwExpectedCRCValue1 and uwExpectedCRCValue2).

BlueNRG_LP board LEDs are used to monitor the example status:
- LED2 is ON when correct CRC values are calculated
- LED3 is ON when there is an error in initialization or if an incorrect CRC value is calculated.

In order to make the program work, you must do the following:
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button

BlueNRG_LP-EVB Set-up
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
#include "CRC_PolynomialUpdate_main.h"

/* Private includes ----------------------------------------------------------*/
#include "rf_driver_ll_crc.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define BUFFER_SIZE    2

/* The user defined polynomial*/
#define CRC_POLYNOMIAL_8B  0x9B   /* X^8 + X^7 + X^4 + X^3 + X + 1 */
#define CRC_POLYNOMIAL_16B 0x1021 /* X^16 + X^12 + X^5 + 1 */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

/* Used for storing CRC Value */
__IO uint32_t uwCRCValue = 0;

/* Buffer containing the data on which the CRC will be calculated */
static const uint32_t aDataBuffer[BUFFER_SIZE] =
{
  0x12345678, 0x12213883
};

/* Expected CRC Value */
uint32_t uwExpectedCRCValue1 = 0xAB;
uint32_t uwExpectedCRCValue2 = 0x9357;

/* Private function prototypes -----------------------------------------------*/
static void MX_CRC_Init(void);
void Error_Handler(void);

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
  MX_CRC_Init();
  
  printf("Data:\n\r");
  for(int i=0; i<BUFFER_SIZE; i++)
  {
    printf("0x");
    printf("%02X", aDataBuffer[i]);
    printf("\n\r");
  }
  
  /*-2- Compute the CRC of "aDataBuffer" */
  /* First computation is done using 8-bit generating Polynomial (CRC_POLYNOMIAL_8B) 
  as configured in CRC handle, wtih HAL_CRC_Init */
  printf("First computation is done using 8-bit generating Polynomial (CRC_POLYNOMIAL_8B) as configured in CRC handle, wtih HAL_CRC_Init:\n\r");
  uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)&aDataBuffer, BUFFER_SIZE);
  
  /*-3- Compare the CRC value to the Expected one */
  printf("Compare the CRC value to the Expected one\n\r");
  printf("%#X\tuwCRCValue\n\r", uwCRCValue);
  printf("%#X\tuwExpectedCRCValue1\n\r", uwExpectedCRCValue1);
  
  if (uwCRCValue != uwExpectedCRCValue1)
  {
    /* Wrong CRC value: enter Error_Handler */
    printf("Wrong CRC value: enter Error_Handler.\n\r");
    Error_Handler();
  }
  else
  {
    
    printf("Right CRC value.\n\r");
    /* Update CRC generating polynomial length and value using LL services.
    In this case, using LL services allows to avoid calling HAL_CRC_Init()/HAL_CRC_DeInit */
    printf("Update CRC generating polynomial length and value using LL services. In this case, using LL services allows to avoid calling HAL_CRC_Init()/HAL_CRC_DeInit\n\r");
    LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_16B);
    LL_CRC_SetPolynomialCoef(CRC, CRC_POLYNOMIAL_16B);
    
    /* Following code sequence is needed in order to keep CRC handle structure 
    content in line with IP configuration */
    /* Set the value of the polynomial */
    //    hcrc.Init.GeneratingPolynomial    = CRC_POLYNOMIAL_16B;
    /* The user-defined generating polynomial generates a 16-bit long CRC */
    //    hcrc.Init.CRCLength               = CRC_POLYLENGTH_16B;
    
    /*-4- Compute the CRC of "aDataBuffer" */
    /* Second computation is done using 16-bit generating Polynomial (CRC_POLYNOMIAL_16B) 
    as configured in IP using LL APIs */
    printf("Second computation is done using 16-bit generating Polynomial (CRC_POLYNOMIAL_16B) as configured in IP using LL APIs\n\r");
    uwCRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)&aDataBuffer, BUFFER_SIZE);
    
    /*-5- Compare the CRC value to the Expected one */
    printf("Compare the CRC value to the Expected one\n\r");
    printf("%#X\tuwCRCValue\n\r", uwCRCValue);
    printf("%#X\tuwExpectedCRCValue2\n\r", uwExpectedCRCValue2);
    
    if (uwCRCValue != uwExpectedCRCValue2)
    {
      /* Wrong CRC value: enter Error_Handler */
      printf("Wrong CRC value: enter Error_Handler.\n\r");
      Error_Handler();
    }
    else
    {
      /* Right CRC value: Turn LED2 on */
      printf("Right CRC value.\n\r");
      BSP_LED_On(BSP_LED2);
    }
  }
  /* Infinite loop */
  while (1)
  {
  }
}

/**
* @brief CRC Initialization Function
* @param None
* @retval None
*/
static void MX_CRC_Init(void)
{
  /*-1- Configure the CRC peripheral */
  hcrc.Instance = CRC;
  /* The default polynomial is not used. It is required to defined it in hcrc.Init.GeneratingPolynomial*/  
  hcrc.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_DISABLE;
  /* Set the value of the polynomial */
  hcrc.Init.GeneratingPolynomial    = CRC_POLYNOMIAL_8B;
  /* The user-defined generating polynomial generates a 8-bit long CRC */
  hcrc.Init.CRCLength               = CRC_POLYLENGTH_8B;
  /* The default init value is used */
  hcrc.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
  /* The input data are not inverted */
  hcrc.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
  /* The output data are not inverted */
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  /* The input data are 32- bit long */
  hcrc.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
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
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
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



