
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : PKA_ECDSA_Sign_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 21-September-2015
* Description        : Code demonstrating the PKA functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  PKA_ECDSA_Sign/PKA_ECDSA_Sign_main.c
 * @brief How to use the HAL PKA API to generate an ECC signature.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
 <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\PKA\\PKA_ECDSA_Sign\\MDK-ARM\\{STEVAL-IDB012V1}\\PKA_ECDSA_Sign.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
   <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\PKA\\PKA_ECDSA_Sign\\EWARM\\{STEVAL-IDB012V1}\\PKA_ECDSA_Sign.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\PKA\\PKA_ECDSA_Sign\\WiSE-Studio\\{STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
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
|  PIN name  |    STEVAL-IDB012V1  |
-----------------------------------
|     A1     |       Not Used      |
|     A11    |       Not Used      |
|     A12    |         N.A.        |
|     A13    |         N.A.        |
|     A14    |         N.A.        |
|     A15    |         N.A.        |
|     A4     |         N.A.        |
|     A5     |         N.A.        |
|     A6     |         N.A.        |
|     A7     |         N.A.        |
|     A8     |       Not Used      |
|     A9     |       Not Used      |
|     B0     |       Not Used      |
|     B14    |       Not Used      |
|     B2     |       Not Used      |
|     B3     |       Not Used      |
|     B4     |         DL2         |
|     B5     |       Not Used      |
|     B7     |       Not Used      |
|     B8     |         N.A.        |
|     B9     |         N.A.        |
|     A0     |       Not Used      |
|     A10    |       Not Used      |
|     B1     |       Not Used      |
|     B6     |       Not Used      |
|     B15    |       Not Used      |
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
|  LED name  |                  STEVAL-IDB012V1                 |
-----------------------------------------------------------------
|     DL1    |                     Not Used                     |
|     DL2    |  ON: successful sequence - Slow blinking: error  |
|     DL3    |                     Not Used                     |
|     DL4    |                     Not Used                     |
|     U5     |                     Not Used                     |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |    STEVAL-IDB012V1   |
-----------------------------------------
|      PUSH1     |       Not Used       |
|      PUSH2     |       Not Used       |
|      RESET     |   Reset BlueNRG-LPS  |

@endtable

* \section Usage Usage

How to use the low-layer PKA API to generate an ECDSA signature.

Example execution:
After startup from reset and system configuration, PKA configuration is performed.

To demonstrate the generation of an ECDSA signature, input and output buffers of this example are extracted from National Institute of Standards and Technology (NIST) Cryptographic Algorithm Validation Program (CAVP) (http://csrc.nist.gov/groups/STM/cavp/).

One test vector has been adapted to C buffers in SigGen.c and SigGen.h.

The example:
  Loads the input buffers to PKA RAM.
  Launch the computation in interrupt mode.
  Wait for the interrupt callback.
  Retreive the output buffer.
  Compare to expected results.

After successful sequence, LED2 is turned On. 
In case of errors, LED2 is slowly blinking (1sec period).


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
#include "PKA_ECDSA_Sign_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define PRIME256V1_ORDER_LEN            32

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PKA_HandleTypeDef hpka;
PKA_ECDSASignInTypeDef in = {0};
PKA_ECDSASignOutTypeDef out = {0};
__IO uint32_t operationComplete = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_PKA_Init(void);

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
 
  /* Configure LED2 */
  BSP_LED_Init(BSP_LED2);
  
  /* Initialize all configured peripherals */
  MX_PKA_Init();

  /* Set input parameters */
  in.primeOrderSize =  prime256v1_Order_len;
  in.modulusSize =     prime256v1_Prime_len;
  in.coefSign =        prime256v1_A_sign;
  in.coef =            prime256v1_absA;
  in.modulus =         prime256v1_Prime;
  in.basePointX =      prime256v1_GeneratorX;
  in.basePointY =      prime256v1_GeneratorY;
  in.primeOrder =      prime256v1_Order;

  in.integer =         SigGen_k;
  in.hash =            SigGen_Hash_Msg;
  in.privateKey =      SigGen_d;
  
  /* Launch the verification */
  if(HAL_PKA_ECDSASign(&hpka, &in, 5000) != HAL_OK)
  {
    Error_Handler();
  }

  uint8_t rsign[PRIME256V1_ORDER_LEN] = {0};
  uint8_t ssign[PRIME256V1_ORDER_LEN] = {0};
  /* Allocate required space */
  out.RSign = rsign;
  out.SSign = ssign;
  if(out.RSign == NULL || out.SSign == NULL)
  {
    /* Not enough memory in heap */
    Error_Handler();
  }
  
  /* Copy the result to allocated space */
  HAL_PKA_ECDSASign_GetResult(&hpka , &out, NULL);
  
  /* Compare to expected result */
  if (memcmp(out.RSign, SigGen_R, SigGen_R_len) != 0)
  {
    Error_Handler();
  }
  
  if (memcmp(out.SSign, SigGen_S, SigGen_S_len) != 0)
  {
    Error_Handler();
  }
  
  /* Deinitialize the PKA */
  if(HAL_PKA_DeInit(&hpka) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Success */
  operationComplete = 1;
  BSP_LED_On(BSP_LED2);
  printf("** Test successfully. ** \n\r\n\r");
  
  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief PKA Initialization Function
  * @param None
  * @retval None
  */
static void MX_PKA_Init(void)
{
  hpka.Instance = PKA;
  if (HAL_PKA_Init(&hpka) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  operationComplete = 2;
  while (1)
  {
    /* Error if LED2 is slowly blinking (1 sec. period) */
    BSP_LED_Toggle(BSP_LED2); 
    HAL_Delay(1000);   
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
