
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : PKA_ECC_Sign_main.c
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
 * @file  PKA_ECC_Sign/PKA_ECC_Sign_main.c
 * @brief How to use the HAL PKA API to generate an ECC signature.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
 <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\PKA\\PKA_ECC_Sign\\MDK-ARM\\{STEVAL-IDB011V1}\\PKA_ECC_Sign.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
   <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\PKA\\PKA_ECC_Sign\\EWARM\\{STEVAL-IDB011V1}\\PKA_ECC_Sign.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\PKA\\PKA_ECC_Sign\\WiSE-Studio\\{STEVAL-IDB011V1}</tt> 
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
-----------------------------------
|     A1     |       Not Used      |
|     A11    |       Not Used      |
|     A12    |       Not Used      |
|     A13    |       Not Used      |
|     A14    |       Not Used      |
|     A15    |       Not Used      |
|     A4     |       Not Used      |
|     A5     |       Not Used      |
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
|  LED name  |                  STEVAL-IDB011V1                 |                  STEVAL-IDB011V2                 |
---------------------------------------------------------------------------------------------------------------------
|     DL1    |                     Not Used                     |                     Not Used                     |
|     DL2    |  ON: successful sequence - Slow blinking: error  |  ON: successful sequence - Slow blinking: error  |
|     DL3    |                     Not Used                     |                     Not Used                     |
|     DL4    |                     Not Used                     |                     Not Used                     |
|     U5     |                     Not Used                     |                     Not Used                     |

@endtable

* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |
-------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |
|      RESET     |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

How to use the HAL PKA API to generate an ECC signature.

Example execution:
After startup from reset and system configuration, PKA configuration is performed.

This demo provides a basic examples about how to use the available HAL PKA driver APIs in order to 
perform a basic PKA processing and check the results. 
The Public Key Accelerator (PKA) is a dedicated HW block used for computation of cryptographic 
public key primitives related to ECC (Elliptic curve Cryptography). 
Please notice that the BlueNRG_LP-EVB Bluetooth Low Energy stack uses this peripheral during the 
security pairing procedures, so user application must not use it during such procedures. 


The example:
 - Starting from the PKA know point and from a random generated kA, 
   it performs a PKA process, which generates a new point A on the ellipse. 
 - The same process is repeated from a new generated random kB, leading to a new point B on the ellipse. 
 - At this stage, a new PKA process is started using the kA with the point B coordinates. 
 - This generates a new point C which is still on the same ellipse. 


After successful sequence, LED2 is turned On. 
In case of errors, LED2 is slowly blinking (1sec period).

  
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
#include "PKA_ECC_Sign_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PKA_HandleTypeDef hpka;
RNG_HandleTypeDef hrng;

__IO uint32_t endOfProcess = 0;
uint8_t pResX[32] = {0};
uint8_t pResY[32] = {0};

/* Private function prototypes -----------------------------------------------*/
static void MX_PKA_Init(void);
static void MX_RNG_Init(void);

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
  
  /* Configure LED */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* Initialize all configured peripherals */
  MX_PKA_Init();
  MX_RNG_Init();
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
  
  static uint32_t Public_Key_Out_A[16], Public_Key_Out_B[16];

  static uint32_t RandomKA[16] = {0};
  static uint32_t RandomKB[16] = {0};
  
  /* mode to Elliptic Curve Diffie-Hellman (ECDH)  signature generation in interrupt mode */
  printf("Elliptic Curve Diffie-Hellman (ECDH)  signature generation in interrupt mode.\n\r");
  
  /* ***** PROCEDURE FOR A ***** */
  /* Generate the random K for point A */
  printf("Generate the random K for point A\n\r");
  if (HAL_RNG_GenerateRandomNumber(&hrng, &RandomKA[0]) != HAL_OK)
  {
    Error_Handler();      
  }
  
  /* Start PKA processing */
  endOfProcess = 0;
  /* Launch the computation in interrupt mode */
  if (HAL_PKA_StartProc_IT(&hpka, &RandomKA[0], 10000, NULL) != HAL_OK)
  {
    Error_Handler();      
  }  
  
  /* Wait for the end of PKA processing */
  /* Wait for the interrupt callback */    
  while(endOfProcess != 1);
  
  HAL_PKA_GetResult(&hpka, PKA_DATA_PCX, (uint8_t *)&Public_Key_Out_A[0]);
  HAL_PKA_GetResult(&hpka, PKA_DATA_PCY, (uint8_t *)&Public_Key_Out_A[8]);
  
  printf("Point A of the ellipse generated.\n\r");
  
  /* ***** PROCEDURE FOR B ***** */
  /* Generate the random K for point B */ 
  printf("Generate the random K for point B.\n\r");
  if (HAL_RNG_GenerateRandomNumber(&hrng, &RandomKB[0]) != HAL_OK)
  {
    Error_Handler();      
  }
  
  /* Start PKA processing */
  endOfProcess = 0;
  /* Launch the computation in interrupt mode */
  if (HAL_PKA_StartProc_IT(&hpka, &RandomKB[0], 10000, NULL) != HAL_OK)
  {
    Error_Handler();      
  }    
  
  /* Wait for the end of PKA processing */
  /* Wait for the interrupt callback */    
  while(endOfProcess != 1);
  
  HAL_PKA_GetResult(&hpka, PKA_DATA_PCX, (uint8_t *)&Public_Key_Out_B[0]);
  HAL_PKA_GetResult(&hpka, PKA_DATA_PCY, (uint8_t *)&Public_Key_Out_B[8]);
  
  printf("Point B of the ellipse generated.\n\r");
  
  /* ***** CHECK ELLIPTIC ***** */
  printf("CHECK ELLIPTIC\n\r");
  
  /* Insert the secrete coordinate of the point B */
  
  /* Start PKA processing */
  endOfProcess = 0;
  
  /* Insert the random k used to calculate the point A */
  if (HAL_PKA_StartProc_IT(&hpka, &RandomKA[0], 10000, &Public_Key_Out_B[0]) != HAL_OK)
  {
    Error_Handler();      
  }
  else  
  {
    /* PKA verified */
    printf("PKA verified.\n\r");
    printf("** Test successfully. ** \n\r\n\r");
    BSP_LED_On(BSP_LED2);
  }
  
  /* Wait for the end of PKA processing */
  /* Wait for the interrupt callback */
  while(endOfProcess != 1);
  
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
  /* Configure NVIC for PKA interrupts */
  /*   Set priority for PKA_IRQn */
  /*   Enable PKA_IRQn */
  NVIC_SetPriority(PKA_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(PKA_IRQn); 
}  

void HAL_PKA_OperationCpltCallback(PKA_HandleTypeDef *hpka)
{
  endOfProcess = 1;
}

/**
* @brief RNG Initialization Function
* @param None
* @retval None
*/
static void MX_RNG_Init(void)
{
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
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
  while (1)
  {
    /* Error if LED2 is slowly blinking (1 sec. period) */
    BSP_LED_Toggle(BSP_LED3); 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
