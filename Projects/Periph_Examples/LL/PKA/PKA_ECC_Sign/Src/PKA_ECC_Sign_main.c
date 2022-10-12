
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
 * @brief This example demonstrates how to use the low-layer PKA API to generate an ECC signature
 *


* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
 <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PKA\\PKA_ECC_Sign\\MDK-ARM\\{STEVAL-IDB011V1}\\PKA_ECC_Sign.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
   <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PKA\\PKA_ECC_Sign\\EWARM\\{STEVAL-IDB011V1}\\PKA_ECC_Sign.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\LL\\PKA\\PKA_ECC_Sign\\WiSE-Studio\\{STEVAL-IDB011V1}</tt> 
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
This demo provides a basic examples about how to use the available PKA driver APIs in order to perform a basic PKA processing and check the results. 
The Public Key Accelerator (PKA) is a dedicated HW block used for computation of cryptographic public key primitives related to ECC (Elliptic curve Cryptography).
Please notice that the BlueNRG-LP-EVB Bluetooth Low Energy stack uses this peripheral during the security pairing procedures, so user application must not use it during such procedures.
The PKA demonstration applications performs the following steps: 
Starting from the PKA know point (PKA_SetData with PKA_DATA_PCX, PKA_DATA_PCY) and from a random generated kA, it performs a PKA process, which generates a new point A on the ellipse. 
The same process is repeated from a new generated random kB, leading to a new point B on the ellipse. 
At this stage, a new PKA process is started using the previous random kA with the point B coordinates. 
This generates a new point C which is still on the same ellipse. 

  
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

__IO uint32_t endOfProcess = 0;
uint8_t RBuffer[32] = {0};
uint8_t SBuffer[32] = {0};

/* Private function prototypes -----------------------------------------------*/
void PrintBuffer(uint32_t* pBuffer, uint32_t BufferLength);
static void LL_Init(void);
static void MX_GPIO_Init(void);
static void MX_PKA_Init(void);
static void MX_RNG_Init(void);
void LED_On(void);
void LED_Blinking(uint32_t Period);

#define FIXED_K_VALUE                      0         /* 1: set fixed randoKA and RandomKB, to test the output of the PKA ECC DH */
/*
Expected value with FIXED_K_VALUE equal to 1.

RandomKA
0x00005E5F 0x00000000 0x00000000 0x00000000
0x00000000 0x00000000 0x00000000 0x00000000
0x00000000 0x00000000 0x00000000 0x00000000
0x00000000 0x00000000 0x00000000 0x00000000

RandomKB
0x000083FC 0x00000000 0x00000000 0x00000000
0x00000000 0x00000000 0x00000000 0x00000000
0x00000000 0x00000000 0x00000000 0x00000000
0x00000000 0x00000000 0x00000000 0x00000000

Public_Key_Out_A
Data:
0xEE80AADE 0xF458AD60 0x635B77EA 0xA8CC1FEB
0x700DEE70 0xD31F447C 0xF6A1319A 0x4915ED08
0xF0111A82 0xAD38071A 0xDCB9F308 0x77F0BAB8
0xA0FAFD61 0xF36CA7DA 0xD2F8209C 0x552E3E71

Public_Key_Out_B
Data:
0x69286B4A 0xBB1707AE 0xDF087894 0x4C7C530A
0xC61BBC45 0x2F83FBA4 0xBA4BF2A2 0x3C90B82E
0x8DACE3AE 0xCAEEFACD 0xE9547954 0x0C7AF8F1
0xC74DB802 0xAA2BC7CB 0xF0223A9B 0x0E8AAA11

*/
void GenerateRandomK16Word(uint32_t* buffer);

/* Private user code ---------------------------------------------------------*/

static uint32_t Public_Key_Out_A[16], Public_Key_Out_B[16]; 
  
/**
* @brief  The application entry point.
* @retval int
*/
int main(void)
{
  /* PKA requires a starting point necessary to start the processing */
  const uint32_t PKAStartPoint[16] = {
    INITIAL_START_POINT_X_W1, INITIAL_START_POINT_X_W2, INITIAL_START_POINT_X_W3, INITIAL_START_POINT_X_W4,
    INITIAL_START_POINT_X_W5, INITIAL_START_POINT_X_W6, INITIAL_START_POINT_X_W7, INITIAL_START_POINT_X_W8,
    INITIAL_START_POINT_Y_W1, INITIAL_START_POINT_Y_W2, INITIAL_START_POINT_Y_W3, INITIAL_START_POINT_Y_W4,
    INITIAL_START_POINT_Y_W5, INITIAL_START_POINT_Y_W6, INITIAL_START_POINT_Y_W7, INITIAL_START_POINT_Y_W8};
  
  static uint32_t RandomKA[16] = {0};
  static uint32_t RandomKB[16] = {0};
  
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
  BSP_COM_Init(NULL);
 
  printf("\t\tPKA_ECC_Sign\n\r\n\r");
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_PKA_Init();
  MX_RNG_Init();
  
  /* Elliptic Curve Diffie-Hellman (ECDH)  signature generation in interrupt mode */
  printf("Elliptic Curve Diffie-Hellman (ECDH)  signature generation in interrupt mode.\n\r");
  LL_PKA_EnableIT_ADDRERR(PKA);
  LL_PKA_EnableIT_RAMERR(PKA);
  LL_PKA_EnableIT_PROCEND(PKA);
  
  /* ***** PROCEDURE FOR A ***** */
  /* Generate the random K for point A */
  printf("Generate the random K for point A\n\r");
  /* Starting from the PKA know point (LL_PKA_SetData with PKA_DATA_PCX, PKA_DATA_PCY) 
     and from a random generated kA, it performs a PKA process, 
     which generates a new point A on the ellipse. */
  GenerateRandomK16Word(RandomKA);

#if FIXED_K_VALUE
  /* Fixed value to test */
  RandomKA[0] = 0x00005E5F;
#endif
  
  /* Insert the random K for point A */
  if (LL_PKA_SetData(LL_PKA_DATA_SK, RandomKA) == ERROR)
  {
    LED_Blinking(LED_BLINK_ERROR);
  } 
  
  /* Insert the initial starting point coordinates */
  LL_PKA_SetData(LL_PKA_DATA_PCX, (uint32_t *)&PKAStartPoint[0]);
  LL_PKA_SetData(LL_PKA_DATA_PCY, (uint32_t *)&PKAStartPoint[8]);
  
  /* Launch the computation in interrupt mode */
  LL_PKA_Start(PKA);
  
  printf("RandomKA \n\r");
  PrintBuffer((uint32_t *)&RandomKA[0],16);
  printf("PKAStartPoint \n\r");
  PrintBuffer((uint32_t *)&PKAStartPoint[0],16);
  
  /* Wait for the interrupt callback */
  while(endOfProcess != 1);
  
  /* Compare to expected results */
  if (LL_PKA_VerifyProcess() == ERROR)
  {
    LED_Blinking(LED_BLINK_ERROR);
  } 
  
  /* Get the new calculated point A of the ellipse */
  LL_PKA_GetData(LL_PKA_DATA_PCX, (uint8_t *)&Public_Key_Out_A[0]);
  LL_PKA_GetData(LL_PKA_DATA_PCY, (uint8_t *)&Public_Key_Out_A[8]);
  
  printf("Point A of the ellipse generated.\n\r");

  printf("Public_Key_Out_A \n\r");
  PrintBuffer((uint32_t *)&Public_Key_Out_A[0],16);  

  /* ***** PROCEDURE FOR B ***** */
  /* Generate the random K for point B */
  printf("Generate the random K for point B.\n\r");
  /* The previous process is repeated from a new generated random kB, 
     leading to a new point B on the ellipse. */
  GenerateRandomK16Word(RandomKB); 
  
#if FIXED_K_VALUE
  /* Fixed value to test */
  RandomKB[0] = 0x000083FC;
#endif 
  
  /* Generate the random K for point B */
  LL_PKA_SetData(LL_PKA_DATA_SK, RandomKB);
  
  /* Insert the initial starting point coordinates */
  LL_PKA_SetData(LL_PKA_DATA_PCX, (uint32_t *)&PKAStartPoint[0]);
  LL_PKA_SetData(LL_PKA_DATA_PCY, (uint32_t *)&PKAStartPoint[8]);
  
  endOfProcess = 0;
  /* Start PKA processing */
  LL_PKA_Start(PKA);
    
  printf("RandomKB \n\r");
  PrintBuffer((uint32_t *)&RandomKB[0],16);
  printf("PKAStartPoint \n\r");
  PrintBuffer((uint32_t *)&PKAStartPoint[0],16);
  /* Wait for the end of PKA processing */
  /* Wait for the interrupt callback */
  while(endOfProcess != 1);
  
  /* Verify if the PKA process is ended with success (valid point) */
  if (LL_PKA_VerifyProcess() == ERROR)
  {
    LED_Blinking(LED_BLINK_ERROR);
  }
  
  /* Get the new calculated point B of the ellipse */
  LL_PKA_GetData(LL_PKA_DATA_PCX, (uint8_t *)&Public_Key_Out_B[0]);
  LL_PKA_GetData(LL_PKA_DATA_PCY, (uint8_t *)&Public_Key_Out_B[8]);
  
  printf("Point B of the ellipse generated.\n\r");
  
  printf("Public_Key_Out_B \n\r");
  PrintBuffer((uint32_t *)&Public_Key_Out_B[0],16);
  
  /* ***** CHECK ELLIPTIC ***** */
  printf("CHECK ELLIPTIC\n\r");
  /* New PKA process is started using the RandomKA with the point B coordinates. 
     This generates a new point C which is still on the same ellipse */
  
  /* Insert the random k used to calculate the point A */
  LL_PKA_SetData(LL_PKA_DATA_SK, RandomKA);
  
  /* Insert the secrete coordinate of the point B */
  LL_PKA_SetData(LL_PKA_DATA_PCX, (uint32_t *)&Public_Key_Out_B[0]);
  LL_PKA_SetData(LL_PKA_DATA_PCY, (uint32_t *)&Public_Key_Out_B[8]);
  
  /* Start PKA processing */
  endOfProcess = 0;
  /* Start PKA processing */
  LL_PKA_Start(PKA);
  
  /* Wait for the end of PKA processing */
  /* Wait for the interrupt callback */
  while(endOfProcess != 1);
  
  /* Verify if the PKA process is ended with success (valid point) */
  if (LL_PKA_VerifyProcess() == ERROR)
  {
    LED_Blinking(LED_BLINK_ERROR);
  } 
  else {
    /* PKA verified */
    printf("PKA verified.\n\r");
    LED_On(); 
    printf("** Test successfully. ** \n\r\n\r");
  }
  
  /* Infinite loop */
  while (1)
  {
  }
}

void PrintBuffer(uint32_t* pBuffer, uint32_t BufferLength)
{
  printf("Data:\n\r");
  for(int i=0; i<BufferLength; i++)
  {
    printf("0x%08X ", pBuffer[i++]);
    printf("0x%08X ", pBuffer[i++]);
    printf("0x%08X ", pBuffer[i++]);
    printf("0x%08X ", pBuffer[i]);
    printf("\n\r");
  }
}

static void LL_Init(void)
{
  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, IRQ_HIGH_PRIORITY);
}

/**
* @brief PKA Initialization Function
* @param None
* @retval None
*/
static void MX_PKA_Init(void)
{
  /* Peripheral clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_PKA);
    
  /* Configure NVIC for PKA interrupts */
  /*   Set priority for PKA_IRQn */
  /*   Enable PKA_IRQn */
  NVIC_SetPriority(PKA_IRQn, IRQ_HIGH_PRIORITY);
  NVIC_EnableIRQ(PKA_IRQn);  
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* GPIO Ports Clock Enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
  
  /* Initialize GPIO registers */
  GPIO_InitStruct.Pin = LED2_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);

  /* Turn Off Led2 */
  LL_GPIO_SetOutputPin(LED2_GPIO_PORT, LED2_PIN);
}

/**
* @brief RNG Initialization Function
* @param None
* @retval None
*/
static void MX_RNG_Init(void)
{
  /* Peripheral clock enable */
  LL_AHB_EnableClock(LL_AHB_PERIPH_RNG);
  LL_RNG_Enable(RNG);
}

void PKA_ERROR_callback(void)
{
  LED_Blinking(LED_BLINK_ERROR);
}

void PKA_PROCEND_callback(void)
{
  endOfProcess = 1;
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

void GenerateRandomK16Word(uint32_t* buffer)
{
  register uint8_t index = 0;
  
  /* Initialize random numbers generation */
  LL_RNG_Enable(RNG);
  
  /* Generate Random 16bit Numbers */
  
#if (USE_TIMEOUT == 1)
  Timeout = RNG_GENERATION_TIMEOUT;
#endif /* USE_TIMEOUT */
  
  /* Wait for DRDY flag to be raised */
  while (!LL_RNG_IsActiveFlag_RNGRDY(RNG))
  {
#if (USE_TIMEOUT == 1)
    /* Check Systick counter flag to decrement the time-out value */
    if (LL_SYSTICK_IsActiveCounterFlag()) 
    { 
      if(Timeout-- == 0)
      {
        /* Time-out occurred. Set LED to blinking mode */
        LED_Blinking(LED_BLINK_SLOW);
      }
    } 
#endif /* USE_TIMEOUT */
  }
  
  /* Check if error occurs */
  if (  LL_RNG_IsActiveFlag_FAULT(RNG)  )
  {
    /* Clock or Seed Error detected. Set LED to blinking mode (Error type)*/
    LED_Blinking(LED_BLINK_ERROR);
  }
  
  /* Otherwise, no error detected : Value of generated random number could be retrieved
  and stored in dedicated array */
  buffer[index] = LL_RNG_ReadRandData16(RNG);
  
  /* Stop random numbers generation */
  LL_RNG_Disable(RNG);
  
  /* Values of Generated Random numbers are now available in buffer array. */
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
  ex: printf("Wrong parameters value: file %s on line %d", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
