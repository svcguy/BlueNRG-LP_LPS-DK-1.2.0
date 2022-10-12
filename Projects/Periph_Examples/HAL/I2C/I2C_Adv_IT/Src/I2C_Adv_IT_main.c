
/******************** (C) COPYRIGHT 2021 STMicroelectronics ********************
* File Name          : I2C_Adv_IT_main.c
* Author             : RF Application Team
* Version            : 1.0.0
* Date               : 04-March-2019
* Description        : Code demonstrating the I2C functionality
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file  I2C_Adv_IT/I2C_Adv_IT_main.c
 * @brief How to handle I2C data buffer transmission/reception between two boards, using an interrupt starting a new communication transfer after the first.
 *

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\I2C\\I2C_Adv_IT\\MDK-ARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\I2C_Adv_IT.uvprojx</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG-LP Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\I2C\\I2C_Adv_IT\\EWARM\\{STEVAL-IDB011V1|STEVAL-IDB012V1}\\I2C_Adv_IT.eww</tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \section WISE_project WiSE-Studio project
  To use the project with WiSE-Studio IDE (GCC toolchain), please follow the instructions below:
  -# Open the WiSE-Studio IDE
  -# Select File, Import, Existing Projects into Workspace
     <tt>C:\\Users\\{username}\\ST\\BlueNRG-LP_LPS DK x.x.x\\Projects\\Periph_Examples\\HAL\\I2C\\I2C_Adv_IT\\WiSE-Studio\\{STEVAL-IDB011V1|STEVAL-IDB012V1}</tt> 
  -# Select desired configuration to build
  -# Select Project->Build Project. This will recompile and link the entire application
  -# To download the binary image, please connect an USB cable in your board (CMSIS-DAP upgrade).
  -# Select Run->Run/Debug to download the related binary image.
  -# Alternatively, open the Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Master_board - Master configuration
- \c Slave_board - Slave configuration


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
|     A13    |       I2C2 SCL      |        N.A.        |
|     A14    |       I2C2 SDA      |        N.A.        |
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
|     B7     |       Not Used      |      I2C2 SDA      |
|     B8     |         DL2         |        N.A.        |
|     B9     |         DL3         |        N.A.        |
|     A0     |         N.A.        |      Not Used      |
|     A10    |         N.A.        |      Not Used      |
|     B1     |         N.A.        |      Not Used      |
|     B6     |         N.A.        |      I2C2 SCL      |
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
|            |                                                    Master_board                                                     |||                          Slave_board                          |||
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|  LED name  |            STEVAL-IDB011V1           |            STEVAL-IDB011V2           |            STEVAL-IDB012V1           |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|     DL1    |               Not Used               |               Not Used               |               Not Used               |      Not Used      |      Not Used      |      Not Used      |
|     DL2    |   Blinking: waiting for User action  |   Blinking: waiting for User action  |   Blinking: waiting for User action  |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |               ON: error              |               ON: error              |               ON: error              |      ON: error     |      ON: error     |      ON: error     |
|     DL4    |               Not Used               |               Not Used               |               Not Used               |      Not Used      |      Not Used      |      Not Used      |
|     U5     |               Not Used               |               Not Used               |               Not Used               |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|                |                                                 Master_board                                                  |||                          Slave_board                          |||
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|   BUTTON name  |           STEVAL-IDB011V1          |           STEVAL-IDB011V2          |           STEVAL-IDB012V1          |   STEVAL-IDB011V1  |   STEVAL-IDB011V2  |   STEVAL-IDB012V1  |
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |   Master starts the communication  |   Master starts the communication  |   Master starts the communication  |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |              Not Used              |              Not Used              |              Not Used              |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |          Reset BlueNRG-LP          |          Reset BlueNRG-LP          |          Reset BlueNRG-LP          |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |  Reset BlueNRG-LP  |

@endtable

* \section Usage Usage

How to handle I2C data buffer transmission/reception between two boards, using an interrupt starting a new communication transfer after the first.

At the beginning of the main program the HAL_Init() function is called to reset all the peripherals, initialize the Flash interface and the systick.

The I2C peripheral configuration is ensured by the HAL_I2C_Init() function.
This later is calling the HAL_I2C_MspInit() function which core is implementing the configuration of the needed I2C resources according to the used hardware (CLOCK, GPIO and NVIC). You may update this function to change I2C configuration.

The I2C communication is then initiated.
The project is split in two parts: the Master Board and the Slave Board
- Master Board
  The HAL_I2C_Master_Receive_IT() and the HAL_I2C_Master_Transmit_IT() functions 
  allow respectively the reception and the transmission of a predefined data buffer
  in Master mode using interrupt.
- Slave Board
  The HAL_I2C_Slave_Receive_IT() and the HAL_I2C_Slave_Transmit_IT() functions 
  allow respectively the reception and the transmission of a predefined data buffer
  in Slave mode using interrupt.
The user can choose between Master and Slave through "#define MASTER_BOARD" in the "I2C_Adv_IT_main.c" file:
If the Master board is used, the "#define MASTER_BOARD" must be uncommented.
If the Slave board is used the "#define MASTER_BOARD" must be commented.

For this example two buffers are used 
- aTxBuffer buffer contains the data to be transmitted 
- aRxBuffer buffer is used to save the received data
Note that both buffers have same size
                       
On Master board side:
 - Wait User push-button (PUSH1) to be pressed (used for only synchronization at startup)
 - Request write operation by sending specific command "MASTER_REQ_WRITE" to Slave
 - Send the number of data to be written
 - Transmit aTxBuffer buffer to slave
 - Request read operation by sending specific command "MASTER_REQ_READ" to Slave
 - Send the number of data to be read
 - Receive data from Slave in aRxBuffer
 - Check the correctness of data and Toggle LED2 (using infinite loop, due to the continuous 
   transmission process) when data is received correctly, otherwise LED3 is turned ON and 
   communication is stopped (using infinite loop)


On Slave board side:
 - Receive request from Master
 - Receive the request operation from Master and depending on the operation type (write or read):
   - If Master requests write operation:
      - Receive number of data to be written by Master
      - Receive data from master in aRxBuffer
      - Check the correctness of data and Toggle LED2 when data is received correctly,
        otherwise LED3 is turned ON and communication is stopped (using infinite loop)
   - If Master request read operation:
      - Receive number of data to be written to Master
      - Transmit aTxBuffer buffer to master
   
These operations are repeated periodically and the start of communication is triggered by pushing the key button of the Master board. 

In order to make the program work, you must do the following : 
 -Rebuild all files and load your image into target memory.
    Uncomment "#define MASTER_BOARD" and load the project in Master Board
    Comment   "#define MASTER_BOARD" and load the project in Slave Board
 - Launch serial communication SW on PC
 - Flash the project in the Board
 - Press the RESET button
 
BlueNRG_LP-EVB Set-up
- Connect I2C_SCL line of Master board to I2C_SCL line of Slave Board.
- Connect I2C_SDA line of Master board to I2C_SDA line of Slave Board.
- Connect GND of Master board to GND of Slave Board.
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
#include "I2C_Adv_IT_main.h"

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2cx;

/* Configure the SDA setup, hold time and the SCL high, low period */
/* (uint32_t)0x10320309 = I2C TIMING in Fast Mode                  */
/* (uint32_t)0x00200204 = I2C TIMING in Fast Mode plus             */

/* Configure the SDA setup, hold time and the SCL high, low period 
* For Fast-mode     kHz, PRESC | 0h | SCLDEL | SDADEL | SCLH | SCLL 
*                          1h  | 0h |    3h  |   2h   |  03h |  09h 
* timing = __LL_I2C_CONVERT_TIMINGS(0x01, 0x03, 0x02, 0x03, 0x09);
*/ 

/* Configure the SDA setup, hold time and the SCL high, low period 
* For Fast-mode +   kHz, PRESC | 0h | SCLDEL | SDADEL | SCLH | SCLL 
*                          0h  | 0h |    2h  |   0h   |  02h |  04h 
*  timing = __LL_I2C_CONVERT_TIMINGS(0x00, 0x02, 0x00, 0x02, 0x04);
*/ 

/* I2C TIMING in Fast Mode Plus */
__IO uint32_t timing = __LL_I2C_CONVERT_TIMINGS(0x01, 0x03, 0x02, 0x03, 0x09);

/* Buffer used for transmission */
uint8_t aTxBuffer[] = " ****I2C_TwoBoards communication based on IT****  ****I2C_TwoBoards communication based on IT****  ****I2C_TwoBoards communication based on IT**** ";

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];
__IO uint16_t hTxNumData = 0;
__IO uint16_t hRxNumData = 0;
uint8_t bTransferRequest = 0;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2Cx_Init(void);

/* Private function prototypes -----------------------------------------------*/
static uint16_t Buffercmp(uint8_t *pBuffer1, uint8_t *pBuffer2, uint16_t BufferLength);
static void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength);

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
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2Cx_Init();
  
  /* Configure LED2 and LED3 */
  BSP_LED_Init(BSP_LED2);
  BSP_LED_Init(BSP_LED3);
  
  /* ****************************************************************************
  MASTER SECTION
  */
  
#ifdef MASTER_BOARD
  printf("Master board\n\r");
  /* Configure User push-button (PUSH1) */
  BSP_PB_Init(BSP_PUSH1,BUTTON_MODE_GPIO);
#else
  printf("Slave board\n\r");
#endif /* MASTER_BOARD */
  
  /* Infinite loop */
  
  while (1)
  {
#ifdef MASTER_BOARD
    /* Wait for User push-button (PUSH1) press before starting the Communication */
    printf("Wait for User push-button (PUSH1) press before starting the Communication.\n\r");
    while (BSP_PB_GetState(BSP_PUSH1) == GPIO_PIN_RESET)
    {
      BSP_LED_Toggle(BSP_LED2);
      HAL_Delay(200);
    }
    BSP_LED_Off(BSP_LED2);
    
    /* Initialize number of data variables */
    hTxNumData = TXBUFFERSIZE;
    hRxNumData = RXBUFFERSIZE;
    
    /* Update bTransferRequest to send buffer write request for Slave */
    bTransferRequest = MASTER_REQ_WRITE;
    
    /* Master sends write request for slave */
    printf("Master sends write request for slave.\n\r");
    do
    {
      if(HAL_I2C_Master_Transmit_IT(&hi2cx, (uint16_t)I2C_ADDRESS, (uint8_t*)&bTransferRequest, 1)!= HAL_OK)
      {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      
      /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      } 
      
      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */
    }
    while(HAL_I2C_GetError(&hi2cx) == HAL_I2C_ERROR_AF);
    
    /* Master sends number of data to be written */
    printf("Master sends number of data to be written.\n\r");
    do
    {
      if(HAL_I2C_Master_Transmit_IT(&hi2cx, (uint16_t)I2C_ADDRESS, (uint8_t*)&hTxNumData, 2)!= HAL_OK)
      {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      
      /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      } 
      
      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */
    }
    while(HAL_I2C_GetError(&hi2cx) == HAL_I2C_ERROR_AF);
    
    /* Master sends aTxBuffer to slave */
    printf("Master sends aTxBuffer to slave.\n\r");
    do
    {
      if(HAL_I2C_Master_Transmit_IT(&hi2cx, (uint16_t)I2C_ADDRESS, (uint8_t*)aTxBuffer, TXBUFFERSIZE)!= HAL_OK)
      {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      
      /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      } 
      
      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */
    }
    while(HAL_I2C_GetError(&hi2cx) == HAL_I2C_ERROR_AF);
    
    /* Update bTransferRequest to send buffer read request for Slave */
    bTransferRequest = MASTER_REQ_READ;
    
    /* Master sends read request for slave */
    printf("Master sends read request for slave.n\r");
    do
    {
      if(HAL_I2C_Master_Transmit_IT(&hi2cx, (uint16_t)I2C_ADDRESS, (uint8_t*)&bTransferRequest, 1)!= HAL_OK)
      {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      
      /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      } 
      
      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */
    }
    while(HAL_I2C_GetError(&hi2cx) == HAL_I2C_ERROR_AF);
    
    /* Master sends number of data to be read */
    printf("Master sends number of data to be read.\n\r");
    do
    {
      if(HAL_I2C_Master_Transmit_IT(&hi2cx, (uint16_t)I2C_ADDRESS, (uint8_t*)&hRxNumData, 2)!= HAL_OK)
      {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      
      /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      } 
      
      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */
    }
    while(HAL_I2C_GetError(&hi2cx) == HAL_I2C_ERROR_AF);
    
    /* Master receives aRxBuffer from slave */
    printf("Master receives aRxBuffer from slave.\n\r");
    do
    {
      if(HAL_I2C_Master_Receive_IT(&hi2cx, (uint16_t)I2C_ADDRESS, (uint8_t*)aRxBuffer, RXBUFFERSIZE)!= HAL_OK)
      {
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
      }
      
      /*  Before starting a new communication transfer, you need to check the current   
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the 
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */  
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      } 
      
      /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
      Master restarts communication */
    }
    while(HAL_I2C_GetError(&hi2cx) == HAL_I2C_ERROR_AF);
    
    /* Check correctness of received buffer */
    printf("Check correctness of received buffer\n\r");
    if(Buffercmp((uint8_t*)aTxBuffer,(uint8_t*)aRxBuffer,hRxNumData))
    {
      /* Processing Error */
      printf("Error\n\r");
      Error_Handler();
    }
    else
    {
      printf("OK\n\r");
      HAL_Delay(1000);
    }
    
    /* Flush Rx buffers */
    Flush_Buffer((uint8_t*)aRxBuffer,RXBUFFERSIZE);
    
    
#else /* SLAVE_BOARD */
    /* ****************************************************************************
    SLAVE SECTION
    */
    /* Initialize number of data variables */
    hTxNumData = 0;
    hRxNumData = 0;
    
    /* Slave receive request from master */
    printf("Slave receive request from master.\n\r");
    while(HAL_I2C_Slave_Receive_IT(&hi2cx, (uint8_t*)&bTransferRequest, 1)!= HAL_OK)
    {
    }
    
    /*  Before starting a new communication transfer, you need to check the current
    state of the peripheral; if it’s busy you need to wait for the end of current
    transfer before starting a new one.
    For simplicity reasons, this example is just waiting till the end of the
    transfer, but application may perform other tasks while transfer operation
    is ongoing. */
    while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
    {
    }
    
    /* If master request write operation */
    if (bTransferRequest == MASTER_REQ_WRITE)
    {
      /* Slave receive number of data to be read */
      printf("Slave receive number of data to be read.\n\r");
      while(HAL_I2C_Slave_Receive_IT(&hi2cx, (uint8_t*)&hRxNumData, 2)!= HAL_OK);
      
      /*  Before starting a new communication transfer, you need to check the current
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      }
      
      /* Slave receives aRxBuffer from master */
      printf("Slave receives aRxBuffer from master.\n\r");
      while(HAL_I2C_Slave_Receive_IT(&hi2cx, (uint8_t*)aRxBuffer, hRxNumData)!= HAL_OK);
      
      /*  Before starting a new communication transfer, you need to check the current
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      }
      
      /* Check correctness of received buffer */
      printf("Check correctness of received buffer.\n\r");
      if(Buffercmp((uint8_t*)aTxBuffer,(uint8_t*)aRxBuffer,hRxNumData))
      {
        /* Processing Error */
        printf("Error.\n\r");
        Error_Handler();
      }
      else
      {
        printf("Ok\n\r");
      }
      
      /* Flush Rx buffers */
      Flush_Buffer((uint8_t*)aRxBuffer,RXBUFFERSIZE);
    }
    /* If master request write operation */
    else
    {
      /* Slave receive number of data to be written */
      printf("Slave receive number of data to be written.\n\r");
      while(HAL_I2C_Slave_Receive_IT(&hi2cx, (uint8_t*)&hTxNumData, 2)!= HAL_OK);
      
      /*  Before starting a new communication transfer, you need to check the current
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      }
      
      /* Slave transmit aTxBuffer to master */
      printf("Slave transmit aTxBuffer to master.\n\r");
      while(HAL_I2C_Slave_Transmit_IT(&hi2cx, (uint8_t*)aTxBuffer, RXBUFFERSIZE)!= HAL_OK);
      
      /*  Before starting a new communication transfer, you need to check the current
      state of the peripheral; if it’s busy you need to wait for the end of current
      transfer before starting a new one.
      For simplicity reasons, this example is just waiting till the end of the
      transfer, but application may perform other tasks while transfer operation
      is ongoing. */
      while (HAL_I2C_GetState(&hi2cx) != HAL_I2C_STATE_READY)
      {
      }
    }
    
#endif /* MASTER_BOARD */
    
  }
}

/**
* @brief I2Cx Initialization Function
* @param None
* @retval None
*/
static void MX_I2Cx_Init(void)
{
  hi2cx.Instance = I2Cx;
  hi2cx.Init.Timing = timing;
  hi2cx.Init.OwnAddress1 = I2C_ADDRESS;
  hi2cx.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2cx.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2cx.Init.OwnAddress2 = 0;
  hi2cx.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2cx.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2cx.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2cx) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Analogue filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2cx, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2cx, 0) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** I2C Enable Fast Mode Plus */
  __HAL_RCC_SYSCFG_CLK_ENABLE() ;
  HAL_I2CEx_EnableFastModePlus(SYSCFG_FASTMODEPLUS_PA0);
  HAL_I2CEx_EnableFastModePlus(SYSCFG_FASTMODEPLUS_PA1);
  
  
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
* @brief  Compares two buffers.
* @param  pBuffer1, pBuffer2: buffers to be compared.
* @param  BufferLength: buffer's length
* @retval 0  : pBuffer1 identical to pBuffer2
*         >0 : pBuffer1 differs from pBuffer2
*/
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
  
  return 0;
}

/**
* @brief  Flushes the buffer
* @param  pBuffer: buffers to be flushed.
* @param  BufferLength: buffer's length
* @retval None
*/
static void Flush_Buffer(uint8_t* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    *pBuffer = 0;
    
    pBuffer++;
  }
}

/**
* @brief  I2C error callbacks.
* @param  I2cHandle: I2C handle
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
  /** Error_Handler() function is called when error occurs.
  * 1- When Slave don't acknowledge it's address, Master restarts communication.
  * 2- When Master don't acknowledge the last data transferred, Slave don't care in this example.
  */
  if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
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
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



