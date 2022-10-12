/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
* File Name          : BLE_Beacon_main.c
* Author             : RF Application Team 
* Version            : V1.0.0
* Date               : 12-June-2019
* Description        : NUCLEO-L152RE network coprocessor main file for beacon demo
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/
/** @addtogroup BlueNRGLP_demonstrations_applications
*  BlueNRG-LP Beacon demo \see BLE_Beacon_main.c for documentation.
*
*@{
*/

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
*/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "hal_types.h"
#include "hci.h"
#include "osal.h"
#include "hci_const.h"
#include "bluenrg_lp_types.h"
#include "bluenrg_lp_gap.h"
#include "bluenrg_lp_aci.h"
#include "bluenrg_lp_hci_le.h"
#include "SDK_EVAL_Config.h"


/* External variables --------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define BLE_BEACON_VERSION_STRING "1.0.0"

/** 
* @brief  Enable debug printf
*/ 
#ifndef DEBUG
#define DEBUG 0
#endif

/* Set to 1 for enabling Flags AD Type position at the beginning 
of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

/* Private macros ------------------------------------------------------------*/
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
  /* Set AD Type Flags at beginning on Advertising packet  */
static uint8_t adv_data[] = {
  /* Advertising data: Flags AD Type */
  0x02, 
  0x01, 
  0x06, 
  /* Advertising data: manufacturer specific data */
  26, //len
  AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
  0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
  0x02,       // ID
  0x15,       //Length of the remaining payload
  0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
  0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
  0x00, 0x00, // Major number 
  0x00, 0x07, // Minor number 
  (uint8_t)-56,         // Tx power measured at 1 m of distance
};

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void SystemInit_NWK(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 48000000  32000000
  *            HCLK(Hz)                       = 48000000  32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLLMUL                         = 12 6
  *            PLLDIV                         = 4  3
  *            Flash Latency(WS)              = 1
  * @retval None
  */
void SystemClock_Config(void)
{
#if (HCLK_32MHZ == 1)
  /* Enable ACC64 access and set FLASH latency */
  
#ifdef STM32L152xE
  LL_FLASH_Enable64bitAccess();
#endif
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  LL_RCC_HSI_SetCalibTrimming(0x10);

  /* Enable HSI if not already activated*/
  if (LL_RCC_HSI_IsReady() == 0)
  {
    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };
  }
  
  /* Main PLL configuration and activation */
#ifdef STM32L476xx
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 10, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
#endif
#ifdef STM32L152xE
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_12, LL_RCC_PLL_DIV_4); // LL_RCC_PLL_MUL_6, LL_RCC_PLL_DIV_3);
#endif
  
  LL_RCC_PLL_Enable();
  while(LL_RCC_PLL_IsReady() != 1) 
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) 
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  
#ifdef STM32L476xx
  SysTick_Config(80000000/1000);
  LL_SetSystemCoreClock(80000000);
#endif
#ifdef STM32L152xE
  SysTick_Config(48000000/1000);   // 32000000/1000
  LL_SetSystemCoreClock(48000000); // 32000000
#endif
#endif
  
  
#if (SYSCLK_MSI == 1)
  /* Set FLASH latency */ 
#ifdef STM32L152xE
  LL_FLASH_Enable64bitAccess();
#endif
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  /* Set Voltage scale1 as MCU will run at 32MHz */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (LL_PWR_IsActiveFlag_VOSF() != 0)
  {
  };
  
  /* Set MSI Range */
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  
  /* Enable MSI if not already activated*/
  if (LL_RCC_MSI_IsReady() == 0)
  {
    /* MSI configuration and activation */
    LL_RCC_MSI_Enable();
    while(LL_RCC_MSI_IsReady() != 1)
    {
    };
  }
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  RCC_HCLKConfig(RCC_CFGR_HPRE_DIV1);

  SysTick_Config(4000000/1000);
  LL_SetSystemCoreClock(4000000);
#endif
  
}


static Advertising_Set_Parameters_t Advertising_Set_Parameters[1];

static void Device_Init(void)
{
  uint8_t ret;
  uint16_t service_handle;
  uint16_t dev_name_char_handle;
  uint16_t appearance_char_handle;
  uint8_t bdaddr[] = {0xf5, 0x00, 0x00, 0xE1, 0x80, 0x02};
  
  /* Set the TX Power to 0 dBm */
  ret = aci_hal_set_tx_power_level(0, 24);
  if(ret != 0) {
    PRINTF ("Error in aci_hal_set_tx_power_level() 0x%04x\r\n", ret);
    while(1);
  }
  else
    printf ("aci_hal_set_tx_power_level() --> SUCCESS\r\n");
  
  /* Set the public address */
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
  if(ret != 0) {
    PRINTF ("Error in aci_hal_write_config_data() 0x%04x\r\n", ret);
    while(1);
  }
  else
    printf ("aci_hal_write_config_data() --> SUCCESS\r\n");
  
  /* Init the GATT */
  ret = aci_gatt_srv_init();    
  if(ret != 0) {
    PRINTF ("Error in aci_gatt_srv_init() 0x%04x\r\n", ret);
    while(1);
  }
  else
    printf ("aci_gatt_srv_init() --> SUCCESS\r\n");
  
  /* Init the GAP */
  ret = aci_gap_init(GAP_PERIPHERAL_ROLE, 0, 0x08, 0, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  if(ret != 0) {
    PRINTF ("Error in aci_gap_init() 0x%04xr\n", ret);
    while(1);
  }
  else
    printf ("aci_gap_init() --> SUCCESS\r\n");
}


/* Event used to notify the Host that a hardware failure has occurred in the Controller. 
   See bluenrg_lp_events.h. */
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  if (Hardware_Code <= 0x03)
  {
    NVIC_SystemReset();
  }
}

/**
* @brief  Start beaconing
* @param  None 
* @retval None
*/
static void Start_Beaconing(void)
{  
 uint8_t ret = BLE_STATUS_SUCCESS;
  
  ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                              ADV_PROP_LEGACY,
                                              160, 160,
                                              ADV_CH_ALL,
                                              0,NULL,
                                              ADV_NO_WHITE_LIST_USE,
                                              0, /* 0 dBm */
                                              LE_1M_PHY, /* Primary advertising PHY */
                                              0, /* 0 skips */
                                              LE_1M_PHY, /* Secondary advertising PHY. Not used with legacy advertising. */
                                              0, /* SID */
                                              0 /* No scan request notifications */);  
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_advertising_configuration() 0x%04x\r\n", ret);
    return;
  }
  
  ret = aci_gap_set_advertising_data_nwk(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_advertising_data_nwk() 0x%04x\r\n", ret);
    return;
  }
  
  Advertising_Set_Parameters[0].Advertising_Handle = 0;
  Advertising_Set_Parameters[0].Duration = 0;
  Advertising_Set_Parameters[0].Max_Extended_Advertising_Events = 0;
  
   /* Enable advertising */
  ret = aci_gap_set_advertising_enable(ENABLE, 1, Advertising_Set_Parameters);
  if (ret != BLE_STATUS_SUCCESS)
  {
    printf ("Error in aci_gap_set_advertising_enable() 0x%04x\r\n", ret);
    return;
  }
  else
    printf ("aci_gap_set_advertising_enable() --> SUCCESS\r\n");
}


/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  uint8_t ret;
  
  /* Device Initialization */
  SystemInit_NWK();
  
  /* BlueNRG-LP stack init */
  ret = BlueNRG_Stack_Initialization();
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
    while(1);
  }
  
  /* Init the BlueNRG-LP device */
  Device_Init(); 
  
  /* Start Beacon Non Connectable Mode*/
  Start_Beaconing();
  
  printf("BlueNRG-LP BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING); 
  
  /* Infinite loop */
  while(1) {     
    BTLE_StackTick();
  }
}


void SystemInit_NWK(void)
{
  /* Configure the system clock to 48 MHz */
  SystemClock_Config();
  
  /* Configure the BlueNRG-LP pins - RESET pin */
  Sdk_Eval_Reset_Pin_Init();
  
#ifdef SPI_INTERFACE
  /* Init SPI interface */
  SdkEvalSpiInit();
#else

#ifdef UART_INTERFACE
  DTM_IO_Config();
#endif
#endif

  /* Init the UART peripheral */
  SdkEvalComUartInit(UART_BAUDRATE);
  
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
*/
