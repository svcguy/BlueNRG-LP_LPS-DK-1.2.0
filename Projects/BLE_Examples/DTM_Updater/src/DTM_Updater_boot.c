/******************** (C) COPYRIGHT 2017 STMicroelectronics ********************
* File Name          : DTM_Updater_boot.h
* Author             : AMS - VMA
* Version            : V1.0.0
* Date               : 19-May-2017
* Description        : BlueNRG Low Level Init function:DON'T MODIFY IT.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#include "bluenrg_lpx.h"
#include "DTM_Updater_Config.h"
#include "DTM_Updater.h"

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05
#define CRITICAL_PRIORITY 0
#if ST_OTA_SERVICE_MANAGER_APPLICATION
#define OTA_VALID_APP_TAG (0xAABBCCDD) /* OTA Service Manager has a special valid tag */
#else
#define OTA_VALID_APP_TAG (0xAA5555AA) 
#endif

#undef BLUE_FLAG_TAG
#define BLUE_FLAG_TAG   (0x00000000)

/* TRIMMING Defines */
#define VALIDITY_TAG 0xFCBCECCC

#define VALIDITY_LOCATION    0x10001EF8
#define TRIMMING_LOCATION    0x10001EE4

#define MAIN_REGULATOR_TRIM_Pos (0)
#define MAIN_REGULATOR_TRIM_Msk (0x0F << MAIN_REGULATOR_TRIM_Pos)
#define SMPS_TRIM_Pos           (4)
#define SMPS_TRIM_Msk           (0x07 << SMPS_TRIM_Pos)
#define LSI_LPMU_TRIM_Pos       (8)
#define LSI_LPMU_TRIM_Msk       (0x0F << LSI_LPMU_TRIM_Pos)
#define LSI_BW_TRIM_Pos         (12)
#define LSI_BW_TRIM_Msk         (0x0F << LSI_BW_TRIM_Pos)
#define HSI_TRIM_Pos            (16)
#define HSI_TRIM_Msk            (0x3F << HSI_TRIM_Pos)

/* Interrupt Vector Table */
#define SYSTEM_CONFIG_SMPS_READY_ERROR 0x01
#define SYSTEM_CONFIG_LSE_READY_ERROR  0x02
#define SYSTEM_CONFIG_LSI_READY_ERROR  0x03
#define SYSTEM_CONFIG_HSE_READY_ERROR  0x04
#define SYSTEM_CONFIG_PLL_READY_ERROR  0x05

//typedef void( *intfunc )( void );

//typedef union { intfunc __fun; void * __ptr; } intvec_elem;

//typedef struct RAM_VR_s  
//{ 
//  uint32_t OTAActivation;
//  uint32_t SavedMSP;
//  uint32_t WakeupFromSleepFlag;
//  uint32_t ResetReason;
//  uint32_t AppBase;
//  uint32_t Reserved[5];
//  uint32_t BlueFlag;
//}RAM_VR_TypeDef;


NORETURN_FUNCTION(void NMI_IRQHandler(void))
{
  while (1);
}
NORETURN_FUNCTION(void HardFault_IRQHandler(void))
{
  while (1);
}
void SVC_IRQHandler(void)
{
  while (1);
}
void PendSV_IRQHandler(void)
{
  while (1);
}
void SysTick_IRQHandler(void)
{
  while (1);
}


SECTION(".intvec")
REQUIRED(const intvec_elem __vector_table[]) = {
    {.__ptr = _INITIAL_SP},                   /* Stack address                      */
    {RESET_HANDLER},           		      /* Reset handler is C initialization. */
    {NMI_IRQHandler},                         /* The NMI handler                    */
    {HardFault_IRQHandler},                   /* The hard fault handler             */
    {(intfunc) OTA_VALID_APP_TAG},            /* OTA Application                    */
    {(intfunc) BLUE_FLAG_TAG},                /* Reserved for blue flag DTM updater */ 
};

extern int main(void);
void __iar_program_start(void) @ "ENTRYPOINT"
{
  asm("CPSID i"); // Disable interrupts.
  main();
}



/* Private macros ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void SmpsTrimConfig(void);

/* Exported variables ---------------------------------------------------------*/
SECTION(".ram_vr")
NO_INIT(REQUIRED(RAM_VR_TypeDef RAM_VR));


/**
  * @brief  SMPS and Trimming value Configuration 
  */
static void SmpsTrimConfig(void)
{
  uint32_t main_regulator, smps_out_voltage, lsi_bw, hsi_calib;
#ifdef CONFIG_DEVICE_BLUENRG_LP
  uint32_t lsi_lpmu;
#endif
  uint8_t eng_lsi_bw_flag;
  
  /* After reset wait until SMPS is ready */
  while(LL_PWR_IsSMPSReady() == 0);
  
  /* Configure SMPS BOM */
#ifdef CONFIG_HW_SMPS_10uH
  LL_PWR_SetSMPSBOM(LL_PWR_SMPS_BOM3);
  /* SMPS clock 4 MHz configuration */
  LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_4);
#else
#ifdef CONFIG_HW_SMPS_2_2uH
  LL_PWR_SetSMPSBOM(LL_PWR_SMPS_BOM2);
  /* SMPS clock 8 MHz configuration  */
  LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_2);
#else
#ifdef CONFIG_HW_SMPS_1_5uH
  LL_PWR_SetSMPSBOM(LL_PWR_SMPS_BOM1);
  /* SMPS clock 8 MHz configuration  */
  LL_RCC_SetSMPSPrescaler(LL_RCC_SMPS_DIV_2);
#else
#ifdef CONFIG_HW_SMPS_NONE
  /* SMPS NONE configuration will be done after the trimming configuration values */
#else
#warning "NO SMPS Configuration!!!"
#endif
#endif
#endif
#endif
  
  /* Retrieve Trimming values from engineering flash locations */
  if (*(volatile uint32_t*)VALIDITY_LOCATION == VALIDITY_TAG) {
    main_regulator    = ((*(volatile uint32_t*)TRIMMING_LOCATION) & MAIN_REGULATOR_TRIM_Msk) >> MAIN_REGULATOR_TRIM_Pos;
    smps_out_voltage  = ((*(volatile uint32_t*)TRIMMING_LOCATION) & SMPS_TRIM_Msk) >> SMPS_TRIM_Pos;
#ifdef CONFIG_DEVICE_BLUENRG_LP
    lsi_lpmu          = ((*(volatile uint32_t*)TRIMMING_LOCATION) & LSI_LPMU_TRIM_Msk) >> LSI_LPMU_TRIM_Pos;
#endif /* CONFIG_DEVICE_BLUENRG_LP */
    lsi_bw            = ((*(volatile uint32_t*)TRIMMING_LOCATION) & LSI_BW_TRIM_Msk) >> LSI_BW_TRIM_Pos;
    hsi_calib         = ((*(volatile uint32_t*)TRIMMING_LOCATION) & HSI_TRIM_Msk) >> HSI_TRIM_Pos;
    eng_lsi_bw_flag   = TRUE;
  } else {
#ifdef CONFIG_DEVICE_BLUENRG_LP
    main_regulator    = 0x08;
    lsi_lpmu          = 0x08;
    hsi_calib         = 0x1E;
    eng_lsi_bw_flag   = FALSE;
#endif
#ifdef CONFIG_DEVICE_BLUENRG_LPS
    main_regulator    = 0x03;
    hsi_calib         = 0x1D;
    lsi_bw            = 8;
    eng_lsi_bw_flag   = TRUE;
#endif
    smps_out_voltage  = 0x03;
  }
  
  /* Set HSI Calibration Trimming value */
  LL_RCC_HSI_SetCalibTrimming(hsi_calib);
  
  /* Low speed internal RC trimming value set by software */
  if (eng_lsi_bw_flag)
    LL_RCC_LSI_SetTrimming(lsi_bw);
  
#ifdef CONFIG_DEVICE_BLUENRG_LP
  /* Set LSI LPMU Trimming value */
  LL_PWR_SetLSILPMUTrim(lsi_lpmu);
#endif /* CONFIG_DEVICE_BLUENRG_LP */
  
  /* Set Main Regulator voltage Trimming value */ 
  LL_PWR_SetMRTrim(main_regulator);
  
  /* Set SMPS output voltage Trimming value */
  LL_PWR_SetSMPSTrim(smps_out_voltage);
  
  /* Set SMPS in LP Open */
  LL_PWR_SetSMPSOpenMode(LL_PWR_SMPS_LPOPEN);
  
#ifdef CONFIG_HW_SMPS_NONE
  /* No SMPS configuration */
  LL_PWR_SetSMPSMode(LL_PWR_NO_SMPS);
#endif
  
}


void DTM_SystemInit(void)
{
  /* Remap the vector table */
  LL_FLASH_SetVectTable(FLASH, LL_FLASH_CONFIG_IRQ_VECT_TABLE_FLASH);
  
  /* Vector Table Offset Register */
  SCB->VTOR = (uint32_t) (__vector_table);
  
  /* Enable all the RAM banks in retention during DEEPSTOP */
  LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_1);
#if defined(LL_PWR_RAMRET_2)
  LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_2);
#endif
#if defined(LL_PWR_RAMRET_3)
  LL_PWR_EnableRAMBankRet(LL_PWR_RAMRET_3);
#endif
  
  /* HW SMPS and HW Trimming value Configuration */
  SmpsTrimConfig();
  
  /* Low Speed Crystal Configuration */
  LL_RCC_LSCO_SetSource(LL_RCC_LSCO_CLKSOURCE_LSI);
  LL_RCC_LSI_Enable();
  while (LL_RCC_LSI_IsReady() == 0U);

  /* System Clock Configuration */
  LL_RCC_HSE_Enable();
  while (LL_RCC_HSE_IsReady() == 0U);
  
  LL_RCC_SetRC64MPLLPrescaler(LL_RCC_RC64MPLL_DIV_2);
  
  LL_RCC_RC64MPLL_Enable();
  while(LL_RCC_RC64MPLL_IsReady() == 0U);
  
  LL_FLASH_SetWaitStates(FLASH, LL_FLASH_WAIT_STATES_0);
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
