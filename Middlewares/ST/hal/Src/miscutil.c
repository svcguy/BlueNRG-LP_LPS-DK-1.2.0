/**
******************************************************************************
* @file    miscutil.c 
* @author  AMS - RF Application Team
* @version V1.0.0
* @date    3-April-2019
* @brief   Miscellaneous utilities for radio HW
******************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
******************************************************************************
*/ 
/* Includes ------------------------------------------------------------------*/
#include "rf_driver_ll_system.h"
#include "rf_driver_ll_utils.h"
#include "rf_driver_ll_bus.h"
#include "system_BlueNRG_LP.h"
#include "hal_miscutil.h"
#include "bleplat.h"
#include "ble_status.h"
#include "rf_driver_ll_gpio.h"
#include "ble_controller.h"
#include "miscutil.h"

/** @addtogroup BlueNRG_LP_Miscellaneous_Utilities
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define TX_POWER_LEVELS                (32U)

#define LOWEST_TX_POWER_LEVEL_INDEX     (1U)

/** Minimum supported TX power in dBm. */
#define MIN_TX_POWER_LOW  (normal_pa_level_table[LOWEST_TX_POWER_LEVEL_INDEX]) /* high power mode disabled */
#define MIN_TX_POWER_HIGH (high_power_pa_level_table[LOWEST_TX_POWER_LEVEL_INDEX]) /* high power mode enabled */

/** Maximum supported TX power in dBm. */
#define MAX_TX_POWER_LOW  (normal_pa_level_table[TX_POWER_LEVELS-1]) /* high power mode disabled */
#define MAX_TX_POWER_HIGH (high_power_pa_level_table[TX_POWER_LEVELS-1]) /* high power mode enabled */

/* Parameters of the RSSI Exponential Moving Average algorithm */ /* @todo: review */
#define MAX_RSSI_FILTER_COEFF       (4U)
#define RSSI_EMA_SMOOTH_FACTOR_BITS (3)

/* Parameters of the RSSI calculation algorithm */
#define RSSI_OFFSET  (118)

#ifndef ANTENNA_ID_BIT_SHIFT
#define ANTENNA_ID_BIT_SHIFT (0)
#endif

/* This macro can be set to avoid breaking communication by changing the function
  of pins with aci_hal_set_antenna_switch_parameters().  */
#define RESERVED_GPIOS  0x00 


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*---------------------------------------------------------------------------*/

/**
 * @brief Get Device ID, Version and Revision numbers
 */
void BLEPLAT_get_part_info(uint8_t *device_id, uint8_t *major_cut, uint8_t *minor_cut)
{
   PartInfoType partInfo;
   
   /* get partInfo */
   HAL_GetPartInfo(&partInfo);
  
  /* Set device ID */
  *device_id  = partInfo.die_id;
  
  /* Set major cut  */
  *major_cut = partInfo.die_major; 
 
  /* Set minor cut */
  *minor_cut = partInfo.die_cut;
}

/* Expected TX output power (dBm) for each PA level when SMPS voltage is 1.4V */
const int8_t normal_pa_level_table[TX_POWER_LEVELS] = {
    -54, -21, -20, -19, -17, -16, -15, -14,
    -13, -12, -11, -10,  -9,  -8,  -7,  -6,
     -6,  -4,  -3,  -3,  -2,  -2,  -1,  -1,
      0,   0,   1,   2,   3,   4,   5,   6
};

/* Expected TX output power (dBm) for each PA level when SMPS voltage is 1.9V
   (high power mode). */
const int8_t high_power_pa_level_table[TX_POWER_LEVELS] = {
    -54, -19, -18, -17, -16, -15, -14, -13,
    -12, -11, -10,  -9,  -8,  -7,  -6,  -5,
     -4,  -3,  -3,  -2,  -1,   0,   1,   2,
      3,   8,   8,   8,   8,   8,   8,   8
};

uint8_t BLEPLAT_DBmToPALevel(int8_t TX_dBm, uint8_t high_power)
{
  uint8_t i;
  const int8_t *pa_level_table = high_power?high_power_pa_level_table:normal_pa_level_table;
  
  for(i = 0; i < TX_POWER_LEVELS; i++)
  {
    if(pa_level_table[i] >= TX_dBm)
      break;
  }
  if (((pa_level_table[i] > TX_dBm) && (i > LOWEST_TX_POWER_LEVEL_INDEX)) ||
      (i == TX_POWER_LEVELS))
  {
    i--;
  }
  
  return i;  
}

uint8_t BLEPLAT_DBmToPALevelGe(int8_t TX_dBm, uint8_t high_power)
{
    const int8_t *pa_level_table = high_power ? high_power_pa_level_table : normal_pa_level_table;
    uint8_t i;
    
    for(i = LOWEST_TX_POWER_LEVEL_INDEX; i < TX_POWER_LEVELS; i++)
    {
        if (pa_level_table[i] >= TX_dBm)
            break;
    }
    
    if(i == TX_POWER_LEVELS)
    {
        i--;
    }
    
    return i;  
}

int8_t BLEPLAT_PALevelToDBm(uint8_t PA_Level, uint8_t high_power)
{
  const int8_t *pa_level_table = high_power?high_power_pa_level_table:normal_pa_level_table;
  
  if(PA_Level < LOWEST_TX_POWER_LEVEL_INDEX || PA_Level >= TX_POWER_LEVELS)
  {
    return 127;
  }
  
  return pa_level_table[PA_Level];
}

void BLEPLAT_ReadTransmitPower(uint8_t high_power, int8_t *Min_Tx_Power, int8_t *Max_Tx_Power)
{
    if (high_power)
    {
        *Min_Tx_Power = MIN_TX_POWER_HIGH;
        *Max_Tx_Power = MAX_TX_POWER_HIGH;
    }
    else
    {
        *Min_Tx_Power = MIN_TX_POWER_LOW;
        *Max_Tx_Power = MAX_TX_POWER_LOW;
    }
}

void BLEPLAT_SetHighPower(uint8_t enable)
{
  HAL_SetHighPower((FunctionalState)enable);
}

void BLEPLAT_RadioControllerReset(void)
{
  LL_APB2_ForceReset(LL_APB2_PERIPH_MRBLE);
  LL_APB2_ReleaseReset(LL_APB2_PERIPH_MRBLE);
  MrBleBiasTrimConfig(FALSE); // Restore configuration, lost after controller reset.  
}

void BLEPLAT_GetRawRSSIRegs(uint32_t *rssi_reg, uint32_t *agc_reg)
{
    volatile uint32_t rssi0 = RRM->RSSI0_DIG_OUT;
    volatile uint32_t rssi1 = RRM->RSSI1_DIG_OUT;
    
    *rssi_reg  = rssi0 & 0xFFu;
    *rssi_reg |= (rssi1 & 0xFFu) << 8;
    
    *agc_reg   = RRM->AGC_DIG_OUT;
}

int8_t BLEPLAT_CalculateRSSI(void)
{
    int32_t rssi_dbm;
    uint32_t rssi;
    uint32_t agc;
    
    BLEPLAT_GetRawRSSIRegs(&rssi, &agc);
    
    if ((rssi == 0U) || (agc > 0xbU))
    {
        rssi_dbm = RSSI_INVALID;
    }
    else
    {
        rssi_dbm = (int32_t)agc * 6 - RSSI_OFFSET;
        while (rssi > 30U)
        {
            rssi_dbm += 6;
            rssi >>= 1;
        }
        rssi_dbm += (int32_t)(uint32_t)((417U * rssi + 18080U) >> 10);
    }
    
    return (int8_t)rssi_dbm;
}

/* @todo: review with the use of linear values instead of dBm values to have more precision */
const int8_t rssi_ema_smoothing_factor_table[MAX_RSSI_FILTER_COEFF + 1] = {
    7, 5, 3, 2, 1
};

int8_t BLEPLAT_UpdateAvgRSSI(int8_t avg_rssi, int8_t rssi, uint8_t rssi_filter_coeff)
{
    if (avg_rssi == RSSI_INVALID)
    {
        return rssi;
    }
    
    if ((rssi == RSSI_INVALID) || (rssi_filter_coeff > MAX_RSSI_FILTER_COEFF))
    {
        return avg_rssi;
    }
    
    return (avg_rssi +
            (((rssi - avg_rssi) * rssi_ema_smoothing_factor_table[rssi_filter_coeff])
             >> RSSI_EMA_SMOOTH_FACTOR_BITS));
}

/* ------------------- Section for CTE functions -----------------------------*/

#define PLL_ADC_CALIB_CORR  2
#define CP_ISEL_Msk         (0x07UL)
#define SYNTH0_ANA_ENG      (*(volatile uint32_t *)0x60001610)
#define SYNTHCAL3_ANA_TST   (*(volatile uint32_t *)0x600015A4)

#if CONFIG_DEVICE_BLUENRG_LPS  
uint32_t SYNTH0_ANA_ENG_bak, PWR_ENGTRIM_bak;
#endif

void BLEPLAT_InitCTE(void)
{
#if CONFIG_DEVICE_BLUENRG_LPS  
  SYNTH0_ANA_ENG_bak = SYNTH0_ANA_ENG;
  PWR_ENGTRIM_bak = PWR->ENGTRIM;
  
  SYNTHCAL3_ANA_TST = 0;
  // Set RFD_PLL_CP_ISEL = 0 in RRM->SYNTH0_ANA_ENG
  SYNTH0_ANA_ENG &= ~CP_ISEL_Msk;
  PWR->ENGTRIM = 0x00000001;
#endif
}

void BLEPLAT_DeinitCTE(void)
{
#if CONFIG_DEVICE_BLUENRG_LPS
  PWR->ENGTRIM = PWR_ENGTRIM_bak;
  SYNTH0_ANA_ENG = SYNTH0_ANA_ENG_bak;
  SYNTHCAL3_ANA_TST = 0;
#endif
}

void BLEPLAT_CalibrateCTE(void)
{
#if CONFIG_DEVICE_BLUENRG_LPS
  uint32_t dac_word = RRM->SYNTHCAL4_DIG_OUT & RRM_SYNTHCAL4_DIG_OUT_MOD_REF_DAC_WORD_OUT_Msk;
  dac_word += PLL_ADC_CALIB_CORR;
  dac_word &= RRM_SYNTHCAL4_DIG_OUT_MOD_REF_DAC_WORD_OUT_Msk;  
  // Set SYNTHCAL3_ANA_TST
  SYNTHCAL3_ANA_TST = dac_word | 0x80;  
#endif  
}

uint8_t antenna_ID_shift = ANTENNA_ID_BIT_SHIFT;

void BLEPLAT_AntIdxRemap(uint8_t antPattLen, uint8_t *antRamTableP, uint8_t* antPattP)
{
#if defined(CONFIG_DEVICE_BLUENRG_LPS)
    for (uint8_t i=0; i<antPattLen; i++)
    {
        antRamTableP[i] = (antPattP[i] << antenna_ID_shift);
    }
#endif
}

#ifdef CONFIG_DEVICE_BLUENRG_LPS
tBleStatus aci_hal_set_antenna_switch_parameters(uint8_t Antenna_IDs,
                                                 uint8_t Antenna_ID_Shift,
                                                 uint8_t Default_Antenna_ID,
                                                 uint8_t RF_Activity_Enable)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {
    .Pin = Antenna_IDs, // With this we assume that Antenna_IDs bitmask is equal to the pin bitmask.
    .Mode = LL_GPIO_MODE_ALTERNATE,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    .Pull = LL_GPIO_PULL_NO, // TODO: with sleep enabled, a pull down is probably needed.
    .Alternate = LL_GPIO_AF_6
  };
  
  if(Antenna_IDs > 0x7F || Antenna_IDs & RESERVED_GPIOS || Antenna_ID_Shift > 7 ||
     Default_Antenna_ID > 0x7F || RF_Activity_Enable > 1)
  {
    return BLE_ERROR_INVALID_HCI_CMD_PARAMS;
  }
  
  antenna_ID_shift = Antenna_ID_Shift;
  
  BLECNTR_GlobSetDefaultAntennaid(Default_Antenna_ID);
  
  if(RF_Activity_Enable)
  {
    GPIO_InitStruct.Pin |= LL_GPIO_PIN_7;
  }    
  
  LL_AHB_EnableClock(LL_AHB_PERIPH_GPIOB);
    
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  LL_GPIO_ResetOutputPin(GPIOB, Antenna_IDs);
  
  return BLE_STATUS_SUCCESS;
}
#else
tBleStatus aci_hal_set_antenna_switch_parameters(uint8_t Antenna_IDs,
                                                 uint8_t Antenna_ID_Shift,
                                                 uint8_t Default_Antenna_ID,
                                                 uint8_t RF_Activity_Enable)
{
  return BLE_ERROR_UNKNOWN_HCI_COMMAND;
}
#endif

/* ---------------------------------------------------------------------------*/


/** 
 *@
} */ /* End of group BlueNRG_LP_Miscellaneous_Utilities */
