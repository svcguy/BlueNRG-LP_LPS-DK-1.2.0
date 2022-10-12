/**
  ******************************************************************************
  * @file    main_common.h 
  * @author  RF Application Team
  * @brief   Library configuration file.
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
  * <h2><center>&copy; COPYRIGHT 2020 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MAIN_COMMON_H
#define MAIN_COMMON_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/
#define CALIBRATION_INTERVAL_CONF   10000

#if defined CONFIG_HW_LS_RO  

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        500

/* Calibration must be done */
#define INITIAL_CALIBRATION TRUE
#define CALIBRATION_INTERVAL        CALIBRATION_INTERVAL_CONF

#elif defined CONFIG_HW_LS_XTAL

/* Sleep clock accuracy. */
#define SLEEP_CLOCK_ACCURACY        100

/* No Calibration */
#define INITIAL_CALIBRATION FALSE
#define CALIBRATION_INTERVAL        0

#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define ------------------------------------------------------------*/
#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x8E89BED6)
#define FREQUENCY_CHANNEL       (uint8_t)(24)    // RF channel 22
#define HS_STARTUP_TIME         (uint16_t)(1)   /* High Speed start up time min value */
#define TX_WAKEUP_TIME           400      /* 400 us */
#define RX_WAKEUP_TIME           300      /* 300 us */
#define RX_TIMEOUT            100000      /* 100 ms */
#define RX_TIMEOUT_ACK           400      /* 400 us */

/* Exported functions ------------------------------------------------------- */


#endif /* MAIN_COMMON_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
