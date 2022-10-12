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


/* High Speed start up time */
#define HS_STARTUP_TIME 328 // 800 us

#define BLE_ADV_ACCESS_ADDRESS  (uint32_t)(0x8E89BED6)
#define FREQUENCY_CHANNEL       (uint8_t)(24)    // RF channel 22
#define TX_WAKEUP_TIME         (10000+700)      /* 10 ms + 700 us of guard time */
#define RX_WAKEUP_TIME          10000            /* 10 ms */
#define RX_TIMEOUT_OK           5000            /* 5 ms */
#define RX_TIMEOUT_NOTOK        30000           /* 30 ms */
#define RX_TIMEOUT_ACK          1000            /* 1 ms */

/* Exported functions ------------------------------------------------------- */


#endif /* MAIN_COMMON_H */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/
