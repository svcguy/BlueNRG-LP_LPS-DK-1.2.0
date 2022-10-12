/**
  ******************************************************************************
  * @file    DTM_Updater_Config.h 
  * @author  VMA RF Application Team
  * @version V1.0.0
  * @date    July-2015
  * @brief   
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
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DTM_UPDATER_CONFIG_H
#define DTM_UPDATER_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "bluenrg_lp_evb_config.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#ifdef CONFIG_DEVICE_BLUENRG_LPS
#define BLUE_FLAG_FLASH_BASE_ADDRESS    (0x10041014)
#define DTM_APP_ADDR                    (0x10041000)
#else
#define BLUE_FLAG_FLASH_BASE_ADDRESS    (0x10042014)
#define DTM_APP_ADDR                    (0x10042000)
#endif

#define BLUE_FLAG_RAM_RESET             (0x01010101)
#define BLUE_FLAG_RESET                 (0x00000000)
#define BLUE_FLAG_SET                   (0x424C5545)
    
#ifdef CONFIG_DEVICE_BLUENRG_LP
#define DTM_SPI_BOOT_PIN_PDA()  LL_PWR_EnablePDA(LL_PWR_PUPD_IO15)
#define DTM_SPI_BOOT_PIN_PUA()  LL_PWR_EnablePUA(LL_PWR_PUPD_IO15)

#endif

#ifdef CONFIG_DEVICE_BLUENRG_LPS
#define DTM_SPI_BOOT_PIN_PDA()  LL_PWR_EnablePDA(LL_PWR_PUPD_IO11)
#define DTM_SPI_BOOT_PIN_PUA()  LL_PWR_EnablePUA(LL_PWR_PUPD_IO11)

#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* DTM_UPDATER_CONFIG_H */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/