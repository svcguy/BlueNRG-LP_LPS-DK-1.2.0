/**
******************************************************************************
* @file    DTM_Updater_main.c 
* @author  AMS RF Application Team
* @version V2.0.0
* @date    February-2020
* @brief   DTM updater main. IMPORTANT: user must NOT modify the updater algorithm and code.
*          MEMORY_FLASH_APP_SIZE=0x2000 (DTM_updater size is 8K)
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
* <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
******************************************************************************
*/ 

#include "DTM_Updater.h"
#include "DTM_Updater_Config.h"
#include "system_BlueNRG_LP.h"

#define ENTERED_REASON_ACI_CMD          2
#define ENTERED_REASON_BAD_BLUEFLAG     3
#define ENTERED_REASON_IRQ_PIN          4

#define RESET_WAKE_DEEPSLEEP_REASONS 0x05

typedef void (*EntryPoint)(void);

int main(void)
{
  uint32_t boot_pin;
  
  //  if ((CKGEN_SOC->REASON_RST == 0) && (CKGEN_BLE->REASON_RST > RESET_WAKE_DEEPSLEEP_REASONS)) {
  if ((RCC->CSR == 0) && (PWR->SR1 != 0)) {
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(DTM_APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) DTM_APP_ADDR);
    entryPoint();
    
    while(1);
  }
  
  // The boot pin is the SPI_MOSI. It is configured in Input Pull-Up @ reset state.
  // We need to re-configure the boot pin with the Pull-Down resistor.
  DTM_SPI_BOOT_PIN_PDA();
  //If the SPI_MOSI is floating we need to read the line multiple times to have the correct value
  for (int i=0; i<4; i++) {
    boot_pin = LL_GPIO_IsInputPinSet(BSP_SPI_MOSI_GPIO_PORT, BSP_SPI_MOSI_PIN);
  }
  // Reconfigure the reset pull value
  DTM_SPI_BOOT_PIN_PUA();    
  
  /* if BLUE_FLAG_RESET => a previous programming operation went bad */
  if(RAM_VR.BlueFlag == BLUE_FLAG_RAM_RESET) {
    RAM_VR.BlueFlag = BLUE_FLAG_SET;
    updater_init();
    updater(ENTERED_REASON_ACI_CMD);
  }
  else if(*(uint32_t *)BLUE_FLAG_FLASH_BASE_ADDRESS != BLUE_FLAG_SET) {
    updater_init();
    updater(ENTERED_REASON_BAD_BLUEFLAG);
  }
  else if(boot_pin != 0) {
    updater_init();
    updater(ENTERED_REASON_IRQ_PIN);
  }
  else {
    EntryPoint entryPoint = (EntryPoint)(*(volatile uint32_t *)(DTM_APP_ADDR + 4));
    __set_MSP(*(volatile uint32_t*) DTM_APP_ADDR);
    entryPoint();
    
    while(1);
  }
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
