/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
* @file    Micro/MICRO_Hello_World/Src/Micro_Hello_World_main.c
* @author  RF Application Team
* @brief   Code demonstrating Hello World.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "rf_device_it.h"

#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
#include "bluenrg_lp_evb_config.h"
#endif
#include "hal_miscutil.h"

/** @addtogroup BlueNRGLP_StdPeriph_Examples LL Peripheral Examples
  * @{
  */


/** @addtogroup Micro_Examples Micro Examples
  * @{
  */

/** @addtogroup Micro_HelloWorld  Micro Hello World Example
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  uint32_t counter = 0;
  PartInfoType partInfo;
  crash_info_t crashInfo;
  
  /* System initialization function */
  if (SystemInit(SYSCLK_64M, RADIO_SYSCLK_NONE) != SUCCESS)
  {
    /* Error during system clock configuration take appropriate action */
    while(1);
  }
  
  /* Init the UART peripheral */
  BSP_COM_Init(NULL);
  
  HAL_GetPartInfo(&partInfo);
  
  HAL_GetCrashInfo(&crashInfo);
  if ((crashInfo.signature & 0xFFFF0000) == CRASH_SIGNATURE_BASE) {
    printf("Application crash detected\r\n");
    printf("Crash Info signature = 0x%08lx\r\n", crashInfo.signature);
    printf("Crash Info SP        = 0x%08lx\r\n", crashInfo.SP);
    printf("Crash Info R0        = 0x%08lx\r\n", crashInfo.R0);
    printf("Crash Info R1        = 0x%08lx\r\n", crashInfo.R1);
    printf("Crash Info R2        = 0x%08lx\r\n", crashInfo.R2);
    printf("Crash Info R3        = 0x%08lx\r\n", crashInfo.R3);
    printf("Crash Info R12       = 0x%08lx\r\n", crashInfo.R12);
    printf("Crash Info LR        = 0x%08lx\r\n", crashInfo.LR);
    printf("Crash Info PC        = 0x%08lx\r\n", crashInfo.PC);
    printf("Crash Info xPSR      = 0x%08lx\r\n", crashInfo.xPSR);
  }
 
  /* infinite loop */
  while(1) 
  {
    if (counter == 0 ) {
#if defined(CONFIG_DEVICE_BLUENRG_LP)
      printf("Hello World: BlueNRG-LP (%d.%d) is here!\r\n",
             partInfo.die_major,
             partInfo.die_cut);
#elif defined CONFIG_DEVICE_BLUENRG_LPS
       printf("Hello World: BlueNRG-LP%c (%d.%d) is here!\r\n",
             (partInfo.jtag_id_code == JTAG_ID_CODE_LPS) ? 'S': ' ',
             partInfo.die_major,
             partInfo.die_cut);
#endif
    }
    counter = (counter +1) & (0xFFFFF);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

#endif


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2020 STMicroelectronics *****END OF FILE****/



