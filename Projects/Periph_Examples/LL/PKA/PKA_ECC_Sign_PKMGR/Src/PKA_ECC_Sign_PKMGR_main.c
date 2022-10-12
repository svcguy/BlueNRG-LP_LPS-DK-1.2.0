/**
******************************************************************************
* @file    LL/PKA/PKA_ECC_Sign_PKMGR/Src/PKA_ECC_Sign_PKMGR_main.c
* @author  RF Application Team
* @brief   This example describes how to use PKA peripheral to generate an
*          ECDSA signature using the BLUENRG_LP PKA LL API.
*          Peripheral initialization done using LL unitary services functions.
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
#include "PKA_ECC_Sign_PKMGR_main.h"

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
uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength);
static void LL_Init(void);
void CallbackNewPointA(PKAMGR_ResultStatus errorCode, void *args); 
void CallbackNewPointB(PKAMGR_ResultStatus errorCode, void *args); 
void CallbackNewPointC(PKAMGR_ResultStatus errorCode, void *args); 

/* Private user code ---------------------------------------------------------*/
static uint32_t PublicKeyOutA[16], PublicKeyOutB[16], PublicKeyOutC[16];
static uint32_t RandomKA[8] = {0};

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
  LL_Init();
  
  /* Set systick to 1ms using system clock frequency */
  LL_Init1msTick(SystemCoreClock);
  
#if defined(CONFIG_DEVICE_BLUENRG_LP) || defined(CONFIG_DEVICE_BLUENRG_LPS)
  /* IO pull configuration with minimum power consumption */
  BSP_IO_Init();
#endif
  
  /* Initialization of COM port */
  BSP_COM_Init(NULL);
   
  printf("\n\r\tPKA_ECC_Sign_PKMGR\tMANAGER VERSION\n\r\n\r");
#ifdef STEVAL_IDB011V1
  printf("\tSTEVAL_IDB011V1\n\r");
#endif
#ifdef STEVAL_IDB012V1
  printf("\tSTEVAL_IDB012V1\n\r");
#endif

  /* Initialize all configured peripherals */
  if(RNGMGR_Init() == RNGMGR_ERROR)
  {
    Error_Handler();
  } 
  
  if(PKAMGR_Init() == PKAMGR_ERROR)
  {
    Error_Handler();
  }

  /* ***** PROCEDURE FOR A ***** */
  if( PKAMGR_StartP256PublicKeyGeneration(&CallbackNewPointA) != PKAMGR_SUCCESS)
  {
    printf("PROCEDURE FOR A - Error : PKAMGR_StartP256PublicKeyGeneration(&CallbackNewPointA)\n\r");
  }  
  
  while(PKAMGR_SleepCheck()==PKAMGR_ERR_BUSY);

  /* ***** PROCEDURE FOR B ***** */
  if( PKAMGR_StartP256PublicKeyGeneration(&CallbackNewPointB) != PKAMGR_SUCCESS)
  {
    printf("PROCEDURE FOR B - Error : PKAMGR_StartP256PublicKeyGeneration(&CallbackNewPointB)\n\r");
  }
  
  while(PKAMGR_SleepCheck()==PKAMGR_ERR_BUSY);

  /* ***** CHECK ELLIPTIC ***** */
  /* New PKA process is started using the RandomKA with the point B coordinates. 
     This generates a new point C which is still on the same ellipse */
  if( PKAMGR_StartP256DHkeyGeneration(RandomKA, (uint32_t *)&PublicKeyOutB[0], &CallbackNewPointC) != PKAMGR_SUCCESS)
  {
    printf("CHECK ELLIPTIC - Error : PKAMGR_StartP256DHkeyGeneration(RandomKA, (uint32_t *)&PublicKeyOutB[0], &CallbackNewPointC)\n\r");
  }
  
  while(PKAMGR_SleepCheck()==PKAMGR_ERR_BUSY);
  
  if(PKAMGR_Deinit() == PKAMGR_ERROR)
  {
    Error_Handler();
  }
  
  printf("RandomKA \n\r");
  PrintBuffer((uint32_t *)&RandomKA[0],8);
  printf("PublicKeyOutA \n\r");
  PrintBuffer((uint32_t *)&PublicKeyOutA[0],16); 
  printf("PublicKeyOutB \n\r");
  PrintBuffer((uint32_t *)&PublicKeyOutB[0],16); 
  printf("CHECK ELLIPTIC\n\r");
  printf("PublicKeyOutC \n\r");
  PrintBuffer((uint32_t *)&PublicKeyOutC[0],16);
  
  /* Infinite loop */
  while (1)
  {
  }
}

void CallbackNewPointA(PKAMGR_ResultStatus errorCode, void *args)
{
  printf("(A) The PKA process is ended with :");
  if(errorCode==PKAMGR_ERROR)
  {
    PublicKeyOutA[0] = ((uint32_t *)args)[0];
    printf("ERROR\terror code: 0x%08X \n\r", ((uint32_t *)args)[0]);
  }
  else
  {
    /* Get the new calculated point NewPoint of the ellipse */
    printf("SUCCESS\n\r");
    for(uint8_t i=0;i<8;i++) {
      RandomKA[i] = ((uint32_t *)args)[i];
    }
    for(uint8_t i=8;i<24;i++) {
      PublicKeyOutA[i-8] = ((uint32_t *)args)[i];
    }
  }
}

void CallbackNewPointB(PKAMGR_ResultStatus errorCode, void *args)
{
  printf("(B) The PKA process is ended with :");
  if(errorCode==PKAMGR_ERROR)
  {
    printf("ERROR\terror code: 0x%08X \n\r", ((uint32_t *)args)[0]);
  }
  else
  {
    /* Get the new calculated point NewPoint of the ellipse */
    printf("SUCCESS\n\r");
    for(uint8_t i=8;i<24;i++) {
      PublicKeyOutB[i-8] = ((uint32_t *)args)[i];
    }
  }
}

void CallbackNewPointC(PKAMGR_ResultStatus errorCode, void *args)
{
  printf("(C) The PKA process is ended with :");
  if(errorCode==PKAMGR_ERROR)
  {
    printf("ERROR\terror code: 0x%08X \n\r", ((uint32_t *)args)[0]);
  }
  else
  {
    /* Get the new calculated point NewPoint of the ellipse */
    printf("SUCCESS\n\r");
    for(uint8_t i=8;i<24;i++) {
      PublicKeyOutC[i-8] = ((uint32_t *)args)[i];
    }
    /* PKA verified */
    printf("PKA verified.\n\r");
    printf("** Test successfully. ** \n\r\n\r");
  }
}

void PrintBuffer(uint32_t* pBuffer, uint32_t BufferLength)
{
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
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0: pBuffer identical to pBuffer1
  *         1: pBuffer differs from pBuffer1
  */
uint8_t Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return 1;
    }
    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
  while(1);
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

