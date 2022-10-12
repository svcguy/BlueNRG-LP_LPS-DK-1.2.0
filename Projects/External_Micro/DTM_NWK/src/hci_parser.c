/**
******************************************************************************
* @file    hci_parser.c 
* @author  RF Application Team - AMG
* @version V1.0.0
* @date    August, 2016
* @brief   HCI parser file
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
* <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/
#include "hci_const.h"
#include "hci_parser.h"
#include "SDK_EVAL_Config.h"
#include <stdio.h>

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define HCI_PACKET_SIZE         532 // Because of extended ACI commands, size can be bigger than standard HCI commands
#define HCI_HDR_SIZE 1

/* Commands */
#define READ_VERSION            0x01
#define BLUENRG_RESET           0x04
#define HW_BOOTLOADER           0x05

/* Types of response index */
#define RESP_VENDOR_CODE_OFFSET     1
#define RESP_LEN_OFFSET_LSB         2
#define RESP_LEN_OFFSET_MSB         3
#define RESP_CMDCODE_OFFSET         4
#define RESP_STATUS_OFFSET          5
#define RESP_PARAM_OFFSET           6

/* Types of vendor codes */
#define ERROR                   0
#define RESPONSE                1

/* Error codes */
#define UNKNOWN_COMMAND	        0x01
#define INVALID_PARAMETERS	    0x12


/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t command_fifo[265];
extern volatile uint16_t command_fifo_size;

uint8_t buffer_out[HCI_PACKET_SIZE];


/* Private function prototypes -----------------------------------------------*/
static uint16_t parse_cmd(uint8_t *hci_buffer, uint16_t hci_pckt_len, uint8_t *buffer_out);


void packet_received(uint8_t *packet, uin16_t pckt_len)
{ 
  uint16_t buffer_out_len=0;

  switch(packet[HCI_TYPE_OFFSET]) {
  case HCI_VENDOR_PKT:
    buffer_out_len = parse_cmd(packet, pckt_len, buffer_out);
    
    for(volatile uint8_t i = 0; i< buffer_out_len;i++)
      putchar(buffer_out[i]);
    break;
    
  default:
    for(uint16_t i=0;i<pckt_len;i++) {
      command_fifo[i] = packet[i];
    }
    command_fifo_size = pckt_len;
    break;
  }
}


/* Command parser
 * bytes
 * 1         Type of packet (FF for special command)
 * 1         cmdcode
 * 2         cmd length (length of arguments)
 * variable  payload
 */
uint16_t parse_cmd(uint8_t *hci_buffer, uint16_t hci_pckt_len, uint8_t *buffer_out)
{
    uint16_t len = 0;
    
    buffer_out[0] = HCI_VENDOR_PKT;
    buffer_out[RESP_VENDOR_CODE_OFFSET] = RESPONSE;
    buffer_out[RESP_CMDCODE_OFFSET] = hci_buffer[HCI_VENDOR_CMDCODE_OFFSET];
    buffer_out[RESP_STATUS_OFFSET] = 0;
    
    switch(hci_buffer[HCI_VENDOR_CMDCODE_OFFSET]) {
    case READ_VERSION:
      buffer_out[RESP_PARAM_OFFSET] = FW_VERSION_MAJOR;
      buffer_out[RESP_PARAM_OFFSET+1] = FW_VERSION_MINOR;
      len = 2;
      break;

    case BLUENRG_RESET:
      BlueNRG_RST();
      break;

    case HW_BOOTLOADER:
      BlueNRG_HW_Bootloader();
//        SdkEvalSpiDtmInit(); // configure the SPI mode
      break;

    default:
      buffer_out[RESP_STATUS_OFFSET] = UNKNOWN_COMMAND;
    }
    
    len += 2; // Status and Command code
    PACK_2_BYTE_PARAMETER(buffer_out+RESP_LEN_OFFSET_LSB,len);
    len += RESP_CMDCODE_OFFSET;     
    
    return len;
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/