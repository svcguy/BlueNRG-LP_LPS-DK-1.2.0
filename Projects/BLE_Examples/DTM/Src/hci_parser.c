/**
******************************************************************************
* @file    hci_parser.c 
* @author  VMA RF Application Team
  * @version V1.2.0
  * @date    April-2018
* @brief   Transport layer file
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

/* Includes ------------------------------------------------------------------*/

#include "ble_const.h"
#include "bluenrg_lp_api.h"
#include "bluenrg_lp_events.h"
#include "bluenrg_lp_stack.h"
#include "hci_parser.h"
#include "hw_config.h"
#include "cmd.h"
#include "transport_layer.h"
#include "osal.h"


/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#ifndef DTM_DEBUG
/* Macro for debug purposes: not relevant x application */
#define DTM_DEBUG 0
#endif 

#define HCI_PACKET_SIZE 536 // Maximum size of HCI packets are 255 bytes + the HCI header (3 bytes) + 1 byte for transport layer.

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t hci_buffer[HCI_PACKET_SIZE];
static volatile uint16_t hci_pckt_len = 0;

uint8_t buffer_out[HCI_PACKET_SIZE];
uint16_t buffer_out_len=0;

/* Private function prototypes -----------------------------------------------*/
void packet_received(void);

hci_state hci_input(uint8_t *buff, uint16_t len)
{
  static hci_state state = WAITING_TYPE;
    
  static uint16_t collected_payload_len = 0;
  static uint16_t payload_len;
  static uint16_t header_len;
  uint8_t byte;
  uint16_t i = 0;
  
  while(hci_pckt_len < HCI_PACKET_SIZE && i++ < len){
        
    byte = *buff++;

    if(state == WAITING_TYPE)
      hci_pckt_len = 0;
    
    hci_buffer[hci_pckt_len++] = byte;        
        
    if(state == WAITING_TYPE){
      
      state = WAITING_HEADER;
      
      if(byte == HCI_COMMAND_PKT){
        header_len = 4;
      }
      else if(byte == HCI_COMMAND_EXT_PKT){
        header_len = 5;
      }
      else if(byte == HCI_ACLDATA_PKT){
        header_len = 5;
      }
      else if(byte == HCI_VENDOR_PKT){
        header_len = 4;
      }
      else {
        state = WAITING_TYPE;        
      }
    }
    else if(state == WAITING_HEADER){
      
      if(hci_pckt_len == header_len){
                    
        // The entire header has been received
        uint8_t pckt_type = hci_buffer[0];
        collected_payload_len = 0;
        payload_len = 0;
        
        if(pckt_type == HCI_COMMAND_PKT){
          hci_cmd_hdr *hdr = (hci_cmd_hdr *)hci_buffer;
          payload_len = hdr->param_len;
        }
        else if(pckt_type == HCI_COMMAND_EXT_PKT){
          hci_cmd_ext_hdr *hdr = (hci_cmd_ext_hdr *)hci_buffer;
          payload_len = hdr->param_len;
        }
        else if(pckt_type == HCI_ACLDATA_PKT){
          hci_acl_hdr *hdr = (hci_acl_hdr *)hci_buffer;
          payload_len = hdr->dlen;
        }
        else if(pckt_type == HCI_VENDOR_PKT){
          hci_vendor_hdr *hdr = (hci_vendor_hdr *)hci_buffer;
          payload_len = hdr->param_len;
        }
        if(payload_len == 0){
          state = WAITING_TYPE;
          packet_received();
        }
        else {
          state = WAITING_PAYLOAD;                      
        }
      }      
    }
    else if(state == WAITING_PAYLOAD){
      collected_payload_len++;
      if(collected_payload_len >= payload_len){
        state = WAITING_TYPE;
        packet_received();
      }      
    }
  }
  
  return state;
}

void packet_received(void)
{ 
  switch(hci_buffer[HCI_TYPE_OFFSET]) {
  case HCI_VENDOR_PKT: /* In SPI mode never gets HCI_VENDOR_PKT */
    buffer_out_len = parse_cmd(hci_buffer, hci_pckt_len, buffer_out);
    send_event(buffer_out, buffer_out_len, 1);
    break;
  case HCI_ACLDATA_PKT:
    {
      uint16_t connHandle;
      uint16_t dataLen;
      uint8_t* pduData;
      uint8_t  pb_flag;
      uint8_t  bc_flag;
      
      connHandle = ((hci_buffer[2] & 0x0F) << 8) + hci_buffer[1];
      dataLen = (hci_buffer[4] << 8) + hci_buffer[3];
      pduData = hci_buffer+5;
      pb_flag = (hci_buffer[2] >> 4) & 0x3;
      bc_flag = (hci_buffer[2] >> 6) & 0x3;
      hci_tx_acl_data(connHandle, pb_flag, bc_flag, dataLen, pduData);
    }
    break;
  case HCI_COMMAND_PKT:
  case HCI_COMMAND_EXT_PKT:
    send_command(hci_buffer, hci_pckt_len);
    break;
  default:
    // Error case not allowed TBR
    break;
  }
}

tBleStatus hci_rx_acl_data_event(uint16_t connHandle, uint8_t  pb_flag, uint8_t  bc_flag, uint16_t  dataLen, uint8_t*  pduData)
{
  uint8_t buffer_out[251+5];
  
  buffer_out[0] = 0x02;
  buffer_out[1] = connHandle & 0xFF;
  buffer_out[2] = (connHandle >> 8 & 0x0F) | (pb_flag << 4) | (bc_flag << 6) ;
  Osal_MemCpy(buffer_out+3,&dataLen, 2);
  Osal_MemCpy(buffer_out+5, pduData, dataLen);
  send_event(buffer_out, dataLen+2+2+1, -1);
  return 0;
}
