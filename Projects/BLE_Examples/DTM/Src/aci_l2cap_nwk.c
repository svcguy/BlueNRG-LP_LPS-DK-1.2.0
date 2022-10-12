
#include "stdint.h"
#include "ble_status.h"
#include "bluenrg_lp_api.h"
#include "aci_l2cap_nwk.h"
#include "bluenrg_lp_l2cap.h"
#include "system_util.h"
#include "string.h"
#include "stack_user_cfg.h"

#if L2CAP_COS_ENABLED == 1

#define NUM_L2CAP_CHANNELS  2

#define RX_SDU_BUFFER_SIZE  600
#define TX_SDU_BUFFER_SIZE  255

typedef struct {
  uint16_t Connection_Handle;
  uint16_t CID;
  uint8_t RX_SDU_Buffer[RX_SDU_BUFFER_SIZE];
  uint8_t TX_SDU_Buffer[TX_SDU_BUFFER_SIZE];
  BOOL TX_buff_in_use;
}l2cap_buffers_t;

l2cap_buffers_t buffers[NUM_L2CAP_CHANNELS];

/**
* @brief  Initialize the module for buffer allocation. Mandatory before any use of the module.
* @retval None
*/
void aci_l2cap_nwk_init(void)
{
  uint8_t i;
  
  for(i = 0; i < NUM_L2CAP_CHANNELS; i++)
  {
    buffers[i].Connection_Handle = 0xFFFF;
  }
}

static int8_t get_free_buff_index(void)
{
  for(uint8_t i = 0; i < NUM_L2CAP_CHANNELS; i++)
  {
    if(buffers[i].Connection_Handle == 0xFFFF)
      return i;
  }
  
  return -1;  
}

static int8_t get_buff_index(uint16_t Connection_Handle, uint16_t CID)
{
  for(uint8_t i = 0; i < NUM_L2CAP_CHANNELS; i++)
  {
    if(buffers[i].Connection_Handle == Connection_Handle && buffers[i].CID == CID)
      return i;
  }
  
  return -1;  
}

static void allocate_buffer(uint8_t i, uint16_t Connection_Handle, uint16_t CID)
{  
  buffers[i].Connection_Handle = Connection_Handle;
  buffers[i].CID = CID;
  buffers[i].TX_buff_in_use = FALSE;
}

static void free_buffer(uint16_t Connection_Handle, uint16_t CID)
{
  int8_t i;
  
  i = get_buff_index(Connection_Handle, CID);
  
  if(i < 0)
  {
    return;
  }
  
  buffers[i].Connection_Handle = 0xFFFF;
}


tBleStatus aci_l2cap_cfc_connection_req_nwk(uint16_t Connection_Handle,
                                            uint16_t SPSM,
                                            uint16_t CID,
                                            uint16_t MTU,
                                            uint16_t MPS,
                                            uint8_t CFC_Policy)
{
  tBleStatus ret;
  int8_t buff_index;
  
  buff_index = get_free_buff_index();
  
  if(buff_index < 0)
  {
    return BLE_ERROR_MEMORY_CAPACITY_EXCEEDED;
  }
  
  ret = aci_l2cap_cfc_connection_req(Connection_Handle, SPSM, CID, MTU, MPS, CFC_Policy,
                                      RX_SDU_BUFFER_SIZE, buffers[buff_index].RX_SDU_Buffer);
  
  if(ret == BLE_STATUS_SUCCESS)
  {
    allocate_buffer(buff_index, Connection_Handle, CID);
  }
  
  return ret;
}

tBleStatus aci_l2cap_cfc_connection_resp_nwk(uint16_t Connection_Handle,
                                             uint8_t Identifier,
                                             uint16_t CID,
                                             uint16_t MTU,
                                             uint16_t MPS,
                                             uint16_t Result,
                                             uint8_t CFC_Policy)

{
  tBleStatus ret;
  int8_t buff_index;
  
  buff_index = get_free_buff_index();
  
  if(buff_index < 0)
  {
    return BLE_ERROR_MEMORY_CAPACITY_EXCEEDED;
  }  
  
  ret = aci_l2cap_cfc_connection_resp(Connection_Handle, Identifier, CID, MTU, MPS, Result, CFC_Policy,
                                       RX_SDU_BUFFER_SIZE, buffers[buff_index].RX_SDU_Buffer);
  
  if(ret == BLE_STATUS_SUCCESS)
  {
    allocate_buffer(buff_index, Connection_Handle, CID);
  }
  
  return ret;
}

int aci_l2cap_cfc_connection_event_preprocess(uint16_t Connection_Handle,
                                              uint8_t Event_Type,
                                              uint16_t Result,
                                              uint8_t Identifier,
                                              uint16_t SPSM,
                                              uint16_t CID,
                                              uint16_t Remote_CID,
                                              uint16_t Peer_MTU,
                                              uint16_t Peer_MPS,
                                              uint16_t Initial_Credits)
{
  if(Event_Type == L2CAP_CONN_RESP && Result != 0)
  {
    free_buffer(Connection_Handle, CID);  
  }

  return 0;
}

int aci_l2cap_disconnection_complete_event_preprocess(uint16_t Connection_Handle,
                                                      uint16_t CID)
{
  free_buffer(Connection_Handle, CID);  
  
  return 0;  
}

void aci_l2cap_sdu_data_rx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t RX_Credit_Balance,
                                 uint16_t SDU_Length)
{
  tBleStatus ret;
  
  uint8_t SDU_Data_Buffer[RX_SDU_BUFFER_SIZE]; //TODO: Probably need to move outside of CSTACK.
  
  ret = aci_l2cap_extract_sdu_data(Connection_Handle, CID, sizeof(SDU_Data_Buffer), SDU_Data_Buffer, &SDU_Length);
  
  if(ret == BLE_STATUS_SUCCESS)
  {
    aci_l2cap_sdu_data_rx_nwk_event(Connection_Handle, CID, RX_Credit_Balance, SDU_Length, SDU_Data_Buffer);
  }
  
}

tBleStatus aci_l2cap_transmit_sdu_data_nwk(uint16_t Connection_Handle,
                                           uint16_t CID,
                                           uint16_t SDU_Length,
                                           uint8_t SDU_Data[])
{
  tBleStatus ret;  
  int8_t buff_index;
  
  buff_index = get_buff_index(Connection_Handle, CID);
  
  if(buff_index < 0)
  {
    return BLE_ERROR_INVALID_HCI_CMD_PARAMS;
  }
  
  if(buffers[buff_index].TX_buff_in_use)
  {
    return BLE_STATUS_INSUFFICIENT_RESOURCES;
  }
  if(SDU_Length > TX_SDU_BUFFER_SIZE)
  {
    return BLE_ERROR_MEMORY_CAPACITY_EXCEEDED;
  }
  
  memcpy(buffers[buff_index].TX_SDU_Buffer, SDU_Data, SDU_Length);
  
  ret = aci_l2cap_transmit_sdu_data(Connection_Handle, CID, SDU_Length, buffers[buff_index].TX_SDU_Buffer);
  if(ret == BLE_STATUS_SUCCESS)
  {
    buffers[buff_index].TX_buff_in_use = TRUE;
  }
  return ret;
}

void aci_l2cap_sdu_data_tx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t SDU_Length,
                                 void * SDU_Data_Buffer,
                                 uint16_t TX_Credit_Balance)
{
  int8_t buff_index;
  
  buff_index = get_buff_index(Connection_Handle, CID);
  
  if(buff_index < 0)
  {
    return; // This should not happen.
  }
  
  buffers[buff_index].TX_buff_in_use = FALSE;
  
  aci_l2cap_sdu_data_tx_nwk_event(Connection_Handle, CID, SDU_Length, TX_Credit_Balance);    
}

#else /* L2CAP_COS_ENABLED == 0 */

void aci_l2cap_nwk_init(void)
{
}

int aci_l2cap_cfc_connection_event_preprocess(uint16_t Connection_Handle,
                                              uint8_t Event_Type,
                                              uint16_t Result,
                                              uint8_t Identifier,
                                              uint16_t SPSM,
                                              uint16_t CID,
                                              uint16_t Remote_CID,
                                              uint16_t Peer_MTU,
                                              uint16_t Peer_MPS,
                                              uint16_t Initial_Credits)
{
  return 0;
}

int aci_l2cap_disconnection_complete_event_preprocess(uint16_t Connection_Handle,
                                                      uint16_t CID)
{ 
  return 0;  
}

tBleStatus aci_l2cap_transmit_sdu_data_nwk(uint16_t Connection_Handle,
                                           uint16_t CID,
                                           uint16_t SDU_Length,
                                           uint8_t SDU_Data[])
{
  return BLE_ERROR_UNKNOWN_HCI_COMMAND;
}

tBleStatus aci_l2cap_cfc_connection_req_nwk(uint16_t Connection_Handle,
                                            uint16_t SPSM,
                                            uint16_t CID,
                                            uint16_t MTU,
                                            uint16_t MPS,
                                            uint8_t CFC_Policy)
{
  return BLE_ERROR_UNKNOWN_HCI_COMMAND;
}

tBleStatus aci_l2cap_cfc_connection_resp_nwk(uint16_t Connection_Handle,
                                             uint8_t Identifier,
                                             uint16_t CID,
                                             uint16_t MTU,
                                             uint16_t MPS,
                                             uint16_t Result,
                                             uint8_t CFC_Policy)
{
  return BLE_ERROR_UNKNOWN_HCI_COMMAND;
}

#endif
