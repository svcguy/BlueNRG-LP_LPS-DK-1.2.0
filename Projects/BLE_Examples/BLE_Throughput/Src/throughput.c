/******************** (C) COPYRIGHT 2020 STMicroelectronics ********************
 * File Name          : throughput.c
 * Author             : AMS - RF  Application team
 * Version            : V1.0.0
 * Date               : 08-June-2020
 * Description        : This file handles bytes received from USB and the init
 *                      function.
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "rf_device_it.h"
#include "ble_const.h"
#include "bluenrg_lp_stack.h"
#include "app_state.h"
#include "throughput.h"
#include "osal.h"
#include "gatt_db.h"
#include "clock.h"
#include "gp_timer.h"
#include "bluenrg_lp_api.h"
#include "gap_profile.h"

/*
 * External variables --------------------------------------------------------
 * Private typedef -----------------------------------------------------------
 * Private defines -----------------------------------------------------------
 */
#define PRINT_INT(x)    ((int)(x))
#define PRINT_FLOAT(x)  (x>0)? ((int) (((x) - PRINT_INT(x)) * 1000)) : (-1*(((int) (((x) - PRINT_INT(x)) * 1000))))

#define COMMAND_SET(n)              (command |= (1 << n))
#define COMMAND_CLEAR(n)            (command &= ~(1 << n))
#define COMMAND_IS_SET(n)           (command & (1 << n))
#define COMMAND_RESET()             (command = 0)
#define SEND_NOTIFICATION           (0x01)
#define SEND_INDICATION             (0x02)
#if SERVER
#define DEVICE_NAME                 "SERVER"
#else
#define DEVICE_NAME                 "CLIENT"
#endif
#define PRINT_APP_FLAG_FORMAT(f)    (APP_FLAG(f) ? "ON" : ". ")
#define MAX_DESCRIPTION_LEN         (100)
#define CMD_TABLE_SIZE              (sizeof(command_table) / sizeof(command_table[0]))

#define TX_OCTETS_TO_TIME(OCTET)    ((OCTET + 14) * 8)
#define THROUGHPUT_DOWN_COUNTER     (10000)

#define LOCAL_NAME_LENGTH   9
#define LOCAL_NAME 'B','l','u','e','N','R','G','L','P'

#define L2C_COS_RX_SDU_BUFFER_SIZE_X_MTU(MTU)   (((MTU) + 2) * 3)

enum {
    CMD_DATA_LEN_UPDATE_REQ_LOW = 0,
    CMD_DATA_LEN_UPDATE_REQ_MID,
    CMD_DATA_LEN_UPDATE_REQ_HIGH,
    CMD_ATT_MTU,
    CMD_BIDIRECT,
    CMD_CONN_UPDATE,
    CMD_SEND_NOTIFICATION,
    CMD_SEND_INDICATION,
    CMD_L2C_COS_TX,
    CMD_PRINT_FLAGS,
    CMD_HELP,
    CMD_TH_SLOW_TOGGLE,
    CMD_L2C_COS_CHANGE_MTU,
    CMD_L2C_COS_CHANGE_MPS,
#if SET_PHY 
    CMD_PHY_CODED_RX,
    CMD_PHY_CODED_TX,
    CMD_PHY_1M_TX,
    CMD_PHY_1M_RX,
    CMD_PHY_2M_TX,
    CMD_PHY_2M_RX,
    CMD_PHY_READ,
#endif    
    
#if(SERVER)
    CMD_FLUSH,
    CMD_DISABLE_NOTIFY,
#endif
    CMD_RESET,
    CMD_COUNT,
};

typedef struct _cmd_t {
    char cmd;
    char description[MAX_DESCRIPTION_LEN];
    uint32_t command_flag;
} cmd_t;

typedef PACKED(struct) _header_t { 
    uint32_t sn;
    uint32_t crc;
} header_t;

typedef PACKED(struct) _pckt_t {
    header_t header;
    uint8_t data[];
} pckt_t;

static cmd_t command_table[] = {
    {
        .cmd = 'u',
        .description = "Send data len update request for 27 bytes",
        .command_flag = CMD_DATA_LEN_UPDATE_REQ_LOW,
    },
    {
        .cmd = 'm',
        .description = "Send data len update request for 100 bytes",
        .command_flag = CMD_DATA_LEN_UPDATE_REQ_MID,
    },
    {
        .cmd = 'U',
        .description = "Send data len update request for 251 bytes",
        .command_flag = CMD_DATA_LEN_UPDATE_REQ_HIGH,
    },
#if CLIENT
    {
        .cmd = 'a',
        .description = "Send ATT_MTU exchange",
        .command_flag = CMD_ATT_MTU,
    },
#endif 
#if SET_L2CAP_COS 
    {
        .cmd = 'l',
        .description = "Enable/disable L2CAP COS txing",
        .command_flag = CMD_L2C_COS_TX,
    },
#endif
    {
        .cmd = 'z',
        .description = "Enable/disable slow throughput",
        .command_flag = CMD_TH_SLOW_TOGGLE,
    },
    {
        .cmd = '1',
        .description = "Change L2C COS MTU value",
        .command_flag = CMD_L2C_COS_CHANGE_MTU,
    },
    {
        .cmd = '2',
        .description = "Change L2C COS MPS value",
        .command_flag = CMD_L2C_COS_CHANGE_MPS,
    },
    
#if CLIENT
    {
        .cmd = 'b',
        .description = "Switch bidirectional test on-off",
        .command_flag = CMD_BIDIRECT
    },
    {
        .cmd = 'n',
        .description = "Send notifications",
        .command_flag = CMD_SEND_NOTIFICATION,
    },
    {
        .cmd = 'i',
        .description = "Send indication",
        .command_flag = CMD_SEND_INDICATION,
    },
#else
    {
        .cmd = 'c',
        .description = "Send connection parameter update request",
        .command_flag = CMD_CONN_UPDATE,
    },
#endif
#if (SERVER)
    {
        .cmd = 'f',
        .description = "Enable/disable flushable PDUs",
        .command_flag = CMD_FLUSH,
    },
    {
        .cmd = 'e',
        .description = "Toggle notify",
        .command_flag = CMD_DISABLE_NOTIFY,
    },
#endif
    {
        .cmd = 'p',
        .description = "Print APP flags",
        .command_flag = CMD_PRINT_FLAGS,
    },
#if SET_PHY 
    {
        .cmd = 's',
        .description = "Read LE PHY (TX, RX) ",
        .command_flag = CMD_PHY_READ,
    },
    {
        .cmd = 'd',
        .description = "Set LE RX PHY to Coded",
        .command_flag = CMD_PHY_CODED_RX,
    },
    {
        .cmd = 'D',
        .description = "Set LE TX PHY to Coded",
        .command_flag = CMD_PHY_CODED_TX,
    },
    {
        .cmd = 't',
        .description = "Set LE TX PHY to 1 Mbps ",
        .command_flag = CMD_PHY_1M_TX,
    },
    {
        .cmd = 'r',
        .description = "Set LE RX PHY to 1 Mpbs",
        .command_flag = CMD_PHY_1M_RX,
    },
    {
        .cmd = 'T',
        .description = "Set LE TX PHY to 2 Mbps ",
        .command_flag = CMD_PHY_2M_TX,
    },
    {
        .cmd = 'R',
        .description = "Set LE RX PHY to 2 Mpbs",
        .command_flag = CMD_PHY_2M_RX,
    },
#endif 
    {
        .cmd = 'x',
        .description = "System reset",
        .command_flag = CMD_RESET,
    },

    {
        .cmd = '?',
        .description = "Print help",
        .command_flag = CMD_HELP,
    },
};

typedef struct _app_context_t {
    tClockTime start_time;
    tClockTime end_time;
    uint8_t not_first_packet;
    uint32_t packets;
    uint32_t sn;
    uint32_t data_size;
    uint32_t lost_pckts;
	uint32_t duplicated_pckts;
    uint32_t corrupted_pckts;
    uint32_t m_buffer_miss;
    float avg_val;
} app_context_t;

static const uint16_t l2c_cos_mtu_table[] = {23, 44, 90, 182, 245, 366, 548, 1024};
#define L2C_COS_MTU_TABLE_SIZE      (sizeof(l2c_cos_mtu_table) / sizeof(uint16_t))

static const uint16_t l2c_cos_mps_table[] = {23, 46, 92, 184, 247, 368, 550, 1024};
#define L2C_COS_MPS_TABLE_SIZE      (sizeof(l2c_cos_mps_table) / sizeof(uint16_t))

static uint8_t l2c_cos_mtu_table_index =  L2C_COS_MTU_TABLE_SIZE - 1;
static uint8_t l2c_cos_mps_table_index =  L2C_COS_MPS_TABLE_SIZE - 1;

static uint16_t l2c_cos_local_mtu = L2C_COS_MTU;
static uint16_t l2c_cos_local_mps = L2C_COS_MPS;

static app_context_t rx_app_context;
static app_context_t tx_app_context;
static app_context_t l2c_cos_rx_app_context;
static app_context_t l2c_cos_tx_app_context;

uint8_t connInfo[20];
volatile int app_flags = SET_CONNECTABLE;
volatile uint16_t connection_handle = 0;
extern uint8_t charUuidTX[16];
extern uint8_t charUuidRX[16];
/**
 * @brief  Handle of TX,RX  Characteristics.
 */
#ifdef CLIENT
uint16_t tx_handle;
uint16_t rx_handle;
#else
static uint8_t notification_flags = 0x00;
static uint8_t notOrInd = SEND_NOTIFICATION;
#endif


uint16_t att_mtu;
static uint8_t ll_data_len;
static uint8_t data[CHAR_SIZE];
static uint32_t command = 0;
static uint32_t g_seed;
static struct timer throughput_timer;
#if defined(SECURE_CONNECTION)
uint8_t force_rebond;
#endif

#if SET_PHY 
static uint8_t all_phy = 0x0; //ALL_PHYS = 0 --> host has preferences for both TX, RX 
static uint8_t ll_tx_phy = 0x1; //1 mbps 
static uint8_t ll_rx_phy = 0x1; //1 mbps 
#endif

#define ALIGN_UPTO_32BITS(VAL)            (((((unsigned int)(VAL)) - 1U) | \
                                            (sizeof(uint32_t) - 1U)) + 1U)
#if SET_L2CAP_COS 
static uint8_t l2c_cos_rx_sdu_buffer[L2C_COS_RX_SDU_BUFFER_SIZE_X_MTU(L2C_MAX_MTU)];
static uint8_t l2c_cos_tx_sdu_buffer[L2C_COS_NUM_TX_BUFFER][L2C_TX_BUFFER_SIZE];
static uint8_t l2c_cos_tx_sdu_buffer_idx = 0;
static uint16_t l2c_peer_mtu = 0;
#endif
static uint32_t th_down_counter = 0;
static uint32_t th_start_counter = 0;

/* This is CRC-16-CCITT (non-reflected poly, non-inverted input/output).
 * crc16() is only used to bootstrap an initial 256-entry lookup table. */
#define POLY 0x1021
uint16_t crc16(const void *in_data, uint32_t len)
{
    const uint8_t *d = in_data;
    uint16_t crc = d[0];

    for (uint32_t i = 1; i < len; i++)
    {
        crc = crc ^ (d[i] << 8);
        for (int j = 0; j < 8; j++)
        {
            if(crc & 0x8000)
            {
                crc = (crc << 1) ^ POLY;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

uint32_t fastrand()
{ 
    g_seed = (214013 * g_seed + 2531011);

    return (g_seed >> 16) & 0x7FFF;
}

void print_help()
{
    uint8_t j;

    printf("HELP:\n------------------------------------------\n");
    for (j = 0; j < CMD_TABLE_SIZE; j++)
    {
        printf("\tPress %c to %s\n", command_table[j].cmd, command_table[j].description);
    }
    printf("------------------------------------------\n");
}

void reset_context()
{
    if(APP_FLAG(TIMER_STARTED))
    {
        Timer_Reset(&throughput_timer);
    }
    memset(&rx_app_context, 0, sizeof(app_context_t));
    memset(&tx_app_context, 0, sizeof(app_context_t));
    memset(&l2c_cos_tx_app_context, 0, sizeof(app_context_t));
    memset(&l2c_cos_rx_app_context, 0, sizeof(app_context_t));
}

void print_app_flags()
{
    printf("\n---------------------------------------------\n");
#if CLIENT
    printf("CLIENT\n");
#else
    printf("SERVER\n");
#endif
    printf("Connection Handle = 0x%04x\n", connection_handle);
    printf("ATT MTU = %d bytes\n", att_mtu);
    printf("TX Link Layer Packet data length = %d bytes\n", ll_data_len);
    printf("TX average throughput = %d.%01d\n", PRINT_INT(tx_app_context.avg_val),PRINT_FLOAT(tx_app_context.avg_val));
    printf("RX average throughput = %d.%01d\n", PRINT_INT(rx_app_context.avg_val),PRINT_FLOAT(rx_app_context.avg_val));
    printf("Aggregated average throughput = %d.%01d\n", PRINT_INT( (tx_app_context.avg_val + rx_app_context.avg_val) ),PRINT_FLOAT( (tx_app_context.avg_val + rx_app_context.avg_val)));
#if SET_L2CAP_COS 
    printf("L2CAP COS TX average throughput = %d.%01d\n", PRINT_INT(l2c_cos_tx_app_context.avg_val),PRINT_FLOAT(l2c_cos_tx_app_context.avg_val));
    printf("L2CAP COS RX average throughput = %d.%01d\n", PRINT_INT(l2c_cos_rx_app_context.avg_val),PRINT_FLOAT(l2c_cos_rx_app_context.avg_val));
#endif 
#if SET_PHY 
    printf("LE PHY (1 --> 1 Mbps; 2 --> 2 Mbps); TX = %d, RX= %d\n", ll_tx_phy ,ll_rx_phy);
#endif 
#if SERVER
    printf("Notification type = %s\n", (notOrInd == SEND_INDICATION) ? "INDICATION" : "NOTIFICATION");
#endif
    printf("APP FLAGS : 0x%04x\n", app_flags);
    printf("Command buffer = 0x%04x\n", command);
    printf("\n---------------------------------------------\n");
    printf("\t %s = CONNECTED\n", PRINT_APP_FLAG_FORMAT(CONNECTED) );
    printf("\t %s = SET_CONNECTABLE\n", PRINT_APP_FLAG_FORMAT(SET_CONNECTABLE));
    printf("\t %s = NOTIFICATIONS_ENABLED\n", PRINT_APP_FLAG_FORMAT(NOTIFICATIONS_ENABLED));
    printf("\t %s = CONN_PARAM_UPD_SENT\n", PRINT_APP_FLAG_FORMAT(CONN_PARAM_UPD_SENT));
#if SERVER 
#if SET_L2CAP_COS 
    printf("\t %s = L2CAP_PARAM_UPD_SENT\n", PRINT_APP_FLAG_FORMAT(L2CAP_PARAM_UPD_SENT));
#endif 
#endif
    printf("\t %s = TX_BUFFER_FULL\n", PRINT_APP_FLAG_FORMAT(TX_BUFFER_FULL));
    printf("\t %s = ATT_MTU_START_EXCHANGE\n", PRINT_APP_FLAG_FORMAT(ATT_MTU_START_EXCHANGE));
    printf("\t %s = ATT_MTU_EXCHANGED\n", PRINT_APP_FLAG_FORMAT(ATT_MTU_EXCHANGED));
    printf("\t %s = START_READ_TX_CHAR_HANDLE\n", PRINT_APP_FLAG_FORMAT(START_READ_TX_CHAR_HANDLE));
    printf("\t %s = END_READ_TX_CHAR_HANDLE\n", PRINT_APP_FLAG_FORMAT(END_READ_TX_CHAR_HANDLE));
    printf("\t %s = START_READ_RX_CHAR_HANDLE\n", PRINT_APP_FLAG_FORMAT(START_READ_RX_CHAR_HANDLE));
    printf("\t %s = END_READ_RX_CHAR_HANDLE\n", PRINT_APP_FLAG_FORMAT(END_READ_RX_CHAR_HANDLE));
    printf("\t %s = SEND_COMMAND\n", PRINT_APP_FLAG_FORMAT(SEND_COMMAND));
    printf("\t %s = INDICATION_PENDING\n", PRINT_APP_FLAG_FORMAT(INDICATION_PENDING));
    printf("\n---------------------------------------------\n");
}

float calc_throughput(app_context_t *ctx_p, tClockTime time_now)
{
    float thr;
    tClockTime diff;

    if(ctx_p->data_size != 0)
    {
        diff = ctx_p->end_time - ctx_p->start_time;
        thr = ctx_p->data_size * 8.0 / diff;
        ctx_p->start_time = ctx_p->end_time = time_now;
        ctx_p->data_size = 0;
    }
    else
    {
        thr = 0;
    }
    ctx_p->avg_val = (ctx_p->avg_val > 0) ? ((0.95 * ctx_p->avg_val) + (0.05 * thr)) : thr;

    return thr;
    }

void print_throughput()
    {
    float tx_thr, rx_thr, l2c_cos_tx_thr, l2c_cos_rx_thr;
    tClockTime time_now;

    time_now = Clock_Time();
    tx_thr = calc_throughput(&tx_app_context, time_now);
    rx_thr = calc_throughput(&rx_app_context, time_now);
    l2c_cos_tx_thr = calc_throughput(&l2c_cos_tx_app_context, time_now);
    l2c_cos_rx_thr = calc_throughput(&l2c_cos_rx_app_context, time_now);

    printf("%d sec: TX=%d.%01d kbps, RX=%d.%01d kbps, COSTX=%d.%01d kbps, COSRX=%d.%01d kbps",
                                    (int)(time_now / 1000),
                                    PRINT_INT((float)tx_thr),PRINT_FLOAT((float)tx_thr),
                                    PRINT_INT((float)rx_thr),PRINT_FLOAT((float)rx_thr),
                                    PRINT_INT((float)l2c_cos_tx_thr),PRINT_FLOAT((float)l2c_cos_tx_thr),
                                    PRINT_INT((float)l2c_cos_rx_thr),PRINT_FLOAT((float)l2c_cos_rx_thr));

    if (rx_app_context.corrupted_pckts != 0)
    {
        printf(", CRPT=%d", (int)rx_app_context.corrupted_pckts);
    }
    if (rx_app_context.lost_pckts != 0)
    {
        printf(", LOST=%d", (int)rx_app_context.lost_pckts);
    }
    if (rx_app_context.duplicated_pckts != 0)
    {
        printf(", DUP=%d", (int)rx_app_context.duplicated_pckts);
    }

    if (l2c_cos_rx_app_context.corrupted_pckts != 0)
    {
        printf(", COS_CRPT=%d", (int)l2c_cos_rx_app_context.corrupted_pckts);
    }
    if (l2c_cos_rx_app_context.lost_pckts != 0)
    {
        printf(", COS_LOST=%d", (int)l2c_cos_rx_app_context.lost_pckts);
    }
    if (l2c_cos_rx_app_context.duplicated_pckts != 0)
    {
        printf(", COS_DUP=%d", (int)l2c_cos_rx_app_context.duplicated_pckts);
    }

    printf("\r\n");
    tx_app_context.m_buffer_miss = 0;
}

uint8_t Configure_Advertising(void)
{
    uint8_t ret; 
    /* NOTE: Updated original Server advertising data in order to be also recognized by “BLE Sensor” app Client */
    tBDAddr bdaddr = {SERVER_ADDRESS};

    static uint8_t adv_data[] = { 
      0x02,AD_TYPE_FLAGS, FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE|FLAG_BIT_BR_EDR_NOT_SUPPORTED,
      2,                      /* Length of AD type Transmission Power */
      0x0A, 0x00,             /* Transmission Power = 0 dBm */ 
      LOCAL_NAME_LENGTH+1,    /* Length of AD type Complete Local Name */
      0x09,                   /* AD type Complete Local Name */ 
      LOCAL_NAME,             /* Local Name */            
      13,                     /* Length of AD type Manufacturer info */
      0xFF,                   /* AD type Manufacturer info */
      0x01,                   /* Protocol version */
      0x05,		      /* Device ID: 0x05 = STEVAL-BCN002V1 Board */
      0x00,                   /* Feature Mask byte#1 */
      0x00,                   /* Feature Mask byte#2 */
      0x00,                   /* Feature Mask byte#3 */
      0x00,                   /* Feature Mask byte#4 */
      0x00,                   /* BLE MAC start */
      0x00, 
      0x00, 
      0x00, 
      0x00, 
      0x00                    /* BLE MAC stop */
    };
    
    
    for (int var = 0; var < 6; ++var) {
      adv_data[26 - var] = bdaddr[var];
    }
    
      
    ret = aci_gap_set_advertising_configuration(0, GAP_MODE_GENERAL_DISCOVERABLE,
                                                ADV_PROP_CONNECTABLE|ADV_PROP_SCANNABLE|ADV_PROP_LEGACY,
                                                ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
                                                ADV_CH_ALL,
                                                0,NULL,
                                                ADV_NO_WHITE_LIST_USE,
                                                0, 1, 0, 1, 0, 0);
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf("%s Error in aci_gap_set_advertising_configuration(): 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    printf("Advertising configuration %02X\n", ret);
    
    ret = aci_gap_set_advertising_data(0, ADV_COMPLETE_DATA, sizeof(adv_data), adv_data);
        if(ret != BLE_STATUS_SUCCESS)
    {
        printf("%s Error in aci_gap_set_advertising_data(): 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    printf("Set advertising data %02X\n", ret);  

    return (ret);
  
}

uint8_t DeviceInit(void)
{
    uint8_t ret;
    uint16_t service_handle;
    uint16_t dev_name_char_handle;
    uint16_t appearance_char_handle;
    uint8_t name[] = {LOCAL_NAME};

#if SERVER
    uint8_t role = GAP_PERIPHERAL_ROLE;
    uint8_t bdaddr[] = {SERVER_ADDRESS};
#else
    uint8_t role = GAP_CENTRAL_ROLE;
    uint8_t bdaddr[] = {CLIENT_ADDRESS};
#endif

    reset_context();
    /* Configure Public address */
    ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf("%s Setting BD_ADDR failed: 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }

#ifndef TX_POWER
#define TX_POWER (24) /* Set the TX power to 0 dBm */
#endif
    aci_hal_set_tx_power_level(0, TX_POWER);

    /* GATT Init */
    ret = aci_gatt_srv_init();
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in aci_gatt_srv_init(): 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s aci_gatt_srv_init() --> SUCCESS\r\n", DEVICE_NAME);
    }

    /* GAP Init */
    ret = aci_gap_init(role, 0x00, 0x08, PUBLIC_ADDR, &service_handle,
                       &dev_name_char_handle, &appearance_char_handle); 
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in aci_gap_init() 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s aci_gap_init() --> SUCCESS\r\n", DEVICE_NAME);
    }

    /* Set the device name */
    ret = Gap_profile_set_dev_name(0, sizeof(name), name);
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in Gatt Update characteristic value 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s aci_gatt_update_char_value_ext() --> SUCCESS\r\n", DEVICE_NAME);
    }

#if defined(SECURE_CONNECTION)
    force_rebond = 0;
    ret = aci_gap_clear_security_db();
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in aci_gap_clear_security_db() 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s aci_gap_clear_security_db() --> SUCCESS\r\n", DEVICE_NAME);
    }

    uint8_t event_mask[8] ={0xDF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    ret = hci_le_set_event_mask(event_mask);
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in hci_le_set_event_mask() 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s hci_le_set_event_mask() --> SUCCESS\r\n", DEVICE_NAME);
    }

    ret = aci_gap_set_io_capability(0x02); //keyboard only
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in aci_gap_set_io_capability() 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s aci_gap_set_io_capability() --> SUCCESS\r\n", DEVICE_NAME);
    }

    uint8_t Bonding_Mode = 0x01;
    uint8_t MITM_Mode = 0x01; //It requires authentication

    /* LE Secure connections:
            0x00: Not supported
            0x01: Supported but optional (i.e. a Legacy Pairing may be accepted)
            0x02: Supported but mandatory (i.e. do not accept Legacy Pairing but only Secure Connections v.4.2 Pairing).
    */
    uint8_t SC_Support = 0x02;
    uint8_t KeyPress_Notification_Support = 0x00;  //keyPress Notification feature Not Supported
    uint8_t Min_Encryption_Key_Size = 0x07;
    uint8_t Max_Encryption_Key_Size = 0x10;
    uint8_t Use_Fixed_Pin = 0x00; //0 means fixed pin!
    uint32_t Fixed_Pin = 123456;
    ret = aci_gap_set_authentication_requirement(Bonding_Mode,
                                                 MITM_Mode,
                                                 SC_Support,
                                                 KeyPress_Notification_Support,
                                                 Min_Encryption_Key_Size,
                                                 Max_Encryption_Key_Size,
                                                 Use_Fixed_Pin,
                                                 Fixed_Pin);
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("%s Error in aci_gap_set_authentication_requirement() 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf ("%s aci_gap_set_authentication_requirement() --> SUCCESS\r\n", DEVICE_NAME);
    }
#endif
#if  SERVER
    ret = Add_Throughput_Service();
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf("%s Error in Add_Throughput_Service 0x%02x\r\n", DEVICE_NAME, ret);
        return ret;
    }
    else
    {
        printf("%s Add_Throughput_Service() --> SUCCESS\r\n", DEVICE_NAME);
    }
#endif

    
#if SERVER
    
  Configure_Advertising(); 
  
  
#else
  
  ret = aci_gap_set_scan_configuration(DUPLICATE_FILTER_ENABLED,SCAN_ACCEPT_ALL, LE_1M_PHY_BIT, PASSIVE_SCAN, 0x4000, 0x4000);
  
  printf("Scan configuration %02X\n", ret);
    
  ret = aci_gap_set_connection_configuration(LE_1M_PHY_BIT, 40, 40, 0, 100, 2000, 2000); 
  
  printf("Connection configuration %02X\n", ret);
  
#endif
  
    att_mtu = ATT_DEF_SIZE;
    COMMAND_RESET();
    APP_FLAG_RESET();
    APP_FLAG_SET(SET_CONNECTABLE);

    return BLE_STATUS_SUCCESS;
}

tBleStatus Make_Connection(void)
{
    tBleStatus ret;

#if CLIENT
    tBDAddr bdaddr = {SERVER_ADDRESS};
    
    ret = aci_gap_create_connection(LE_1M_PHY_BIT, PUBLIC_ADDR, bdaddr);

    if(ret != BLE_STATUS_SUCCESS)
    {
        printf("%s Error while starting connection: 0x%04x\r\n", DEVICE_NAME, ret);
        Clock_Wait(100);

        return ret;
    }
#else

	aci_gap_set_scan_response_data(0, 0, NULL);
   
    Advertising_Set_Parameters_t adv_set = {0};

    ret = aci_gap_set_advertising_enable(ENABLE, 1, &adv_set);
    if(ret != BLE_STATUS_SUCCESS)
    {
        printf ("Error in aci_gap_set_advertising_enable(): 0x%02x\r\n", ret);
        return ret;
    }
    
#endif

    return BLE_STATUS_SUCCESS;
}

#if CLIENT
tBleStatus set_notification_type(uint16_t connection_handle, uint16_t tx_handle, uint8_t notification_type)
{
    static uint8_t client_char_conf_data[] = {0x00, 0x00};
    tBleStatus ret;

    APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
    client_char_conf_data[0] = notification_type;
    ret = aci_gatt_clt_write(connection_handle, tx_handle + 2, 2, client_char_conf_data);
    if(ret != BLE_STATUS_SUCCESS)
    {
        return ret;
    }
    APP_FLAG_SET(NOTIFICATIONS_ENABLED);
    if(!APP_FLAG(TIMER_STARTED))
    {
        Timer_Set(&throughput_timer, CLOCK_SECOND * THROUGHPUT_TIMER_VAL_SEC);
        APP_FLAG_SET(TIMER_STARTED);
    }

    printf("Subscribed to receive %s\n", (notification_type == SEND_NOTIFICATION) ? "NOTIFICATION" : "INDICATION");

    return BLE_STATUS_SUCCESS;
}
#endif

void APP_Tick(void)
{
    tBleStatus ret;

    if(APP_FLAG(SET_CONNECTABLE))
    {
        ret = Make_Connection();
        if (ret == BLE_STATUS_SUCCESS)
        {
            APP_FLAG_CLEAR(SET_CONNECTABLE);
        }
    }
#if defined(SECURE_CONNECTION)
#if CLIENT
    else if(APP_FLAG(DEV_PAIRING_REQ))
    {
        ret = aci_gap_send_pairing_req(connection_handle, force_rebond);
        if (ret == BLE_STATUS_SUCCESS)
        {
            printf("Sent aci_gap_send_pairing_req() connection_handle = 0x%04x, force_rebond = 0x%02x\n",
                                                connection_handle, force_rebond);
            APP_FLAG_CLEAR(DEV_PAIRING_REQ);
            force_rebond = 0;
        }
        else
        {
            printf("Fail to call aci_gap_send_pairing_req() ret = 0x%02x\n", ret);
        }
    }
#endif
    else if(APP_FLAG(CONNECTED) && APP_FLAG(DEV_PAIRED))
#else
    else if(APP_FLAG(CONNECTED))
#endif
    {
        if(APP_FLAG(TIMER_STARTED))
        {
            if(Timer_Expired(&throughput_timer))
            {
                Timer_Set(&throughput_timer, CLOCK_SECOND * THROUGHPUT_TIMER_VAL_SEC);
                print_throughput();
            }
        }

        if(APP_FLAG(INDICATION_PENDING))
        {
            ret = aci_gatt_clt_confirm_indication(connection_handle);
            if(ret == BLE_STATUS_SUCCESS)
            {
                APP_FLAG_CLEAR(INDICATION_PENDING);
            }
        }
        
        if(COMMAND_IS_SET(CMD_TH_SLOW_TOGGLE))
        {
            if (th_start_counter > 0)
            {
                printf("Disable slow throughput \r\n");
                th_start_counter = 0;
            }
            else
            {
                printf("Enable slow throughput \r\n");
                th_start_counter = THROUGHPUT_DOWN_COUNTER;
            }
            
            th_down_counter = th_start_counter;

            COMMAND_CLEAR(CMD_TH_SLOW_TOGGLE);
        }

        if (COMMAND_IS_SET(CMD_L2C_COS_CHANGE_MTU))
        {
            l2c_cos_mtu_table_index++;
            if (l2c_cos_mtu_table_index == L2C_COS_MTU_TABLE_SIZE)
            {
                l2c_cos_mtu_table_index = 0;
            }
            l2c_cos_local_mtu = l2c_cos_mtu_table[l2c_cos_mtu_table_index];
            printf("L2C COS MTU set to %d \r\n", l2c_cos_local_mtu);
            COMMAND_CLEAR(CMD_L2C_COS_CHANGE_MTU);
        }
        if (COMMAND_IS_SET(CMD_L2C_COS_CHANGE_MPS))
        {
            l2c_cos_mps_table_index++;
            if (l2c_cos_mps_table_index == L2C_COS_MPS_TABLE_SIZE)
            {
                l2c_cos_mps_table_index = 0;
            }
            l2c_cos_local_mps = l2c_cos_mps_table[l2c_cos_mps_table_index];
            printf("L2C COS MPS set to %d \r\n", l2c_cos_local_mps);
            COMMAND_CLEAR(CMD_L2C_COS_CHANGE_MPS);
        }

#if SET_PHY 
        if(COMMAND_IS_SET(CMD_PHY_READ))
        {
          uint8_t TX_PHY, RX_PHY; 
          ret = hci_le_read_phy(connection_handle,
                                &TX_PHY,
                                &RX_PHY); 
          if(ret != BLE_STATUS_SUCCESS)
          {
            printf ("Error in hci_le_read_phy(0x%02x)\n",connection_handle);
          }
          else
          {
            printf ("TX PHY: 0x%0x, RX PHY: 0x%0x\n",TX_PHY,RX_PHY);
          }
          COMMAND_CLEAR(CMD_PHY_READ);
        }

        if(COMMAND_IS_SET(CMD_PHY_CODED_TX))
        {
          ll_tx_phy = 0x4; //TX: Coded PHY

          ret = hci_le_set_phy(connection_handle, all_phy, ll_tx_phy,ll_rx_phy,0);
          if(ret != BLE_STATUS_SUCCESS)
          {
            printf ("Error in hci_le_set_phy() to TX PHY Coded: 0x%02x\n",ret);
          }
          COMMAND_CLEAR(CMD_PHY_CODED_TX);
        }
        if(COMMAND_IS_SET(CMD_PHY_CODED_RX))
        {
          ll_rx_phy = 0x4; //RX: Coded PHY

          ret = hci_le_set_phy(connection_handle, all_phy, ll_tx_phy,ll_rx_phy,0);
          if(ret != BLE_STATUS_SUCCESS)
          {
            printf ("Error in hci_le_set_phy() to RX PHY Coded: 0x%02x\n",ret);
          }
          COMMAND_CLEAR(CMD_PHY_CODED_RX);
        }
        if(COMMAND_IS_SET(CMD_PHY_1M_TX))
        {
          ll_tx_phy = 0x1; //TX: 1Mbps
          
          ret = hci_le_set_phy(connection_handle, all_phy, ll_tx_phy,ll_rx_phy,0);
          if(ret != BLE_STATUS_SUCCESS)
          {
            printf ("Error in hci_le_set_phy() to TX PHY 1 Mpbs: 0x%02x\n",ret);
          }
          COMMAND_CLEAR(CMD_PHY_1M_TX);
          
        }
        if(COMMAND_IS_SET(CMD_PHY_1M_RX))
        {
          ll_rx_phy = 0x1; //RX: 1Mbps
          ret = hci_le_set_phy(connection_handle, all_phy, ll_tx_phy,ll_rx_phy,0);
          if(ret != BLE_STATUS_SUCCESS)
          {
              printf ("Error in hci_le_set_phy() RX PHY to 1 Mpbs: 0x%02x\n",ret);
          }
          COMMAND_CLEAR(CMD_PHY_1M_RX);
        }
        if(COMMAND_IS_SET(CMD_PHY_2M_TX))
        {
          ll_tx_phy = 0x2; //TX: 2Mbps
          ret = hci_le_set_phy(connection_handle, all_phy, ll_tx_phy,ll_rx_phy,0);
          if(ret != BLE_STATUS_SUCCESS)
          {
              printf ("Error in hci_le_set_phy() TX PHY to 2 Mpbs: 0x%02x\n",ret);
          }
          
          COMMAND_CLEAR(CMD_PHY_2M_TX);
        }
        if(COMMAND_IS_SET(CMD_PHY_2M_RX))
        {
          ll_rx_phy = 0x2; //RX: 2Mbps
          ret = hci_le_set_phy(connection_handle, all_phy, ll_tx_phy,ll_rx_phy,0);
          if(ret != BLE_STATUS_SUCCESS)
          {
              printf ("Error in hci_le_set_phy() RX PHY to 2 Mpbs:0x%02x\n",ret);
          }
          COMMAND_CLEAR(CMD_PHY_2M_RX);
        }
#endif 
#if SET_L2CAP_COS 
        if(COMMAND_IS_SET(CMD_L2C_COS_TX))
        {
            if(APP_FLAG(L2C_COS_TX))
            {
                ret = aci_l2cap_disconnect(connection_handle, L2C_COS_CID);
                if (ret != BLE_STATUS_SUCCESS)
                {
                    printf ("Error calling aci_l2cap_disconnect()  = 0x%02x\n", ret);
                }
                else
                {
                    printf ("Executing aci_l2cap_disconnect()\r\n");
                }
            }
            else
            {
                ret = aci_l2cap_cfc_connection_req(connection_handle,
                                                   L2C_COS_SPSM,
                                                   L2C_COS_CID,
                                                   l2c_cos_local_mtu,
                                                   l2c_cos_local_mps,
                                                   L2C_COS_CFC_POLICY,
                                                   L2C_COS_RX_SDU_BUFFER_SIZE_X_MTU(l2c_cos_local_mtu),
                                                   l2c_cos_rx_sdu_buffer);
                if (ret != BLE_STATUS_SUCCESS)
                {
                    printf ("Error calling aci_l2cap_cfc_connection_req()  = 0x%02x\n", ret);
                }
                else
                {
                    printf ("Executing aci_l2cap_cfc_connection_req()\r\n");
                }
            }

            COMMAND_CLEAR(CMD_L2C_COS_TX);
        }
#endif 
        else if(COMMAND_IS_SET(CMD_PRINT_FLAGS))
        {
            print_app_flags();
            COMMAND_CLEAR(CMD_PRINT_FLAGS);
        }
        else if((COMMAND_IS_SET(CMD_DATA_LEN_UPDATE_REQ_LOW) || COMMAND_IS_SET(CMD_DATA_LEN_UPDATE_REQ_MID) ||
            COMMAND_IS_SET(CMD_DATA_LEN_UPDATE_REQ_HIGH)))
        {
            uint8_t data_len_req;
            if(COMMAND_IS_SET(CMD_DATA_LEN_UPDATE_REQ_LOW))
            {
                data_len_req = LL_DATA_LEN_LOW;
                COMMAND_CLEAR(CMD_DATA_LEN_UPDATE_REQ_LOW);
            }
            else if(COMMAND_IS_SET(CMD_DATA_LEN_UPDATE_REQ_MID))
            {
                data_len_req = LL_DATA_LEN_MID;
                COMMAND_CLEAR(CMD_DATA_LEN_UPDATE_REQ_MID);
            }
            else if(COMMAND_IS_SET(CMD_DATA_LEN_UPDATE_REQ_HIGH))
            {
                data_len_req = LL_DATA_LEN_HIGH;
                COMMAND_CLEAR(CMD_DATA_LEN_UPDATE_REQ_HIGH);
            }
            printf("%s Send data len update request with value %d\n", DEVICE_NAME, data_len_req);
            ret = hci_le_set_data_length(connection_handle, data_len_req, TX_OCTETS_TO_TIME(data_len_req));
            if(ret != BLE_STATUS_SUCCESS)
            {
                printf ("%s Error in hci_le_set_data_length()\n", DEVICE_NAME);
            }
        }
        else if(COMMAND_IS_SET(CMD_ATT_MTU))
        {
            if(!APP_FLAG(ATT_MTU_EXCHANGED) && !APP_FLAG(ATT_MTU_START_EXCHANGE) && !APP_FLAG(TX_BUFFER_FULL))
            {
                ret = aci_gatt_clt_exchange_config(connection_handle);
                if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
                {
                    /* Radio is busy (buffer full). */
                    APP_FLAG_SET(TX_BUFFER_FULL);
                    tx_app_context.m_buffer_miss++;
                }
                else if(ret != BLE_STATUS_SUCCESS)
                {
                    printf ("Error in aci_gatt_exchange_config() ret = 0x%02xr\n", ret);
                }
                else
                {
                    APP_FLAG_SET(ATT_MTU_START_EXCHANGE);
                    COMMAND_CLEAR(CMD_ATT_MTU);
                }
            }
            else
            {
                printf ("Error aci_gatt_exchange_config can be called just one time\n");
                COMMAND_CLEAR(CMD_ATT_MTU);
            }
        }
        else
#if CLIENT
        if(COMMAND_IS_SET(CMD_BIDIRECT))
        {
            if(APP_FLAG(SEND_COMMAND))
            {
                APP_FLAG_CLEAR(SEND_COMMAND);
                printf("Set unidirectional mode\n");
            }
            else
            {
                APP_FLAG_SET(SEND_COMMAND);
                printf("Set bidirectional mode\n");
                if(!APP_FLAG(TIMER_STARTED))
                {
                    Timer_Set(&throughput_timer, CLOCK_SECOND * THROUGHPUT_TIMER_VAL_SEC);
                    APP_FLAG_SET(TIMER_STARTED);
                }
            }
            COMMAND_CLEAR(CMD_BIDIRECT);
        }
        else if(COMMAND_IS_SET(CMD_SEND_INDICATION))
        {
            ret = set_notification_type(connection_handle, tx_handle, SEND_INDICATION);
            if(ret == BLE_STATUS_SUCCESS)
            {
                COMMAND_CLEAR(CMD_SEND_INDICATION);
                printf("Sent Indication subscription\n");
            }
        }
        else if(COMMAND_IS_SET(CMD_SEND_NOTIFICATION))
        {
            ret =  set_notification_type(connection_handle, tx_handle, SEND_NOTIFICATION);
            if(ret == BLE_STATUS_SUCCESS)
            {
                COMMAND_CLEAR(CMD_SEND_NOTIFICATION);
                printf("Sent Notification subscription\n");
            }
        }
        else if(!APP_FLAG(END_READ_TX_CHAR_HANDLE))
        {
            /* Start TX handle Characteristic discovery if not yet done */
            if(!APP_FLAG(START_READ_TX_CHAR_HANDLE))
            {
                /* Discovery TX characteristic handle by UUID 128 bits */
                UUID_t uuid = {.UUID_128 = {TX_CHR_UUID}};
                ret = aci_gatt_clt_disc_char_by_uuid(connection_handle,
                                                     0x01, 0xFFFF, 2, &uuid);
                if(ret != BLE_STATUS_SUCCESS)
                {
                    printf ("Error in aci_gatt_disc_char_by_uuid() for TX characteristic: 0x%02xr\n", ret);
                }
                else
                {
                    APP_FLAG_SET(START_READ_TX_CHAR_HANDLE);
                }
            }
        }
        /* Start RX handle Characteristic discovery if not yet done */
        else if(!APP_FLAG(END_READ_RX_CHAR_HANDLE))
        {
            /* Discovery RX characteristic handle by UUID 128 bits */
            if(!APP_FLAG(START_READ_RX_CHAR_HANDLE))
            {
                /* Discovery TX characteristic handle by UUID 128 bits */
                UUID_t uuid = {.UUID_128 = {RX_CHR_UUID}};
                ret = aci_gatt_clt_disc_char_by_uuid(connection_handle,
                                                     0x01, 0xFFFF, 2, &uuid);
                if(ret != BLE_STATUS_SUCCESS)
                {
                    printf ("Error in aci_gatt_disc_char_by_uuid() for RX characteristic: 0x%02xr\n", ret);
                }
                else
                {
                    APP_FLAG_SET(START_READ_RX_CHAR_HANDLE);
                }
            }
        }
        else if(!APP_FLAG(NOTIFICATIONS_ENABLED))
        {
            /**
             * Subscribe to receive notification
             */
            set_notification_type(connection_handle, tx_handle, SEND_NOTIFICATION);
        }
        else if(APP_FLAG(SEND_COMMAND))
        {
            if (th_down_counter == 0)
            {
            if (!APP_FLAG(TX_BUFFER_FULL))
            {
                uint8_t i;
                uint16_t *v, data_len;
                pckt_t *pckt;

                pckt = (pckt_t *)data;
                pckt->header.sn = tx_app_context.packets;
                v = (uint16_t *)pckt->data;
                data_len = att_mtu - 3 - sizeof(header_t);
                for (i = 0; i < (data_len/ 2); i++)
                {
                    v[i] = (uint16_t)(fastrand() & 0xFFFF);
                    //v[i] = i;
                }
                pckt->header.crc = crc16(pckt->data, data_len);


                tBleStatus ret = aci_gatt_clt_write_without_resp(connection_handle,
                                                             rx_handle + 1,
                                                             data_len + sizeof(header_t),
                                                             data);  
                if(ret == BLE_STATUS_SUCCESS)
                {
                  if(tx_app_context.data_size == 0)
                  {
                    tx_app_context.start_time = Clock_Time();
                  }
                  tx_app_context.packets++;
                  tx_app_context.data_size += (att_mtu - 3);
                  tx_app_context.end_time = Clock_Time();
                }                
                else if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
                {
                  /* Radio is busy (buffer full). */
                  APP_FLAG_SET(TX_BUFFER_FULL);
                  tx_app_context.m_buffer_miss++;
                } 
                else
                {
                  printf("Error 0x%02X\r\n", ret);
                }
                    th_down_counter = th_start_counter;
            }
            else
            {
              tx_app_context.m_buffer_miss++;
            }
        }
            else
            {
                th_down_counter--;
            }
        }
#else  //SERVER
        if (COMMAND_IS_SET(CMD_DISABLE_NOTIFY))
        {
            if(APP_FLAG(DISABLE_NOTIFY))
            {
                printf("Enable notification sending\r\n");
                APP_FLAG_CLEAR(DISABLE_NOTIFY);
            }
            else
            {
                printf("Disable notification sending\r\n");
                APP_FLAG_SET(DISABLE_NOTIFY);
            }
            COMMAND_CLEAR(CMD_DISABLE_NOTIFY);
        }
   
        if(COMMAND_IS_SET(CMD_CONN_UPDATE) && !APP_FLAG(L2CAP_PARAM_UPD_SENT))
        {
            printf("Send connection parameter update\n");
            ret = aci_l2cap_connection_parameter_update_req(connection_handle,
                                                            8, 16, 0, 600);
            if(ret != BLE_STATUS_SUCCESS)
            {
                printf ("Error in aci_l2cap_connection_parameter_update_req(): 0x%02xr\n", ret);
            }
            else
            {
                APP_FLAG_SET(L2CAP_PARAM_UPD_SENT);
                COMMAND_CLEAR(CMD_CONN_UPDATE);
            }
        }
        else if(COMMAND_IS_SET(CMD_FLUSH))
        {
            if (notification_flags == BLE_GATT_SRV_NOTIFY_FLAG_NOTIFICATION)
            {
                printf ("Enabling flushable notifications\n");
                notification_flags = BLE_GATT_SRV_NOTIFY_FLAG_FLUSHABLE_NOTIFICATION;
            }
            else if (notification_flags == BLE_GATT_SRV_NOTIFY_FLAG_FLUSHABLE_NOTIFICATION)
            {
                printf ("Disabling flushable notifications\n");
                notification_flags = BLE_GATT_SRV_NOTIFY_FLAG_NOTIFICATION;
            }
          COMMAND_CLEAR(CMD_FLUSH);
        }
        else if((!APP_FLAG(DISABLE_NOTIFY)) &&
            (APP_FLAG(NOTIFICATIONS_ENABLED) && !APP_FLAG(INDICATION_PENDING)))
        {
            if (th_down_counter == 0)
        {
            if (!APP_FLAG(TX_BUFFER_FULL))
            {
                uint8_t i;
                uint16_t *v, data_len;
                pckt_t *pckt;

                pckt = (pckt_t *)data;
                pckt->header.sn = tx_app_context.packets;
                v = (uint16_t *)pckt->data;
                data_len = att_mtu - 3 - sizeof(header_t);
                for (i = 0; i < (data_len / 2); i++)
                {
                    v[i] = (uint16_t)(fastrand() & 0xFFFF);
                    //v[i] = i;
                }
                pckt->header.crc = crc16(pckt->data, data_len);

                ret = aci_gatt_srv_notify(connection_handle, TXCharHandle + 1U,
                                     notification_flags,
                                     data_len + sizeof(header_t), data);

                if(ret == BLE_STATUS_INSUFFICIENT_RESOURCES)
                {
                    /* Radio is busy (buffer full). */
                    APP_FLAG_SET(TX_BUFFER_FULL);
                    tx_app_context.m_buffer_miss++;
                }
                else if(ret == BLE_STATUS_SUCCESS)
                {
                    if(tx_app_context.data_size == 0)
                    {
                        tx_app_context.start_time = Clock_Time();
                    }
                    if(notOrInd == SEND_INDICATION)
                    {
                        APP_FLAG_SET(INDICATION_PENDING);
                    }
                    tx_app_context.data_size += (att_mtu - 3);
                    tx_app_context.end_time = Clock_Time();
                    tx_app_context.packets++;
                }
                    th_down_counter = th_start_counter;
            }
            else
            {
                tx_app_context.m_buffer_miss++;
            }
        }
            else
            {
                th_down_counter--;
            }
        }
#endif
        else if (APP_FLAG(CONNECTED) && !APP_FLAG(FEAT_EXCHANGED) && !APP_FLAG(FEAT_EXCHANGE_REQ))
        {
          ret = hci_le_read_remote_used_features(connection_handle);
          if(ret == BLE_STATUS_SUCCESS)
          {
            APP_FLAG_SET(FEAT_EXCHANGE_REQ);
    }
}
#if SET_L2CAP_COS 
        if(APP_FLAG(L2C_COS_TX))
        {
            uint16_t i;
            uint16_t *v, data_len;
            pckt_t *pckt;

            pckt = (pckt_t *)&l2c_cos_tx_sdu_buffer[l2c_cos_tx_sdu_buffer_idx];
            pckt->header.sn = l2c_cos_tx_app_context.packets;
            v = (uint16_t *)pckt->data;
            data_len = MIN(l2c_peer_mtu, L2C_COS_MTU) - sizeof(header_t);
            for (i = 0; i < (data_len / 2); i++)
            {
                v[i] = (uint16_t)(fastrand() & 0xFFFF);
            }
            pckt->header.crc = crc16(pckt->data, data_len);
            ret = aci_l2cap_transmit_sdu_data(connection_handle,
                                              L2C_COS_CID,
                                              data_len + sizeof(header_t),
                                              (uint8_t *)pckt);
            if (ret == BLE_STATUS_SUCCESS)
            {
                l2c_cos_tx_sdu_buffer_idx++;
                if (l2c_cos_tx_sdu_buffer_idx == L2C_COS_NUM_TX_BUFFER)
                {
                    l2c_cos_tx_sdu_buffer_idx = 0;
                }
                if(l2c_cos_tx_app_context.data_size == 0)
                {
                    l2c_cos_tx_app_context.start_time = Clock_Time();
                }
                l2c_cos_tx_app_context.packets++;
                l2c_cos_tx_app_context.data_size += data_len + sizeof(header_t);
                l2c_cos_tx_app_context.end_time = Clock_Time();
            }
        }
#endif
    }
}


void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes)
{
    uint8_t i, j;

    for (i = 0; i < Nb_bytes; i++)
    {
        for (j = 0; j < CMD_TABLE_SIZE; j++)
        {
            if(data_buffer[i] == command_table[j].cmd)
            {
                if (command_table[j].command_flag == CMD_RESET)
                {
                  NVIC_SystemReset();
                }
                else
                {
                COMMAND_SET(command_table[j].command_flag);
                }
                break;
            }
        }
        if((j == CMD_TABLE_SIZE) || COMMAND_IS_SET(CMD_HELP))
        {
            print_help();
            COMMAND_CLEAR(CMD_HELP);
        }
       // printf("CMD_TABLE_SIZE = %d received data_buffer[%d] = %c Nb_bytes = %d\n", CMD_TABLE_SIZE, i, data_buffer[i], Nb_bytes);
    }
}

void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)

{
    if(Status != BLE_STATUS_SUCCESS)
        return;  
  
    connection_handle = Connection_Handle;
    att_mtu = ATT_DEF_SIZE;
    ll_data_len = LL_DEFAULT_DATA_LEN;
    printf("Device is connected with Connection_Handle = 0x%04x\n", Connection_Handle);
    APP_FLAG_SET(CONNECTED);
#if defined(SECURE_CONNECTION) && CLIENT
    APP_FLAG_SET(DEV_PAIRING_REQ);
#endif
    APP_FLAG_CLEAR(L2CAP_PARAM_UPD_SENT);
}

void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{
    printf("Device disconnected from Connection_Handle = 0x%04x, Reason = 0x%02x\r\n", Connection_Handle, Reason);

    reset_context();
    APP_FLAG_RESET();
    COMMAND_RESET();
#ifndef NO_RECONNECT
    /* Make the device connectable again. */
    APP_FLAG_SET(SET_CONNECTABLE);
#endif
}

/*******************************************************************************
 * Function Name  : hci_le_enhanced_connection_complete_event.
 * Description    : This event indicates that a new connection has been created
 * Input          : See file bluenrg_lp_events.h
 * Output         : See file bluenrg_lp_events.h
 * Return         : See file bluenrg_lp_events.h
 *******************************************************************************/
void hci_le_enhanced_connection_complete_event(uint8_t Status,
                                               uint16_t Connection_Handle,
                                               uint8_t Role,
                                               uint8_t Peer_Address_Type,
                                               uint8_t Peer_Address[6],
                                               uint8_t Local_Resolvable_Private_Address[6],
                                               uint8_t Peer_Resolvable_Private_Address[6],
                                               uint16_t Conn_Interval,
                                               uint16_t Conn_Latency,
                                               uint16_t Supervision_Timeout,
                                               uint8_t Master_Clock_Accuracy)
{
  
  hci_le_connection_complete_event(Status,
                                   Connection_Handle,
                                   Role,
                                   Peer_Address_Type,
                                   Peer_Address,
                                   Conn_Interval,
                                   Conn_Latency,
                                   Supervision_Timeout,
                                   Master_Clock_Accuracy);
}

void handle_value_update(app_context_t *context_p,
                         uint16_t data_len,
                         uint8_t data[])
{
    uint16_t crc;
    pckt_t *pckt;
    uint32_t sn;
    tClockTime time_now;

    time_now = Clock_Time();
    pckt = (pckt_t *)&data[0];
    sn = pckt->header.sn;

    if(context_p->not_first_packet == 0)
    {
        context_p->start_time = time_now;
        context_p->sn = (uint32_t)(sn - 1);
        context_p->not_first_packet = 1;
    }


    if(sn == context_p->sn)
    {
        printf("Duplicated 0x04%x !!!\n", sn);
        context_p->duplicated_pckts++;
    }
    else if(sn != (context_p->sn + 1))
    {
        if(sn <  context_p->sn)
        {
            context_p->duplicated_pckts++;
        }
        else
        {
            context_p->lost_pckts += sn - (context_p->sn + 1);
        }
    }

    crc = crc16(pckt->data, data_len - sizeof(header_t));
    if(crc != pckt->header.crc)
    {
        context_p->corrupted_pckts++;
    }
    context_p->sn = sn;
    context_p->data_size += data_len;
    context_p->end_time = Clock_Time();
}

void aci_gatt_srv_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t handle,
                                       uint16_t data_length,
                                       uint8_t att_data[])
{
#if SERVER
    if(handle == TXCharHandle + 2)
    {
        if(att_data[0] == 0x1)
        {
            APP_FLAG_SET(NOTIFICATIONS_ENABLED);
            notification_flags = BLE_GATT_SRV_NOTIFY_FLAG_NOTIFICATION;
            printf ("Client has request to receive NOTIFICATION\n");
        }
        else if(att_data[0] == 0x2)
        {
            APP_FLAG_SET(NOTIFICATIONS_ENABLED);
            notification_flags = BLE_GATT_SRV_NOTIFY_FLAG_INDICATION;
            printf ("Client has request to receive INDICATION\n");
        }
        else
        {
            APP_FLAG_CLEAR(NOTIFICATIONS_ENABLED);
            APP_FLAG_CLEAR(INDICATION_PENDING);
            notification_flags = 0x00U;
        }

        if(!APP_FLAG(TIMER_STARTED))
        {
            Timer_Set(&throughput_timer, CLOCK_SECOND * THROUGHPUT_TIMER_VAL_SEC);
            APP_FLAG_SET(TIMER_STARTED);
        }
    }
    else
    {
        printf("Unexpected handle: 0x%04d.\n",handle);
    }
#else
    printf("Unexpected data for CLIENT!!!\n\n");
    for(int i = 0; i < data_length; i++)
    {
        printf("%c", att_data[i]);
    }
#endif
}

#if SERVER
void aci_gatt_srv_write_event(uint16_t Connection_Handle,
                                 uint8_t Resp_Needed,
                                 uint16_t Attribute_Handle,
                                 uint16_t Data_Length,
                                 uint8_t Data[])
{
    uint8_t att_error;

    if(Attribute_Handle == RXCharHandle + 1)
    {
        handle_value_update(&rx_app_context, Data_Length, Data);
        att_error = BLE_ATT_ERR_NONE;
    }
    else
    {
        printf("Received data from not recognized attribute handle 0x%4X\n",
               Attribute_Handle);
        att_error = BLE_ATT_ERR_INVALID_HANDLE;
    }

    if (Resp_Needed == 1U)
    {
        aci_gatt_srv_resp(Connection_Handle, Attribute_Handle, att_error, 0,  NULL);
    }
}
#endif

void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
    printf("%s hci_le_data_length_change_event ", DEVICE_NAME);
    printf("\nconn_handle = 0x%02x\nMaxTxOctets = %d\nMaxTxTime = %d\nMaxRxOctets = %d\nMaxRxTime = %d\n",
           Connection_Handle, MaxTxOctets, MaxTxTime, MaxRxOctets, MaxRxTime);
    ll_data_len = MaxTxOctets;
}

#if CLIENT
void aci_gatt_clt_notification_event(uint16_t Connection_Handle,
                                     uint16_t Attribute_Handle,
                                     uint16_t Attribute_Value_Length,
                                     uint8_t Attribute_Value[])
{
    handle_value_update(&rx_app_context, Attribute_Value_Length, Attribute_Value);
}

void aci_gatt_clt_indication_event(uint16_t Connection_Handle,
                                   uint16_t Attribute_Handle,
                                   uint16_t Attribute_Value_Length,
                                   uint8_t Attribute_Value[])
{
    tBleStatus ret;

    handle_value_update(&rx_app_context, Attribute_Value_Length, Attribute_Value);
    ret = aci_gatt_clt_confirm_indication(Connection_Handle);
    if(ret != BLE_STATUS_SUCCESS)
    {
        APP_FLAG_SET(INDICATION_PENDING);
    }
}

void aci_gatt_clt_disc_read_char_by_uuid_resp_event(uint16_t Connection_Handle,
                                                uint16_t Attribute_Handle,
                                                uint8_t Attribute_Value_Length,
                                                uint8_t Attribute_Value[])
{
    if(APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
    {
        tx_handle = Attribute_Handle;
        printf("TX Char Handle 0x%04X\n", tx_handle);
    }
    else if(APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
    {
        rx_handle = Attribute_Handle;
        printf("RX Char Handle 0x%04X\n", rx_handle);
    }
}

void aci_gatt_clt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
    if(APP_FLAG(START_READ_TX_CHAR_HANDLE) && !APP_FLAG(END_READ_TX_CHAR_HANDLE))
    {
        APP_FLAG_SET(END_READ_TX_CHAR_HANDLE);
    }
    else if(APP_FLAG(START_READ_RX_CHAR_HANDLE) && !APP_FLAG(END_READ_RX_CHAR_HANDLE))
    {
        APP_FLAG_SET(END_READ_RX_CHAR_HANDLE);
    }
}
#else
void aci_gatt_srv_confirmation_event(uint16_t Connection_Handle)
{
    APP_FLAG_CLEAR(INDICATION_PENDING);
}

#endif

/*******************************************************************************
 * Function Name  : aci_gatt_tx_pool_available_event.
 * Description    : This event occurs when a TX pool available is received.
 * Input          : See file bluenrg1_events.h
 * Output         : See file bluenrg1_events.h
 * Return         : See file bluenrg1_events.h
 *******************************************************************************/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{
    /* It allows to notify when at least 2 GATT TX buffers are available */
    APP_FLAG_CLEAR(TX_BUFFER_FULL);
}

void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Att_MTU)
{
    att_mtu = Att_MTU;
    APP_FLAG_SET(ATT_MTU_EXCHANGED);
    printf("ATT mtu exchanged with value = %d\n", Att_MTU);
}

#if defined(SECURE_CONNECTION)
void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
    printf("Received aci_gap_pairing_complete_event with Status = 0x%02X Reason = 0x%02X\n", Status, Reason);
    if (Status == BLE_STATUS_SUCCESS)
    {
        APP_FLAG_SET(DEV_PAIRED);
    }
    else
    {
#if CLIENT
        APP_FLAG_SET(DEV_PAIRING_REQ);
#endif
    }
}

void hci_encryption_change_event(uint8_t Status,
                                 uint16_t Connection_Handle,
                                 uint8_t Encryption_Enabled)
{
    if ((Status != 0) || (Encryption_Enabled == 0))
    {
        APP_FLAG_CLEAR(DEV_PAIRED);
        APP_FLAG_SET(DEV_PAIRING_REQ);
        force_rebond = 1;
        printf("hci_encryption_change_event with Status = 0x%02X, Encryption_Enabled = 0x%02X\n", Status, Encryption_Enabled);
    }
}
#endif

#if SET_PHY 
void hci_le_phy_update_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t TX_PHY,
                                      uint8_t RX_PHY)
{                                    
  printf("phy_update_complete_event: Status = 0x%02X, Connection= 0x%02X, TX_PHY = 0x%0X , RX_PHY = 0x%0X  \n", Status, Connection_Handle,TX_PHY,RX_PHY);                                     
}
#endif

void hci_le_read_remote_used_features_complete_event(uint8_t Status,
                                                     uint16_t Connection_Handle,
                                                     uint8_t LE_Features[8])
{
  printf("Feature exchange completed\n");
  APP_FLAG_CLEAR(FEAT_EXCHANGE_REQ);
  APP_FLAG_SET(FEAT_EXCHANGED);
}

void aci_blue_crash_info_event(uint8_t Crash_Type,
                               uint32_t SP,
                               uint32_t R0,
                               uint32_t R1,
                               uint32_t R2,
                               uint32_t R3,
                               uint32_t R12,
                               uint32_t LR,
                               uint32_t PC,
                               uint32_t xPSR,
                               uint8_t Debug_Data_Length,
                               uint8_t Debug_Data[])
{
    int i;

    printf("ERROR: aci_blue_crash_info_event:\n");
    printf("Crash_Type = 0x%02X\n", Crash_Type);
    printf("SP = 0x%02X\n", SP);
    printf("R0 = 0x%02X\n", R0);
    printf("R1 = 0x%02X\n", R1);
    printf("R2 = 0x%02X\n", R2);
    printf("R3 = 0x%02X\n", R3);
    printf("R12 = 0x%02X\n", R12);
    printf("LR = 0x%02X\n", LR);
    printf("PC = 0x%02X\n", PC);
    printf("xPSR = 0x%02X\n", xPSR);

    printf("Debug_Data:\n");
    for(i = 0; i < Debug_Data_Length; i++)
    {
        printf("0x%02X ", Debug_Data[i]);
        if ((i % 16) == 0)
        {
            printf("\n");
        }
    }

    while(1)
    {
        /** Break application execution. */
    }
}

#if SET_L2CAP_COS 
void aci_l2cap_cfc_connection_event(uint16_t Connection_Handle,
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
    tBleStatus ret;

    printf("Received aci_l2cap_cfc_connection_event:\r\n");
    printf("Connection Handle = %d\r\n", Connection_Handle);
    printf("Event type = 0x%02X\r\n", Event_Type);
    printf("Result = 0x%04X\r\n", Result);
    printf("Identifier = 0x%02X\r\n", Identifier);
    printf("SPSM = 0x%04X\r\n", SPSM);
    printf("CID = 0x%04X\r\n", CID);
    printf("Remote CID = %d\r\n", Remote_CID);
    printf("Peer MTU = %d\r\n", Peer_MTU);
    printf("Peer MPS = %d\r\n", Peer_MPS);
    printf("Initial Credit = %d\r\n", Initial_Credits);

    if(Event_Type == L2CAP_CONN_REQ)
    {
        /* reply to the incoming Connection Event (Request) by sending a Connection response
         * with our own CID irrespective of the CID coming with connection event that is inevitably NULL */
        ret = aci_l2cap_cfc_connection_resp(Connection_Handle,
                                            Identifier,
                                            L2C_COS_CID + 1,
                                            l2c_cos_local_mtu,
                                            l2c_cos_local_mps,
                                            0x0000U,
                                            L2C_COS_CFC_POLICY,
                                            L2C_COS_RX_SDU_BUFFER_SIZE_X_MTU(l2c_cos_local_mtu),
                                            l2c_cos_rx_sdu_buffer);
        if (ret != BLE_STATUS_SUCCESS)
        {
            printf("Error calling aci_l2cap_cfc_connection_resp = 0x%02X\r\n", ret);
        }
    }
    else if(Event_Type == L2CAP_CONN_RESP)
    {
        APP_FLAG_SET(L2C_COS_TX);
        l2c_peer_mtu = Peer_MTU;
    }
}

void aci_l2cap_sdu_data_rx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t RX_Credit_Balance,
                                 uint16_t SDU_Length)
{
    tBleStatus ret;
    uint8_t rx_buffer[L2C_COS_RX_SDU_BUFFER_SIZE];
    uint16_t sdu_len;
    /** printf("aci_l2cap_sdu_data_rx_event Connection_Handle=0x%04X CID=0x%04X RX_Credit_Balance=%d SDU_Length=%d\r\n", Connection_Handle, CID, RX_Credit_Balance, SDU_Length); */
    ret = aci_l2cap_extract_sdu_data(Connection_Handle,
                                     CID,
                                     L2C_COS_RX_SDU_BUFFER_SIZE,
                                     rx_buffer,
                                     &sdu_len);
    if (ret != BLE_STATUS_SUCCESS)
    {
        printf("Error calling aci_l2cap_extract_sdu_data = 0x%02X\r\n", ret);
    }
    else
    {
        handle_value_update(&l2c_cos_rx_app_context, sdu_len, rx_buffer);
    }
}

#endif

void aci_l2cap_disconnection_complete_event(uint16_t Connection_Handle,
                                            uint16_t CID)
{
    printf("aci_l2cap_disconnection_complete_event Connection_Handle=0x%04X CID=0x%02X\r\n",
                                                        Connection_Handle, CID);
    if (CID == L2C_COS_CID)
    {
        APP_FLAG_CLEAR(L2C_COS_TX);
    }
}

#if SET_L2CAP_COS 
void aci_l2cap_sdu_data_tx_event(uint16_t Connection_Handle,
                                 uint16_t CID,
                                 uint16_t SDU_Length,
                                 void * SDU_Data_Buffer,
                                 uint16_t TX_Credit_Balance)
{
    /** printf("aci_l2cap_sdu_data_tx_event Connection_Handle=0x%04X CID=0x%02X SDU_Length=%d SDU_Data_Buffer=%p TX_Credit_Balance=%d\r\n",
                                                        Connection_Handle, CID, SDU_Length, SDU_Data_Buffer, TX_Credit_Balance); */
}

void aci_l2cap_flow_control_credit_event(uint16_t Connection_Handle,
                                         uint16_t CID,
                                         uint16_t TX_Credits,
                                         uint16_t TX_Credit_Balance)
{
   /** printf("aci_l2cap_flow_control_credit_event Connection_Handle=0x%04X CID=0x%02X TX_Credits=%d TX_Credit_Balance=%d\r\n",
                                                        Connection_Handle, CID, TX_Credits, TX_Credit_Balance);*/
}
#endif 
