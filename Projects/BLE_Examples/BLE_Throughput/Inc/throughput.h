
#ifndef _THROUGHPUT_H_
#define _THROUGHPUT_H_

#define CHAR_SIZE                       (512)
#define ATT_DEF_SIZE                    (23)
#define THROUGHPUT_TIMER_VAL_SEC        (2)
#define LL_DATA_LEN_LOW                 (27)
#define LL_DATA_LEN_MID                 (100)
#define LL_DATA_LEN_HIGH                (251)
#define LL_DEFAULT_DATA_LEN             (LL_DATA_LEN_LOW)
#define ADV_INTERVAL_MIN                (200)     /* ms */
#define ADV_INTERVAL_MAX                (300)     /* ms */
#define CLIENT_ADDRESS                  0xAA, 0x00, 0x03, 0xE1, 0x80, 0x02
#define SERVER_ADDRESS                  0xAD, 0x00, 0x03, 0xE1, 0x80, 0x02

//L2CAP COS constants
#define L2C_COS_SPSM                    (0x80) //Fixed [0x80...0xFF]

#define L2C_MAX_MTU                     (1024)
#define L2C_TX_BUFFER_SIZE              (L2C_MAX_MTU + 2)
#define L2C_COS_NUM_TX_BUFFER           (3)

#ifdef CLIENT
#define L2C_COS_CID                     (0x40)
#define L2C_COS_MTU                     (1024)
#define L2C_COS_MPS                     (550)
#define L2C_COS_CFC_POLICY              (1)
#define L2C_COS_RX_SDU_BUFFER_SIZE      ((L2C_COS_MTU + 2) * 3)//(247 + 4)
#else
#define L2C_COS_CID                     (0x50)
#define L2C_COS_MTU                     (1024)
#define L2C_COS_MPS                     (1024)
#define L2C_COS_CFC_POLICY              (1)
#define L2C_COS_RX_SDU_BUFFER_SIZE      ((L2C_COS_MTU + 2) * 3)//(247 + 4)
#endif

/*
 * UUIDs:
 * D973F2E0-B19E-11E2-9E96-0800200C9A66
 * D973F2E1-B19E-11E2-9E96-0800200C9A66
 * D973F2E2-B19E-11E2-9E96-0800200C9A66
 */
#define SRVC_UUID                       0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9
#define TX_CHR_UUID                     0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9
#define RX_CHR_UUID                     0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9

uint8_t DeviceInit(void);
void APP_Tick(void);
void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);
void print_help(void);
#endif // _THROUGHPUT_H_
