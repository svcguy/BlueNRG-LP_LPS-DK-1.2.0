
#ifndef _MAIN_H_
#define _MAIN_H_

#define BD_ADDR_MASTER      0xe0, 0x00, 0x00, 0xE1, 0x80, 0x02
#define BD_ADDR_SLAVE       0xe1, 0x00, 0x00, 0xE1, 0x80, 0x02

#define ADV_INTERVAL_MIN        (20*1000/625)   //   20 ms
#define ADV_INTERVAL_MAX        (20*1000/625)   //   20 ms

#define SCAN_INTERVAL           (100*1000/625)  //  100 ms
#define SCAN_WINDOW             (100*1000/625)  //  100 ms
#define CONN_INTERVAL_MIN       (40*100/125)    //   40 ms
#define CONN_INTERVAL_MAX       (40*100/125)    //   40 ms
#define SUPERVISION_TIMEOUT     (1000/10)       // 1000 ms
#define CE_LENGTH               (2*1000/625)    //    2 ms

/* Values for power control */
#define RSSI_TARGET_1M              -70  // dBm
#define RSSI_TARGET_2M              -67  // dBm
#define RSSI_TARGET_CODED_S8        -77  // dBm
#define RSSI_TARGET_CODED_S2        -73  // dBm
#define RSSI_HYSTERESIS               8  // dB

/* Values for path loss monitoring */
#define HIGH_THRESHOLD      74  // dB
#define HIGH_HYSTERESIS     6   // dB
#define LOW_THRESHOLD       55  // dB
#define LOW_HYSTERESIS      6   // dB
#define MIN_TIME            4   // Connection events

#define ADVSCAN_CONN_LED        BSP_LED2
#define PATH_LOSS_LED           BSP_LED1
#define ADVSCAN_LED_INTERVAL_MS 500
#define PATHLOSS_LOW_LED_INTERVAL_MS   100
#define PATHLOSS_MID_LED_INTERVAL_MS   250
#define PATHLOSS_HIGH_LED_INTERVAL_MS  500

uint8_t ProfileInit(void);
void APP_Tick(void);

#endif /* _MAIN_H_ */
