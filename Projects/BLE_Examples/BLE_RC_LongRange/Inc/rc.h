
#ifndef _RC_H_
#define _RC_H_

#define OUTPUT_POWER_LEVEL  24  // 0 dBm

#define BD_ADDR_MASTER      0xe0, 0x00, 0x00, 0xE1, 0x80, 0x02
#define BD_ADDR_SLAVE       0xe1, 0x00, 0x00, 0xE1, 0x80, 0x02

#define ADV_INTERVAL_MIN        (20*1000/625)   //   20 ms
#define ADV_INTERVAL_MAX        (20*1000/625)   //   20 ms

#define SCAN_INTERVAL           (100*1000/625)  //  100 ms
#define SCAN_WINDOW             (100*1000/625)  //  100 ms
#define CONN_INTERVAL_MIN       (20*100/125)    //   20 ms
#define CONN_INTERVAL_MAX       (20*100/125)    //   20 ms
#define SUPERVISION_TIMEOUT     (1000/10)       // 1000 ms
#define CE_LENGTH               (7*1000/625)
#define INITIATING_PHY          LE_1M_PHY   // Choose one between LE_1M_PHY and LE_CODED_PHY

#define ADVSCAN_CONN_LED        BSP_LED2
#define LONG_RANGE_LED          BSP_LED1
#define CONTROL_LED             BSP_LED3
#define ADVSCAN_LED_INTERVAL_MS 500

#define TEMPERATURE_UPDATE_RATE  2000
#define DISCONNECTION_TIMEOUT    0 // In milliseconds. 0 to disable.

#define AUTO_TOGGLE_LED          1
#define WRITE_INTERVAL_MS        300

uint8_t RC_DeviceInit(void);
void APP_Tick(void);

#endif /* _RC_H_ */
