

#ifndef _GATT_DB_H_
#define _GATT_DB_H_

#define CHARACTERISTIC_LEN  20

tBleStatus Add_Chat_Service(void);
uint8_t Add_Sync_Service(void);

extern uint16_t vclockCharHandle;

extern const uint8_t sync_service_uuid[16];
extern const uint8_t vclock_char_uuid[16];

#endif /* _GATT_DB_H_ */
