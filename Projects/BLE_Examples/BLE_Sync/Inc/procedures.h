#ifndef _PROCEDURES_H_
#define _PROCEDURES_H_

extern const char name[];

tBleStatus ConfigureAdvertising(void);
tBleStatus StartAdvertising(void);
tBleStatus StartGeneralConnectionEstablishment(void);

#endif /* _PROCEDURES_H_ */
