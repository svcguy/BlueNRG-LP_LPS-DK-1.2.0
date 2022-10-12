
#ifndef _PROFILE_H_
#define _PROFILE_H_

#ifndef MAX_NUM_SLAVES
#define MAX_NUM_SLAVES 1
#endif


#define TEST_PULSE_GPIO_PORT        GPIOB
#define TEST_PULSE_GPIO_PIN         LL_GPIO_PIN_4

// Service data
#define PROFILE_DATA_NODE        0
#define PROFILE_DATA_COLLECTOR   1


// Roles
#define ROLE_PERIPHERAL     1
#define ROLE_CENTRAL        2

#if PROFILE_ROLE == ROLE_CENTRAL

#define SERVICE_DATA_TO_SEARCH_FOR  PROFILE_DATA_NODE
#define SERVICE_DATA_TO_ADVERTISE   PROFILE_DATA_COLLECTOR
#define LOCAL_NAME "SyncCentral"

#elif PROFILE_ROLE == ROLE_PERIPHERAL

/* No service data to search for */
#define SERVICE_DATA_TO_ADVERTISE    PROFILE_DATA_NODE
#define LOCAL_NAME "SyncPeriph"

#endif

uint8_t DeviceInit(void);
void APP_Tick(void);

#endif // _PROFILE_H_
