

#ifndef _SENSOR_H_
#define _SENSOR_H_

#if defined (CONFIG_DEVICE_BLUENRG_LP)
#define ENABLE_BLUEVOICE (1)
#elif defined (CONFIG_DEVICE_BLUENRG_LPS)
#define ENABLE_BLUEVOICE (0)
#else
#error "Please define ENABLE_BLUEVOICE according to selected device" 
#endif


#if ENABLE_BLUEVOICE

/**** Audio sampling frequency ****/
#define FS 8000 

/* DMA CHANNEL FOR ADC PDM PORT */
#define LL_DMA_CHANNEL_ADC      LL_DMA_CHANNEL_3
      
/* BLUENRG-LP with EXTENDED DATA PACKETS */ 
/* Define EXT_DATA_PCK in the preprocessor */
/* if you choose 8 kHz, set supported max value for MAX_ATT_MTU = 43 in *_config.h */
/* if you choose 16 kHz, set supported max value for MAX_ATT_MTU = 83 in *_config.h */


#define MS_IN 10   /* number of millisecond acquired */

#if (FS == 16000)
  #define PCM_BUFFER_SIZE 16 * MS_IN * 2                        /* 1ms, 16 kHz */
#elif (FS == 8000)
  #define PCM_BUFFER_SIZE 8 * MS_IN * 2                         /* 1ms, 8 kH */
#endif

#if (FS == 16000)
  #define AUDIO_SAMPLING_FREQUENCY                (uint16_t) (16000)            /* 16 kHZ */
#elif (FS == 8000)
  #define AUDIO_SAMPLING_FREQUENCY                (uint16_t) (8000)             /* 8 kHZ */
#endif

/* Exported variables --------------------------------------------------------*/

typedef struct {
	int32_t Z;
	int32_t oldOut;
	int32_t oldIn;
} HP_FilterState_TypeDef;

typedef enum {
	BV_APP_SUCCESS = 0x00, /*!< BV_APP Success.*/
	BV_APP_ERROR = 0x10 /*!< BV_APP Error.*/
} BV_APP_Status;


extern volatile uint8_t AudioNotification;
extern volatile uint8_t SyncNotification;

BV_APP_Status BVL_APP_PER_Init_BLE(void);
BV_APP_Status BV_APP_PER_ProfileInit(void);
void TC_IT_Callback(void);
void HT_IT_Callback(void);
void BVL_APP_PER_AudioProcess(uint16_t* PCM_Buffer);
void DMA_Rearm(uint32_t dma_channel, uint32_t buffer, uint32_t size);
#endif /* ENABLE_BLUEVOICE */

extern uint8_t Application_Max_Attribute_Records[]; 

uint8_t Sensor_DeviceInit(void);
void APP_Tick(void);

#endif /* _SENSOR_H_ */
