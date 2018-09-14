#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "cminterface.h"

typedef void (*adc_complete_cb)(void *act);

typedef struct
{
   uint8_t ucGain;
   uint8_t ucRate;
   uint8_t ucMode;
}AD7799_CONFIG_STRU;

typedef struct
{
    int     iDummy;

    int     iSubChlIdx[2];

    volatile int  iPwmSyncEvent;

    int     iData[2][3];

    AD7799_CONFIG_STRU ad7799Cfg[2][3];

    uint8_t aucChlReady[SENSOR_CHANNEL_NUMBER];      

    int     aiTempData[SENSOR_CHANNEL_NUMBER];
    
}DISPLAY_STRU;

extern DISPLAY_STRU Display;

void Display_SecondTask(void);

void Display_Init(void);

void Display_StartAdc(int iAdc,adc_complete_cb cb);

void Display_SwitchMultiplex(void);

void Display_msg_Handler(Message *Msg);

uint32_t Display_ReadAdc(int iAdc);
uint32_t GetAdcData(uint8_t ucChl);

#endif
