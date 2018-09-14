#ifndef _RELAY_H_
#define _RELAY_H_

#include "stm32_eval.h"

#include "DtypeStm32.h"

typedef enum
{
   RELAY_CONDUCTIVE_SAMPLE_SEL1       = 0,  //
   RELAY_CONDUCTIVE_SAMPLE_SEL2,
   RELAY_CONDUCTIVE_SAMPLE_SEL3,
   RELAY_CONDUCTIVE_SAMPLE_SEL4,
   RELAY_CONDUCTIVE_SAMPLE_SEL5,

   RELAY_NUMBER,
}RELAY_ENUM;

typedef void (*RelayPulse_Cb)(void);


void InitRelays(void);
void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable);
UINT8 GetRelayLogicStatus(UINT8 ucChannel);
void RelayToggle(UINT8 ucChannel);
void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb);
void RelayLogicTurnoffAll(void);
void RelayLogicTurnonAll(void);
void RelayBlink(UINT8 ucChannel,uint32_t duration);

#endif
