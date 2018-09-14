#include "relay.h"

#include "Config.h"

#include <string.h>

#include "sys_time.h"

typedef struct 
{
    uint8_t ucChl;
    sys_timeo to;

    RelayPulse_Cb cb;
    
}RELAY_PULSE_STRU;

const uint8_t relays[RELAY_NUMBER] = {
    STM32F103_GPB(4),     
    STM32F103_GPB(5),     
    STM32F103_GPB(12),     
    STM32F103_GPB(13),     
    STM32F103_GPB(14),     
};

RELAY_PULSE_STRU aPulseCtrl[RELAY_NUMBER];

const uint8_t relays_on[RELAY_NUMBER] = {
    1,1,1,1,1
};


void InitRelays(void)
{
    int iLoop;
    for (iLoop = 0; iLoop < RELAY_NUMBER; iLoop++)
    {
        stm32_gpio_cfgpin(relays[iLoop],MAKE_PIN_CFG(GPIO_Speed_2MHz,GPIO_Mode_Out_PP)); 

        memset(&aPulseCtrl[iLoop],0,sizeof(RELAY_PULSE_STRU));

        RelayLogicCtrl(iLoop,FALSE);

        aPulseCtrl[iLoop].ucChl = iLoop;
    }

}

void RelayLogicCtrl(UINT8 ucChannel,UINT8 ucEnable)
{
    if (ucEnable)
    {
        stm32_gpio_set_value(relays[ucChannel],relays_on[ucChannel]); // ylf:  inactive low
    }
    else
    {
        stm32_gpio_set_value(relays[ucChannel],!relays_on[ucChannel]); // ylf:  inactive low
    }
}

UINT8 GetRelayLogicStatus(UINT8 ucChannel)
{

    return (!!stm32_gpio_get_value(relays[ucChannel])) == relays_on[ucChannel];

}


void RelayToggle(UINT8 ucChannel)
{
    stm32_gpio_toggle_value(relays[ucChannel]); // ylf:  inactive low
    
}

void RelayPulse_cb(void *para)
{
    RELAY_PULSE_STRU *pulse = (RELAY_PULSE_STRU *)para;
    
    RelayLogicCtrl(pulse->ucChl,FALSE);
    
    if (pulse->cb)  (pulse->cb)();
}


void RelayPulse(UINT8 ucChannel,uint32_t duration,RelayPulse_Cb cb)
{
    aPulseCtrl[ucChannel].cb = cb;

    RelayLogicCtrl(ucChannel,TRUE);

    sys_timeout(duration,SYS_TIMER_ONE_SHOT,duration,RelayPulse_cb,&aPulseCtrl[ucChannel],&aPulseCtrl[ucChannel].to);
}

void RelayBlink_cb(void *para)
{
    RELAY_PULSE_STRU *pulse = (RELAY_PULSE_STRU *)para;
    
    RelayToggle(pulse->ucChl);
    
    if (pulse->cb)  (pulse->cb)();
}

void RelayBlink(UINT8 ucChannel,uint32_t duration)
{
    aPulseCtrl[ucChannel].cb = NULL;

    if (0 == duration)
    {
        RelayLogicCtrl(ucChannel,FALSE);

        sys_untimeout(&aPulseCtrl[ucChannel].to);
    }
    else
    {
        RelayLogicCtrl(ucChannel,TRUE);

        sys_timeout(duration,SYS_TIMER_PERIOD,duration,RelayBlink_cb,&aPulseCtrl[ucChannel],&aPulseCtrl[ucChannel].to);
    }
}


/***********************************
    Put all device to its default state.
************************************/
void RelayLogicTurnoffAll(void)
{
    UINT8 ucLoop;

    for (ucLoop = 0; ucLoop < RELAY_NUMBER ; ucLoop++)
    {
        RelayLogicCtrl(ucLoop,FALSE);
    }
        
}


/***********************************
    Put all device to its default state.
************************************/
void RelayLogicTurnonAll(void)
{
    UINT8 ucLoop;

    for (ucLoop = 0; ucLoop <  RELAY_NUMBER ; ucLoop++)
    {
        RelayLogicCtrl(ucLoop,TRUE);
    }
        
}



