#include <string.h>
#include <stdlib.h>

#include "stm32_eval.h"

#include <string.h>

#include <stdio.h>

#include    <ucos_ii.h>

#include    <cpu.h>

#include    <app_cfg.h>
#include    <bsp.h>

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "Config.h"

#include "Display.h"

#include "keyboard.h"

#include "Relay.h"

#include "Beep.h"

#include "Cancmd.h"

#include "Uartcmd.h"

#include "IWDG_Driver.h"

#include "Rtc_driver.h"

#include "ad7799_multi_Driver.h"

#include "hal_spi_driver.h"

#include "dica.h"

#include "Timer_Driver.h"

#include "list.h"

#define ADC_MODE AD7799_MOD1
#define ADC_RATE AD7799_UR14
#define ADC_SPAN 40

typedef struct
{
    MsgHead msgHead;
    void *para;
    void *para2;
}DISP_MSG;

#define DISP_MSG_LENGHT (sizeof(DISP_MSG)-sizeof(MsgHead))


#define MAX_ACTIONS 16

enum ACTION_TYPE_ENUM
{
    ACTION_TYPE_ADC_SAMPLE = 0,
    ACTION_TYPE_DELAY,
};

typedef struct
{

    uint8_t ucMainChl;
    uint8_t ucSubChl;
    uint8_t ucDelayMs;
}ACTION_ADC_PARA_STRU;

typedef struct
{
    uint8_t ucMs;
}ACTION_DELAY_PARA_STRU;

typedef void (*display_msg_cb)(void *);



typedef struct
{
  list_t list;
  int iActType;  /* refer ACTION_TYPE_ENUM */
   union 
   {
      ACTION_ADC_PARA_STRU   adc;
      ACTION_DELAY_PARA_STRU delay;
   }u;

   adc_complete_cb cb;
   
}AD7799_ACTION_STRU;

typedef struct
{
    AD7799_ACTION_STRU actions[MAX_ACTIONS];

    list_t head[2];

    uint8_t ucActFront;
    uint8_t ucActRear;
    
    uint8_t iCurAction; 

    sys_timeo to4ActionDelay;
    
}AD7799_SCHEDULER_STRU;

#define AD7799_QUEUE_FULL(front,rear) ((((front) + 1) % MAX_ACTIONS )== (rear))
#define AD7799_QUEUE_EMPTY(front,rear) ((front) == (rear))

static AD7799_SCHEDULER_STRU sAd7799Scheduler;

DISPLAY_STRU Display;

static const uint8_t spi_cs[2] = {
    STM32F103_GPA(4),     
    STM32F103_GPB(3),     
};

void Display_Ad7799_Schedule(void);
void Display_Ad7799_Schedule_inner(uint8_t ucAction);

void hal_SPI1_CS_LOW(uint8_t ucChl)
{
    int iLoop;
    for (iLoop = 0; iLoop < 2; iLoop++)
    {
        if (ucChl == iLoop)
        {
            continue;
        }
         stm32_gpio_set_value(spi_cs[iLoop],1); // ylf:  inactive
    }
    
    stm32_gpio_set_value(spi_cs[ucChl],0); // ylf:  active low
}

/**
  * @brief  Deselect sFLASH: Chip Select pin high
  */
void hal_SPI1_CS_HIGH(uint8_t ucChl)
{
    stm32_gpio_set_value(spi_cs[ucChl],1); // ylf:  inactive low
}   

void Display_msg_Handler(Message *Msg)
{
    DISP_MSG *dmsg = (DISP_MSG *)Msg;

    if (dmsg->para)
    {
        ((display_msg_cb)dmsg->para)(dmsg->para2);
    }
}


// function for serialization
void Display_report(void *para,void *para2)
{
   Message *Msg;
   Msg = MessageAlloc(PID_SELF,DISP_MSG_LENGHT);

   if (Msg)
   {
       DISP_MSG *dmsg = (DISP_MSG *)Msg;
       dmsg->msgHead.nMsgType = SELF_MSG_CODE_USER_AD7799;
       dmsg->para = para;
       dmsg->para2 = para2;
       MessageSend(Msg);
   }
}


void Display_SecondTask(void)
{

    //VOS_LOG(VOS_LOG_DEBUG,"0:%d&%d&%d\r\n",Display.iData[0][0],Display.iData[0][1],Display.iData[0][2]);
    //VOS_LOG(VOS_LOG_DEBUG,"1:%d&%d&%d\r\n",Display.iData[1][0],Display.iData[1][1],Display.iData[1][2]);
    //VOS_LOG(VOS_LOG_DEBUG,"2:%d&%d&%d&%d&%d\r\n",Display.aiTempData[0],Display.aiTempData[1],Display.aiTempData[2],Display.aiTempData[3],Display.aiTempData[4]);

    //Display_StartAdc(0,NULL);
    //Display_StartAdc(1,NULL);
    //Display_StartAdc(2,NULL);
    //Display_StartAdc(3,NULL);
    //Display_StartAdc(4,NULL);

}

uint8_t Display_AD7799_Init(uint8_t ucChl)  
{  
    uint8_t ID;
    //uint8_t Cmd[2];  

    mAD7799_Reset(ucChl);  
    OSTimeDlyHMSM(0,0,0,20);  
    mAD7799_ReadReg(ucChl,AD7799_MAKE_REG(AD7799_RADDR_ID),&ID,1);               //读取器件ID  
    if((ID & 0XF) != 9)
    {
        VOS_LOG(VOS_LOG_ERROR,"as chl:%x fail\r\n",ucChl);
        
        return 1;  
    }
    
    mAD7799_Calibrate(ucChl,AD7799_CH0,Display.ad7799Cfg[ucChl][0].ucGain);             //通道1校准  for conductive rate
    mAD7799_Calibrate(ucChl,AD7799_CH1,Display.ad7799Cfg[ucChl][1].ucGain);             //通道2校准  for temperature
    mAD7799_Calibrate(ucChl,AD7799_CH2,Display.ad7799Cfg[ucChl][2].ucGain);             //通道2校准  for temperature
    
    VOS_LOG(VOS_LOG_DEBUG,"Initialize ad7799 chl:%x succeed\r\n",ucChl);
    
    return 0;  
}  

void Display_AD7799_Start(uint8_t ucChl,uint8_t CovChx,uint8_t CovGain,uint8_t CovRate,uint8_t CovMode)  
{  
    uint8_t Cmd[2];  
    
    Cmd[0] = CON_REG_UNIPOL|CovGain;  
    Cmd[1] = CON_REG_BUFFER|CovChx;  
    mAD7799_WriteRegEx(ucChl,0,AD7799_MAKE_REG(AD7799_WADDR_CONFIG),Cmd,2); 
    
    Cmd[0] = CovMode;  
    Cmd[1] = CovRate;  
    mAD7799_WriteRegEx(ucChl,1,AD7799_MAKE_REG(AD7799_WADDR_MODE),Cmd,2);  
}  

uint32_t Display_AD7799_Read(uint8_t ucChl)  
{  
    uint8_t  Cmd[4];  
    uint32_t D;  
    Cmd[0]=0;  
    mAD7799_ReadRegEx(ucChl,1,AD7799_MAKE_REG(AD7799_RADDR_DATA),&Cmd[1],3);  
    D = (Cmd[1]<<16)|(Cmd[2]<<8)|Cmd[3];  
    return D;  
} 


void Display_SwitchChannel(int iMain,int iSub)
{
   if (iSub != Display.iSubChlIdx[iMain])
   {
       Display.iSubChlIdx[iMain]  = iSub;

       if (ADC_MODE == AD7799_MOD0)
       {
          Display_AD7799_Start(iMain,iSub,Display.ad7799Cfg[iMain][iSub].ucGain,Display.ad7799Cfg[iMain][iSub].ucRate,(Display.ad7799Cfg[iMain][iSub].ucMode <<5)); 
       }
       else
       {
          Display_AD7799_Start(iMain,iSub,Display.ad7799Cfg[iMain][iSub].ucGain,Display.ad7799Cfg[iMain][iSub].ucRate,(AD7799_MOD2<<5)); 
       }
   }
}

void Display_StartAdc(int iAdc,adc_complete_cb cb)
{
   int iMainChl = iAdc / 3;
   int iSubChl  = iAdc % 3;
   CPU_SR cpu_sr;

   if (AD7799_QUEUE_FULL(sAd7799Scheduler.ucActFront,sAd7799Scheduler.ucActRear))
   {
       VOS_LOG(VOS_LOG_DEBUG,"Queue full \r\n");
   
       return ;
   }

   CPU_CRITICAL_ENTER();

   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].cb              = cb;
   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].iActType        = ACTION_TYPE_ADC_SAMPLE;
   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].u.adc.ucMainChl = iMainChl;
   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].u.adc.ucSubChl  = iSubChl;
   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].u.adc.ucDelayMs = 0;

   if (list_empty(&sAd7799Scheduler.head[iMainChl]))
   {
       if (Display.iSubChlIdx[iMainChl] != iSubChl)
       {
           sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].u.adc.ucDelayMs = ADC_SPAN;
       }
   }
   else
   {
       AD7799_ACTION_STRU *pNode;
       
       pNode = list_entry(sAd7799Scheduler.head[iMainChl].prev,AD7799_ACTION_STRU,list);

       if (pNode->u.adc.ucSubChl != iSubChl)
       {
           sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].u.adc.ucDelayMs = ADC_SPAN;
       }
   }

   list_add_tail(&sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].list,&sAd7799Scheduler.head[iMainChl]);

   sAd7799Scheduler.ucActFront = (sAd7799Scheduler.ucActFront + 1) % MAX_ACTIONS;

   CPU_CRITICAL_EXIT();

   Display_Ad7799_Schedule();
}

void Display_StartDelay(uint8_t delayTime,adc_complete_cb cb)
{
   CPU_SR cpu_sr;

   if (AD7799_QUEUE_FULL(sAd7799Scheduler.ucActFront,sAd7799Scheduler.ucActRear))
   {
       VOS_LOG(VOS_LOG_DEBUG,"Queue full \r\n");
   
       return ;
   }

   CPU_CRITICAL_ENTER();

   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].cb           = cb;
   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].iActType     = ACTION_TYPE_DELAY;
   sAd7799Scheduler.actions[sAd7799Scheduler.ucActFront].u.delay.ucMs = delayTime;
   sAd7799Scheduler.ucActFront = (sAd7799Scheduler.ucActFront + 1) % MAX_ACTIONS;

   CPU_CRITICAL_EXIT();
   
   Display_Ad7799_Schedule();
}

void Display_SyncEvent_msg_cb(void *para)
{
    AD7799_ACTION_STRU *pAct;

   /* get adc data */
    if (MAX_ACTIONS == sAd7799Scheduler.iCurAction)
    {
        return ;
    }
    pAct = &sAd7799Scheduler.actions[sAd7799Scheduler.iCurAction];
    
    switch(pAct->iActType)
    {
    case ACTION_TYPE_ADC_SAMPLE:
        {
            int  iMain;
            int  iSub;
            uint32_t D;
            CPU_SR cpu_sr;

            INT32U     ticks = OSTimeGet();
            INT32U     systick = SysTick->VAL;

           /* delay some time */
            SysTick_DelayUs(300);
            //SysTick_DelayUs(500);

            iMain = pAct->u.adc.ucMainChl;
            iSub  = pAct->u.adc.ucSubChl;

            if (ADC_MODE != AD7799_MOD0)
            {
                Display_AD7799_Start(iMain,iSub,Display.ad7799Cfg[iMain][iSub].ucGain,Display.ad7799Cfg[iMain][iSub].ucRate,(Display.ad7799Cfg[iMain][iSub].ucMode <<5)); 
            }

            if (!mAD7799_WaitBusy(iMain))
            {
                D = Display_AD7799_Read(iMain);
            }

            Display.iData[iMain][iSub] = D;

            //printf("chl = %d,%d,%d\r\n",iMain,OSTimeGet() - ticks,systick - SysTick->VAL);

            CPU_CRITICAL_ENTER();
            list_del_init(&pAct->list);
            sAd7799Scheduler.ucActRear  = (sAd7799Scheduler.ucActRear + 1) % MAX_ACTIONS;
            sAd7799Scheduler.iCurAction = MAX_ACTIONS;
            CPU_CRITICAL_EXIT();    

            Display_Ad7799_Schedule();
            return ;            
        }
    default:
        break;
    }       
}

void Display_PwmSyncCallback(void)
{
    //Display_report(Display_SyncEvent_msg_cb,NULL);
}

void Display_CommActionDelay_msg_cb(void *para)
{
    int iAction = (int)para;

    AD7799_ACTION_STRU *pAct;

    if (MAX_ACTIONS == iAction)
    {
        return ;
    }
    pAct = &sAd7799Scheduler.actions[iAction];
    
    switch(pAct->iActType)
    {
    case ACTION_TYPE_DELAY:
        {
            CPU_SR cpu_sr;
            CPU_CRITICAL_ENTER();
            sAd7799Scheduler.ucActRear  = (sAd7799Scheduler.ucActRear + 1) % MAX_ACTIONS;
            sAd7799Scheduler.iCurAction = MAX_ACTIONS;
            CPU_CRITICAL_EXIT();    

            Display_Ad7799_Schedule();
            return ;
        }
    default:
        Display_Ad7799_Schedule_inner(iAction);
        break;
    }    
}


void Display_CommActionDelayTimeoutHandler(void *para)
{
    Display_report(Display_CommActionDelay_msg_cb,para);
}

void Display_Ad7799_Schedule_inner(uint8_t ucAction)
{
    AD7799_ACTION_STRU *pAct;

    pAct = &sAd7799Scheduler.actions[ucAction];

    switch(pAct->iActType)
    {
    case ACTION_TYPE_ADC_SAMPLE:
        if (pAct->u.adc.ucDelayMs != 0)
        {
           /* switch channel & wait */
           Display_SwitchChannel(pAct->u.adc.ucMainChl,pAct->u.adc.ucSubChl);

           pAct->u.adc.ucDelayMs = 0;

           sys_timeout(pAct->u.adc.ucDelayMs,SYS_TIMER_ONE_SHOT,pAct->u.adc.ucDelayMs,Display_CommActionDelayTimeoutHandler,(void *)ucAction,&sAd7799Scheduler.to4ActionDelay); 
        }
        else
        {
            Display_SwitchChannel(pAct->u.adc.ucMainChl,pAct->u.adc.ucSubChl);
        
           Display.iPwmSyncEvent = 1;

           while(Display.iPwmSyncEvent);
           
           Display_SyncEvent_msg_cb(NULL);           

        }
        break;
    case ACTION_TYPE_DELAY:
        {
           sys_timeout(pAct->u.delay.ucMs,SYS_TIMER_ONE_SHOT,pAct->u.delay.ucMs,Display_CommActionDelayTimeoutHandler,(void *)ucAction,&sAd7799Scheduler.to4ActionDelay); 
        }
        break;
    }

}


void Display_Ad7799_Schedule(void)
{
    if (AD7799_QUEUE_EMPTY(sAd7799Scheduler.ucActFront,sAd7799Scheduler.ucActRear))
    {
        sAd7799Scheduler.iCurAction = MAX_ACTIONS;
        return ;
    }

    if (MAX_ACTIONS != sAd7799Scheduler.iCurAction)
    {
        /* Busy */
        return ;
    }    

    sAd7799Scheduler.iCurAction = sAd7799Scheduler.ucActRear;

    Display_Ad7799_Schedule_inner(sAd7799Scheduler.ucActRear);
}


uint32_t Display_ReadAdc(int iAdc)
{
   int iMainChl = iAdc / 3;
   int iSubChl  = iAdc % 3;

   uint32_t D;

   Display_SwitchChannel(iMainChl,iSubChl);
   
   SysTick_DelayMs(40);

   Display.iPwmSyncEvent = 1;
   
   /*waiting here for synchronizing event */
   while(Display.iPwmSyncEvent);
   
   SysTick_DelayUs(300);
   
   if (ADC_MODE != AD7799_MOD0)
   {
       Display_AD7799_Start(iMainChl,iSubChl,Display.ad7799Cfg[iMainChl][iSubChl].ucGain,Display.ad7799Cfg[iMainChl][iSubChl].ucRate,(Display.ad7799Cfg[iMainChl][iSubChl].ucMode <<5)); 
   }
   
   if (!mAD7799_WaitBusy(iMainChl))
   {
       D = Display_AD7799_Read(iMainChl);

       return D;
   }
   return 0XFFFFFFFFUL;
}


int TIM3_Handler(int Tim,int event,void *para)
{
    static int sMultiplex = 0;
    
    GPIO_TOGGLE(GPIOB,(GPIO_Pin_0 |GPIO_Pin_1));

    if (0 == (sMultiplex % 10))
    {
        GPIO_TOGGLE(GPIOB,(GPIO_Pin_6 |GPIO_Pin_7));
        
        GPIO_TOGGLE(GPIOA,(GPIO_Pin_8));
    }

    /* Use lower frequency signal to trigger synchronizing event */
    if (GPIO_STATE_HIGH(GPIOB,GPIO_Pin_6))
    {
       if (Display.iPwmSyncEvent) 
       {
           Display.iPwmSyncEvent = 0;

           /* fire an sync event */
           Display_PwmSyncCallback();
       }
    }

    sMultiplex++;
    return 0;
}


void TIM_Init_3(int iFreq)
{
    TIM_Init_General(TIMER2,iFreq);

    TIM_InstallHandler(TIMER2,TIM_IT_Update,TIM3_Handler,NULL);

    {
        GPIO_InitTypeDef GPIO_InitStructure;
        
       /* Enable the GPIO_LED Clock */
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
       
       /* Configure the GPIO_LED pin */
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_Init(GPIOB, &GPIO_InitStructure);

       /* Configure the GPIO_LED pin */
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_Init(GPIOB, &GPIO_InitStructure);
       
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_Init(GPIOA, &GPIO_InitStructure);
       

       GPIO_HIGH(GPIOB,GPIO_Pin_0 |GPIO_Pin_1|GPIO_Pin_6 |GPIO_Pin_7);
       GPIO_HIGH(GPIOA,GPIO_Pin_8);

     }    
}


void Display_Init(void)
{
    int iLoop;
    
    memset(&Display,0,sizeof(Display));

    for (iLoop = 0; iLoop < 2; iLoop++)
    {
        stm32_gpio_cfgpin(spi_cs[iLoop],MAKE_PIN_CFG(GPIO_Speed_50MHz,GPIO_Mode_Out_PP)); 
        
        stm32_gpio_set_value(spi_cs[iLoop],1); // ylf:  active low
    }

    Display.ad7799Cfg[0][0].ucGain = AD7799_GAIN2;
    Display.ad7799Cfg[0][0].ucRate = ADC_RATE;
    Display.ad7799Cfg[0][0].ucMode = ADC_MODE;

    Display.ad7799Cfg[0][1].ucGain = AD7799_GAIN0;
    Display.ad7799Cfg[0][1].ucRate = ADC_RATE;
    Display.ad7799Cfg[0][1].ucMode = ADC_MODE;
    
    Display.ad7799Cfg[0][2].ucGain = AD7799_GAIN0;
    Display.ad7799Cfg[0][2].ucRate = ADC_RATE;
    Display.ad7799Cfg[0][2].ucMode = ADC_MODE;
    
    Display.ad7799Cfg[1][0].ucGain = AD7799_GAIN0;
    Display.ad7799Cfg[1][0].ucRate = ADC_RATE;
    Display.ad7799Cfg[1][0].ucMode = ADC_MODE;
    
    Display.ad7799Cfg[1][1].ucGain = AD7799_GAIN0;
    Display.ad7799Cfg[1][1].ucRate = ADC_RATE;
    Display.ad7799Cfg[1][1].ucMode = ADC_MODE;

    Display.ad7799Cfg[1][2].ucGain = AD7799_GAIN0;
    Display.ad7799Cfg[1][2].ucRate = ADC_RATE;
    Display.ad7799Cfg[1][2].ucMode = ADC_MODE;

    memset(&sAd7799Scheduler,0,sizeof(AD7799_SCHEDULER_STRU));

    INIT_LIST_HEAD(&sAd7799Scheduler.head[0]);
    INIT_LIST_HEAD(&sAd7799Scheduler.head[1]);

    sAd7799Scheduler.iCurAction = MAX_ACTIONS;

    mAD7799_Init(HAL_SPI0);
    
    Display_AD7799_Init(0);

    Display_AD7799_Init(1);

    Display_AD7799_Start(0,0,AD7799_GAIN0,AD7799_UR12,(AD7799_MOD2<<5)); 
    
    Display_AD7799_Start(1,2,AD7799_GAIN0,AD7799_UR12,(AD7799_MOD2<<5)); 
    
    TIM_Init_3(1400);
    
}

#if 0
void Display_Init_PWM1(int freq,int duty)
{
#define TIM_PWM_FREQ (3000000)
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  

    GPIO_InitTypeDef GPIO_InitStructure;

    TIM_OCInitTypeDef  TIM_OCInitStructure; 

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    GPIO_StructInit(&GPIO_InitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure); 
    
    if ((TIM_PWM_FREQ + freq - 1)/(freq) < 65536)
    {
        TIM_TimeBaseStructure.TIM_Period    = (TIM_PWM_FREQ + freq - 1)/(freq); // period in us
        TIM_TimeBaseStructure.TIM_Prescaler = (24 - 1); // in us
    }
    else
    {
        uint32_t ulTemp = 72000000;
        uint32_t ulRate = 1;
    
        while( ulTemp/ (freq * ulRate) > 65535)
        {
            ulRate += 10;
        }
    
        ulTemp = 72000000 / ulRate;
        
        TIM_TimeBaseStructure.TIM_Period    = (ulTemp + freq - 1)/(freq); // period in us
        TIM_TimeBaseStructure.TIM_Prescaler = (ulRate - 1); // in us
    }
    
    /* Time base configuration */  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  

    
    /* Output Compare Timing Mode configuration: Channel1 */  
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // init Enable  
    TIM_OCInitStructure.TIM_Pulse       = (TIM_TimeBaseStructure.TIM_Period * duty)/100;  // ylf: init ccr value
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;//TIM_OCPolarity_High;  

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    
    /*Alternative function clock setup  for GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

   TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);  

   /* for channel5*/
   TIM_OC1Init(TIM1, &TIM_OCInitStructure);  
   TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  
   
   TIM_OC2Init(TIM1, &TIM_OCInitStructure);  
   TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);  
   
   TIM_OC3Init(TIM1, &TIM_OCInitStructure);  
   TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable); 
   
   TIM_ARRPreloadConfig(TIM1, ENABLE); 
   
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


void Display_Init_PWM3(int freq,int duty)
{
#define TIM_PWM_FREQ (3000000)
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  

    GPIO_InitTypeDef GPIO_InitStructure;

    TIM_OCInitTypeDef  TIM_OCInitStructure; 

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    GPIO_StructInit(&GPIO_InitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure); 
    
    if ((TIM_PWM_FREQ + freq - 1)/(freq) < 65536)
    {
        TIM_TimeBaseStructure.TIM_Period    = (TIM_PWM_FREQ + freq - 1)/(freq); // period in us
        TIM_TimeBaseStructure.TIM_Prescaler = (24 - 1); // in us
    }
    else
    {
        uint32_t ulTemp = 72000000;
        uint32_t ulRate = 1;
    
        while( ulTemp/ (freq * ulRate) > 65535)
        {
            ulRate += 10;
        }
    
        ulTemp = 72000000 / ulRate;
        
        TIM_TimeBaseStructure.TIM_Period    = (ulTemp + freq - 1)/(freq); // period in us
        TIM_TimeBaseStructure.TIM_Prescaler = (ulRate - 1); // in us
    }
    
    /* Time base configuration */  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  

    
    /* Output Compare Timing Mode configuration: Channel1 */  
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // init Enable  
    TIM_OCInitStructure.TIM_Pulse       = (TIM_TimeBaseStructure.TIM_Period * duty)/100;  // ylf: init ccr value
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;//TIM_OCPolarity_High;  

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);  

        
    /*Alternative function clock setup  for GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* for channel1 & 2*/
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);  
    
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);  
    
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable); 

    TIM_OC4Init(TIM3, &TIM_OCInitStructure);  
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 
    
    TIM_ARRPreloadConfig(TIM3, ENABLE); 
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}


void Display_Init_PWM4(int freq,int duty)
{
#define TIM_PWM_FREQ (3000000)
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;  

    GPIO_InitTypeDef GPIO_InitStructure;

    TIM_OCInitTypeDef  TIM_OCInitStructure; 

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    GPIO_StructInit(&GPIO_InitStructure);

    TIM_OCStructInit(&TIM_OCInitStructure); 
    
    if ((TIM_PWM_FREQ + freq - 1)/(freq) < 65536)
    {
        TIM_TimeBaseStructure.TIM_Period    = (TIM_PWM_FREQ + freq - 1)/(freq); // period in us
        TIM_TimeBaseStructure.TIM_Prescaler = (24 - 1); // in us
    }
    else
    {
        uint32_t ulTemp = 72000000;
        uint32_t ulRate = 1;
    
        while( ulTemp/ (freq * ulRate) > 65535)
        {
            ulRate += 10;
        }
    
        ulTemp = 72000000 / ulRate;
        
        TIM_TimeBaseStructure.TIM_Period    = (ulTemp + freq - 1)/(freq); // period in us
        TIM_TimeBaseStructure.TIM_Prescaler = (ulRate - 1); // in us
    }
    
    /* Time base configuration */  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;  

    
    /* Output Compare Timing Mode configuration: Channel1 */  
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;  
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // init Enable  
    TIM_OCInitStructure.TIM_Pulse       = (TIM_TimeBaseStructure.TIM_Period * duty)/100;  // ylf: init ccr value
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;//TIM_OCPolarity_High;  
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  
        
    /*Alternative function clock setup  for GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* for channel3 & 4*/
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
    
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
    
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable); 
    
    TIM_ARRPreloadConfig(TIM4, ENABLE); 
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    

}

void Display_PwmStart1(int hz)
{
     Display_Init_PWM1(hz,50);

     /* TIM Enable counter */
     TIM_Cmd(TIM1, ENABLE );
}
void Display_PwmStart3(int hz)
{
     Display_Init_PWM3(hz,50);

     /* TIM Enable counter */
     TIM_Cmd(TIM3, ENABLE );
}
void Display_PwmStart4(int hz)
{
     Display_Init_PWM4(hz,50);

     /* TIM Enable counter */
     TIM_Cmd(TIM4, ENABLE );
}

int TIM4_Handler(int Tim,int event,void *para)
{

    GPIO_TOGGLE(GPIOB,(GPIO_Pin_6 |GPIO_Pin_7));

    GPIO_TOGGLE(GPIOA,(GPIO_Pin_8));

    if (GPIO_STATE_HIGH(GPIOB,GPIO_Pin_6))
    {
        Display.iPwmSyncEvent |= 0XC;
    }
    else
    {
        Display.iPwmSyncEvent &= ~0XC;
    }   

    if (GPIO_STATE_HIGH(GPIOA,GPIO_Pin_8))
    {
        Display.iPwmSyncEvent |= 0X10;
    }
    else
    {
        Display.iPwmSyncEvent &= ~0X10;
    }    
    
    return 0;
}


void TIM_Init_4(int iFreq)
{
    TIM_Init_General(TIMER3,iFreq);

    TIM_InstallHandler(TIMER3,TIM_IT_Update,TIM4_Handler,NULL);

    {
       GPIO_InitTypeDef GPIO_InitStructure;
        
       /* Enable the GPIO_LED Clock */
       RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
       
       /* Configure the GPIO_LED pin */
       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 |GPIO_Pin_7;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_Init(GPIOB, &GPIO_InitStructure);

       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
       GPIO_Init(GPIOA, &GPIO_InitStructure);

     }    
}



/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void Display_PwmStop(void)
{

    TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_Active);
    
    TIM_SelectOCxM(TIM3, TIM_Channel_3, TIM_ForcedAction_Active);
    TIM_SelectOCxM(TIM3, TIM_Channel_4, TIM_ForcedAction_Active);

    TIM_SelectOCxM(TIM4, TIM_Channel_1, TIM_ForcedAction_Active);
    TIM_SelectOCxM(TIM4, TIM_Channel_2, TIM_ForcedAction_Active);

    TIM_Cmd(TIM1, DISABLE );
    TIM_Cmd(TIM4, DISABLE );
    TIM_Cmd(TIM3, DISABLE );
}


void Display_SwitchMultiplex(void)
{
   /* get data & switch to next channel */
   int iRet;

   /* read data */
   iRet = mAD7799_WaitBusy(0);
   if (!iRet)
   {
       Display.iData[0][Display.iSubChlIdx] = Display_AD7799_Read(0);
   }

   iRet = mAD7799_WaitBusy(1);
   if (!iRet)
   {
       Display.iData[1][Display.iSubChlIdx] = Display_AD7799_Read(1);
   }
   
   Display.iSubChlIdx  = (Display.iSubChlIdx + 1) % 3;

   Display_AD7799_Start(0,Display.iSubChlIdx,AD7799_GAIN0,AD7799_UR7,(AD7799_MOD0<<5)); 
   
   Display_AD7799_Start(1,Display.iSubChlIdx,AD7799_GAIN0,AD7799_UR7,(AD7799_MOD0<<5));  
   
}


#endif
