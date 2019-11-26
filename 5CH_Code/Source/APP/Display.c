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


DISPLAY_STRU Display;

static const uint8_t spi_cs[2] = {
    STM32F103_GPA(4),     
    STM32F103_GPB(3),     
};


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

void Display_SecondTask(void)
{
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

void Display_PwmSyncCallback(void)
{
    //Display_report(Display_SyncEvent_msg_cb,NULL);
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

    mAD7799_Init(HAL_SPI0);
    
    Display_AD7799_Init(0);

    Display_AD7799_Init(1);

    Display_AD7799_Start(0,0,AD7799_GAIN0,AD7799_UR12,(AD7799_MOD2<<5)); 
    
    Display_AD7799_Start(1,2,AD7799_GAIN0,AD7799_UR12,(AD7799_MOD2<<5)); 
    
    TIM_Init_3(1400);
    
}

