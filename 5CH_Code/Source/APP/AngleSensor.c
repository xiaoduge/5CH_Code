#include    <ucos_ii.h>

#include    <string.h>

#include "stm32f10x.h"


#include "stm32_eval.h"

#include "app_cfg.h"


#include "ad7799_multi_Driver.h"

#include "hal_spi_driver.h"

#include "Display.h"

#include "gpio.h"

#include "AngleSensor.h"

#include "UartCmd.h"

static int sCurChannel ;

static const uint8_t spi_cs[2] = {
    STM32F103_GPB(12),     
    STM32F103_GPA(11),     
};

static const uint8_t rdy_ind[1] = {
    STM32F103_GPC(6),     
};

void hal_SPI2_CS_LOW(uint8_t ucChl)
{
    stm32_gpio_set_value(spi_cs[ucChl],0); // ylf:  inactive low

}
/**
  * @brief  Deselect sFLASH: Chip Select pin high
  */
void hal_SPI2_CS_HIGH(uint8_t ucChl)
{
    stm32_gpio_set_value(spi_cs[ucChl],1); // ylf:  inactive low
}	

int AngleSensor_cb(int event,int chl,void *para)
{
    if (DICA_SENSOR_EVENT_FALLING == event)
    {
        DISPLAY_STRU *pDisplay = (DISPLAY_STRU *)para;

        int iData;

        stm32_gpio_disable_irq(rdy_ind[0]);   

        iData = pDisplay->aiAngleValue[sCurChannel] = AngleSensor_Device_Read(sCurChannel);

        {
            iData -= 0x800000; // for sign extension
        }
        
        iData = ((250*iData/256)*10)/4/4/2500 + 2048; // to +-2048 (+- 2.5v)
        
        if (iData >= 4096)  iData = 4095;
		else if (iData < 0) iData = 0;

        switch(sCurChannel)
        {
        case 0:
            Display.pucAdc4Angle1Data[0] = iData & 0XFF;
            Display.pucAdc4Angle1Data[1] = (iData >> 8)& 0XFF;
            Display.ucFlag4Angle1        = 0;            
            break;
        default:
            Display.pucAdc4Angle2Data[0] = iData & 0XFF;
            Display.pucAdc4Angle2Data[1] = (iData >> 8)& 0XFF;
            Display.ucFlag4Angle2        = 0;            
            break;
        }

		//AngleSensor_SwitchMultiplex();

        stm32_gpio_enable_irq(rdy_ind[0],EXTI_Trigger_Falling);     

    }

    return TRUE;
}

uint8_t AngleSensor_Device_Init(uint8_t ucChl,uint8_t Gain)  
{  
    uint8_t ID;
	//uint8_t Cmd[2];  

    // sAD7799_Install_WaitBusy_Callback(AD7799_sh,0);
    mAD7799_Reset(ucChl);  
    OSTimeDlyHMSM(0,0,0,20);  
    mAD7799_ReadReg(ucChl,AD7799_MAKE_REG(AD7799_RADDR_ID),&ID,1);               //读取器件ID  
    if((ID & 0XF) != 9)
    {
        UartCmdPrintf(VOS_LOG_ERROR,"as chl:%x fail\r\n",ucChl);
        
        return 1;  
    }

    //VOS_LOG(VOS_LOG_DEBUG,"AD7799 ID =%02x",ID);
    
    mAD7799_Calibrate(ucChl,AD7799_CH0,Gain);             //通道1校准  for conductive rate
    mAD7799_Calibrate(ucChl,AD7799_CH1,Gain);             //通道2校准  for temperature

    // ylf: example code for setting CH2 to general IO
    // Cmd[0] = IO_REG_ENABLE|IO_REG_5|IO_REG_4;  
    // sAD7799_WriteReg(AD7799_WADDR_IO,Cmd,1);  
    return 0;  
}  

void AngleSensor_Device_Start(uint8_t ucChl,uint8_t CovChx,uint8_t CovGain,uint8_t CovRate,uint8_t CovMode)  
{  
    uint8_t Cmd[2];  
    
    Cmd[0] = /*CON_REG_UNIPOL|*/CovGain;  
    Cmd[1] = CON_REG_BUFFER|CovChx;  
    mAD7799_WriteRegEx(ucChl,0,AD7799_MAKE_REG(AD7799_WADDR_CONFIG),Cmd,2); 
    
    Cmd[0] = CovMode;  
    Cmd[1] = CovRate;  
    mAD7799_WriteRegEx(ucChl,1,AD7799_MAKE_REG(AD7799_WADDR_MODE),Cmd,2);  
}  

uint32_t AngleSensor_Device_Read(uint8_t ucChl)  
{  
    uint8_t  Cmd[4];  
    uint32_t D;  
    Cmd[0]=0;  
    mAD7799_ReadRegEx(ucChl,0,AD7799_MAKE_REG(AD7799_RADDR_DATA),&Cmd[1],3);  
    D = (Cmd[1]<<16)|(Cmd[2]<<8)|Cmd[3];  
    return D;  
} 


int AngleSensor_Install_WaitBusy_Callback(dica_event_callback handle,void *para,int trigs)
{
    stm32_gpio_cfg_irq(rdy_ind[0],trigs);

    switch(trigs)
    {
    case EXTI_Trigger_Rising:
        InstallSensorHandler(DICA_SENSOR_EVENT_RISING,stm32_gpio_get_ext_line(rdy_ind[0]),0,DICA_TYPE_PERIOD,handle,para);
        break;
    case EXTI_Trigger_Falling:
        InstallSensorHandler(DICA_SENSOR_EVENT_FALLING,stm32_gpio_get_ext_line(rdy_ind[0]),0,DICA_TYPE_PERIOD,handle,para);
        break;
    case EXTI_Trigger_Rising_Falling:
        InstallSensorHandler(DICA_SENSOR_EVENT_RISING,stm32_gpio_get_ext_line(rdy_ind[0]),0,DICA_TYPE_PERIOD,handle,para);
        InstallSensorHandler(DICA_SENSOR_EVENT_FALLING,stm32_gpio_get_ext_line(rdy_ind[0]),0,DICA_TYPE_PERIOD,handle,para);
        break;
    }

    return 0;
}

void AngleSensor_SecondTask(void)
{

}

void AngleSensor_SwitchMultiplex(void)
{
	hal_SPI2_CS_HIGH(sCurChannel);
	
	sCurChannel = (sCurChannel + 1) % AS_MAX_NUM;
	
	hal_SPI2_CS_LOW(sCurChannel);

}

void AngleSensor_Init(void)
{
   int iLoop;

   sCurChannel = 0;
   
   for (iLoop = 0; iLoop < AS_MAX_NUM; iLoop++)
   {
       stm32_gpio_cfgpin(spi_cs[iLoop],MAKE_PIN_CFG(GPIO_Speed_50MHz,GPIO_Mode_Out_PP)); 
       
       stm32_gpio_set_value(spi_cs[iLoop],1); // ylf:  active low
   }

   for (iLoop = 0; iLoop < 1; iLoop++)
   {
       stm32_gpio_cfgpin(rdy_ind[iLoop],MAKE_PIN_CFG(GPIO_Speed_50MHz,GPIO_Mode_IPU)); 
   }

   mAD7799_Init(HAL_SPI1);

   AngleSensor_Device_Init(0,AD7799_GAIN0);

   AngleSensor_Device_Init(1,AD7799_GAIN0);
   
   AngleSensor_Install_WaitBusy_Callback(AngleSensor_cb,&Display,EXTI_Trigger_Falling);
   
   AngleSensor_Device_Start(0,AD7799_CH0,AD7799_GAIN0,AD7799_UR2,(AD7799_MOD0<<5));
   
   AngleSensor_Device_Start(1,AD7799_CH0,AD7799_GAIN0,AD7799_UR2,(AD7799_MOD0<<5));

   /* enable channel report */
   hal_SPI2_CS_LOW(sCurChannel);

}


