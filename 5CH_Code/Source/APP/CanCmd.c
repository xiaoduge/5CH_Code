#include    <ucos_ii.h>

#include    <cpu.h>
#include    <lib_def.h>
#include    <lib_mem.h>
#include    <lib_str.h>

#include    <string.h>

#include <stdarg.h>

#include "stm32f10x.h"

#include "CanCmd.h"

#include "memory.h"
#include "msg.h"
#include "timer.h"

#include "stm32_eval.h"

#include "Can_driver.h"

#include "app_cfg.h"

#include "Config.h"

#include "Beep.h"

#include "Errorcode.h"

#include "common.h"

#include "sapp.h"

#include "cminterface.h"

#include "osal_snv.h"


#include "RTC_Driver.h"

#include "Display.h"

#include "Task.h"

#include "Relay.h"

#include "Adc.h"

#define CAN_1mbps     (SystemCoreClock/2/8/1000000)       
#define CAN_500kbps   (SystemCoreClock/2/8/500000)       
#define CAN_250kbps   (SystemCoreClock/2/8/250000)
#define CAN_125kbps   (SystemCoreClock/2/8/125000)   
#define CAN_62_5kbps  (SystemCoreClock/2/8/62500)   
#define CAN_31_25kbps (SystemCoreClock/2/8/31250)      



void CanCmdInitizeCAN(void)
{
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    UINT32 dwCanId;
  #ifdef CAN_ADR_FILTER  
	  UINT32 dwMask;
  #endif  
        /* CAN register init */
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
    
    /* CAN cell init */  //  ylf: BaudRate = PCLK2/(CAN_Prescaler)/(BS1+BS2+1)
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;   // ylf: The transmit mailboxes as a transmit FIFO
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;// CAN_Mode_Normal; // CAN_Mode_LoopBack;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1  = CAN_BS1_4tq; // ylf: PROP_SEG and PHASE_SEG1
    CAN_InitStructure.CAN_BS2  = CAN_BS2_3tq;  // ylf: PHASE_SEG2
    CAN_InitStructure.CAN_Prescaler = CAN_125kbps;
    
    
    STM_EVAL_CANInit(&CAN_InitStructure);


    /* CAN filter init */
    dwCanId = 0;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_FilterInitStructure.CAN_FilterNumber = 1;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = (dwCanId >> 16) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterIdLow =  (dwCanId) & 0XFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 1;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure); 


    CAN_ITConfig(CAN1, CAN_IT_FMP0 /*| CAN_IT_FF0 | CAN_IT_FOV0*/, ENABLE);  // fifo0中断
    CAN_ITConfig(CAN1, CAN_IT_FMP1 /*| CAN_IT_FF1 | CAN_IT_FOV1*/, ENABLE);  // fifo1中断
    CAN_ITConfig(CAN1, CAN_IT_TME, DISABLE);                              // 发送中断
    //CAN_ITConfig(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC 
    //            | CAN_IT_ERR | CAN_IT_WKU | CAN_IT_SLK, ENABLE);         // ERR中断

}

