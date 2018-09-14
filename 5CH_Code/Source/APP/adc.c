#include "stm32_eval.h"

#include "Timer_Driver.h"

#include "Adc_Driver.h"

#include "memory.h"
#include "msg.h"

#include "Display.h"

#include "adc.h"

#include "Beep.h"

#include <stdio.h>


#define ADC_REFER_VOLT (3347)

#define ADC_MAX_NUMBER (5)

#define ADC_MAX_INPUT_NUMBER (ADC_MAX_NUMBER)

#define ADC_Oversampling_NUM     (1<<ADC_Additional_Bits)                       /* pow(2, ADC_Additional_Bits) */
#define ADC_Oversampling_Factor  (ADC_MAX_NUMBER*ADC_Oversampling_NUM)       /* pow(4, ADC_Additional_Bits) */

extern u16 ADC_ConvertedValue[ADC_Oversampling_NUM][ADC_MAX_INPUT_NUMBER];

u16 ADC_ConvertedValue[ADC_Oversampling_NUM][ADC_MAX_NUMBER];

u32 ADC_Result[ADC_MAX_NUMBER];

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable DMA channel1 IRQ Channel -----------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;//DMA1_Channel1_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xf;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
}

int TIM2_Handler(int Tim,int event,void *para)
{
    /* Update the Number of DMA transfer */ 
    DMA1_Channel1->CNDTR = ADC_Oversampling_Factor;
    
    /* Update the Destination Memory of the DMA pointer */ 
    DMA1_Channel1->CMAR = (u32)ADC_ConvertedValue;  
    
    DMA1_Channel1->CCR |= 0x00000001;

    ADC_Cmd(ADC1, ENABLE);
    
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);

    //MainAlarmWithDuration(1);

	return 0;

}

/*******************************************************************************
* Function Name  : TIM2_Configuration
* Description    : Configures the TIM2 to generate an interrupt after each 
*                     sampling period                  
* Input          : The required oversampling period in us
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_Configuration ( u32 Sampling_Period )
{  
    int freq = 1000000/Sampling_Period;

    TIM_Init_General(TIMER1,freq);

    TIM_InstallHandler(TIMER1,TIM_IT_Update,TIM2_Handler,NULL);

}  


/*******************************************************************************
* Function Name  : CurrentMeas_Init
* Description    : Configures the Init current measurement 
* Input          : 
*       @param Sampling_Period: sampling period ,unit us
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Meas_Init(u32 Sampling_Period )
{
  /* Peripherals InitStructure define -----------------------------------------*/
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed = (GPIOSpeed_TypeDef)0;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  NVIC_Configuration();

  RCC_ADCCLKConfig(RCC_PCLK2_Div8);

  DMA_ADC_ConfigurationExt((u32)ADC_ConvertedValue,ADC_Oversampling_Factor,ADC_Channel_10,ADC_MAX_NUMBER,ADC_SampleTime_28Cycles5);

  TIM2_Configuration(Sampling_Period);
  
  ADC_SoftwareStartConvCmd(ADC1,ENABLE);
  
}

/*******************************************************************************
* Function Name  : u32 Oversampling_GetConversion
* Description    : Gives the ADC oversampled conversion result  
* Input          : None 
* Output         : Oversampled Result
* Return         : None
*******************************************************************************/
void Oversampling_GetConversion ( void )
{
  u32 index = 0;
  u32 chloop = 0;

  for (chloop = 0; chloop < ADC_MAX_NUMBER; chloop++)
  {
      ADC_Result[chloop] = 0;
  }

  /* sort */
  for (chloop = 0; chloop < ADC_MAX_NUMBER;chloop++)
  {
      for (index = 0; index < ADC_Oversampling_NUM; index++)
      {
          ADC_Result[chloop] += ADC_ConvertedValue[index][chloop];
      }
  }
    
}  

/*******************************************************************************
* Function Name  : GetAdcData
* Description    : get the current measurement 
* Input          : 
*       @param ucChl: channel number ,in range [0~2]
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t GetAdcData(uint8_t ucChl)
{
    if (ucChl >= ADC_MAX_NUMBER)
    {
        return 0;
    }
    return ADC_Result[ucChl] ;
}

/*******************************************************************************
* Function Name  : PidAdcProcess
* Description    : PID proc for adc driver module 
* Input          : 
*       @param pMsg: message pointer
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 PidAdcProcess(Message *pMsg)
{

    int iChl = 0 ;

    for (iChl = 0; iChl < 5; iChl++)
    {
        Display.aiTempData[iChl] = GetAdcData(iChl);
    }

    return 0;
}


