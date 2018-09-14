#ifndef _ADC_MEASUREMENT_H_
#define _ADC_MEASUREMENT_H_

#define ADC_Additional_Bits    3   // should satisfy : 2^ADC_Additional_Bits =  ADC_Oversampling_NUM

UINT8 PidAdcProcess(Message *pMsg);

void  Oversampling_GetConversion ( void );

void ADC_Meas_Init(u32 Sampling_Period );

void CanCcbAdcItf(uint16_t usAdcValue);

void BubbleSort(u16 * pDataArray, int iDataNum)  ;

#endif
