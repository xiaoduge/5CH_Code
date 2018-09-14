#ifndef _DACx760_DRIVER_H_
#define _DACx760_DRIVER_H_

typedef enum
{
    DACX760_WADDR_NOP           = 0X0,
    DACX760_WADDR_DAC_DATA      = 0X1,
    DACX760_WADDR_REG_READ      = 0X2,
    DACX760_WADDR_CONTROL       = 0X55,
    DACX760_WADDR_RESET         = 0X56,
    DACX760_WADDR_CONFIGURATION = 0X57,
    DACX760_WADDR_DAC_GAIN_CAL  = 0X58,
    DACX760_WADDR_DAC_ZERO_CALI = 0X59,
    DACX760_WADDR_WDG_RESET     = 0X95,
        
}DACX760_WADDR_EUNM;


typedef enum
{
    DACX760_RADDR_STATUS           = 0X0,
    DACX760_RADDR_DAC_DATA         = 0X1,
    DACX760_RADDR_CONTROL          = 0X2,
    DACX760_RADDR_CONFIGURATION    = 0XB,
    DACX760_RADDR_DAC_GAIN_CALI    = 0X13,
    DACX760_RADDR_DAC_ZERO_CALI    = 0X17,

}DACX760_RADDR_EUNM;

void DACx7660_Write(uint8_t ucChl,uint32_t data);
uint32_t DACx7660_Read(uint8_t ucChl,uint32_t data);

/**
  * @brief  Initializes the peripherals used by the SPI driver.
              type
              000 0 V to +5 V
              001 0 V to +10 V
              010 ¡À5 V
              011 ¡À10 V
              100 Not allowed
              (1)
              101 4 mA to 20 mA
              110 0 mA to 20 mA
              111 0 mA to 24 mA              
*/
void DACx7660_Init_Setting(uint8_t ucChl,int type);
void DACx7660_Set_DacData(uint8_t ucChl,uint32_t data);
uint32_t DACx7660_Get_DacData(uint8_t ucChl);
void DACx7660_Set_DacCtrl(uint8_t ucChl,uint32_t data);
uint32_t DACx7660_Get_DacCtrl(uint8_t ucChl);
void DACx7660_Init(void);

#endif
