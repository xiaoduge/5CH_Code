#ifndef _LCD12684_DRIVER_H_
#define _LCD12684_DRIVER_H_

// 12684 Vertical Scan !!!

#define LCD_PIXEL_WIDTH          128
#define LCD_PIXEL_HEIGHT         64

#define LCD_PIXEL_PAGE           8

#define LCD_PAGE_MAX             ((LCD_PIXEL_HEIGHT + LCD_PIXEL_PAGE -1) /LCD_PIXEL_PAGE)

#define LCD_PWR_STATE_OFF 0
#define LCD_PWR_STATE_ON  1

typedef enum {
    LCD12684_CMD_DISP_ON = 0XAF,
    LCD12684_CMD_DISP_OFF = 0XAE,
    LCD12684_CMD_SET_PAGE_ADDR = 0XB0,// default page address 0 (0~7)
    LCD12684_CMD_SET_COL_ADDR = 0X10,// two write:0001 A7 A6 A5 A4, 0000 A3 A2 A1 A0 (0~131)
    LCD12684_CMD_ADC_Select_Nomal = 0xa0,
    LCD12684_CMD_ADC_Select_Reverse = 0xa1,
    LCD12684_CMD_Display_Nomal = 0xa6,
    LCD12684_CMD_Display_Reverse = 0xa7,
    LCD12684_CMD_DISP_ALL_OFF = 0XA4,
    LCD12684_CMD_DISP_ALL_ON = 0XA5,
    LCD12684_CMD_Bias_Set_1_9 = 0XA2,
    LCD12684_CMD_Bias_Set_1_7 = 0XA3,
    LCD12684_CMD_RESET = 0XE2,
    LCD12684_CMD_Common_Output_Mode_Select_Nomal = 0xc0,
    LCD12684_CMD_Common_Output_Mode_Select_Reverse = 0xc8,
    LCD12684_CMD_Electronic_Volume_Mode_Set = 0x81, // two write, 0x81,data byte(0~63)
    LCD12684_CMD_Static_Indicator_OFF = 0XBC, //  two write, 0XBC,data byte(0~3)
    LCD12684_CMD_Static_Indicator_ON = 0XBD,//  
}LCD12684_CMD_ENUM;

void LCD_InitCmds(void);
void LCD_Clear(void);
void LCD_SetXY(unsigned char  xl,unsigned char yl);
void LCD_Suspend(void);
void LCD_Resume(void);
uint8_t LCD_GetPwrState(void);
void LCD_DisplayRect(uint8_t orgx,uint8_t orgy,uint8_t height,uint8_t width,uint8_t *dat);
#endif
