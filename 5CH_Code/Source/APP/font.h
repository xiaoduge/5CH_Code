#ifndef _FONT_H_
#define _FONT_H_



#define FONT12 12
#define FONT16 16

#define FONTSZIE (FONT12)

#if FONTSZIE == FONT16
#define ASCII_LENGHT (8) // 8*16
#define HZ_LENGHT (16) // 16*16 
#else
#define ASCII_LENGHT (6) // 6*12 (how to find 8*12)
#define HZ_LENGHT (12) // 12*12 
#endif

#define ASCII_NUM 95
#define HZ_NUM   (76)//+6
#define HZ_NUM24 (10 - 4)//+6

typedef struct
{
    unsigned char Index[2];
    unsigned char Msk[HZ_LENGHT*2];

} typFNT_GB16;

typedef struct
{
    unsigned char Msk[ASCII_LENGHT*2];

} typFNT_ASCII;

typedef struct
{
    unsigned char Index[2];
    unsigned char Msk[12*2];

} typFNT_GB12;

typedef struct                             
{
       unsigned char  Index[2];              
       unsigned char   Msk[72];              
}typFNT_GB24;


extern const typFNT_ASCII ASCII[ASCII_NUM];
extern const typFNT_GB12  HZ[HZ_NUM];
extern const typFNT_GB24 HZ24[HZ_NUM24] ;

extern const unsigned char Ascii5x8[];
extern const unsigned char Ascii12x24[];

#endif
