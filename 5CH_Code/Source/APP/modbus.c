#include <ucos_ii.h>
#include <string.h>
#include <stdio.h>

#include "stm32f10x.h"

#include "memory.h"
#include "task.h"
#include "timer.h"

#include "stm32_eval.h"

#include "serial_driver.h"

#include "Serial.h"

#include "relay.h"

#include "modbus.h"

#include "Beep.h"

#include "config.h"

#include "CanCmd.h"

#include "UartCmd.h"

#include "dica.h"

#include "Display.h"

#include "adc.h"

#include "math.h"

#define MAX_TADC_VALUE (0XFFF << ADC_Additional_Bits)
#define RREF_RESISTENCE (10000)

#define MODBUS_ADR_POS (0)
#define MODBUS_CODE_POS (1)
#define MODBUS_CONT_POS (2)

#define MODBUS_HEADER_LEN (2)

#define Start_addr_E  STM32_NV_APP_PARAM_AREA
#define backup_addr_E STM32_NV_APP_PARAM_BACK
#define Start_addr_R  ((uint8_t *)&paramarray[0])
#define dat_len       90


#define T1_ch 0x00
#define T2_ch 0x01
#define T3_ch 0x02
#define T4_ch 0x03
#define T5_ch 0x04


#define C1HR_mode 1   /*CH1 High Range */
#define C1HH_mode 2
#define C1HL_mode 3
#define C1LH_mode 4
#define C1LL_mode 5
#define C1LR_mode 6   /*CH1 Low Range */

#define C2HR_mode 7
#define C2HH_mode 8
#define C2HL_mode 9
#define C2LH_mode 10
#define C2LL_mode 11
#define C2LR_mode 12

#define C3HR_mode 13
#define C3HH_mode 14
#define C3HL_mode 15
#define C3LH_mode 16    
#define C3LL_mode 17    
#define C3LR_mode 18    

#define C4HR_mode 19
#define C4HH_mode 20
#define C4HL_mode 21
#define C4LH_mode 22
#define C4LL_mode 23
#define C4LR_mode 24

#define C5HR_mode 25
#define C5HH_mode 26
#define C5HL_mode 27
#define C5LH_mode 28
#define C5LL_mode 29
#define C5LR_mode 30

#define meas_mode 31
#define C1RT_mode 32
#define C2RT_mode 33
#define C3RT_mode 34
#define C4RT_mode 35
#define C5RT_mode 36
#define parameter_mode 37
#define backup_mode    38     /* backup */
#define rece_mode      39     /* recover */
#define stop_mode      40
#define save_mode      41

unsigned char RD_mode;                              
#define c1CRD_flg 0                     
#define c2CRD_flg 1                     
#define c3CRD_flg 2                     
#define c4CRD_flg 3                     
#define c5CRD_flg 4                     
#define OR1MEDAT 5                      
#define OR2MEDAT 6                      
#define OR3MEDAT 7                      
#define OR4MEDAT 8                      
#define OR5MEDAT 9                      
#define c0CRD_flg 10                    
#define c0ORD_flg 11                    
#define READSTOP 12                 
#define RDchk_flg 13                

#define RVRH1CAL 14                 
#define RC1HHCAL 15                 
#define RC1HLCAL 16                 
#define RC1LHCAL 17                 
#define RC1LLCAL 18                 
#define RVRL1CAL 19                 
#define RDC1_OFS 20                 
#define RDC1_CON 21    /* Read constants of channel 1*/             
#define RC1RTCAL 22    /* Read RT calibration of channel 1*/                 

#define RVRH2CAL 24                 
#define RC2HHCAL 25                 
#define RC2HLCAL 26                 
#define RC2LHCAL 27                 
#define RC2LLCAL 28                 
#define RVRL2CAL 29                 
#define RDC2_OFS 30                 
#define RDC2_CON 31                 
#define RC2RTCAL 32                 

#define RVRH3CAL 34                 
#define RC3HHCAL 35                 
#define RC3HLCAL 36                 
#define RC3LHCAL 37                 
#define RC3LLCAL 38                 
#define RVRL3CAL 39 
#define RDC3_OFS 40
#define RDC3_CON 41
#define RC3RTCAL 42

#define RVRH4CAL 44
#define RC4HHCAL 45
#define RC4HLCAL 46
#define RC4LHCAL 47
#define RC4LLCAL 48
#define RVRL4CAL 49
#define RDC4_OFS 50
#define RDC4_CON 51
#define RC4RTCAL 52

#define RVRH5CAL 54
#define RC5HHCAL 55
#define RC5HLCAL 56
#define RC5LHCAL 57
#define RC5LLCAL 58
#define RVRL5CAL 59
#define RDC5_OFS 60
#define RDC5_CON 61
#define RC5RTCAL 62

/* add according to Lefeng Brother Little Du */
#define OR1CAL 63                      
#define OR2CAL 64                      
#define OR3CAL 65                      
#define OR4CAL 66                      
#define OR5CAL 67    

//#define B3950


//#define SAFE
const float maxG25 = 18.2;
const float safePar = 1.05;

#pragma   pack(1)

struct s_segs {
    unsigned char  aucSeg1[3];
    float          fG25x;
    unsigned short tx;
    unsigned char  aucSeg2[3];
};

typedef struct
{
   struct s_segs s;

}MODBUS_CHL_DATA_STRU;

struct s_debug_segs {
    unsigned char  aucSeg1[3];
    float          fG25x;
    unsigned short vx;
    unsigned short rx;
    unsigned short tx;
    unsigned char  aucSeg2[3];
};

typedef struct
{
   struct s_debug_segs s;

}MODBUS_CHL_DEBUG_STRU;


#pragma   pack()



unsigned char work_mode;
unsigned char pcacnt;
unsigned char cmd[9];

uint8_t ent_flg;
uint8_t Asc_EN;
uint8_t Uart1flg;   /* YFL: FLAG for a valid UART FRAME RECEIVED */
uint8_t IAP_CONTR;
uint8_t ES ;

uint8_t ch1_Range_flg;
uint8_t ch2_Range_flg;
uint8_t ch3_Range_flg;
uint8_t ch4_Range_flg;
uint8_t ch5_Range_flg;

uint8_t debug_range_flg;

uint8_t AD1_EN;                 
uint8_t AD2_EN;                 
uint8_t AD3_EN;                 
uint8_t AD45_EN;                
uint8_t RDFlag;  

uint16_t value;
float Kcoef,Kp;
uint16_t CRC_Reg;
uint16_t MCRC_Reg;

uint16_t paramarray[90/2];

#define ch1HVR (*((uint16_t *)&paramarray[0]))
#define ch1HH  (*((uint16_t *)&paramarray[1]))
#define ch1HL  (*((uint16_t *)&paramarray[2]))
#define ch1LH  (*((uint16_t *)&paramarray[3])) 
#define ch1LL  (*((uint16_t *)&paramarray[4])) 
#define ch1LVR (*((uint16_t *)&paramarray[5]))
#define t1offset (*((int16_t *)&paramarray[6]))
#define const1 (*((uint16_t *)&paramarray[7]))    

#define ch2HVR (*((uint16_t *)&paramarray[8]))    
#define ch2HH  (*((uint16_t *)&paramarray[9]))     
#define ch2HL  (*((uint16_t *)&paramarray[10]))     
#define ch2LH  (*((uint16_t *)&paramarray[11]))     
#define ch2LL  (*((uint16_t *)&paramarray[12]))     
#define ch2LVR (*((uint16_t *)&paramarray[13]))    
#define t2offset (*((int16_t *)&paramarray[14]))   
#define  const2 (*((uint16_t *)&paramarray[15]))    

#define  ch3HVR (*((uint16_t *)&paramarray[16]))    
#define  ch3HH (*((uint16_t *)&paramarray[17]))     
#define  ch3HL (*((uint16_t *)&paramarray[18]))     
#define  ch3LH (*((uint16_t *)&paramarray[19]))     
#define  ch3LL (*((uint16_t *)&paramarray[20]))     
#define  ch3LVR (*((uint16_t *)&paramarray[21]))    
#define  t3offset (*((int16_t *)&paramarray[22]))   
#define  const3 (*((uint16_t *)&paramarray[23]))    

#define  ch4HVR (*((uint16_t *)&paramarray[24]))    
#define  ch4HH (*((uint16_t *)&paramarray[25]))     
#define  ch4HL (*((uint16_t *)&paramarray[26]))     
#define  ch4LH (*((uint16_t *)&paramarray[27]))     
#define  ch4LL (*((uint16_t *)&paramarray[28]))     
#define  ch4LVR (*((uint16_t *)&paramarray[29]))    
#define  t4offset (*((int16_t *)&paramarray[30]))   
#define  const4 (*((uint16_t *)&paramarray[31]))    

#define  ch5HVR (*((uint16_t *)&paramarray[32]))   
#define  ch5HH (*((uint16_t *)&paramarray[33]))     
#define  ch5HL (*((uint16_t *)&paramarray[34]))     
#define  ch5LH (*((uint16_t *)&paramarray[35]))     
#define  ch5LL (*((uint16_t *)&paramarray[36]))     
#define  ch5LVR (*((uint16_t *)&paramarray[37]))  
#define  t5offset (*((uint16_t *)&paramarray[38]))
#define  const5 (*((uint16_t *)&paramarray[39]))  

#define  RT1Ref (*((uint16_t *)&paramarray[40]))    
#define  RT2Ref (*((uint16_t *)&paramarray[41]))
#define  RT3Ref (*((uint16_t *)&paramarray[42]))
#define  RT4Ref (*((uint16_t *)&paramarray[43]))
#define  RT5Ref (*((uint16_t *)&paramarray[44]))

uint16_t c1addat;
uint16_t c2addat;
uint16_t c3addat;
uint16_t c4addat;
uint16_t c5addat;
uint16_t t1addat;
uint16_t t2addat;
uint16_t t3addat;
uint16_t t4addat;
uint16_t t5addat;

unsigned char  SendReg[9];
unsigned char  us1,us2,us3,us4,us5;

MODBUS_CHL_DATA_STRU  ch1_data;
MODBUS_CHL_DATA_STRU  ch2_data;
MODBUS_CHL_DATA_STRU  ch3_data;
MODBUS_CHL_DATA_STRU  ch4_data;
MODBUS_CHL_DATA_STRU  ch5_data;

MODBUS_CHL_DEBUG_STRU ch1_dbg_data;
MODBUS_CHL_DEBUG_STRU ch2_dbg_data;
MODBUS_CHL_DEBUG_STRU ch3_dbg_data;
MODBUS_CHL_DEBUG_STRU ch4_dbg_data;
MODBUS_CHL_DEBUG_STRU ch5_dbg_data;

unsigned char  *ch1_dbg_data_addr = (unsigned char *)&ch1_dbg_data;
unsigned char  *ch2_dbg_data_addr = (unsigned char *)&ch2_dbg_data;
unsigned char  *ch3_dbg_data_addr = (unsigned char *)&ch3_dbg_data;
unsigned char  *ch4_dbg_data_addr = (unsigned char *)&ch4_dbg_data;
unsigned char  *ch5_dbg_data_addr = (unsigned char *)&ch5_dbg_data;

unsigned char  *ch1_data_addr = (unsigned char *)&ch1_data; /*_at_ 0x0000*/;
unsigned char  *ch2_data_addr = (unsigned char *)&ch2_data; /*_at_ 0x000b*/;
unsigned char  *ch3_data_addr = (unsigned char *)&ch3_data; /*_at_ 0x0016*/;
unsigned char  *ch4_data_addr = (unsigned char *)&ch4_data; /*_at_ 0x0021*/;
unsigned char  *ch5_data_addr = (unsigned char *)&ch5_data; /*_at_ 0x002c*/;

#define  G25x1  ch1_data.s.fG25x                  /* ylf: index into ch1_data_addr */
#define  tx1    ch1_data.s.tx                     /*_at_ 0x0007*/    

#define   G25x2 ch2_data.s.fG25x                  /* ylf: index into ch1_data_addr */
#define   tx2   ch2_data.s.tx                     /*_at_ 0x0007*/    

#define   R25x3 ch3_data.s.fG25x                  /* ylf: index into ch1_data_addr */
#define   tx3   ch3_data.s.tx                     /*_at_ 0x0007*/    

#define   R25x4 ch4_data.s.fG25x                   /* ylf: index into ch1_data_addr */
#define   tx4   ch4_data.s.tx                      /*_at_ 0x0007*/ 

#define   R25x5 ch5_data.s.fG25x                   /* ylf: index into ch1_data_addr */
#define   tx5   ch5_data.s.tx                      /*_at_ 0x0007*/   


#define  G25x1_dbg  ch1_dbg_data.s.fG25x                 
#define  vx1_dbg    ch1_dbg_data.s.vx                         
#define  rx1_dbg    ch1_dbg_data.s.rx                        
#define  tx1_dbg    ch1_dbg_data.s.tx                        

#define   G25x2_dbg ch2_dbg_data.s.fG25x                  
#define  vx2_dbg    ch2_dbg_data.s.vx                       
#define  rx2_dbg    ch2_dbg_data.s.rx                        
#define  tx2_dbg    ch2_dbg_data.s.tx                        

#define   R25x3_dbg ch3_dbg_data.s.fG25x                  
#define  vx3_dbg    ch3_dbg_data.s.vx                         
#define  rx3_dbg    ch3_dbg_data.s.rx                        
#define  tx3_dbg    ch3_dbg_data.s.tx                        

#define   R25x4_dbg ch4_dbg_data.s.fG25x                   
#define  vx4_dbg    ch4_dbg_data.s.vx                         
#define  rx4_dbg    ch4_dbg_data.s.rx                        
#define  tx4_dbg    ch4_dbg_data.s.tx                        

#define   R25x5_dbg ch5_dbg_data.s.fG25x                   
#define  vx5_dbg    ch5_dbg_data.s.vx                        
#define  rx5_dbg    ch5_dbg_data.s.rx                        
#define  tx5_dbg    ch5_dbg_data.s.tx                        


uint16_t RT10k [] = {
44626,42326,40159,38115,36187,34367,32650,  //-6~0
31029,29498,28051,26684,25391,24168,23011,21916,20880,19898,  //1-10
18969,18088,17253,16461,15710,14998,14322,13680,13070,12492,  //11-20
11942,11419,10922,10449,10000,9572,9165,8778,8409,8057, //21-30
7722,7403,7099,6808,6531,6268,6016,5776,5546,5327,  //31-40
5118,4918,4727,4544,4369,4202,4042,3889,3743,3603,  //41-50
3468,3340,3217,3099,2986,2878,2775,2675,2580,2488,  //51-60
2400,2316,2235,2157,2083,2011,1943,1876,1813,1752,  //61-70
1693,1636,1582,1530,1479,1431,1384,1340,1297,1255,  //71-80
1215,1177,1140,1104,1070,1037,1005,974,944,915,     //81-90
888,861,835,810,786,763,741,719,699,679             //90-100
};

void HEXto_SendReg(uint8_t ch);

uint16_t get_trx(int Rx) 
{
  uint16_t i;
  uint16_t Rx1,Rx2;
  int tx;
  for(i=0;i<107;i++)
  {
    Rx1 = RT10k [i];
    Rx2 = RT10k [i+1];
    if (Rx<=Rx1 & Rx > Rx2)
    {
      tx=((uint16_t)i*10-60+(uint32_t)(Rx1-Rx)*10/(Rx1-Rx2));
      return(tx);
    }
  }
  return (tx = 1001);
}

//old PURIST A+ T Calc
int ex_get_tx(int ADx,uint16_t RTRef)
{
	uint32_t Rtx;
    double tmp;
    int tx;
    Rtx = (long)ADx*RTRef/(MAX_TADC_VALUE-ADx);  
    if(Rtx > 32650)    tmp = 0;    /* ylf: minus 100 celsius */
    else if(Rtx < 679) tmp = 100;  /* ylf: 100 celsius */
	else
	{
#ifdef B3950
		tmp = 3950 / ((3950/298.15) + log(Rtx) - log(10000)) - 273.15; //T Calc
#else
		tmp = 3984 / ((3984/298.15) + log(Rtx) - log(10000)) - 273.15; //T Calc
#endif
	}
	
    tx=(int)(tmp*10);
	
    return(tx);
}


int get_tx(int ADx,uint16_t RTRef)
{
    uint16_t Rtx;
    int tmp;
    int tx;
    Rtx = (long)ADx*RTRef/(MAX_TADC_VALUE-ADx);  
    if(Rtx > 32650)    tmp = -1000; /* ylf: minus 100 celsius */
    else if(Rtx < 679) tmp = 1000;  /* ylf: 100 celsius */
    else 
    {
        tmp = get_trx(Rtx);
        if(tmp > 340) tmp -= 0x04;
        else if(tmp > 330) tmp -= 0x02;
    }
    tx=tmp;
    return(tx);
}
                            
         
void Delay1ms(void)
{
   OSTimeDlyHMSM(0,0,0,1);  
}
void tdelay(uint16_t t)
{
    OSTimeDlyHMSM(0,0,0,t);  
}


typedef void (*modbus_msg_cb)(void);


typedef struct
{
    MsgHead msgHead;
    void *para;
}MODBUS_MSG;

#define MODBUS_MSG_LENGHT (sizeof(MODBUS_MSG)-sizeof(MsgHead))

uint8_t aucModbus_buff[32];

unsigned short calcrc16( unsigned char * buf, int len)
{
    unsigned short crc = 0;
    int i,j;
    for ( j = 0; j < len; j++)
    {
        unsigned char b = buf[j];
        for (i = 0; i < 8; i++)
        {
            crc = ((b ^ (unsigned char)crc) & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
            b >>= 1;
        }
    }
    return crc;
}

UINT8  Modbus_FillSndBuf(UINT8 ucPort,UINT8 *pData,UINT16 usLength)
{
    return Serial_FillSndBuf(ucPort + SERIAL_PORT1,pData,usLength);
}

void Modbus_MakeMessage(unsigned char *pModbusMsg,unsigned char ucLength)
{
    unsigned short usCrc;

    usCrc = (unsigned short)calcrc16(pModbusMsg,ucLength); // six bytes for preset single register

    pModbusMsg[ucLength] = (usCrc >> 8) & 0xff;
    pModbusMsg[ucLength+1] = (usCrc >> 0) & 0xff;
}

void modbus_config_cb(uint8_t ucPort)
{
    memset(&Serial[ucPort],0,offsetof(SERIAL_STRU,ucDriverType));

    Serial[ucPort].ucDriverType = MSG_DRIVER;
    Serial[ucPort].ucPortType   = RS485;
    Serial[ucPort].ucPortCtrl   = 0; // DONT CARE FOR RS232


    Serial[ucPort].UsartDef = EVAL_COM2;
    Serial[ucPort].iIrq     = EVAL_COM2_IRQn;
    Serial[ucPort].iComIdx  = COM2;
    Serial[ucPort].ucPortCtrl = STM32F103_GPA(1);

    Serial_RetriveConfig(ucPort);

    Serial[ucPort].SerialConfig.BaundRate  = 9600;
    Serial[ucPort].SerialConfig.ucDataBits = BAUD_DATA_8BITS;
    Serial[ucPort].SerialConfig.ucStopBits = BAUD_STOP_1BITS;
    Serial[ucPort].SerialConfig.ucParity   = BAUD_PARITY_NO;

}

void Uart1_Send(uint8_t i)
{
    UartCmdSendMsg(&i,1);
}

void Uart1s_Send(char *s)
{
    printf(s);
}

void HextoAsc(uint16_t dat)
{
    uint16_t m = dat;
    us1=(m%100000/10000)+0x30;
    us2=(m%10000/1000)+0x30;
    us3=(m%1000/100)+0x30;
    us4=(m%100/10)+0x30;
    us5=(m%10)+0x30;
}

void constHextoAsc(uint16_t dat)
{
    uint16_t m;
    m=dat;
    us1=(m%10000/1000)+0x30;
    us2=(m%1000/100)+0x30;
    us3=(m%100/10)+0x30;
    us4=(m%10)+0x30;
    us5= 0x20;
}

void Send_Enter()
{
    Uart1_Send(0x0d);
    Uart1_Send(0x0a);
}

void AsctoSend()
{
    Uart1_Send(us1);
    Uart1_Send(us2);
    Uart1_Send(us3);
    Uart1_Send(us4);
    Uart1_Send(us5);
    Send_Enter();
}

void TnoffsetHextoAsc(int dat)
{
    uint8_t pol; 
    uint16_t m;
    if(dat<0) { pol=1; m=~(dat+1); }
    else m=dat;
    if(pol)
    {
        us1=0x2d;
        us2=(m%100/10)+0x30;
        us3=(m%10)+0x30;
    }
    else
    {
        us1=0x20;
        us2=(m%100/10)+0x30;
        us3=(m%10)+0x30;
    }
}

void Tnoffset_Send()
{
    Uart1_Send(us1);
    Uart1_Send(us2);
    Uart1_Send(0x2e);
    Uart1_Send(us3);
    Send_Enter();
}       

void Send_RT(uint16_t RTx)
{
    uint8_t dx1,dx2,dx3,dx4;
    dx1=(RTx%100000)/10000;
    dx2=(RTx%10000)/1000;
    dx3=(RTx%1000)/100;
    dx4=(RTx%100)/10;
    Uart1_Send(dx1+0x30);
    Uart1_Send(dx2+0x30);
    Uart1_Send(0x2e);
    Uart1_Send(dx3+0x30);
    Uart1_Send(dx4+0x30);
}

void Rdchkparameter(uint8_t *addr,uint8_t cnt)
{
    uint8_t i,c;
    if(Asc_EN)
    {
        HextoAsc(ch1HVR);Uart1s_Send("C1HVR=");AsctoSend();
        HextoAsc(ch1HH);Uart1s_Send("C1HH=");AsctoSend();       
        HextoAsc(ch1HL);Uart1s_Send("C1HL=");AsctoSend();       
        HextoAsc(ch1LH);Uart1s_Send("C1LH=");AsctoSend();       
        HextoAsc(ch1LL);Uart1s_Send("C1LL=");AsctoSend();       
        HextoAsc(ch1LVR);Uart1s_Send("C1LVR=");AsctoSend(); 
        Uart1s_Send("RT1=");Send_RT(RT1Ref);Uart1s_Send("k");Send_Enter();
        TnoffsetHextoAsc(t1offset);Uart1s_Send("T1Offset=");Tnoffset_Send();
        constHextoAsc(const1);Uart1s_Send("C1Const=");AsctoSend();              

        HextoAsc(ch2HVR);Uart1s_Send("C2HVR=");AsctoSend();         
        HextoAsc(ch2HH);Uart1s_Send("C2HH=");AsctoSend();               
        HextoAsc(ch2HL);Uart1s_Send("C2HL=");AsctoSend();               
        HextoAsc(ch2LH);Uart1s_Send("C2LH=");AsctoSend();               
        HextoAsc(ch2LL);Uart1s_Send("C2LL=");AsctoSend();               
        HextoAsc(ch2LVR);Uart1s_Send("C2LVR=");AsctoSend();         
        Uart1s_Send("RT2=");Send_RT(RT2Ref);Uart1s_Send("k");Send_Enter();
        TnoffsetHextoAsc(t2offset);Uart1s_Send("T2Offset=");Tnoffset_Send();
        constHextoAsc(const2);Uart1s_Send("C2Const=");AsctoSend();                  

        HextoAsc(ch3HVR);Uart1s_Send("C3HVR=");AsctoSend();         
        HextoAsc(ch3HH);Uart1s_Send("C3HH=");AsctoSend();               
        HextoAsc(ch3HL);Uart1s_Send("C3HL=");AsctoSend();               
        HextoAsc(ch3LH);Uart1s_Send("C3LH=");AsctoSend();               
        HextoAsc(ch3LL);Uart1s_Send("C3LL=");AsctoSend();               
        HextoAsc(ch3LVR);Uart1s_Send("C3LVR=");AsctoSend();         
        Uart1s_Send("RT3=");Send_RT(RT3Ref);Uart1s_Send("k");Send_Enter();
        TnoffsetHextoAsc(t3offset);Uart1s_Send("T3Offset=");Tnoffset_Send();    
        constHextoAsc(const3);Uart1s_Send("C3Const=");AsctoSend();                  

        HextoAsc(ch4HVR);Uart1s_Send("C4HVR=");AsctoSend(); 
        HextoAsc(ch4HH);Uart1s_Send("C4HH=");AsctoSend();       
        HextoAsc(ch4HL);Uart1s_Send("C4HL=");AsctoSend();       
        HextoAsc(ch4LH);Uart1s_Send("C4LH=");AsctoSend();       
        HextoAsc(ch4LL);Uart1s_Send("C4LL=");AsctoSend();       
        HextoAsc(ch4LVR);Uart1s_Send("C4LVR=");AsctoSend(); 
        Uart1s_Send("RT4=");Send_RT(RT4Ref);Uart1s_Send("k");Send_Enter();
        TnoffsetHextoAsc(t4offset);Uart1s_Send("T4Offset=");Tnoffset_Send();
        constHextoAsc(const4);Uart1s_Send("C4Const=");AsctoSend();                  

        HextoAsc(ch5HVR);Uart1s_Send("C5HVR=");AsctoSend();         
        HextoAsc(ch5HH);Uart1s_Send("C5HH=");AsctoSend();               
        HextoAsc(ch5HL);Uart1s_Send("C5HL=");AsctoSend();               
        HextoAsc(ch5LH);Uart1s_Send("C5LH=");AsctoSend();               
        HextoAsc(ch5LL);Uart1s_Send("C5LL=");AsctoSend();               
        HextoAsc(ch5LVR);Uart1s_Send("C5LVR=");AsctoSend();         
        Uart1s_Send("RT5=");Send_RT(RT5Ref);Uart1s_Send("k");Send_Enter();
        TnoffsetHextoAsc(t5offset);Uart1s_Send("T5Offset=");Tnoffset_Send();
        constHextoAsc(const5);Uart1s_Send("C5Const=");AsctoSend();  
    }
    else
    {
        for(i=0;i<cnt;i++)
        {
            c=*(addr+i);
            Uart1_Send(c);
        }
    }
}

void MCRC16(uint8_t *dat_addr,uint8_t start,uint8_t CRC_cnt)
{
    MCRC_Reg = calcrc16(dat_addr+start,CRC_cnt);
}

void CRC16(uint8_t *dat_addr,uint8_t start,uint8_t CRC_cnt)
{
    MCRC_Reg = calcrc16(dat_addr+start,CRC_cnt);
}

uint8_t chk_CRC16(uint8_t start,uint8_t Bcnt)
{
    uint16_t m;
    m=cmd[Bcnt]*256+cmd[Bcnt+1];    
    CRC16(cmd,start,Bcnt);              
    if(m==CRC_Reg) return(1);
    else return(0); 
}

/*
����ȷ��        C0  03  01  00  00  00  FC  54
У����������    C0  03  02  00  00  00  B8  54
У�������ָ�    C0  03  03  00  00  00  44  55
����ISP����     C0  03  04  00  00  00  30  54
�����������    C0  03  06  00  00  00  88  55
У���������    C0  03  07  00  00  00  74  54
ASC MODBUS�л�  C0  03  08  00  00  00  60  57
ASC MODBUS�л�  C0  03  09  00  00  00  9C  56
ָ�����        C0  03  05  00  00  00  CC  55
ָ�����        C0  83  01              CC  A0

History: 
    ����������� : save all calibrations to FLash ,add by ylf 
    У��������� : make RDchk_flg complete by adding the command,add by ylf 2018/01/20
*/
void get_c03_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x0100: ent_flg=1;break;
    case 0x0200: work_mode=backup_mode;ent_flg=1;break;
    case 0x0300: work_mode=rece_mode;ent_flg=1;break;
    case 0x0400: IAP_CONTR=0x60;break;
    case 0x0600: work_mode=save_mode;ent_flg=1;break; 
    case 0x0700: RD_mode=RDchk_flg;break; 
    case 0x0800: Asc_EN =1;break; 
    case 0x0900: Asc_EN =0;break; 
    default: Uart1flg=0;break;
    }
}
/*

������VR    C1  02  80  81  00  02  38  91
Ӧ��        C1  02  02�ֽ�  uint_H+uint_L               
                                
�����̸���  C1  02  82  83  00  02  40  31
Ӧ��        C1  02  02�ֽ�  uint_H+uint_L               
                                
�����̵���  C1  02  84  85  00  02  C9  D1
Ӧ��        C1  02  02�ֽ�  uint_H+uint_L               
                                
�����̸���  C1  02  86  87  00  02  B1  71
Ӧ��        C1  02  02�ֽ�  uint_H+uint_L               
                                
�����̵���  C1  02  88  89  00  02  9A  12
Ӧ��        C1  02  02�ֽ�  uint_H+uint_L               
                                
������VR    C1  02  8A  8B  00  02  E2  B2
Ӧ��        C1  02  02�ֽ�  uint_H+uint_L               
                                
�¶Ȳ�������궨    C1  02  D0  D1  00  02  29  80
Ӧ��                C1  02  02�ֽ�  uint_H+uint_L               
                                
C1������                C1  02  00  03  00  06  13  19
Ӧ�����ݣ�float+int��   C1  02  06�ֽ���    EC3-EC0+T1T0                
                                
C1������    C1  02  8E  8F  00  02  13  F2
Ӧ��        C1  02  02�ֽ���    uint_H+uint_L               
                                
C1���¶�ƫ���ֵ  C1  02  8C  8D  00  02  6B  52
Ӧ��                C1  02  02�ֽ���    int_H+int_L             

*/

void get_c12_proc()
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x8081:
        {
            if(work_mode!=C1HR_mode)
            {
                //ch1_sw=1;     
                RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,1);
                work_mode=C1HR_mode;
            }
            RD_mode=RVRH1CAL;               
        }break;         
    case 0x8283:                                
        {
            if(work_mode!=C1HH_mode)
            {
               // ch1_sw=1;                       
                RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,1);
                work_mode=C1HH_mode;
            }
            RD_mode=RC1HHCAL;               
        }break;                                                             
    case 0x8485:                                
        {
            if(work_mode!=C1HL_mode)
            {
                //ch1_sw=1; 
                RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,1);
                
                work_mode=C1HL_mode;
            }
            RD_mode=RC1HLCAL;               
        }break;
    case 0x8687:
        {
            if(work_mode!=C1LH_mode)
            {
                //ch1_sw=0;  
                RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,0);
                
                work_mode=C1LH_mode;
            }
            RD_mode=RC1LHCAL;               
        }break;         
    case 0x8889:                                
        {
            if(work_mode!=C1LL_mode)
            {
                //ch1_sw=0;                       
                RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,0);
                work_mode=C1LL_mode;
            }
            RD_mode=RC1LLCAL;               
        }break;
    case 0x8a8b:                                
        {
            if(work_mode != C1LR_mode)
            {
                //ch1_sw=0;                       
                RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,0);
                work_mode=C1LR_mode;
            }
            RD_mode=RVRL1CAL;               
        }break;
    case 0x8c8d:RD_mode=RDC1_OFS;break;  /* read parameter (0x03e8) */
    case 0x8e8f:RD_mode=RDC1_CON;break;  /* read parameter */
    case 0xd0d1:                         /* set */                             
        {
            if(work_mode != C1RT_mode)
            {
                work_mode = C1RT_mode;
            }
            RD_mode=RC1RTCAL;                               
        }break;
    case 0x0003:RD_mode=OR1MEDAT;break; // ylf:  for temp & water
    case 0x0004:RD_mode=OR1CAL;debug_range_flg = 1;break; // ylf:  for temp & water
    case 0x0005:RD_mode=OR1CAL;debug_range_flg = 0;break; // ylf:  for temp & water
    default:Uart1flg=0;ES=1;break;  /* ylf: invalid frame */
    }
}


/*

дC1�¶Ȳ���ֵ  C1  03  8C  8D  int_H   int_L   6A  EE
дC1�¶Ȳ���ֵ  C1  03  8C  8D  int_H   int_L   DA  EF
Ӧ��            C1  03  8C  8D  int_H   int_L       
                                
дC1�缫����    C1  03  8E  8F  uint_H  uint_L  AC  4E
дC1�缫����    C1  03  8E  8F  03  E7  A8  0E
Ӧ��            C1  03  8E  8F  uint_H  uint_L      

*/
void get_c13_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x8c8d: /* set �¶Ȳ���ֵ */
        {
            t1offset=cmd[4]*256+cmd[5];
            work_mode=parameter_mode;       
            ent_flg=1;                                  
        }break;
    case 0x8e8f: /* set �缫���� */
        {
            const1=cmd[4]*256+cmd[5];       
            work_mode=parameter_mode;       
            ent_flg=1;
        }break;
    default:Uart1flg=0;ES=1;break;
    }
}

/*
������VR    C2  02  90  91  00  02  0E  94
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
�����̸���  C2  02  92  93  00  02  76  34
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
�����̵���  C2  02  94  95  00  02  FF  D4
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
�����̸���  C2  02  96  97  00  02  87  74
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
�����̵���  C2  02  98  99  00  02  AC  17
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
������VR�궨    C2  02  9A  9B  00  02  D4  B7
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
�¶Ȳ�������궨    C2  02  D2  D3  00  02  62  20
Ӧ��    C2  02  02�ֽ�  uint_H+uint_L               
                                
C2������    C2  02  00  0E  00  06  E3  88
Ӧ�����ݣ�float+int��   C2  02  06�ֽ���    EC3-EC0+T1T0                
                                
C2������    C2  02  9E  9F  00  02  25  F7
Ӧ��    C2  02  02�ֽ���    uint_H+uint_L               
                                
C2���¶�ƫ���ֵ  C2  02  9C  9D  00  02  5D  57
Ӧ��    C2  02  02�ֽ���    int_H+int_L             

*/
void get_c22_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0x9091:
            {
                if(work_mode!=C2HR_mode)        
                {
                    //ch2_sw=1;    
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,1);
                    work_mode = C2HR_mode;
                }
                RD_mode=RVRH2CAL;                       
            }break;
        case 0x9293:                                        
            {
                if(work_mode!=C2HH_mode)        
                {
                    //ch2_sw=1;                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,1);
                    work_mode=C2HH_mode;
                }
                RD_mode=RC2HHCAL;                       
            }break;
        case 0x9495:                                        
            {
                if(work_mode!=C2HL_mode)        
                {
                    //ch2_sw=1;                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,1);
                    work_mode=C2HL_mode;
                }
                RD_mode=RC2HLCAL;                       
            }break;
        case 0x9697:                                        
            {
                if(work_mode!=C2LH_mode)        
                {
                    //ch2_sw=0;                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,0);
                    work_mode=C2LH_mode;
                }
                RD_mode=RC2LHCAL;                       
            }break;
        case 0x9899:                                        
            {
                if(work_mode!=C2LL_mode)        
                {
                    //ch2_sw=0;                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,0);
                    work_mode=C2LL_mode;
                }
                RD_mode=RC2LLCAL;                       
            }break;
        case 0x9a9b:                                        
            {
                if(work_mode!=C2LR_mode)        
                {
                    //ch2_sw=0;                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,0);
                    work_mode=C2LR_mode;
                }
                RD_mode=RVRL2CAL;                       
            }break;
        case 0x9c9d:RD_mode=RDC2_OFS;break;
        case 0x9e9f:RD_mode=RDC2_CON;break;
        case 0xd2d3:                                                
            {
                if(work_mode!=C2RT_mode)
                {
                    work_mode=C2RT_mode;
                }
                RD_mode=RC2RTCAL;                               
            }break;
        case 0x000e:RD_mode=OR2MEDAT;break; 
        case 0x000f:RD_mode=OR2CAL;debug_range_flg = 1; break; 
        case 0x0010:RD_mode=OR2CAL;debug_range_flg = 0; break; 
        default:Uart1flg=0;ES=1;break;
    }
}

/*

дC2�¶Ȳ���ֵ  C1  03  8C  8D  int_H   int_L   6A  EE
дC2�¶Ȳ���ֵ  C1  03  8C  8D  int_H   int_L   DA  EF
Ӧ��            C1  03  8C  8D  int_H   int_L       
                                
дC2�缫����    C1  03  8E  8F  uint_H  uint_L  AC  4E
дC2�缫����    C1  03  8E  8F  03  E7  A8  0E
Ӧ��            C1  03  8E  8F  uint_H  uint_L      

*/
void get_c23_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0x9c9d:
        {
            t2offset=cmd[4]*256+cmd[5];         
            work_mode=parameter_mode;               
            ent_flg=1;                                          
        }break;
    case 0x9e9f:
        {
            const2=cmd[4]*256+cmd[5];               
            work_mode=parameter_mode;               
            ent_flg=1;
        }break;
    default:Uart1flg=0;ES=1;break;
    }
}

void get_c32_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xa0a1:                                                
            {
                if(work_mode!=C3HR_mode)                
                {
                    //ch3_sw=1;                                       
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,1);
                    work_mode=C3HR_mode;
                }
                RD_mode=RVRH3CAL;                               
            }break;         
        case 0xa2a3:                                                
            {
                if(work_mode!=C3HH_mode)                
                {
                    //ch3_sw=1;                                       
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,1);
                    work_mode=C3HH_mode;
                }
                RD_mode=RC3HHCAL;                                   
            }break;                                                             
        case 0xa4a5:                                                    
            {
                if(work_mode!=C3HL_mode)                    
                {
                    //ch3_sw=1;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,1);
                    work_mode=C3HL_mode;
                }
                RD_mode=RC3HLCAL;                                   
            }break;
        case 0xa6a7:                                                    
            {
                if(work_mode!=C3LH_mode)                    
                {
                    //ch3_sw=0;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,0);
                    work_mode=C3LH_mode;
                }
                RD_mode=RC3LHCAL;                                   
            }break;         
        case 0xa8a9:                                                    
            {
                if(work_mode!=C3LL_mode)                    
                {
                    //ch3_sw=0;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,0);
                    work_mode=C3LL_mode;
                }
                RD_mode=RC3LLCAL;                                   
            }break;
        case 0xaaab:                                                    
            {
                if(work_mode!=C3LR_mode)                    
                {
                    //ch3_sw=0;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,0);
                    work_mode=C3LR_mode;
                }
                RD_mode=RVRL3CAL;                                   
            }break;
        case 0xacad:RD_mode=RDC3_OFS;break;     
        case 0xaeaf:RD_mode=RDC3_CON;break;     
        case 0xd4d5:                                                    
            {
                if(work_mode!=C3RT_mode)
                {
                    work_mode=C3RT_mode;
                }
                RD_mode=RC3RTCAL;                                   
            }break;
        case 0x0019:RD_mode=OR3MEDAT;break;     
        case 0x001a:RD_mode=OR3CAL;debug_range_flg = 1;break;     
        case 0x001b:RD_mode=OR3CAL;debug_range_flg = 0;break;     
        default:Uart1flg=0;ES=1;break;
    }
}


void get_c33_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xacad:
            {
                t3offset=cmd[4]*256+cmd[5];         
                work_mode=parameter_mode;               
                ent_flg=1;                                          
            }break;
        case 0xaeaf:
            {
                const3=cmd[4]*256+cmd[5];               
                work_mode=parameter_mode;               
                ent_flg=1;
            }break;
        default:Uart1flg=0;ES=1;break;
    }
}

void get_c42_proc() 
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xb0b1:                                                        
            {
                if(work_mode!=C4HR_mode)                        
                {
                    //ch4_sw=1;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,1);
                    work_mode=C4HR_mode;
                }
                RD_mode=RVRH4CAL;                                       
            }break;         
        case 0xb2b3:                                                        
            {
                if(work_mode!=C4HH_mode)                        
                {
                    //ch4_sw=1;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,1);
                    work_mode=C4HH_mode;
                }
                RD_mode=RC4HHCAL;                                       
            }break;                                                             
        case 0xb4b5:                                                        
            {
                if(work_mode!=C4HL_mode)                        
                {
                    //ch4_sw=1;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,1);
                    work_mode=C4HL_mode;
                }
                RD_mode=RC4HLCAL;                                       
            }break;
        case 0xb6b7:                                                        
            {
                if(work_mode!=C4LH_mode)                        
                {
                    //ch4_sw=0;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,0);
                    work_mode=C4LH_mode;
                }
                RD_mode=RC4LHCAL;                                       
            }break;         
        case 0xb8b9:                                                        
            {
                if(work_mode!=C4LL_mode)                        
                {
                    //ch4_sw=0;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,0);
                    work_mode=C4LL_mode;
                }
                RD_mode=RC4LLCAL;                                       
            }break;
        case 0xbabb:                                                        
            {
                if(work_mode != C4LR_mode)                        
                {
                    //ch4_sw=0;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,0);
                    work_mode=C4LR_mode;
                }
                RD_mode=RVRL4CAL;                                       
            }break;
        case 0xbcbd:RD_mode=RDC4_OFS;break;         
        case 0xbebf:RD_mode=RDC4_CON;break;         
        case 0xd6d7:                                                        
            {
                if(work_mode!=C4RT_mode)
                {
                    work_mode=C4RT_mode;
                }
                RD_mode=RC4RTCAL;                                       
            }break;
        case 0x0021:RD_mode=OR4MEDAT;break;         
        case 0x0022:RD_mode=OR4CAL  ;debug_range_flg = 1;break;         
        case 0x0023:RD_mode=OR4CAL  ;debug_range_flg = 0;break;         
        default:Uart1flg=0;ES=1;break;
    }
}

void get_c43_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xbcbd:
            {
                t4offset=cmd[4]*256+cmd[5]; 
                work_mode=parameter_mode;       
                ent_flg=1;                                  
            }break;
        case 0xbebf:
            {
                const4=cmd[4]*256+cmd[5];       
                work_mode=parameter_mode;       
                ent_flg=1;
            }break;
        default:Uart1flg=0;ES=1;break;
    }
}

void get_c52_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
        case 0xc0c1:                                                    
            {
                if(work_mode!=C5HR_mode)                    
                {
                    //ch5_sw=1;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,1);
                    work_mode=C5HR_mode;
                }
                RD_mode=RVRH5CAL;                                   
            }break;         
        case 0xc2c3:                                                    
            {
                if(work_mode!=C5HH_mode)                    
                {
                    //ch5_sw=1;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,1);
                    work_mode=C5HH_mode;
                }
                RD_mode=RC5HHCAL;                                   
            }break;                                                             
        case 0xc4c5:                                                    
            {
                if(work_mode!=C5HL_mode)                    
                {
                    //ch5_sw=1;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,1);
                    work_mode=C5HL_mode;
                }
                RD_mode=RC5HLCAL;                                   
            }break;
        case 0xc6c7:                                                
            {
                if(work_mode!=C5LH_mode)                
                {
                    //ch5_sw=0;                                       
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,0);
                    work_mode=C5LH_mode;
                }
                RD_mode=RC5LHCAL;                                   
            }break;         
        case 0xc8c9:                                                    
            {
                if(work_mode!=C5LL_mode)                    
                {
                    //ch5_sw=0;                                           
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,0);
                    work_mode=C5LL_mode;
                }
                RD_mode=RC5LLCAL;                                   
            }break;
        case 0xcacb:                                                    
            {
                if(work_mode!=C5LR_mode)                        
                {
                    //ch5_sw=0;                                               
                    RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,0);
                    work_mode=C5LR_mode;
                }
                RD_mode=RVRL5CAL;                                       
            }break;
        case 0xcccd:RD_mode=RDC5_OFS;break;         
        case 0xcecf:RD_mode=RDC5_CON;break;         
        case 0xd8d9:       
            {
                if(work_mode!=C5RT_mode)
                {
                    work_mode=C5RT_mode;
                }
                RD_mode=RC5RTCAL;                                       
            }break;
        case 0x002c:RD_mode=OR5MEDAT;break;         
        case 0x002D:RD_mode=OR5CAL;debug_range_flg = 1;break;         
        case 0x002E:RD_mode=OR5CAL;debug_range_flg = 0;break;         
        default:Uart1flg=0;ES=1;break;
    }
}

void get_c53_proc()     
{
    uint16_t addr=cmd[2]*256+cmd[3];
    switch(addr)
    {
    case 0xcccd:
        {
            t5offset=cmd[4]*256+cmd[5];         
            work_mode=parameter_mode;               
            ent_flg=1;                                          
        }break;
    case 0xcecf:
        {
            const5=cmd[4]*256+cmd[5];               
            work_mode=parameter_mode;               
            ent_flg=1;
        }break;
    default:Uart1flg=0;ES=1;break;
    }
}

void Tx_data(uint8_t *dat_addr,uint8_t cnt) 
{
    Modbus_FillSndBuf(0,dat_addr,cnt);
}

uint16_t get_RtnRef(int ADx)
{
    uint16_t m;
    m=(uint32_t)(MAX_TADC_VALUE-ADx)*RREF_RESISTENCE/ADx; /* ylf: 0x2710 = 10000 */
    return(m);
}


uint16_t get_c1addat()
{
    return (Display_ReadAdc(0) >> 8);
}

uint16_t get_c2addat()
{
    return (Display_ReadAdc(1) >> 8);
}

uint16_t get_c3addat()
{
    return (Display_ReadAdc(2) >> 8);
}

uint16_t get_c4addat()
{
    return (Display_ReadAdc(3) >> 8);
}

uint16_t get_c5addat()
{
    return (Display_ReadAdc(4) >> 8);
}

uint16_t getRx_proc(uint16_t ADx,uint16_t RRef,uint16_t VR) 
{
    uint16_t Rx;
    /* ylf: simplify circuit to : power source---Rfix----Rx---GND */
    Rx = 10000*(long)(VR-RRef)/RRef;  
    Rx = (long)ADx*Rx/(VR-ADx); 
    return(Rx);
}

void tobcd_puts(uint16_t tmp,uint8_t dp)
{
    uint8_t m1,m2,m3,m4;
    m1=tmp%10000/1000;
    m2=tmp%1000/100;
    m3=tmp%100/10;
    m4=tmp%10;
    switch(dp)
    {
    case 0:
        {
            Uart1_Send(m1+0x30);
            Uart1_Send(m2+0x30);
            Uart1_Send(m3+0x30);
            Uart1_Send(m4+0x30);
        }break;
    case 1:
        {
            Uart1_Send(m1+0x30);
            Uart1_Send('.');
            Uart1_Send(m2+0x30);
            Uart1_Send(m3+0x30);
            Uart1_Send(m4+0x30);
        }break;
    case 2:
        {
            Uart1_Send(m1+0x30);
            Uart1_Send(m2+0x30);
            Uart1_Send('.');
            Uart1_Send(m3+0x30);
            Uart1_Send(m4+0x30);
        }break;
    case 3:
        {
            Uart1_Send(m1+0x30);
            Uart1_Send(m2+0x30);
            Uart1_Send(m3+0x30);
            Uart1_Send('.');
            Uart1_Send(m4+0x30);
        }break;
    }
}

void txtobcd_puts(uint16_t tx)
{
    uint8_t t1,t2,t3;
    t1=tx%1000/100;
    t2=tx%100/10;
    t3=tx%10;
    Uart1_Send(t1+0x30);
    Uart1_Send(t2+0x30);
    Uart1_Send('.');
    Uart1_Send(t3+0x30);
}


void get_Kcoef_Kp(float t)
{
    if(t<20)
    {
        Kcoef=0.00079*t*t-0.0539*t+1.873;
        Kp=0.365e-4*t*t+0.775e-3*t+0.0119;
    }
    else if(t<40)
    {
        Kcoef=0.0000125*t*t-0.02705*t+1.602;
        Kp=0.59e-4*t*t-0.9e-4*t+0.0202;
    }
    else
    {
        Kcoef=-0.000065*t*t-0.00215*t+0.91;
        Kp=0.849e-4*t*t-0.205e-2*t+0.057;
    }
}


void Tx_data_init()
{
    ch1_data_addr[0]=0xc1;
    ch1_data_addr[1]=0x02;
    ch1_data_addr[2]=0x06;

    ch2_data_addr[0]=0xc2;
    ch2_data_addr[1]=0x02;
    ch2_data_addr[2]=0x06;

    ch3_data_addr[0]=0xc3;
    ch3_data_addr[1]=0x02;
    ch3_data_addr[2]=0x06;

    ch4_data_addr[0]=0xc4;
    ch4_data_addr[1]=0x02;
    ch4_data_addr[2]=0x06;

    ch5_data_addr[0]=0xc5;
    ch5_data_addr[1]=0x02;
    ch5_data_addr[2]=0x06;

    ch1_dbg_data_addr[0]=0xc1;
    ch1_dbg_data_addr[1]=0x03;
    ch1_dbg_data_addr[2]=0x0A;
    
    ch2_dbg_data_addr[0]=0xc2;
    ch2_dbg_data_addr[1]=0x03;
    ch2_dbg_data_addr[2]=0x0A;
    
    ch3_dbg_data_addr[0]=0xc3;
    ch3_dbg_data_addr[1]=0x03;
    ch3_dbg_data_addr[2]=0x0A;
    
    ch4_dbg_data_addr[0]=0xc4;
    ch4_dbg_data_addr[1]=0x03;
    ch4_dbg_data_addr[2]=0x0A;
    
    ch5_dbg_data_addr[0]=0xc5;
    ch5_dbg_data_addr[1]=0x03;
    ch5_dbg_data_addr[2]=0x0A;
    
}

int ad_mean(uint8_t ch)   
{
   return  GetAdcData(ch);
}


void c1_puts()
{
    uint16_t tmp;
    if(Asc_EN)
    {
        Uart1s_Send("C1=");
        if(G25x1<200)
        {
            tmp=(uint16_t)((G25x1+0.05)*10);
            tobcd_puts(tmp,3);
            Uart1s_Send("��S/cm ");
        }
        else if(G25x1<2000)
        {
            tmp=(uint16_t)(G25x1+0.5);
            tobcd_puts(tmp,0);
            Uart1s_Send("��S/cm ");
        }
        else
        {
            tmp=(uint16_t)((G25x1+5)/10);
            tobcd_puts(tmp,2);
            Uart1s_Send("mS/cm  ");
        }
        Uart1s_Send("T1=");
        txtobcd_puts(tx1);
        Uart1s_Send("��");
        Send_Enter();
    }
    else
    {
        /* switch to big endian */
        unsigned char ucTemp;
        
        ucTemp = ch1_data_addr[3];
        ch1_data_addr[3] = ch1_data_addr[3 + 3];
        ch1_data_addr[3 + 3] = ucTemp;

        ucTemp = ch1_data_addr[4];
        ch1_data_addr[4] = ch1_data_addr[5];
        ch1_data_addr[5] = ucTemp;

        ucTemp = ch1_data_addr[7];
        ch1_data_addr[7] = ch1_data_addr[8];
        ch1_data_addr[8] = ucTemp;
        
        MCRC16(ch1_data_addr,0,9);
        ch1_data_addr[9]=MCRC_Reg>>8;
        ch1_data_addr[10]=MCRC_Reg;
        Tx_data(ch1_data_addr,11);  
    }
    Uart1flg=0;ES=1;
}

void ch1_meas_proc()
{
    uint16_t Rx;
    t1addat=ad_mean(T1_ch); /* ylf: internal adc (from calibrating procedure ch1HL > ch1HH )*/
    c1addat=get_c1addat();  /* ylf: external adc */
    tx1= ex_get_tx(t1addat,RT1Ref) + t1offset;
    //tx1=get_tx(t1addat,RT1Ref)+t1offset; /* ylf: tx1 is one member of ch1_data_addr */
    
    if(ch1_Range_flg)
    {
        Rx=getRx_proc(c1addat,ch1HH,ch1HVR); /* satisfy : ch1HH(100) < ch1HL < ch1HVR */
        G25x1=(float)const1/1000/(float)Rx*1E8/(1+0.02*((float)tx1/10-25));
        if(c1addat>(ch1HL+100))
        {
            //ch1_sw=0;    /* ylf: raw adc input  data channel selection */
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,0);           
            ch1_Range_flg=0;
            tdelay(200);
        }
    }
    else
    {
        Rx=getRx_proc(c1addat,ch1LL,ch1LVR); /* satisfy : ch1LH < ch1LL(1k) < ch1LVR */
        G25x1=(float)const1/1000/(float)Rx*1E7/(1+0.02*((float)tx1/10-25));
        if(c1addat<(ch1LH-100))
        {
            //ch1_sw=1;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,1);           
            ch1_Range_flg=1;    
            tdelay(200);            
        }
    }
}

void c2_puts()
{
    uint16_t tmp;
    if(Asc_EN)
    {
        Uart1s_Send("C2=");
        if(G25x2<2)
        {
            tmp=(uint16_t)((G25x2+0.0005)*1000);
            tobcd_puts(tmp,1);
            Uart1s_Send("��S/cm  ");
        }
        else if(G25x2<20)
        {
            tmp=(uint16_t)((G25x2+0.005)*100);
            tobcd_puts(tmp,2);
            Uart1s_Send("��S/cm ");
        }
        else
        {
            tmp=(uint16_t)((G25x2+0.05)*10);
            tobcd_puts(tmp,3);
            Uart1s_Send("��S/cm ");
        }
        Uart1s_Send("T2=");
        txtobcd_puts(tx2);
        Uart1s_Send("��");
        Send_Enter();
    }
    else
    {
        /* switch to big endian */
        unsigned char ucTemp;
        
        ucTemp = ch2_data_addr[3];
        ch2_data_addr[3] = ch2_data_addr[3 + 3];
        ch2_data_addr[3 + 3] = ucTemp;

        ucTemp = ch2_data_addr[4];
        ch2_data_addr[4] = ch2_data_addr[5];
        ch2_data_addr[5] = ucTemp;

        ucTemp = ch2_data_addr[7];
        ch2_data_addr[7] = ch2_data_addr[8];
        ch2_data_addr[8] = ucTemp;
    
        MCRC16(ch2_data_addr,0,9);
        ch2_data_addr[9]=MCRC_Reg>>8;
        ch2_data_addr[10]=MCRC_Reg;
        Tx_data(ch2_data_addr,11);
    }
    Uart1flg=0;ES=1;
}

void ch2_meas_proc()
{
    uint16_t Rx;
    t2addat=ad_mean(T2_ch);
    c2addat=get_c2addat();  
    tx2= ex_get_tx(t2addat,RT2Ref) + t2offset;
    //tx2=get_tx(t2addat,RT2Ref)+t2offset;        

    if(ch2_Range_flg)                                                   
    {
        Rx=getRx_proc(c2addat,ch2HH,ch2HVR);    /* satisfy : ch2HH(10k) < ch2HL < ch2HVR */
        G25x2=(float)const2/1000/(float)Rx*1E6/(1+0.02*((float)tx2/10-25));
        if(c2addat>(ch2HL+100))
        {
            //ch2_sw=0;  
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,0);
            ch2_Range_flg=0;                                
            tdelay(200);                                        
        }
    }
    else                                                                        
    {
        Rx=getRx_proc(c2addat,ch2LL,ch2LVR);/* satisfy : ch2LH < ch2LL(100k) < ch2LVR */
        G25x2=(float)const2/1000/(float)Rx*1E5/(1+0.02*((float)tx2/10-25));
        if(c2addat<(ch2LH-100))
        {
            //ch2_sw=1;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,1);
            ch2_Range_flg=1;    
            tdelay(200);            
        }
    }
}

void c3_puts()
{
    uint16_t tmp;
    if(Asc_EN)
    {
        Uart1s_Send("C3=");
        if(R25x3<2)
        {
            tmp=(uint16_t)((R25x3+0.0005)*1000);
            tobcd_puts(tmp,1);                  
        }
        else
        {
            tmp=(uint16_t)((R25x3+0.005)*100);
            tobcd_puts(tmp,2);
        }
        Uart1s_Send("M��*cm ");
        Uart1s_Send("T3=");
        txtobcd_puts(tx3);
        Uart1s_Send("��");
        Send_Enter();
    }
    else
    {
        unsigned char ucTemp;
        
        ucTemp = ch3_data_addr[3];
        ch3_data_addr[3] = ch3_data_addr[3 + 3];
        ch3_data_addr[3 + 3] = ucTemp;

        ucTemp = ch3_data_addr[4];
        ch3_data_addr[4] = ch3_data_addr[5];
        ch3_data_addr[5] = ucTemp;

        ucTemp = ch3_data_addr[7];
        ch3_data_addr[7] = ch3_data_addr[8];
        ch3_data_addr[8] = ucTemp;   
        
        MCRC16(ch3_data_addr,0,9);
        ch3_data_addr[9]=MCRC_Reg>>8;
        ch3_data_addr[10]=MCRC_Reg;
        Tx_data(ch3_data_addr,11);
    }
    Uart1flg=0;ES=1;
}

void ch3_meas_proc()
{
    uint16_t Rx;
    t3addat=ad_mean(T3_ch);
    c3addat=get_c3addat();
    tx3= ex_get_tx(t3addat,RT3Ref) + t3offset;
    //tx3=get_tx(t3addat,RT3Ref)+t3offset;
    get_Kcoef_Kp(((float)tx3)/10);

    if(ch3_Range_flg)
    {
        Rx=getRx_proc(c3addat,ch3HH,ch3HVR);/* satisfy : ch3HH(10k) < ch3HL < ch3HVR */
        R25x3=1/(Kcoef*(1e7/((float)Rx*(float)const3)-Kp)+0.05482);
        if(c3addat>(ch3HL+100))
        {
            //ch3_sw=0;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,0);
            ch3_Range_flg=0;    
            tdelay(200);
        }
    }
    else
    {
        Rx=getRx_proc(c3addat,ch3LL,ch3LVR);/* satisfy : ch3LH < ch3LL(100k) < ch3LVR */
        R25x3=1/(Kcoef*(1e6/((float)Rx*(float)const3)-Kp)+0.05482);
        if(c3addat<(ch3LH-100))
        {
            //ch3_sw=1;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,1);
            ch3_Range_flg=1;
            tdelay(200);
        }
    }

#ifdef SAFE
	R25x3 *= safePar;
	if(R25x3 > maxG25)
	{
		R25x3 = 18.2;
	}
#endif
}

void c4_puts()
{
    uint16_t tmp;
    if(Asc_EN)
    {
        Uart1s_Send("C4=");
        if(R25x4<2)
        {
            tmp=(uint16_t)((R25x4+0.0005)*1000);
            tobcd_puts(tmp,1);                  
        }
        else
        {
            tmp=(uint16_t)((R25x4+0.005)*100);
            tobcd_puts(tmp,2);
        }
        Uart1s_Send("M��*cm ");
        Uart1s_Send("T4=");
        txtobcd_puts(tx4);
        Uart1s_Send("��");
        Send_Enter();
    }
    else
    {
        unsigned char ucTemp;
        
        ucTemp = ch4_data_addr[3];
        ch4_data_addr[3] = ch4_data_addr[3 + 3];
        ch4_data_addr[3 + 3] = ucTemp;

        ucTemp = ch4_data_addr[4];
        ch4_data_addr[4] = ch4_data_addr[5];
        ch4_data_addr[5] = ucTemp;

        ucTemp = ch4_data_addr[7];
        ch4_data_addr[7] = ch4_data_addr[8];
        ch4_data_addr[8] = ucTemp; 
        
        MCRC16(ch4_data_addr,0,9);
        ch4_data_addr[9]=MCRC_Reg>>8;
        ch4_data_addr[10]=MCRC_Reg;
        Tx_data(ch4_data_addr,11);
    }
    Uart1flg=0;ES=1;
}

void ch4_meas_proc()
{
    uint16_t Rx;
    t4addat=ad_mean(T4_ch);
    c4addat=get_c4addat();
    tx4= ex_get_tx(t4addat,RT4Ref) + t4offset;
    //tx4=get_tx(t4addat,RT4Ref)+t4offset;
    get_Kcoef_Kp((float)tx4/10);
    if(ch4_Range_flg)
    {
        Rx=getRx_proc(c4addat,ch4HH,ch4HVR); /* satisfy : ch3HH(10k) < ch3HL < ch3HVR */
        R25x4=1/(Kcoef*(1e7/((float)Rx*(float)const4)-Kp)+0.05482);
        if(c4addat>(ch4HL+100))
        {
            //ch4_sw=0;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,0);
            ch4_Range_flg=0;
            tdelay(200);
        }
    }
    else
    {
        Rx=getRx_proc(c4addat,ch4LL,ch4LVR);
        R25x4=1/(Kcoef*(1e6/((float)Rx*(float)const4)-Kp)+0.05482);
        if(c4addat<(ch4LH-100))
        {
            //ch4_sw=1;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,1);
            ch4_Range_flg=1;
            tdelay(200);
        }
    }

#ifdef SAFE
	R25x4 *= safePar;
	if(R25x4 > maxG25)
	{
		R25x4 = 18.2;
	}
#endif
}

void c5_puts()
{
    uint16_t tmp;
    if(Asc_EN)
    {
        Uart1s_Send("C5=");
        if(R25x5<2)
        {
            tmp=(uint16_t)((R25x5+0.0005)*1000);
            tobcd_puts(tmp,1);                  
        }
        else
        {
            tmp=(uint16_t)((R25x5+0.005)*100);
            tobcd_puts(tmp,2);
        }
        Uart1s_Send("M��*cm  ");
        Uart1s_Send("T5=");
        txtobcd_puts(tx5);
        Uart1s_Send("��");
        Send_Enter();
    }
    else
    {
        unsigned char ucTemp;
        
        ucTemp = ch5_data_addr[3];
        ch5_data_addr[3] = ch5_data_addr[3 + 3];
        ch5_data_addr[3 + 3] = ucTemp;

        ucTemp = ch5_data_addr[4];
        ch5_data_addr[4] = ch5_data_addr[5];
        ch5_data_addr[5] = ucTemp;

        ucTemp = ch5_data_addr[7];
        ch5_data_addr[7] = ch5_data_addr[8];
        ch5_data_addr[8] = ucTemp; 
    
        MCRC16(ch5_data_addr,0,9);
        ch5_data_addr[9]=MCRC_Reg>>8;
        ch5_data_addr[10]=MCRC_Reg;
        Tx_data(ch5_data_addr,11);
    }
    Uart1flg=0;ES=1;
}

void ch5_meas_proc()
{
    uint16_t Rx;
    t5addat=ad_mean(T5_ch);
    c5addat=get_c5addat();
    tx5= ex_get_tx(t5addat,RT5Ref) + t5offset;
    //tx5=get_tx(t5addat,RT5Ref)+t5offset;
    get_Kcoef_Kp((float)tx5/10);
    if(ch5_Range_flg)
    {
        Rx=getRx_proc(c5addat,ch5HH,ch5HVR);/* satisfy : ch5HH(10k) < ch5HL < ch5HVR */
        R25x5=1/(Kcoef*(1e7/((float)Rx*(float)const5)-Kp)+0.05482);
        if(c5addat>(ch5HL+100))
        {
            //ch5_sw=0;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,0);
            ch5_Range_flg=0;
            tdelay(200);
        }
    }
    else
    {
        Rx=getRx_proc(c5addat,ch5LL,ch5LVR);
        R25x5=1/(Kcoef*(1e6/((float)Rx*(float)const5)-Kp)+0.05482);
        if(c5addat<(ch5LH-100))
        {
            //ch5_sw=1;
            RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,1);
            ch5_Range_flg=1;
            tdelay(200);
        }
    }

#ifdef SAFE
	R25x5 *= safePar;
	if(R25x5 > maxG25)
	{
		R25x5 = 18.2;
	}
#endif
}

void c1_debug_puts()
{

    /* switch to big endian */
    MCRC16(ch1_dbg_data_addr,0,13);
    ch1_dbg_data_addr[13]=MCRC_Reg>>8;
    ch1_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch1_dbg_data_addr,15);  
}

void ch1_debug_proc()
{
    uint16_t Rx;
    
    t1addat=ad_mean(T1_ch); 
    c1addat=get_c1addat();  
    tx1_dbg= ex_get_tx(t1addat,RT1Ref) + t1offset;
    //tx1_dbg=get_tx(t1addat,RT1Ref)+t1offset; 
    vx1_dbg=c1addat;
    
    if(debug_range_flg)
    {
        Rx=getRx_proc(c1addat,ch1HH,ch1HVR);
        G25x1_dbg=(float)const1/1000/(float)Rx*1E8/(1+0.02*((float)tx1_dbg/10-25));

    }
    else
    {
        Rx=getRx_proc(c1addat,ch1LL,ch1LVR);
        G25x1_dbg=(float)const1/1000/(float)Rx*1E7/(1+0.02*((float)tx1_dbg/10-25));
    }
    
    rx1_dbg = Rx; 
}

void c2_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch2_dbg_data_addr,0,13);
    ch2_dbg_data_addr[13]=MCRC_Reg>>8;
    ch2_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch2_dbg_data_addr,15);  

}

void ch2_debug_proc()
{
    uint16_t Rx;
    
    t2addat=ad_mean(T2_ch);
    c2addat=get_c2addat(); 
    tx2_dbg= ex_get_tx(t2addat,RT2Ref) + t2offset;
    //tx2_dbg=get_tx(t2addat,RT2Ref)+t2offset;        
    vx2_dbg=c2addat;

    if(debug_range_flg)                                                   
    {
        Rx=getRx_proc(c2addat,ch2HH,ch2HVR);    
        G25x2_dbg=(float)const2/1000/(float)Rx*1E6/(1+0.02*((float)tx2_dbg/10-25));
    }
    else                                                                        
    {
        Rx=getRx_proc(c2addat,ch2LL,ch2LVR);
        G25x2_dbg=(float)const2/1000/(float)Rx*1E5/(1+0.02*((float)tx2_dbg/10-25));
    }
    rx2_dbg = Rx;
}

void c3_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch3_dbg_data_addr,0,13);
    ch3_dbg_data_addr[13]=MCRC_Reg>>8;
    ch3_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch3_dbg_data_addr,15);  

}

void ch3_debug_proc()
{
    uint16_t Rx;
    t3addat=ad_mean(T3_ch);
    c3addat=get_c3addat();
    tx3_dbg= ex_get_tx(t3addat,RT3Ref) + t3offset;
   // tx3_dbg=get_tx(t3addat,RT3Ref)+t3offset;
    get_Kcoef_Kp(((float)tx3_dbg)/10);
    vx3_dbg=c3addat;

    if(debug_range_flg)
    {
        Rx=getRx_proc(c3addat,ch3HH,ch3HVR);
        R25x3_dbg=1/(Kcoef*(1e7/((float)Rx*(float)const3)-Kp)+0.05482);
    }
    else
    {
        Rx=getRx_proc(c3addat,ch3LL,ch3LVR);
        R25x3_dbg=1/(Kcoef*(1e6/((float)Rx*(float)const3)-Kp)+0.05482);
    }
    rx3_dbg = Rx;
}

void c4_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch4_dbg_data_addr,0,13);
    ch4_dbg_data_addr[13]=MCRC_Reg>>8;
    ch4_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch4_dbg_data_addr,15);  

}

void ch4_debug_proc()
{
    uint16_t Rx;
    t4addat=ad_mean(T4_ch);
    c4addat=get_c4addat();
    tx4_dbg= ex_get_tx(t4addat,RT4Ref) + t4offset;
    //tx4_dbg=get_tx(t4addat,RT4Ref)+t4offset;
    get_Kcoef_Kp(((float)tx4_dbg)/10);
    vx4_dbg=c4addat;

    if(debug_range_flg)
    {
        Rx=getRx_proc(c4addat,ch4HH,ch4HVR);
        R25x4_dbg=1/(Kcoef*(1e7/((float)Rx*(float)const4)-Kp)+0.05482);
    }
    else
    {
        Rx=getRx_proc(c4addat,ch4LL,ch4LVR);
        R25x4_dbg=1/(Kcoef*(1e6/((float)Rx*(float)const4)-Kp)+0.05482);
    }
    rx4_dbg = Rx;
}

void c5_debug_puts()
{
    /* switch to big endian */
    MCRC16(ch5_dbg_data_addr,0,13);
    ch5_dbg_data_addr[13]=MCRC_Reg>>8;
    ch5_dbg_data_addr[14]=MCRC_Reg;
    Tx_data(ch5_dbg_data_addr,15);  

}

void ch5_debug_proc()
{
    uint16_t Rx;
    t5addat=ad_mean(T4_ch);
    c5addat=get_c5addat();
    tx5_dbg= ex_get_tx(t5addat,RT5Ref) + t5offset;
    //tx5_dbg=get_tx(t5addat,RT5Ref)+t5offset;
    get_Kcoef_Kp(((float)tx5_dbg)/10);
    vx5_dbg=c5addat;

    if(debug_range_flg)
    {
        Rx=getRx_proc(c5addat,ch5HH,ch5HVR);
        R25x5_dbg=1/(Kcoef*(1e7/((float)Rx*(float)const5)-Kp)+0.05482);
    }
    else
    {
        Rx=getRx_proc(c5addat,ch5LL,ch5LVR);
        R25x5_dbg=1/(Kcoef*(1e6/((float)Rx*(float)const4)-Kp)+0.05482);
    }
    rx5_dbg = Rx;
}


void IapProgramMoreByte(uint16_t addr,uint8_t *Mcu_addr,uint8_t len)
{
     Config_SetItem(addr,len,Mcu_addr); 
}

void backup_proc(void)
{
    Config_GetItem(STM32_NV_APP_PARAM_AREA,dat_len,Start_addr_R);

    Config_SetItem(STM32_NV_APP_PARAM_BACK,dat_len,Start_addr_R);
}

void rece_proc(void)
{
    if (ERROR_SUCCESS == Config_GetItem(backup_addr_E,dat_len,Start_addr_R))
    {
        Config_SetItem(Start_addr_E,dat_len,Start_addr_R);
    }
}


void cmd_Rece(void)
{
    uint8_t i;
    if(Asc_EN)
    {
        switch(work_mode)
        {
            case backup_mode:Uart1s_Send("backup");break;
            case rece_mode:Uart1s_Send("recover");break;
            default:Uart1s_Send("ENT");break;   
        }
        Send_Enter();
    }
    else
    {
        for(i=0;i<8;i++)
        {
            Uart1_Send(cmd[i]);
        }
    }
}


void Send_Cal(uint8_t No)
{
    if(Asc_EN)
    {
        switch(No)
        {
            case 1:Uart1s_Send("C1HVR=");HextoAsc(c1addat);break;
            case 2:Uart1s_Send("C1HH=");HextoAsc(c1addat);break;
            case 3:Uart1s_Send("C1HL=");HextoAsc(c1addat);break;
            case 4:Uart1s_Send("C1LH=");HextoAsc(c1addat);break;
            case 5:Uart1s_Send("C1LL=");HextoAsc(c1addat);break;
            case 6:Uart1s_Send("C1LVR=");HextoAsc(c1addat);break;

            case 7:Uart1s_Send("C2HVR=");HextoAsc(c2addat);break;
            case 8:Uart1s_Send("C2HH=");HextoAsc(c2addat);break;
            case 9:Uart1s_Send("C2HL=");HextoAsc(c2addat);break;
            case 10:Uart1s_Send("C2LH=");HextoAsc(c2addat);break;
            case 11:Uart1s_Send("C2LL=");HextoAsc(c2addat);break;
            case 12:Uart1s_Send("C2LVR=");HextoAsc(c2addat);break;

            case 13:Uart1s_Send("C3HVR=");HextoAsc(c3addat);break;
            case 14:Uart1s_Send("C3HH=");HextoAsc(c3addat);break;
            case 15:Uart1s_Send("C3HL=");HextoAsc(c3addat);break;
            case 16:Uart1s_Send("C3LH=");HextoAsc(c3addat);break;
            case 17:Uart1s_Send("C3LL=");HextoAsc(c3addat);break;
            case 18:Uart1s_Send("C3LVR=");HextoAsc(c3addat);break;

            case 19:Uart1s_Send("C4HVR=");HextoAsc(c4addat);break;
            case 20:Uart1s_Send("C4HH=");HextoAsc(c4addat);break;
            case 21:Uart1s_Send("C4HL=");HextoAsc(c4addat);break;
            case 22:Uart1s_Send("C4LH=");HextoAsc(c4addat);break;
            case 23:Uart1s_Send("C4LL=");HextoAsc(c4addat);break;
            case 24:Uart1s_Send("C4LVR=");HextoAsc(c4addat);break;

            case 25:Uart1s_Send("C5HVR=");HextoAsc(c5addat);break;
            case 26:Uart1s_Send("C5HH=");HextoAsc(c5addat);break;
            case 27:Uart1s_Send("C5HL=");HextoAsc(c5addat);break;
            case 28:Uart1s_Send("C5LH=");HextoAsc(c5addat);break;
            case 29:Uart1s_Send("C5LL=");HextoAsc(c5addat);break;
            case 30:Uart1s_Send("C5LVR=");HextoAsc(c5addat);break;
            
            case 31:Uart1s_Send("RT1=");RT1Ref=get_RtnRef(t1addat);HextoAsc(RT1Ref);break; /* ylf: get & send ,maybe later save to e2prom */
            case 32:Uart1s_Send("RT2=");RT2Ref=get_RtnRef(t2addat);HextoAsc(RT2Ref);break;
            case 33:Uart1s_Send("RT3=");RT3Ref=get_RtnRef(t3addat);HextoAsc(RT3Ref);break;
            case 34:Uart1s_Send("RT4=");RT4Ref=get_RtnRef(t4addat);HextoAsc(RT4Ref);break;
            case 35:Uart1s_Send("RT5=");RT5Ref=get_RtnRef(t5addat);HextoAsc(RT5Ref);break;
            case 36:Uart1s_Send("C1CNT=");HextoAsc(const1);break;
            case 37:Uart1s_Send("C1OFS=");HextoAsc(t1offset);break;
            case 38:Uart1s_Send("C2CNT=");HextoAsc(const2);break;
            case 39:Uart1s_Send("C2OFS=");HextoAsc(t2offset);break;
            case 40:Uart1s_Send("C3CNT=");HextoAsc(const3);break;
            case 41:Uart1s_Send("C3OFS=");HextoAsc(t3offset);break;
            case 42:Uart1s_Send("C4CNT=");HextoAsc(const4);break;
            case 43:Uart1s_Send("C4OFS=");HextoAsc(t4offset);break;
            case 44:Uart1s_Send("C5CNT=");HextoAsc(const5);break;
            case 45:Uart1s_Send("C5OFS=");HextoAsc(t5offset);break;
        }
        AsctoSend();
    }
    else
    {
        switch(No)
        {
            case 1:HEXto_SendReg(1);break;
            case 2:HEXto_SendReg(1);break;
            case 3:HEXto_SendReg(1);break;
            case 4:HEXto_SendReg(1);break;
            case 5:HEXto_SendReg(1);break;
            case 6:HEXto_SendReg(1);break;

            case 7:HEXto_SendReg(2);break;
            case 8:HEXto_SendReg(2);break;  
            case 9:HEXto_SendReg(2);break;  
            case 10:HEXto_SendReg(2);break;
            case 11:HEXto_SendReg(2);break;
            case 12:HEXto_SendReg(2);break;

            case 13:HEXto_SendReg(3);break;
            case 14:HEXto_SendReg(3);break;
            case 15:HEXto_SendReg(3);break;
            case 16:HEXto_SendReg(3);break;
            case 17:HEXto_SendReg(3);break;
            case 18:HEXto_SendReg(3);break;

            case 19:HEXto_SendReg(4);break;
            case 20:HEXto_SendReg(4);break;
            case 21:HEXto_SendReg(4);break;
            case 22:HEXto_SendReg(4);break;
            case 23:HEXto_SendReg(4);break;
            case 24:HEXto_SendReg(4);break;

            case 25:HEXto_SendReg(5);break;
            case 26:HEXto_SendReg(5);break;
            case 27:HEXto_SendReg(5);break;
            case 28:HEXto_SendReg(5);break;
            case 29:HEXto_SendReg(5);break;
            case 30:HEXto_SendReg(5);break;
            
            case 31:HEXto_SendReg(6);break;
            case 32:HEXto_SendReg(7);break;
            case 33:HEXto_SendReg(8);break;
            case 34:HEXto_SendReg(9);break;
            case 35:HEXto_SendReg(10);break;
            case 36:HEXto_SendReg(11);break;
            case 37:HEXto_SendReg(12);break;
            case 38:HEXto_SendReg(13);break;
            case 39:HEXto_SendReg(14);break;
            case 40:HEXto_SendReg(15);break;
            case 41:HEXto_SendReg(16);break;
            case 42:HEXto_SendReg(17);break;
            case 43:HEXto_SendReg(18);break;
            case 44:HEXto_SendReg(19);break;
            case 45:HEXto_SendReg(20);break;
        }
        Tx_data(SendReg,7);
    }
    Uart1flg=0;ES=1;
}


void HEXto_SendReg(uint8_t ch)                        
{
    uint16_t tmp;
    switch(ch)
    {
        case  1:SendReg[0]=0xC1;tmp=c1addat;break;   
        case  2:SendReg[0]=0xC2;tmp=c2addat;break;
        case  3:SendReg[0]=0xC3;tmp=c3addat;break;
        case  4:SendReg[0]=0xC4;tmp=c4addat;break;
        case  5:SendReg[0]=0xC5;tmp=c5addat;break;
        case  6:SendReg[0]=0xC1;RT1Ref=get_RtnRef(t1addat);tmp=RT1Ref;break; /* ylf: first get then send ,maybe later save to e2prom */
        case  7:SendReg[0]=0xC2;RT2Ref=get_RtnRef(t2addat);tmp=RT2Ref;break;
        case  8:SendReg[0]=0xC3;RT3Ref=get_RtnRef(t3addat);tmp=RT3Ref;break;
        case  9:SendReg[0]=0xC4;RT4Ref=get_RtnRef(t4addat);tmp=RT4Ref;break;
        case 10:SendReg[0]=0xC5;RT5Ref=get_RtnRef(t5addat);tmp=RT5Ref;break;
        case 11:SendReg[0]=0xC1;tmp=const1;break;
        case 12:SendReg[0]=0xC1;tmp=t1offset;break;
        case 13:SendReg[0]=0xC2;tmp=const2;break;
        case 14:SendReg[0]=0xC2;tmp=t2offset;break;
        case 15:SendReg[0]=0xC3;tmp=const3;break;
        case 16:SendReg[0]=0xC3;tmp=t3offset;break;
        case 17:SendReg[0]=0xC4;tmp=const4;break;
        case 18:SendReg[0]=0xC4;tmp=t4offset;break;
        case 19:SendReg[0]=0xC5;tmp=const5;break;
        case 20:SendReg[0]=0xC5;tmp=t5offset;break;
    }
    SendReg[1]=0x02;
    SendReg[2]=0x02;
    SendReg[3]=tmp>>8;
    SendReg[4]=tmp;
    MCRC16(SendReg,0,5);            
    SendReg[5]=MCRC_Reg>>8;
    SendReg[6]=MCRC_Reg;
}


void Modbus_MsgHandler(uint8_t *pData, uint8_t ucLen)
{
    MainAlarmWithDuration(1);

    memcpy(cmd,pData,ucLen);
    
    switch(cmd[0]*256+cmd[1])
    {
    case 0xc003:
        {
            if((cmd[4]*256+cmd[5])==0)
            {
                if(chk_CRC16(0,6))  
                {
                    ES=0;
                    Uart1flg=1;         
                    get_c03_proc(); 
                }
            }
            else return;
        }break;
    case 0xc102: /* get wq & temperature measurement result */
        {
            if((cmd[4]*256+cmd[5])==0x0002
               ||(cmd[4]*256+cmd[5])==0x0006
               ||(cmd[4]*256+cmd[5])==0x000A) /* ylf: we go 0006 */
            {
                if(chk_CRC16(0,6))      
                {
                    ES=0;Uart1flg=1;    
                    get_c12_proc();     
                }
            }
        }break;
    case 0xc103: 
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;    
                get_c13_proc();     
            }
        }break;
    case 0xc202:
        {
            if((cmd[4]*256+cmd[5])==0x0002
                ||(cmd[4]*256+cmd[5])==0x0006
                ||(cmd[4]*256+cmd[5])==0x000A)
            {
                if(chk_CRC16(0,6))  
                {
                    ES=0;Uart1flg=1;
                    get_c22_proc(); 
                }
            }
        }break;
    case 0xc203:
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;    
                get_c23_proc();     
            }
        }break;
    case 0xc302:
        {
            if((cmd[4]*256+cmd[5])==0x0002
                ||(cmd[4]*256+cmd[5])==0x0006
                ||(cmd[4]*256+cmd[5])==0x000A)
            {
                if(chk_CRC16(0,6))      
                {
                    ES=0;Uart1flg=1;    
                    get_c32_proc(); 
                }
            }
        }break;
    case 0xc303:
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;    
                get_c33_proc();     
            }
        }break;
    case 0xc402:
        {
            if((cmd[4]*256+cmd[5])==0x0002
                ||(cmd[4]*256+cmd[5])==0x0006
                ||(cmd[4]*256+cmd[5])==0x000A)
            {
                if(chk_CRC16(0,6))      
                {
                    ES=0;Uart1flg=1;    
                    get_c42_proc();     
                }
            }
        }break;
    case 0xc403:
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;Uart1flg=1;            
                get_c43_proc();     
            }
        }break;
    case 0xc502:
        {
            if((cmd[4]*256+cmd[5])==0x0002
                ||(cmd[4]*256+cmd[5])==0x0006
                ||(cmd[4]*256+cmd[5])==0x000A)
            {
                if(chk_CRC16(0,6))  
                {
                    ES=0;Uart1flg=1;
                    get_c52_proc(); 
                }
            }
        }break;
    case 0xc503:
        {
            if(chk_CRC16(0,6))      
            {
                ES=0;
                Uart1flg=1;             
                get_c53_proc();     
            }
        }break;
    }

    switch(RD_mode)
    {
    case RVRH1CAL: c1addat=get_c1addat();Send_Cal(1);RD_mode=READSTOP;break;
    case RC1HHCAL: c1addat=get_c1addat();Send_Cal(2);RD_mode=READSTOP;break;
    case RC1HLCAL: c1addat=get_c1addat();Send_Cal(3);RD_mode=READSTOP;break;
    case RC1LHCAL: c1addat=get_c1addat();Send_Cal(4);RD_mode=READSTOP;break;
    case RC1LLCAL: c1addat=get_c1addat();Send_Cal(5);RD_mode=READSTOP;break;
    case RVRL1CAL: c1addat=get_c1addat();Send_Cal(6);RD_mode=READSTOP;break;
    case RC1RTCAL: t1addat=ad_mean(T1_ch);Send_Cal(31);RD_mode=READSTOP;break;
    case RDC1_CON: Send_Cal(36);RD_mode=READSTOP;break;
    case RDC1_OFS: Send_Cal(37);RD_mode=READSTOP;break;
        
    case RVRH2CAL: c2addat=get_c2addat();Send_Cal(7);RD_mode=READSTOP;break;  
    case RC2HHCAL: c2addat=get_c2addat();Send_Cal(8);RD_mode=READSTOP;break;  
    case RC2HLCAL: c2addat=get_c2addat();Send_Cal(9);RD_mode=READSTOP;break;  
    case RC2LHCAL: c2addat=get_c2addat();Send_Cal(10);RD_mode=READSTOP;break;  
    case RC2LLCAL: c2addat=get_c2addat();Send_Cal(11);RD_mode=READSTOP;break;  
    case RVRL2CAL: c2addat=get_c2addat();Send_Cal(12);RD_mode=READSTOP;break;  
    case RC2RTCAL: t2addat=ad_mean(T2_ch);Send_Cal(32);RD_mode=READSTOP;break;
    case RDC2_CON: Send_Cal(38);RD_mode=READSTOP;break;
    case RDC2_OFS: Send_Cal(39);RD_mode=READSTOP;break;
    
    case RVRH3CAL: c3addat=get_c3addat();Send_Cal(13);RD_mode=READSTOP;break;
    case RC3HHCAL: c3addat=get_c3addat();Send_Cal(14);RD_mode=READSTOP;break;
    case RC3HLCAL: c3addat=get_c3addat();Send_Cal(15);RD_mode=READSTOP;break;
    case RC3LHCAL: c3addat=get_c3addat();Send_Cal(16);RD_mode=READSTOP;break;
    case RC3LLCAL: c3addat=get_c3addat();Send_Cal(17);RD_mode=READSTOP;break;
    case RVRL3CAL: c3addat=get_c3addat();Send_Cal(18);RD_mode=READSTOP;break;
    case RC3RTCAL: t3addat=ad_mean(T3_ch);Send_Cal(33);RD_mode=READSTOP;break;
    case RDC3_CON: Send_Cal(40);RD_mode=READSTOP;break;
    case RDC3_OFS: Send_Cal(41);RD_mode=READSTOP;break;
        
    case RVRH4CAL: c4addat=get_c4addat();Send_Cal(19);RD_mode=READSTOP;break;
    case RC4HHCAL: c4addat=get_c4addat();Send_Cal(20);RD_mode=READSTOP;break;
    case RC4HLCAL: c4addat=get_c4addat();Send_Cal(21);RD_mode=READSTOP;break;
    case RC4LHCAL: c4addat=get_c4addat();Send_Cal(22);RD_mode=READSTOP;break;
    case RC4LLCAL: c4addat=get_c4addat();Send_Cal(23);RD_mode=READSTOP;break;
    case RVRL4CAL: c4addat=get_c4addat();Send_Cal(24);RD_mode=READSTOP;break;
    case RC4RTCAL: t4addat=ad_mean(T4_ch);Send_Cal(34);RD_mode=READSTOP;break;
    case RDC4_CON: Send_Cal(42);RD_mode=READSTOP;break;
    case RDC4_OFS: Send_Cal(43);RD_mode=READSTOP;break;

    case RVRH5CAL: c5addat=get_c5addat();Send_Cal(25);RD_mode=READSTOP;break;
    case RC5HHCAL: c5addat=get_c5addat();Send_Cal(26);RD_mode=READSTOP;break;
    case RC5HLCAL: c5addat=get_c5addat();Send_Cal(27);RD_mode=READSTOP;break;
    case RC5LHCAL: c5addat=get_c5addat();Send_Cal(28);RD_mode=READSTOP;break;
    case RC5LLCAL: c5addat=get_c5addat();Send_Cal(29);RD_mode=READSTOP;break;
    case RVRL5CAL: c5addat=get_c5addat();Send_Cal(30);RD_mode=READSTOP;break;
    case RC5RTCAL: t5addat=ad_mean(T5_ch);Send_Cal(35);RD_mode=READSTOP;break;
    case RDC5_CON: Send_Cal(44);RD_mode=READSTOP;break;
    case RDC5_OFS: Send_Cal(45);RD_mode=READSTOP;break;
    case OR1MEDAT: ch1_meas_proc();c1_puts();RD_mode=READSTOP;break;
    case OR2MEDAT: ch2_meas_proc();c2_puts();RD_mode=READSTOP;break;
    case OR3MEDAT: ch3_meas_proc();c3_puts();RD_mode=READSTOP;break;
    case OR4MEDAT: ch4_meas_proc();c4_puts();RD_mode=READSTOP;break;
    case OR5MEDAT: ch5_meas_proc();c5_puts();RD_mode=READSTOP;break;
    case OR1CAL: ch1_debug_proc();c1_debug_puts();RD_mode=READSTOP;break;
    case OR2CAL: ch2_debug_proc();c2_debug_puts();RD_mode=READSTOP;break;
    case OR3CAL: ch3_debug_proc();c3_debug_puts();RD_mode=READSTOP;break;
    case OR4CAL: ch4_debug_proc();c4_debug_puts();RD_mode=READSTOP;break;
    case OR5CAL: ch5_debug_proc();c5_debug_puts();RD_mode=READSTOP;break;
    case READSTOP: break;
    case RDchk_flg:
        {
            Rdchkparameter(Start_addr_R,90);RD_mode=READSTOP;
        }break;
    }
    
    if(ent_flg)
    {
        switch(work_mode)
        {
        case C1HR_mode:
            {
                ch1HVR=c1addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1HH_mode:
            {
                ch1HH=c1addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1HL_mode:
            {
                ch1HL=c1addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1LH_mode:
            {
                ch1LH=c1addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1LL_mode:
            {
                ch1LL=c1addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C1LR_mode:
            {
                ch1LVR=c1addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        
        case C2HR_mode:
            {
                ch2HVR=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2HH_mode:
            {
                ch2HH=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2HL_mode:
            {
                ch2HL=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2LH_mode:
            {
                ch2LH=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2LL_mode:
            {
                ch2LL=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2LR_mode:
            {
                ch2LVR=c2addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        
        case C3HR_mode:
            {
                ch3HVR=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3HH_mode:
            {
                ch3HH=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3HL_mode:
            {
                ch3HL=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3LH_mode:
            {
                ch3LH=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3LL_mode:
            {
                ch3LL=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3LR_mode:
            {
                ch3LVR=c3addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        
        case C4HR_mode:
            {
                ch4HVR=c4addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C4HH_mode:
            {
                ch4HH=c4addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C4HL_mode:
            {
                ch4HL=c4addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C4LH_mode:
            {
                ch4LH=c4addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C4LL_mode:
            {
                ch4LL=c4addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C4LR_mode:
            {
                ch4LVR=c4addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        
        case C5HR_mode:
            {
                ch5HVR=c5addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C5HH_mode:
            {
                ch5HH=c5addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C5HL_mode:
            {
                ch5HL=c5addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C5LH_mode:
            {
                ch5LH=c5addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C5LL_mode:
            {
                ch5LL=c5addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C5LR_mode:
            {
                ch5LVR=c5addat;
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;

        case C1RT_mode: /* temperature calibrating */
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C2RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C3RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C4RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case C5RT_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case parameter_mode:
            {
                //IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);
            }break;
        case backup_mode:       
            {
                backup_proc();
            }break;
        case rece_mode:         
            {
                rece_proc();
            }break;
        case save_mode:
            {
                IapProgramMoreByte(Start_addr_E,Start_addr_R,dat_len);

                work_mode = meas_mode; /* prevent repeatly saving */
            }
            break;
        }
        cmd_Rece();
        Uart1flg=0;
        ES = 1;                       /* ready for the next uart frame */
        ent_flg=0;
    }

}

void modbus_ItfProcess(Message *pMsg)
{

    // first parse message
    // 1. check modbus address
    UINT8   ucCnt = pMsg->msgHead.nMsgLen;
    UINT16  usCrc;
    // UINT8  *pModbus = (UINT8 *)pMsg->data;

//#ifdef TESTER
//#endif

    // check crc
    if (ucCnt != 8)
    {
        return;
    }

    ucCnt -= 2;
    
    usCrc = calcrc16((UINT8 *)pMsg->data,ucCnt);

    if (((usCrc >> 8 ) & 0xff) != pMsg->data[ucCnt]
        || ((usCrc >> 0 ) & 0xff) != pMsg->data[ucCnt+1])
    {
        VOS_LOG(VOS_LOG_DEBUG, "mitf crc fail %x,%x",usCrc,(pMsg->data[ucCnt]<<8)|pMsg->data[ucCnt+1]);
        return;
    }

    Modbus_MsgHandler((uint8_t *)pMsg->data,ucCnt);
    
}

void modbus_msg_Handler(Message *Msg)
{
    MODBUS_MSG *dmsg = (MODBUS_MSG *)Msg;

    if (dmsg->para)
    {
        ((modbus_msg_cb)dmsg->para)();
    }
}

void ModbusSecondTask(void)
{
    SerialTxCheck(SERIAL_PORT1,3000);
}

void ModbusInit(void)
{
   uint8_t ucPortIdx = SERIAL_PORT1;
   
   work_mode = meas_mode;
   RD_mode   = READSTOP;

   ent_flg   = 0;
   Asc_EN    = 0;
   Uart1flg  = 0;   /* YFL: FLAG for a valid UART FRAME RECEIVED */
   IAP_CONTR = 0;
   ES = 0;

#if 0
	//20180522�޸ģ����10����������1--->0
   ch1_Range_flg=0;
   ch2_Range_flg=0;
   ch3_Range_flg=0;
   ch4_Range_flg=0;
   ch5_Range_flg=0;
#endif
	//
	
    
   ch1_Range_flg=1;
   ch2_Range_flg=1;
   ch3_Range_flg=1;
   ch4_Range_flg=1;
   ch5_Range_flg=1;

   RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL5,1);
   RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL4,1);
   RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL3,1);
   RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL2,1);
   RelayLogicCtrl(RELAY_CONDUCTIVE_SAMPLE_SEL1,1);

   //end
   if (ERROR_SUCCESS != Config_GetItem(STM32_NV_APP_PARAM_AREA,dat_len,Start_addr_R))
   {
       memset(Start_addr_R,0,dat_len);
   }

   Tx_data_init();   

   for (ucPortIdx = SERIAL_PORT1; ucPortIdx <= SERIAL_PORT2;ucPortIdx++)
   {
       Serial[ucPortIdx].sccb = modbus_config_cb;
   
       Serial[ucPortIdx].mcb  = modbus_ItfProcess;
   
       SerialInitPort(ucPortIdx);   
   }
#if 0
    //Fixed calibration parameter
    ch1HVR=55384;
    ch1HH=27126;
    ch1HL=45865;
    ch1LH=17543;
    ch1LL=27226;
    ch1LVR=50010;
    RT1Ref=9.88*1000; 
    t1offset=0.0;
    const1=1000;

    ch2HVR=42756;
    ch2HH=21359;
    ch2HL=35541;
    ch2LH=14320;
    ch2LL=21439;
    ch2LVR=42783;
    RT2Ref=9.87*1000;
    t2offset=0.0;
    const2=1000;

    ch3HVR=43924;
    ch3HH=21885;
    ch3HL=36567;
    ch3LH=14649;
    ch3LL=21940;
    ch3LVR=43824;
    RT3Ref=9.88*1000;
    t3offset=0.0;
    const3=1000;

    ch4HVR=43968;
    ch4HH=21856;
    ch4HL=36593;
    ch4LH=14594;
    ch4LL=21944;
    ch4LVR=44047;
    RT4Ref=9.87*1000;
    t4offset=0.0;
    const4=1000;

    ch5HVR=43920;
    ch5HH=21871;
    ch5HL=36624;
    ch5LH=14543;
    ch5LL=21896;
    ch5LVR=43986;
    RT5Ref=9.85*1000;
    t5offset=0.0;
    const5=1000;
#endif
}


