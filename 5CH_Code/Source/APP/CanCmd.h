#ifndef _CAN_CMD_H_
#define _CAN_CMD_H_


#include "stm32f10x_can.h"

#include "DtypeStm32.h"

#include "msg.h"

#include "sys_time.h"


extern UINT16 CanAddress ;

void CanCmdInitizeCAN(void);

#endif

