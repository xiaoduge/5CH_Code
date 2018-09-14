#ifndef _ANGLE_SENSOR_H_
#define _ANGLE_SENSOR_H_

#define AS_MAX_NUM (2)
void AngleSensor_Init(void);
void AngleSensor_SecondTask(void);
void AngleSensor_SwitchMultiplex(void);

uint32_t AngleSensor_Device_Read(uint8_t ucChl) ; 

#endif
