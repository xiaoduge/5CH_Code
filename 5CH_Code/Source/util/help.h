#ifndef _VOS_HELP_H_
#define _VOS_HELP_H_
/**
  ******************************************************************************
  * @file    help.h 
  * @author  VOS Team
  * @version V1.0.0
  * @date    10/10/2014
  * @brief   Interface for Mis access.
  ******************************************************************************
  *          DO NOT MODIFY ANY OF THE FOLLOWING CODE EXCEPT UNDER PERMISSION OF ITS PROPRIETOR
  * @copy
  *
  *
  * <h2><center>&copy; COPYRIGHT 2012 Shanghai ZhenNeng Corporation</center></h2>
  */ 

extern void att_helper_trim(char **pstr);
extern int Att_value_parse(char *parg,char sep,char **pptag,char **ppvalue);
extern int Att_value_split(char *parg,char sep);
extern int Att_value_count(char *parg,char sep);
extern int Att_value_split_list(char *parg,char sep,char *argv[] , int argc);
extern int Att_value_replace(char *parg,char src,char dst);
extern float Att_latof(char *strf);
extern int Att_latoi (char s[]);
extern unsigned int Att_latodate(char s[],char sep);
#endif

