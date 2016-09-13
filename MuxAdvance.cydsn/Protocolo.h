/*
*********************************************************************************************************
*                                           GRP550/700 CODE
*
*                             (c) Copyright 2013; Sistemas Insepet LTDA
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                             GRP550/700 CODE
*
*                                             CYPRESS PSoC5LP
*                                                with the
*                                            CY8C5969AXI-LP035
*
* Filename      : Protocolo.h
* Version       : V1.00
* Programmer(s) : 
                  
*********************************************************************************************************
*/
#ifndef PROTOCOLO_H
#define PROTOCOLO_H
#include <device.h>
    
uint8 state_ex(uint8 val);
uint8 get_position(void);
uint8 get_state(uint8 dir);
void get_error(uint8 val);
void stop(uint8 val);
uint8 sale_in(uint8 val);
void sale(uint8 sidePump);
uint8 get_total(uint8 dir);
uint8 program(uint8 dir, uint8 grade, uint8 *value, uint8 preset);
uint8 price_change(uint8 dir);
#endif

//[] END OF FILE
