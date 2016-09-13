/*
*********************************************************************************************************
*                                           MUX ADVANCE CODE
*
*                             (c) Copyright 2016; Sistemas Insepet LTDA
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
*                                          MUX ADVANCE CODE CODE
*
*                                             CYPRESS PSoC5LP
*                                                with the
*                                            CY8C5969AXI-LP035
*
* Filename      : protocol.h
* Version       : V1.00
* Programmer(s) : 
                  
*********************************************************************************************************
*/
#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <device.h>
    
uint8 get_state(uint8 pos);
uint8 get_position(void);
uint8 get_handle(uint8 pos);
uint8 get_totals(uint8 pos);
uint8 price_change(uint8 pos,uint8 handle,uint8 *value);
uint8 preset_data(uint8 pos, uint8 grade, uint8 *value, uint8 preset);
uint8 get_sale(uint8 pos);

#endif

//[] END OF FILE
