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
* Filename      : operations.h
* Version       : V1.00
* Programmer(s) : 
                  
*********************************************************************************************************
*/
#ifndef OPERATIONS_H
#define OPERATIONS_H
#include <device.h>

uint8 compare_ppu(uint8 *ppuPump,uint8 handle,uint8 pos);
uint8 subtraction(uint8 *minuendo,uint8 *sustraendo);
uint8 higher(uint8 *value1,uint8 *value2);
void multiplication_market(uint8 *value1,uint8 *value2);
void addition_market(uint8 *value1,uint8 *value2, uint8 *value3);

#endif

//[] END OF FILE