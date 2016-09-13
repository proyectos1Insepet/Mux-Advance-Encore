/*
*********************************************************************************************************
*                                         MUX ADVANCE CODE
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
*                                            MUX ADVANCE CODE
*
*                                             CYPRESS PSoC5LP
*                                                with the
*                                            CY8C5969AXI-LP035
*
* Filename      : codebar.c
* Version       : V1.00
* Programmer(s) : 
                  
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <device.h>
#include "variables.h"

/*
*********************************************************************************************************
*                                        uint8 serial_codebar()
*
* Description : 
*               
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   :  
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8 serial_codebar(){
    uint32 size;
	if(Code_Bar_GetRxBufferSize() >= 1){
        CyDelay(100);
		size=1;
		while(Code_Bar_GetRxBufferSize()>0){
    		temporal[size]=Code_Bar_ReadRxData();
    		size++;
    	}
		temporal[0]=size-1;
		Code_Bar_ClearRxBuffer();
		CyDelay(5);
		return 1;
	}
    return 0;
}