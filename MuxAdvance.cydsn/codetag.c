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
* Filename      : codetag.c
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
*                                        uint8 serial_codetag(uint8 side)
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

uint8 serial_codetag(uint8 side){
    uint8 buffer[20],x,check;       
    Tag_ClearRxBuffer();
    Tag_ClearTxBuffer();
    Tag_PutChar('M');
    Tag_PutChar('R');
    Tag_PutChar(side);
    CyDelay(100);
    if(Tag_GetRxBufferSize()>=9){
        x=0;
        while(Tag_GetRxBufferSize()>0){
           buffer[x]=Tag_ReadRxData();
           x++;
        }
        Tag_ClearRxBuffer();
        check=0;
        for(x=1;x<=8;x++){
            check^=buffer[x];
        }
        if(buffer[9]==check && buffer[0]=='H' && (buffer[10]&0x03)==side){
            for(x=1;x<=8;x++){
                temporal[x]=buffer[x];
            }
            temporal[0]=8;
            CyDelay(50);
            Tag_PutChar('O');
            Tag_PutChar('K');
            Tag_PutChar(side);
            return 1;
        }else{
            return 0;
        }
    }
    
    return 0;
}