/*
*********************************************************************************************************
*                                         MUX ADVANCE CODE
*
*                             (c) Copyright 2015; Sistemas Insepet LTDA
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
* Filename      : I2C.c
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
*                                         uint8 read_time( void )
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
uint8 read_time(){
	uint8 status;
    uint32 i;
	for(i=0;i<=2;i++){
        I2C_1_MasterClearStatus();
        status = I2C_1_MasterSendStart(0x68, I2C_1_WRITE_XFER_MODE);
        if(I2C_1_MSTR_NO_ERROR == status)								 		/* Check if transfer completed without errors */
    	{
            status = I2C_1_MasterWriteByte(i);
            if(status != I2C_1_MSTR_NO_ERROR)
            {
                return 0;
            }
			else{
		        I2C_1_MasterSendStop(); 										/* Send Stop */
		        CyDelay(10);
		        status = I2C_1_MasterSendStart(0x68, I2C_1_READ_XFER_MODE);
		        if(I2C_1_MSTR_NO_ERROR == status){
		            time[i] = I2C_1_MasterReadByte(I2C_1_NAK_DATA);
		        }
				else{
					return 0;
				}
		        I2C_1_MasterSendStop();	
			}
        }
		else{
			return 0;
		}		
	}
	return 1;
}

/*
*********************************************************************************************************
*                                         uint8 read_date( void )
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
uint8 read_date(){
	uint8 status;
    uint32 i;
	for(i=4;i<=6;i++){
        I2C_1_MasterClearStatus();
        status = I2C_1_MasterSendStart(0x68, I2C_1_WRITE_XFER_MODE);
        if(I2C_1_MSTR_NO_ERROR == status)								 		/* Check if transfer completed without errors */
    	{
            status = I2C_1_MasterWriteByte(i);
            if(status != I2C_1_MSTR_NO_ERROR)
            {
                return 0;
            }
			else{
		        I2C_1_MasterSendStop(); 										/* Send Stop */
		        CyDelay(10);
		        status = I2C_1_MasterSendStart(0x68, I2C_1_READ_XFER_MODE);
		        if(I2C_1_MSTR_NO_ERROR == status){
		            date[i-4] = I2C_1_MasterReadByte(I2C_1_NAK_DATA);
		        }
				else{
					return 0;
				}
		        I2C_1_MasterSendStop();	
			}
        }
		else{
			return 0;
		}		
	}
	return 1;
}

/*
*********************************************************************************************************
*                                         uint8 write_time( void )
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

uint8 write_time( void ){
	uint8 status, data[6];
    uint32 i;
	data[0]=0;
	data[1]=time[0];
	data[2]=time[1];
    data[3]=time[2];
	I2C_1_MasterClearStatus();
    status = I2C_1_MasterSendStart(0x68, I2C_1_WRITE_XFER_MODE);
    if(I2C_1_MSTR_NO_ERROR == status) 
    {
        for(i=0; i<4; i++)
        {
            status = I2C_1_MasterWriteByte(data[i]);
            if(status != I2C_1_MSTR_NO_ERROR)
            {
                return 0;
            }
        }
    }
    else{
		return 0;
    }
    I2C_1_MasterSendStop();	
	return 1;
}

/*
*********************************************************************************************************
*                                         uint8 write_date( void )
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

uint8 write_date( void ){
	uint8 status, data[6];
    uint32 i;
	data[0]=4;
	data[1]=date[0];
	data[2]=date[1];
	data[3]=date[2];
	I2C_1_MasterClearStatus();
    status = I2C_1_MasterSendStart(0x68, I2C_1_WRITE_XFER_MODE);
    if(I2C_1_MSTR_NO_ERROR == status) 
    {
        for(i=0; i<4; i++)
        {
            status = I2C_1_MasterWriteByte(data[i]);
            if(status != I2C_1_MSTR_NO_ERROR)
            {
                return 0;
            }
        }
    }
    else{
		return 0;
    }
    I2C_1_MasterSendStop();	
	return 1;
}
