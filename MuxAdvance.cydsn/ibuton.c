/*
*********************************************************************************************************
*                                         MUX ADVANCE CODE
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
*                                             MUX ADVANCE CODE
*
*                                             CYPRESS PSoC5LP
*                                                with the
*                                            CY8C5969AXI-LP035
*
* Filename      : ibutton.c
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

/*
*********************************************************************************************************
*                                       uint8 touch_present(uint8 ibutton)
*
* Description : Revisa que hay ibutton en el lector
*               
*
* Argument(s) : uint8 ibutton, indica el lado donde se encuentra el ibutton
*
* Return(s)   : 0 no hay ibutton , 1 si hay ibutton
*
* Caller(s)   : 
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8 touch_present(uint8 ibutton){  
    uint8 present;
	if(ibutton==1){
	    IB1_Write(0);
	    CyDelayUs(500);
	    IB1_Write(1);
	    CyDelayUs(5);

	    if(!IB1_Read()){
	        return 0;
	    }

	    CyDelayUs(65);
	    present=!IB1_Read();
	    CyDelayUs(240);
	    if(present){
	        return 1;
	    }
	    else{
	        return 0;
	    } 
	}
	else{
	    IB2_Write(0);
	    CyDelayUs(500);
	    IB2_Write(1);
	    CyDelayUs(5);

	    if(!IB2_Read()){
	        return 0;
	    }

	    CyDelayUs(65);
	    present=!IB2_Read();
	    CyDelayUs(240);
	    if(present){
	        return 1;
	    }
	    else{
	        return 0;
	    }	
	}
}

/*
*********************************************************************************************************
*                                  uint8 touch_write(uint8 ibutton, uint8 dato)
*
* Description : Escribe un byte en el Ibutton
*               
*
* Argument(s) : uint8 ibutton = lado
				uint8 dato = byte que se va a escribir
*
* Return(s)   : 0 operacion correcta
				1 operacion incorrecta
*
* Caller(s)   : 
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8 touch_write(uint8 ibutton, uint8 data){
    uint8 i;
	if(ibutton==1){
	    for(i=0;i<=7;i++){
	        IB1_Write(0); 
	        CyDelayUs(10);
	        if(((data>>i)&1)){
	           IB1_Write(1);
	           CyDelayUs(10);
	           if(!IB1_Read()){
	                return 0;                
	           }
	        }
	        else{
	            IB1_Write(0); 
	            CyDelayUs(10);
	           if(IB1_Read()){
	                return 0;                
	           }            
	        }
	        CyDelayUs(50);
	        IB1_Write(1);
	        CyDelayUs(50);
	    }
	    return 1;
	}
	else{
	    for(i=0;i<=7;i++){
	        IB2_Write(0); 
	        CyDelayUs(10);
	        if(((data>>i)&1)){
	           IB2_Write(1);
	           CyDelayUs(10);
	           if(!IB2_Read()){
	                return 0;                
	           }
	        }
	        else{
	            IB2_Write(0); 
	            CyDelayUs(10);
	           if(IB2_Read()){
	                return 0;                
	           }            
	        }
	        CyDelayUs(50);
	        IB2_Write(1);
	        CyDelayUs(50);
	    }
	    return 1;	
	}
}

/*
*********************************************************************************************************
*                                   uint8 touch_read_byte(uint8 ibutton)
*
* Description : Lee un byte del ibutton, se usa para leer los bytes del serial
*               
*
* Argument(s) : uint8 ibutton = lado
*
* Return(s)   : 0 = no hubo lectura
				!=0 dato leido
*
* Caller(s)   : 
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8 touch_read_byte(uint8 ibutton){
    uint8 i, dato=0;
	if(ibutton==1){
	    for(i=0;i<=7;i++){
	        IB1_Write(0);    
	        CyDelayUs(14);
	        IB1_Write(1);
	        CyDelayUs(7);
	        dato|=(IB1_Read()<<i);
	        CyDelayUs(100);
	    }
	    return dato;
	}
	else{
	    for(i=0;i<=7;i++){
	        IB2_Write(0);    
	        CyDelayUs(14);
	        IB2_Write(1);
	        CyDelayUs(7);
	        dato|=(IB2_Read()<<i);
	        CyDelayUs(100);
	    }
	    return dato;	
	}
} 

/*
*********************************************************************************************************
*                                   uint8 crc_check(uint8 crc, uint8 dato)
*
* Description : Lee un byte del ibutton, se usa para leer los bytes del serial
*               
*
* Argument(s) : uint8 ibutton = lado
*
* Return(s)   : 0 = no hubo lectura
				!=0 dato leido
*
* Caller(s)   : 
*
* Note(s)     : none.
*********************************************************************************************************
*/
uint8 crc_check(uint8 crc, uint8 data){
    uint32 i;
    crc = crc ^ data;
    for (i = 0; i < 8; i++){
        if (crc & 0x01){
            crc = (crc >> 1) ^ 0x8C;
        }else{
            crc >>= 1;
        }
    }
    return crc;
}

/* [] END OF FILE */
