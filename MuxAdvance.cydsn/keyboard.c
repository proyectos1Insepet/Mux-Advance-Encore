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
* Filename      : keyboard.c
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
#include "LCD.h"

/*
*********************************************************************************************************
*               uint8 alphanumeric_keyboard(uint8 lcd, uint8 maxNumberKeys, uint8 character)
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

uint8 alphanumeric_keyboard(uint8 lcd, uint8 maxNumberKeys, uint8 character){
    if(lcd==1){
        if(LCD1_GetRxBufferSize()==8){
            if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                countAnimation1=0;
                if(numberKeys1<=(maxNumberKeys-1)){
                    if(LCD1_rxBuffer[3]>=0x20 && LCD1_rxBuffer[3]<=0x2B){ //Comandos ascii sin incluir coma (,)
                        numberKeys1++;  
                        bufferLCD1.valueKeys[numberKeys1]=LCD1_rxBuffer[3];
                        if(character==0){
                            write_LCD(1,bufferLCD1.valueKeys[numberKeys1],2,numberKeys1+2,1,0x0000,'N');
                        }else{
                            write_LCD(1,character,2,numberKeys1+2,1,0x0000,'N');
                        }
                    }else if(LCD1_rxBuffer[3]>=0x2D && LCD1_rxBuffer[3]<=0x7E){ //Comandos ascii sin incluir coma (,)
                        numberKeys1++;  
                        bufferLCD1.valueKeys[numberKeys1]=LCD1_rxBuffer[3];
                        if(character==0){
                            write_LCD(1,bufferLCD1.valueKeys[numberKeys1],2,numberKeys1+2,1,0x0000,'N');
                        }else{
                            write_LCD(1,character,2,numberKeys1+2,1,0x0000,'N');
                        }
                    }else if(LCD1_rxBuffer[3]==',' && numberKeys1>=1 && flagPoint1==0){  //Coma (,)
                        numberKeys1++;  
                        flagPoint1=1;
                        bufferLCD1.valueKeys[numberKeys1]=LCD1_rxBuffer[3];
                        if(character==0){
                            write_LCD(1,bufferLCD1.valueKeys[numberKeys1],2,numberKeys1+2,1,0x0000,'N');
                        }else{
                            write_LCD(1,character,2,numberKeys1+2,1,0x0000,'N');
                        }
                    }
                }  
                if(LCD1_rxBuffer[3]==0x0B){ //Borrar
                    if(numberKeys1==0){		//Si no ha presionado nada regresa al menu anterior
                        CyDelay(20);         
                        LCD1_ClearRxBuffer();
                        return 0;
                    }
                    else{
                        write_LCD(1,' ',2,numberKeys1+2,1,0x0000,'N');	//Si ya presiono borra el dato	
                        if(bufferLCD1.valueKeys[numberKeys1]==0x2C){
                            flagPoint1=0;
                        }
                        numberKeys1--;
                    }
                }
                if(LCD1_rxBuffer[3]==0x0A){ //Enter
                    if(numberKeys1>0){
                        bufferLCD1.valueKeys[0]=numberKeys1;
                        CyDelay(20);         
                        LCD1_ClearRxBuffer();
                        return 1;
                    }
                }
                if(LCD1_rxBuffer[3]==0x0F){ //Atras
                    CyDelay(20);         
                    LCD1_ClearRxBuffer();
                    return 0;
                }
            } 
            CyDelay(20);         
            LCD1_ClearRxBuffer();
        }
    }else{
        if(LCD2_GetRxBufferSize()==8){
            if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                countAnimation2=0;
                if(numberKeys2<=(maxNumberKeys-1)){
                    if(LCD2_rxBuffer[3]>=0x20 && LCD2_rxBuffer[3]<=0x2B){ //Comandos ascii sin incluir coma (,)
                        numberKeys2++;  
                        bufferLCD2.valueKeys[numberKeys2]=LCD2_rxBuffer[3];
                        if(character==0){
                            write_LCD(2,bufferLCD2.valueKeys[numberKeys2],2,numberKeys2+2,1,0x0000,'N');
                        }else{
                            write_LCD(2,character,2,numberKeys2+2,1,0x0000,'N');
                        }
                    }else if(LCD2_rxBuffer[3]>=0x2D && LCD2_rxBuffer[3]<=0x7E){ //Comandos ascii sin incluir coma (,)
                        numberKeys2++;  
                        bufferLCD2.valueKeys[numberKeys2]=LCD2_rxBuffer[3];
                        if(character==0){
                            write_LCD(2,bufferLCD2.valueKeys[numberKeys2],2,numberKeys2+2,1,0x0000,'N');
                        }else{
                            write_LCD(2,character,2,numberKeys2+2,1,0x0000,'N');
                        }
                    }else if(LCD2_rxBuffer[3]==',' && numberKeys2>=1 && flagPoint2==0){  //Coma (,)
                        numberKeys2++;  
                        flagPoint2=1;
                        bufferLCD2.valueKeys[numberKeys2]=LCD2_rxBuffer[3];
                        if(character==0){
                            write_LCD(2,bufferLCD2.valueKeys[numberKeys2],2,numberKeys2+2,1,0x0000,'N');
                        }else{
                            write_LCD(2,character,2,numberKeys2+2,1,0x0000,'N');
                        }
                    }
                }
                if(LCD2_rxBuffer[3]==0x0B){ //Borrar
                    if(numberKeys2==0){		//Si no ha presionado nada regresa al menu anterior
                        CyDelay(20);         
                        LCD2_ClearRxBuffer();
                        return 0;
                    }
                    else{
                        write_LCD(2,' ',2,numberKeys2+2,1,0x0000,'N');	//Si ya presiono borra el dato	
                        if(bufferLCD2.valueKeys[numberKeys2]==0x2C){
                            flagPoint2=0;
                        }
                        numberKeys2--;
                    }
                }
                if(LCD2_rxBuffer[3]==0x0A){ //Enter
                    if(numberKeys2>0){
                        bufferLCD2.valueKeys[0]=numberKeys2;
                        CyDelay(20);         
                        LCD2_ClearRxBuffer();
                        return 1;
                    }
                }
                if(LCD2_rxBuffer[3]==0x0F){ //Atras
                    CyDelay(20);         
                    LCD2_ClearRxBuffer();
                    return 0;
                }
            } 
            CyDelay(20);         
            LCD2_ClearRxBuffer();
        }
    }
    return 2;
}