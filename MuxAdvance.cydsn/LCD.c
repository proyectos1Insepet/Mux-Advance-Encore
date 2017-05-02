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
* Filename      : LCD.c
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
#include "operations.h"
#include "I2C.h"
#include "LCD.h"


/*
*********************************************************************************************************
*                                        void set_picture(uint8 lcd, uint16 picture)
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

void set_picture(uint8 lcd, uint16 picture){
    uint8 buffer[8]={0xAA,0x70,0x00,0x00,0xCC,0x33,0xC3,0x3C},x;
    buffer[2]=(picture>>8)&0xFF;
    buffer[3]=picture&0xFF;
    for(x=0;x<8;x++){
        if(lcd==1){
            LCD1_PutChar(buffer[x]);
        }else{
            LCD2_PutChar(buffer[x]);
        }
    }
}

/*
*********************************************************************************************************
*          void write_LCD(uint8_t lcd, uint8_t data, uint16_t posy, uint16_t posx,uint8_t size)
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

void write_LCD(uint8_t lcd,uint8_t data, uint16_t posy, uint16_t posx,uint8_t size, uint16_t fgcolor, uint8_t bottomless){
uint8_t buffer[18]={0xAA,0x98,0,0,0x01,0x39,0x23,0xC5,0x02,0x00,0x00,0xFF,0xFF,0,0xCC,0x33,0xC3,0x3C},x;
	buffer[2] =(0x0C*(posx*(size+1)))>>8;
	buffer[3] =(0x0C*(posx*(size+1)))&0xFF;
	buffer[4] =(0x0E*posy)>>8;
	buffer[5] =(0x0E*posy)&0xFF;
	buffer[6] =0x23+size;
    if(bottomless=='Y'){
         buffer[7]=0x85;
    }
    buffer[9] =(fgcolor&0xFF00)>>8;
    buffer[10]=(fgcolor&0x00FF);
	buffer[13]=data;
	for(x=0;x<=17;x++){
		if(lcd==1){
			LCD1_PutChar(buffer[x]);
		}
		else{
			LCD2_PutChar(buffer[x]);
		}	
	}	
}

/*
*********************************************************************************************************
*                                    void write_button(uint8 lcd, uint8 screen)
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

void write_button(uint8 lcd, uint8 screen){
    uint8 x,y,flag,mixto[8]="Mix     ";
    switch(screen){
        case 'M'://Menu
            for(x=0;x<8;x++){
                write_LCD(lcd,button[0][x],9,x+4,1,0xFFFF,'Y');     //Nombre de boton Ventas
                write_LCD(lcd,button[1][x],9,x+15,1,0xFFFF,'Y');    //Nombre de boton Turnos
                write_LCD(lcd,button[2][x],15,x+4,1,0xFFFF,'Y');    //Nombre de boton Canasta
                write_LCD(lcd,button[3][x],15,x+15,1,0xFFFF,'Y');   //Nombre de boton Consignaciones
                write_LCD(lcd,button[4][x],21,x+4,1,0xFFFF,'Y');    //Nombre de boton Mantenimiento
                write_LCD(lcd,button[5][x],21,x+15,1,0xFFFF,'Y');   //Nombre de boton Copia de Recibo
            }
        break;
            
        case 'I'://Informacion
            for(x=0;x<8;x++){
                write_LCD(lcd,button[6][x],0,x+4,0,0x2110,'Y');     //Nombre caja Version
                write_LCD(lcd,button[7][x],0,x+29,0,0x2110,'Y');    //Nombre caja Posicion
                write_LCD(lcd,button[8][x],9,x+4,0,0x2110,'Y');     //Nombre caja Hora
                write_LCD(lcd,button[9][x],9,x+29,0,0x2110,'Y');    //Nombre caja Fecha
                write_LCD(lcd,button[10][x],18,x+4,0,0x2110,'Y');   //Nombre caja Direccion IP
                write_LCD(lcd,button[11][x],18,x+29,0,0x2110,'Y');  //Nombre caja Digitos Surtidor
            }
        break;
            
        case 'S'://Venta
            for(x=0;x<8;x++){
                write_LCD(lcd,button[12][x],10,x+11,1,0xFFFF,'Y');  //Nombre de boton Contado
                write_LCD(lcd,button[13][x],16,x+11,1,0xFFFF,'Y');  //Nombre de boton Credito
                write_LCD(lcd,button[70][x],22,x+11,1,0xFFFF,'Y');  //Nombre de boton Forma de pago
                write_LCD(lcd,mixto[x],28,x+11,1,0xFFFF,'Y');   //Nombre de boton Forma de pago MIXTA
            }
        break;
            
        case 'C'://Contado
            for(x=0;x<8;x++){
                write_LCD(lcd,button[71][x],2,x+5,2,0x2110,'Y');    //Nombre de boton Preset
                write_LCD(lcd,button[14][x],9,x+4,1,0xFFFF,'Y');    //Nombre de boton Dinero
                write_LCD(lcd,button[15][x],15,x+4,1,0xFFFF,'Y');   //Nombre de boton Volumen
                write_LCD(lcd,button[16][x],21,x+4,1,0xFFFF,'Y');   //Nombre de boton Lleno
            }
            for(y=0;y<=2;y++){
                write_LCD(lcd,symbols[0],9+(6*y),15,1,0xFFFF,'Y');
                flag=0;
                for(x=1;x<=presetFast[y][0];x++){
                    if(presetFast[y][x]=='0' && flag==0){
                    }else{
                        write_LCD(lcd,presetFast[y][x],9+(6*y),x+15,1,0xFFFF,'Y');
                        flag=1;
                    }
                }
            }
        break;
            
        case '1'://Teclado Dinero
            for(x=0;x<8;x++){
                write_LCD(lcd,button[20][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Dinero Renglon 1
                write_LCD(lcd,button[21][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Dinero Renglon 2
            }
        break;
            
        case '2'://Teclado Volumen
            for(x=0;x<8;x++){
                write_LCD(lcd,button[22][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Volumen Renglon 1
                write_LCD(lcd,button[23][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Volumen Renglon 2
            }
        break;
            
        case 'P'://Escoja producto
            for(x=0;x<8;x++){
                write_LCD(lcd,button[24][x],0,x+9,2,0x2110,'Y');   //Nombre de imagen Escoja producto Renglon 1
                write_LCD(lcd,button[25][x],4,x+9,2,0x2110,'Y');   //Nombre de imagen Escoja producto Renglon 2 
                write_LCD(lcd,button[26][x],10,x+15,1,0xFFFF,'Y');   //Nombre de boton Producto 1
                write_LCD(lcd,button[27][x],16,x+15,1,0xFFFF,'Y');   //Nombre de boton Producto 2  
                write_LCD(lcd,button[28][x],22,x+15,1,0xFFFF,'Y');   //Nombre de boton Producto 3
                write_LCD(lcd,button[29][x],28,x+15,1,0xFFFF,'Y');   //Nombre de boton Producto 4
            }
        break;
            
        case 'U'://Suba la manija
            for(x=0;x<8;x++){
                write_LCD(lcd,button[30][x],18,x+5,2,0x2110,'Y');   //Nombre de imagen Suba la manija Renglon 1
                write_LCD(lcd,button[31][x],22,x+5,2,0x2110,'Y');   //Nombre de imagen Suba la manija Renglon 2
            }
        break;
            
        case 'W'://Espere 1 momento
            for(x=0;x<8;x++){
                write_LCD(lcd,button[32][x],18,x+5,2,0x2110,'Y');   //Nombre de imagen Esperando Renglon 1
                write_LCD(lcd,button[33][x],22,x+5,2,0x2110,'Y');   //Nombre de imagen Esperando Renglon 2
            }
        break;
          
        case 'E'://Error de sistema
            for(x=0;x<8;x++){
                write_LCD(lcd,button[34][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Error de sistema Renglon 1
                write_LCD(lcd,button[35][x],7,x+3,3,0x2110,'Y');   //Nombre de imagen Error de sistema Renglon 2 
            }
        break;
            
        case 'T'://Tanqueando
            for(x=0;x<8;x++){
                write_LCD(lcd,button[36][x],16,x+3,3,0x2110,'Y');   //Nombre de imagen Tanqueando Renglon 1
                write_LCD(lcd,button[37][x],22,x+3,3,0x2110,'Y');   //Nombre de imagen Tanqueando Renglon 2
                write_LCD(lcd,button[71][x],2,x+3,0,0xD000,'Y');   //Nombre de boton preset
            }
            if(lcd==1){
                switch (bufferLCD1.presetType[1]){
                    case 'D':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[14][x],2,x+12,0,0x2110,'Y');   //Nombre de boton Dinero
                        }
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case 'V':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[15][x],2,x+12,0,0x2110,'Y');   //Nombre de boton Volumen
                        }
                        write_LCD(lcd,symbols[1],4,2,1,0xD000,'Y');   //Simbolo de Volumen
                    break;
                        
                    case 'F':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[16][x],2,x+12,0,0x2110,'Y');   //Nombre de boton Lleno
                        }
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '1':
                        write_LCD(lcd,'P',2,12,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'1',2,13,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '2':
                        write_LCD(lcd,'P',2,12,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'2',2,13,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '3':
                        write_LCD(lcd,'P',2,12,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'3',2,13,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                }
                flag=0;
                for(x=1;x<=bufferLCD1.presetValue[1][0];x++){
                    if(bufferLCD1.presetValue[1][x]=='0' && flag==0){
                    }else{
                        write_LCD(lcd,bufferLCD1.presetValue[1][x],4,x+2,1,0x2110,'Y');
                        flag=1;
                    }
                }
            }else{
                switch (bufferLCD2.presetType[1]){
                    case 'D':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[14][x],2,x+12,0,0x2110,'Y');   //Nombre de boton Dinero
                        }
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case 'V':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[15][x],2,x+12,0,0x2110,'Y');   //Nombre de boton Volumen
                        }
                        write_LCD(lcd,symbols[1],4,2,1,0xD000,'Y');   //Simbolo de Volumen
                    break;
                        
                    case 'F':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[16][x],2,x+12,0,0x2110,'Y');   //Nombre de boton Lleno
                        }
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '1':
                        write_LCD(lcd,'P',2,12,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'1',2,13,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '2':
                        write_LCD(lcd,'P',2,12,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'2',2,13,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '3':
                        write_LCD(lcd,'P',2,12,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'3',2,13,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,2,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                }
                flag=0;
                for(x=1;x<=bufferLCD2.presetValue[1][0];x++){
                    if(bufferLCD2.presetValue[1][x]=='0' && flag==0){
                    }else{
                        write_LCD(lcd,bufferLCD2.presetValue[1][x],4,x+2,1,0x2110,'Y');
                        flag=1;
                    }
                }
            }
        break;
            
        case 't'://Tanqueando con datos
            for(x=0;x<8;x++){
                write_LCD(lcd,button[71][x],2,x+31,0,0xD000,'Y');   //Nombre de boton preset
                write_LCD(lcd,button[36][x],1,x+1,2,0x2110,'Y');   //Nombre de imagen Tanqueando Renglon 1
                write_LCD(lcd,button[37][x],5,x+1,2,0x2110,'Y');   //Nombre de imagen Tanqueando Renglon 2 
                write_LCD(lcd,button[39][x],12,x+3,1,0xD000,'Y');   //Nombre de imagen Teclado Placa Renglon 2
                write_LCD(lcd,button[41][x],20,x+3,1,0xD000,'Y');   //Nombre de imagen Teclado Kilometraje Renglon 2
                write_LCD(lcd,button[43][x],28,x+3,1,0xD000,'Y');   //Nombre de imagen Teclado Nit/CC Renglon 2
            }
            if(lcd==1){
                for(x=1;x<=bufferLCD1.licenceSale[0];x++){
                    write_LCD(lcd,bufferLCD1.licenceSale[x],12,x+13,1,0x0000,'N');   //Placa
                }
                for(x=1;x<=bufferLCD1.mileageSale[0];x++){
                    write_LCD(lcd,bufferLCD1.mileageSale[x],20,x+13,1,0x0000,'N');   //Kilometraje
                }
                for(x=1;x<=bufferLCD1.identySale[0];x++){
                    write_LCD(lcd,bufferLCD1.identySale[x],28,x+13,1,0x0000,'N');   //CC/NIT
                }
                switch (bufferLCD1.presetType[1]){
                    case 'D':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[14][x],2,x+40,0,0x2110,'Y');   //Nombre de boton Dinero
                        }
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case 'V':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[15][x],2,x+40,0,0x2110,'Y');   //Nombre de boton Volumen
                        }
                        write_LCD(lcd,symbols[1],4,16,1,0xD000,'Y');   //Simbolo de Volumen
                    break;
                        
                    case 'F':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[16][x],2,x+40,0,0x2110,'Y');   //Nombre de boton Lleno
                        }
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '1':
                        write_LCD(lcd,'P',2,40,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'1',2,41,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '2':
                        write_LCD(lcd,'P',2,40,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'2',2,41,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '3':
                        write_LCD(lcd,'P',2,40,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'3',2,41,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                }
                flag=0;
                for(x=1;x<=bufferLCD1.presetValue[1][0];x++){
                    if(bufferLCD1.presetValue[1][x]=='0' && flag==0){
                    }else{
                        write_LCD(lcd,bufferLCD1.presetValue[1][x],4,x+16,1,0x2110,'Y');
                        flag=1;
                    }
                }
            }else{
                for(x=1;x<=bufferLCD2.licenceSale[0];x++){
                    write_LCD(lcd,bufferLCD2.licenceSale[x],12,x+13,1,0x0000,'N');   //Placa
                }
                for(x=1;x<=bufferLCD2.mileageSale[0];x++){
                    write_LCD(lcd,bufferLCD2.mileageSale[x],20,x+13,1,0x0000,'N');   //Kilometraje
                }
                for(x=1;x<=bufferLCD2.identySale[0];x++){
                    write_LCD(lcd,bufferLCD2.identySale[x],28,x+13,1,0x0000,'N');   //CC/NIT
                }
                switch (bufferLCD2.presetType[1]){
                    case 'D':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[14][x],2,x+40,0,0x2110,'Y');   //Nombre de boton Dinero
                        }
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case 'V':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[15][x],2,x+40,0,0x2110,'Y');   //Nombre de boton Volumen
                        }
                        write_LCD(lcd,symbols[1],4,16,1,0xD000,'Y');   //Simbolo de Volumen
                    break;
                        
                    case 'F':
                        for(x=0;x<8;x++){
                            write_LCD(lcd,button[16][x],2,x+40,0,0x2110,'Y');   //Nombre de boton Lleno
                        }
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '1':
                        write_LCD(lcd,'P',2,40,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'1',2,41,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '2':
                        write_LCD(lcd,'P',2,40,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'2',2,41,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                    
                    case '3':
                        write_LCD(lcd,'P',2,40,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,'3',2,41,0,0x2110,'Y');   //Nombre de boton P1
                        write_LCD(lcd,symbols[0],4,16,1,0xD000,'Y');   //Simbolo de dinero
                    break;
                }
                flag=0;
                for(x=1;x<=bufferLCD2.presetValue[1][0];x++){
                    if(bufferLCD2.presetValue[1][x]=='0' && flag==0){
                    }else{
                        write_LCD(lcd,bufferLCD2.presetValue[1][x],4,x+16,1,0x2110,'Y');
                        flag=1;
                    }
                }
            }
        break;
            
        case '3'://Teclado Placa
            for(x=0;x<8;x++){
                write_LCD(lcd,button[38][x],7,x+6,1,0x2110,'Y');   //Nombre de imagen Teclado Placa Renglon 1
                write_LCD(lcd,button[39][x],7,x+14,1,0x2110,'Y');   //Nombre de imagen Teclado Placa Renglon 2
            }
        break;
            
        case '4'://Teclado Kilometraje
            for(x=0;x<8;x++){
                write_LCD(lcd,button[40][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Kilometraje Renglon 1
                write_LCD(lcd,button[41][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Kilometraje Renglon 2
            }
        break;
            
        case '5'://Teclado Nit/CC
            for(x=0;x<8;x++){
                write_LCD(lcd,button[42][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Nit/CC Renglon 1
                write_LCD(lcd,button[43][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Nit/CC Renglon 2
            }
        break;
        
        case '6'://Teclado N° Venta F. de pago
            for(x=0;x<8;x++){
                write_LCD(lcd,button[84][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Venta F. de pago Renglon 1
                write_LCD(lcd,button[85][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Venta F. de pago Renglon 2
            }
        break;
            
        case '7'://Teclado Dinero Venta F. de pago
            for(x=0;x<8;x++){
                write_LCD(lcd,button[86][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Venta F. de pago Renglon 1
                write_LCD(lcd,button[87][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Venta F. de pago Renglon 2
            }
        break;
        
        case '8'://Teclado Serial
            for(x=0;x<8;x++){
                write_LCD(lcd,button[38][x],7,x+6,1,0x2110,'Y');   //Nombre de imagen Teclado Serial Renglon 1
                write_LCD(lcd,button[81][x],7,x+14,1,0x2110,'Y');   //Nombre de imagen Teclado Serial Renglon 2
            }
        break;
            
        case '9'://Teclado Codigo de barras
            for(x=0;x<8;x++){
                write_LCD(lcd,button[104][x],7,x+6,1,0x2110,'Y');   //Nombre de imagen Codigo de barras Renglon 1
                write_LCD(lcd,button[105][x],7,x+14,1,0x2110,'Y');   //Nombre de imagen Codigo de barras Renglon 2
            }
        break;
            
        case '0'://Teclado cantidad canasta
            for(x=0;x<8;x++){
                write_LCD(lcd,button[104][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Venta F. de pago Renglon 1
                write_LCD(lcd,button[101][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Venta F. de pago Renglon 2
            }
        break;
            
        case '*'://Teclado contraseña
            for(x=0;x<8;x++){
                write_LCD(lcd,button[112][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Contraseña Renglon 1
                write_LCD(lcd,button[113][x],18,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Contraseña Renglon 2
            }
        break;
            
        case 'F'://Formas de pago
            for(x=0;x<8;x++){
                write_LCD(lcd,button[44][x],2,x+1,2,0x2110,'Y');   //Nombre de imagen Formas de pago Renglon 1
                write_LCD(lcd,button[45][x],2,x+9,2,0x2110,'Y');   //Nombre de imagen Formas de pago Renglon 2 
            }
        break;
            
        case 1://Nombres a Formas de pago pagina 1
            for(x=0;x<8;x++){
                write_LCD(lcd,button[46][x],9,x+4,1,0xFFFF,'Y');     //Nombre Forma de pago 1
                write_LCD(lcd,button[47][x],9,x+15,1,0xFFFF,'Y');    //Nombre Forma de pago 2
                write_LCD(lcd,button[48][x],15,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 3
                write_LCD(lcd,button[49][x],15,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 4
                write_LCD(lcd,button[50][x],21,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 5
                write_LCD(lcd,button[51][x],21,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 6
            }
        break;
            
        case 2://Nombres a Formas de pago pagina 2
            for(x=0;x<8;x++){
                write_LCD(lcd,button[52][x],9,x+4,1,0xFFFF,'Y');     //Nombre Forma de pago 7
                write_LCD(lcd,button[53][x],9,x+15,1,0xFFFF,'Y');    //Nombre Forma de pago 8
                write_LCD(lcd,button[54][x],15,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 9
                write_LCD(lcd,button[55][x],15,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 10
                write_LCD(lcd,button[56][x],21,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 11
                write_LCD(lcd,button[57][x],21,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 12
            }
        break;
            
        case 3://Nombres a Formas de pago pagina 3
            for(x=0;x<8;x++){
                write_LCD(lcd,button[58][x],9,x+4,1,0xFFFF,'Y');     //Nombre Forma de pago 13
                write_LCD(lcd,button[59][x],9,x+15,1,0xFFFF,'Y');    //Nombre Forma de pago 14
                write_LCD(lcd,button[60][x],15,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 15
                write_LCD(lcd,button[61][x],15,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 16
                write_LCD(lcd,button[62][x],21,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 17
                write_LCD(lcd,button[63][x],21,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 18
            }
        break;
            
        case 4://Nombres a Formas de pago pagina 4
            for(x=0;x<8;x++){
                write_LCD(lcd,button[64][x],9,x+4,1,0xFFFF,'Y');     //Nombre Forma de pago 19
                write_LCD(lcd,button[65][x],9,x+15,1,0xFFFF,'Y');    //Nombre Forma de pago 20
                write_LCD(lcd,button[66][x],15,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 21
                write_LCD(lcd,button[67][x],15,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 22
                write_LCD(lcd,button[68][x],21,x+4,1,0xFFFF,'Y');    //Nombre Forma de pago 23
                write_LCD(lcd,button[69][x],21,x+15,1,0xFFFF,'Y');   //Nombre Forma de pago 24
            }
        break;  
            
        case 'V'://Tipo de vehiculo
            for(x=0;x<8;x++){
                write_LCD(lcd,button[72][x],2,x+1,2,0x2110,'Y');   //Nombre de imagen Tipo de vehiculo Renglon 1
                write_LCD(lcd,button[73][x],2,x+9,2,0x2110,'Y');   //Nombre de imagen Tipo de vehiculo Renglon 2
                write_LCD(lcd,button[74][x],9,x+4,1,0xFFFF,'Y');     //Nombre Tipo de vehiculo 1
                write_LCD(lcd,button[75][x],9,x+15,1,0xFFFF,'Y');    //Nombre Tipo de vehiculo 2
                write_LCD(lcd,button[76][x],15,x+4,1,0xFFFF,'Y');    //Nombre Tipo de vehiculo 3
                write_LCD(lcd,button[77][x],15,x+15,1,0xFFFF,'Y');   //Nombre Tipo de vehiculo 4
                write_LCD(lcd,button[78][x],21,x+4,1,0xFFFF,'Y');    //Nombre Tipo de vehiculo 5
                write_LCD(lcd,button[79][x],21,x+15,1,0xFFFF,'Y');   //Nombre Tipo de vehiculo 6
            }
        break;
            
        case 'G'://Gracias
            for(x=0;x<8;x++){
                write_LCD(lcd,button[80][x],2,x+3,3,0x2110,'Y');   //Nombre de imagen Gracias Renglon 1
                write_LCD(lcd,button[82][x],8,x+1,2,0x2110,'Y');   //Nombre de imagen Gracias Renglon 2
                write_LCD(lcd,button[83][x],8,x+9,2,0x2110,'Y');   //Nombre de imagen Gracias Renglon 3
            }
        break;
            
        case 'A'://Accion Anulada
            for(x=0;x<8;x++){
                write_LCD(lcd,button[88][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Error de sistema Renglon 1
                write_LCD(lcd,button[89][x],7,x+3,3,0x2110,'Y');   //Nombre de imagen Error de sistema Renglon 2 
            }
        break;
            
        case 'm'://Metodos ID
            for(x=0;x<8;x++){
                write_LCD(lcd,button[90][x],2,x+1,2,0x2110,'Y');   //Nombre de imagen Tipo de vehiculo Renglon 1
                write_LCD(lcd,button[91][x],2,x+9,2,0x2110,'Y');   //Nombre de imagen Tipo de vehiculo Renglon 2
                write_LCD(lcd,button[92][x],9,x+4,1,0xFFFF,'Y');     //Nombre Tipo de vehiculo 1
                write_LCD(lcd,button[95][x],9,x+15,1,0xFFFF,'Y');    //Nombre Tipo de vehiculo 2
                write_LCD(lcd,button[93][x],15,x+4,1,0xFFFF,'Y');    //Nombre Tipo de vehiculo 3
                write_LCD(lcd,button[96][x],15,x+15,1,0xFFFF,'Y');   //Nombre Tipo de vehiculo 4
                write_LCD(lcd,button[94][x],21,x+4,1,0xFFFF,'Y');    //Nombre Tipo de vehiculo 5
                write_LCD(lcd,button[97][x],21,x+15,1,0xFFFF,'Y');   //Nombre Tipo de vehiculo 6
            }
        break;
            
        case 'i'://Metodo 1
            for(x=0;x<8;x++){
                write_LCD(lcd,button[98][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Esperando
                write_LCD(lcd,button[92][x],7,x+3,3,0x2110,'Y');   //Nombre de boton metodo 1 
            }
        break;
            
        case 'j'://Metodo 2
            for(x=0;x<8;x++){
                write_LCD(lcd,button[98][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Esperando
                write_LCD(lcd,button[93][x],7,x+3,3,0x2110,'Y');   //Nombre de boton metodo 2 
            }
        break;
            
        case 'k'://Metodo 3
            for(x=0;x<8;x++){
                write_LCD(lcd,button[20][x],14,x+1,2,0x2110,'Y');   //Nombre de imagen Teclado Dinero Renglon 1
                write_LCD(lcd,button[94][x],18,x+1,2,0x2110,'Y');   //Nombre de boton metodo 3
            }
        break;
            
        case 'l'://Metodo 4
            for(x=0;x<8;x++){
                write_LCD(lcd,button[38][x],7,x+6,1,0x2110,'Y');   //Nombre de imagen Teclado Placa Renglon 1
                write_LCD(lcd,button[95][x],7,x+14,1,0x2110,'Y');   //Nombre de boton metodo 3
            }
        break;
            
        case 'n'://Metodo 5
            for(x=0;x<8;x++){
                write_LCD(lcd,button[98][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Esperando
                write_LCD(lcd,button[96][x],7,x+3,3,0x2110,'Y');   //Nombre de boton metodo 5
            }
        break;
            
        case 'o'://Continuar
            for(x=0;x<8;x++){
                write_LCD(lcd,button[99][x],21,x+11,1,0xFFFF,'Y');  //Nombre de boton Continuar
            }
        break;
            
        case 'c'://Canasta
            for(x=0;x<8;x++){
                write_LCD(lcd,button[100][x],1,x+8,0,0x2110,'Y');     //Nombre caja Producto 1
                write_LCD(lcd,button[101][x],1,x+23,0,0x2110,'Y');    //Nombre caja Cantidad 1
                write_LCD(lcd,button[102][x],1,x+35,0,0x2110,'Y');     //Nombre caja Precio 1
                write_LCD(lcd,button[100][x],9,x+8,0,0x2110,'Y');     //Nombre caja Producto 2
                write_LCD(lcd,button[101][x],9,x+23,0,0x2110,'Y');    //Nombre caja Cantidad 2
                write_LCD(lcd,button[102][x],9,x+35,0,0x2110,'Y');     //Nombre caja Precio 2
                write_LCD(lcd,button[100][x],17,x+8,0,0x2110,'Y');     //Nombre caja Producto 3
                write_LCD(lcd,button[101][x],17,x+23,0,0x2110,'Y');    //Nombre caja Cantidad 3
                write_LCD(lcd,button[102][x],17,x+35,0,0x2110,'Y');     //Nombre caja Precio 3
                write_LCD(lcd,button[103][x],25,x+15,1,0x2110,'Y');    //Nombre caja Total
            }
            write_LCD(lcd,symbols[0],5,34,0,0x0000,'Y');
            write_LCD(lcd,symbols[0],13,34,0,0x0000,'Y');
            write_LCD(lcd,symbols[0],21,34,0,0x0000,'Y');
            write_LCD(lcd,symbols[0],29,13,1,0x0000,'Y');
        break;
            
        case 'u'://Mantenimiento
            for(x=0;x<8;x++){
                write_LCD(lcd,button[106][x],9,x+11,1,0xFFFF,'Y');  //Nombre de boton Calibrar
                write_LCD(lcd,button[107][x],15,x+11,1,0xFFFF,'Y');  //Nombre de boton Imprimir
                write_LCD(lcd,button[18][x],21,x+11,1,0xFFFF,'Y');  //Nombre de boton Surtidor
            }
        break;
            
        case 'p'://impresoras
            for(x=0;x<8;x++){
                write_LCD(lcd,button[108][x],6,x+1,2,0x2110,'Y');  //Nombre de boton Calibrar
                write_LCD(lcd,button[109][x],6,x+9,2,0x2110,'Y');  //Nombre de boton Imprimir
                write_LCD(lcd,button[99][x],21,x+11,1,0xFFFF,'Y');  //Nombre de boton Continuar
            }
        break;
            
        case 'y'://Abrir Turno
            for(x=0;x<8;x++){
                write_LCD(lcd,button[1][x],10,x+1,3,0x2110,'Y');    //Nombre de boton Turnos
                write_LCD(lcd,button[110][x],21,x+11,1,0xFFFF,'Y');  //Nombre de boton Abrir
            }
        break;
            
        case 'x'://Cerrar Turno
            for(x=0;x<8;x++){
                write_LCD(lcd,button[1][x],10,x+1,3,0x2110,'Y');    //Nombre de boton Turnos
                write_LCD(lcd,button[111][x],21,x+11,1,0xFFFF,'Y');  //Nombre de boton Cerrar
            }
        break;
       
        case 'a': //Error abrir Turno
            for(x=0;x<8;x++){
                write_LCD(lcd,button[110][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Error de surtidor Renglon 1
                write_LCD(lcd,button[1][x],7,x+3,3,0x2110,'Y');   //Nombre de imagen Error de surtidor Renglon 2 
            }
        break;    
            
        case 'Q'://Error de Surtidor
            for(x=0;x<8;x++){
                write_LCD(lcd,button[17][x],1,x+3,3,0x2110,'Y');   //Nombre de imagen Error de surtidor Renglon 1
                write_LCD(lcd,button[18][x],7,x+3,3,0x2110,'Y');   //Nombre de imagen Error de surtidor Renglon 2 
            }
        break;
            
        case 'K'://Corte 
            for(x=0;x<8;x++){
                write_LCD(lcd,button[19][x],15,x+5,2,0x2110,'Y');   //Nombre de imagen Corte Renglon 1
                write_LCD(lcd,button[32][x],19,x+5,2,0x2110,'Y');   //Nombre de imagen Corte Renglon 2
                write_LCD(lcd,button[33][x],21,x+5,2,0x2110,'Y');   //Nombre de imagen Corte Renglon 3
            }
        break; 
            
        case 'v'://ID Vendedor
            flag=0;
            y=5;
            write_LCD(lcd,'I',4,2,1,0x0000,'Y');
            write_LCD(lcd,'D',4,3,1,0x0000,'Y');
            for(x=1;x<=20;x++){
                if(idSeller[x]=='0' && flag==0){
                }else{
                    write_LCD(lcd,idSeller[x],4,y,1,0x0000,'Y');  //Numero de identificacion del vendedor
                    flag=1;
                    y++;
                }
            }
        break;
    }
}

/*
*********************************************************************************************************
*                                    void show_time_date(uint8 lcd)
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

void show_time_date(uint8 lcd){
    uint8 x;
    read_time();
    read_date();
    for(x=0;x<=2;x++){
        write_LCD(lcd,((time[2-x]&0xF0)>>4)+48,13,3+(3*x),1,0x0000,'N');
        write_LCD(lcd,(time[2-x]&0x0F)+48,13,4+(3*x),1,0x0000,'N');
        write_LCD(lcd,((date[2-x]&0xF0)>>4)+48,13,17+(3*x),1,0x0000,'N');
        write_LCD(lcd,(date[2-x]&0x0F)+48,13,18+(3*x),1,0x0000,'N');
    }
    write_LCD(lcd,'2',13,15,1,0x0000,'N');
    write_LCD(lcd,'0',13,16,1,0x0000,'N');
    write_LCD(lcd,':',13,5,1,0x0000,'N');
    write_LCD(lcd,':',13,8,1,0x0000,'N');
    write_LCD(lcd,'/',13,19,1,0x0000,'N');
    write_LCD(lcd,'/',13,22,1,0x0000,'N');
}

/*
*********************************************************************************************************
*                                    void show_info(uint8 lcd)
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

void show_info(uint8 lcd,uint8 *version){
    uint8 x,y=0;
    for(x=0;x<10;x++){
        write_LCD(lcd,version[x],4,2+x,1,0x0000,'Y');
    }
    if(lcd==1){
        write_LCD(lcd,(side.a.dir/10)+48,4,19,1,0x0000,'Y');
        write_LCD(lcd,(side.a.dir%10)+48,4,20,1,0x0000,'Y');
    }else{
        write_LCD(lcd,(side.b.dir/10)+48,4,19,1,0x0000,'Y');
        write_LCD(lcd,(side.b.dir%10)+48,4,20,1,0x0000,'Y');
    }
    for(x=0;x<12;x++){
        write_LCD(lcd,(ipAdress[x]&0x0f)+48,22,6+x+y,0,0x0000,'Y');
        if(x==2 || x==5 || x==8){
            write_LCD(lcd,'.',22,7+x+y,0,0x0000,'Y');
            y++;
        }
    }
    for(x=1;x<=11;x++){
        write_LCD(lcd,'|',22,(x*2)+27,0,0x0000,'Y');
    }
    write_LCD(lcd,digits+48,22,30,0,0x0000,'Y');
    write_LCD(lcd,decimalMoney+48,22,32,0,0x0000,'Y');
    write_LCD(lcd,decimalVolume+48,22,34,0,0x0000,'Y');
    write_LCD(lcd,ppux10+48,22,36,0,0x0000,'Y');
    write_LCD(lcd,productNumber+48,22,38,0,0x0000,'Y');
    write_LCD(lcd,symbols[0],22,40,0,0x0000,'Y');
    write_LCD(lcd,symbols[1],22,42,0,0x0000,'Y');
    write_LCD(lcd,screen[0],22,44,0,0x0000,'Y');
    write_LCD(lcd,screen[1],22,46,0,0x0000,'Y');
    write_LCD(lcd,turn+48,22,48,0,0x0000,'Y');
    
}

/*
*********************************************************************************************************
*                             void show_message(uint8 lcd,uint8 *msg)
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

void show_message(uint8 lcd,uint8 *msg){
    uint8 x;
    for(x=0;x<20;x++){
        write_LCD(lcd,msg[x],7,x+3,1,0x0000,'Y');
        write_LCD(lcd,msg[x+20],11,x+3,1,0x0000,'Y');
        write_LCD(lcd,msg[x+40],15,x+3,1,0x0000,'Y');
    }
}

/*
*********************************************************************************************************
*                             void show_market(uint8 lcd)
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

void show_market(uint8 lcd){
    uint8 x,y,flag;
    if(lcd==1){
        for(y=0;y<=2;y++){    
            for(x=1;x<=20;x++){
                write_LCD(lcd,bufferLCD1.productMarket[y][x],(y*8)+5,x+1,0,0x0000,'Y');
            }
        }
        flag=0;
        for(y=0;y<=2;y++){
            for(x=1;x<=3;x++){
                if(bufferLCD1.quantityMarket[y][x]=='0' && flag==0){
                }else{
                    write_LCD(lcd,bufferLCD1.quantityMarket[y][x],(y*8)+5,x+25,0,0x0000,'Y');
                    flag=1;
                }
            }
            flag=0;
        }
        for(y=0;y<=2;y++){
            multiplication_market(bufferLCD1.quantityMarket[y],bufferLCD1.priceUnitMarket[y]);
            for(x=0;x<=temporal[0];x++){
                bufferLCD1.priceTotalMarket[y][x]=temporal[x];
            }
        }
        flag=0;
        for(y=0;y<=2;y++){
            for(x=1;x<=8;x++){
                if(bufferLCD1.priceTotalMarket[y][x]=='0' && flag==0){
                }else{
                    write_LCD(lcd,bufferLCD1.priceTotalMarket[y][x],(y*8)+5,x+35,0,0x0000,'Y');
                    flag=1;
                }
            }
            flag=0;
        }
        addition_market(bufferLCD1.priceTotalMarket[0],bufferLCD1.priceTotalMarket[1],bufferLCD1.priceTotalMarket[2]);
        for(x=0;x<=temporal[0];x++){
            bufferLCD1.totalMarket[x]=temporal[x];
        }
        flag=0;
        for(x=1;x<=9;x++){
            if(bufferLCD1.totalMarket[x]=='0' && flag==0){
            }else{
                write_LCD(lcd,bufferLCD1.totalMarket[x],29,x+13,1,0x0000,'Y');
                flag=1;
            }
        }
    }else{
        for(y=0;y<=2;y++){    
            for(x=1;x<=20;x++){
                write_LCD(lcd,bufferLCD2.productMarket[y][x],(y*8)+5,x+1,0,0x0000,'Y');
            }
        }
        flag=0;
        for(y=0;y<=2;y++){
            for(x=1;x<=3;x++){
                if(bufferLCD2.quantityMarket[y][x]=='0' && flag==0){
                }else{
                    write_LCD(lcd,bufferLCD2.quantityMarket[y][x],(y*8)+5,x+25,0,0x0000,'Y');
                    flag=1;
                }
            }
            flag=0;
        }
        for(y=0;y<=2;y++){
            multiplication_market(bufferLCD2.quantityMarket[y],bufferLCD2.priceUnitMarket[y]);
            for(x=0;x<=temporal[0];x++){
                bufferLCD2.priceTotalMarket[y][x]=temporal[x];
            }
        }
        flag=0;
        for(y=0;y<=2;y++){
            for(x=1;x<=8;x++){
                if(bufferLCD2.priceTotalMarket[y][x]=='0' && flag==0){
                }else{
                    write_LCD(lcd,bufferLCD2.priceTotalMarket[y][x],(y*8)+5,x+35,0,0x0000,'Y');
                    flag=1;
                }
            }
            flag=0;
        }
        addition_market(bufferLCD2.priceTotalMarket[0],bufferLCD2.priceTotalMarket[1],bufferLCD2.priceTotalMarket[2]);
        for(x=0;x<=temporal[0];x++){
            bufferLCD2.totalMarket[x]=temporal[x];
        }
        flag=0;
        for(x=1;x<=9;x++){
            if(bufferLCD2.totalMarket[x]=='0' && flag==0){
            }else{
                write_LCD(lcd,bufferLCD2.totalMarket[x],29,x+13,1,0x0000,'Y');
                flag=1;
            }
        }
    }
}