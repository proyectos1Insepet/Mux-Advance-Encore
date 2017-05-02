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
*                                             MUX ADVANCE CODE
*
*                                             CYPRESS PSoC5LP
*                                                with the
*                                            CY8C5969AXI-LP035
*
* Filename      : main.c
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
#include <math.h>
#include <protocol.h>
#include <variables.h>
#include <ibutton.h>
#include <codebar.h>
#include <codetag.h>
#include <i2c.h>
#include <LCD.h>
#include <keyboard.h>
#include <operations.h>
#include <memoryEP.h>
CY_ISR(timerAnimation1);
CY_ISR(timerAnimation2);
CY_ISR(timerBeagleTX);
CY_ISR(timerPump);
uint8 version[10]="Ctr V 2.37";
uint8 passwordPump[6]="102030";  //Contraseña para habilitar surtidor


/*
*********************************************************************************************************
*              void show_picture(uint8 lcd,uint16 picture,uint8 seconds)
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
void show_picture(uint8 lcd,uint16 picture,uint8 seconds){
    set_picture(lcd,picture);
    if(lcd==1){
        delayPicture1=seconds*7;
        flowLCD1=0;
        countAnimation1=0;
        isr_1_StartEx(timerAnimation1); 
        Waitable_1_Start();
    }else{
        delayPicture2=seconds*7;
        flowLCD2=0;
        countAnimation2=0;
        isr_2_StartEx(timerAnimation2); 
        Waitable_2_Start();
    }
}

/*
*********************************************************************************************************
*                                         init_mux( void )
*
* Description : Verifica el serial, inicia los perosfericos, la version y los datos de la estación.
*               
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : main()
*
* Note(s)     : none.
*********************************************************************************************************
*/
void init_mux(void){
	/****Inicialización de perisfericos****/
    CyGlobalIntEnable;
    Pump_AL_EnableRxInt();
    Pump_AL_Start();
    Code_Bar_EnableRxInt();
    Code_Bar_Start();
    Beagle_EnableRxInt();
    Beagle_Start();
    Tag_EnableRxInt();
    Tag_Start();
    LCD1_Start();
    LCD2_Start();
    I2C_1_Start();
    EEPROM_1_Start();
    isr_3_StartEx(timerBeagleTX); 
    Waitable_3_Start();
    isr_4_StartEx(timerPump); 
    Waitable_4_Start();
    countBeagleTX=0;
    CyDelay(1000);

    /****Inicialización de Variables****/
    uint8 x,y;
    int z;
    for(y=0;y<=113;y++){
        for(x=0;x<8;x++){
            button[y][x]=CY_GET_REG8(CYDEV_EE_BASE + (x+(8*y)));
        }
    }
    set_picture(1,55);
    write_button(1,'W');//Espere 1 momento
    set_picture(2,55);
    write_button(2,'W');//Espere 1 momento
    digits=(CY_GET_REG8(CYDEV_EE_BASE + 944))&0x07;
    decimalMoney=((CY_GET_REG8(CYDEV_EE_BASE + 945))/10)&0x07;
    decimalVolume=((CY_GET_REG8(CYDEV_EE_BASE + 945))%10)&0x07;
    ppux10=(CY_GET_REG8(CYDEV_EE_BASE + 946))&0x01;
    productNumber=(CY_GET_REG8(CYDEV_EE_BASE + 947))&0x07;
    symbols[0]=CY_GET_REG8(CYDEV_EE_BASE + 948);
    symbols[1]=CY_GET_REG8(CYDEV_EE_BASE + 949);
    y=0;
    for(x=0;x<4;x++){
        ipAdress[0+y]=(((CY_GET_REG8(CYDEV_EE_BASE + (950+x)))/100)&0x03)+0x30;
        ipAdress[1+y]=((((CY_GET_REG8(CYDEV_EE_BASE + (950+x)))/10)%10)&0x0f)+0x30;
        ipAdress[2+y]=(((CY_GET_REG8(CYDEV_EE_BASE + (950+x)))%10)&0x0f)+0x30;
        y=y+3;
    }
    turn=(CY_GET_REG8(CYDEV_EE_BASE + 954))&0x01;
    lockTurn=0;
    typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
    idSeller[0]=20;
    for(x=1;x<=20;x++){
        idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (955+x));
    }
    z=975;
    for(y=0;y<4;y++){
        for(x=1;x<=6;x++){
            side.a.ppuAuthorized[y][x]=CY_GET_REG8(CYDEV_EE_BASE + (z+x));
        }
        side.a.ppuAuthorized[y][0]=6;
        z=z+6;
    }
    z=999;
    for(y=0;y<4;y++){
        for(x=1;x<=6;x++){
            side.b.ppuAuthorized[y][x]=CY_GET_REG8(CYDEV_EE_BASE + (z+x));
        }
        side.b.ppuAuthorized[y][0]=6;
        z=z+6;
    }
    z=911;
    for(y=0;y<3;y++){
        for(x=1;x<=8;x++){
            presetFast[y][x]=CY_GET_REG8(CYDEV_EE_BASE + (z+x));
        }
        presetFast[y][0]=8;
        z=z+8;
    }
    for(y=0;y<3;y++){
        for(x=1;x<=digits;x++){
            presetFast[y][x]=presetFast[y][x+(presetFast[y][0]-digits)];
        }
        presetFast[y][0]=digits;
    }
    screen[0]=((CY_GET_REG8(CYDEV_EE_BASE + 936))&0x01)+0x30;
    screen[1]=((CY_GET_REG8(CYDEV_EE_BASE + 937))&0x01)+0x30;
    for(x=0;x<4;x++){
        passwordSeller[x+1]=(CY_GET_REG8(CYDEV_EE_BASE + (938+x)));
    }
    bufferLCD1.stateMux[1]=0x16;
    bufferLCD2.stateMux[1]=0x16;
    bufferLCD1.stateMux[2]=0;
    bufferLCD2.stateMux[2]=0;
    bufferLCD1.flagChangePPU=0;
    bufferLCD2.flagChangePPU=0;
    flagResetMux=0;
    flowLCD1=1;
    flowLCD2=1;
    stateBeagleSoft=0;//En comunicacion
}

/*
*********************************************************************************************************
*                                         init_pump( void )
*
* Description : Obtiene posiciones y verifica estado del surtidor.
*               
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : main()
*
* Note(s)     : none.
*********************************************************************************************************
*/
void init_pump(void){
    uint8 flag=0,x;
    
    while(flag==0){
		switch(get_position()){
			case 0://No responde ninguna posicion
				set_picture(1,44);
                write_button(1,'Q');//Error de sistema
                set_picture(2,44);
                write_button(2,'Q');//Error de sistema
			break;
			
			case 1://Una de las pos no responde
				set_picture(1,44);
                write_button(1,'Q');//Error de sistema
                set_picture(2,44);
                write_button(2,'Q');//Error de sistema										
			break;
			
			case 2:
    			CyDelay(100);
                if(get_state(side.a.dir)==6){	
    				if(get_state(side.b.dir)==6){
                        for(x=0;x<productNumber;x++){
                            price_change(side.a.dir,x+1,side.a.ppuAuthorized[x]);
                            price_change(side.b.dir,x+1,side.b.ppuAuthorized[x]);
                        }
    					flag=2;							//Las dos pos contestaron
    				}else{
                        set_picture(1,44);
                        write_button(1,'Q');//Error de sistema
                        set_picture(2,44);
                        write_button(2,'Q');//Error de sistema
                    }
    			}else{
                    set_picture(1,44);
                    write_button(1,'Q');//Error de sistema
                    set_picture(2,44);
                    write_button(2,'Q');//Error de sistema
                }
			break;
		}
	}
}

/*
*********************************************************************************************************
*                                      polling_Beagle_RX( void )
*
* Description : transmite datos por serial de la posicion 1.
*               
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : main()
*
* Note(s)     : none.
*********************************************************************************************************
*/
void polling_Beagle_RX(void){
    unsigned int state1, state2;
    uint8 bufferBeagleRX[1100],bufferBeagleTX[400];
    uint32 sizeRX,sizeTX,x,y=0,z,w;
	if(Beagle_GetRxBufferSize() >= 1){
		state1=Beagle_GetRxBufferSize();
		CyDelay(50);
		state2=Beagle_GetRxBufferSize();
		if(state1==state2){
			sizeRX=0;
			while(Beagle_GetRxBufferSize()>0){
        		bufferBeagleRX[sizeRX]=Beagle_ReadRxData();
        		sizeRX++;
        	}
        	Beagle_ClearRxBuffer();
            CyWdtClear();
            bufferBeagleTX[0]='B';
            bufferBeagleTX[1]='G';
            bufferBeagleTX[2]='E';
            bufferBeagleTX[3]=side.a.dir;
            bufferBeagleTX[4]='c';
            bufferBeagleTX[5]=';';
            bufferBeagleTX[7]=';';
            bufferBeagleTX[8]='&';
            if((bufferBeagleRX[0]=='M') && (bufferBeagleRX[1]=='U') && (bufferBeagleRX[2]=='X') && (bufferBeagleRX[sizeRX-1]=='*')){
                if(bufferBeagleRX[3]==side.a.dir){
                    switch(bufferBeagleRX[4]){
                        case 0x41://Respuesta a peticion numero de venta a discriminar en forma de pago
                            if(bufferBeagleRX[6]=='1'){//Aceptado
                                bufferLCD1.moneySelectedSale[0]=8;
                                for(x=1;x<=8;x++){
                                    bufferLCD1.moneySelectedSale[x]=bufferBeagleRX[x+7];
                                }
                                bufferLCD1.keyboardWayToPay=(bufferBeagleRX[17]&0x01);
                                bufferLCD1.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD1.authorizationFlag=2;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x42://Aceptacion de forma de pago 
                            if(bufferBeagleRX[6]=='1'){//Aceptado
                                bufferLCD1.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD1.authorizationFlag=2;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x43://Respuesta a peticion identificacion de usuario para venta combustible credito
                            if(bufferBeagleRX[6]=='1'){//Aceptado
                                bufferLCD1.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD1.authorizationFlag=2;
                            }
                            bufferLCD1.volumeQuota[0]=8;
                            for(x=1;x<=8;x++){
                                bufferLCD1.volumeQuota[x]=bufferBeagleRX[x+7];
                            }
                            bufferLCD1.moneyQuota[0]=8;
                            for(x=1;x<=8;x++){
                                bufferLCD1.moneyQuota[x]=bufferBeagleRX[x+16];
                            }
                            bufferLCD1.ppuQuota[0]=6;
                            for(x=1;x<=6;x++){
                                bufferLCD1.ppuQuota[x]=bufferBeagleRX[x+25];
                            }
                            for(x=0;x<60;x++){
                                bufferLCD1.message[x]=bufferBeagleRX[x+33];
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x44:////Respuesta a peticion identificacion de producto canasta
                            if(bufferBeagleRX[6]=='1'){//Aceptado
                                bufferLCD1.authorizationFlag=1;
                                bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][3]='1';
                            }else{//No aceptado
                                bufferLCD1.authorizationFlag=2;
                                bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][3]='0';
                                bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][0]=1;
                                bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][1]=' ';
                            }
                            bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][0]=3;
                            bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][1]='0';
                            bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][2]='0';   
                            bufferLCD1.productMarket[bufferLCD1.flagProductmarket][0]=20;
                            for(x=1;x<=20;x++){
                                bufferLCD1.productMarket[bufferLCD1.flagProductmarket][x]=bufferBeagleRX[x+7];
                            }
                            bufferLCD1.priceUnitMarket[bufferLCD1.flagProductmarket][0]=8;
                            for(x=1;x<=8;x++){
                                bufferLCD1.priceUnitMarket[bufferLCD1.flagProductmarket][x]=bufferBeagleRX[x+28];
                            }
                            bufferLCD1.quantAvailableMark[bufferLCD1.flagProductmarket][0]=3;
                            bufferLCD1.quantAvailableMark[bufferLCD1.flagProductmarket][1]=bufferBeagleRX[38];
                            bufferLCD1.quantAvailableMark[bufferLCD1.flagProductmarket][2]=bufferBeagleRX[39];
                            bufferLCD1.quantAvailableMark[bufferLCD1.flagProductmarket][3]=bufferBeagleRX[40];
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x45://Respuesta a peticion Consignacion
                            if(bufferBeagleRX[6]=='1'){//Aceptado
                                bufferLCD1.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD1.authorizationFlag=2;
                            }
                            for(x=0;x<60;x++){
                                bufferLCD1.message[x]=bufferBeagleRX[x+8];
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break; 
                        
                        case 0x46://Respuesta a peticion Turno
                            if(bufferBeagleRX[6]=='1'){//Aceptado
                                bufferLCD1.authorizationFlag=1;
                                turn=(!turn);
                                write_settings();
                            }else{//No aceptado
                                for(x=1;x<=20;x++){
                                    idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (x+955));
                                }
                                idSeller[0]=20;
                                typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
                                bufferLCD1.authorizationFlag=2;
                            }
                            for(x=0;x<60;x++){
                                bufferLCD1.message[x]=bufferBeagleRX[x+8];
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;     
                        
                        case 0x47://Peticion de totales
                            if(bufferBeagleRX[6]=='1'){//Abrir Turno
                                turn=1;
                            }else{//Cerrar Turno
                                turn=0;
//                                flowLCD1=1;
//                                flowLCD2=1;
                            }
                            write_settings();
                            if(get_totals(side.a.dir)==0 || get_totals(side.b.dir)==0){
                                for(x=0;x<4;x++){
                                    for(y=0;y<3;y++){
                                        for(z=0;z<14;z++){
                                            side.a.totalsHandle[x][y][z]=0;
                                            side.b.totalsHandle[x][y][z]=0;
                                        }
                                    }
                                }
                            }
		                    CyWdtClear();
                            for(w=0;w<=3;w++){
                                bufferBeagleTX[5+(w*35)]=';';
                                y=side.a.totalsHandle[w][0][0];
                                if(y==0){
                                    y=1;
                                    side.a.totalsHandle[w][0][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[5+x+(w*35)]=side.a.totalsHandle[w][0][y];
                                    y--;
                                    z++;
                                    if(z==(decimalVolume-1)){
                                        x--;
                                        bufferBeagleTX[5+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[5+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[19+(w*35)]=';';
                                y=side.a.totalsHandle[w][1][0];
                                if(y==0){
                                    y=1;
                                    side.a.totalsHandle[w][1][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[19+x+(w*35)]=side.a.totalsHandle[w][1][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[19+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[19+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[33+(w*35)]=';';
                                y=side.a.totalsHandle[w][2][0];
                                if(y==0){
                                    y=1;
                                    side.a.totalsHandle[w][2][1]='0';
                                }
                                z=0;
                                for(x=6;x>0;x--){
                                    bufferBeagleTX[33+x+(w*35)]=side.a.totalsHandle[w][2][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[33+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[33+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                            }
                            for(w=0;w<=3;w++){
                                bufferBeagleTX[145+(w*35)]=';';
                                y=side.b.totalsHandle[w][0][0];
                                if(y==0){
                                    y=1;
                                    side.b.totalsHandle[w][0][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[145+x+(w*35)]=side.b.totalsHandle[w][0][y];
                                    y--;
                                    z++;
                                    if(z==(decimalVolume-1)){
                                        x--;
                                        bufferBeagleTX[145+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[145+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[159+(w*35)]=';';
                                y=side.b.totalsHandle[w][1][0];
                                if(y==0){
                                    y=1;
                                    side.b.totalsHandle[w][1][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[159+x+(w*35)]=side.b.totalsHandle[w][1][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[159+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[159+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[173+(w*35)]=';';
                                y=side.b.totalsHandle[w][2][0];
                                if(y==0){
                                    y=1;
                                    side.b.totalsHandle[w][2][1]='0';
                                }
                                z=0;
                                for(x=6;x>0;x--){
                                    bufferBeagleTX[173+x+(w*35)]=side.b.totalsHandle[w][2][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[173+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[173+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                            }
                            bufferBeagleTX[4]='l';
                            sizeTX=284;
                        break;
                        
                        case 0x48://Configuraciones iniciales
                            date[2]=(bufferBeagleRX[8]&0x0F)<<4;	//Decenas de año
                        	date[2]|=(bufferBeagleRX[9]&0x0F);		//Unidades de año
                        	date[1]=(bufferBeagleRX[10]&0x0F)<<4;	//Decenas de mes
                        	date[1]|=(bufferBeagleRX[11]&0x0F);		//Unidades de mes
                        	date[0]=(bufferBeagleRX[12]&0x0F)<<4; 	//Decenas de dia
                        	date[0]|=(bufferBeagleRX[13]&0x0F);		//Unidades de dia
                        	write_date();
                        	time[2]=(bufferBeagleRX[14]&0x0F)<<4;	//Decenas de hora
                        	time[2]|=(bufferBeagleRX[15]&0x0F);		//Unidades de Hora
                        	time[1]=(bufferBeagleRX[16]&0x0F)<<4;	//Decenas de Minutos
                        	time[1]|=(bufferBeagleRX[17]&0x0F);		//Unidades de Minutos
                            time[0]=(bufferBeagleRX[18]&0x0F)<<4;	//Decenas de Segundos
                        	time[0]|=(bufferBeagleRX[19]&0x0F);		//Unidades de Segundos
                            write_time();
                            digits=bufferBeagleRX[21]&0x07;
                            decimalMoney=bufferBeagleRX[23]&0x07;
                            decimalVolume=bufferBeagleRX[25]&0x07;
                            ppux10=bufferBeagleRX[27]&0x01;
                            productNumber=bufferBeagleRX[29]&0x07;
                            symbols[0]=bufferBeagleRX[31];
                            symbols[1]=bufferBeagleRX[33];
                            for(x=0;x<12;x++){
                                ipAdress[x]=bufferBeagleRX[35+x];
                            }
                            w=47;
                            for(z=0;z<3;z++){
                                for(x=1;x<=8;x++){
                                    presetFast[z][x]=bufferBeagleRX[w+x];
                                }
                                presetFast[z][0]=8;
                                w=w+9;
                            }
                            screen[0]=(bufferBeagleRX[75]&0x01)+0x30;
                            screen[1]=(bufferBeagleRX[77]&0x01)+0x30;
                            write_settings();
                            for(z=0;z<3;z++){
                                for(x=1;x<=digits;x++){
                                    presetFast[z][x]=presetFast[z][x+(presetFast[z][0]-digits)];
                                }
                                presetFast[z][0]=digits;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                        
                        case 0x49://PPUs Autorizados
                            z=5;
                            for(w=0;w<4;w++){
                                for(x=1;x<=6;x++){
                                    side.a.ppuAuthorized[w][x]=bufferBeagleRX[x+z];
                                }
                                side.a.ppuAuthorized[w][0]=6;
                                z=z+7;
                            }
                            bufferLCD1.flagChangePPU=1;
                            write_ppus();
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                        
                        case 0x4A://Nombre de Botones
                            w=6;
                            for(x=0;x<=113;x++){
                                for(z=0;z<8;z++){
                                    button[x][z]=bufferBeagleRX[w];
                                    w++;
                                }
                                w++;
                            }
                            write_buttons();
		                    CyWdtClear();
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                        
                        case 0x4B://Resetear MUX para programar
                            if(bufferBeagleRX[6]=='1'){//Reset Modulo
                                flagResetMux=1;
                            }else{
                                flagResetMux=1;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x4C://Programar venta Autoservicio
                            bufferLCD1.presetType[1]=bufferBeagleRX[6];
                            if(bufferLCD1.presetType[1]=='V'){
                                bufferLCD1.presetType[0]=1;
                            }else{
                                bufferLCD1.presetType[0]=2;
                            }
                            z=15;
                            for(x=digits;x>0;x--){
                                bufferLCD1.presetValue[0][x]=bufferBeagleRX[z];
                                bufferLCD1.presetValue[1][x]=bufferBeagleRX[z];
                                z--;
                            }
                            bufferLCD1.presetValue[0][0]=digits;
                            bufferLCD1.presetValue[1][0]=digits;
                            bufferLCD1.productType=bufferBeagleRX[17]&0x07;
                            for(x=1;x<=6;x++){
                                bufferLCD1.ppuQuota[x]=bufferBeagleRX[x+18];
                            }
                            bufferLCD1.ppuQuota[0]=6;
                            isr_1_Stop(); 
                            Waitable_1_Stop();
                            bufferLCD1.salePerform='1';
                            bufferLCD1.saleType='4';
                            bufferLCD1.vehicleType=0;
                            bufferLCD1.licenceSale[0]=0;
                            bufferLCD1.mileageSale[0]=0;
                            bufferLCD1.identySale[0]=0;
                            bufferLCD1.idType=0;
                            bufferLCD1.idSerial[0]=0;
                            set_picture(1,43);
                            write_button(1,'U');    //Suba la manija
                            bufferLCD1.stateMux[1]=0x2B;//Espera de subir la manija en venta autoservicio
                            flowLCD1=28; 
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x4D://Bloquear/Desploquear por corte
                            if(bufferBeagleRX[6]=='1'){//Bloquear
                                bufferLCD1.stateMux[1]=0x38;
                                set_picture(1,65);
                                write_button(1,'K');//corte
                                isr_1_StartEx(timerAnimation1); 
                                Waitable_1_Start();
                                countAnimation1=0;
                                flowLCD1=44;
                            }else{//Desloquear
                                bufferLCD1.stateMux[1]=0x16;
                                flowLCD1=1;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x4E://No Ejecutar
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x4F://Datos Vendedor
                            typeIdSeller=bufferBeagleRX[6];
                            idSeller[0]=20;
                            for(x=1;x<=20;x++){
                                idSeller[x]=bufferBeagleRX[x+7];
                            }
                            passwordSeller[0]=4;
                            for(x=1;x<=4;x++){
                                passwordSeller[x]=bufferBeagleRX[x+28];
                            }
                            write_settings();
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x50://Estado de comunicacion NSX
                            if(bufferBeagleRX[6]=='1'){//En comunicacion
                                stateBeagleSoft=1;
                            }else{//Sin comunicacion
                                stateBeagleSoft=0;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x59://Reset Estado
                            switch(bufferLCD1.stateMux[1]){
                                case 0x19://Surtiendo
                                    bufferLCD1.flagLiftHandle=1;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                
                                case 0x1A://Fin Venta Combustible
                                    bufferLCD1.stateMux[1]=0x16;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                
                                case 0x21://Peticion Venta Seleccionada Forma de Pago
                                    bufferLCD1.stateMux[1]=0x22;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                
                                case 0x24://Fin Forma de Pago
                                    bufferLCD1.stateMux[1]=0x25;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                 
                                case 0x26://Peticion Identificacion usuario credito
                                    bufferLCD1.stateMux[1]=0x27;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break; 
                                    
                                case 0x29://Peticio Calibracion
                                    bufferLCD1.stateMux[1]=0x2A;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                case 0x31://Peticion Identificacion producto canasta
                                    bufferLCD1.stateMux[1]=0x32;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                case 0x33://Fin Venta Canasta
                                    bufferLCD1.stateMux[0]=0;
                                    bufferLCD1.stateMux[1]=0x16;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;  
                                
                                case 0x35://Peticion Consignacion
                                    bufferLCD1.stateMux[1]=0x36;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;    
                                
                                case 0x41://Peticion Turno
                                    bufferLCD1.stateMux[1]=0x42;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                default:
                                    bufferLCD1.stateMux[1]=0x16;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                            }
                        break;
                    }
		            CyWdtClear();
                    for(x=0;x<=sizeTX;x++){
                        Beagle_PutChar(bufferBeagleTX[x]);
                    }
                    Beagle_PutChar(';');
                    Beagle_PutChar('&');
                }else{
                    bufferBeagleTX[6]='0';
                    for(x=0;x<=8;x++){
                        Beagle_PutChar(bufferBeagleTX[x]);
                    }
                }
                for(y=0;y<=sizeRX-1;y++){
                    if(bufferBeagleRX[y]=='&'){
                        y++;
                        break;
                    }
                }
                bufferBeagleTX[4]='c';
                bufferBeagleTX[5]=';';
                if(bufferBeagleRX[y]==side.b.dir){
                    y++;
                    switch(bufferBeagleRX[y]){
                        case 0x41://Respuesta a peticion numero de venta a discriminar en forma de pago
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Aceptado
                                y++;
                                bufferLCD2.moneySelectedSale[0]=8;
                                for(x=1;x<=8;x++){
                                    bufferLCD2.moneySelectedSale[x]=bufferBeagleRX[x+y];
                                }
                                y=y+10;
                                bufferLCD2.keyboardWayToPay=(bufferBeagleRX[y]&0x01);
                                bufferLCD2.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD2.authorizationFlag=2;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x42://Aceptacion de forma de pago 
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Aceptado
                                bufferLCD2.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD2.authorizationFlag=2;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x43://Respuesta a peticion identificacion de usuario para venta combustible credito
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Aceptado
                                bufferLCD2.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD2.authorizationFlag=2;
                            }
                            y++;
                            bufferLCD2.volumeQuota[0]=8;
                            for(x=1;x<=8;x++){
                                bufferLCD2.volumeQuota[x]=bufferBeagleRX[x+y];
                            }
                            y=y+9;
                            bufferLCD2.moneyQuota[0]=8;
                            for(x=1;x<=8;x++){
                                bufferLCD2.moneyQuota[x]=bufferBeagleRX[x+y];
                            }
                            y=y+9;
                            bufferLCD2.ppuQuota[0]=6;
                            for(x=1;x<=6;x++){
                                bufferLCD2.ppuQuota[x]=bufferBeagleRX[x+y];
                            }
                            y=y+8;
                            for(x=0;x<60;x++){
                                bufferLCD2.message[x]=bufferBeagleRX[x+y];
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x44://Respuesta a peticion identificacion de producto canasta
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Aceptado
                                bufferLCD2.authorizationFlag=1;
                            bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][3]='1';
                            }else{//No aceptado
                                bufferLCD2.authorizationFlag=2;
                                bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][3]='0';
                                bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][0]=1;
                                bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][1]=' ';
                            }
                            bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][0]=3;
                            bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][1]='0';
                            bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][2]='0';
                            y++;
                            bufferLCD2.productMarket[bufferLCD2.flagProductmarket][0]=20;
                            for(x=1;x<=20;x++){
                                bufferLCD2.productMarket[bufferLCD2.flagProductmarket][x]=bufferBeagleRX[x+y];
                            }
                            y=y+21;
                            bufferLCD2.priceUnitMarket[bufferLCD2.flagProductmarket][0]=8;
                            for(x=1;x<=8;x++){
                                bufferLCD2.priceUnitMarket[bufferLCD2.flagProductmarket][x]=bufferBeagleRX[x+y];
                            }
                            y=y+10;
                            bufferLCD2.quantAvailableMark[bufferLCD2.flagProductmarket][0]=3;
                            bufferLCD2.quantAvailableMark[bufferLCD2.flagProductmarket][1]=bufferBeagleRX[y];
                            bufferLCD2.quantAvailableMark[bufferLCD2.flagProductmarket][2]=bufferBeagleRX[y+1];
                            bufferLCD2.quantAvailableMark[bufferLCD2.flagProductmarket][3]=bufferBeagleRX[y+2];
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                        
                        case 0x45://Respuesta a peticion Consignacion
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Aceptado
                                bufferLCD2.authorizationFlag=1;
                            }else{//No aceptado
                                bufferLCD2.authorizationFlag=2;
                            }
                            y=y+2;
                            for(x=0;x<60;x++){
                                bufferLCD2.message[x]=bufferBeagleRX[x+y];
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x46://Respuesta a peticion Turno
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Aceptado
                                bufferLCD2.authorizationFlag=1;
                                turn=(!turn);
                                write_settings();
                            }else{//No aceptado
                                for(x=1;x<=20;x++){
                                    idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (x+955));
                                }
                                idSeller[0]=20;
                                typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
                                bufferLCD2.authorizationFlag=2;
                            }
                            y=y+2;
                            for(x=0;x<60;x++){
                                bufferLCD2.message[x]=bufferBeagleRX[x+y];
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x47://Peticion de totales
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Abrir Turno
                                turn=1;
                            }else{//Cerrar Turno
                                turn=0;
//                                flowLCD1=1;
//                                flowLCD2=1;
                            }
                            write_settings();
                            if(get_totals(side.a.dir)==0 || get_totals(side.b.dir)==0){
                                for(x=0;x<4;x++){
                                    for(y=0;y<3;y++){
                                        for(z=0;z<14;z++){
                                            side.a.totalsHandle[x][y][z]=0;
                                            side.b.totalsHandle[x][y][z]=0;
                                        }
                                    }
                                }
                            }
		                    CyWdtClear();
                            for(w=0;w<=3;w++){
                                bufferBeagleTX[5+(w*35)]=';';
                                y=side.a.totalsHandle[w][0][0];
                                if(y==0){
                                    y=1;
                                    side.a.totalsHandle[w][0][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[5+x+(w*35)]=side.a.totalsHandle[w][0][y];
                                    y--;
                                    z++;
                                    if(z==(decimalVolume-1)){
                                        x--;
                                        bufferBeagleTX[5+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[5+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[19+(w*35)]=';';
                                y=side.a.totalsHandle[w][1][0];
                                if(y==0){
                                    y=1;
                                    side.a.totalsHandle[w][1][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[19+x+(w*35)]=side.a.totalsHandle[w][1][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[19+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[19+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[33+(w*35)]=';';
                                y=side.a.totalsHandle[w][2][0];
                                if(y==0){
                                    y=1;
                                    side.a.totalsHandle[w][2][1]='0';
                                }
                                z=0;
                                for(x=6;x>0;x--){
                                    bufferBeagleTX[33+x+(w*35)]=side.a.totalsHandle[w][2][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[33+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[33+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                            }
                            for(w=0;w<=3;w++){
                                bufferBeagleTX[145+(w*35)]=';';
                                y=side.b.totalsHandle[w][0][0];
                                if(y==0){
                                    y=1;
                                    side.b.totalsHandle[w][0][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[145+x+(w*35)]=side.b.totalsHandle[w][0][y];
                                    y--;
                                    z++;
                                    if(z==(decimalVolume-1)){
                                        x--;
                                        bufferBeagleTX[145+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[145+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[159+(w*35)]=';';
                                y=side.b.totalsHandle[w][1][0];
                                if(y==0){
                                    y=1;
                                    side.b.totalsHandle[w][1][1]='0';
                                }
                                z=0;
                                for(x=13;x>0;x--){
                                    bufferBeagleTX[159+x+(w*35)]=side.b.totalsHandle[w][1][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[159+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[159+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                                bufferBeagleTX[173+(w*35)]=';';
                                y=side.b.totalsHandle[w][2][0];
                                if(y==0){
                                    y=1;
                                    side.b.totalsHandle[w][2][1]='0';
                                }
                                z=0;
                                for(x=6;x>0;x--){
                                    bufferBeagleTX[173+x+(w*35)]=side.b.totalsHandle[w][2][y];
                                    y--;
                                    z++;
                                    if(z==decimalMoney){
                                        x--;
                                        bufferBeagleTX[173+x+(w*35)]=',';
                                    }
                                    if(y<=0){
                                        while(x>1){
                                            x--;
                                            bufferBeagleTX[173+x+(w*35)]='0';
                                        }
                                        break;
                                    }
                                }
                            }
                            bufferBeagleTX[4]='l';
                            sizeTX=284;
                        break;   
                            
                        case 0x48://Configuraciones iniciales
                            y=y+2;
                            date[2]=(bufferBeagleRX[y+2]&0x0F)<<4;	//Decenas de año
                        	date[2]|=(bufferBeagleRX[y+3]&0x0F);		//Unidades de año
                        	date[1]=(bufferBeagleRX[y+4]&0x0F)<<4;	//Decenas de mes
                        	date[1]|=(bufferBeagleRX[y+5]&0x0F);		//Unidades de mes
                        	date[0]=(bufferBeagleRX[y+6]&0x0F)<<4; 	//Decenas de dia
                        	date[0]|=(bufferBeagleRX[y+7]&0x0F);		//Unidades de dia
                        	write_date();
                        	time[2]=(bufferBeagleRX[y+8]&0x0F)<<4;	//Decenas de hora
                        	time[2]|=(bufferBeagleRX[y+9]&0x0F);		//Unidades de Hora
                        	time[1]=(bufferBeagleRX[y+10]&0x0F)<<4;	//Decenas de Minutos
                        	time[1]|=(bufferBeagleRX[y+11]&0x0F);		//Unidades de Minutos
                            time[0]=(bufferBeagleRX[y+12]&0x0F)<<4;	//Decenas de Segundos
                        	time[0]|=(bufferBeagleRX[y+13]&0x0F);		//Unidades de Segundos
                            write_time();
                            y=y+15;
                            digits=bufferBeagleRX[y]&0x07;
                            y=y+2;
                            decimalMoney=bufferBeagleRX[y]&0x07;
                            y=y+2;
                            decimalVolume=bufferBeagleRX[y]&0x07;
                            y=y+2;
                            ppux10=bufferBeagleRX[y]&0x01;
                            y=y+2;
                            productNumber=bufferBeagleRX[y]&0x07;
                            y=y+2;
                            symbols[0]=bufferBeagleRX[y];
                            y=y+2;
                            symbols[1]=bufferBeagleRX[y];
                            y=y+2;
                            for(x=0;x<12;x++){
                                ipAdress[x]=bufferBeagleRX[y+x];
                            }
                            y=y+12;
                            for(z=0;z<3;z++){
                                for(x=1;x<=8;x++){
                                    presetFast[z][x]=bufferBeagleRX[y+x];
                                }
                                presetFast[z][0]=8;
                                y=y+9;
                            }
                            screen[0]=(bufferBeagleRX[y+1]&0x01)+0x30;
                            screen[1]=(bufferBeagleRX[y+3]&0x01)+0x30;
                            write_settings();
                            for(z=0;z<3;z++){
                                for(x=1;x<=digits;x++){
                                    presetFast[z][x]=presetFast[z][x+(presetFast[z][0]-digits)];
                                }
                                presetFast[z][0]=digits;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break; 
                        
                        case 0x49://PPUs Autorizados
                            y++;
                            for(w=0;w<4;w++){
                                for(x=1;x<=6;x++){
                                    side.b.ppuAuthorized[w][x]=bufferBeagleRX[x+y];
                                }
                                side.b.ppuAuthorized[w][0]=6;
                                y=y+7;
                            }
                            bufferLCD2.flagChangePPU=1;
                            write_ppus();
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                        
                        case 0x4A://Nombre de Botones
                            y=y+2;
                            for(x=0;x<=113;x++){
                                for(z=0;z<8;z++){
                                    button[x][z]=bufferBeagleRX[y];
                                    y++;
                                }
                                y++;
                            }
                            write_buttons();
		                    CyWdtClear();
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x4B://Resetear MUX para programar
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Reset Modulo
                                flagResetMux=1;
                            }else{
                                flagResetMux=0;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break; 
                        
                        case 0x4C://Programar venta Autoservicio
                            y=y+2;
                            bufferLCD2.presetType[1]=bufferBeagleRX[y];
                            if(bufferLCD2.presetType[1]=='V'){
                                bufferLCD2.presetType[0]=1;
                            }else{
                                bufferLCD2.presetType[0]=2;
                            }
                            y=y+9;
                            z=y;
                            for(x=digits;x>0;x--){
                                bufferLCD2.presetValue[0][x]=bufferBeagleRX[z];
                                bufferLCD2.presetValue[1][x]=bufferBeagleRX[z];
                                z--;
                            }
                            bufferLCD2.presetValue[0][0]=digits;
                            bufferLCD2.presetValue[1][0]=digits;
                            y=y+2;
                            bufferLCD2.productType=bufferBeagleRX[y]&0x07;
                            y++;
                            for(x=1;x<=6;x++){
                                bufferLCD2.ppuQuota[x]=bufferBeagleRX[x+y];
                            }
                            bufferLCD2.ppuQuota[0]=6;
                            isr_2_Stop(); 
                            Waitable_2_Stop();
                            bufferLCD2.salePerform='1';
                            bufferLCD2.saleType='4';
                            bufferLCD2.vehicleType=0;
                            bufferLCD2.licenceSale[0]=0;
                            bufferLCD2.mileageSale[0]=0;
                            bufferLCD2.identySale[0]=0;
                            bufferLCD2.idType=0;
                            bufferLCD2.idSerial[0]=0;
                            set_picture(2,43);
                            write_button(2,'U');    //Suba la manija
                            bufferLCD2.stateMux[1]=0x2B;//Espera de subir la manija en venta autoservicio
                            flowLCD2=28; 
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                        
                        case 0x4D://Bloquear/Desploquear por corte
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//Bloquear
                                bufferLCD2.stateMux[1]=0x38;
                                set_picture(2,65);
                                write_button(2,'K');//Corte
                                isr_2_StartEx(timerAnimation2); 
                                Waitable_2_Start();
                                countAnimation2=0;
                                flowLCD2=44;
                            }else{//Desloquear
                                bufferLCD2.stateMux[1]=0x16;
                                flowLCD2=1;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;    
                            
                        case 0x4E://No Ejecutar
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                            
                        case 0x4F://Datos Vendedor
                            y=y+2;
                            typeIdSeller=bufferBeagleRX[y];
                            y++;
                            idSeller[0]=20;
                            for(x=1;x<=20;x++){
                                idSeller[x]=bufferBeagleRX[x+y];
                            }
                            y=y+21;
                            passwordSeller[0]=4;
                            for(x=1;x<=4;x++){
                                passwordSeller[x]=bufferBeagleRX[x+y];
                            }
                            write_settings();
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x50://Estado de comunicacion NSX
                            y=y+2;
                            if(bufferBeagleRX[y]=='1'){//En comunicacion
                                stateBeagleSoft=1;
                            }else{//Sin comunicacion
                                stateBeagleSoft=0;
                            }
                            bufferBeagleTX[6]='1';
                            sizeTX=6;
                        break;
                        
                        case 0x59://Reset Estado
                            switch(bufferLCD2.stateMux[1]){
                                case 0x19://Surtiendo
                                    bufferLCD2.flagLiftHandle=1;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                
                                case 0x1A://Fin Venta
                                    bufferLCD2.stateMux[1]=0x16;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                
                                case 0x21://Venta Seleccionada Forma de Pago
                                    bufferLCD2.stateMux[1]=0x22;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                case 0x24://Fin Forma de Pago
                                    bufferLCD2.stateMux[1]=0x25;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;  
                                    
                                case 0x26://Peticion Identificacion usuario credito
                                    bufferLCD2.stateMux[1]=0x27;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break; 
                                    
                                case 0x29://Peticio Calibracion
                                    bufferLCD2.stateMux[1]=0x2A;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break; 
                                    
                                case 0x31://Peticion Identificacion producto canasta
                                    bufferLCD2.stateMux[1]=0x32;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                case 0x33://Fin Venta Canasta
                                    bufferLCD2.stateMux[0]=0;
                                    bufferLCD2.stateMux[1]=0x16;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;  
                                    
                                case 0x35://Peticion Consignacion
                                    bufferLCD2.stateMux[1]=0x36;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                case 0x41://Peticion Turno
                                    bufferLCD2.stateMux[1]=0x42;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                                    
                                default:
                                    bufferLCD2.stateMux[1]=0x16;
                                    bufferBeagleTX[6]='1';
                                    sizeTX=6;
                                break;
                            }
                        break;
                    }
                    bufferBeagleTX[3]=side.b.dir;
		            CyWdtClear();
                    for(x=3;x<=sizeTX;x++){
                        Beagle_PutChar(bufferBeagleTX[x]);
                    }
                    Beagle_PutChar(';');
                    Beagle_PutChar('&');
                }else{
                    bufferBeagleTX[3]=side.b.dir;
                    bufferBeagleTX[6]='0';
                    for(x=3;x<=8;x++){
                        Beagle_PutChar(bufferBeagleTX[x]);
                    }
                }
            }else{
                bufferBeagleTX[6]='0';
                for(x=0;x<=8;x++){
                    Beagle_PutChar(bufferBeagleTX[x]);
                }
                bufferBeagleTX[3]=side.b.dir;
                for(x=3;x<=8;x++){
                    Beagle_PutChar(bufferBeagleTX[x]);
                }
            }
            Beagle_PutChar('*');
            if(bufferLCD1.flagChangePPU==1){
                for(x=0;x<productNumber;x++){
                    price_change(side.a.dir,x+1,side.a.ppuAuthorized[x]);
                }
                bufferLCD1.flagChangePPU=0;
		        CyWdtClear();
            }
            if(bufferLCD2.flagChangePPU==1){
                for(x=0;x<productNumber;x++){
                    price_change(side.b.dir,x+1,side.b.ppuAuthorized[x]);
                }
                bufferLCD2.flagChangePPU=0;
		        CyWdtClear();
            }
            if(flagResetMux==1){
                flowLCD1=3;
                flowLCD2=3;
                set_picture(1,55);
                write_button(1,'W');//Espere 1 momento
                set_picture(2,55);
                write_button(2,'W');//Espere 1 momento
                while(1);
            }
            CyDelay(50);
        }
    }
}

/*
*********************************************************************************************************
*                                      polling_beagle_TX( void )
*
* Description : transmite datos por serial de la posicion 1.
*               
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : main()
*
* Note(s)     : none.
*********************************************************************************************************
*/
void polling_beagle_TX(void){
    uint8 bufferBeagleTX[400];
    uint16 size=0,x,z,w;
    int y;
    if(countBeagleTX>50){
        bufferBeagleTX[0]='B';
        bufferBeagleTX[1]='G';
        bufferBeagleTX[2]='E';
        bufferBeagleTX[3]=side.a.dir;
        switch(bufferLCD1.stateMux[1]){
            case 0x19://Surtiendo
                if(bufferLCD1.flagLiftHandle==0){
                    bufferBeagleTX[4]='m';
                    bufferBeagleTX[5]=';';
                    bufferBeagleTX[6]=bufferLCD1.presetType[1];
                    bufferBeagleTX[7]=';';
                    y=bufferLCD1.presetValue[1][0];
                    for(x=8;x>0;x--){
                        bufferBeagleTX[7+x]=bufferLCD1.presetValue[1][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[7+x]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[16]=';';
                    bufferBeagleTX[17]=bufferLCD1.productType+0x30;
                    bufferBeagleTX[18]=';';
                    y=bufferLCD1.totalVolumePrevious[0];
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[18+x]=bufferLCD1.totalVolumePrevious[y];
                        y--;
                        z++;
                        if(z==(decimalVolume-1)){
                            x--;
                            bufferBeagleTX[18+x]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[18+x]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[32]=';';
                    y=bufferLCD1.totalMoneyPrevious[0];
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[32+x]=bufferLCD1.totalMoneyPrevious[y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[32+x]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[32+x]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[46]=';';
                    y=bufferLCD1.totalPPUPrevious[0];
                    z=0;
                    for(x=6;x>0;x--){
                        bufferBeagleTX[46+x]=bufferLCD1.totalPPUPrevious[y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[46+x]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[46+x]='0';
                            }
                            break;
                        }
                    }
                    size=52;
                }else{
                    bufferBeagleTX[4]='a';
                    bufferBeagleTX[5]=';';
                    bufferBeagleTX[6]=0x19;//Surtiendo
                    size=6;
                }
            break;
                
            case 0x1A://Fin Venta
                bufferBeagleTX[4]='b';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=bufferLCD1.saleType;
                bufferBeagleTX[7]=';';
                bufferBeagleTX[8]=bufferLCD1.presetType[1];
                bufferBeagleTX[9]=';';
                y=bufferLCD1.presetValue[1][0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[9+x]=bufferLCD1.presetValue[1][y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[9+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[18]=';';
                bufferBeagleTX[19]=bufferLCD1.productType+0x30;
                bufferBeagleTX[20]=';';
                bufferBeagleTX[21]='2';
                bufferBeagleTX[22]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[23+x]=bufferLCD1.dateLiftHandle[x];
                    bufferBeagleTX[29+x]=bufferLCD1.timeLiftHandle[x];
                }
                bufferBeagleTX[35]=';';
                y=bufferLCD1.totalVolumePrevious[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[35+x]=bufferLCD1.totalVolumePrevious[y];
                    y--;
                    z++;
                    if(z==(decimalVolume-1)){
                        x--;
                        bufferBeagleTX[35+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[35+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[49]=';';
                y=bufferLCD1.totalMoneyPrevious[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[49+x]=bufferLCD1.totalMoneyPrevious[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[49+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[49+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[63]=';';
                y=bufferLCD1.totalPPUPrevious[0];
                z=0;
                for(x=6;x>0;x--){
                    bufferBeagleTX[63+x]=bufferLCD1.totalPPUPrevious[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[63+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[63+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[70]=';';
                y=side.a.volumeSale[0];
                z=0;
                for(x=8;x>0;x--){
                    bufferBeagleTX[70+x]=side.a.volumeSale[y];
                    y--;
                    z++;
                    if(z==decimalVolume){
                        x--;
                        bufferBeagleTX[70+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[70+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[79]=';';
                y=side.a.moneySale[0];
                z=0;
                for(x=8;x>0;x--){
                    bufferBeagleTX[79+x]=side.a.moneySale[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[79+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[79+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[88]=';';
                y=side.a.ppuSale[0];
                z=0;
                for(x=6;x>0;x--){
                    bufferBeagleTX[88+x]=side.a.ppuSale[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[88+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[88+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[95]=';';
                bufferBeagleTX[96]=side.a.productSale;
                bufferBeagleTX[97]=';';
                if((y=bufferLCD1.licenceSale[0])==0){
                    y=1;
                    bufferLCD1.licenceSale[1]=' ';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[97+x]=bufferLCD1.licenceSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[97+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[108]=';';
                if((y=bufferLCD1.mileageSale[0])==0){
                    y=1;
                    bufferLCD1.mileageSale[1]='0';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[108+x]=bufferLCD1.mileageSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[108+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[119]=';';
                if((y=bufferLCD1.identySale[0])==0){
                    y=1;
                    bufferLCD1.identySale[1]='0';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[119+x]=bufferLCD1.identySale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[119+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[130]=';';
                bufferBeagleTX[131]='2';
                bufferBeagleTX[132]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[133+x]=bufferLCD1.dateDownHandle[x];
                    bufferBeagleTX[139+x]=bufferLCD1.timeDownHandle[x];
                }
                bufferBeagleTX[145]=';';
                bufferBeagleTX[146]=(bufferLCD1.vehicleType&0x07)+0x30;
                bufferBeagleTX[147]=';';
                bufferBeagleTX[148]=(bufferLCD1.idType&0x07)+0x30;
                bufferBeagleTX[149]=';';
                if((y=bufferLCD1.idSerial[0])==0){
                    y=1;
                    bufferLCD1.idSerial[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[149+x]=bufferLCD1.idSerial[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[149+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[170]=';';
                bufferBeagleTX[171]=typeIdSeller;
                bufferBeagleTX[172]=';';
                if((y=idSeller[0])==0){
                    y=1;
                    idSeller[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[172+x]=idSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[172+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[193]=';';
                y=bufferLCD1.totalVolumeAfter[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[193+x]=bufferLCD1.totalVolumeAfter[y];
                    y--;
                    z++;
                    if(z==(decimalVolume-1)){
                        x--;
                        bufferBeagleTX[193+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[193+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[207]=';';
                y=bufferLCD1.totalMoneyAfter[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[207+x]=bufferLCD1.totalMoneyAfter[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[207+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[207+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[221]=';';
                y=bufferLCD1.totalPPUAfter[0];
                z=0;
                for(x=6;x>0;x--){
                    bufferBeagleTX[221+x]=bufferLCD1.totalPPUAfter[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[221+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[221+x]='0';
                        }
                        break;
                    }
                }
                size=227;
            break;
                
            case 0x21://Peticion Venta Seleccionada Forma de Pago
                bufferBeagleTX[4]='d';
                bufferBeagleTX[5]=';';
                y=bufferLCD1.selectedSale[0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD1.selectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[14]=';';
                bufferBeagleTX[15]=(bufferLCD1.wayToPay/10)+48;
                bufferBeagleTX[16]=(bufferLCD1.wayToPay%10)+48;
                bufferBeagleTX[17]=';';
                if(bufferLCD1.flagWayToPayMixed==1){
                    bufferBeagleTX[18]='3';
                }else{
                     bufferBeagleTX[18]=bufferLCD1.salePerform;
                }
                size=18;
            break;
            
            case 0x24://Fin Forma de Pago
                bufferBeagleTX[4]='e';
                bufferBeagleTX[5]=';';
                y=bufferLCD1.selectedSale[0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD1.selectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[14]=';';
                bufferBeagleTX[15]=(bufferLCD1.wayToPay/10)+48;
                bufferBeagleTX[16]=(bufferLCD1.wayToPay%10)+48;
                bufferBeagleTX[17]=';';
                bufferBeagleTX[18]=bufferLCD1.salePerform;
                bufferBeagleTX[19]=';';
                y=bufferLCD1.moneySelectedSale[0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[19+x]=bufferLCD1.moneySelectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[19+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[28]=';';
                y=bufferLCD1.serialSelectedSale[0];
                for(x=20;x>0;x--){
                    bufferBeagleTX[28+x]=bufferLCD1.serialSelectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[28+x]=' ';
                        }
                        break;
                    }
                }
                size=48;
            break;
            
            case 0x26://Peticion Identificacion usuario credito
                bufferBeagleTX[4]='f';
                bufferBeagleTX[5]=';';
                y=bufferLCD1.mileageSale[0];
                if((y=bufferLCD1.mileageSale[0])==0){
                    y=1;
                    bufferLCD1.mileageSale[1]='0';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD1.mileageSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[16]=';';
                bufferBeagleTX[17]=bufferLCD1.idType;
                bufferBeagleTX[18]=';';
                y=bufferLCD1.idSerial[0];
                for(x=20;x>0;x--){
                    bufferBeagleTX[18+x]=bufferLCD1.idSerial[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[18+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[39]=';';
                bufferBeagleTX[40]=bufferLCD1.salePerform;
                bufferBeagleTX[41]=';';
                bufferBeagleTX[42]=(bufferLCD1.productType&0x07)+0x30;
                size=42;
            break;
            
            case 0x31://Peticion Identificacion producto canasta
                bufferBeagleTX[4]='g';
                bufferBeagleTX[5]=';';
                y=bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][0];
                if((y=bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][0])==0){
                    y=1;
                    bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][1]='0';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]=' ';
                        }
                        break;
                    }
                }
                size=25;
            break;
                
            case 0x33://Fin venta canasta
                bufferBeagleTX[4]='h';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=bufferLCD1.saleType;
                for(z=0;z<=2;z++){
                    bufferBeagleTX[7+(z*34)]=';';
                    y=bufferLCD1.serialMarket[z][0];
                    for(x=20;x>0;x--){
                        bufferBeagleTX[7+x+(z*34)]=bufferLCD1.serialMarket[z][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[7+x+(z*34)]=' ';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[28+(z*34)]=';';
                    y=bufferLCD1.quantityMarket[z][0];
                    for(x=3;x>0;x--){
                        bufferBeagleTX[28+x+(z*34)]=bufferLCD1.quantityMarket[z][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[28+x+(z*34)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[32+(z*34)]=';';
                    y=bufferLCD1.priceTotalMarket[z][0];
                    for(x=8;x>0;x--){
                        bufferBeagleTX[32+x+(z*34)]=bufferLCD1.priceTotalMarket[z][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[32+x+(z*34)]='0';
                            }
                            break;
                        }
                    }
                }
                bufferBeagleTX[109]=';';
                y=bufferLCD1.totalMarket[0];
                for(x=9;x>0;x--){
                    bufferBeagleTX[109+x]=bufferLCD1.totalMarket[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[109+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[119]=';';
                bufferBeagleTX[120]='2';
                bufferBeagleTX[121]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[122+x]=bufferLCD1.dateDownHandle[x];
                    bufferBeagleTX[128+x]=bufferLCD1.timeDownHandle[x];
                }
                bufferBeagleTX[134]=';';
                bufferBeagleTX[135]=(bufferLCD1.idType&0x07)+0x30;
                bufferBeagleTX[136]=';';
                if((y=bufferLCD1.idSerial[0])==0){
                    y=1;
                    bufferLCD1.idSerial[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[136+x]=bufferLCD1.idSerial[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[136+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[157]=';';
                bufferBeagleTX[158]=typeIdSeller;
                bufferBeagleTX[159]=';';
                if((y=idSeller[0])==0){
                    y=1;
                    idSeller[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[159+x]=idSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[159+x]='0';
                        }
                        break;
                    }
                }
                size=179;
            break;
            
            case 0x35://Peticion Consignacion
                bufferBeagleTX[4]='i';
                bufferBeagleTX[5]=';';
                y=bufferLCD1.priceConsign[0];
                for(x=10;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD1.priceConsign[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                size=15;
            break;
            
            case 0x37://Peticion Configurar Impresoras
                bufferBeagleTX[4]='j';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=(bufferLCD1.printers[0]&0x01)+0x30;
                bufferBeagleTX[7]=';';
                bufferBeagleTX[8]=(bufferLCD1.printers[1]&0x01)+0x30;
                size=8;
            break;
             
            case 0x41://Peticion Turno Apertura o Cierre
                bufferBeagleTX[4]='k';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=(!(turn&0x01))+0x30;
                bufferBeagleTX[7]=';';
                bufferBeagleTX[8]=typeIdSeller;
                bufferBeagleTX[9]=';';
                y=idSeller[0];
                if(y==0){
                    y=1;
                    idSeller[1]='0';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[9+x]=idSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[9+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[30]=';';
                y=passwordSeller[0];
                if(y==0){
                    y=1;
                    passwordSeller[1]='0';
                }
                for(x=4;x>0;x--){
                    bufferBeagleTX[30+x]=passwordSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[30+x]='0';
                        }
                        break;
                    }
                }
                for(w=0;w<=3;w++){
                    bufferBeagleTX[35+(w*35)]=';';
                    y=side.a.totalsHandle[w][0][0];
                    if(y==0){
                        y=1;
                        side.a.totalsHandle[w][0][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[35+x+(w*35)]=side.a.totalsHandle[w][0][y];
                        y--;
                        z++;
                        if(z==(decimalVolume-1)){
                            x--;
                            bufferBeagleTX[35+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[35+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[49+(w*35)]=';';
                    y=side.a.totalsHandle[w][1][0];
                    if(y==0){
                        y=1;
                        side.a.totalsHandle[w][1][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[49+x+(w*35)]=side.a.totalsHandle[w][1][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[49+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[49+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[63+(w*35)]=';';
                    y=side.a.totalsHandle[w][2][0];
                    if(y==0){
                        y=1;
                        side.a.totalsHandle[w][2][1]='0';
                    }
                    z=0;
                    for(x=6;x>0;x--){
                        bufferBeagleTX[63+x+(w*35)]=side.a.totalsHandle[w][2][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[63+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[63+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                }
                for(w=0;w<=3;w++){
                    bufferBeagleTX[175+(w*35)]=';';
                    y=side.b.totalsHandle[w][0][0];
                    if(y==0){
                        y=1;
                        side.b.totalsHandle[w][0][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[175+x+(w*35)]=side.b.totalsHandle[w][0][y];
                        y--;
                        z++;
                        if(z==(decimalVolume-1)){
                            x--;
                            bufferBeagleTX[175+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[175+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[189+(w*35)]=';';
                    y=side.b.totalsHandle[w][1][0];
                    if(y==0){
                        y=1;
                        side.b.totalsHandle[w][1][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[189+x+(w*35)]=side.b.totalsHandle[w][1][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[189+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[189+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[203+(w*35)]=';';
                    y=side.b.totalsHandle[w][2][0];
                    if(y==0){
                        y=1;
                        side.b.totalsHandle[w][2][1]='0';
                    }
                    z=0;
                    for(x=6;x>0;x--){
                        bufferBeagleTX[203+x+(w*35)]=side.b.totalsHandle[w][2][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[203+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[203+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                }
                bufferBeagleTX[315]=';';
                bufferBeagleTX[316]='2';
                bufferBeagleTX[317]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[318+x]=bufferLCD1.dateDownHandle[x];
                    bufferBeagleTX[324+x]=bufferLCD1.timeDownHandle[x];
                }
                size=329;
            break;
                
            default://Estado
                bufferBeagleTX[4]='a';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=bufferLCD1.stateMux[1];
                size=6;
            break;
        }
        for(x=0;x<=size;x++){
            Beagle_PutChar(bufferBeagleTX[x]);
        }
        Beagle_PutChar(';');
        Beagle_PutChar('&');
        bufferBeagleTX[3]=side.b.dir;
        switch(bufferLCD2.stateMux[1]){
            case 0x19://Surtiendo
                if(bufferLCD2.flagLiftHandle==0){
                    bufferBeagleTX[4]='m';
                    bufferBeagleTX[5]=';';
                    bufferBeagleTX[6]=bufferLCD2.presetType[1];
                    bufferBeagleTX[7]=';';
                    y=bufferLCD2.presetValue[1][0];
                    for(x=8;x>0;x--){
                        bufferBeagleTX[7+x]=bufferLCD2.presetValue[1][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[7+x]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[16]=';';
                    bufferBeagleTX[17]=bufferLCD2.productType+0x30;
                    bufferBeagleTX[18]=';';
                    y=bufferLCD2.totalVolumePrevious[0];
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[18+x]=bufferLCD2.totalVolumePrevious[y];
                        y--;
                        z++;
                        if(z==(decimalVolume-1)){
                            x--;
                            bufferBeagleTX[18+x]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[18+x]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[32]=';';
                    y=bufferLCD2.totalMoneyPrevious[0];
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[32+x]=bufferLCD2.totalMoneyPrevious[y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[32+x]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[32+x]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[46]=';';
                    y=bufferLCD2.totalPPUPrevious[0];
                    z=0;
                    for(x=6;x>0;x--){
                        bufferBeagleTX[46+x]=bufferLCD2.totalPPUPrevious[y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[46+x]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[46+x]='0';
                            }
                            break;
                        }
                    }
                    size=52;
                }else{
                    bufferBeagleTX[4]='a';
                    bufferBeagleTX[5]=';';
                    bufferBeagleTX[6]=0x19;//Surtiendo
                    size=6;
                }
            break;
                
            case 0x1A://Fin Venta
                bufferBeagleTX[4]='b';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=bufferLCD2.saleType;
                bufferBeagleTX[7]=';';
                bufferBeagleTX[8]=bufferLCD2.presetType[1];
                bufferBeagleTX[9]=';';
                y=bufferLCD2.presetValue[1][0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[9+x]=bufferLCD2.presetValue[1][y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[9+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[18]=';';
                bufferBeagleTX[19]=bufferLCD2.productType+0x30;
                bufferBeagleTX[20]=';';
                bufferBeagleTX[21]='2';
                bufferBeagleTX[22]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[23+x]=bufferLCD2.dateLiftHandle[x];
                    bufferBeagleTX[29+x]=bufferLCD2.timeLiftHandle[x];
                }
                bufferBeagleTX[35]=';';
                y=bufferLCD2.totalVolumePrevious[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[35+x]=bufferLCD2.totalVolumePrevious[y];
                    y--;
                    z++;
                    if(z==(decimalVolume-1)){
                        x--;
                        bufferBeagleTX[35+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[35+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[49]=';';
                y=bufferLCD2.totalMoneyPrevious[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[49+x]=bufferLCD2.totalMoneyPrevious[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[49+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[49+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[63]=';';
                y=bufferLCD2.totalPPUPrevious[0];
                z=0;
                for(x=6;x>0;x--){
                    bufferBeagleTX[63+x]=bufferLCD2.totalPPUPrevious[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[63+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[63+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[70]=';'; 
                y=side.b.volumeSale[0];
                z=0;
                for(x=8;x>0;x--){
                    bufferBeagleTX[70+x]=side.b.volumeSale[y];
                    y--;
                    z++;
                    if(z==decimalVolume){
                        x--;
                        bufferBeagleTX[70+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[70+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[79]=';';
                y=side.b.moneySale[0];
                z=0;
                for(x=8;x>0;x--){
                    bufferBeagleTX[79+x]=side.b.moneySale[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[79+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[79+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[88]=';';
                y=side.b.ppuSale[0];
                z=0;
                for(x=6;x>0;x--){
                    bufferBeagleTX[88+x]=side.b.ppuSale[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[88+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[88+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[95]=';';
                bufferBeagleTX[96]=side.b.productSale;
                bufferBeagleTX[97]=';';
                if((y=bufferLCD2.licenceSale[0])==0){
                    y=1;
                    bufferLCD2.licenceSale[1]=' ';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[97+x]=bufferLCD2.licenceSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[97+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[108]=';';
                if((y=bufferLCD2.mileageSale[0])==0){
                    y=1;
                    bufferLCD2.mileageSale[1]='0';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[108+x]=bufferLCD2.mileageSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[108+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[119]=';';
                if((y=bufferLCD2.identySale[0])==0){
                    y=1;
                    bufferLCD2.identySale[1]='0';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[119+x]=bufferLCD2.identySale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[119+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[130]=';';
                bufferBeagleTX[131]='2';
                bufferBeagleTX[132]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[133+x]=bufferLCD2.dateDownHandle[x];
                    bufferBeagleTX[139+x]=bufferLCD2.timeDownHandle[x];
                }
                bufferBeagleTX[145]=';';
                bufferBeagleTX[146]=(bufferLCD2.vehicleType&0x07)+0x30;
                bufferBeagleTX[147]=';';
                bufferBeagleTX[148]=(bufferLCD2.idType&0x07)+0x30;
                bufferBeagleTX[149]=';';
                if((y=bufferLCD2.idSerial[0])==0){
                    y=1;
                    bufferLCD2.idSerial[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[149+x]=bufferLCD2.idSerial[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[149+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[170]=';';
                bufferBeagleTX[171]=typeIdSeller;
                bufferBeagleTX[172]=';';
                if((y=idSeller[0])==0){
                    y=1;
                    idSeller[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[172+x]=idSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[172+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[193]=';';
                y=bufferLCD2.totalVolumeAfter[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[193+x]=bufferLCD2.totalVolumeAfter[y];
                    y--;
                    z++;
                    if(z==(decimalVolume-1)){
                        x--;
                        bufferBeagleTX[193+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[193+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[207]=';';
                y=bufferLCD2.totalMoneyAfter[0];
                z=0;
                for(x=13;x>0;x--){
                    bufferBeagleTX[207+x]=bufferLCD2.totalMoneyAfter[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[207+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[207+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[221]=';';
                y=bufferLCD2.totalPPUAfter[0];
                z=0;
                for(x=6;x>0;x--){
                    bufferBeagleTX[221+x]=bufferLCD2.totalPPUAfter[y];
                    y--;
                    z++;
                    if(z==decimalMoney){
                        x--;
                        bufferBeagleTX[221+x]=',';
                    }
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[221+x]='0';
                        }
                        break;
                    }
                }
                size=227;
            break;
            
            case 0x21://Venta Seleccionada Forma de Pago
                bufferBeagleTX[4]='d';
                bufferBeagleTX[5]=';';
                y=bufferLCD2.selectedSale[0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD2.selectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[14]=';';
                bufferBeagleTX[15]=(bufferLCD2.wayToPay/10)+48;
                bufferBeagleTX[16]=(bufferLCD2.wayToPay%10)+48;
                bufferBeagleTX[17]=';';
                if(bufferLCD2.flagWayToPayMixed==1){
                    bufferBeagleTX[18]='3';
                }else{
                    bufferBeagleTX[18]=bufferLCD2.salePerform;
                }
                size=18;
            break;
            
            case 0x24://Fin Forma de Pago
                bufferBeagleTX[4]='e';
                bufferBeagleTX[5]=';';
                y=bufferLCD2.selectedSale[0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD2.selectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[14]=';';
                bufferBeagleTX[15]=(bufferLCD2.wayToPay/10)+48;
                bufferBeagleTX[16]=(bufferLCD2.wayToPay%10)+48;
                bufferBeagleTX[17]=';';
                bufferBeagleTX[18]=bufferLCD2.salePerform;
                bufferBeagleTX[19]=';';
                y=bufferLCD2.moneySelectedSale[0];
                for(x=8;x>0;x--){
                    bufferBeagleTX[19+x]=bufferLCD2.moneySelectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[19+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[28]=';';
                y=bufferLCD2.serialSelectedSale[0];
                for(x=20;x>0;x--){
                    bufferBeagleTX[28+x]=bufferLCD2.serialSelectedSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[28+x]=' ';
                        }
                        break;
                    }
                }
                size=48;
            break;
            
            case 0x26://Peticion Identificacion usuario credito
                bufferBeagleTX[4]='f';
                bufferBeagleTX[5]=';';
                y=bufferLCD2.mileageSale[0];
                if((y=bufferLCD2.mileageSale[0])==0){
                    y=1;
                    bufferLCD2.mileageSale[1]='0';
                }
                for(x=10;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD2.mileageSale[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[16]=';';
                bufferBeagleTX[17]=bufferLCD2.idType;
                bufferBeagleTX[18]=';';
                y=bufferLCD2.idSerial[0];
                for(x=20;x>0;x--){
                    bufferBeagleTX[18+x]=bufferLCD2.idSerial[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[18+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[39]=';';
                bufferBeagleTX[40]=bufferLCD2.salePerform;
                bufferBeagleTX[41]=';';
                bufferBeagleTX[42]=(bufferLCD2.productType&0x07)+0x30;
                size=42;
            break;
            
            case 0x31://Peticion Identificacion producto canasta
                bufferBeagleTX[4]='g';
                bufferBeagleTX[5]=';';
                y=bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][0];
                if((y=bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][0])==0){
                    y=1;
                    bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][1]='0';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]=' ';
                        }
                        break;
                    }
                }
                size=25;
            break;
            
            case 0x33://Fin venta canasta
                bufferBeagleTX[4]='h';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=bufferLCD2.saleType;
                for(z=0;z<=2;z++){
                    bufferBeagleTX[7+(z*34)]=';';
                    y=bufferLCD2.serialMarket[z][0];
                    for(x=20;x>0;x--){
                        bufferBeagleTX[7+x+(z*34)]=bufferLCD2.serialMarket[z][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[7+x+(z*34)]=' ';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[28+(z*34)]=';';
                    y=bufferLCD2.quantityMarket[z][0];
                    for(x=3;x>0;x--){
                        bufferBeagleTX[28+x+(z*34)]=bufferLCD2.quantityMarket[z][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[28+x+(z*34)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[32+(z*34)]=';';
                    y=bufferLCD2.priceTotalMarket[z][0];
                    for(x=8;x>0;x--){
                        bufferBeagleTX[32+x+(z*34)]=bufferLCD2.priceTotalMarket[z][y];
                        y--;
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[32+x+(z*34)]='0';
                            }
                            break;
                        }
                    }
                }
                bufferBeagleTX[109]=';';
                y=bufferLCD2.totalMarket[0];
                for(x=9;x>0;x--){
                    bufferBeagleTX[109+x]=bufferLCD2.totalMarket[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[109+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[119]=';';
                bufferBeagleTX[120]='2';
                bufferBeagleTX[121]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[122+x]=bufferLCD2.dateDownHandle[x];
                    bufferBeagleTX[128+x]=bufferLCD2.timeDownHandle[x];
                }
                bufferBeagleTX[134]=';';
                bufferBeagleTX[135]=(bufferLCD2.idType&0x07)+0x30;
                bufferBeagleTX[136]=';';
                if((y=bufferLCD2.idSerial[0])==0){
                    y=1;
                    bufferLCD2.idSerial[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[136+x]=bufferLCD2.idSerial[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[136+x]=' ';
                        }
                        break;
                    }
                }
                bufferBeagleTX[157]=';';
                bufferBeagleTX[158]=typeIdSeller;
                bufferBeagleTX[159]=';';
                if((y=idSeller[0])==0){
                    y=1;
                    idSeller[1]=' ';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[159+x]=idSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[159+x]='0';
                        }
                        break;
                    }
                }
                size=179;
            break;    
            
            case 0x35://Peticion Consignacion
                bufferBeagleTX[4]='i';
                bufferBeagleTX[5]=';';
                y=bufferLCD2.priceConsign[0];
                for(x=10;x>0;x--){
                    bufferBeagleTX[5+x]=bufferLCD2.priceConsign[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[5+x]='0';
                        }
                        break;
                    }
                }
                size=15;
            break;
                
            case 0x37://Peticion Configurar Impresoras
                bufferBeagleTX[4]='j';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=(bufferLCD2.printers[0]&0x01)+0x30;
                bufferBeagleTX[7]=';';
                bufferBeagleTX[8]=(bufferLCD2.printers[1]&0x01)+0x30;
                size=8;
            break;    
            
            case 0x41://Peticion Turno Apertura o Cierre
                bufferBeagleTX[4]='k';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=(!(turn&0x01))+0x30;
                bufferBeagleTX[7]=';';
                bufferBeagleTX[8]=typeIdSeller;
                bufferBeagleTX[9]=';';
                y=idSeller[0];
                if(y==0){
                    y=1;
                    idSeller[1]='0';
                }
                for(x=20;x>0;x--){
                    bufferBeagleTX[9+x]=idSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[9+x]='0';
                        }
                        break;
                    }
                }
                bufferBeagleTX[30]=';';
                y=passwordSeller[0];
                if(y==0){
                    y=1;
                    passwordSeller[1]='0';
                }
                for(x=4;x>0;x--){
                    bufferBeagleTX[30+x]=passwordSeller[y];
                    y--;
                    if(y<=0){
                        while(x>1){
                            x--;
                            bufferBeagleTX[30+x]='0';
                        }
                        break;
                    }
                }
                for(w=0;w<=3;w++){
                    bufferBeagleTX[35+(w*35)]=';';
                    y=side.a.totalsHandle[w][0][0];
                    if(y==0){
                        y=1;
                        side.a.totalsHandle[w][0][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[35+x+(w*35)]=side.a.totalsHandle[w][0][y];
                        y--;
                        z++;
                        if(z==(decimalVolume-1)){
                            x--;
                            bufferBeagleTX[35+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[35+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[49+(w*35)]=';';
                    y=side.a.totalsHandle[w][1][0];
                    if(y==0){
                        y=1;
                        side.a.totalsHandle[w][1][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[49+x+(w*35)]=side.a.totalsHandle[w][1][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[49+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[49+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[63+(w*35)]=';';
                    y=side.a.totalsHandle[w][2][0];
                    if(y==0){
                        y=1;
                        side.a.totalsHandle[w][2][1]='0';
                    }
                    z=0;
                    for(x=6;x>0;x--){
                        bufferBeagleTX[63+x+(w*35)]=side.a.totalsHandle[w][2][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[63+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[63+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                }
                for(w=0;w<=3;w++){
                    bufferBeagleTX[175+(w*35)]=';';
                    y=side.b.totalsHandle[w][0][0];
                    if(y==0){
                        y=1;
                        side.b.totalsHandle[w][0][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[175+x+(w*35)]=side.b.totalsHandle[w][0][y];
                        y--;
                        z++;
                        if(z==(decimalVolume-1)){
                            x--;
                            bufferBeagleTX[175+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[175+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[189+(w*35)]=';';
                    y=side.b.totalsHandle[w][1][0];
                    if(y==0){
                        y=1;
                        side.b.totalsHandle[w][1][1]='0';
                    }
                    z=0;
                    for(x=13;x>0;x--){
                        bufferBeagleTX[189+x+(w*35)]=side.b.totalsHandle[w][1][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[189+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[189+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                    bufferBeagleTX[203+(w*35)]=';';
                    y=side.b.totalsHandle[w][2][0];
                    if(y==0){
                        y=1;
                        side.b.totalsHandle[w][2][1]='0';
                    }
                    z=0;
                    for(x=6;x>0;x--){
                        bufferBeagleTX[203+x+(w*35)]=side.b.totalsHandle[w][2][y];
                        y--;
                        z++;
                        if(z==decimalMoney){
                            x--;
                            bufferBeagleTX[203+x+(w*35)]=',';
                        }
                        if(y<=0){
                            while(x>1){
                                x--;
                                bufferBeagleTX[203+x+(w*35)]='0';
                            }
                            break;
                        }
                    }
                }
                bufferBeagleTX[315]=';';
                bufferBeagleTX[316]='2';
                bufferBeagleTX[317]='0';
                for(x=0;x<6;x++){
                    bufferBeagleTX[318+x]=bufferLCD2.dateDownHandle[x];
                    bufferBeagleTX[324+x]=bufferLCD2.timeDownHandle[x];
                }
                size=329;
            break;     
                
            default://Estado
                bufferBeagleTX[4]='a';
                bufferBeagleTX[5]=';';
                bufferBeagleTX[6]=bufferLCD2.stateMux[1];
                size=6;
            break;
        }
        for(x=3;x<=size;x++){
            Beagle_PutChar(bufferBeagleTX[x]);
        }
        Beagle_PutChar(';');
        Beagle_PutChar('&');
        Beagle_PutChar('*');
        countBeagleTX=0;
    }
}


/*
*********************************************************************************************************
*                                         void polling_LCD1(void)
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

void polling_LCD1(void){
    uint16 x,y;
    switch(flowLCD1){
        case 0:
            if(countAnimation1>=delayPicture1){
                isr_1_Stop(); 
                Waitable_1_Stop();
                flowLCD1=1;
            }
        break;
        
        case 1:
            isr_1_StartEx(timerAnimation1); 
            Waitable_1_Start();
            countAnimation1=0;
            flowLCD1=2;
        break;
            
        case 2: //Animacion
            if(LCD1_GetRxBufferSize()==8){
                isr_1_Stop(); 
                Waitable_1_Stop();
                set_picture(1,30);
                write_button(1,'M');    //Menu
                flowLCD1=3;
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 3: //Menu
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){
                        case 0x01:  //Ventas
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                bufferLCD1.flagLiftHandle=0;
                                set_picture(1,33);
                                write_button(1,'S');    //Venta
                                bufferLCD1.salePerform='1';
                                flowLCD1=5;
                            }else{
                                if(lockTurn==1){
                                    show_picture(1,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(1,67,3);
                                }else{
                                    show_picture(1,44,3);
                                    write_button(1,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x02:  //Turnos
                            if(flowLCD2>4){
                                show_picture(1,44,3);
                                write_button(1,'E');//Error de sistema
                            }else{
                                set_picture(1,64);
                                flowLCD1=40;
                                if(turn==1){//Abierto
                                    write_button(1,'v');//ID Vendedor
                                    write_button(1,'x');//Cerrar Turno
                                }else{//Cerrado
                                    write_button(1,'y');//Abrir Turno
                                }
                            }
                        break;
                       
                        case 0x03:  //Canasta 
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                set_picture(1,33);
                                write_button(1,'S');    //Venta
                                bufferLCD1.salePerform='2';
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=20;x++){
                                        bufferLCD1.serialMarket[y][x]='0';
                                    }
                                    bufferLCD1.serialMarket[y][0]=20;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=20;x++){
                                        bufferLCD1.productMarket[y][x]='-';
                                    }
                                    bufferLCD1.productMarket[y][0]=20;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=3;x++){
                                        bufferLCD1.quantityMarket[y][x]='0';
                                        bufferLCD1.quantAvailableMark[y][x]='0';
                                    }
                                    bufferLCD1.quantityMarket[y][0]=3;
                                    bufferLCD1.quantAvailableMark[y][0]=3;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=8;x++){
                                        bufferLCD1.priceTotalMarket[y][x]='0';
                                    }
                                    bufferLCD1.priceTotalMarket[y][0]=8;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=8;x++){
                                        bufferLCD1.priceUnitMarket[y][x]='0';
                                    }
                                    bufferLCD1.priceUnitMarket[y][0]=8;
                                }
                                for(x=1;x<=9;x++){
                                    bufferLCD1.totalMarket[x]='0';
                                }
                                bufferLCD1.totalMarket[0]=9;
                                flowLCD1=5;
                            }else{
                                if(lockTurn==1){
                                    show_picture(1,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(1,67,3);
                                }else{
                                    show_picture(1,44,3);
                                    write_button(1,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x04:  //Consignaciones 
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                set_picture(1,37);
                                write_button(1,'0');    //Teclado antidad Canasta / Cantidad Consignacion
                                write_LCD(1,symbols[0],2,2,1,0x0000,'N');
                                numberKeys1=0;
                                if(decimalMoney>0){
                                    flagPoint1=0;
                                }else{
                                    flagPoint1=1;
                                }
                                flowLCD1=35;
                            }else{
                                if(lockTurn==1){
                                    show_picture(1,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(1,67,3);
                                }else{
                                    show_picture(1,44,3);
                                    write_button(1,'a');//Error abrir turno
                                }
                            }
                        break; 	 
                        
                        case 0x05:  //Mantenimiento 
                            if(turn==1 && lockTurn==0){//Abierto y desbloqueado
                                set_picture(1,61);
                                write_button(1,'u');//Mantenimiento
                                flowLCD1=38;
                            }else{
                                if(lockTurn==1){
                                    show_picture(1,66,3);
                                }else{
                                    show_picture(1,44,3);
                                    write_button(1,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x06:  //Copia de recibo 
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                show_picture(1,55,3);
                                bufferLCD1.stateMux[1]=0x34;//Recibo de impresion
                                write_button(1,'W');//Espere 1 momento
                            }else{
                                if(lockTurn==1){
                                    show_picture(1,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(1,67,3);
                                }else{
                                    show_picture(1,44,3);
                                    write_button(1,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x07:  //Informacion
                            set_picture(1,32);
                            write_button(1,'I');
                            show_info(1,version);
                            flowLCD1=4;
                        break;
                        
                        case 0x0F:  //Atras
                            flowLCD1=1;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
        	
        case 4: //Informacion
            show_time_date(1);
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){
                        case 0x0e:  //Configuraciones
                            show_picture(1,55,5);
                            write_button(1,'W');//Espere 1 momento
                            bufferLCD1.stateMux[1]=0x3A;//Solicitud de Configuraciones
                        break;
                        
                        case 0x0f:  //Atras
                            set_picture(1,30);
                            write_button(1,'M');    //Menu
                            flowLCD1=3;
                        break;
                    } 
                }
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 5: //Ventas
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){ 
                        case 0x08:  //Contado
                            bufferLCD1.saleType='1';
                            if(bufferLCD1.salePerform=='1'){//Combustible
                                if(screen[0]=='1'){
                                    set_picture(1,49);
                                    write_button(1,'V');    //Tipo de vehiculo
                                    flowLCD1=6;
                                }else{
                                    bufferLCD1.vehicleType=0;
                                    set_picture(1,35);
                                    write_button(1,'C');    //Contado
                                    flowLCD1=7;
                                }
                            }else if(bufferLCD1.salePerform=='2'){//Canasta
                                bufferLCD1.stateMux[0]=0;
                                set_picture(1,59);
                                write_button(1,'c');    //Canasta
                                show_market(1);
                                bufferLCD1.idType='0';
                                bufferLCD1.idSerial[0]=0;
                                bufferLCD1.stateMux[1]=0x39;//Entra a menu de canasta
                                flowLCD1=31;
                            }
                        break;
                       
                        case 0x09:  //Credito
                            bufferLCD1.saleType='2';
                            if(bufferLCD1.salePerform=='1'){//Combustible
                                set_picture(1,37);
                                write_button(1,'4');    //Teclado Kilometraje
                                numberKeys1=0;
                                flagPoint1=1;
                                flowLCD1=20;
                            }else if(bufferLCD1.salePerform=='2'){//Canasta
                                bufferLCD1.mileageSale[0]=0;
                                bufferLCD1.productType=0;
                                set_picture(1,49);
                                write_button(1,'m');//Metodos ID
                                flowLCD1=21;
                            }    
                        break;
                        
                        case 0x0A://Formas de pago
                            bufferLCD1.flagWayToPayMixed=0;
                            if(screen[1]=='1'){
                                set_picture(1,37);
                                write_button(1,'6');//Teclado Venta F. de pago
                                bufferLCD1.valueKeys[1]='0';
                                numberKeys1=1;
                                write_LCD(1,bufferLCD1.valueKeys[numberKeys1],2,numberKeys1+2,1,0x0000,'N');
                                flagPoint1=1;
                                flowLCD1=14;
                            }else{
                                bufferLCD1.selectedSale[0]=1;
                                bufferLCD1.selectedSale[1]='0';
                                set_picture(1,49);
                                write_button(1,'F');//Formas de pago
                                bufferLCD1.flagPayment=1;
                                write_button(1,bufferLCD1.flagPayment);//Nombres a Formas de pago
                                flowLCD1=15;
                            }
                        break;
                            
                        case 0x0B://Forma de pago Mixto
                            bufferLCD1.flagWayToPayMixed=1;
                            if(screen[1]=='1'){
                                set_picture(1,37);
                                write_button(1,'6');//Teclado Venta F. de pago
                                bufferLCD1.valueKeys[1]='0';
                                numberKeys1=1;
                                write_LCD(1,bufferLCD1.valueKeys[numberKeys1],2,numberKeys1+2,1,0x0000,'N');
                                flagPoint1=1;
                                flowLCD1=14;
                            }else{
                                bufferLCD1.selectedSale[0]=1;
                                bufferLCD1.selectedSale[1]='0';
                                set_picture(1,49);
                                write_button(1,'F');//Formas de pago
                                bufferLCD1.flagPayment=1;
                                write_button(1,bufferLCD1.flagPayment);//Nombres a Formas de pago
                                flowLCD1=15;
                            }
                        break;
                            
                        case 0x0F:  //Atras
                            set_picture(1,30);
                            write_button(1,'M');    //Menu
                            flowLCD1=3;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
        
        case 6: //Tipo de vehiculo
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    if(LCD1_rxBuffer[3]<=6){
                        bufferLCD1.vehicleType=LCD1_rxBuffer[3];
                        set_picture(1,35);
                        write_button(1,'C');    //Contado
                        flowLCD1=7;
                    }else if(LCD1_rxBuffer[3]==0x1A){//Atras
                        set_picture(1,33);
                        write_button(1,'S');    //Venta
                        flowLCD1=5;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 7: //Preset Contado
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){ 
                        case 0x10:  //Dinero
                            set_picture(1,37);
                            write_button(1,'1');    //Teclado Dinero
                            write_LCD(1,symbols[0],2,2,1,0x0000,'N');
                            bufferLCD1.presetType[0]=2;
                            bufferLCD1.presetType[1]='D';
                            numberKeys1=0;
                            if(decimalMoney>0){
                                flagPoint1=0;
                            }else{
                                flagPoint1=1;
                            }
                            flowLCD1=8;
                        break;
                       
                        case 0x11:  //Volumen 
                            set_picture(1,37);
                            write_button(1,'2');    //Teclado Volumen
                            write_LCD(1,symbols[1],2,2,1,0x0000,'N');
                            bufferLCD1.presetType[0]=1;
                             bufferLCD1.presetType[1]='V';
                            numberKeys1=0;
                            if(decimalVolume>0){
                                flagPoint1=0;
                            }else{
                                flagPoint1=1;
                            }
                            flowLCD1=8;
                        break;
                        
                        case 0x12:  //Full
                            bufferLCD1.presetType[0]=2;
                             bufferLCD1.presetType[1]='F';
                            for(x=1;x<(digits-1);x++){
                                bufferLCD1.presetValue[0][x]='9';
                            }
                            bufferLCD1.presetValue[0][x]='0';
                            bufferLCD1.presetValue[0][x+1]='0';
                            bufferLCD1.presetValue[0][0]=digits;
                            for(x=0;x<=bufferLCD1.presetValue[0][0];x++){
                                bufferLCD1.presetValue[1][x]=bufferLCD1.presetValue[0][x];
                            }
                            if(productNumber==1){
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                bufferLCD1.productType=1;
                                flowLCD1=10; 
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');//Escoja producto
                                flowLCD1=9;
                            }
                        break;
                       
                        case 0x13:  //P1
                            bufferLCD1.presetType[0]=2;
                             bufferLCD1.presetType[1]='1';
                            for(x=0;x<=presetFast[0][0];x++){
                                bufferLCD1.presetValue[0][x]=presetFast[0][x];
                                bufferLCD1.presetValue[1][x]=presetFast[0][x];
                            }
                            if(productNumber==1){
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                bufferLCD1.productType=1;
                                flowLCD1=10; 
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');//Escoja producto
                                flowLCD1=9;
                            }
                        break;
                        
                        case 0x14:  //P2
                            bufferLCD1.presetType[0]=2;
                             bufferLCD1.presetType[1]='2';
                            for(x=0;x<=presetFast[1][0];x++){
                                bufferLCD1.presetValue[0][x]=presetFast[1][x];
                                bufferLCD1.presetValue[1][x]=presetFast[1][x];
                            }
                            if(productNumber==1){
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                bufferLCD1.productType=1;
                                flowLCD1=10; 
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');//Escoja producto
                                flowLCD1=9;
                            }
                        break;
                       
                        case 0x15:  //P3
                            bufferLCD1.presetType[0]=2;
                             bufferLCD1.presetType[1]='3';
                            for(x=0;x<=presetFast[2][0];x++){
                                bufferLCD1.presetValue[0][x]=presetFast[2][x];
                                bufferLCD1.presetValue[1][x]=presetFast[2][x];
                            }
                            if(productNumber==1){
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                bufferLCD1.productType=1;
                                flowLCD1=10; 
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');//Escoja producto
                                flowLCD1=9;
                            }
                        break;
                        
                        case 0x0F:  //Atras
                            if(screen[0]=='1'){
                                set_picture(1,49);//33
                                write_button(1,'V');    //Tipo de vehiculo//S
                                flowLCD1=6;
                            }else{
                                set_picture(1,33);
                                write_button(1,'S');    //Venta
                                flowLCD1=5;
                            }
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 8: //Teclado dinero efectivo
            switch (alphanumeric_keyboard(1,digits,0)){
                case 0: //Cancelar
                    set_picture(1,35);
                    write_button(1,'C');//Contado
                    flowLCD1=7;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.presetValue[0][x]=bufferLCD1.valueKeys[x];
                        bufferLCD1.presetValue[1][x]=bufferLCD1.valueKeys[x];
                    }
                    temporal[0]=1;
                    temporal[1]='0';
                    if(higher(bufferLCD1.presetValue[1],temporal)==1){
                        if(productNumber==1){
                            set_picture(1,43);
                            write_button(1,'U');    //Suba la manija
                            bufferLCD1.productType=1;
                            flowLCD1=10;
                        }else{
                            set_picture(1,39+(productNumber-2));
                            write_button(1,'P');    //Escoja producto
                            flowLCD1=9; 
                        }
                    }
                break;
            }
        break;
            
        case 9: //Escoja producto
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    if(LCD1_rxBuffer[3]<=0x04){
                            bufferLCD1.productType=(LCD1_rxBuffer[3]&0x07);
                            set_picture(1,43);
                            write_button(1,'U');    //Suba la manija
                            flowLCD1=10;
                    }else if(LCD1_rxBuffer[3]==0x0F){
                        set_picture(1,35);
                        write_button(1,'C');//Contado
                        flowLCD1=7;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 10: //Suba la manija
            if(turn==0){//Turno cerrado
                flowLCD1=1;
                LCD1_ClearRxBuffer();
                break;
            }
            if(LCD1_GetRxBufferSize()==8){
                flowLCD1=1;
                LCD1_ClearRxBuffer();
                break;
            }
            if(get_state(side.a.dir)==0x07){
                CyDelay(100);
                if(get_handle(side.a.dir)==bufferLCD1.productType){
                    if(read_date()==1 && read_time()==1){
                        for(x=0;x<=2;x++){
                            bufferLCD1.timeLiftHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                            bufferLCD1.timeLiftHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                            bufferLCD1.dateLiftHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                            bufferLCD1.dateLiftHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                        }
                    }else{
                        for(x=0;x<6;x++){
                            bufferLCD1.timeLiftHandle[x]='0';
                            bufferLCD1.dateLiftHandle[x]='0';
                        }
                    }
                    set_picture(1,43);
                    write_button(1,'W');//Espere 1 momento
                    flowLCD1=11;
                    LCD1_ClearRxBuffer();
                }else{
                    bufferLCD1.stateMux[1]=0x16;//Espera
                    show_picture(1,44,3);
                    write_button(1,'E');//Error de sistema
                }
            }
        break;
            
        case 11: //Programar Equipo
            if(turn==0){//Turno cerrado
                flowLCD1=1;
                LCD1_ClearRxBuffer();
                break;
            }
            if(get_totals(side.a.dir)!=0){
                for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                    bufferLCD1.totalVolumePrevious[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                    bufferLCD1.totalMoneyPrevious[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                    bufferLCD1.totalPPUPrevious[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                }
                if(compare_ppu(bufferLCD1.totalPPUPrevious,bufferLCD1.productType,side.a.dir)==0){                       
                    if(price_change(side.a.dir,bufferLCD1.productType,side.a.ppuAuthorized[bufferLCD1.productType-1])==0){
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                        break;
                    }
                }
                if(preset_data(side.a.dir,bufferLCD1.productType,bufferLCD1.presetValue[0],bufferLCD1.presetType[0]&0x03)==0){
                    show_picture(1,44,3);
                    bufferLCD1.stateMux[1]=0x10;//Error
                    write_button(1,'E');//Error de sistema
                    break;
                }
                if(get_handle(side.a.dir)!=0){
                    Pump_AL_PutChar(0x10|side.a.dir);//Autoriza el surtidor
                    set_picture(1,48);
                    bufferLCD1.licenceSale[0]=0;
                    bufferLCD1.mileageSale[0]=0;
                    bufferLCD1.identySale[0]=0;
                    bufferLCD1.flagEndSale=0;
                    bufferLCD1.idType='0';
                    bufferLCD1.idSerial[0]=0;
                    write_button(1,'t');//Tanqueando con Datos
                    flowLCD1=12;
                    LCD1_ClearRxBuffer();
                }else{
                    show_picture(1,44,3);
                    bufferLCD1.stateMux[1]=0x10;//Error
                    write_button(1,'E');//Error de sistema
                }
            }else{
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x10;//Error
                write_button(1,'E');//Error de sistema
            }
        break;
            
        case 12: //Tanqueando
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){ 
                        case 0x01:  //Placa
                            set_picture(1,46);
                            write_button(1,'3');//Teclado Placa
                            numberKeys1=0;
                            bufferLCD1.flagKeyboard=1;
                            bufferLCD1.flagEndSale=0;
                            countAnimation1=0;
                            flowLCD1=13;
                        break;
                       
                        case 0x02:  //Kilometraje
                            set_picture(1,37);
                            write_button(1,'4');//Teclado Kilometraje
                            numberKeys1=0;
                            flagPoint1=0;
                            bufferLCD1.flagKeyboard=2;
                            bufferLCD1.flagEndSale=0;
                            countAnimation1=0;
                            flowLCD1=13;
                        break;
                        
                        case 0x03:  //CC/NIT
                            set_picture(1,37);
                            write_button(1,'5');//Teclado Nit/CC
                            numberKeys1=0;
                            flagPoint1=0;
                            bufferLCD1.flagKeyboard=3;
                            bufferLCD1.flagEndSale=0;
                            countAnimation1=0;
                            flowLCD1=13;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
                break;
            }
            switch(get_state(side.a.dir)){
                case 0x09://Surtiendo
                    bufferLCD1.stateMux[1]=0x19;//Surtiendo
                break;
                
                case 0x0A://Reporte de venta
                    set_picture(1,43);
                    write_button(1,'W');//Espere 1 momento
                    if(get_sale(side.a.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD1.timeDownHandle[x]='0';
                                bufferLCD1.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.a.dir)!=0){
                            for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                                bufferLCD1.totalVolumeAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                                bufferLCD1.totalMoneyAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                                bufferLCD1.totalPPUAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                            }
                        }else{
                            bufferLCD1.totalVolumeAfter[0]=1;
                            bufferLCD1.totalMoneyAfter[0]=1;
                            bufferLCD1.totalPPUAfter[0]=1;
                            bufferLCD1.totalVolumeAfter[1]='0';
                            bufferLCD1.totalMoneyAfter[1]='0';
                            bufferLCD1.totalPPUAfter[1]='0';
                        }
                        bufferLCD1.flagEndSale=1;
                        if(bufferLCD1.licenceSale[0]==0){
                            set_picture(1,46);
                            write_button(1,'3');//Teclado Placa
                            numberKeys1=0;
                            bufferLCD1.flagKeyboard=1;
                            isr_1_StartEx(timerAnimation1); 
                            Waitable_1_Start();
                            countAnimation1=0;
                            flowLCD1=13;
                        }else{
                            show_picture(1,54,3);
                            bufferLCD1.stateMux[1]=0x1A;//Fin venta
                            write_button(1,'G');//Gracias
                        }
                    }else{
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                    }
                break;
                
                case 0x0B://Reporte de venta
                    set_picture(1,43);
                    write_button(1,'W');//Espere 1 momento
                    if(get_sale(side.a.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD1.timeDownHandle[x]='0';
                                bufferLCD1.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.a.dir)!=0){
                            for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                                bufferLCD1.totalVolumeAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                                bufferLCD1.totalMoneyAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                                bufferLCD1.totalPPUAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                            }
                        }else{
                            bufferLCD1.totalVolumeAfter[0]=1;
                            bufferLCD1.totalMoneyAfter[0]=1;
                            bufferLCD1.totalPPUAfter[0]=1;
                            bufferLCD1.totalVolumeAfter[1]='0';
                            bufferLCD1.totalMoneyAfter[1]='0';
                            bufferLCD1.totalPPUAfter[1]='0';
                        }
                        bufferLCD1.flagEndSale=1;
                        if(bufferLCD1.licenceSale[0]==0){
                            set_picture(1,46);
                            write_button(1,'3');//Teclado Placa
                            numberKeys1=0;
                            bufferLCD1.flagKeyboard=1;
                            isr_1_StartEx(timerAnimation1); 
                            Waitable_1_Start();
                            countAnimation1=0;
                            flowLCD1=13;
                        }else{
                            show_picture(1,54,3);
                            bufferLCD1.stateMux[1]=0x1A;//Fin venta
                            write_button(1,'G');//Gracias
                        }
                    }else{
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                    }
                break;
                
                case 0x06://Espera
                    if(get_totals(side.a.dir)!=0){
                        for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                            bufferLCD1.totalVolumeAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                            bufferLCD1.totalMoneyAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                            bufferLCD1.totalPPUAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                        }
                        if(subtraction(bufferLCD1.totalVolumeAfter,bufferLCD1.totalVolumePrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.a.volumeSale[x]=residue[x];
            				}	
                            side.a.volumeSale[x]='0';
                            side.a.volumeSale[0]++;
                        }else{
                            bufferLCD1.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD1=1;
                            break;
                        }
                        if(subtraction(bufferLCD1.totalMoneyAfter,bufferLCD1.totalMoneyPrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.a.moneySale[x]=residue[x];
            				}
                        }else{
                            bufferLCD1.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD1=1;
                            break;
                        }
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD1.timeDownHandle[x]='0';
                                bufferLCD1.dateDownHandle[x]='0';
                            }
                        }
                        side.a.productSale=(bufferLCD1.productType&0x0F)+0x30;
        				for(x=0;x<=bufferLCD1.totalPPUAfter[0];x++){
        					side.a.ppuSale[x]=bufferLCD1.totalPPUAfter[x];
        				}
                        bufferLCD1.flagEndSale=1;
                        if(bufferLCD1.licenceSale[0]==0){
                            set_picture(1,46);
                            write_button(1,'3');//Teclado Placa
                            numberKeys1=0;
                            bufferLCD1.flagKeyboard=1;
                            isr_1_StartEx(timerAnimation1); 
                            Waitable_1_Start();
                            countAnimation1=0;
                            flowLCD1=13;
                        }else{
                            show_picture(1,54,3);
                            bufferLCD1.stateMux[1]=0x1A;//Fin venta
                            write_button(1,'G');//Gracias
                        }
                    }else{
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                    }
                break;
            }
        break;
            
        case 13: //Teclado Placa
            get_state(side.a.dir);
            switch (alphanumeric_keyboard(1,10,0)){
                case 0: //Cancelar
                    switch(bufferLCD1.flagKeyboard){
                        case 1://Placa
                            for(x=0;x<=10;x++){
                                bufferLCD1.licenceSale[x]=0;
                            }
                        break;
                        
                        case 2://Kilometraje
                            for(x=0;x<=10;x++){
                                bufferLCD1.mileageSale[x]=0;
                            }
                        break;
                        
                        case 3://CC/NIT
                            for(x=0;x<=10;x++){
                                bufferLCD1.identySale[x]=0;
                            }
                        break;
                    }
                    if(bufferLCD1.flagEndSale==1){
                        isr_1_Stop(); 
                        Waitable_1_Stop();
                        show_picture(1,54,3);
                        bufferLCD1.stateMux[1]=0x1A;//Fin venta
                        write_button(1,'G');//Gracias
                    }else{
                        set_picture(1,48);
                        write_button(1,'t');//Tanqueando con Datos
                        flowLCD1=12;
                    }
                    LCD1_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    switch(bufferLCD1.flagKeyboard){
                        case 1://Placa
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                bufferLCD1.licenceSale[x]=bufferLCD1.valueKeys[x];
                            }
                        break;
                        
                        case 2://Kilometraje
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                bufferLCD1.mileageSale[x]=bufferLCD1.valueKeys[x];
                            }
                        break;
                        
                        case 3://CC/NIT
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                bufferLCD1.identySale[x]=bufferLCD1.valueKeys[x];
                            }
                        break;
                    }
                    if(bufferLCD1.flagEndSale==1){
                        isr_1_Stop(); 
                        Waitable_1_Stop();
                        show_picture(1,54,3);
                        bufferLCD1.stateMux[1]=0x1A;//Fin venta
                        write_button(1,'G');//Gracias
                    }else{
                        set_picture(1,48);
                        write_button(1,'t');//Tanqueando con Datos
                        flowLCD1=12;
                    }
                    LCD1_ClearRxBuffer();
                break;
            }
            if(countAnimation1>100){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,54,3);
                bufferLCD1.stateMux[1]=0x1A;//Fin venta
                write_button(1,'G');//Gracias
            }
        break;
        
        case 14://Teclado Venta F. de pago 
            switch (alphanumeric_keyboard(1,8,0)){
                case 0: //Cancelar
                    set_picture(1,33);
                    write_button(1,'S');    //Venta
                    flowLCD1=5;
                    LCD1_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.selectedSale[x]=bufferLCD1.valueKeys[x];
                    }
                    set_picture(1,49);
                    write_button(1,'F');//Formas de pago
                    bufferLCD1.flagPayment=1;
                    write_button(1,bufferLCD1.flagPayment);//Nombres a Formas de pago
                    flowLCD1=15;
                    LCD1_ClearRxBuffer();
                break;
            }
        break;
        
        case 15: //Formas de pago
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    if(LCD1_rxBuffer[3]<=24){
                        bufferLCD1.wayToPay=LCD1_rxBuffer[3];    
                        bufferLCD1.stateMux[1]=0x21;
                        set_picture(1,55);
                        write_button(1,'W');//Espere 1 momento
                        isr_1_StartEx(timerAnimation1); 
                        Waitable_1_Start();
                        countAnimation1=0;
                        bufferLCD1.authorizationFlag=0;
                        flowLCD1=16;
                    }else if(LCD1_rxBuffer[3]==0x1A){
                        if(bufferLCD1.flagPayment==1){
                            if(screen[1]=='1'){
                                set_picture(1,37);
                                write_button(1,'6');//Teclado Venta F. de pago
                                bufferLCD1.valueKeys[1]='0';
                                numberKeys1=1;
                                write_LCD(1,bufferLCD1.valueKeys[numberKeys1],2,numberKeys1+2,1,0x0000,'N');
                                write_button(1,'6');//Teclado Venta F. de pago
                                flagPoint1=1;
                                flowLCD1=14;
                            }else{
                                set_picture(1,33);
                                write_button(1,'S');    //Venta
                                flowLCD1=5;
                            }
                        }else{
                            set_picture(1,47+bufferLCD1.flagPayment);
                            write_button(1,'F');//Formas de pago
                            bufferLCD1.flagPayment--;
                            write_button(1,bufferLCD1.flagPayment);//Nombres a Formas de pago
                        }                        
                    }else if(LCD1_rxBuffer[3]==0x1B){
                        set_picture(1,49+bufferLCD1.flagPayment);
                        write_button(1,'F');//Formas de pago
                        bufferLCD1.flagPayment++;
                        write_button(1,bufferLCD1.flagPayment);//Nombres a Formas de pago
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 16: //Esperando respuesta a peticion
            if(bufferLCD1.authorizationFlag!=0){
                isr_1_Stop(); 
                Waitable_1_Stop();
                if(bufferLCD1.authorizationFlag==1){
                    if(bufferLCD1.flagWayToPayMixed==1){
                        bufferLCD1.stateMux[1]=0x16;//Espera
                        show_picture(1,54,3);
                        write_button(1,'G');//Gracias
                    }else{
                        set_picture(1,37);
                        write_button(1,'7');//Teclado Dinero Venta F. de pago
                        write_LCD(1,symbols[0],2,2,1,0x0000,'N');
                        y=0;
                        for(x=1;x<=bufferLCD1.moneySelectedSale[0];x++){
                            if(bufferLCD1.moneySelectedSale[x]!='0'){
                                bufferLCD1.moneySelectedSale[0]-=(x-1);
                                for(y=1;y<=bufferLCD1.moneySelectedSale[0];y++){
                                    bufferLCD1.moneySelectedSale[y]=bufferLCD1.moneySelectedSale[x];
                                    bufferLCD1.valueKeys[y]=bufferLCD1.moneySelectedSale[x];
                                    x++;
                                }
                                break;
                            }
                        }
                        if(decimalMoney>0){
                            flagPoint1=0;
                        }else{
                            flagPoint1=1;
                        }
                         for(x=1;x<=bufferLCD1.moneySelectedSale[0];x++){
                            write_LCD(1,bufferLCD1.moneySelectedSale[x],2,x+2,1,0x0000,'N');
                            if(bufferLCD1.moneySelectedSale[x]==','){
                                flagPoint1=1;
                            }
                        }
                        numberKeys1=bufferLCD1.moneySelectedSale[0];
                        bufferLCD1.stateMux[1]=0x23;
                        flowLCD1=17;
                    }
                }else{
                    isr_1_Stop(); 
                    Waitable_1_Stop();
                    show_picture(1,44,3);
                    bufferLCD1.stateMux[1]=0x16;//Espera
                    write_button(1,'A');//Error de sistema
                }
                break;
            }
            if(countAnimation1>200){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'E');//Error de sistema
            }
        break;
            
        case 17://Teclado Discriminar dinero
            switch (alphanumeric_keyboard(1,bufferLCD1.moneySelectedSale[0],0)){
                case 0: //Cancelar
                    show_picture(1,44,3);
                    bufferLCD1.stateMux[1]=0x16;//Espera
                    write_button(1,'A');//Error de sistema
                    LCD1_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    temporal[0]=1;
                    temporal[1]='0';
                    if(higher(bufferLCD1.valueKeys,temporal)==1){
                        if(higher(bufferLCD1.moneySelectedSale,bufferLCD1.valueKeys)!=2){
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                bufferLCD1.moneySelectedSale[x]=bufferLCD1.valueKeys[x];
                            }
                            numberKeys1=0;
                            flagPoint1=0;
                            flowLCD1=18;
                            if(bufferLCD1.keyboardWayToPay==0){
                                bufferLCD1.serialSelectedSale[0]=1;
                                bufferLCD1.serialSelectedSale[1]='0';
                                bufferLCD1.stateMux[1]=0x24;
                                set_picture(1,55);
                                write_button(1,'W');//Espere 1 momento
                                isr_1_StartEx(timerAnimation1); 
                                Waitable_1_Start();
                                countAnimation1=0;
                                bufferLCD1.authorizationFlag=0;
                                flowLCD1=19;
                            }else{
                                set_picture(1,46);
                                write_button(1,'8');//Teclado Serial
                            }
                        }
                    }
                    LCD1_ClearRxBuffer();
                break;
            }
        break;
            
        case 18://Teclado Serial
            switch (alphanumeric_keyboard(1,20,0)){
                case 0: //Cancelar
                    show_picture(1,44,3);
                    bufferLCD1.stateMux[1]=0x16;//Espera
                    write_button(1,'A');//Error de sistema
                    LCD1_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.serialSelectedSale[x]=bufferLCD1.valueKeys[x];
                    }
                    bufferLCD1.stateMux[1]=0x24;
                    set_picture(1,55);
                    write_button(1,'W');//Espere 1 momento
                    isr_1_StartEx(timerAnimation1); 
                    Waitable_1_Start();
                    countAnimation1=0;
                    bufferLCD1.authorizationFlag=0;
                    flowLCD1=19;
                    LCD1_ClearRxBuffer();
                break;
            }
        break;
            
        case 19: //Esperando respuesta a envio forma de pago
            if(bufferLCD1.authorizationFlag!=0){
                isr_1_Stop(); 
                Waitable_1_Stop();
                bufferLCD1.stateMux[1]=0x16;//Espera
                if(bufferLCD1.authorizationFlag==1){
                    show_picture(1,54,3);
                    write_button(1,'G');//Gracias
                }else{
                    show_picture(1,44,3);
                    write_button(1,'A');//Accion anulada
                }
                break;
            }
            if(countAnimation1>200){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'E');//Error de sistema
            }
        break;
            
        case 20://Teclado Kilometraje Credito
            switch (alphanumeric_keyboard(1,10,0)){
                case 0: //Cancelar
                    set_picture(1,33);
                    write_button(1,'S');    //Venta
                    flowLCD1=5;
                    LCD1_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.mileageSale[x]=bufferLCD1.valueKeys[x];
                    }
                    set_picture(1,49);
                    write_button(1,'m');//Metodos ID
                    flowLCD1=21;
                    LCD1_ClearRxBuffer();
                break;
            }
        break;
            
        case 21://Tipo de Metodos ID
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){ 
                        case 0x01:  //Ibutton
                            bufferLCD1.idType='1';
                            set_picture(1,56);
                            write_button(1,'i');//Metodo 1
                            flowLCD1=22;
                        break;
                       
                        case 0x02:  //Tag
                            bufferLCD1.idType='2';
                            set_picture(1,56);
                            write_button(1,'j');//Metodo 2
                            flowLCD1=22;
                            Tag_ClearRxBuffer();
                            Tag_ClearTxBuffer();
                            Tag_PutChar('O');
                            Tag_PutChar('K');
                            Tag_PutChar(0x01);
                            CyDelay(100);
                        break;
                        
                        case 0x03:  //Numerico
                            bufferLCD1.idType='3';
                            set_picture(1,37);
                            write_button(1,'k');//Metodo 3
                            numberKeys1=0;
                            flagPoint1=1;
                            flowLCD1=22;
                        break;
                        
                        case 0x04:  //Alfanumerico
                            bufferLCD1.idType='4';
                            set_picture(1,46);
                            write_button(1,'l');//Metodo 4
                            numberKeys1=0;
                            flagPoint1=0;
                            flowLCD1=22;
                        break;
                        
                        case 0x05:  //Codigo de barras
                            bufferLCD1.idType='5';
                            set_picture(1,56);
                            write_button(1,'n');//Metodo 5
                            flowLCD1=22;
                            Code_Bar_ClearRxBuffer();
                        break;
                        
                        case 0x06:  //
                            bufferLCD1.idType='6';
                            write_button(1,'m');//Metodos ID
                        break;
                            
                        case 0x1A:  //Atras
                            if(bufferLCD1.salePerform=='1'){//Combustible
                                set_picture(1,37);
                                write_button(1,'4');    //Teclado Kilometraje
                                numberKeys1=0;
                                flagPoint1=1;
                                flowLCD1=20;
                            }else if(bufferLCD1.salePerform=='2'){//Canasta
                                set_picture(1,33);
                                write_button(1,'S');    //Venta
                                flowLCD1=5;
                            }
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 22://Identificacion de Metodos ID
            switch(bufferLCD1.idType){
                case '1':  //Ibutton
                    if(LCD1_GetRxBufferSize()==8){
                        set_picture(1,49);
                        write_button(1,'m');//Metodos ID
                        flowLCD1=21;
                        LCD1_ClearRxBuffer();
                        break;
                    }
                    if(touch_present(1)==1){
        				if(touch_write(1,0x33)){
        					for(x=1;x<=8;x++){
        						temporal[x]=touch_read_byte(1);
        					}
        					y=0;
        					for(x=1;x<8;x++){
                                y=crc_check(y,temporal[x]);
                            }
        					if(y==temporal[8]){
        						bufferLCD1.idSerial[0]=16;
        						y=16;
        						for(x=1;x<=8;x++){
        							if((temporal[x]&0x0F)>=10){
        								bufferLCD1.idSerial[y]=(temporal[x]&0x0F)+55;
        							}else{
        								bufferLCD1.idSerial[y]=(temporal[x]&0x0F)+48;				
        							}
                                    y--;
        							if(((temporal[x]>>4)&0x0F)>=10){
        								bufferLCD1.idSerial[y]=((temporal[x]>>4)&0x0F)+55;
        							}else{
        								bufferLCD1.idSerial[y]=((temporal[x]>>4)&0x0F)+48;				
        							}
                                    y--;
        						}
                                if(productNumber==1 || bufferLCD1.salePerform=='2'){
                                    bufferLCD1.productType=1;
                                    bufferLCD1.stateMux[1]=0x26;
                                    set_picture(1,55);
                                    write_button(1,'W');//Espere 1 momento
                                    isr_1_StartEx(timerAnimation1); 
                                    Waitable_1_Start();
                                    countAnimation1=0;
                                    bufferLCD1.authorizationFlag=0;
                                    flowLCD1=24;
                                }else{
                                    set_picture(1,39+(productNumber-2));
                                    write_button(1,'P');    //Escoja producto
                                    flowLCD1=23;
                                }
                                LCD1_ClearRxBuffer();
        					}
        				}
        			}
                break;
                
                case '2':  //Tag
                    if(LCD1_GetRxBufferSize()==8){
                        set_picture(1,49);
                        write_button(1,'m');//Metodos ID
                        flowLCD1=21;
                        LCD1_ClearRxBuffer();
                        break;
                    }
                    for(x=0;x<=29;x++){
                        temporal[x]=0x00;
                    }
                    if(serial_codetag(1)==1){
                        for(x=0;x<=temporal[0];x++){
                            bufferLCD1.idSerial[x]=temporal[x];
                        }
                        bufferLCD1.idSerial[0]=16;
						y=16;
						for(x=1;x<=8;x++){
							if((temporal[x]&0x0F)>=10){
								bufferLCD1.idSerial[y]=(temporal[x]&0x0F)+55;
							}else{
								bufferLCD1.idSerial[y]=(temporal[x]&0x0F)+48;				
							}
                            y--;
							if(((temporal[x]>>4)&0x0F)>=10){
								bufferLCD1.idSerial[y]=((temporal[x]>>4)&0x0F)+55;
							}else{
								bufferLCD1.idSerial[y]=((temporal[x]>>4)&0x0F)+48;				
							}
                            y--;
						}
                        if(productNumber==1 || bufferLCD1.salePerform=='2'){
                            bufferLCD1.productType=1;
                            bufferLCD1.stateMux[1]=0x26;
                            set_picture(1,55);
                            write_button(1,'W');//Espere 1 momento
                            isr_1_StartEx(timerAnimation1); 
                            Waitable_1_Start();
                            countAnimation1=0;
                            bufferLCD1.authorizationFlag=0;
                            flowLCD1=24;
                        }else{
                            set_picture(1,39+(productNumber-2));
                            write_button(1,'P');    //Escoja producto
                            flowLCD1=23;
                        }
                        LCD1_ClearRxBuffer();
                    }
                break;
                
                case '3':  //Numerico
                    switch (alphanumeric_keyboard(1,10,0)){
                        case 0: //Cancelar
                            set_picture(1,49);
                            write_button(1,'m');//Metodos ID
                            flowLCD1=21;
                            LCD1_ClearRxBuffer();
                        break;
                            
                        case 1: //Enter
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                bufferLCD1.idSerial[x]=bufferLCD1.valueKeys[x];
                            }
                            if(productNumber==1 || bufferLCD1.salePerform=='2'){
                                bufferLCD1.productType=1;
                                bufferLCD1.stateMux[1]=0x26;
                                set_picture(1,55);
                                write_button(1,'W');//Espere 1 momento
                                isr_1_StartEx(timerAnimation1); 
                                Waitable_1_Start();
                                countAnimation1=0;
                                bufferLCD1.authorizationFlag=0;
                                flowLCD1=24;
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');    //Escoja producto
                                flowLCD1=23;
                            }
                            LCD1_ClearRxBuffer();
                        break;
                    }
                break;
                
                case '4':   //Alfanumerico
                    switch (alphanumeric_keyboard(1,20,0)){
                        case 0: //Cancelar
                            set_picture(1,49);
                            write_button(1,'m');//Metodos ID
                            flowLCD1=21;
                            LCD1_ClearRxBuffer();
                        break;
                            
                        case 1: //Enter
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                bufferLCD1.idSerial[x]=bufferLCD1.valueKeys[x];
                            }
                            if(productNumber==1 || bufferLCD1.salePerform=='2'){
                                bufferLCD1.productType=1;
                                bufferLCD1.stateMux[1]=0x26;
                                set_picture(1,55);
                                write_button(1,'W');//Espere 1 momento
                                isr_1_StartEx(timerAnimation1); 
                                Waitable_1_Start();
                                countAnimation1=0;
                                bufferLCD1.authorizationFlag=0;
                                flowLCD1=24;
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');    //Escoja producto
                                flowLCD1=23;
                            }
                            LCD1_ClearRxBuffer();
                        break;
                    }
                break;
                
                case '5':   //Codigo de barras
                    if(LCD1_GetRxBufferSize()==8){
                        set_picture(1,49);
                        write_button(1,'m');//Metodos ID
                        flowLCD1=21;
                        LCD1_ClearRxBuffer();
                        break;
                    }
                    if(serial_codebar()==1){
                        for(x=0;x<=temporal[0];x++){
                            bufferLCD1.idSerial[x]=temporal[x];
                        }
                        if(productNumber==1 || bufferLCD1.salePerform=='2'){
                            bufferLCD1.productType=1;
                            bufferLCD1.stateMux[1]=0x26;
                            set_picture(1,55);
                            write_button(1,'W');//Espere 1 momento
                            isr_1_StartEx(timerAnimation1); 
                            Waitable_1_Start();
                            countAnimation1=0;
                            bufferLCD1.authorizationFlag=0;
                            flowLCD1=24;
                        }else{
                            set_picture(1,39+(productNumber-2));
                            write_button(1,'P');    //Escoja producto
                            flowLCD1=23;
                        }
                        LCD1_ClearRxBuffer();
                    }
                break;
                
                case '6':   //
                    /*No establecido*/
                break;
            }
        break;
        
        case 23: //Escoja producto venta Credito
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    if(LCD1_rxBuffer[3]<=0x04){
                        bufferLCD1.productType=(LCD1_rxBuffer[3]&0x07);
                        bufferLCD1.stateMux[1]=0x26;
                        set_picture(1,55);
                        write_button(1,'W');//Espere 1 momento
                        isr_1_StartEx(timerAnimation1); 
                        Waitable_1_Start();
                        countAnimation1=0;
                        bufferLCD1.authorizationFlag=0;
                        flowLCD1=24;
                    }else if(LCD1_rxBuffer[3]==0x0F){
                        if(bufferLCD1.salePerform=='3'){
                            set_picture(1,61);
                            write_button(1,'u');//Mantenimiento
                            flowLCD1=38;
                        }else{
                            set_picture(1,49);
                            write_button(1,'m');//Metodos ID
                            flowLCD1=21;
                        }
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 24://Esperando respuesta a envio de identificacion credito
            if(bufferLCD1.authorizationFlag!=0){
                if(bufferLCD1.authorizationFlag==1){
                    if(bufferLCD1.salePerform=='1' || bufferLCD1.salePerform=='3'){//Combustible o Calibracion
                        for(x=1;x<=digits;x++){
                            bufferLCD1.volumeQuota[x]=bufferLCD1.volumeQuota[x+(bufferLCD1.volumeQuota[0]-digits)];
                            bufferLCD1.moneyQuota[x]=bufferLCD1.moneyQuota[x+(bufferLCD1.moneyQuota[0]-digits)];
                        }
                        bufferLCD1.volumeQuota[0]=digits;
                        bufferLCD1.moneyQuota[0]=digits;
                    }
                    bufferLCD1.stateMux[1]=0x28;//Espera transaccion Credito
                    bufferLCD1.stateMux[0]=0x28;//Bandera habilitada al momento de identicar un producto en canastilla
                    countAnimation1=0;
                    flowLCD1=25;
                }else{
                    isr_1_Stop(); 
                    Waitable_1_Stop();
                    bufferLCD1.stateMux[1]=0x16;//Espera
                    flowLCD1=37;
                }
                set_picture(1,57);
                write_button(1,'o');//Continuar
                show_message(1,bufferLCD1.message);
                LCD1_ClearRxBuffer();
                break;
            }
            if(countAnimation1>200){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'E');//Error de sistema
            } 
        break;
            
        case 25://Mensaje autorizacion venta credito combustible
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){ 
                        case 0x0E:  //Continuar
                            if(bufferLCD1.salePerform=='1' || bufferLCD1.salePerform=='3'){//Combustible o calibracion
                                set_picture(1,35);
                                write_button(1,'C');    //Contado
                                flowLCD1=26;
                            }else if(bufferLCD1.salePerform=='2'){//Canasta
                                isr_1_Stop(); 
                                Waitable_1_Stop();
                                set_picture(1,59);
                                write_button(1,'c');    //Canasta
                                show_market(1);
                                flowLCD1=31;
                            }
                        break;
                            
                        case 0x0F:  //Atras
                            isr_1_Stop(); 
                            Waitable_1_Stop();
                            show_picture(1,44,3);
                            write_button(1,'A');//Accion anulada
                            bufferLCD1.stateMux[1]=0x16;//Espera
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
            if(countAnimation1>240){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'A');//Accion Anulada
            }
        break;
            
        case 26: //Preset Credito
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){ 
                        case 0x10:  //Dinero
                            set_picture(1,37);
                            write_button(1,'1');    //Teclado Dinero
                            write_LCD(1,symbols[0],2,2,1,0x0000,'N');
                            bufferLCD1.presetType[0]=2;
                            bufferLCD1.presetType[1]='D';
                            numberKeys1=0;
                            if(decimalMoney>0){
                                flagPoint1=0;
                            }else{
                                flagPoint1=1;
                            }
                            flowLCD1=27;
                        break;
                       
                        case 0x11:  //Volumen 
                            set_picture(1,37);
                            write_button(1,'2');    //Teclado Volumen
                            write_LCD(1,symbols[1],2,2,1,0x0000,'N');
                            bufferLCD1.presetType[0]=1;
                            bufferLCD1.presetType[1]='V';
                            numberKeys1=0;
                            if(decimalVolume>0){
                                flagPoint1=0;
                            }else{
                                flagPoint1=1;
                            }
                            flowLCD1=27;
                        break;
                        
                        case 0x12:  //Full
                            bufferLCD1.presetType[0]=2;
                             bufferLCD1.presetType[1]='F';
                            for(x=0;x<=digits;x++){
                                bufferLCD1.presetValue[0][x]=bufferLCD1.moneyQuota[x];
                                bufferLCD1.presetValue[1][x]=bufferLCD1.moneyQuota[x];
                            }
                            set_picture(1,43);
                            write_button(1,'U');//Suba la manija
                            flowLCD1=28; 
                        break;
                       
                        case 0x13:  //P1
                            if(higher(presetFast[0],bufferLCD1.moneyQuota)!=1){
                                bufferLCD1.presetType[0]=2;
                                bufferLCD1.presetType[1]='1';
                                for(x=0;x<=presetFast[0][0];x++){
                                    bufferLCD1.presetValue[0][x]=presetFast[0][x];
                                    bufferLCD1.presetValue[1][x]=presetFast[0][x];
                                }
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                flowLCD1=28;
                            }else{
                                flowLCD1=25;
                                set_picture(1,57);
                                show_message(1,bufferLCD1.message);
                                write_button(1,'o');//Continuar
                            }
                        break;
                        
                        case 0x14:  //P2
                            if(higher(presetFast[1],bufferLCD1.moneyQuota)!=1){
                                bufferLCD1.presetType[0]=2;
                                bufferLCD1.presetType[1]='2';
                                for(x=0;x<=presetFast[1][0];x++){
                                    bufferLCD1.presetValue[0][x]=presetFast[1][x];
                                    bufferLCD1.presetValue[1][x]=presetFast[1][x];
                                }
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                flowLCD1=28; 
                            }else{
                                flowLCD1=25;
                                set_picture(1,57);
                                show_message(1,bufferLCD1.message);
                                write_button(1,'o');//Continuar
                            }
                        break;
                       
                        case 0x15:  //P3
                            if(higher(presetFast[2],bufferLCD1.moneyQuota)!=1){
                                bufferLCD1.presetType[0]=2;
                                bufferLCD1.presetType[1]='3';
                                for(x=0;x<=presetFast[2][0];x++){
                                    bufferLCD1.presetValue[0][x]=presetFast[2][x];
                                    bufferLCD1.presetValue[1][x]=presetFast[2][x];
                                }
                                set_picture(1,43);
                                write_button(1,'U');//Suba la manija
                                flowLCD1=28; 
                            }else{
                                flowLCD1=25;
                                set_picture(1,57);
                                show_message(1,bufferLCD1.message);
                                write_button(1,'o');//Continuar
                            }
                        break;
                        
                        case 0x0F:  //Atras
                            flowLCD1=25;
                            set_picture(1,57);
                            show_message(1,bufferLCD1.message);
                            write_button(1,'o');//Continuar
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
            if(countAnimation1>240){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'A');//Accion Anulada
            }
        break;
        
        case 27: //Teclado dinero credito
            switch (alphanumeric_keyboard(1,digits,0)){
                case 0: //Cancelar
                    set_picture(1,35);
                    write_button(1,'C');    //Contado
                    flowLCD1=26;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.presetValue[0][x]=bufferLCD1.valueKeys[x];
                        bufferLCD1.presetValue[1][x]=bufferLCD1.valueKeys[x];
                    }
                    temporal[0]=1;
                    temporal[1]='0';
                    if(higher(bufferLCD1.presetValue[1],temporal)==1){
                        if(bufferLCD1.presetType[1]=='V'){
                            if(higher(bufferLCD1.presetValue[0],bufferLCD1.volumeQuota)!=1){
                                set_picture(1,43);
                                write_button(1,'U');    //Suba la manija
                                flowLCD1=28;                      
                            }else{
                                flowLCD1=25;
                                set_picture(1,57);
                                show_message(1,bufferLCD1.message);
                                write_button(1,'o');//Continuar
                            }
                        }else{
                            if(higher(bufferLCD1.presetValue[0],bufferLCD1.moneyQuota)!=1){
                                set_picture(1,43);
                                write_button(1,'U');    //Suba la manija
                                flowLCD1=28;                    
                            }else{
                                flowLCD1=25;
                                set_picture(1,57);
                                show_message(1,bufferLCD1.message);
                                write_button(1,'o');//Continuar
                            }
                        }
                    }
                break;
            }
            if(countAnimation1>240){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'A');//Accion Anulada
            }
        break;
            
        case 28: //Suba la manija
            if(countAnimation1>240){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'A');//Accion Anulada
                break;
            }
            if(LCD1_GetRxBufferSize()==8){
                isr_1_Stop(); 
                Waitable_1_Stop();
                flowLCD1=1;
                bufferLCD1.stateMux[1]=0x16;//Espera
                LCD1_ClearRxBuffer();
                break;
            }
            if(get_state(side.a.dir)==0x07){
                isr_1_Stop(); 
                Waitable_1_Stop();
                CyDelay(100);
                if((get_handle(side.a.dir)==bufferLCD1.productType)){
                    if(read_date()==1 && read_time()==1){
                        for(x=0;x<=2;x++){
                            bufferLCD1.timeLiftHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                            bufferLCD1.timeLiftHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                            bufferLCD1.dateLiftHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                            bufferLCD1.dateLiftHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                        }
                    }else{
                        for(x=0;x<6;x++){
                            bufferLCD1.timeLiftHandle[x]='0';
                            bufferLCD1.dateLiftHandle[x]='0';
                        }
                    }
                    set_picture(1,43);
                    write_button(1,'W');//Espere 1 momento
                    flowLCD1=29;
                    LCD1_ClearRxBuffer();
                }else{
                    bufferLCD1.stateMux[1]=0x16;//Espera
                    show_picture(1,44,3);
                    write_button(1,'E');//Error de sistema
                }
            }
        break;
            
        case 29: //Programar Equipo
            if(price_change(side.a.dir,bufferLCD1.productType,bufferLCD1.ppuQuota)==0){
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x10;//Error
                write_button(1,'E');//Error de sistema
                break;
            }
            if(get_totals(side.a.dir)!=0){
                for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                    bufferLCD1.totalVolumePrevious[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                    bufferLCD1.totalMoneyPrevious[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                    bufferLCD1.totalPPUPrevious[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                }
            }else{
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x10;//Error
                write_button(1,'E');//Error de sistema
            }
            if(preset_data(side.a.dir,bufferLCD1.productType,bufferLCD1.presetValue[0],bufferLCD1.presetType[0]&0x03)==0){
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x10;//Error
                write_button(1,'E');//Error de sistema
                break;
            }
            if(get_handle(side.a.dir)!=0){
                Pump_AL_PutChar(0x10|side.a.dir);//Autoriza el surtidor
                set_picture(1,45);
                bufferLCD1.licenceSale[0]=0;
                bufferLCD1.identySale[0]=0;
                write_button(1,'T');//Tanqueando
                flowLCD1=30;
                LCD1_ClearRxBuffer();
            }else{
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x10;//Error
                write_button(1,'E');//Error de sistema
            }
            
        break;
            
        case 30: //Tanqueando venta credito
            switch(get_state(side.a.dir)){
                case 0x09://Surtiendo
                    bufferLCD1.stateMux[1]=0x19;//Surtiendo
                break;
                
                case 0x0A://Reporte de venta
                    set_picture(1,43);
                    write_button(1,'W');//Espere 1 momento
                    if(get_sale(side.a.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD1.timeDownHandle[x]='0';
                                bufferLCD1.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.a.dir)!=0){
                            for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                                bufferLCD1.totalVolumeAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                                bufferLCD1.totalMoneyAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                                bufferLCD1.totalPPUAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                            }
                        }else{
                            bufferLCD1.totalVolumeAfter[0]=1;
                            bufferLCD1.totalMoneyAfter[0]=1;
                            bufferLCD1.totalPPUAfter[0]=1;
                            bufferLCD1.totalVolumeAfter[1]='0';
                            bufferLCD1.totalMoneyAfter[1]='0';
                            bufferLCD1.totalPPUAfter[1]='0';
                        }
                        if(price_change(side.a.dir,bufferLCD1.productType,side.a.ppuAuthorized[bufferLCD1.productType-1])==0){
                            show_picture(1,44,3);
                            bufferLCD1.stateMux[1]=0x10;//Error
                            write_button(1,'E');//Error de sistema
                            break;
                        }
                        show_picture(1,54,3);
                        bufferLCD1.stateMux[1]=0x1A;//Fin venta
                        write_button(1,'G');//Gracias
                    }else{
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                    }
                break;
                
                case 0x0B://Reporte de venta
                    set_picture(1,43);
                    write_button(1,'W');//Espere 1 momento
                    if(get_sale(side.a.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD1.timeDownHandle[x]='0';
                                bufferLCD1.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.a.dir)!=0){
                            for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                                bufferLCD1.totalVolumeAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                                bufferLCD1.totalMoneyAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                                bufferLCD1.totalPPUAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                            }
                        }else{
                            bufferLCD1.totalVolumeAfter[0]=1;
                            bufferLCD1.totalMoneyAfter[0]=1;
                            bufferLCD1.totalPPUAfter[0]=1;
                            bufferLCD1.totalVolumeAfter[1]='0';
                            bufferLCD1.totalMoneyAfter[1]='0';
                            bufferLCD1.totalPPUAfter[1]='0';
                        }
                        if(price_change(side.a.dir,bufferLCD1.productType,side.a.ppuAuthorized[bufferLCD1.productType-1])==0){
                            show_picture(1,44,3);
                            bufferLCD1.stateMux[1]=0x10;//Error
                            write_button(1,'E');//Error de sistema
                            break;
                        }
                        show_picture(1,54,3);
                        bufferLCD1.stateMux[1]=0x1A;//Fin venta
                        write_button(1,'G');//Gracias
                    }else{
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                    }
                break;
                
                case 0x06://Espera
                    if(get_totals(side.a.dir)!=0){
                        for(x=0;x<=side.a.totalsHandle[bufferLCD1.productType-1][0][0];x++){
                            bufferLCD1.totalVolumeAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][0][x];
                            bufferLCD1.totalMoneyAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][1][x];
                            bufferLCD1.totalPPUAfter[x]=side.a.totalsHandle[bufferLCD1.productType-1][2][x];
                        }
                        if(price_change(side.a.dir,bufferLCD1.productType,side.a.ppuAuthorized[bufferLCD1.productType-1])==0){
                            show_picture(1,44,3);
                            bufferLCD1.stateMux[1]=0x10;//Error
                            write_button(1,'E');//Error de sistema
                            break;
                        }
                        if(subtraction(bufferLCD1.totalVolumeAfter,bufferLCD1.totalVolumePrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.a.volumeSale[x]=residue[x];
            				}	
                            side.a.volumeSale[x]='0';
                            side.a.volumeSale[0]++;
                        }else{
                            bufferLCD1.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD1=1;
                            break;
                        }
                        if(subtraction(bufferLCD1.totalMoneyAfter,bufferLCD1.totalMoneyPrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.a.moneySale[x]=residue[x];
            				}
                        }else{
                            bufferLCD1.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD1=1;
                            break;
                        }
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD1.timeDownHandle[x]='0';
                                bufferLCD1.dateDownHandle[x]='0';
                            }
                        }
                        side.a.productSale=(bufferLCD1.productType&0x0F)+0x30;
        				for(x=0;x<=bufferLCD1.totalPPUAfter[0];x++){
        					side.a.ppuSale[x]=bufferLCD1.totalPPUAfter[x];
        				}
                        show_picture(1,54,3);
                        bufferLCD1.stateMux[1]=0x1A;//Fin venta
                        write_button(1,'G');//Gracias
                    }else{
                        show_picture(1,44,3);
                        bufferLCD1.stateMux[1]=0x10;//Error
                        write_button(1,'E');//Error de sistema
                    }
                break;
            }
        break;
            
        case 31://Formulario canastilla
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    if(LCD1_rxBuffer[3]<=3){//Productos
                        set_picture(1,46);
                        write_button(1,'9');
                        bufferLCD1.flagProductmarket=LCD1_rxBuffer[3]-1;
                        numberKeys1=0;
                        flowLCD1=32;
                        Code_Bar_ClearRxBuffer();
                    }else if(LCD1_rxBuffer[3]<=6){//Cantidades
                        set_picture(1,37);
                        write_button(1,'0');
                        bufferLCD1.flagKeyboard=LCD1_rxBuffer[3]-4;
                        numberKeys1=0;
                        flagPoint1=1;
                        flowLCD1=33;
                    }else if(LCD1_rxBuffer[3]<=9){//Borrar productos
                        for(x=1;x<=20;x++){
                            bufferLCD1.serialMarket[LCD1_rxBuffer[3]-7][x]='0';
                        }
                        bufferLCD1.serialMarket[LCD1_rxBuffer[3]-7][0]=20;
                        for(x=1;x<=20;x++){
                            bufferLCD1.productMarket[LCD1_rxBuffer[3]-7][x]='-';
                        }
                        bufferLCD1.productMarket[LCD1_rxBuffer[3]-7][0]=20;
                        for(x=1;x<=3;x++){
                            bufferLCD1.quantityMarket[LCD1_rxBuffer[3]-7][x]='0';
                            bufferLCD1.quantAvailableMark[LCD1_rxBuffer[3]-7][0]='0';
                        }
                        bufferLCD1.quantityMarket[LCD1_rxBuffer[3]-7][0]=3;
                        bufferLCD1.quantAvailableMark[LCD1_rxBuffer[3]-7][0]=3;
                        for(x=1;x<=8;x++){
                            bufferLCD1.priceTotalMarket[LCD1_rxBuffer[3]-7][x]='0';
                        }
                        bufferLCD1.priceTotalMarket[LCD1_rxBuffer[3]-7][0]=8;
                        for(x=1;x<=8;x++){
                            bufferLCD1.priceUnitMarket[LCD1_rxBuffer[3]-7][x]='0';
                        }
                        bufferLCD1.priceUnitMarket[LCD1_rxBuffer[3]-7][0]=8;
                        set_picture(1,59);
                        write_button(1,'c');    //Canasta
                        show_market(1);
                    }else if(LCD1_rxBuffer[3]==0x0E){//Enter
                        temporal[0]=1;
                        temporal[1]='0';
                        if(higher(temporal,bufferLCD1.totalMarket)==2){
                            if(read_date()==1 && read_time()==1){
                                for(x=0;x<=2;x++){
                                    bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                    bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                    bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                    bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                                }
                            }else{
                                for(x=0;x<6;x++){
                                    bufferLCD1.timeDownHandle[x]='0';
                                    bufferLCD1.dateDownHandle[x]='0';
                                }
                            }
                            if(bufferLCD1.saleType=='1'){//Contado
                                show_picture(1,54,4);
                                bufferLCD1.stateMux[1]=0x33;//Fin venta Canastilla
                                write_button(1,'G');//Gracias
                            }else{//credito
                                if(higher(bufferLCD1.totalMarket,bufferLCD1.moneyQuota)!=1){
                                    show_picture(1,54,4);
                                    bufferLCD1.stateMux[1]=0x33;//Fin venta Canastilla
                                    write_button(1,'G');//Gracias
                                }else{
                                    flowLCD1=24;
                                    set_picture(1,57);
                                    show_message(1,bufferLCD1.message);
                                    write_button(1,'o');//Continuar
                                }
                            }
                        }
                    }else if(LCD1_rxBuffer[3]==0x0F){//Atras
                        if(bufferLCD1.saleType=='1'){//Contado
                            set_picture(1,33);
                            write_button(1,'S');    //Venta
                            bufferLCD1.stateMux[1]=0x16;//Espera
                            flowLCD1=5;
                        }else{//credito
                            flowLCD1=24;
                            set_picture(1,57);
                            write_button(1,'o');//Continuar
                            show_message(1,bufferLCD1.message);
                        }
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 32://Teclado codigo producto y/o lector de barras
            if(serial_codebar()==1){
                for(x=0;x<=temporal[0];x++){
                    bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][x]=temporal[x];
                }
                for(x=1;x<=bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][0];x++){
                    write_LCD(1,bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][x],2,x+2,1,0x0000,'N');
                }
                bufferLCD1.stateMux[1]=0x31;
                set_picture(1,55);
                write_button(1,'W');//Espere 1 momento
                isr_1_StartEx(timerAnimation1); 
                Waitable_1_Start();
                countAnimation1=0;
                bufferLCD1.authorizationFlag=0;
                flowLCD1=34;
                LCD1_ClearRxBuffer();
                break;
            }
            switch (alphanumeric_keyboard(1,20,0)){
                case 0: //Cancelar
                    set_picture(1,59);
                    write_button(1,'c');    //Canasta
                    show_market(1);
                    flowLCD1=31;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][x]=bufferLCD1.valueKeys[x];
                    }
                    bufferLCD1.stateMux[1]=0x31;
                    set_picture(1,55);
                    write_button(1,'W');//Espere 1 momento
                    isr_1_StartEx(timerAnimation1); 
                    Waitable_1_Start();
                    countAnimation1=0;
                    bufferLCD1.authorizationFlag=0;
                    flowLCD1=34;
                break;
            }
        break;
            
        case 33://Teclado cantidades canasta
            switch (alphanumeric_keyboard(1,3,0)){
                case 0: //Cancelar
                    set_picture(1,59);
                    write_button(1,'c');    //Canasta
                    show_market(1);
                    flowLCD1=31;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][x]=bufferLCD1.valueKeys[x];
                    }
                    if(higher(bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard],bufferLCD1.quantAvailableMark[bufferLCD1.flagKeyboard])!=1){
                        if(bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][0]<3){
                            y=bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][0];
                            for(x=3;x>=1;x--){
                                bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][x]=bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][y];
                                y--;
                                if(y==0 && x>1){
                                    while(x>1){
                                        x--;
                                        bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][x]='0';
                                    }
                                    break;
                                }
                            }
                            bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][0]=3;
                        }
                    }else{
                        for(x=0;x<=3;x++){
                            bufferLCD1.quantityMarket[bufferLCD1.flagKeyboard][x]=bufferLCD1.quantAvailableMark[bufferLCD1.flagKeyboard][x];
                        }
                    }
                    set_picture(1,59);
                    write_button(1,'c');    //Canasta
                    show_market(1);
                    flowLCD1=31;
                break;
            }
        break;
            
        case 34://Esperando respuesta a envio de indentificacion de produto
            if(bufferLCD1.authorizationFlag!=0){
                isr_1_Stop(); 
                Waitable_1_Stop();
                if(bufferLCD1.stateMux[0]==0x28){
                    bufferLCD1.stateMux[1]=0x28;//Espera transaccion Credito
                }else{
                    bufferLCD1.stateMux[1]=0x39;//Entra a menu de canasta
                }
                set_picture(1,59);
                write_button(1,'c');    //Canasta
                show_market(1);
                flowLCD1=31;
                LCD1_ClearRxBuffer();
                break;
            }
            if(countAnimation1>200){
                isr_1_Stop(); 
                Waitable_1_Stop();
                for(x=1;x<=20;x++){
                    bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][x]='0';
                }
                bufferLCD1.serialMarket[bufferLCD1.flagProductmarket][0]=20;
                for(x=1;x<=20;x++){
                    bufferLCD1.productMarket[bufferLCD1.flagProductmarket][x]='-';
                }
                bufferLCD1.productMarket[bufferLCD1.flagProductmarket][0]=20;
                for(x=1;x<=3;x++){
                    bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][x]='0';
                }
                bufferLCD1.quantityMarket[bufferLCD1.flagProductmarket][0]=3;
                for(x=1;x<=8;x++){
                    bufferLCD1.priceTotalMarket[bufferLCD1.flagProductmarket][x]='0';
                }
                bufferLCD1.priceTotalMarket[bufferLCD1.flagProductmarket][0]=8;
                for(x=1;x<=8;x++){
                    bufferLCD1.priceUnitMarket[bufferLCD1.flagProductmarket][x]='0';
                }
                bufferLCD1.priceUnitMarket[bufferLCD1.flagProductmarket][0]=8;
                if(bufferLCD1.stateMux[0]==0x28){
                    bufferLCD1.stateMux[1]=0x28;//Espera transaccion Credito
                }else{
                    bufferLCD1.stateMux[1]=0x39;//Entra a menu de canasta
                }
                set_picture(1,59);
                write_button(1,'c');    //Canasta
                show_market(1);
                flowLCD1=31;
            }
        break;
            
        case 35://Teclado cantidades consignacion
            switch (alphanumeric_keyboard(1,10,0)){
                case 0: //Cancelar
                    set_picture(1,30);
                    write_button(1,'M');    //Menu
                    flowLCD1=3;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        bufferLCD1.priceConsign[x]=bufferLCD1.valueKeys[x];
                    }
                    bufferLCD1.stateMux[1]=0x35;
                    set_picture(1,55);
                    write_button(1,'W');//Espere 1 momento
                    isr_1_StartEx(timerAnimation1); 
                    Waitable_1_Start();
                    countAnimation1=0;
                    bufferLCD1.authorizationFlag=0;
                    flowLCD1=36;
                break;
            }
        break;
            
        case 36://Esperando respuesta a envio de consignacion
            if(bufferLCD1.authorizationFlag!=0){
                isr_1_Stop(); 
                Waitable_1_Stop();
                bufferLCD1.stateMux[1]=0x16;
                set_picture(1,57);
                show_message(1,bufferLCD1.message);
                write_button(1,'o');//Continuar
                flowLCD1=37;
                LCD1_ClearRxBuffer();
                break;
            }
            if(countAnimation1>200){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'E');//Error de sistema
            }
        break;
            
        case 37://Mensaje autorizacion consignacion o Turnos o creditos 
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    if(LCD1_rxBuffer[3]==0x0F || LCD1_rxBuffer[3]==0x0E){
                        flowLCD1=1;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 38://Mantenimiento
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){
                        case 0x01://calibrar
                            bufferLCD1.salePerform='3';
                            bufferLCD1.saleType='3';
                            bufferLCD1.licenceSale[0]=0;
                            bufferLCD1.mileageSale[0]=0;
                            bufferLCD1.identySale[0]=0;
                            bufferLCD1.idType='0';
                            bufferLCD1.idSerial[0]=1;
                            bufferLCD1.idSerial[1]=' ';
                            if(productNumber==1){
                                bufferLCD1.productType=1;
                                bufferLCD1.stateMux[1]=0x26;
                                set_picture(1,55);
                                write_button(1,'W');//Espere 1 momento
                                isr_1_StartEx(timerAnimation1); 
                                Waitable_1_Start();
                                countAnimation1=0;
                                bufferLCD1.authorizationFlag=0;
                                flowLCD1=24;
                            }else{
                                set_picture(1,39+(productNumber-2));
                                write_button(1,'P');    //Escoja producto
                                flowLCD1=23;
                            }
                        break;
                        
                        case 0x02://Impresion
                            set_picture(1,63);
                            write_button(1,'p');    //impresoras
                            bufferLCD1.printers[0]=1;
                            bufferLCD1.printers[1]=1;
                            flowLCD1=39;
                        break;
                        
                        case 0x03://Desbloquear surtidor
                            set_picture(1,37);
                            write_button(1,'*');//Teclado Contraseña
                            numberKeys1=0;
                            flagPoint1=1;
                            flowLCD1=45;
                        break;
                            
                        case 0x0F://Atras
                            set_picture(1,30);
                            write_button(1,'M');    //Menu
                            flowLCD1=3;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 39://Configurar impresoras
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){
                        case 0x01://Impresora 1
                            if(bufferLCD1.printers[0]==1){
                                bufferLCD1.printers[0]=0;
                                write_LCD(1,'X',12,4,3,0xF800,'Y');
                            }else{
                                bufferLCD1.printers[0]=1;
                                set_picture(1,63);
                                write_button(1,'p');    //impresoras
                                if(bufferLCD1.printers[1]==0){
                                    write_LCD(1,'X',12,8,3,0xF800,'Y');
                                }
                            }
                        break;
                        
                        case 0x02://Impresora 1
                            if(bufferLCD1.printers[1]==1){
                                bufferLCD1.printers[1]=0;
                                write_LCD(1,'X',12,8,3,0xF800,'Y');
                            }else{
                                bufferLCD1.printers[1]=1;
                                set_picture(1,63);
                                write_button(1,'p');    //impresoras
                                if(bufferLCD1.printers[0]==0){
                                    write_LCD(1,'X',12,4,3,0xF800,'Y');
                                }
                            }
                        break;
                            
                        case 0x0E://Continuar
                            bufferLCD1.stateMux[1]=0x37;//Configuracion Impresoras
                            show_picture(1,54,4);
                            write_button(1,'G');
                        break;
                        
                        case 0x0F://Atras
                            set_picture(1,61);
                            write_button(1,'u');//Mantenimiento
                            flowLCD1=38;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
        
        case 40://Abrir ó cerrar turno
            if(LCD1_GetRxBufferSize()==8){
                if((LCD1_rxBuffer[0]==0xAA) && (LCD1_rxBuffer[6]==0xC3) && (LCD1_rxBuffer[7]==0x3C)){
                    switch(LCD1_rxBuffer[3]){
                        case 0x0D://Bloquear posicion
                            if(turn==1){//Abierto el turno
                                set_picture(1,37);
                                write_button(1,'*');//Teclado Contraseña
                                numberKeys1=0;
                                flagPoint1=1;
                                flowLCD1=46;
                            }
                        break;
                        
                        case 0x0E://Continuar
                            if(lockTurn==0){//desbloqueado
                                bufferLCD1.stateMux[1]=0x40;//Pendiente datos turno
                                set_picture(1,37);
                                write_button(1,'5');//Teclado Nit/CC
                                numberKeys1=0;
                                flagPoint1=1;
                                flowLCD1=41;
                            }else{
                                show_picture(1,66,3);
                            }
                        break;
                        
                        case 0x0F://Atras
                            if(bufferLCD1.stateMux[1]==0x40){//Pendiente datos turno
                                bufferLCD1.stateMux[1]=0x16;//Espera
                            }
                            set_picture(1,30);
                            write_button(1,'M');    //Menu
                            flowLCD1=3;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD1_ClearRxBuffer();
            }
        break;
            
        case 41://Teclado Cedula Vendedor
            switch (alphanumeric_keyboard(1,10,0)){
                case 0: //Cancelar
                    set_picture(1,64);
                    flowLCD1=40;
                    for(x=1;x<=20;x++){
                        idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (x+955));
                    }
                    idSeller[0]=20;
                    typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
                    for(x=0;x<4;x++){
                        passwordSeller[x+1]=(CY_GET_REG8(CYDEV_EE_BASE + (938+x)));
                    }
                    if(turn==1){//Abierto
                        write_button(1,'x');//Cerrar Turno
                        write_button(1,'v');//ID Vendedor
                    }else{//Cerrado
                        write_button(1,'y');//Abrir Turno
                    }
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                        idSeller[x]=bufferLCD1.valueKeys[x];
                    }
                    typeIdSeller='C';
                    set_picture(1,37);
                    write_button(1,'*');//Teclado Contraseña
                    numberKeys1=0;
                    flagPoint1=1;
                    flowLCD1=42;
                break;
            }
        break;
            
        case 42://Contraseña Cedula Vendedor
            switch (alphanumeric_keyboard(1,4,'*')){
                case 0: //Cancelar
                    set_picture(1,37);
                    write_button(1,'5');//Teclado Nit/CC
                    numberKeys1=0;
                    flagPoint1=1;
                    flowLCD1=41;
                break;
                    
                case 1: //Enter
                    if(flowLCD2>4){
                        show_picture(1,44,3);
                        write_button(1,'E');//Error de sistema
                    }else{
                        if(bufferLCD1.valueKeys[0]==4){
                            for(x=0;x<=bufferLCD1.valueKeys[0];x++){
                                passwordSeller[x]=bufferLCD1.valueKeys[x];
                            }
                            set_picture(1,55);
                            write_button(1,'W');//Espere 1 momento
                            if(get_totals(side.a.dir)!=0 && get_totals(side.b.dir)!=0){
                                if(read_date()==1 && read_time()==1){
                                    for(x=0;x<=2;x++){
                                        bufferLCD1.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                        bufferLCD1.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                        bufferLCD1.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                        bufferLCD1.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                                    }
                                }else{
                                    for(x=0;x<6;x++){
                                        bufferLCD1.timeDownHandle[x]='0';
                                        bufferLCD1.dateDownHandle[x]='0';
                                    }
                                }
                                isr_1_StartEx(timerAnimation1); 
                                Waitable_1_Start();
                                countAnimation1=0;
                                bufferLCD1.stateMux[1]=0x41;//Peticion Turno
                                bufferLCD1.authorizationFlag=0;
                                show_picture(2,55,10);
                                write_button(2,'W');//Espere 1 momento
                                flowLCD1=43;
                            }else{
                                show_picture(1,44,3);
                                write_button(1,'E');//Error de sistema
                            }
                        }
                    }
                break;
            }
        break;
            
        case 43://Esperando respuesta a envio de Turno
            if(bufferLCD1.authorizationFlag!=0){
                isr_1_Stop(); 
                Waitable_1_Stop();
                bufferLCD1.stateMux[1]=0x16;
                set_picture(1,57);
                show_message(1,bufferLCD1.message);
                write_button(1,'o');//Continuar
                flowLCD1=37;
                LCD1_ClearRxBuffer();
                break;
            }
            if(countAnimation1>200){
                for(x=1;x<=20;x++){
                    idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (x+955));
                }
                idSeller[0]=20;
                typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
                for(x=0;x<4;x++){
                    passwordSeller[x+1]=(CY_GET_REG8(CYDEV_EE_BASE + (938+x)));
                }
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'E');//Error de sistema
            }
        break;
            
        case 44://Posicion Bloqueada por Corte
            if(countAnimation1>12000){
                isr_1_Stop(); 
                Waitable_1_Stop();
                show_picture(1,44,3);
                bufferLCD1.stateMux[1]=0x16;//Espera
                write_button(1,'E');//Error de sistema
            }
        break;
            
        case 45://Contraseña Desbloquear surtidor
            switch (alphanumeric_keyboard(1,6,'*')){
                case 0: //Cancelar
                    set_picture(1,61);
                    write_button(1,'u');//Mantenimiento
                    flowLCD1=38;
                break;
                    
                case 1: //Enter
                    for(x=1;x<=6;x++){
                        if(bufferLCD1.valueKeys[x]!=passwordPump[x-1]){
                            break;
                        }
                    }
                    if(x>6){
                        stateBeagleSoft=1;//En comunicacion
                        show_picture(1,54,4);
                        write_button(1,'G');//Gracias
                        for(x=0;x<3;x++){
                            if(get_state(side.a.dir)==0x06){
                                CyDelay(60);
                                Pump_AL_PutChar(0x10|side.a.dir);//Autoriza el surtidor
                                CyDelay(60);
                                Pump_AL_PutChar(0x30|side.a.dir);//Detiene el surtidor
                            }
                            CyWdtClear();
                            CyDelay(250);
                        }
                    }else{
                        show_picture(1,44,3);
                        write_button(1,'E');//Error de sistema
                    }
                break;
            }
        break;
            
        case 46://Contraseña Vendedor para bloquear turno
            switch (alphanumeric_keyboard(1,4,'*')){
                case 0: //Cancelar
                    set_picture(1,64);
                    flowLCD1=40;
                    if(turn==1){//Abierto
                        write_button(1,'x');//Cerrar Turno
                        write_button(1,'v');//ID Vendedor
                    }else{//Cerrado
                        write_button(1,'y');//Abrir Turno
                    }
                break;
                    
                case 1: //Enter
                    if(flowLCD2>4){
                        show_picture(1,44,3);
                        write_button(1,'E');//Error de sistema
                    }else{
                        if(bufferLCD1.valueKeys[0]==4){
                            for(x=1;x<=4;x++){
                                if(bufferLCD1.valueKeys[x]!=passwordSeller[x]){
                                    show_picture(1,44,3);
                                    write_button(1,'E');//Error de sistema
                                    break;
                                }
                            }
                            if(x>=5){
                                lockTurn=(!lockTurn);
                                show_picture(1,54,4);
                                write_button(1,'G');
                            }
                        }
                    }
                break;
            }
        break;
    }
}

/*
*********************************************************************************************************
*                                         void polling_LCD2(void)
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

void polling_LCD2(void){
    uint16 x,y;
    switch(flowLCD2){
        case 0:
            if(countAnimation2>=delayPicture2){
                isr_2_Stop(); 
                Waitable_2_Stop();
                flowLCD2=1;
            }
        break;
        
        case 1:
            isr_2_StartEx(timerAnimation2); 
            Waitable_2_Start();
            countAnimation2=0;
            flowLCD2=2;
        break;
            
        case 2: //Animacion
            if(LCD2_GetRxBufferSize()==8){
                isr_2_Stop(); 
                Waitable_2_Stop();
                set_picture(2,30);
                write_button(2,'M');    //Menu
                flowLCD2=3;
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 3: //Menu
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){
                        case 0x01:  //Ventas
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                bufferLCD2.flagLiftHandle=0;
                                set_picture(2,33);
                                write_button(2,'S');    //Venta
                                bufferLCD2.salePerform='1';
                                flowLCD2=5;
                            }else{
                                if(lockTurn==1){
                                    show_picture(2,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(2,67,3);
                                }else{
                                    show_picture(2,44,3);
                                    write_button(2,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x02:  //Turnos
                            if(flowLCD1>4){
                                show_picture(2,44,3);
                                write_button(2,'E');//Error de sistema
                            }else{
                                set_picture(2,64);
                                flowLCD2=40;
                                if(turn==1){//Abierto
                                    write_button(2,'x');//Cerrar Turno
                                    write_button(2,'v');//ID Vendedor
                                }else{//Cerrado
                                    write_button(2,'y');//Abrir Turno
                                }
                            }
                        break;
                       
                        case 0x03:  //Canasta 
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                set_picture(2,33);
                                write_button(2,'S');    //Venta
                                bufferLCD2.salePerform='2';
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=20;x++){
                                        bufferLCD2.serialMarket[y][x]='0';
                                    }
                                    bufferLCD2.serialMarket[y][0]=20;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=20;x++){
                                        bufferLCD2.productMarket[y][x]='-';
                                    }
                                    bufferLCD2.productMarket[y][0]=20;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=3;x++){
                                        bufferLCD2.quantityMarket[y][x]='0';
                                        bufferLCD2.quantAvailableMark[y][x]='0';
                                    }
                                    bufferLCD2.quantityMarket[y][0]=3;
                                    bufferLCD2.quantAvailableMark[y][0]=3;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=8;x++){
                                        bufferLCD2.priceTotalMarket[y][x]='0';
                                    }
                                    bufferLCD2.priceTotalMarket[y][0]=8;
                                }
                                for(y=0;y<=2;y++){
                                    for(x=1;x<=8;x++){
                                        bufferLCD2.priceUnitMarket[y][x]='0';
                                    }
                                    bufferLCD2.priceUnitMarket[y][0]=8;
                                }
                                for(x=1;x<=9;x++){
                                    bufferLCD2.totalMarket[x]='0';
                                }
                                bufferLCD2.totalMarket[0]=9;
                                flowLCD2=5;
                            }else{
                                if(lockTurn==1){
                                    show_picture(2,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(2,67,3);
                                }else{
                                    show_picture(2,44,3);
                                    write_button(2,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x04:  //Consignaciones 
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                set_picture(2,37);
                                write_button(2,'0');    //Teclado antidad Canasta / Cantidad Consignacion
                                write_LCD(2,symbols[0],2,2,1,0x0000,'N');
                                numberKeys2=0;
                                if(decimalMoney>0){
                                    flagPoint2=0;
                                }else{
                                    flagPoint2=1;
                                }
                                flowLCD2=35;
                            }else{
                                if(lockTurn==1){
                                    show_picture(2,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(2,67,3);
                                }else{
                                    show_picture(2,44,3);
                                    write_button(2,'a');//Error abrir turno
                                }
                            }
                        break; 	 
                        
                        case 0x05:  //Mantenimiento 
                            if(turn==1 && lockTurn==0){//Abierto y desbloqueado
                                set_picture(2,61);
                                write_button(2,'u');//Mantenimiento
                                flowLCD2=38;
                            }else{
                                if(lockTurn==1){
                                    show_picture(2,66,3);
                                }else{
                                    show_picture(2,44,3);
                                    write_button(2,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x06:  //Copia de recibo 
                            if(turn==1 && lockTurn==0 && stateBeagleSoft==1){//Abierto y desbloqueado y comunicando
                                show_picture(2,55,3);
                                bufferLCD2.stateMux[1]=0x34;//Recibo de impresion
                                write_button(2,'W');//Espere 1 momento
                            }else{
                                if(lockTurn==1){
                                    show_picture(2,66,3);
                                }else if(stateBeagleSoft==0){
                                    show_picture(2,67,3);
                                }else{
                                    show_picture(2,44,3);
                                    write_button(2,'a');//Error abrir turno
                                }
                            }
                        break;
                        
                        case 0x07:  //Informacion
                            set_picture(2,32);
                            write_button(2,'I');
                            show_info(2,version);
                            flowLCD2=4;
                        break;
                        
                        case 0x0F:  //Atras
                            flowLCD2=1;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
             }	
        break;
            
        case 4: //Informacion
            show_time_date(2);
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){
                        case 0x0e:  //Configuraciones
                            show_picture(2,55,5);
                            write_button(2,'W');//Espere 1 momento
                            bufferLCD2.stateMux[1]=0x3A;//Solicitud de Configuraciones
                        break;
                        
                        case 0x0f:  //Atras
                            set_picture(2,30);
                            write_button(2,'M');    //Menu
                            flowLCD2=3;
                        break;
                    } 
                }
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 5: //Ventas
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){ 
                        case 0x08:  //Contado
                            bufferLCD2.saleType='1';
                            if(bufferLCD2.salePerform=='1'){//Combustible
                                if(screen[0]=='1'){
                                    set_picture(2,49);
                                    write_button(2,'V');
                                    flowLCD2=6;
                                }else{
                                    bufferLCD2.vehicleType=0;
                                    set_picture(2,35);
                                    write_button(2,'C');    //Contado
                                    flowLCD2=7;
                                }
                            }else if(bufferLCD2.salePerform=='2'){//Canasta
                                bufferLCD2.stateMux[0]=0;
                                set_picture(2,59);
                                write_button(2,'c');    //Canasta
                                show_market(2);
                                bufferLCD2.idType='0';
                                bufferLCD2.idSerial[0]=0;
                                bufferLCD2.stateMux[1]=0x39;//Entra a menu de canasta
                                flowLCD2=31;
                            }
                        break;
                       
                        case 0x09:  //Credito 
                            bufferLCD2.saleType='2';
                            if(bufferLCD2.salePerform=='1'){//Combustible
                                set_picture(2,37);
                                write_button(2,'4');    //Teclado Kilometraje
                                numberKeys2=0;
                                flagPoint2=1;
                                flowLCD2=20;
                            }else if(bufferLCD2.salePerform=='2'){//Canasta
                                bufferLCD2.mileageSale[0]=0;
                                bufferLCD2.productType=0;
                                set_picture(2,49);
                                write_button(2,'m');//Metodos ID
                                flowLCD2=21;
                            }
                        break;
                            
                        case 0x0A://Formas de pago
                            bufferLCD2.flagWayToPayMixed=0;
                            if(screen[1]=='1'){
                                set_picture(2,37);
                                write_button(2,'6');//Teclado Venta F. de pago
                                bufferLCD2.valueKeys[1]='0';
                                numberKeys2=1;
                                write_LCD(2,bufferLCD2.valueKeys[numberKeys2],2,numberKeys2+2,1,0x0000,'N');
                                flagPoint2=1;
                                flowLCD2=14;
                            }else{
                                bufferLCD2.selectedSale[0]=1;
                                bufferLCD2.selectedSale[1]='0';
                                set_picture(2,49);
                                write_button(2,'F');//Formas de pago
                                bufferLCD2.flagPayment=1;
                                write_button(2,bufferLCD2.flagPayment);//Nombres a Formas de pago
                                flowLCD2=15;
                            }
                        break;
                            
                        case 0x0B://Forma de pago Mixto
                            bufferLCD2.flagWayToPayMixed=1;
                            if(screen[1]=='1'){
                                set_picture(2,37);
                                write_button(2,'6');//Teclado Venta F. de pago
                                bufferLCD2.valueKeys[1]='0';
                                numberKeys2=1;
                                write_LCD(2,bufferLCD2.valueKeys[numberKeys2],2,numberKeys2+2,1,0x0000,'N');
                                flagPoint2=1;
                                flowLCD2=14;
                            }else{
                                bufferLCD2.selectedSale[0]=1;
                                bufferLCD2.selectedSale[1]='0';
                                set_picture(2,49);
                                write_button(2,'F');//Formas de pago
                                bufferLCD2.flagPayment=1;
                                write_button(2,bufferLCD2.flagPayment);//Nombres a Formas de pago
                                flowLCD2=15;
                            }
                        break;
                        
                        case 0x0F:  //Atras
                            set_picture(2,30);
                            write_button(2,'M');    //Menu
                            flowLCD2=3;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
        
        case 6: //Tipo de vehiculo
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    if(LCD2_rxBuffer[3]<=6){
                        bufferLCD2.vehicleType=LCD2_rxBuffer[3];
                        set_picture(2,35);
                        write_button(2,'C');    //Contado
                        flowLCD2=7;
                    }else if(LCD2_rxBuffer[3]==0x1A){//Atras
                        set_picture(2,33);
                        write_button(2,'S');    //Venta
                        flowLCD2=5;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 7: //Contado
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){ 
                        case 0x10:  //Dinero
                            set_picture(2,37);
                            write_button(2,'1');    //Teclado Dinero
                            write_LCD(2,symbols[0],2,2,1,0x0000,'N');
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='D';
                            numberKeys2=0;
                            if(decimalMoney>0){
                                flagPoint2=0;
                            }else{
                                flagPoint2=1;
                            }
                            flowLCD2=8;
                        break;
                       
                        case 0x11:  //Volumen 
                            set_picture(2,37);
                            write_button(2,'2');    //Teclado Volumen
                            write_LCD(2,symbols[1],2,2,1,0x0000,'N');
                            bufferLCD2.presetType[0]=1;
                            bufferLCD2.presetType[1]='V';
                            numberKeys2=0;
                            if(decimalVolume>0){
                                flagPoint2=0;
                            }else{
                                flagPoint2=1;
                            }
                            flowLCD2=8;
                        break;
                        
                        case 0x12:  //Full
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='F';
                            for(x=1;x<(digits-1);x++){
                                bufferLCD2.presetValue[0][x]='9';
                            }
                            bufferLCD2.presetValue[0][x]='0';
                            bufferLCD2.presetValue[0][x+1]='0';
                            bufferLCD2.presetValue[0][0]=digits;
                            for(x=0;x<=bufferLCD2.presetValue[0][0];x++){
                                bufferLCD2.presetValue[1][x]=bufferLCD2.presetValue[0][x];
                            }
                            if(productNumber==1){
                                set_picture(2,43);
                                write_button(2,'U');
                                bufferLCD2.productType=1;
                                flowLCD2=10; 
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');
                                flowLCD2=9;
                            }
                        break;
                       
                        case 0x13:  //P1
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='1';
                            for(x=0;x<=presetFast[0][0];x++){
                                bufferLCD2.presetValue[0][x]=presetFast[0][x];
                                bufferLCD2.presetValue[1][x]=presetFast[0][x];
                            }
                            if(productNumber==1){
                                set_picture(2,43);
                                write_button(2,'U');//Suba la manija
                                bufferLCD2.productType=1;
                                flowLCD2=10; 
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');//Escoja producto
                                flowLCD2=9;
                            }
                        break;
                        
                        case 0x14:  //P2
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='2';
                            for(x=0;x<=presetFast[1][0];x++){
                                bufferLCD2.presetValue[0][x]=presetFast[1][x];
                                bufferLCD2.presetValue[1][x]=presetFast[1][x];
                            }
                            if(productNumber==1){
                                set_picture(2,43);
                                write_button(2,'U');//Suba la manija
                                bufferLCD2.productType=1;
                                flowLCD2=10; 
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');//Escoja producto
                                flowLCD2=9;
                            }
                        break;
                       
                        case 0x15:  //P3
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='3';
                            for(x=0;x<=presetFast[2][0];x++){
                                bufferLCD2.presetValue[0][x]=presetFast[2][x];
                                bufferLCD2.presetValue[1][x]=presetFast[2][x];
                            }
                            if(productNumber==1){
                                set_picture(2,43);
                                write_button(2,'U');//Suba la manija
                                bufferLCD2.productType=1;
                                flowLCD2=10; 
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');//Escoja producto
                                flowLCD2=9;
                            }
                        break;
                        
                        case 0x0F:  //Atras
                            if(screen[0]=='1'){
                                set_picture(2,49);
                                write_button(2,'V');    //Tipo de vehiculo
                                flowLCD2=6;
                            }else{
                                set_picture(2,33);
                                write_button(2,'S');    //Venta
                                flowLCD2=5;
                            }
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 8: //Teclado preset
            switch (alphanumeric_keyboard(2,digits,0)){
                case 0: //Cancelar
                    set_picture(2,35);
                    write_button(2,'C');
                    flowLCD2=7;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.presetValue[0][x]=bufferLCD2.valueKeys[x];
                        bufferLCD2.presetValue[1][x]=bufferLCD2.valueKeys[x];
                    }
                    temporal[0]=1;
                    temporal[1]='0';
                    if(higher(bufferLCD2.presetValue[1],temporal)==1){
                        if(productNumber==1){
                            set_picture(2,43);
                            write_button(2,'U');    //Suba la manija
                            bufferLCD2.productType=1;
                            flowLCD2=10;
                        }else{
                            set_picture(2,39+(productNumber-2));
                            write_button(2,'P');    //Escoja producto
                            flowLCD2=9; 
                        }
                    }
                break;
            }
        break;
            
        case 9: //Escoja producto
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    if(LCD2_rxBuffer[3]<=0x04){
                            bufferLCD2.productType=(LCD2_rxBuffer[3]&0x07);
                            set_picture(2,43);
                            write_button(2,'U');    //Suba la manija
                            flowLCD2=10;
                    }else if(LCD2_rxBuffer[3]==0x0F){
                        set_picture(2,35);
                        write_button(2,'C');//Contado
                        flowLCD2=7;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 10: //Suba la manija
            if(turn==0){//Turno cerrado
                flowLCD2=1;
                LCD2_ClearRxBuffer();
                break;
            }
            if(LCD2_GetRxBufferSize()==8){
                flowLCD2=1;
                LCD2_ClearRxBuffer();
                break;
            }
            if(get_state(side.b.dir)==0x07){
                CyDelay(100);
                if(get_handle(side.b.dir)==bufferLCD2.productType){
                    if(read_date()==1 && read_time()==1){
                        for(x=0;x<=2;x++){
                            bufferLCD2.timeLiftHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                            bufferLCD2.timeLiftHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                            bufferLCD2.dateLiftHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                            bufferLCD2.dateLiftHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                        }
                    }else{
                        for(x=0;x<6;x++){
                            bufferLCD2.timeLiftHandle[x]='0';
                            bufferLCD2.dateLiftHandle[x]='0';
                        }
                    }
                    set_picture(2,43);
                    write_button(2,'W');//Espere 1 momento
                    flowLCD2=11;
                    LCD2_ClearRxBuffer();
                }else{
                    bufferLCD2.stateMux[1]=0x16;//Espera
                    show_picture(2,44,3);
                    write_button(2,'E');//Error de sistema
                }
            }
        break;
            
        case 11: //Programar Equipo
            if(turn==0){//Turno cerrado
                flowLCD2=1;
                LCD2_ClearRxBuffer();
                break;
            }
            if(get_totals(side.b.dir)!=0){
                for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                    bufferLCD2.totalVolumePrevious[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                    bufferLCD2.totalMoneyPrevious[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                    bufferLCD2.totalPPUPrevious[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                }
                if(compare_ppu(bufferLCD2.totalPPUPrevious,bufferLCD2.productType,side.b.dir)==0){
                    if(price_change(side.b.dir,bufferLCD2.productType,side.b.ppuAuthorized[bufferLCD2.productType-1])==0){
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                        break;
                    }
                }
                if(preset_data(side.b.dir,bufferLCD2.productType,bufferLCD2.presetValue[0],bufferLCD2.presetType[0]&0x03)==0){
                    show_picture(2,44,3);
                    bufferLCD2.stateMux[1]=0x10;//Error
                    write_button(2,'E');//Error de sistema
                    break;
                }
                if(get_handle(side.b.dir)!=0){
                    Pump_AL_PutChar(0x10|side.b.dir);//Autoriza el surtidor
                    set_picture(2,48);
                    bufferLCD2.licenceSale[0]=0;
                    bufferLCD2.mileageSale[0]=0;
                    bufferLCD2.identySale[0]=0;
                    bufferLCD2.flagEndSale=0;
                    bufferLCD2.idType='0';
                    bufferLCD2.idSerial[0]=0;
                    write_button(2,'t');//Tanqueando con Datos
                    flowLCD2=12;
                    LCD2_ClearRxBuffer();
                }else{
                    show_picture(2,44,3);
                    bufferLCD2.stateMux[1]=0x10;//Error
                    write_button(2,'E');//Error de sistema
                }
            }else{
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x10;//Error
                write_button(2,'E');//Error de sistema
            }
        break;
            
        case 12: //Tanqueando
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){ 
                        case 0x01:  //Placa
                            set_picture(2,46);
                            write_button(2,'3');//Teclado Placa
                            numberKeys2=0;
                            bufferLCD2.flagKeyboard=1;
                            bufferLCD2.flagEndSale=0;
                            countAnimation2=0;
                            flowLCD2=13;
                        break;
                       
                        case 0x02:  //Kilometraje
                            set_picture(2,37);
                            write_button(2,'4');//Teclado Kilometraje
                            numberKeys2=0;
                            flagPoint2=0;
                            bufferLCD2.flagKeyboard=2;
                            bufferLCD2.flagEndSale=0;
                            countAnimation2=0;
                            flowLCD2=13;
                        break;
                        
                        case 0x03:  //CC/NIT
                            set_picture(2,37);
                            write_button(2,'5');//Teclado Nit/CC
                            numberKeys2=0;
                            flagPoint2=0;
                            bufferLCD2.flagKeyboard=3;
                            bufferLCD2.flagEndSale=0;
                            countAnimation2=0;
                            flowLCD2=13;
                        break;
                    }
                }
                CyDelay(10);         
                LCD2_ClearRxBuffer();
                break;
            }
            switch(get_state(side.b.dir)){
                case 0x09://Surtiendo
                    bufferLCD2.stateMux[1]=0x19;//Surtiendo
                break;
                
                case 0x0A://Reporte de venta
                    set_picture(2,43);
                    write_button(2,'W');//Espere 1 momento
                    if(get_sale(side.b.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD2.timeDownHandle[x]='0';
                                bufferLCD2.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.b.dir)!=0){
                            for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                                bufferLCD2.totalVolumeAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                                bufferLCD2.totalMoneyAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                                bufferLCD2.totalPPUAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                            }
                        }else{
                            bufferLCD2.totalVolumeAfter[0]=1;
                            bufferLCD2.totalMoneyAfter[0]=1;
                            bufferLCD2.totalPPUAfter[0]=1;
                            bufferLCD2.totalVolumeAfter[1]='0';
                            bufferLCD2.totalMoneyAfter[1]='0';
                            bufferLCD2.totalPPUAfter[1]='0';
                        }
                        bufferLCD2.flagEndSale=1;
                        if(bufferLCD2.licenceSale[0]==0){
                            set_picture(2,46);
                            write_button(2,'3');//Teclado Placa
                            numberKeys2=0;
                            bufferLCD2.flagKeyboard=1;
                            isr_2_StartEx(timerAnimation2); 
                            Waitable_2_Start();
                            countAnimation2=0;
                            flowLCD2=13;
                        }else{
                            show_picture(2,54,3);
                            bufferLCD2.stateMux[1]=0x1A;//Fin venta
                            write_button(2,'G');//Gracias
                        }
                    }else{
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                    }
                break;
                
                case 0x0B://Reporte de venta
                    set_picture(2,43);
                    write_button(2,'W');//Espere 1 momento
                    if(get_sale(side.b.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD2.timeDownHandle[x]='0';
                                bufferLCD2.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.b.dir)!=0){
                            for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                                bufferLCD2.totalVolumeAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                                bufferLCD2.totalMoneyAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                                bufferLCD2.totalPPUAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                            }
                        }else{
                            bufferLCD2.totalVolumeAfter[0]=1;
                            bufferLCD2.totalMoneyAfter[0]=1;
                            bufferLCD2.totalPPUAfter[0]=1;
                            bufferLCD2.totalVolumeAfter[1]='0';
                            bufferLCD2.totalMoneyAfter[1]='0';
                            bufferLCD2.totalPPUAfter[1]='0';
                        }
                        bufferLCD2.flagEndSale=1;
                        if(bufferLCD2.licenceSale[0]==0){
                            set_picture(2,46);
                            write_button(2,'3');//Teclado Placa
                            numberKeys2=0;
                            bufferLCD2.flagKeyboard=1;
                            isr_2_StartEx(timerAnimation2); 
                            Waitable_2_Start();
                            countAnimation2=0;
                            flowLCD2=13;
                        }else{
                            show_picture(2,54,3);
                            bufferLCD2.stateMux[1]=0x1A;//Fin venta
                            write_button(2,'G');//Gracias
                        }
                    }else{
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                    }
                break;
                
                case 0x06://Espera
                    if(get_totals(side.b.dir)!=0){
                        for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                            bufferLCD2.totalVolumeAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                            bufferLCD2.totalMoneyAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                            bufferLCD2.totalPPUAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                        }
                        if(subtraction(bufferLCD2.totalVolumeAfter,bufferLCD2.totalVolumePrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.b.volumeSale[x]=residue[x];
            				}	
                            side.b.volumeSale[x]='0';
                            side.b.volumeSale[0]++;
                        }else{
                            bufferLCD2.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD2=1;
                            break;
                        }
                        if(subtraction(bufferLCD2.totalMoneyAfter,bufferLCD2.totalMoneyPrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.b.moneySale[x]=residue[x];
            				}
                        }else{
                            bufferLCD2.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD2=1;
                            break;
                        }
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD2.timeDownHandle[x]='0';
                                bufferLCD2.dateDownHandle[x]='0';
                            }
                        }
                        side.b.productSale=(bufferLCD2.productType&0x0F)+0x30;
        				for(x=0;x<=bufferLCD2.totalPPUAfter[0];x++){
        					side.b.ppuSale[x]=bufferLCD2.totalPPUAfter[x];
        				}
                        bufferLCD2.flagEndSale=1;
                        if(bufferLCD2.licenceSale[0]==0){
                            set_picture(2,46);
                            write_button(2,'3');//Teclado Placa
                            numberKeys2=0;
                            bufferLCD2.flagKeyboard=1;
                            isr_2_StartEx(timerAnimation2); 
                            Waitable_2_Start();
                            countAnimation2=0;
                            flowLCD2=13;
                        }else{
                            show_picture(2,54,3);
                            bufferLCD2.stateMux[1]=0x1A;//Fin venta
                            write_button(2,'G');//Gracias
                        }
                    }else{
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                    }
                break;
            }
        break;
            
        case 13: //Teclado Placa
            get_state(side.b.dir);
            switch (alphanumeric_keyboard(2,10,0)){
                case 0: //Cancelar
                    switch(bufferLCD2.flagKeyboard){
                        case 1://Placa
                            for(x=0;x<=10;x++){
                                bufferLCD2.licenceSale[x]=0;
                            }
                        break;
                        
                        case 2://Kilometraje
                            for(x=0;x<=10;x++){
                                bufferLCD2.mileageSale[x]=0;
                            }
                        break;
                        
                        case 3://CC/NIT
                            for(x=0;x<=10;x++){
                                bufferLCD2.identySale[x]=0;
                            }
                        break;
                    }
                    if(bufferLCD2.flagEndSale==1){
                        isr_2_Stop(); 
                        Waitable_2_Stop();
                        show_picture(2,54,3);
                        bufferLCD2.stateMux[1]=0x1A;//Fin venta
                        write_button(2,'G');//Gracias
                    }else{
                        set_picture(2,48);
                        write_button(2,'t');//Tanqueando con Datos
                        flowLCD2=12;
                    }
                    LCD2_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    switch(bufferLCD2.flagKeyboard){
                        case 1://Placa
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                bufferLCD2.licenceSale[x]=bufferLCD2.valueKeys[x];
                            }
                        break;
                        
                        case 2://Kilometraje
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                bufferLCD2.mileageSale[x]=bufferLCD2.valueKeys[x];
                            }
                        break;
                        
                        case 3://CC/NIT
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                bufferLCD2.identySale[x]=bufferLCD2.valueKeys[x];
                            }
                        break;
                    }
                    if(bufferLCD2.flagEndSale==1){
                        isr_2_Stop(); 
                        Waitable_2_Stop();
                        show_picture(2,54,3);
                        bufferLCD2.stateMux[1]=0x1A;//Fin venta
                        write_button(2,'G');//Gracias
                    }else{
                        set_picture(2,48);
                        write_button(2,'t');//Tanqueando con Datos
                        flowLCD2=12;
                    }
                    LCD2_ClearRxBuffer();
                break;
            }
            if(countAnimation2>100){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,54,3);
                bufferLCD2.stateMux[1]=0x1A;//Fin venta
                write_button(2,'G');//Gracias
            }
        break;
            
        case 14://Teclado Venta F. de pago 
            switch (alphanumeric_keyboard(2,8,0)){
                case 0: //Cancelar
                    set_picture(2,33);
                    write_button(2,'S');    //Venta
                    flowLCD2=5;
                    LCD2_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.selectedSale[x]=bufferLCD2.valueKeys[x];
                    }
                    set_picture(2,49);
                    write_button(2,'F');//Formas de pago
                    bufferLCD2.flagPayment=1;
                    write_button(2,bufferLCD2.flagPayment);//Nombres a Formas de pago
                    flowLCD2=15;
                    LCD2_ClearRxBuffer();
                break;
            }
        break;  
            
        case 15: //Formas de pago
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    if(LCD2_rxBuffer[3]<=24){
                        bufferLCD2.wayToPay=LCD2_rxBuffer[3];
                        bufferLCD2.stateMux[1]=0x21;
                        set_picture(2,55);
                        write_button(2,'W');//Espere 1 momento
                        isr_2_StartEx(timerAnimation2); 
                        Waitable_2_Start();
                        countAnimation2=0;
                        bufferLCD2.authorizationFlag=0;
                        flowLCD2=16;
                    }else if(LCD2_rxBuffer[3]==0x1A){
                        if(bufferLCD2.flagPayment==1){
                            if(screen[1]=='1'){
                                set_picture(2,37);
                                write_button(2,'6');//Teclado Venta F. de pago
                                bufferLCD2.valueKeys[1]='0';
                                numberKeys2=1;
                                write_LCD(2,bufferLCD2.valueKeys[numberKeys2],2,numberKeys2+2,1,0x0000,'N');
                                write_button(2,'6');//Teclado Venta F. de pago
                                flagPoint2=1;
                                flowLCD2=14;
                            }else{
                                set_picture(2,33);
                                write_button(2,'S');    //Venta
                                flowLCD2=5;
                            }
                        }else{
                            set_picture(2,47+bufferLCD2.flagPayment);
                            write_button(2,'F');//Formas de pago
                            bufferLCD2.flagPayment--;
                            write_button(2,bufferLCD2.flagPayment);//Nombres a Formas de pago
                        }                        
                    }else if(LCD2_rxBuffer[3]==0x1B){
                        set_picture(2,49+bufferLCD2.flagPayment);
                        write_button(2,'F');//Formas de pago
                        bufferLCD2.flagPayment++;
                        write_button(2,bufferLCD2.flagPayment);//Nombres a Formas de pago
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 16: //Esperando respuesta a peticion
            if(bufferLCD2.authorizationFlag!=0){
                isr_2_Stop(); 
                Waitable_2_Stop();
                if(bufferLCD2.authorizationFlag==1){
                    if(bufferLCD2.flagWayToPayMixed==1){
                        bufferLCD2.stateMux[1]=0x16;//Espera
                        show_picture(2,54,3);
                        write_button(2,'G');//Gracias
                    }else{
                        set_picture(2,37);
                        write_button(2,'7');//Teclado Dinero Venta F. de pago
                        write_LCD(2,symbols[0],2,2,1,0x0000,'N');
                        y=0;
                        for(x=1;x<=bufferLCD2.moneySelectedSale[0];x++){
                            if(bufferLCD2.moneySelectedSale[x]!='0'){
                                bufferLCD2.moneySelectedSale[0]-=(x-1);
                                for(y=1;y<=bufferLCD2.moneySelectedSale[0];y++){
                                    bufferLCD2.moneySelectedSale[y]=bufferLCD2.moneySelectedSale[x];
                                    bufferLCD2.valueKeys[y]=bufferLCD2.moneySelectedSale[x];
                                    x++;
                                }
                                break;
                            }
                        }
                        if(decimalMoney>0){
                            flagPoint2=0;
                        }else{
                            flagPoint2=1;
                        }
                         for(x=1;x<=bufferLCD2.moneySelectedSale[0];x++){
                            write_LCD(2,bufferLCD2.moneySelectedSale[x],2,x+2,1,0x0000,'N');
                            if(bufferLCD2.moneySelectedSale[x]==','){
                                flagPoint2=1;
                            }
                        }
                        numberKeys2=bufferLCD2.moneySelectedSale[0];
                        bufferLCD2.stateMux[1]=0x23;
                        flowLCD2=17;
                    }
                }else{
                    isr_2_Stop(); 
                    Waitable_2_Stop();
                    show_picture(2,44,3);
                    bufferLCD2.stateMux[1]=0x16;//Espera
                    write_button(2,'A');//Error de sistema
                }
                break;
            }
            if(countAnimation2>200){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'E');//Error de sistema
            }
        break;
            
        case 17://Teclado Discriminar dinero
            switch (alphanumeric_keyboard(2,bufferLCD2.moneySelectedSale[0],0)){
                case 0: //Cancelar
                    show_picture(2,44,3);
                    bufferLCD2.stateMux[1]=0x16;//Espera
                    write_button(2,'A');//Error de sistema
                    LCD2_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    temporal[0]=1;
                    temporal[1]='0';
                    if(higher(bufferLCD2.valueKeys,temporal)==1){
                        if(higher(bufferLCD2.moneySelectedSale,bufferLCD2.valueKeys)!=2){
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                bufferLCD2.moneySelectedSale[x]=bufferLCD2.valueKeys[x];
                            }
                            numberKeys2=0;
                            flagPoint2=0;
                            flowLCD2=18;
                            if(bufferLCD2.keyboardWayToPay==0){
                                bufferLCD2.serialSelectedSale[0]=1;
                                bufferLCD2.serialSelectedSale[1]='0';
                                bufferLCD2.stateMux[1]=0x24;
                                set_picture(2,55);
                                write_button(2,'W');//Espere 1 momento
                                isr_2_StartEx(timerAnimation2); 
                                Waitable_2_Start();
                                countAnimation2=0;
                                bufferLCD2.authorizationFlag=0;
                                flowLCD2=19;
                            }else{
                                set_picture(2,46);
                                write_button(2,'8');//Teclado Serial
                            }
                        }
                    }
                    LCD2_ClearRxBuffer();
                break;
            }
        break;
            
        case 18://Teclado Serial
            switch (alphanumeric_keyboard(2,20,0)){
                case 0: //Cancelar
                    show_picture(2,44,3);
                    bufferLCD2.stateMux[1]=0x16;//Espera
                    write_button(2,'A');//Error de sistema
                    LCD2_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.serialSelectedSale[x]=bufferLCD2.valueKeys[x];
                    }
                    bufferLCD2.stateMux[1]=0x24;
                    set_picture(2,55);
                    write_button(2,'W');//Espere 1 momento
                    isr_2_StartEx(timerAnimation2); 
                    Waitable_2_Start();
                    countAnimation2=0;
                    bufferLCD2.authorizationFlag=0;
                    flowLCD2=19;
                    LCD2_ClearRxBuffer();
                break;
            }
        break;
            
        case 19: //Esperando respuesta a envio forma de pago
            if(bufferLCD2.authorizationFlag!=0){
                isr_2_Stop(); 
                Waitable_2_Stop();
                bufferLCD2.stateMux[1]=0x16;//Espera
                if(bufferLCD2.authorizationFlag==1){
                    show_picture(2,54,3);
                    write_button(2,'G');//Gracias
                }else{
                    show_picture(2,44,3);
                    write_button(2,'A');//Error de sistema
                }
                break;
            }
            if(countAnimation2>200){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'E');//Error de sistema
            }
        break;
            
        case 20://Teclado Kilometraje Credito
            switch (alphanumeric_keyboard(2,10,0)){
                case 0: //Cancelar
                    set_picture(2,33);
                    write_button(2,'S');    //Venta
                    flowLCD2=5;
                    LCD2_ClearRxBuffer();
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.mileageSale[x]=bufferLCD2.valueKeys[x];
                    }
                    set_picture(2,49);
                    write_button(2,'m');//Metodos ID
                    flowLCD2=21;
                    LCD2_ClearRxBuffer();
                break;
            }
        break;
            
        case 21://Tipo de Metodos ID
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){ 
                        case 0x01:  //Ibutton
                            bufferLCD2.idType='1';
                            set_picture(2,56);
                            write_button(2,'i');//Metodo 1
                            flowLCD2=22;
                        break;
                       
                        case 0x02:  //Tag
                            bufferLCD2.idType='2';
                            set_picture(2,56);
                            write_button(2,'j');//Metodo 2
                            flowLCD2=22;
                            Tag_ClearRxBuffer();
                            Tag_ClearTxBuffer();
                            Tag_PutChar('O');
                            Tag_PutChar('K');
                            Tag_PutChar(0x02);
                            CyDelay(100);
                        break;
                        
                        case 0x03:  //Numerico
                            bufferLCD2.idType='3';
                            set_picture(2,37);
                            write_button(2,'k');//Metodo 3
                            numberKeys2=0;
                            flagPoint2=1;
                            flowLCD2=22;
                        break;
                        
                        case 0x04:  //Alfanumerico
                            bufferLCD2.idType='4';
                            set_picture(2,46);
                            write_button(2,'l');//Metodo 4
                            numberKeys2=0;
                            flagPoint2=0;
                            flowLCD2=22;
                        break;
                        
                        case 0x05:  //Codigo de barras
                            bufferLCD2.idType='5';
                            set_picture(2,56);
                            write_button(2,'n');//Metodo 5
                            flowLCD2=22;
                            Code_Bar_ClearRxBuffer();
                        break;
                        
                        case 0x06:  //
                            bufferLCD2.idType='6';
                            write_button(2,'m');//Metodos ID
                        break;
                            
                        case 0x1A:  //Atras
                            if(bufferLCD2.salePerform=='1'){//Combustible
                                set_picture(2,37);
                                write_button(2,'4');    //Teclado Kilometraje
                                numberKeys2=0;
                                flagPoint2=1;
                                flowLCD2=20;
                            }else if(bufferLCD2.salePerform=='2'){//Canasta
                                set_picture(2,33);
                                write_button(2,'S');    //Venta
                                flowLCD2=5;
                            }
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 22://Identificacion de Metodos ID
            switch(bufferLCD2.idType){
                case '1':  //Ibutton
                    if(LCD2_GetRxBufferSize()==8){
                        set_picture(2,49);
                        write_button(2,'m');//Metodos ID
                        flowLCD2=21;
                        LCD2_ClearRxBuffer();
                        break;
                    }
                    if(touch_present(2)==1){
        				if(touch_write(2,0x33)){
        					for(x=1;x<=8;x++){
        						temporal[x]=touch_read_byte(2);
        					}
        					y=0;
        					for(x=1;x<8;x++){
                                y=crc_check(y,temporal[x]);
                            }
        					if(y==temporal[8]){
        						bufferLCD2.idSerial[0]=16;
        						y=16;
        						for(x=1;x<=8;x++){
        							if((temporal[x]&0x0F)>=10){
        								bufferLCD2.idSerial[y]=(temporal[x]&0x0F)+55;
        							}else{
        								bufferLCD2.idSerial[y]=(temporal[x]&0x0F)+48;				
        							}
                                    y--;
        							if(((temporal[x]>>4)&0x0F)>=10){
        								bufferLCD2.idSerial[y]=((temporal[x]>>4)&0x0F)+55;
        							}else{
        								bufferLCD2.idSerial[y]=((temporal[x]>>4)&0x0F)+48;				
        							}
                                    y--;
        						}
                                if(productNumber==1 || bufferLCD2.salePerform=='2'){
                                    bufferLCD2.productType=1;
                                    bufferLCD2.stateMux[1]=0x26;
                                    set_picture(2,55);
                                    write_button(2,'W');//Espere 1 momento
                                    isr_2_StartEx(timerAnimation2); 
                                    Waitable_2_Start();
                                    countAnimation2=0;
                                    bufferLCD2.authorizationFlag=0;
                                    flowLCD2=24;
                                }else{
                                    set_picture(2,39+(productNumber-2));
                                    write_button(2,'P');    //Escoja producto
                                    flowLCD2=23; 
                                }
                                LCD2_ClearRxBuffer();
        					}
        				}
        			}
                break;
                
                case '2':  //Tag
                    if(LCD2_GetRxBufferSize()==8){
                        set_picture(2,49);
                        write_button(2,'m');//Metodos ID
                        flowLCD2=21;
                        LCD2_ClearRxBuffer();
                        break;
                    }
                    for(x=0;x<=29;x++){
                        temporal[x]=0x00;
                    }
                    if(serial_codetag(2)==1){
                        for(x=0;x<=temporal[0];x++){
                            bufferLCD2.idSerial[x]=temporal[x];
                        }
                        bufferLCD2.idSerial[0]=16;
						y=16;
						for(x=1;x<=8;x++){
							if((temporal[x]&0x0F)>=10){
								bufferLCD2.idSerial[y]=(temporal[x]&0x0F)+55;
							}else{
								bufferLCD2.idSerial[y]=(temporal[x]&0x0F)+48;				
							}
                            y--;
							if(((temporal[x]>>4)&0x0F)>=10){
								bufferLCD2.idSerial[y]=((temporal[x]>>4)&0x0F)+55;
							}else{
								bufferLCD2.idSerial[y]=((temporal[x]>>4)&0x0F)+48;				
							}
                            y--;
						}
                        if(productNumber==1 || bufferLCD2.salePerform=='2'){
                            bufferLCD2.productType=1;
                            bufferLCD2.stateMux[1]=0x26;
                            set_picture(2,55);
                            write_button(2,'W');//Espere 1 momento
                            isr_2_StartEx(timerAnimation2); 
                            Waitable_2_Start();
                            countAnimation2=0;
                            bufferLCD2.authorizationFlag=0;
                            flowLCD2=24;
                        }else{
                            set_picture(2,39+(productNumber-2));
                            write_button(2,'P');    //Escoja producto
                            flowLCD2=23; 
                        }
                        LCD2_ClearRxBuffer();
                    }
                break;
                
                case '3':  //Numerico
                    switch (alphanumeric_keyboard(2,10,0)){
                        case 0: //Cancelar
                            set_picture(2,49);
                            write_button(2,'m');//Metodos ID
                            flowLCD2=21;
                            LCD2_ClearRxBuffer();
                        break;
                            
                        case 1: //Enter
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                bufferLCD2.idSerial[x]=bufferLCD2.valueKeys[x];
                            }
                            if(productNumber==1 || bufferLCD2.salePerform=='2'){
                                bufferLCD2.productType=1;
                                bufferLCD2.stateMux[1]=0x26;
                                set_picture(2,55);
                                write_button(2,'W');//Espere 1 momento
                                isr_2_StartEx(timerAnimation2); 
                                Waitable_2_Start();
                                countAnimation2=0;
                                bufferLCD2.authorizationFlag=0;
                                flowLCD2=24;
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');    //Escoja producto
                                flowLCD2=23; 
                            }
                            LCD2_ClearRxBuffer();
                        break;
                    }
                break;
                
                case '4':   //Alfanumerico
                    switch (alphanumeric_keyboard(2,20,0)){
                        case 0: //Cancelar
                            set_picture(2,49);
                            write_button(2,'m');//Metodos ID
                            flowLCD2=21;
                            LCD2_ClearRxBuffer();
                        break;
                            
                        case 1: //Enter
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                bufferLCD2.idSerial[x]=bufferLCD2.valueKeys[x];
                            }
                            if(productNumber==1 || bufferLCD2.salePerform=='2'){
                                bufferLCD2.productType=1;
                                bufferLCD2.stateMux[1]=0x26;
                                set_picture(2,55);
                                write_button(2,'W');//Espere 1 momento
                                isr_2_StartEx(timerAnimation2); 
                                Waitable_2_Start();
                                countAnimation2=0;
                                bufferLCD2.authorizationFlag=0;
                                flowLCD2=24;
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');    //Escoja producto
                                flowLCD2=23; 
                            }
                            LCD2_ClearRxBuffer();
                        break;
                    }
                break;
                
                case '5':   //Codigo de barras
                    if(LCD2_GetRxBufferSize()==8){
                        set_picture(2,49);
                        write_button(2,'m');//Metodos ID
                        flowLCD2=21;
                        LCD2_ClearRxBuffer();
                        break;
                    }
                    if(serial_codebar()==1){
                        for(x=0;x<=temporal[0];x++){
                            bufferLCD2.idSerial[x]=temporal[x];
                        }
                        if(productNumber==1 || bufferLCD2.salePerform=='2'){
                            bufferLCD2.productType=1;
                            bufferLCD2.stateMux[1]=0x26;
                            set_picture(2,55);
                            write_button(2,'W');//Espere 1 momento
                            isr_2_StartEx(timerAnimation2); 
                            Waitable_2_Start();
                            countAnimation2=0;
                            bufferLCD2.authorizationFlag=0;
                            flowLCD2=24;
                        }else{
                            set_picture(2,39+(productNumber-2));
                            write_button(2,'P');    //Escoja producto
                            flowLCD2=23; 
                        }
                        LCD2_ClearRxBuffer();
                    }
                break;
                
                case '6':   //
                    /*No establecido*/
                break;
            }
        break;
        
        case 23: //Escoja producto venta Credito
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    if(LCD2_rxBuffer[3]<=0x04){
                        bufferLCD2.productType=(LCD2_rxBuffer[3]&0x07);
                        bufferLCD2.stateMux[1]=0x26;
                        set_picture(2,55);
                        write_button(2,'W');//Espere 1 momento
                        isr_2_StartEx(timerAnimation2); 
                        Waitable_2_Start();
                        countAnimation2=0;
                        bufferLCD2.authorizationFlag=0;
                        flowLCD2=24;
                    }else if(LCD2_rxBuffer[3]==0x0F){
                        if(bufferLCD2.salePerform=='3'){
                            set_picture(2,61);
                            write_button(2,'u');//Mantenimiento
                            flowLCD2=38;
                        }else{
                            set_picture(2,49);
                            write_button(2,'m');//Metodos ID
                            flowLCD2=21;
                        }
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 24://Esperando respuesta a envio de identificacion credito
            if(bufferLCD2.authorizationFlag!=0){
                if(bufferLCD2.authorizationFlag==1){
                    if(bufferLCD2.salePerform=='1' || bufferLCD2.salePerform=='3'){//Combustible o Calibracion
                        for(x=1;x<=digits;x++){
                            bufferLCD2.volumeQuota[x]=bufferLCD2.volumeQuota[x+(bufferLCD2.volumeQuota[0]-digits)];
                            bufferLCD2.moneyQuota[x]=bufferLCD2.moneyQuota[x+(bufferLCD2.moneyQuota[0]-digits)];
                        }
                        bufferLCD2.volumeQuota[0]=digits;
                        bufferLCD2.moneyQuota[0]=digits;
                    } 
                    bufferLCD2.stateMux[1]=0x28;//Espera transaccion Credito
                    bufferLCD2.stateMux[0]=0x28;//Bandera habilitada al momento de identicar un producto en canastilla
                    countAnimation2=0;
                    flowLCD2=25;
                }else{
                    isr_2_Stop(); 
                    Waitable_2_Stop();
                    bufferLCD2.stateMux[1]=0x16;//Espera
                    flowLCD2=37;
                }
                set_picture(2,57);
                write_button(2,'o');//Continuar
                show_message(2,bufferLCD2.message);
                LCD2_ClearRxBuffer();
                break;
            }
            if(countAnimation2>200){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'E');//Error de sistema
            } 
        break;
            
        case 25://Mensaje autorizacion venta credito combustible
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){ 
                        case 0x0E:  //Continuar
                            if(bufferLCD2.salePerform=='1' || bufferLCD2.salePerform=='3'){//Combustible o calibracion
                                set_picture(2,35);
                                write_button(2,'C');    //Contado
                                flowLCD2=26;
                            }else if(bufferLCD2.salePerform=='2'){//Canasta
                                isr_2_Stop(); 
                                Waitable_2_Stop();
                                set_picture(2,59);
                                write_button(2,'c');    //Canasta
                                show_market(2);
                                flowLCD2=31;
                            } 
                        break;
                            
                        case 0x0F:  //Atras
                            isr_2_Stop(); 
                            Waitable_2_Stop();
                            show_picture(2,44,3);
                            write_button(2,'A');//Accion anulada
                            bufferLCD2.stateMux[1]=0x16;//Espera
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
            if(countAnimation2>240){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'A');//Accion anulada
            }
        break;
            
        case 26: //Preset Credito
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){ 
                        case 0x10:  //Dinero
                            set_picture(2,37);
                            write_button(2,'1');    //Teclado Dinero
                            write_LCD(2,symbols[0],2,2,1,0x0000,'N');
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='D';
                            numberKeys2=0;
                            if(decimalMoney>0){
                                flagPoint2=0;
                            }else{
                                flagPoint2=1;
                            }
                            flowLCD2=27;
                        break;
                       
                        case 0x11:  //Volumen 
                            set_picture(2,37);
                            write_button(2,'2');    //Teclado Volumen
                            write_LCD(2,symbols[1],2,2,1,0x0000,'N');
                            bufferLCD2.presetType[0]=1;
                            bufferLCD2.presetType[1]='V';
                            numberKeys2=0;
                            if(decimalVolume>0){
                                flagPoint2=0;
                            }else{
                                flagPoint2=1;
                            }
                            flowLCD2=27;
                        break;
                        
                        case 0x12:  //Full
                            bufferLCD2.presetType[0]=2;
                            bufferLCD2.presetType[1]='F';
                            for(x=0;x<=digits;x++){
                                bufferLCD2.presetValue[0][x]=bufferLCD2.moneyQuota[x];
                                bufferLCD2.presetValue[1][x]=bufferLCD2.moneyQuota[x];
                            }
                            set_picture(2,43);
                            write_button(2,'U');//Suba la manija
                            flowLCD2=28;
                        break;
                       
                        case 0x13:  //P1
                            if(higher(presetFast[0],bufferLCD2.moneyQuota)!=1){
                                bufferLCD2.presetType[0]=2;
                                bufferLCD2.presetType[1]='1';
                                for(x=0;x<=presetFast[0][0];x++){
                                    bufferLCD2.presetValue[0][x]=presetFast[0][x];
                                    bufferLCD2.presetValue[1][x]=presetFast[0][x];
                                }
                                set_picture(2,43);
                                write_button(2,'U');//Suba la manija
                                flowLCD2=28;
                            }else{
                                flowLCD2=25;
                                set_picture(2,57);
                                show_message(2,bufferLCD2.message);
                                write_button(2,'o');//Continuar
                            }
                        break;
                        
                        case 0x14:  //P2
                            if(higher(presetFast[1],bufferLCD2.moneyQuota)!=1){
                                bufferLCD2.presetType[0]=2;
                                bufferLCD2.presetType[1]='2';
                                for(x=0;x<=presetFast[1][0];x++){
                                    bufferLCD2.presetValue[0][x]=presetFast[1][x];
                                    bufferLCD2.presetValue[1][x]=presetFast[1][x];
                                }
                                set_picture(2,43);
                                write_button(2,'U');//Suba la manija
                                flowLCD2=28; 
                            }else{
                                flowLCD2=25;
                                set_picture(2,57);
                                show_message(2,bufferLCD2.message);
                                write_button(2,'o');//Continuar
                            }
                        break;
                       
                        case 0x15:  //P3
                            if(higher(presetFast[2],bufferLCD2.moneyQuota)!=1){
                                bufferLCD2.presetType[0]=2;
                                bufferLCD2.presetType[1]='3';
                                for(x=0;x<=presetFast[2][0];x++){
                                    bufferLCD2.presetValue[0][x]=presetFast[2][x];
                                    bufferLCD2.presetValue[1][x]=presetFast[2][x];
                                }
                                set_picture(2,43);
                                write_button(2,'U');//Suba la manija
                                flowLCD2=28; 
                            }else{
                                flowLCD2=25;
                                set_picture(2,57);
                                show_message(2,bufferLCD2.message);
                                write_button(2,'o');//Continuar
                            }
                        break;
                        
                        case 0x0F:  //Atras
                            flowLCD2=25;
                            set_picture(2,57);
                            show_message(2,bufferLCD2.message);
                            write_button(2,'o');//Continuar
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
            if(countAnimation2>240){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'A');//Accion Anulada
            }
        break;
            
        case 27: //Teclado dinero credito
            switch (alphanumeric_keyboard(2,digits,0)){
                case 0: //Cancelar
                    set_picture(2,35);
                    write_button(2,'C');    //Contado
                    flowLCD2=26;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.presetValue[0][x]=bufferLCD2.valueKeys[x];
                        bufferLCD2.presetValue[1][x]=bufferLCD2.valueKeys[x];
                    }
                    temporal[0]=1;
                    temporal[1]='0';
                    if(higher(bufferLCD2.presetValue[1],temporal)==1){
                        if(bufferLCD2.presetType[1]=='V'){
                            if(higher(bufferLCD2.presetValue[0],bufferLCD2.volumeQuota)!=1){
                                set_picture(2,43);
                                write_button(2,'U');    //Suba la manija
                                flowLCD2=28;                     
                            }else{
                                flowLCD2=25;
                                set_picture(2,57);
                                show_message(2,bufferLCD2.message);
                                write_button(2,'o');//Continuar
                            }
                        }else{
                            if(higher(bufferLCD2.presetValue[0],bufferLCD2.moneyQuota)!=1){
                                set_picture(2,43);
                                write_button(2,'U');    //Suba la manija
                                flowLCD2=28;                   
                            }else{
                                flowLCD2=25;
                                set_picture(2,57);
                                show_message(2,bufferLCD2.message);
                                write_button(2,'o');//Continuar
                            }
                        }
                    }
                break;
            }
            if(countAnimation2>240){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'A');//Accion Anulada
            }
        break;
            
        case 28: //Suba la manija
            if(countAnimation2>240){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'A');//Accion Anulada
                break;
            }
            if(LCD2_GetRxBufferSize()==8){
                isr_2_Stop(); 
                Waitable_2_Stop();
                flowLCD2=1;
                bufferLCD2.stateMux[1]=0x16;//Espera
                LCD2_ClearRxBuffer();
                break;
            }
            if(get_state(side.b.dir)==0x07){
                isr_2_Stop(); 
                Waitable_2_Stop();
                CyDelay(100);
                if((get_handle(side.b.dir)==bufferLCD2.productType)){
                    if(read_date()==1 && read_time()==1){
                        for(x=0;x<=2;x++){
                            bufferLCD2.timeLiftHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                            bufferLCD2.timeLiftHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                            bufferLCD2.dateLiftHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                            bufferLCD2.dateLiftHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                        }
                    }else{
                        for(x=0;x<6;x++){
                            bufferLCD2.timeLiftHandle[x]='0';
                            bufferLCD2.dateLiftHandle[x]='0';
                        }
                    }
                    set_picture(2,43);
                    write_button(2,'W');//Espere 1 momento
                    flowLCD2=29;
                    LCD2_ClearRxBuffer();
                }else{
                    bufferLCD2.stateMux[1]=0x16;//Espera
                    show_picture(2,44,3);
                    write_button(2,'E');//Error de sistema
                }
            }
        break;
            
        case 29: //Programar Equipo  
            if(price_change(side.b.dir,bufferLCD2.productType,bufferLCD2.ppuQuota)==0){
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x10;//Error
                write_button(2,'E');//Error de sistema
                break;
            }
            if(get_totals(side.b.dir)!=0){
                for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                    bufferLCD2.totalVolumePrevious[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                    bufferLCD2.totalMoneyPrevious[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                    bufferLCD2.totalPPUPrevious[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                }
            }else{
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x10;//Error
                write_button(2,'E');//Error de sistema
            }
            if(preset_data(side.b.dir,bufferLCD2.productType,bufferLCD2.presetValue[0],bufferLCD2.presetType[0]&0x03)==0){
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x10;//Error
                write_button(2,'E');//Error de sistema
                break;
            }
            if(get_handle(side.b.dir)!=0){
                Pump_AL_PutChar(0x10|side.b.dir);//Autoriza el surtidor
                set_picture(2,45);
                bufferLCD2.licenceSale[0]=0;
                bufferLCD2.identySale[0]=0;
                write_button(2,'T');//Tanqueando
                flowLCD2=30;
                LCD2_ClearRxBuffer();
            }else{
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x10;//Error
                write_button(2,'E');//Error de sistema
            }
        break;
            
        case 30: //Tanqueando venta credito
            switch(get_state(side.b.dir)){
                case 0x09://Surtiendo
                    bufferLCD2.stateMux[1]=0x19;//Surtiendo
                break;
                
                case 0x0A://Reporte de venta
                    set_picture(2,43);
                    write_button(2,'W');//Espere 1 momento
                    if(get_sale(side.b.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD2.timeDownHandle[x]='0';
                                bufferLCD2.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.b.dir)!=0){
                            for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                                bufferLCD2.totalVolumeAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                                bufferLCD2.totalMoneyAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                                bufferLCD2.totalPPUAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                            }
                        }else{
                            bufferLCD2.totalVolumeAfter[0]=1;
                            bufferLCD2.totalMoneyAfter[0]=1;
                            bufferLCD2.totalPPUAfter[0]=1;
                            bufferLCD2.totalVolumeAfter[1]='0';
                            bufferLCD2.totalMoneyAfter[1]='0';
                            bufferLCD2.totalPPUAfter[1]='0';
                        }
                        if(price_change(side.b.dir,bufferLCD2.productType,side.b.ppuAuthorized[bufferLCD2.productType-1])==0){
                            show_picture(2,44,3);
                            bufferLCD2.stateMux[1]=0x10;//Error
                            write_button(2,'E');//Error de sistema
                            break;
                        }
                        show_picture(2,54,3);
                        bufferLCD2.stateMux[1]=0x1A;//Fin venta
                        write_button(2,'G');//Gracias
                    }else{
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                    }
                break;
                
                case 0x0B://Reporte de venta
                    set_picture(2,43);
                    write_button(2,'W');//Espere 1 momento
                    if(get_sale(side.b.dir)==1){
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD2.timeDownHandle[x]='0';
                                bufferLCD2.dateDownHandle[x]='0';
                            }
                        }
                        if(get_totals(side.b.dir)!=0){
                            for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                                bufferLCD2.totalVolumeAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                                bufferLCD2.totalMoneyAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                                bufferLCD2.totalPPUAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                            }
                        }else{
                            bufferLCD2.totalVolumeAfter[0]=1;
                            bufferLCD2.totalMoneyAfter[0]=1;
                            bufferLCD2.totalPPUAfter[0]=1;
                            bufferLCD2.totalVolumeAfter[1]='0';
                            bufferLCD2.totalMoneyAfter[1]='0';
                            bufferLCD2.totalPPUAfter[1]='0';
                        }
                        if(price_change(side.b.dir,bufferLCD2.productType,side.b.ppuAuthorized[bufferLCD2.productType-1])==0){
                            show_picture(2,44,3);
                            bufferLCD2.stateMux[1]=0x10;//Error
                            write_button(2,'E');//Error de sistema
                            break;
                        }
                        show_picture(2,54,3);
                        bufferLCD2.stateMux[1]=0x1A;//Fin venta
                        write_button(2,'G');//Gracias
                    }else{
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                    }
                break;
                
                case 0x06://Espera
                    if(get_totals(side.b.dir)!=0){
                        for(x=0;x<=side.b.totalsHandle[bufferLCD2.productType-1][0][0];x++){
                            bufferLCD2.totalVolumeAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][0][x];
                            bufferLCD2.totalMoneyAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][1][x];
                            bufferLCD2.totalPPUAfter[x]=side.b.totalsHandle[bufferLCD2.productType-1][2][x];
                        }
                        if(price_change(side.b.dir,bufferLCD2.productType,side.b.ppuAuthorized[bufferLCD2.productType-1])==0){
                            show_picture(2,44,3);
                            bufferLCD2.stateMux[1]=0x10;//Error
                            write_button(2,'E');//Error de sistema
                            break;
                        }
                        if(subtraction(bufferLCD2.totalVolumeAfter,bufferLCD2.totalVolumePrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.b.volumeSale[x]=residue[x];
            				}	
                            side.b.volumeSale[x]='0';
                            side.b.volumeSale[0]++;
                        }else{
                            bufferLCD2.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD2=1;
                            break;
                        }
                        if(subtraction(bufferLCD2.totalMoneyAfter,bufferLCD2.totalMoneyPrevious)==0){
            				for(x=0;x<=residue[0];x++){
            					side.b.moneySale[x]=residue[x];
            				}
                        }else{
                            bufferLCD2.stateMux[1]=0x1B;//Fin Venta en Cero
                            flowLCD2=1;
                            break;
                        }
                        if(read_date()==1 && read_time()==1){
                            for(x=0;x<=2;x++){
                                bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                            }
                        }else{
                            for(x=0;x<6;x++){
                                bufferLCD2.timeDownHandle[x]='0';
                                bufferLCD2.dateDownHandle[x]='0';
                            }
                        }
                        side.b.productSale=(bufferLCD2.productType&0x0F)+0x30;
        				for(x=0;x<=bufferLCD2.totalPPUAfter[0];x++){
        					side.b.ppuSale[x]=bufferLCD2.totalPPUAfter[x];
        				}
                        show_picture(2,54,3);
                        bufferLCD2.stateMux[1]=0x1A;//Fin venta
                        write_button(2,'G');//Gracias
                    }else{
                        show_picture(2,44,3);
                        bufferLCD2.stateMux[1]=0x10;//Error
                        write_button(2,'E');//Error de sistema
                    }
                break;
            }
        break;
            
        case 31://Formulario canastilla
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    if(LCD2_rxBuffer[3]<=3){//Productos
                        set_picture(2,46);
                        write_button(2,'9');
                        bufferLCD2.flagProductmarket=LCD2_rxBuffer[3]-1;
                        numberKeys2=0;
                        flowLCD2=32;
                        Code_Bar_ClearRxBuffer();
                    }else if(LCD2_rxBuffer[3]<=6){//Cantidades
                        set_picture(2,37);
                        write_button(2,'0');
                        bufferLCD2.flagKeyboard=LCD2_rxBuffer[3]-4;
                        numberKeys2=0;
                        flagPoint2=1;
                        flowLCD2=33;
                    }else if(LCD2_rxBuffer[3]<=9){//Borrar productos
                        for(x=1;x<=20;x++){
                            bufferLCD2.serialMarket[LCD2_rxBuffer[3]-7][x]='0';
                        }
                        bufferLCD2.serialMarket[LCD2_rxBuffer[3]-7][0]=20;
                        for(x=1;x<=20;x++){
                            bufferLCD2.productMarket[LCD2_rxBuffer[3]-7][x]='-';
                        }
                        bufferLCD2.productMarket[LCD2_rxBuffer[3]-7][0]=20;
                        for(x=1;x<=3;x++){
                            bufferLCD2.quantityMarket[LCD2_rxBuffer[3]-7][x]='0';
                            bufferLCD2.quantAvailableMark[LCD2_rxBuffer[3]-7][0]='0';
                        }
                        bufferLCD2.quantAvailableMark[LCD2_rxBuffer[3]-7][0]=3;
                        bufferLCD2.quantityMarket[LCD2_rxBuffer[3]-7][0]=3;
                        for(x=1;x<=8;x++){
                            bufferLCD2.priceTotalMarket[LCD2_rxBuffer[3]-7][x]='0';
                        }
                        bufferLCD2.priceTotalMarket[LCD2_rxBuffer[3]-7][0]=8;
                        for(x=1;x<=8;x++){
                            bufferLCD2.priceUnitMarket[LCD2_rxBuffer[3]-7][x]='0';
                        }
                        bufferLCD2.priceUnitMarket[LCD2_rxBuffer[3]-7][0]=8;
                        set_picture(2,59);
                        write_button(2,'c');    //Canasta
                        show_market(2);
                    }else if(LCD2_rxBuffer[3]==0x0E){//Enter
                        temporal[0]=1;
                        temporal[1]='0';
                        if(higher(temporal,bufferLCD2.totalMarket)==2){
                            if(read_date()==1 && read_time()==1){
                                for(x=0;x<=2;x++){
                                    bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                    bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                    bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                    bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                                }
                            }else{
                                for(x=0;x<6;x++){
                                    bufferLCD2.timeDownHandle[x]='0';
                                    bufferLCD2.dateDownHandle[x]='0';
                                }
                            }
                            if(bufferLCD2.saleType=='1'){//Contado
                                show_picture(2,54,4);
                                bufferLCD2.stateMux[1]=0x33;//Fin venta Canastilla
                                write_button(2,'G');//Gracias
                            }else{//credito
                                if(higher(bufferLCD2.totalMarket,bufferLCD2.moneyQuota)!=1){
                                    show_picture(2,54,4);
                                    bufferLCD2.stateMux[1]=0x33;//Fin venta Canastilla
                                    write_button(2,'G');//Gracias
                                }else{
                                    flowLCD2=24;
                                    set_picture(2,57);
                                    show_message(2,bufferLCD2.message);
                                    write_button(2,'o');//Continuar
                                }
                            }
                        }
                    }else if(LCD2_rxBuffer[3]==0x0F){//Atras
                        if(bufferLCD2.saleType=='1'){//Contado
                            set_picture(2,33);
                            write_button(2,'S');    //Venta
                            bufferLCD2.stateMux[1]=0x16;//Espera
                            flowLCD2=5;
                        }else{//credito
                            flowLCD2=24;
                            set_picture(2,57);
                            write_button(2,'o');//Continuar
                            show_message(2,bufferLCD2.message);
                        }
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 32://Teclado codigo producto y/o lector de barras
            if(serial_codebar()==1){
                for(x=0;x<=temporal[0];x++){
                    bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][x]=temporal[x];
                }
                for(x=1;x<=bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][0];x++){
                    write_LCD(2,bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][x],2,x+2,1,0x0000,'N');
                }
                bufferLCD2.stateMux[1]=0x31;
                set_picture(2,55);
                write_button(2,'W');//Espere 1 momento
                isr_2_StartEx(timerAnimation2); 
                Waitable_2_Start();
                countAnimation2=0;
                bufferLCD2.authorizationFlag=0;
                flowLCD2=34;
                LCD2_ClearRxBuffer();
                break;
            }
            switch (alphanumeric_keyboard(2,20,0)){
                case 0: //Cancelar
                    set_picture(2,59);
                    write_button(2,'c');    //Canasta
                    show_market(2);
                    flowLCD2=31;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][x]=bufferLCD2.valueKeys[x];
                    }
                    bufferLCD2.stateMux[1]=0x31;
                    set_picture(2,55);
                    write_button(2,'W');//Espere 1 momento
                    isr_2_StartEx(timerAnimation2); 
                    Waitable_2_Start();
                    countAnimation2=0;
                    bufferLCD2.authorizationFlag=0;
                    flowLCD2=34;
                break;
            }
        break;
            
        case 33://Teclado cantidades canasta
            switch (alphanumeric_keyboard(2,3,0)){
                case 0: //Cancelar
                    set_picture(2,59);
                    write_button(2,'c');    //Canasta
                    show_market(2);
                    flowLCD2=31;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][x]=bufferLCD2.valueKeys[x];
                    }
                    if(higher(bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard],bufferLCD2.quantAvailableMark[bufferLCD2.flagKeyboard])!=1){
                        if(bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][0]<3){
                            y=bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][0];
                            for(x=3;x>=1;x--){
                                bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][x]=bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][y];
                                y--;
                                if(y==0 && x>1){
                                    while(x>1){
                                        x--;
                                        bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][x]='0';
                                    }
                                    break;
                                }
                            }
                            bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][0]=3;
                        }
                    }else{
                        for(x=0;x<=3;x++){
                            bufferLCD2.quantityMarket[bufferLCD2.flagKeyboard][x]=bufferLCD2.quantAvailableMark[bufferLCD2.flagKeyboard][x];
                        }
                    }
                    set_picture(2,59);
                    write_button(2,'c');    //Canasta
                    show_market(2);
                    flowLCD2=31;
                break;
            }
        break;
            
        case 34://Esperando respuesta a envio de indentificacion de produto
            if(bufferLCD2.authorizationFlag!=0){
                isr_2_Stop(); 
                Waitable_2_Stop();
                if(bufferLCD2.stateMux[0]==0x28){
                    bufferLCD2.stateMux[1]=0x28;//Espera transaccion Credito
                }else{
                    bufferLCD2.stateMux[1]=0x39;//Entra a menu de canasta
                }
                set_picture(2,59);
                write_button(2,'c');    //Canasta
                show_market(2);
                flowLCD2=31;
                LCD2_ClearRxBuffer();
                break;
            }
            if(countAnimation2>200){
                isr_2_Stop(); 
                Waitable_2_Stop();
                for(x=1;x<=20;x++){
                    bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][x]='0';
                }
                bufferLCD2.serialMarket[bufferLCD2.flagProductmarket][0]=20;
                for(x=1;x<=20;x++){
                    bufferLCD2.productMarket[bufferLCD2.flagProductmarket][x]='-';
                }
                bufferLCD2.productMarket[bufferLCD2.flagProductmarket][0]=20;
                for(x=1;x<=3;x++){
                    bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][x]='0';
                }
                bufferLCD2.quantityMarket[bufferLCD2.flagProductmarket][0]=3;
                for(x=1;x<=8;x++){
                    bufferLCD2.priceTotalMarket[bufferLCD2.flagProductmarket][x]='0';
                }
                bufferLCD2.priceTotalMarket[bufferLCD2.flagProductmarket][0]=8;
                for(x=1;x<=8;x++){
                    bufferLCD2.priceUnitMarket[bufferLCD2.flagProductmarket][x]='0';
                }
                bufferLCD2.priceUnitMarket[bufferLCD2.flagProductmarket][0]=8;
                if(bufferLCD2.stateMux[0]==0x28){
                    bufferLCD2.stateMux[1]=0x28;//Espera transaccion Credito
                }else{
                    bufferLCD2.stateMux[1]=0x39;//Entra a menu de canasta
                }
                set_picture(2,59);
                write_button(2,'c');    //Canasta
                show_market(2);
                flowLCD2=31;
            }
        break;
            
        case 35://Teclado cantidades consignacion
            switch (alphanumeric_keyboard(2,10,0)){
                case 0: //Cancelar
                    set_picture(2,30);
                    write_button(2,'M');    //Menu
                    flowLCD2=3;
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        bufferLCD2.priceConsign[x]=bufferLCD2.valueKeys[x];
                    }
                    bufferLCD2.stateMux[1]=0x35;
                    set_picture(2,55);
                    write_button(2,'W');//Espere 1 momento
                    isr_2_StartEx(timerAnimation2); 
                    Waitable_2_Start();
                    countAnimation2=0;
                    bufferLCD2.authorizationFlag=0;
                    flowLCD2=36;
                break;
            }
        break;
            
        case 36://Esperando respuesta a envio de consignacion
            if(bufferLCD2.authorizationFlag!=0){
                isr_2_Stop(); 
                Waitable_2_Stop();
                bufferLCD2.stateMux[1]=0x16;
                set_picture(2,57);
                show_message(2,bufferLCD2.message);
                write_button(2,'o');//Continuar
                flowLCD2=37;
                LCD2_ClearRxBuffer();
                break;
            }
            if(countAnimation2>200){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'E');//Error de sistema
            }
        break; 
            
        case 37://Mensaje autorizacion consignacion o Turnos o Creditos
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    if(LCD2_rxBuffer[3]==0x0F || LCD2_rxBuffer[3]==0x0E){
                        flowLCD2=1;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 38:
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){
                        case 0x01://calibrar
                            bufferLCD2.salePerform='3';
                            bufferLCD2.saleType='3';
                            bufferLCD2.licenceSale[0]=0;
                            bufferLCD2.mileageSale[0]=0;
                            bufferLCD2.identySale[0]=0;
                            bufferLCD2.idType='0';
                            bufferLCD2.idSerial[0]=1;
                            bufferLCD2.idSerial[1]=' ';
                            if(productNumber==1){
                                bufferLCD2.productType=1;
                                bufferLCD2.stateMux[1]=0x26;
                                set_picture(2,55);
                                write_button(2,'W');//Espere 1 momento 
                                isr_2_StartEx(timerAnimation2); 
                                Waitable_2_Start();
                                countAnimation2=0;
                                bufferLCD2.authorizationFlag=0;
                                flowLCD2=24;
                            }else{
                                set_picture(2,39+(productNumber-2));
                                write_button(2,'P');    //Escoja producto
                                flowLCD2=23;
                            }
                        break;
                        
                        case 0x02://Impresion
                            set_picture(2,63);
                            write_button(2,'p');    //impresoras
                            bufferLCD2.printers[0]=1;
                            bufferLCD2.printers[1]=1;
                            flowLCD2=39;
                        break;
                            
                        case 0x03://Desbloquear surtidor
                            set_picture(2,37);
                            write_button(2,'*');//Teclado Contraseña
                            numberKeys2=0;
                            flagPoint2=1;
                            flowLCD2=45;
                        break;
                            
                        case 0x0F://Atras
                            set_picture(2,30);
                            write_button(2,'M');    //Menu
                            flowLCD2=3;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 39://Configurar impresoras
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){
                        case 0x01://Impresora 1
                            if(bufferLCD2.printers[0]==1){
                                bufferLCD2.printers[0]=0;
                                write_LCD(2,'X',12,4,3,0xF800,'Y');
                            }else{
                                bufferLCD2.printers[0]=1;
                                set_picture(2,63);
                                write_button(2,'p');    //impresoras
                                if(bufferLCD2.printers[1]==0){
                                    write_LCD(2,'X',12,8,3,0xF800,'Y');
                                }
                            }
                        break;
                        
                        case 0x02://Impresora 1
                            if(bufferLCD2.printers[1]==1){
                                bufferLCD2.printers[1]=0;
                                write_LCD(2,'X',12,8,3,0xF800,'Y');
                            }else{
                                bufferLCD2.printers[1]=1;
                                set_picture(2,63);
                                write_button(2,'p');    //impresoras
                                if(bufferLCD2.printers[0]==0){
                                    write_LCD(2,'X',12,4,3,0xF800,'Y');
                                }
                            }
                        break;
                            
                        case 0x0E://Continuar
                            bufferLCD2.stateMux[1]=0x37;//Configuracion Impresoras
                            show_picture(2,54,4);
                            write_button(2,'G');
                        break;
                        
                        case 0x0F://Atras
                            set_picture(2,61);
                            write_button(2,'u');//Mantenimiento
                            flowLCD2=38;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 40://Abrir ó cerrar turno
            if(LCD2_GetRxBufferSize()==8){
                if((LCD2_rxBuffer[0]==0xAA) && (LCD2_rxBuffer[6]==0xC3) && (LCD2_rxBuffer[7]==0x3C)){
                    switch(LCD2_rxBuffer[3]){
                        case 0x0D://Bloquear posicion
                            if(turn==1){//Abierto el turno
                                set_picture(2,37);
                                write_button(2,'*');//Teclado Contraseña
                                numberKeys2=0;
                                flagPoint2=1;
                                flowLCD2=46;
                            }
                        break;
                        
                        case 0x0E://Continuar
                            if(lockTurn==0){//desbloqueado
                                bufferLCD2.stateMux[1]=0x40;//Pendiente datos turno
                                set_picture(2,37);
                                write_button(2,'5');//Teclado Nit/CC
                                numberKeys2=0;
                                flagPoint2=1;
                                flowLCD2=41;
                            }else{
                                show_picture(2,66,3);
                            }
                        break;
                        
                        case 0x0F://Atras
                            if(bufferLCD2.stateMux[1]==0x40){//Pendiente datos turno
                                bufferLCD2.stateMux[1]=0x16;//Espera
                            }
                            set_picture(2,30);
                            write_button(2,'M');    //Menu
                            flowLCD2=3;
                        break;
                    }
                }  
                CyDelay(10);         
                LCD2_ClearRxBuffer();
            }
        break;
            
        case 41://Teclado Cedula Vendedor
            switch (alphanumeric_keyboard(2,10,0)){
                case 0: //Cancelar
                    set_picture(2,64);
                    flowLCD2=40;
                    for(x=1;x<=20;x++){
                        idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (x+955));
                    }
                    idSeller[0]=20;
                    typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
                    for(x=0;x<4;x++){
                        passwordSeller[x+1]=(CY_GET_REG8(CYDEV_EE_BASE + (938+x)));
                    }
                    if(turn==1){//Abierto
                        write_button(2,'x');//Cerrar Turno
                        write_button(2,'v');//ID Vendedor
                    }else{//Cerrado
                        write_button(2,'y');//Abrir Turno
                    }
                break;
                    
                case 1: //Enter
                    for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                        idSeller[x]=bufferLCD2.valueKeys[x];
                    }
                    typeIdSeller='C';
                    set_picture(2,37);
                    write_button(2,'*');//Teclado Contraseña
                    numberKeys2=0;
                    flagPoint2=1;
                    flowLCD2=42;
                break;
            }
        break;
            
        case 42://Contraseña Cedula Vendedor
            switch (alphanumeric_keyboard(2,4,'*')){
                case 0: //Cancelar
                    set_picture(2,37);
                    write_button(2,'5');//Teclado Nit/CC
                    numberKeys2=0;
                    flagPoint2=1;
                    flowLCD2=41;
                break;
                    
                case 1: //Enter
                    if(flowLCD1>4){
                        show_picture(2,44,3);
                        write_button(2,'E');//Error de sistema
                    }else{
                        if(bufferLCD2.valueKeys[0]==4){
                            for(x=0;x<=bufferLCD2.valueKeys[0];x++){
                                passwordSeller[x]=bufferLCD2.valueKeys[x];
                            }
                            set_picture(2,55);
                            write_button(2,'W');//Espere 1 momento
                            if(get_totals(side.a.dir)!=0 && get_totals(side.b.dir)!=0){
                                if(read_date()==1 && read_time()==1){
                                    for(x=0;x<=2;x++){
                                        bufferLCD2.timeDownHandle[x*2]=((time[2-x]&0xF0)>>4)+48;
                                        bufferLCD2.timeDownHandle[(x*2)+1]=(time[2-x]&0x0F)+48;
                                        bufferLCD2.dateDownHandle[x*2]=((date[2-x]&0xF0)>>4)+48;
                                        bufferLCD2.dateDownHandle[(x*2)+1]=(date[2-x]&0x0F)+48;
                                    }
                                }else{
                                    for(x=0;x<6;x++){
                                        bufferLCD2.timeDownHandle[x]='0';
                                        bufferLCD2.dateDownHandle[x]='0';
                                    }
                                }
                                isr_2_StartEx(timerAnimation2); 
                                Waitable_2_Start();
                                countAnimation2=0;
                                bufferLCD2.stateMux[1]=0x41;//Peticion Turno
                                bufferLCD2.authorizationFlag=0;
                                show_picture(1,55,10);
                                write_button(1,'W');//Espere 1 momento
                                flowLCD2=43;
                            }else{
                                show_picture(2,44,3);
                                write_button(2,'E');//Error de sistema
                            }
                        }
                    }
                break;
            }
        break;
            
        case 43://Esperando respuesta a envio de Turno
            if(bufferLCD2.authorizationFlag!=0){
                isr_2_Stop(); 
                Waitable_2_Stop();
                bufferLCD2.stateMux[1]=0x16;
                set_picture(2,57);
                show_message(2,bufferLCD2.message);
                write_button(2,'o');//Continuar
                flowLCD2=37;
                LCD2_ClearRxBuffer();
                break;
            }
            if(countAnimation2>200){
                for(x=1;x<=20;x++){
                    idSeller[x]=CY_GET_REG8(CYDEV_EE_BASE + (x+955));
                }
                idSeller[0]=20;
                typeIdSeller=CY_GET_REG8(CYDEV_EE_BASE + 955);
                for(x=0;x<4;x++){
                    passwordSeller[x+1]=(CY_GET_REG8(CYDEV_EE_BASE + (938+x)));
                }
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'E');//Error de sistema
            }
        break;
            
        case 44://Posicion Bloqueada por Corte
            if(countAnimation2>12000){
                isr_2_Stop(); 
                Waitable_2_Stop();
                show_picture(2,44,3);
                bufferLCD2.stateMux[1]=0x16;//Espera
                write_button(2,'E');//Error de sistema
            }
        break;
            
        case 45://Contraseña Desbloquear surtidor
            switch (alphanumeric_keyboard(2,6,'*')){
                case 0: //Cancelar
                    set_picture(2,61);
                    write_button(2,'u');//Mantenimiento
                    flowLCD2=38;
                break;
                    
                case 1: //Enter
                    for(x=1;x<=6;x++){
                        if(bufferLCD2.valueKeys[x]!=passwordPump[x-1]){
                            break;
                        }
                    }
                    if(x>6){
                        stateBeagleSoft=1;//En comunicacion
                        show_picture(2,54,4);
                        write_button(2,'G');//Gracias
                        for(x=0;x<3;x++){
                            if(get_state(side.b.dir)==0x06){
                                CyDelay(60);
                                Pump_AL_PutChar(0x10|side.b.dir);//Autoriza el surtidor
                                CyDelay(60);
                                Pump_AL_PutChar(0x30|side.b.dir);//Detiene el surtidor
                            }
                            CyWdtClear();
                            CyDelay(250);
                        }
                    }else{
                        show_picture(2,44,3);
                        write_button(2,'E');//Error de sistema
                    }
                break;
            }
        break;
            
        case 46://Contraseña Vendedor para bloquear turno
            switch (alphanumeric_keyboard(2,4,'*')){
                case 0: //Cancelar
                    set_picture(2,64);
                    flowLCD2=40;
                    if(turn==1){//Abierto
                        write_button(2,'x');//Cerrar Turno
                        write_button(2,'v');//ID Vendedor
                    }else{//Cerrado
                        write_button(2,'y');//Abrir Turno
                    }
                break;
                    
                case 1: //Enter
                    if(flowLCD1>4){
                        show_picture(2,44,3);
                        write_button(2,'E');//Error de sistema
                    }else{
                        if(bufferLCD2.valueKeys[0]==4){
                            for(x=1;x<=4;x++){
                                if(bufferLCD2.valueKeys[x]!=passwordSeller[x]){
                                    show_picture(2,44,3);
                                    write_button(2,'E');//Error de sistema
                                    break;
                                }
                            }
                            if(x>=5){
                                lockTurn=!lockTurn;
                                show_picture(2,54,4);
                                write_button(2,'G');
                            }
                        }
                    }
                break;
            }
        break;
    }
}

/*
*********************************************************************************************************
*                                         void polling_Pump(void)
*
* Description : Pregunta estado al surtidor cada segundo
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

void polling_Pump(void){
    uint8 state=0;
    if(countPump>4){
        if(bufferLCD1.stateMux[1]==0x16 || bufferLCD1.stateMux[1]==0x1F){//Espera o Lazo desconectado
            Pump_AL_ClearRxBuffer();
            Pump_AL_PutChar(side.a.dir);
            CyDelay(65);
            if(Pump_AL_GetRxBufferSize()>=1){
                state=Pump_AL_ReadRxData();
                if(state==(0x60|side.a.dir) || state==(0x70|side.a.dir) || state==(0x80|side.a.dir) || state==(0x90|side.a.dir)){
                    bufferLCD1.stateMux[1]=0x16;
                }else{
                    bufferLCD1.stateMux[1]=0x1F;
                }
                Pump_AL_ClearRxBuffer();
            }else{
                bufferLCD1.stateMux[1]=0x1F;
            }
        }
        if(bufferLCD2.stateMux[1]==0x16 || bufferLCD2.stateMux[1]==0x1F){//Espera o Lazo desconectado
            Pump_AL_ClearRxBuffer();
            Pump_AL_PutChar(side.b.dir);
            CyDelay(65);
            if(Pump_AL_GetRxBufferSize()>=1){
                state=Pump_AL_ReadRxData();
                if(state==(0x60|side.b.dir) || state==(0x70|side.b.dir) || state==(0x80|side.b.dir) || state==(0x90|side.b.dir)){
                    bufferLCD2.stateMux[1]=0x16;
                }else{
                    bufferLCD2.stateMux[1]=0x1F;
                }
                Pump_AL_ClearRxBuffer();
            }else{
                bufferLCD2.stateMux[1]=0x1F;
            }
        }
        if(flowLCD1==44){
            Pump_AL_PutChar(0x10|side.a.dir);//Autoriza el surtidor
            CyDelay(60);
            Pump_AL_PutChar(0x30|side.a.dir);//Detiene el surtidor
        }
        if(flowLCD2==44){
            Pump_AL_PutChar(0x10|side.b.dir);//Autoriza el surtidor
            CyDelay(60);
            Pump_AL_PutChar(0x30|side.b.dir);//Detiene el surtidor 
        }
        countPump=0;
    }
}

/*
*********************************************************************************************************
*                                         CY_ISR(timerAnimation1)
*
* Description : Interrupcion que temporiza tiempos de espera
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
CY_ISR(timerAnimation1){
    Waitable_1_ReadStatusRegister();
    if(flowLCD1==2){
        if(countAnimation1<29){
            countAnimation1++;
            set_picture(1,29-countAnimation1);  
        }
        else{
           countAnimation1=0; 
           set_picture(1,29-countAnimation1);  
        }
    }else{
        countAnimation1++;
    }
}

/*
*********************************************************************************************************
*                                         CY_ISR(timerAnimation2)
*
* Description : Interrupcion que temporiza tiempos de espera
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
CY_ISR(timerAnimation2){
    Waitable_2_ReadStatusRegister();
    if(flowLCD2==2){
        if(countAnimation2<29){
            countAnimation2++;
            set_picture(2,29-countAnimation2);  
        }
        else{
           countAnimation2=0; 
           set_picture(2,29-countAnimation2);  
        }
    }else{
        countAnimation2++;
    }
}

/*
*********************************************************************************************************
*                                         CY_ISR(timerBeagleTX1)
*
* Description : Interrupcion que temporiza tiempos de espera
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
CY_ISR(timerBeagleTX){
    Waitable_3_ReadStatusRegister();
    countBeagleTX++;
}

/*
*********************************************************************************************************
*                                         CY_ISR(timerPump)
*
* Description : Interrupcion que temporiza tiempos de espera para preguntar al surtidor
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
CY_ISR(timerPump){
    Waitable_4_ReadStatusRegister();
    countPump++;
}

/*
*********************************************************************************************************
*                                         main( void )
*
* Description : Ejecuta las funciones de inicio y verifica el estado de las pantallas
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
int main(){
    init_mux();
    init_pump();
    /********************PRUEBAS**********************/
//    Rs1_Write(1);
//    Rs2_Write(1);
//    Rs3_Write(1);
//    Rs4_Write(0);
//    Rs5_Write(0);
//    side.a.ppuAuthorized[0][0]=6;
//    side.a.ppuAuthorized[0][1]='0';
//    side.a.ppuAuthorized[0][2]='0';
//    side.a.ppuAuthorized[0][3]='8';
//    side.a.ppuAuthorized[0][4]='7';
//    side.a.ppuAuthorized[0][5]='8';
//    side.a.ppuAuthorized[0][6]='5';
//    decimalMoney=0;
//    turn=1;
//    decimalVolume=3;
//    digits=6;
//    ppux10=0;
//    productNumber=3;
//    symbols[0]='$';
//    symbols[1]='G';
//    screen[0]=0x31;
//    screen[1]=0x31;
//    side.a.dir=1;
//    side.b.dir=2;
//    uint8 prueb1[16]={'C','a','l','i','b','r','a','r','I','m','p','r','i','m','i','r'};
//    EEPROM_1_Write(prueb1,53);
//    uint8 prueb2[16]={'E','l','e','g','i','r',' ','I','m','p','r','e','s','o','r','a'};
//    EEPROM_1_Write(prueb2,54);
    /********************PRUEBAS**********************/  
	CyWdtStart(CYWDT_1024_TICKS,CYWDT_LPMODE_NOCHANGE); 
    for(;;){  
		CyWdtClear();
        polling_Beagle_RX();
        polling_LCD1();
		CyWdtClear();
        polling_LCD2();
		CyWdtClear();
        polling_Pump();
        CyWdtClear();
        polling_beagle_TX();
    }	
}

/* [] END OF FILE */
