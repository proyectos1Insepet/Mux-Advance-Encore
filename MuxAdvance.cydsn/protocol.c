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
*                                             MUX ADVANCE CODE
*
*                                             CYPRESS PSoC5LP
*                                                with the
*                                            CY8C5969AXI-LP035
*
* Filename      : protocol.c
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
#include <variables.h>

/*
*********************************************************************************************************
*                                    uint8 get_state(uint8 pos)
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

uint8 get_state(uint8 pos){
    uint8 state;
    Pump_AL_ClearRxBuffer();
    Pump_AL_PutChar(pos);
    CyDelay(65);
    if(Pump_AL_GetRxBufferSize()>=1){
       state=(Pump_AL_ReadRxData()&0xF0)>>4;
       Pump_AL_ClearRxBuffer();
       return state;
    }
    else{
        return 0;
    }
}

/*
*********************************************************************************************************
*                                         uint8 get_position(void)
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
uint8 get_position(void){
    uint8 x;
        
    side.a.dir=0xff;
    side.b.dir=0xff;
    for(x=1;x<=16;x++){
        Pump_AL_PutChar(x);
        CyDelay(100);
        if((Pump_AL_GetRxBufferSize()>=1)&&(side.a.dir==0xff)){
           side.a.dir=x&0x0f;	
           Pump_AL_ClearRxBuffer();
        }
        if((Pump_AL_GetRxBufferSize()>=1)&&(x!=side.a.dir)){
           side.b.dir=x&0x0f;
           Pump_AL_ClearRxBuffer();
        }
    }
    if((side.a.dir!=0xff)&&(side.b.dir!=0xff)){
        return 2;
    }
    else{
        if((side.a.dir!=0xff)||(side.b.dir!=0xff)){
            return 1;
        }
        else{
            return 0;
        }
    }
}

/*
*********************************************************************************************************
*                                    uint8 get_handle(uint8 pos)
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

uint8 get_handle(uint8 pos){
    uint8 buffer[19],command[9]={0xFF,0xE9,0xFE,0xE0,0xE1,0xE0,0xFB,0xEE,0xF0},x;
    Pump_AL_ClearRxBuffer();
    Pump_AL_PutChar(0x20|pos);
    CyDelay(100);
    if(Pump_AL_GetRxBufferSize()>=1){
        if(Pump_AL_ReadRxData()==(0xD0|pos)){
            Pump_AL_ClearRxBuffer();
            for(x=0;x<9;x++){
               Pump_AL_PutChar(command[x]);
            }
            CyDelay(200);
            if(Pump_AL_GetRxBufferSize()>=19){
                for(x=0;x<=18;x++){
                   buffer[x]=Pump_AL_ReadRxData(); 
                }
                Pump_AL_ClearRxBuffer();
                if((buffer[0]==0xBA)&&(buffer[17]==0x8D)&&(buffer[18]==0x8A)&&(buffer[12]==0xB1)&&(buffer[14]>=0xB1)&&(buffer[14]<=0xB4)){
                    return buffer[14]&0x07;
                }
                else{
                    return 0;
                }
            }
            else{
                return 0;
            }
        }
        else{
            return 0;
        }
    }
    else{
        return 0;
    }
}

/*
*********************************************************************************************************
*                                    uint8 get_totals(uint8 pos)
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

uint8 get_totals(uint8 pos){
    uint8 x,y,z,w,gradeHandle;
    Pump_AL_ClearRxBuffer();
    Pump_AL_PutChar(0x50|pos);
    CyWdtClear();
    CyDelay(900);
    CyWdtClear();
    x=Pump_AL_GetRxBufferSize();
    if((x==34)||(x==64)||(x==94)||(x==124)){//Version 5 ó 6 digitos
        if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[3]==0xF9)){
            gradeHandle=(x/30)&0x07;//Identificando cantidad de mangueras en el surtidor
            if(pos==side.a.dir){
                for(x=0;x<4;x++){
                    for(y=0;y<3;y++){
                        for(z=0;z<14;z++){
                            side.a.totalsHandle[x][y][z]=0;
                        }
                    }
                }
                w=0;
                for(x=0;x<gradeHandle;x++){
                    for(y=0;y<=2;y++){
                        for(z=1;z<=8;z++){
                            side.a.totalsHandle[x][y][9-z]=(Pump_AL_rxBuffer[w+4]&0x0F)+0x30;
                            w++;
                            if((y==2)&&(z==4)){
                                break;
                            }
                        }
                        w++;
                    }
                    w=w+7;
                    for(z=4;z>=1;z--){
                        side.a.totalsHandle[x][2][z]=side.a.totalsHandle[x][2][z+4];
                    }
                    side.a.totalsHandle[x][0][0]=8;
                    side.a.totalsHandle[x][1][0]=8;
                    side.a.totalsHandle[x][2][0]=4;
                    if(ppux10==1){
                        side.a.totalsHandle[x][2][0]=5;
                        side.a.totalsHandle[x][2][5]='0';
                    }
                }
                Pump_AL_ClearRxBuffer();
                return gradeHandle;
            }else{
                for(x=0;x<4;x++){
                    for(y=0;y<3;y++){
                        for(z=0;z<14;z++){
                            side.b.totalsHandle[x][y][z]=0;
                        }
                    }
                }
                w=0;
                for(x=0;x<gradeHandle;x++){
                    for(y=0;y<=2;y++){
                        for(z=1;z<=8;z++){
                            side.b.totalsHandle[x][y][9-z]=(Pump_AL_rxBuffer[w+4]&0x0F)+0x30;
                            w++;
                            if((y==2)&&(z==4)){
                                break;
                            }
                        }
                        w++;
                    }
                    w=w+7;
                    for(z=4;z>=1;z--){
                        side.b.totalsHandle[x][2][z]=side.b.totalsHandle[x][2][z+4];
                    }
                    side.b.totalsHandle[x][0][0]=8;
                    side.b.totalsHandle[x][1][0]=8;
                    side.b.totalsHandle[x][2][0]=4;
                    if(ppux10==1){
                        side.b.totalsHandle[x][2][0]=5;
                        side.b.totalsHandle[x][2][5]='0';
                    }
                }
                Pump_AL_ClearRxBuffer();
                return gradeHandle;
            }
        }else{
            Pump_AL_ClearRxBuffer();
            return 0;
        }
    }else if((x==46)||(x==88)||(x==130)||(x==172)){//Version 7 digitos
        if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[3]==0xF9)){
            gradeHandle=(x/40)&0x07;//Identificando cantidad de mangueras en el surtidor
            if(pos==side.a.dir){
                for(x=0;x<4;x++){
                    for(y=0;y<3;y++){
                        for(z=0;z<14;z++){
                            side.a.totalsHandle[x][y][z]=0;
                        }
                    }
                }
                w=0;
                for(x=0;x<gradeHandle;x++){
                    for(y=0;y<=2;y++){
                        for(z=1;z<=12;z++){
                            side.a.totalsHandle[x][y][13-z]=(Pump_AL_rxBuffer[w+4]&0x0F)+0x30;
                            w++;
                            if((y==2)&&(z==6)){
                                break;
                            }
                        }
                        w++;
                    }
                    w=w+9;
                    for(z=6;z>=1;z--){
                        side.a.totalsHandle[x][2][z]=side.a.totalsHandle[x][2][z+6];
                    }
                    side.a.totalsHandle[x][0][0]=12;
                    side.a.totalsHandle[x][1][0]=12;
                    side.a.totalsHandle[x][2][0]=6;
                }
                Pump_AL_ClearRxBuffer();
                return gradeHandle;
            }else{
                for(x=0;x<4;x++){
                    for(y=0;y<3;y++){
                        for(z=0;z<14;z++){
                            side.b.totalsHandle[x][y][z]=0;
                        }
                    }
                }
                w=0;
                for(x=0;x<gradeHandle;x++){
                    for(y=0;y<=2;y++){
                        for(z=1;z<=12;z++){
                            side.b.totalsHandle[x][y][13-z]=(Pump_AL_rxBuffer[w+4]&0x0F)+0x30;
                            w++;
                            if((y==2)&&(z==6)){
                                break;
                            }
                        }
                        w++;
                    }
                    w=w+9;
                    for(z=6;z>=1;z--){
                        side.b.totalsHandle[x][2][z]=side.b.totalsHandle[x][2][z+6];
                    }
                    side.b.totalsHandle[x][0][0]=12;
                    side.b.totalsHandle[x][1][0]=12;
                    side.b.totalsHandle[x][2][0]=6;
                }
                Pump_AL_ClearRxBuffer();
                return gradeHandle;
            }
        }else{
            Pump_AL_ClearRxBuffer();
            return 0;
        }
    }else{
        Pump_AL_ClearRxBuffer();
        return 0;
    }
    Pump_AL_ClearRxBuffer();
    return 0;
}

/*
*********************************************************************************************************
*                                         uint8 price_change(uint8 pos)
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
uint8 price_change(uint8 pos,uint8 handle,uint8 *value){
	uint8 buffer[15]={0xFF,0,0xF4,0xF6,0,0xF7,0,0,0,0,0,0,0,0,0},size,x,y;
    for(x=1;x<=6;x++){
        if(value[x]==','){
            for(y=x;y>=2;y--){
                value[y]=value[y-1];
            }
            value[1]='0';
            break;
        }
    }
	if(digits!=7){
		buffer[1]=0xE5;
		buffer[4]=(0xE0|((handle&0x0f)-1));
		if(ppux10==0){
			for(x=0;x<4;x++){
				buffer[6+x]=(0xE0|(value[6-x]&0x0F));
			}
		}else{
            for(x=0;x<4;x++){
				buffer[6+x]=(0xE0|(value[5-x]&0x0F));
			}	
		}
		buffer[10]=0xFB;
		for(x=0;x<=10;x++){
			buffer[11]+=(buffer[x]&0x0F);	
		}
		buffer[11]=(((~buffer[11])+1)&0x0F)|0xE0;
		buffer[12]=0xF0;
		size=12;
	}
	if(digits==7){
		buffer[1]=0xE3;
		buffer[4]=(0xE0|((handle&0x0f)-1));	
        for(x=0;x<6;x++){
            buffer[6+x]=(0xE0|(value[6-x]&0x0F));
        }	
		buffer[12]=0xFB;
		for(x=0;x<=12;x++){
			buffer[13]+=(buffer[x]&0x0F);	
		}
		buffer[13]=(((~buffer[13])+1)&0x0F)|0xE0;
		buffer[14]=0xF0;
		size=14;
	}	
    Pump_AL_ClearRxBuffer();
	Pump_AL_PutChar(0x20|pos);	
    CyDelay(100);
    if(Pump_AL_GetRxBufferSize()>=1){
		if(Pump_AL_ReadRxData()==(0xD0|pos)){
            Pump_AL_ClearRxBuffer();
            for(x=0;x<=size;x++){
               	Pump_AL_PutChar(buffer[x]); 	
            }
            Pump_AL_ClearRxBuffer();
			CyDelay(50);
			if(get_state(pos)==0){
				return 0;
			}
			else{
				return 1;
			}
		}
		else{
			return 0;
		}
	}
	else{
		return 0;	
	}
}

/*
*********************************************************************************************************
*                  uint8 preset_data(uint8 pos, uint8 grade, uint8 *value, uint8 preset)
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
uint8 preset_data(uint8 pos, uint8 grade, uint8 *value, uint8 preset){
    uint8 buffer[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},x,y,z,decimal;
    buffer[0]=0xFF;
    buffer[1]=0xE4;
    if(((digits==5)||(digits==6))&&(preset==1)){
		buffer[1]=0xE3;
	}
	if((digits==5)&&(preset==2)){
		buffer[1]=0xE5;
	}
	if((digits==7)&&(preset==1)){
		buffer[1]=0xE0;
	}
	if((digits==7)&&(preset==2)){
		buffer[1]=0xE2;
	}
    buffer[2]=(0xF0|preset);
    buffer[3]=0xF4;
    decimal=0;
    for(x=1;x<=value[0];x++){
        if(value[x]==','){
            if((decimal=value[0]-x)>3){
                return 0;
            }
            for(y=x;y>=2;y--){
                value[y]=value[y-1];
            }
            value[1]='0';
            break;
        }
    }
    if(digits!=value[0]){
        for(x=0;x<value[0];x++){
            value[digits-x]=value[value[0]-x];
        }
        for(x=(digits-value[0]);x>0;x--){
            value[x]='0';
        }
        value[0]=digits;
    }
    if(preset==1){//Volumen
        buffer[4]=0xF6;
        buffer[5]=(0xE0|(grade-1));
        buffer[6]=0xF8;
        z=7;
        if(digits!=7){
            if(decimal>(decimalVolume-1)){
                for(x=value[0];x>1;x--){
                    value[x]=value[x-1];
                }
                value[1]='0';
            }else if(decimal<(decimalVolume-1)){
                y=(decimalVolume-1)-(decimal);
                for(x=1;x<=value[0];x++){
                    value[x]=value[y+x];
                    if((x+y)==value[0]){
                        for(y=1;y<=(value[0]-x);y++){
                            value[x+y]='0';
                        }
                        break;
                    }
                }
            }
            if(digits==6){
                for(x=1;x<value[0];x++){
                    value[x]=value[x+1];
                }
                value[0]--;
            }
        }else{
            if(decimal!=decimalVolume){
                y=decimalVolume-decimal;
                for(x=1;x<=value[0];x++){
                    value[x]=value[y+x];
                    if((x+y)==value[0]){
                        for(y=1;y<=(value[0]-x);y++){
                            value[x+y]='0';
                        }
                        break;
                    }
                }
            }
        }
    }else{//Dinero
        buffer[4]=0xF8;	
        z=5;
        if(decimal!=decimalMoney){
            y=decimalMoney-decimal;
            for(x=1;x<=value[0];x++){
                value[x]=value[y+x];
                if((x+y)==value[0]){
                    for(y=1;y<=(value[0]-x);y++){
                        value[x+y]='0';
                    }
                    break;
                }
            }
        }
    }
    for(x=0;x<value[0];x++){
        buffer[z+x]=0xE0|(value[value[0]-x]&0x0F);
    }
    if(value[0]>=7){
        buffer[z+x]=0xE0;
        x++;
    }
    buffer[z+x]=0xFB;
    z=z+x+1;
    for(x=0;x<z;x++){
		buffer[z]+=(buffer[x]&0x0F);	
	}
    buffer[z]=(((~buffer[z])+1)&0x0F)|0xE0;
    z++;
    buffer[z]=0xF0;
    Pump_AL_ClearRxBuffer();
	Pump_AL_PutChar(0x20|pos);	
    CyDelay(50);
    if(Pump_AL_GetRxBufferSize()>=1){
		if(Pump_AL_ReadRxData()==(0xD0|pos)){
            Pump_AL_ClearRxBuffer();
            for(x=0;x<=z;x++){
               	Pump_AL_PutChar(buffer[x]);
            }
			CyDelay(50);
			if(get_state(pos)==0){
				return 0; 
			}
			else{
				return 1;
			}
		}
		else{
			return 0;
		}
	}
	else{
		return 0;	
	}      
}


/*
*********************************************************************************************************
*                                         uint8 get_sale(uint8 pos)
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
uint8 get_sale(uint8 pos){
	uint8 x;
	Pump_AL_ClearRxBuffer();
	Pump_AL_PutChar(0x40|pos);
    CyDelay(300);
    CyWdtClear();
	if((digits!=7)&&(Pump_AL_GetRxBufferSize()==33)){
		if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[2]==0xF8)&&(Pump_AL_rxBuffer[32]==0xF0)){
			if(pos==side.a.dir){
				side.a.productSale=((Pump_AL_rxBuffer[9]&0x0F)+1)+0x30;
				for(x=1;x<=4;x++){
					side.a.ppuSale[5-x]=((Pump_AL_rxBuffer[x+11]&0x0F)+0x30);
				}
                side.a.ppuSale[0]=4;
                if(ppux10==1){
                    side.a.ppuSale[0]=5;
                    side.a.ppuSale[5]='0';
                }
				for(x=1;x<=6;x++){
					side.a.volumeSale[7-x]=((Pump_AL_rxBuffer[x+16]&0x0F)+0x30);
				}		
                side.a.volumeSale[0]=6;
				for(x=1;x<=6;x++){
					side.a.moneySale[7-x]=((Pump_AL_rxBuffer[x+23]&0x0F)+0x30);
				}
                if(digits==5){
                    for(x=6;x>1;x--){
                        side.a.moneySale[x]=side.a.moneySale[x-1];
                    }
                    side.a.moneySale[1]='0';
                }
                side.a.moneySale[0]=6;
			}
			else{
				side.b.productSale=((Pump_AL_rxBuffer[9]&0x0F)+1)+0x30;
				for(x=1;x<=4;x++){
					side.b.ppuSale[5-x]=((Pump_AL_rxBuffer[x+11]&0x0F)+0x30);
				}
                side.b.ppuSale[0]=4;
                if(ppux10==1){
                    side.b.ppuSale[0]=5;
                    side.b.ppuSale[5]='0';
                }
				for(x=1;x<=6;x++){
					side.b.volumeSale[7-x]=((Pump_AL_rxBuffer[x+16]&0x0F)+0x30);
				}
                side.b.volumeSale[0]=6;
				for(x=1;x<=6;x++){
					side.b.moneySale[7-x]=((Pump_AL_rxBuffer[x+23]&0x0F)+0x30);
				}	
                if(digits==5){
                    for(x=6;x>1;x--){
                        side.b.moneySale[x]=side.b.moneySale[x-1];
                    }
                    side.b.moneySale[1]='0';
                }
                side.b.moneySale[0]=6;
			}
			return 1;
		}
		else{
			return 0;
		}
	}
	else if((digits==7)&&(Pump_AL_GetRxBufferSize()==39)){
		if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[2]==0xF8)&&(Pump_AL_rxBuffer[38]==0xF0)){
			if(pos==side.a.dir){
				side.a.productSale=((Pump_AL_rxBuffer[9]&0x0F)+1)+0x30;
				for(x=1;x<=6;x++){
					side.a.ppuSale[7-x]=((Pump_AL_rxBuffer[x+11]&0x0F)+0x30);
				}
                side.a.ppuSale[0]=6;
				for(x=1;x<=8;x++){
					side.a.volumeSale[9-x]=((Pump_AL_rxBuffer[x+18]&0x0F)+0x30);
				}		
                side.a.volumeSale[0]=8;
				for(x=1;x<=7;x++){
					side.a.moneySale[9-x]=((Pump_AL_rxBuffer[x+28]&0x0F)+0x30);
				}
                side.a.moneySale[1]='0';
                side.a.moneySale[0]=8;
			}
			else{
				side.b.productSale=((Pump_AL_rxBuffer[9]&0x0F)+1)+0x30;
				for(x=1;x<=6;x++){
					side.b.ppuSale[7-x]=((Pump_AL_rxBuffer[x+11]&0x0F)+0x30);
				}
                side.b.ppuSale[0]=6;
				for(x=1;x<=8;x++){
					side.b.volumeSale[9-x]=((Pump_AL_rxBuffer[x+18]&0x0F)+0x30);
				}	
                side.b.volumeSale[0]=8;
				for(x=1;x<=7;x++){
					side.b.moneySale[9-x]=((Pump_AL_rxBuffer[x+28]&0x0F)+0x30);
				}	
                side.b.moneySale[1]='0';
                side.b.moneySale[0]=8;
			}
			return 1;
		}
		else{
			return 0;
		}
	}
	return 0;
}