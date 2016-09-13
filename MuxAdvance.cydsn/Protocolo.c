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
* Filename      : pump.c
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
#include "I2C.h"

/*
*********************************************************************************************************
*                                      uint8 state_ex(uint8 val)
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
uint8 state_ex(uint8 val){
    uint8 rx_extend[19],x,comand_exten[9]={0xFF,0xE9,0xFE,0xE0,0xE1,0xE0,0xFB,0xEE,0xF0};
    Pump_AL_PutChar(0x20|val);
    product=0xff;
    CyDelay(100);
    if(Pump_AL_GetRxBufferSize()>=1){
        if(Pump_AL_ReadRxData()==(0xD0|val)){
            Pump_AL_ClearRxBuffer();
            for(x=0;x<=8;x++){
               Pump_AL_PutChar(comand_exten[x]); 
            }
            CyDelay(200);
            if(Pump_AL_GetRxBufferSize()>=19){
                for(x=0;x<=18;x++){
                    rx_extend[x]=Pump_AL_ReadRxData(); 
                    if(val==side.a.dir){
                        side.a.extend[x+1]=rx_extend[x];
                    }else{
                        side.b.extend[x+1]=rx_extend[x];
                    }
                }
                side.a.extend[0]=19;
                side.b.extend[0]=19;
                Pump_AL_ClearRxBuffer();
                if((rx_extend[0]==0xBA)&&(rx_extend[17]==0x8D)&&(rx_extend[18]==0x8A)&&(rx_extend[12]==0xB1)){
                    switch(rx_extend[14]){
                        case 0xB1:
                            return 1;
                            break;
                        case 0xB2:
                            return 2;
                            break;
                        case 0xB3:
                            return 3;
                            break;
                        default:
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
    uint32 x;
        
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
*                                    uint8 get_state(uint8 dir)
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

uint8 get_state(uint8 dir){
    uint8 state;
    Pump_AL_PutChar(dir);
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
*                                      void get_error(uint8 val)
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
void get_error(uint8 val){
    uint8 state;
    Pump_AL_PutChar(val);
    CyDelay(200);
    if(Pump_AL_GetRxBufferSize()>=1){
       state=(Pump_AL_ReadRxData()&0xF0)>>4;
       Pump_AL_ClearRxBuffer();
       if(state==0){

       }
    }    
}

/*
*********************************************************************************************************
*                                         void stop(uint8 val)
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
void stop(uint8 val){
    Pump_AL_PutChar(0x30|val);
    CyDelay(200);
}


/*
*********************************************************************************************************
*                                  uint8 sale_in(uint8 val)
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
uint8 sale_in(uint8 val){
    uint8 rx_sale[39],pos;
    uint32 j,x=0,y;
    
    Pump_AL_PutChar(0x40|val);
    CyDelay(250);
    if(Pump_AL_GetRxBufferSize()<=39){
        while(Pump_AL_GetRxBufferSize()>0){
            rx_sale[x]=Pump_AL_ReadRxData();  
            if(val==side.a.dir){
                side.a.sale[x+1]=rx_sale[x];
            }else{
                side.b.sale[x+1]=rx_sale[x];
            }
            x++;
        }
        if(val==side.a.dir){
            side.a.sale[0]=x;
        }else{
            side.b.sale[0]=x;
        }
        Pump_AL_ClearRxBuffer();
        if(val==0){
            pos=0x10;
            bSale.position=pos;
        }
        else{
            pos=val;
            bSale.position=pos;
        }
        if((rx_sale[0]==0xFF)&&(rx_sale[2]==0xF8)&&((rx_sale[4]&0x0F)==(pos-1))){
            x=0;
            while((rx_sale[x]!=0xF6)||(x>=39)){
                x++;
            }
            x++;
            bSale.product=(rx_sale[x]&0x0F)+1;
            while((rx_sale[x]!=0xF7)||(x>=39)){
                x++;
            }
            x++;
            y=1;
            while((rx_sale[x]!=0xF9)||(x>=39)){    
                bSale.ppu[y]=(rx_sale[x]&0x0F);
                x++;
                y++;
            }
			bSale.ppu[0]=y-1;
            x++;
            y=1;
            while((rx_sale[x]!=0xFA)||(x>=39)){
                bSale.volume[y]=(rx_sale[x]&0x0F);
                x++;
                y++;    
            }
			bSale.volume[0]=y-1;
            x++;
            y=1;
            while((rx_sale[x]!=0xFB)||(x>=39)){
                bSale.money[y]=(rx_sale[x]&0x0F);
                x++;
                y++;    
            }
			bSale.money[0]=y-1;
            x+=2;
            if((rx_sale[x]==0xF0)&&(x<=38)){								
				if(val==side.a.dir){
					for(j=0;j<=bSale.money[0];j++){
    					side.a.money[j]=bSale.money[j];
    				}
    				for(j=0;j<=bSale.volume[0];j++){
    					side.a.volume[j]=bSale.volume[j];
    				}
    				for(j=0;j<=bSale.ppu[0];j++){
    					side.a.ppu[j]=bSale.ppu[j];
    				}
                    side.a.product=bSale.product;
                    countImp[1]=0;
                    read_date();
                    read_time();
                    for(j=0;j<=2;j++){
                        dateSale1[j]=date[j];
                        timeSale1[j]=time[j];
                    }
				}else{
					for(j=0;j<=bSale.money[0];j++){
    					side.b.money[j]=bSale.money[j];
    				}
    				for(j=0;j<=bSale.volume[0];j++){
    					side.b.volume[j]=bSale.volume[j];
    				}
    				for(j=0;j<=bSale.ppu[0];j++){
    					side.b.ppu[j]=bSale.ppu[j];
    				}	
                    side.b.product=bSale.product;
                    countImp[2]=0;
                    read_date();
                    read_time();
                    for(j=0;j<=2;j++){
                        dateSale2[j]=date[j];
                        timeSale2[j]=time[j];
                    }
				}
                return 1;
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
*                                  uint8 sale(uint8 val)
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
void sale(uint8 sidePump){
    uint8 pos; 
    uint32 j,x=0,y;
    if(sidePump==0){
        pos=0x10;
        bSale.position=pos;
    }
    else{
        pos=sidePump;
        bSale.position=pos;
    }
    if((RxPu[0]==0xFF)&&(RxPu[2]==0xF8)&&((RxPu[4]&0x0F)==(pos-1))){
        x=0;
        while((RxPu[x]!=0xF6)||(x>=39)){
            x++;
        }
        x++;
        bSale.product=(RxPu[x]&0x0F)+1;
        while((RxPu[x]!=0xF7)||(x>=39)){
            x++;
        }
        x++;
        y=1;
        while((RxPu[x]!=0xF9)||(x>=39)){    
            bSale.ppu[y]=(RxPu[x]&0x0F);
            x++;
            y++;
        }
		bSale.ppu[0]=y-1;
        x++;
        y=1;
        while((RxPu[x]!=0xFA)||(x>=39)){
            bSale.volume[y]=(RxPu[x]&0x0F);
            x++;
            y++;    
        }
		bSale.volume[0]=y-1;
        x++;
        y=1;
        while((RxPu[x]!=0xFB)||(x>=39)){
            bSale.money[y]=(RxPu[x]&0x0F);
            x++;
            y++;    
        }
		bSale.money[0]=y-1;
        x+=2;
        if((RxPu[x]==0xF0)&&(x<=38)){								
			if(sidePump==side.a.dir){
				for(j=0;j<=bSale.money[0];j++){
					side.a.money[j]=bSale.money[j];
				}
				for(j=0;j<=bSale.volume[0];j++){
					side.a.volume[j]=bSale.volume[j];
				}
				for(j=0;j<=bSale.ppu[0];j++){
					side.a.ppu[j]=bSale.ppu[j];
				}
                side.a.product=bSale.product&0x07;
                countImp[1]=0;
                read_date();
                read_time();
                for(j=0;j<=2;j++){
                    dateSale1[j]=date[j];
                    timeSale1[j]=time[j];
                }
			}else{
				for(j=0;j<=bSale.money[0];j++){
					side.b.money[j]=bSale.money[j];
				}
				for(j=0;j<=bSale.volume[0];j++){
					side.b.volume[j]=bSale.volume[j];
				}
				for(j=0;j<=bSale.ppu[0];j++){
					side.b.ppu[j]=bSale.ppu[j];
				}	
                side.b.product=bSale.product&0x07;
                countImp[2]=0;
                read_date();
                read_time();
                for(j=0;j<=2;j++){
                    dateSale2[j]=date[j];
                    timeSale2[j]=time[j];
                }
			}
        }
    }
}

/*
*********************************************************************************************************
*                                         get_total(uint8 dir)
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
uint8 get_total(uint8 dir){
    uint32 j,x;
    Pump_AL_PutChar(0x50|dir);
    CyDelay(880);
	if(dir==side.a.dir){
        side.a.total[0]=Pump_AL_GetRxBufferSize();
        for(x=1;x<=side.a.total[0];x++){
            side.a.total[x]=Pump_AL_rxBuffer[x-1];
        }
		switch(side.a.total[0]){
			case 34://5 digitos una manguera
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[33]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){			
					bufferLCD1.totalVolume1[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD1.totalVolume1[j]=',';
							j--;
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney1[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD1.totalMoney1[j]=(Pump_AL_rxBuffer[x+13]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu1[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD1.ppu1[j]=(Pump_AL_rxBuffer[x+22]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 1;
				}
				else{
					return 0;
				}
			break;
			
			case 64://5 digitos dos mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[63]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD1.totalVolume1[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD1.totalVolume1[j]=',';
							j--;
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney1[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD1.totalMoney1[j]=(Pump_AL_rxBuffer[13+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu1[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD1.ppu1[j]=(Pump_AL_rxBuffer[x+22]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD1.totalVolume2[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD1.totalVolume2[j]=',';
							j--;
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney2[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD1.totalMoney2[j]=(Pump_AL_rxBuffer[43+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu2[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD1.ppu2[j]=(Pump_AL_rxBuffer[x+52]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 2;
				}			
				else{
					return 0;
				}
			break;
				
			case 94://5 digitos tres mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[93]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD1.totalVolume1[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD1.totalVolume1[j]=',';
							j--;
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney1[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD1.totalMoney1[j]=(Pump_AL_rxBuffer[13+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu1[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD1.ppu1[j]=(Pump_AL_rxBuffer[x+22]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD1.totalVolume2[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD1.totalVolume2[j]=',';
							j--;
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney2[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD1.totalMoney2[j]=(Pump_AL_rxBuffer[43+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu2[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD1.ppu2[j]=(Pump_AL_rxBuffer[x+52]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD1.totalVolume3[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD1.totalVolume3[j]=',';
							j--;
							bufferLCD1.totalVolume3[j]=(Pump_AL_rxBuffer[64+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume3[j]=(Pump_AL_rxBuffer[64+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney3[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD1.totalMoney3[j]=(Pump_AL_rxBuffer[73+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu3[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD1.ppu3[j]=(Pump_AL_rxBuffer[x+82]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 3;					
				}			
				else{
					return 0;
				}			
			break;
				
			case 46://7 digitos una manguera
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[45]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD1.totalVolume1[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD1.totalVolume1[j]=',';
							j--;
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney1[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD1.totalMoney1[j]=(Pump_AL_rxBuffer[x+17]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu1[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD1.ppu1[j]=(Pump_AL_rxBuffer[x+30]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 1;
				}
				else{
					return 0;
				}			
			break;
				
			case 88://7 digitos dos mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[87]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD1.totalVolume1[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD1.totalVolume1[j]=',';
							j--;
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney1[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD1.totalMoney1[j]=(Pump_AL_rxBuffer[17+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu1[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD1.ppu1[j]=(Pump_AL_rxBuffer[x+30]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD1.totalVolume2[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD1.totalVolume2[j]=',';
							j--;
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney2[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD1.totalMoney2[j]=(Pump_AL_rxBuffer[59+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu2[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD1.ppu2[j]=(Pump_AL_rxBuffer[x+72]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 2;					
				}			
				else{
					return 0;
				}			
			break;
		
			case 130://7 digitos tres mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[129]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD1.totalVolume1[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD1.totalVolume1[j]=',';
							j--;
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney1[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD1.totalMoney1[j]=(Pump_AL_rxBuffer[17+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu1[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD1.ppu1[j]=(Pump_AL_rxBuffer[x+30]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD1.totalVolume2[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD1.totalVolume2[j]=',';
							j--;
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney2[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD1.totalMoney2[j]=(Pump_AL_rxBuffer[59+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu2[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD1.ppu2[j]=(Pump_AL_rxBuffer[x+72]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD1.totalVolume3[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD1.totalVolume3[j]=',';
							j--;
							bufferLCD1.totalVolume3[j]=(Pump_AL_rxBuffer[88+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD1.totalVolume3[j]=(Pump_AL_rxBuffer[88+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD1.totalMoney3[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD1.totalMoney3[j]=(Pump_AL_rxBuffer[101+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD1.ppu3[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD1.ppu3[j]=(Pump_AL_rxBuffer[x+114]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 3;
				}
				else{
					return 0;
				}
			break;
				
			default:
				return 0;	
			break;		
		}
	}else if(dir==side.b.dir){
        side.b.total[0]=Pump_AL_GetRxBufferSize();
        for(x=1;x<=side.b.total[0];x++){
            side.b.total[x]=Pump_AL_rxBuffer[x-1];
        }
		switch(side.b.total[0]){
			case 34://5 digitos una manguera
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[33]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){			
					bufferLCD2.totalVolume1[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD2.totalVolume1[j]=',';
							j--;
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney1[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD2.totalMoney1[j]=(Pump_AL_rxBuffer[x+13]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu1[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD2.ppu1[j]=(Pump_AL_rxBuffer[x+22]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 1;
				}
				else{
					return 0;
				}
			break;
			
			case 64://5 digitos dos mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[63]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD2.totalVolume1[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD2.totalVolume1[j]=',';
							j--;
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney1[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD2.totalMoney1[j]=(Pump_AL_rxBuffer[13+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu1[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD2.ppu1[j]=(Pump_AL_rxBuffer[x+22]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD2.totalVolume2[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD2.totalVolume2[j]=',';
							j--;
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney2[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD2.totalMoney2[j]=(Pump_AL_rxBuffer[43+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu2[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD2.ppu2[j]=(Pump_AL_rxBuffer[x+52]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 2;
				}			
				else{
					return 0;
				}
			break;
				
			case 94://5 digitos tres mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[93]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD2.totalVolume1[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD2.totalVolume1[j]=',';
							j--;
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney1[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD2.totalMoney1[j]=(Pump_AL_rxBuffer[13+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu1[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD2.ppu1[j]=(Pump_AL_rxBuffer[x+22]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD2.totalVolume2[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD2.totalVolume2[j]=',';
							j--;
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[34+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney2[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD2.totalMoney2[j]=(Pump_AL_rxBuffer[43+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu2[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD2.ppu2[j]=(Pump_AL_rxBuffer[x+52]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD2.totalVolume3[0]=9;
					j=9;
					for(x=0;x<=7;x++){
						if(x==2){
							bufferLCD2.totalVolume3[j]=',';
							j--;
							bufferLCD2.totalVolume3[j]=(Pump_AL_rxBuffer[64+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume3[j]=(Pump_AL_rxBuffer[64+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney3[0]=8;
					j=8;
					for(x=0;x<=7;x++){
						bufferLCD2.totalMoney3[j]=(Pump_AL_rxBuffer[73+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu3[0]=4;
                    j=4;
                    for(x=0;x<=3;x++){
                        bufferLCD2.ppu3[j]=(Pump_AL_rxBuffer[x+82]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 3;					
				}			
				else{
					return 0;
				}			
			break;
				
			case 46://7 digitos una manguera
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[45]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD2.totalVolume1[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD2.totalVolume1[j]=',';
							j--;
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[x+4]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney1[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD2.totalMoney1[j]=(Pump_AL_rxBuffer[x+17]&0x0F)+0x30;
						j--;
					}			
                    bufferLCD2.ppu1[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD2.ppu1[j]=(Pump_AL_rxBuffer[x+30]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 1;
				}
				else{
					return 0;
				}			
			break;
				
			case 88://7 digitos dos mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[87]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD2.totalVolume1[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD2.totalVolume1[j]=',';
							j--;
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney1[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD2.totalMoney1[j]=(Pump_AL_rxBuffer[17+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu1[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD2.ppu1[j]=(Pump_AL_rxBuffer[x+30]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD2.totalVolume2[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD2.totalVolume2[j]=',';
							j--;
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney2[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD2.totalMoney2[j]=(Pump_AL_rxBuffer[59+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu2[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD2.ppu2[j]=(Pump_AL_rxBuffer[x+72]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 2;					
				}			
				else{
					return 0;
				}			
			break;
		
			case 130://7 digitos tres mangueras
				if((Pump_AL_rxBuffer[0]==0xFF)&&(Pump_AL_rxBuffer[1]==0xF6)&&(Pump_AL_rxBuffer[129]==0xF0)&&(Pump_AL_rxBuffer[3]==0xF9)){
					bufferLCD2.totalVolume1[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD2.totalVolume1[j]=',';
							j--;
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume1[j]=(Pump_AL_rxBuffer[4+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney1[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD2.totalMoney1[j]=(Pump_AL_rxBuffer[17+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu1[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD2.ppu1[j]=(Pump_AL_rxBuffer[x+30]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD2.totalVolume2[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD2.totalVolume2[j]=',';
							j--;
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume2[j]=(Pump_AL_rxBuffer[46+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney2[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD2.totalMoney2[j]=(Pump_AL_rxBuffer[59+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu2[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD2.ppu2[j]=(Pump_AL_rxBuffer[x+72]&0x0F)+0x30;
                        j--;
                    }
					bufferLCD2.totalVolume3[0]=13;
					j=13;
					for(x=0;x<=11;x++){
						if(x==2){
							bufferLCD2.totalVolume3[j]=',';
							j--;
							bufferLCD2.totalVolume3[j]=(Pump_AL_rxBuffer[88+x]&0x0F)+0x30;
							j--;
						}else{
							bufferLCD2.totalVolume3[j]=(Pump_AL_rxBuffer[88+x]&0x0F)+0x30;
							j--;
						}
					}
					bufferLCD2.totalMoney3[0]=12;
					j=12;
					for(x=0;x<=11;x++){
						bufferLCD2.totalMoney3[j]=(Pump_AL_rxBuffer[101+x]&0x0F)+0x30;
						j--;
					}
                    bufferLCD2.ppu3[0]=6;
                    j=6;
                    for(x=0;x<=5;x++){
                        bufferLCD2.ppu3[j]=(Pump_AL_rxBuffer[x+114]&0x0F)+0x30;
                        j--;
                    }
					Pump_AL_ClearRxBuffer();
					return 3;
				}
				else{
					return 0;
				}
			break;
				
			default:
				return 0;	
			break;	
		}
	}else{
		return 0;
	}
}

/*
*********************************************************************************************************
*                         program(uint8 dir, uint8 grade, uint8 *value, uint8 preset)
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
uint8 program(uint8 dir, uint8 grade, uint8 *value, uint8 preset){
	uint8 buffer[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},size,temp,decimal=0;
    uint32 i,j;
	temp=4;
	if(((digits==5)||(digits==6))&&(preset==1)){
		temp=3;
	}
	if((digits==5)&&(preset==2)){
		temp=5;
	}
	if((digits==7)&&(preset==1)){
		temp=0;
	}
	if((digits==7)&&(preset==2)){
		temp=2;
	}	
	decimal=0;
	for(i=1;i<=value[0];i++){
		if(decimal>=1){
			decimal++;
			value[i]=value[i+1];
		}				
		if(value[i]==','){
			decimal++;
			value[i]=value[i+1];
		}
	}
	if(decimal>=1){
		decimal=decimal-1;
		value[0]=value[0]-1;
	}	
	if(decimal>=4){
		return 0;
	}	
	if(digits!=7){		
		buffer[0]=0xFF;
		buffer[1]=(0xE0|temp);
		buffer[2]=(0xF0|preset);
		buffer[3]=0xF4;
		buffer[4]=0xF8;	
		for(i=5;i<((5+decimalMoney)-decimal);i++){
			buffer[i]=0xE0;
		}
		for(j=i;j<(i+value[0]);j++){
			buffer[j]=(0xE0|(value[value[0]-(j-i)]&0x0F));	
		}
		for(i=j;i<=10;i++){
			buffer[i]=0xE0;
		}
		buffer[11]=0xFB;
		buffer[12]=0;
		for(i=0;i<=11;i++){
			buffer[12]+=(buffer[i]&0x0F);	
		}
		buffer[12]=(((~buffer[12])+1)&0x0F)|0xE0;
		buffer[13]=0xF0;
		size=13;
		if(digits==5){
			buffer[10]=0xFB;
			buffer[11]=0;
			for(i=0;i<=10;i++){
				buffer[11]+=(buffer[i]&0x0F);	
			}
			buffer[11]=(((~buffer[11])+1)&0x0F)|0xE0;
			buffer[12]=0xF0;
			size=12;		
		}
		if(preset==1){
			if(decimal==3){
				value[0]=value[0]-1;
				decimal=decimal-1;
			}
			buffer[4]=0xF6;
			buffer[5]=(0xE0|(grade-1));
			buffer[6]=0xF8;
			for(i=7;i<((7+(decimalVolume-1))-decimal);i++){
				buffer[i]=0xE0;
			}
			for(j=i;j<(i+value[0]);j++){
				buffer[j]=(0xE0|(value[value[0]-(j-i)]&0x0F));	
			}
			for(i=j;i<=11;i++){
				buffer[i]=0xE0;
			}			
			buffer[12]=0xFB;
			buffer[13]=0;
			for(i=0;i<=12;i++){
				buffer[13]+=(buffer[i]&0x0F);	
			}
			buffer[13]=(((~buffer[13])+1)&0x0F)|0xE0;			
			buffer[14]=0xF0;
			size=14;
		}	
	}	
	if(digits==7){
		buffer[0]=0xFF;
		buffer[1]=(0xE0|temp);
		buffer[2]=(0xF0|preset);
		buffer[3]=0xF4;
		buffer[4]=0xF8;	
		for(i=5;i<((5+decimalMoney)-decimal);i++){
			buffer[i]=0xE0;
		}
		for(j=i;j<(i+value[0]);j++){
			buffer[j]=(0xE0|(value[value[0]-(j-i)]&0x0F));	
		}
		for(i=j;i<=12;i++){
			buffer[i]=0xE0;
		}
		buffer[13]=0xFB;
		buffer[14]=0;
		for(i=0;i<=13;i++){
			buffer[14]+=(buffer[i]&0x0F);	
		}
		buffer[14]=(((~buffer[14])+1)&0x0F)|0xE0;
		buffer[15]=0xF0;
		size=15;
		if(preset==1){
			buffer[4]=0xF6;
			buffer[5]=(0xE0|(grade-1));
			buffer[6]=0xF8;
			for(i=7;i<((7+decimalVolume)-decimal);i++){
				buffer[i]=0xE0;
			}
			for(j=i;j<(i+value[0]);j++){
				buffer[j]=(0xE0|(value[value[0]-(j-i)]&0x0F));	
			}
			for(i=j;i<=14;i++){
				buffer[i]=0xE0;
			}			
			buffer[15]=0xFB;
			buffer[16]=0;
			for(i=0;i<=15;i++){
				buffer[16]+=(buffer[i]&0x0F);	
			}
			buffer[16]=(((~buffer[16])+1)&0x0F)|0xE0;			
			buffer[17]=0xF0;
			size=17;
		}		
	}	
	Pump_AL_PutChar(0x20|dir);	
    CyDelay(50);
    if(Pump_AL_GetRxBufferSize()>=1){
		if(Pump_AL_ReadRxData()==(0xD0|dir)){
            Pump_AL_ClearRxBuffer();
            for(i=0;i<=size;i++){
               	Pump_AL_PutChar(buffer[i]);
            }
			CyDelay(50);
			if(get_state(dir)==0){
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
*                                         uint8 price_change(uint8 dir)
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
uint8 price_change(uint8 dir){
	uint8 buffer[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},size;
    uint32 x,y;
	if(digits!=7){
		buffer[0]=0xFF;
		buffer[1]=0xE5;
		buffer[2]=0xF4;
		buffer[3]=0xF6;
		buffer[4]=(0xE0|((bSale.product&0x0f)-1));
		buffer[5]=0xF7;	
		if(ppux10==0){
			for(x=6;x<(6+valuePPU[0]);x++){
				buffer[x]=(0xE0|(valuePPU[valuePPU[0]-(x-6)]&0x0F));
			}
		}
		else{
			valuePPU[0]=valuePPU[0]-1;
			for(x=6;x<(6+valuePPU[0]);x++){
				buffer[x]=(0xE0|(valuePPU[valuePPU[0]-(x-6)]&0x0F));
			}	
		}
		for(y=x;y<=9;y++){
			buffer[y]=0xE0;	
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
		buffer[0]=0xFF;
		buffer[1]=0xE3;
		buffer[2]=0xF4;
		buffer[3]=0xF6;
		buffer[4]=(0xE0|((bSale.product&0x0f)-1));
		buffer[5]=0xF7;		
		for(x=6;x<(6+valuePPU[0]);x++){
			buffer[x]=(0xE0|(valuePPU[valuePPU[0]-(x-6)]&0x0F));
		}	
		for(y=x;y<=11;y++){
			buffer[y]=0xE0;	
		}		
		buffer[12]=0xFB;
		for(x=0;x<=12;x++){
			buffer[13]+=(buffer[x]&0x0F);	
		}
		buffer[13]=(((~buffer[13])+1)&0x0F)|0xE0;
		buffer[14]=0xF0;
		size=14;
	}	
	Pump_AL_PutChar(0x20|dir);	
    CyDelay(100);
    if(Pump_AL_GetRxBufferSize()>=1){
		if(Pump_AL_ReadRxData()==(0xD0|dir)){
            Pump_AL_ClearRxBuffer();
            for(x=0;x<=size;x++){
               	Pump_AL_PutChar(buffer[x]); 	
            }
			CyDelay(50);
			if(get_state(dir)==0){
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

/* [] END OF FILE */
