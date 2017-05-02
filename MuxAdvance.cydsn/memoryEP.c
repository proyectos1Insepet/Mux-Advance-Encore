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
* Filename      : memoryEP.c
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
#include <math.h>

/*
*********************************************************************************************************
*                                         void write_settings()
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

void write_settings(){
    uint8 memoryData[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},x;
    int y;
    
    memoryData[0]=digits;
    memoryData[1]=((decimalMoney&0x07)*10)+(decimalVolume&0x07);
    memoryData[2]=ppux10;
    memoryData[3]=productNumber;
    memoryData[4]=symbols[0];
    memoryData[5]=symbols[1];
    for(x=3;x>0;x--){
        memoryData[6]+=((ipAdress[x-1]&0x0f)*pow(10,(3-x)));
        memoryData[7]+=((ipAdress[x+2]&0x0f)*pow(10,(3-x)));
        memoryData[8]+=((ipAdress[x+5]&0x0f)*pow(10,(3-x)));
        memoryData[9]+=((ipAdress[x+8]&0x0f)*pow(10,(3-x)));
    }
    memoryData[10]=turn;
    memoryData[11]=typeIdSeller;
    if((y=idSeller[0])==0){
        y=1;
        idSeller[1]='0';
    }
    if(y<20){
        for(x=20;x>0;x--){
            idSeller[x]=idSeller[y];
            y--;
            if(y<=0){
                while(x>1){
                    x--;
                    idSeller[x]='0';
                }
                break;
            }
        }
    }
    idSeller[0]=20;
    memoryData[12]=idSeller[1];
    memoryData[13]=idSeller[2];
    memoryData[14]=idSeller[3];
    memoryData[15]=idSeller[4];
    EEPROM_1_Write(memoryData,59);
    for(x=0;x<16;x++){
        memoryData[x]=idSeller[5+x];
    }
    EEPROM_1_Write(memoryData,60);
    y=presetFast[0][0];
    for(x=7;x>=0;x--){
        memoryData[x]=presetFast[0][y];
        y--;
        if(y<=0){
            while(x>0){
                x--;
                memoryData[x]='0';
            }
            break;
        }
    }
    y=presetFast[1][0];
    for(x=15;x>=8;x--){
        memoryData[x]=presetFast[1][y];
        y--;
        if(y<=0){
            while(x>8){
                x--;
                memoryData[x]='0';
            }
            break;
        }
    }
    EEPROM_1_Write(memoryData,57);
    y=presetFast[2][0];
    for(x=7;x>=0;x--){
        memoryData[x]=presetFast[2][y];
        y--;
        if(y<=0){
            while(x>0){
                x--;
                memoryData[x]='0';
            }
            break;
        }
    }
    memoryData[8]=screen[0];
    memoryData[9]=screen[1];
    memoryData[10]=passwordSeller[1];
    memoryData[11]=passwordSeller[2];
    memoryData[12]=passwordSeller[3];
    memoryData[13]=passwordSeller[4];
    EEPROM_1_Write(memoryData,58);
}

/*
*********************************************************************************************************
*                                         void write_ppus()
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

void write_ppus(){
    uint8 memoryData[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},x,y;

    y=1;
    for(x=0;x<6;x++){
        memoryData[x]=side.a.ppuAuthorized[0][y];
        y++;
    }
    y=1;
    for(x=6;x<12;x++){
        memoryData[x]=side.a.ppuAuthorized[1][y];
        y++;
    }
    y=1;
    for(x=12;x<16;x++){
        memoryData[x]=side.a.ppuAuthorized[2][y];
        y++;
    }
    EEPROM_1_Write(memoryData,61);
    memoryData[0]=side.a.ppuAuthorized[2][5];
    memoryData[1]=side.a.ppuAuthorized[2][6];
    y=1;
    for(x=2;x<8;x++){
        memoryData[x]=side.a.ppuAuthorized[3][y];
        y++;
    }
    y=1;
    for(x=8;x<14;x++){
        memoryData[x]=side.b.ppuAuthorized[0][y];
        y++;
    }
    memoryData[14]=side.b.ppuAuthorized[1][1];
    memoryData[15]=side.b.ppuAuthorized[1][2];
    EEPROM_1_Write(memoryData,62);
    y=3;
    for(x=0;x<4;x++){
        memoryData[x]=side.b.ppuAuthorized[1][y];
        y++;
    }
    y=1;
    for(x=4;x<10;x++){
        memoryData[x]=side.b.ppuAuthorized[2][y];
        y++;
    }
    y=1;
    for(x=10;x<16;x++){
        memoryData[x]=side.b.ppuAuthorized[3][y];
        y++;
    }
    EEPROM_1_Write(memoryData,63);
}

/*
*********************************************************************************************************
*                                         void write_buttons()
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

void write_buttons(){
    uint8 memoryData[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},x,y,z;
    z=0;
    for(y=0;y<=113;y++){
        for(x=0;x<8;x++){
            memoryData[x]= button[y][x];
        }
        y++;
        for(x=8;x<16;x++){
            memoryData[x]= button[y][x-8];
        }
        EEPROM_1_Write(memoryData,z);
        z++;
    }
}