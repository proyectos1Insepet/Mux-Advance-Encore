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
* Filename      : operations.c
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
#include <math.h>
//#include <stdio.h>
/*
*********************************************************************************************************
*                       uint8 compare_ppu(uint8 *ppuPump,uint8 handle,uint8 dir)
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

uint8 compare_ppu(uint8 *ppuPump,uint8 handle,uint8 pos){
    uint8 x,y,tempPpu[7];
    if(pos==side.a.dir){
        for(x=0;x<=6;x++){
            tempPpu[x]=side.a.ppuAuthorized[handle-1][x];
        }
    }else{
        for(x=0;x<=6;x++){
            tempPpu[x]=side.b.ppuAuthorized[handle-1][x];
        }
    }
    for(x=1;x<=6;x++){
        if(tempPpu[x]==','){
            for(y=x;y>=2;y--){
                tempPpu[y]=tempPpu[y-1];
            }
            tempPpu[1]='0';
            break;
        }
    }
    ppuPump[0]=ppuPump[0]&0x07;
    if(ppuPump[0]!=6){
        for(x=6;x>=3;x--){
            ppuPump[x]=ppuPump[x-2];
        }
        ppuPump[0]=6;
        ppuPump[1]='0';
        ppuPump[2]='0';
    }
    if(ppux10==1){
        for(x=1;x<ppuPump[0];x++){
            ppuPump[x]=ppuPump[x+1];
        }
        ppuPump[x]='0';
    }
    for(x=1;x<=6;x++){
        if(ppuPump[x]!=tempPpu[x]){
            return 0;
        }
    }
    return 1;
}


/*
*********************************************************************************************************
*                      uint8 subtraction(uint8 *minuendo,uint8 *sustraendo,uint8 decimal)
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

uint8 subtraction(uint8 *minuendo,uint8 *sustraendo){
    uint8 t_residue[14],t_minuendo[14],t_sustraendo[14],carrier,maximumValue[12]={9,9,9,9,9,9,9,9,9,9,9,9},x,y;
    t_minuendo[0]=minuendo[0]&0x0f;
    t_sustraendo[0]=sustraendo[0]&0x0f;
    y=1;
    for(x=t_minuendo[0];x>0;x--){
        t_minuendo[y]=minuendo[x]&0x0f;
        y++;
    }
    y=1;
    for(x=t_sustraendo[0];x>0;x--){
        t_sustraendo[y]=sustraendo[x]&0x0f;
        y++;
    }
	carrier=0;
	if(t_minuendo[t_minuendo[0]]<t_sustraendo[t_minuendo[0]]){
		for(x=1;x<=t_minuendo[0];x++){
			if((maximumValue[x]-carrier)<t_sustraendo[x]){
				t_residue[x]=((maximumValue[x]+10)-carrier)-t_sustraendo[x];
				carrier=1;			
			}
			else{
				t_residue[x]=(maximumValue[x]-carrier)-t_sustraendo[x];
				carrier=0;
			}
		}
		t_residue[0]=t_minuendo[0];	
		for(x=1;x<=t_minuendo[0];x++){
			t_residue[x]=t_minuendo[x]+t_residue[x]+carrier;
			if(t_residue[x]>9){
				t_residue[x]=t_residue[x]-10;
				carrier=1;
			}
			else{
				carrier=0;
			}
		}
		if(carrier==1){
			t_residue[x+1]=1;
			t_residue[0]=t_minuendo[0]+1;
		}
		else{
			t_residue[0]=t_minuendo[0];
		}			
	}
	else{
		for(x=1;x<=t_minuendo[0];x++){
			if((t_minuendo[x]-carrier)<t_sustraendo[x]){
				t_residue[x]=((t_minuendo[x]+10)-carrier)-t_sustraendo[x];
				carrier=1;			
			}
			else{
				t_residue[x]=(t_minuendo[x]-carrier)-t_sustraendo[x];
				carrier=0;
			}
		}
		t_residue[0]=t_minuendo[0];
	}
    residue[0]=t_residue[0]&0x0f;
    y=1;
    for(x=residue[0];x>0;x--){
        residue[y]=(t_residue[x]&0x0f)+0x30;
        y++;
    }
    for(x=1;x<=residue[0];x++){
        if((residue[x])!='0'){
            return 0;
        }
    }
    return 1;
}

/*
*********************************************************************************************************
*                                    uint8 higher(uint8 *value1,uint8 *value2)
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

uint8 higher(uint8 *value1,uint8 *value2){
    double tempV1=0,tempV2=0;
    uint8 x,y;
    y=value1[0];
    for(x=value1[0];x>0;x--){
        if(value1[x]!=','){
            tempV1+=((double)(value1[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV1/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value2[0];
    for(x=value2[0];x>0;x--){
        if(value2[x]!=','){
            tempV2+=((double)(value2[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV2/=pow(10,(y-x));
            y=x-1;
        }
    }
    if(tempV1>tempV2){
        return 1;
    }else if(tempV1<tempV2){
        return 2;
    }else{
        return 0;
    }
}

/*
*********************************************************************************************************
*                         void multiplication_market(uint8 *value1,uint8 *value2)
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

void multiplication_market(uint8 *value1,uint8 *value2){
    /*double tempV1=0,tempV2=0,total;
    int x,y;
    char valueTotal[16];
    char decimal[5]="%.0f*";
    decimal[2]=decimalMoney+0x30;
    y=value1[0];
    for(x=value1[0];x>0;x--){
        if(value1[x]!=','){
            tempV1+=((double)(value1[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV1/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value2[0];
    for(x=value2[0];x>0;x--){
        if(value2[x]!=','){
            tempV2+=((double)(value2[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV2/=pow(10,(y-x));
            y=x-1;
        }
    }
    total=tempV1*tempV2;
    sprintf(valueTotal,decimal,total);
    for(y=0;y<16;y++){
        if(valueTotal[y]=='*'){
            break;
        }
    }
    for(x=1;x<=y;x++){
        temporal[x]=valueTotal[x-1];
        if(valueTotal[x-1]=='.'){
            temporal[x]=',';
        }
    }
    for(x=8;x>0;x--){
        temporal[x]=temporal[y];
        y--;
        if(y<=0){
            while(x>1){
                x--;
                temporal[x]='0';
            }
            break;
        }
    }
    temporal[0]=8;
    CyWdtClear();*/
    /*double tempV1=0,tempV2=0,total;
    int x,y;
    y=value1[0];
    for(x=value1[0];x>0;x--){
        if(value1[x]!=','){
            tempV1+=((double)(value1[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV1/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value2[0];
    for(x=value2[0];x>0;x--){
        if(value2[x]!=','){
            tempV2+=((double)(value2[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV2/=pow(10,(y-x));
            y=x-1;
        }
    }
    total=tempV1*tempV2;
    for(x=1;x<=8;x++){
		temporal[9-x] = (((int)total%(int)pow(10,x))/pow(10,x-1))+0x30;
	}
	temporal[0]=8;*/
    uint8 tempValue1[10],tempValue2[10],tempValue[4][15]={{0},{0}},x,y,z,tenth;
    tempValue1[0]=value1[0];
    for(x=value1[0];x>0;x--){
        tempValue1[x]=value1[x]&0x0f;
        if(value1[x]==','){
            while(x>0){
                tempValue1[x]=value1[x-1]&0x0f;
                x--;
            }
            tempValue1[1]=0;
            break;
        }
    }
    tempValue2[0]=value2[0];
    for(x=value2[0];x>0;x--){
        tempValue2[x]=value2[x]&0x0f;
        if(value2[x]==','){
            while(x>0){
                tempValue2[x]=value2[x-1]&0x0f;
                x--;
            }
            tempValue2[1]=0;
            break;
        }
    }
    z=3;
    for(x=3;x>0;x--){
        tenth=0;
        for(y=8;y>0;y--){
            tempValue[3-x][y+z]=((tempValue1[x]*tempValue2[y])+tenth)%10;
            tenth=((tempValue1[x]*tempValue2[y])+tenth)/10;
        }
        z--;
    }
    tenth=0;
    for(x=11;x>0;x--){
        tempValue[3][x]=((tempValue[0][x]+tempValue[1][x])+(tempValue[2][x]+tenth))%10;
        tenth=((tempValue[0][x]+tempValue[1][x])+(tempValue[2][x]+tenth))/10;
    }
    y=11;
    z=0;
    for(x=8;x>0;x--){
        temporal[x]=(tempValue[3][y]&0x0f)+0x30;
        y--;
        z++;
        if(z==decimalMoney){
            x--;
            temporal[x]=',';
        }
    }
    temporal[0]=8;
}

/*
*********************************************************************************************************
*                    void addition_market(uint8 *value1,uint8 *value2, uint8 *value3)
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

void addition_market(uint8 *value1,uint8 *value2, uint8 *value3){
   /* double tempV1=0,tempV2=0,tempV3=0,total;
    int x,y;
    char valueTotal[12];
    char decimal[5]="%.0f*";
    decimal[2]=decimalMoney+0x30;
    y=value1[0];
    for(x=value1[0];x>0;x--){
        if(value1[x]!=','){
            tempV1+=((double)(value1[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV1/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value2[0];
    for(x=value2[0];x>0;x--){
        if(value2[x]!=','){
            tempV2+=((double)(value2[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV2/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value3[0];
    for(x=value3[0];x>0;x--){
        if(value3[x]!=','){
            tempV3+=((double)(value3[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV3/=pow(10,(y-x));
            y=x-1;
        }
    }
    total=tempV1+tempV2+tempV3;
    sprintf(valueTotal,decimal,total);
    for(y=0;y<12;y++){
        if(valueTotal[y]=='*'){
            break;
        }
    }
    for(x=1;x<=y;x++){
        temporal[x]=valueTotal[x-1];
        if(valueTotal[x-1]=='.'){
            temporal[x]=',';
        }
    }
    for(x=9;x>0;x--){
        temporal[x]=temporal[y];
        y--;
        if(y<=0){
            while(x>1){
                x--;
                temporal[x]='0';
            }
            break;
        }
    }
	temporal[0]=9;
    CyWdtClear();*/
    
    /*double tempV1=0,tempV2=0,tempV3=0,total;
    int x,y;
    y=value1[0];
    for(x=value1[0];x>0;x--){
        if(value1[x]!=','){
            tempV1+=((double)(value1[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV1/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value2[0];
    for(x=value2[0];x>0;x--){
        if(value2[x]!=','){
            tempV2+=((double)(value2[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV2/=pow(10,(y-x));
            y=x-1;
        }
    }
    y=value3[0];
    for(x=value3[0];x>0;x--){
        if(value3[x]!=','){
            tempV3+=((double)(value3[x]&0x0f)*pow(10,(y-x)));
        }else{
            tempV3/=pow(10,(y-x));
            y=x-1;
        }
    }
    total=tempV1+tempV2+tempV3;
    for(x=1;x<=9;x++){
        temporal[10-x]=(((int)total%(int)pow(10,x))/pow(10,x-1))+0x30;
    }
	temporal[0]=9;*/
    uint8 tempValue1[10],tempValue2[10],tempValue3[10],tempValue[10]={0},x,y,z,tenth;
    tempValue1[0]=value1[0];
    for(x=value1[0];x>0;x--){
        tempValue1[x]=value1[x]&0x0f;
        if(value1[x]==','){
            while(x>0){
                tempValue1[x]=value1[x-1]&0x0f;
                x--;
            }
            tempValue1[1]=0;
            break;
        }
    }
    tempValue2[0]=value2[0];
    for(x=value2[0];x>0;x--){
        tempValue2[x]=value2[x]&0x0f;
        if(value2[x]==','){
            while(x>0){
                tempValue2[x]=value2[x-1]&0x0f;
                x--;
            }
            tempValue2[1]=0;
            break;
        }
    }
    tempValue3[0]=value3[0];
    for(x=value3[0];x>0;x--){
        tempValue3[x]=value3[x]&0x0f;
        if(value3[x]==','){
            while(x>0){
                tempValue3[x]=value3[x-1]&0x0f;
                x--;
            }
            tempValue3[1]=0;
            break;
        }
    }
    tenth=0;
    for(x=8;x>0;x--){
        tempValue[x]=((tempValue1[x]+tempValue2[x])+(tempValue3[x]+tenth))%10;
        tenth=((tempValue1[x]+tempValue2[x])+(tempValue3[x]+tenth))/10;
    }
    y=8;
    z=0;
    temporal[1]=0;
    for(x=9;x>0;x--){
        temporal[x]=(tempValue[y]&0x0f)+0x30;
        y--;
        z++;
        if(z==decimalMoney){
            x--;
            temporal[x]=',';
        }
    }
    temporal[0]=9;
}