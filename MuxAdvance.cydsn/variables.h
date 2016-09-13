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
* Filename      : variables.h
* Version       : V1.00
* Programmer(s) : 
                  
*********************************************************************************************************
*/

#ifndef VARIABLES_H
#define VARIABLES_H
#include <device.h>	
    	
 
/*
*********************************************************************************************************
*                                    Declaracion de variables
*
* Description : Variables que no estan atadas a una estructura.
*               
*
*********************************************************************************************************
*/
    volatile uint64 countAnimation1; //Contador de animacion para pantalla 1
    volatile uint64 countAnimation2; //Contador de animacion para pantalla 2
    volatile uint64 countBeagleTX;   //Contador de tiempos de envio de informacion al beagle
    volatile uint32 flowLCD1;        //Flujo pantalla 1
    volatile uint32 flowLCD2;        //Flujo pantalla 2
    volatile uint8 digits;           //Version de digitos del surtidor
    volatile uint8 ipAdress[12];     //Version de digitos del surtidor
    volatile uint8 numberKeys1;      //Cantidad de digitos a digitar en los teclados numericos para pantalla 1
    volatile uint8 numberKeys2;      //Cantidad de digitos a digitar en los teclados numericos para pantalla 1
    volatile uint8 flagPoint1;       //Bandera que indica si ya se digito una coma en el teclado numerico de la pantalla 1 
    volatile uint8 flagPoint2;       //Bandera que indica si ya se digito una coma en el teclado numerico de la pantalla 2 
    volatile uint8 button[120][8];   //Nombre de Botones para las pantallas
    volatile uint8 productNumber;    //Numero de productos que tiene el surtidor
    uint8 presetFast[3][10];         //Presets rapidos establecidos P1[0] P2[1] y P3[2]
    volatile uint32 delayPicture1;    //Tiempo que demora una imagen informativa en el caso 0 del polling_LCD1
    volatile uint32 delayPicture2;    //Tiempo que demora una imagen informativa en el caso 0 del polling_LCD2
    volatile uint8 ppux10;           //ppu por 10, 1=Habilitado 0=Inhabilitado
    volatile uint8 decimalMoney;     //Decimales de Dinero
    volatile uint8 decimalVolume;    //Decimales de Volumen 
    volatile uint8 symbols[2];       //Simbolos visualizados en Teclado Numerico [0]=Dinero[1]=Volumen
    volatile uint8 time[3];          //Hora 
    volatile uint8 date[3];          //Fecha
    volatile uint8 screen[2];        //Pantallas 0:Inhabilitada 1:Habilitada => [0]Tipo de vehiculo [1]Ingrese N° Venta Forma
    volatile uint8 residue[14];      //Residuo de operacion resta 
    uint8 temporal[30];              //Utilizada para realizar operaciones temporales 
    volatile uint8 turn;             //Habilita si el turno esta 1=abierto o 0=cerrado
    volatile uint8 idSeller[25];     //Identificacion de vendedor
    volatile uint8 typeIdSeller;     //Tipo de identificacion del vendedor
    volatile uint8 passwordSeller[8];//Contraseña Vendedor
    volatile uint8 flagResetMux;     //Bandera que habilita Resetar el MUX
    
/*
*********************************************************************************************************
*
*                       Estructura buffer para varaibles en las LCD's
*          
*********************************************************************************************************
*/
struct buffer{
    uint8 idType;                   //Tipo de metodo de Identificacion 
    uint8 idSerial[25];             //Serial del metodo de identificacion
    uint8 wayToPay;                 //Forma de pago seleccionada
    uint8 selectedSale[10];         //N° de Venta seleccionada a discriminar para forma de pago
    uint8 moneySelectedSale[10];    //Valor recibido de la venta seleccionada a discriminar para forma de pago,mismo para enviar
    uint8 serialSelectedSale[25];   //Serial ID de la venta seleccionada a discriminar para forma de pago
    uint8 keyboardWayToPay;         //Tipo de teclado a mostrar atado a la forma de pago
    uint8 authorizationFlag;        //Bandera que habilita si autoriza o no la peticion
    uint8 valueKeys[21];            //Valor temporal de los numeros digitados en el teclado numerico
    uint8 vehicleType;              //Tipo de vehiculo, 1-6
    uint8 saleType;                 //Tipo de venta, 1=Efectivo 2=Credito 3=Calibracion 4=Autoservicio
    uint8 salePerform;              //Venta a realiza, 1=Combustible 2=Canasta 3=Calibracion
    uint8 presetType[2];            //Tipo de programacion, 1V=Volumen 2D=Dinero 2F=Lleno 21=P1 22=P2 23=P3
    uint8 presetValue[2][10];       //Valor de programacion a programar, [0][n]=Valor para surtidor, [1][n]=Valor a mostrar
    uint8 productType;              //Tipo de producto que se eligio en la pantalla (anticontaminante)
    uint8 stateMux[5];              //Estado en el que se encuentra el MUX, cantidad [0], prioridad [1], en cola [2,3,4]
    uint8 dateLiftHandle[6];        //Fecha en la que se sube la manija
    uint8 dateDownHandle[6];        //Fecha en la que se baja la manija
    uint8 timeLiftHandle[6];        //Hora en la que se sube la manija
    uint8 timeDownHandle[6];        //Hora en la que se baja la manija
    uint8 totalMoneyPrevious[14];   //Total de dinero antes de hacer la venta
    uint8 totalVolumePrevious[14];  //Total de volumen antes de hacer la venta
    uint8 totalPPUPrevious[14];     //Total de PPU antes de hacer la venta
    uint8 totalMoneyAfter[14];      //Total de dinero depues de hacer la venta
    uint8 totalVolumeAfter[14];     //Total de volumen depues de hacer la venta
    uint8 totalPPUAfter[14];        //Total de PPU depues de hacer la venta
    uint8 licenceSale[11];          //Placa venta
    uint8 mileageSale[11];          //Kilometraje venta
    uint8 identySale[11];           //CC-NIT-Identificacion venta
    uint8 flagKeyboard;             //Bandera que habilita que teclado se digito al colocar datos mientras tanquea, y cantidad canastilla
    uint8 flagPayment;              //Bandera que habilita que imagen de formas de pago se esta visualizando (1-4) 
    uint8 flagLiftHandle;           //Bandera para indicar el estado de sube manija
    uint8 flagEndSale;              //Bandera que indica que la venta finalizo, se valida en introducir placa
    uint8 message[65];              //Mensaje a puplicar en la LCD
    uint8 moneyQuota[10];           //Cupo de dinero autorizado para ventas credito
    uint8 volumeQuota[10];          //Cupo de volumen autorizado para ventas credito
    uint8 ppuQuota[10];             //PPU autorizado para ventas credito
    uint8 serialMarket[3][25];      //Serial de producto digitado en canastilla, para tres productos
    uint8 productMarket[3][25];     //Nombre de producto digitado en canastilla, para tres productos
    uint8 quantityMarket[3][5];     //Cantidad de productos digitados en canastilla, para tres productos
    uint8 quantAvailableMark[3][5]; //Cantidad disponible de productos canastilla, para tres productos
    uint8 priceTotalMarket[3][10];  //Precio total por producto digitados en canastilla, para tres productos
    uint8 priceUnitMarket[3][10];   //Precio unitario por producto digitados en canastilla, para tres productos
    uint8 totalMarket[10];          //Total de dinero para productos digitados en canastilla
    uint8 flagProductmarket;        //Bandera 
    uint8 priceConsign[15];         //Precio a consignar
    uint8 printers[2];              //Impresoras seleccionadas para imprimir
    uint8 flagChangePPU;            //Bandera que habilita cambiar PPUs cuando los envie el beagle
};

struct buffer bufferLCD1;
struct buffer bufferLCD2;

/*
*********************************************************************************************************
*
*                            Estructura posiciones del surtidor
*          
*********************************************************************************************************
*/

struct position{
    uint8 dir;                      //Direccion
    uint8 totalsHandle[4][3][15];   //Totales Manguera: [0-3] // [0]=Volumen [1]=Dinero [2]=PPU // [8 ó 12 - 4 ó 5]
    uint8 ppuAuthorized[4][8];      //PPU autorizados [Manguera 0-3][Datos]
    uint8 moneySale[14];            //Venta de Dinero vendido
    uint8 volumeSale[14];           //Venta de Volumen vendido
    uint8 ppuSale[14];              //Venta de PPU vendido
    uint8 productSale;              //Venta de Producto vendido
};

struct pump{
   struct position a;
   struct position b;
};

struct pump side;       //lado del surtidor

#endif

//[] END OF FILE
