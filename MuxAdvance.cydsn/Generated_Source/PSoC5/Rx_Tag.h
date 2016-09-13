/*******************************************************************************
* File Name: Rx_Tag.h  
* Version 2.10
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_Rx_Tag_H) /* Pins Rx_Tag_H */
#define CY_PINS_Rx_Tag_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Rx_Tag_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Rx_Tag__PORT == 15 && ((Rx_Tag__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Rx_Tag_Write(uint8 value) ;
void    Rx_Tag_SetDriveMode(uint8 mode) ;
uint8   Rx_Tag_ReadDataReg(void) ;
uint8   Rx_Tag_Read(void) ;
uint8   Rx_Tag_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Rx_Tag_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Rx_Tag_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Rx_Tag_DM_RES_UP          PIN_DM_RES_UP
#define Rx_Tag_DM_RES_DWN         PIN_DM_RES_DWN
#define Rx_Tag_DM_OD_LO           PIN_DM_OD_LO
#define Rx_Tag_DM_OD_HI           PIN_DM_OD_HI
#define Rx_Tag_DM_STRONG          PIN_DM_STRONG
#define Rx_Tag_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Rx_Tag_MASK               Rx_Tag__MASK
#define Rx_Tag_SHIFT              Rx_Tag__SHIFT
#define Rx_Tag_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Rx_Tag_PS                     (* (reg8 *) Rx_Tag__PS)
/* Data Register */
#define Rx_Tag_DR                     (* (reg8 *) Rx_Tag__DR)
/* Port Number */
#define Rx_Tag_PRT_NUM                (* (reg8 *) Rx_Tag__PRT) 
/* Connect to Analog Globals */                                                  
#define Rx_Tag_AG                     (* (reg8 *) Rx_Tag__AG)                       
/* Analog MUX bux enable */
#define Rx_Tag_AMUX                   (* (reg8 *) Rx_Tag__AMUX) 
/* Bidirectional Enable */                                                        
#define Rx_Tag_BIE                    (* (reg8 *) Rx_Tag__BIE)
/* Bit-mask for Aliased Register Access */
#define Rx_Tag_BIT_MASK               (* (reg8 *) Rx_Tag__BIT_MASK)
/* Bypass Enable */
#define Rx_Tag_BYP                    (* (reg8 *) Rx_Tag__BYP)
/* Port wide control signals */                                                   
#define Rx_Tag_CTL                    (* (reg8 *) Rx_Tag__CTL)
/* Drive Modes */
#define Rx_Tag_DM0                    (* (reg8 *) Rx_Tag__DM0) 
#define Rx_Tag_DM1                    (* (reg8 *) Rx_Tag__DM1)
#define Rx_Tag_DM2                    (* (reg8 *) Rx_Tag__DM2) 
/* Input Buffer Disable Override */
#define Rx_Tag_INP_DIS                (* (reg8 *) Rx_Tag__INP_DIS)
/* LCD Common or Segment Drive */
#define Rx_Tag_LCD_COM_SEG            (* (reg8 *) Rx_Tag__LCD_COM_SEG)
/* Enable Segment LCD */
#define Rx_Tag_LCD_EN                 (* (reg8 *) Rx_Tag__LCD_EN)
/* Slew Rate Control */
#define Rx_Tag_SLW                    (* (reg8 *) Rx_Tag__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Rx_Tag_PRTDSI__CAPS_SEL       (* (reg8 *) Rx_Tag__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Rx_Tag_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Rx_Tag__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Rx_Tag_PRTDSI__OE_SEL0        (* (reg8 *) Rx_Tag__PRTDSI__OE_SEL0) 
#define Rx_Tag_PRTDSI__OE_SEL1        (* (reg8 *) Rx_Tag__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Rx_Tag_PRTDSI__OUT_SEL0       (* (reg8 *) Rx_Tag__PRTDSI__OUT_SEL0) 
#define Rx_Tag_PRTDSI__OUT_SEL1       (* (reg8 *) Rx_Tag__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Rx_Tag_PRTDSI__SYNC_OUT       (* (reg8 *) Rx_Tag__PRTDSI__SYNC_OUT) 


#if defined(Rx_Tag__INTSTAT)  /* Interrupt Registers */

    #define Rx_Tag_INTSTAT                (* (reg8 *) Rx_Tag__INTSTAT)
    #define Rx_Tag_SNAP                   (* (reg8 *) Rx_Tag__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Rx_Tag_H */


/* [] END OF FILE */
