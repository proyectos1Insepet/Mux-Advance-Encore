/*******************************************************************************
* File Name: Rs1.h  
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

#if !defined(CY_PINS_Rs1_H) /* Pins Rs1_H */
#define CY_PINS_Rs1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Rs1_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Rs1__PORT == 15 && ((Rs1__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Rs1_Write(uint8 value) ;
void    Rs1_SetDriveMode(uint8 mode) ;
uint8   Rs1_ReadDataReg(void) ;
uint8   Rs1_Read(void) ;
uint8   Rs1_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Rs1_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Rs1_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Rs1_DM_RES_UP          PIN_DM_RES_UP
#define Rs1_DM_RES_DWN         PIN_DM_RES_DWN
#define Rs1_DM_OD_LO           PIN_DM_OD_LO
#define Rs1_DM_OD_HI           PIN_DM_OD_HI
#define Rs1_DM_STRONG          PIN_DM_STRONG
#define Rs1_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Rs1_MASK               Rs1__MASK
#define Rs1_SHIFT              Rs1__SHIFT
#define Rs1_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Rs1_PS                     (* (reg8 *) Rs1__PS)
/* Data Register */
#define Rs1_DR                     (* (reg8 *) Rs1__DR)
/* Port Number */
#define Rs1_PRT_NUM                (* (reg8 *) Rs1__PRT) 
/* Connect to Analog Globals */                                                  
#define Rs1_AG                     (* (reg8 *) Rs1__AG)                       
/* Analog MUX bux enable */
#define Rs1_AMUX                   (* (reg8 *) Rs1__AMUX) 
/* Bidirectional Enable */                                                        
#define Rs1_BIE                    (* (reg8 *) Rs1__BIE)
/* Bit-mask for Aliased Register Access */
#define Rs1_BIT_MASK               (* (reg8 *) Rs1__BIT_MASK)
/* Bypass Enable */
#define Rs1_BYP                    (* (reg8 *) Rs1__BYP)
/* Port wide control signals */                                                   
#define Rs1_CTL                    (* (reg8 *) Rs1__CTL)
/* Drive Modes */
#define Rs1_DM0                    (* (reg8 *) Rs1__DM0) 
#define Rs1_DM1                    (* (reg8 *) Rs1__DM1)
#define Rs1_DM2                    (* (reg8 *) Rs1__DM2) 
/* Input Buffer Disable Override */
#define Rs1_INP_DIS                (* (reg8 *) Rs1__INP_DIS)
/* LCD Common or Segment Drive */
#define Rs1_LCD_COM_SEG            (* (reg8 *) Rs1__LCD_COM_SEG)
/* Enable Segment LCD */
#define Rs1_LCD_EN                 (* (reg8 *) Rs1__LCD_EN)
/* Slew Rate Control */
#define Rs1_SLW                    (* (reg8 *) Rs1__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Rs1_PRTDSI__CAPS_SEL       (* (reg8 *) Rs1__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Rs1_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Rs1__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Rs1_PRTDSI__OE_SEL0        (* (reg8 *) Rs1__PRTDSI__OE_SEL0) 
#define Rs1_PRTDSI__OE_SEL1        (* (reg8 *) Rs1__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Rs1_PRTDSI__OUT_SEL0       (* (reg8 *) Rs1__PRTDSI__OUT_SEL0) 
#define Rs1_PRTDSI__OUT_SEL1       (* (reg8 *) Rs1__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Rs1_PRTDSI__SYNC_OUT       (* (reg8 *) Rs1__PRTDSI__SYNC_OUT) 


#if defined(Rs1__INTSTAT)  /* Interrupt Registers */

    #define Rs1_INTSTAT                (* (reg8 *) Rs1__INTSTAT)
    #define Rs1_SNAP                   (* (reg8 *) Rs1__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Rs1_H */


/* [] END OF FILE */
