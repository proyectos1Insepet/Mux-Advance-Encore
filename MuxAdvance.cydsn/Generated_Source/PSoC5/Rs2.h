/*******************************************************************************
* File Name: Rs2.h  
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

#if !defined(CY_PINS_Rs2_H) /* Pins Rs2_H */
#define CY_PINS_Rs2_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Rs2_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Rs2__PORT == 15 && ((Rs2__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Rs2_Write(uint8 value) ;
void    Rs2_SetDriveMode(uint8 mode) ;
uint8   Rs2_ReadDataReg(void) ;
uint8   Rs2_Read(void) ;
uint8   Rs2_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Rs2_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Rs2_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Rs2_DM_RES_UP          PIN_DM_RES_UP
#define Rs2_DM_RES_DWN         PIN_DM_RES_DWN
#define Rs2_DM_OD_LO           PIN_DM_OD_LO
#define Rs2_DM_OD_HI           PIN_DM_OD_HI
#define Rs2_DM_STRONG          PIN_DM_STRONG
#define Rs2_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Rs2_MASK               Rs2__MASK
#define Rs2_SHIFT              Rs2__SHIFT
#define Rs2_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Rs2_PS                     (* (reg8 *) Rs2__PS)
/* Data Register */
#define Rs2_DR                     (* (reg8 *) Rs2__DR)
/* Port Number */
#define Rs2_PRT_NUM                (* (reg8 *) Rs2__PRT) 
/* Connect to Analog Globals */                                                  
#define Rs2_AG                     (* (reg8 *) Rs2__AG)                       
/* Analog MUX bux enable */
#define Rs2_AMUX                   (* (reg8 *) Rs2__AMUX) 
/* Bidirectional Enable */                                                        
#define Rs2_BIE                    (* (reg8 *) Rs2__BIE)
/* Bit-mask for Aliased Register Access */
#define Rs2_BIT_MASK               (* (reg8 *) Rs2__BIT_MASK)
/* Bypass Enable */
#define Rs2_BYP                    (* (reg8 *) Rs2__BYP)
/* Port wide control signals */                                                   
#define Rs2_CTL                    (* (reg8 *) Rs2__CTL)
/* Drive Modes */
#define Rs2_DM0                    (* (reg8 *) Rs2__DM0) 
#define Rs2_DM1                    (* (reg8 *) Rs2__DM1)
#define Rs2_DM2                    (* (reg8 *) Rs2__DM2) 
/* Input Buffer Disable Override */
#define Rs2_INP_DIS                (* (reg8 *) Rs2__INP_DIS)
/* LCD Common or Segment Drive */
#define Rs2_LCD_COM_SEG            (* (reg8 *) Rs2__LCD_COM_SEG)
/* Enable Segment LCD */
#define Rs2_LCD_EN                 (* (reg8 *) Rs2__LCD_EN)
/* Slew Rate Control */
#define Rs2_SLW                    (* (reg8 *) Rs2__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Rs2_PRTDSI__CAPS_SEL       (* (reg8 *) Rs2__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Rs2_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Rs2__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Rs2_PRTDSI__OE_SEL0        (* (reg8 *) Rs2__PRTDSI__OE_SEL0) 
#define Rs2_PRTDSI__OE_SEL1        (* (reg8 *) Rs2__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Rs2_PRTDSI__OUT_SEL0       (* (reg8 *) Rs2__PRTDSI__OUT_SEL0) 
#define Rs2_PRTDSI__OUT_SEL1       (* (reg8 *) Rs2__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Rs2_PRTDSI__SYNC_OUT       (* (reg8 *) Rs2__PRTDSI__SYNC_OUT) 


#if defined(Rs2__INTSTAT)  /* Interrupt Registers */

    #define Rs2_INTSTAT                (* (reg8 *) Rs2__INTSTAT)
    #define Rs2_SNAP                   (* (reg8 *) Rs2__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Rs2_H */


/* [] END OF FILE */
