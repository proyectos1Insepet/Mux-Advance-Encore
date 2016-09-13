/*******************************************************************************
* File Name: Rs3.h  
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

#if !defined(CY_PINS_Rs3_H) /* Pins Rs3_H */
#define CY_PINS_Rs3_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Rs3_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Rs3__PORT == 15 && ((Rs3__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Rs3_Write(uint8 value) ;
void    Rs3_SetDriveMode(uint8 mode) ;
uint8   Rs3_ReadDataReg(void) ;
uint8   Rs3_Read(void) ;
uint8   Rs3_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Rs3_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Rs3_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Rs3_DM_RES_UP          PIN_DM_RES_UP
#define Rs3_DM_RES_DWN         PIN_DM_RES_DWN
#define Rs3_DM_OD_LO           PIN_DM_OD_LO
#define Rs3_DM_OD_HI           PIN_DM_OD_HI
#define Rs3_DM_STRONG          PIN_DM_STRONG
#define Rs3_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Rs3_MASK               Rs3__MASK
#define Rs3_SHIFT              Rs3__SHIFT
#define Rs3_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Rs3_PS                     (* (reg8 *) Rs3__PS)
/* Data Register */
#define Rs3_DR                     (* (reg8 *) Rs3__DR)
/* Port Number */
#define Rs3_PRT_NUM                (* (reg8 *) Rs3__PRT) 
/* Connect to Analog Globals */                                                  
#define Rs3_AG                     (* (reg8 *) Rs3__AG)                       
/* Analog MUX bux enable */
#define Rs3_AMUX                   (* (reg8 *) Rs3__AMUX) 
/* Bidirectional Enable */                                                        
#define Rs3_BIE                    (* (reg8 *) Rs3__BIE)
/* Bit-mask for Aliased Register Access */
#define Rs3_BIT_MASK               (* (reg8 *) Rs3__BIT_MASK)
/* Bypass Enable */
#define Rs3_BYP                    (* (reg8 *) Rs3__BYP)
/* Port wide control signals */                                                   
#define Rs3_CTL                    (* (reg8 *) Rs3__CTL)
/* Drive Modes */
#define Rs3_DM0                    (* (reg8 *) Rs3__DM0) 
#define Rs3_DM1                    (* (reg8 *) Rs3__DM1)
#define Rs3_DM2                    (* (reg8 *) Rs3__DM2) 
/* Input Buffer Disable Override */
#define Rs3_INP_DIS                (* (reg8 *) Rs3__INP_DIS)
/* LCD Common or Segment Drive */
#define Rs3_LCD_COM_SEG            (* (reg8 *) Rs3__LCD_COM_SEG)
/* Enable Segment LCD */
#define Rs3_LCD_EN                 (* (reg8 *) Rs3__LCD_EN)
/* Slew Rate Control */
#define Rs3_SLW                    (* (reg8 *) Rs3__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Rs3_PRTDSI__CAPS_SEL       (* (reg8 *) Rs3__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Rs3_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Rs3__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Rs3_PRTDSI__OE_SEL0        (* (reg8 *) Rs3__PRTDSI__OE_SEL0) 
#define Rs3_PRTDSI__OE_SEL1        (* (reg8 *) Rs3__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Rs3_PRTDSI__OUT_SEL0       (* (reg8 *) Rs3__PRTDSI__OUT_SEL0) 
#define Rs3_PRTDSI__OUT_SEL1       (* (reg8 *) Rs3__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Rs3_PRTDSI__SYNC_OUT       (* (reg8 *) Rs3__PRTDSI__SYNC_OUT) 


#if defined(Rs3__INTSTAT)  /* Interrupt Registers */

    #define Rs3_INTSTAT                (* (reg8 *) Rs3__INTSTAT)
    #define Rs3_SNAP                   (* (reg8 *) Rs3__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Rs3_H */


/* [] END OF FILE */
