/*******************************************************************************
* File Name: Rs5.h  
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

#if !defined(CY_PINS_Rs5_H) /* Pins Rs5_H */
#define CY_PINS_Rs5_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Rs5_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Rs5__PORT == 15 && ((Rs5__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Rs5_Write(uint8 value) ;
void    Rs5_SetDriveMode(uint8 mode) ;
uint8   Rs5_ReadDataReg(void) ;
uint8   Rs5_Read(void) ;
uint8   Rs5_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Rs5_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Rs5_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Rs5_DM_RES_UP          PIN_DM_RES_UP
#define Rs5_DM_RES_DWN         PIN_DM_RES_DWN
#define Rs5_DM_OD_LO           PIN_DM_OD_LO
#define Rs5_DM_OD_HI           PIN_DM_OD_HI
#define Rs5_DM_STRONG          PIN_DM_STRONG
#define Rs5_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Rs5_MASK               Rs5__MASK
#define Rs5_SHIFT              Rs5__SHIFT
#define Rs5_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Rs5_PS                     (* (reg8 *) Rs5__PS)
/* Data Register */
#define Rs5_DR                     (* (reg8 *) Rs5__DR)
/* Port Number */
#define Rs5_PRT_NUM                (* (reg8 *) Rs5__PRT) 
/* Connect to Analog Globals */                                                  
#define Rs5_AG                     (* (reg8 *) Rs5__AG)                       
/* Analog MUX bux enable */
#define Rs5_AMUX                   (* (reg8 *) Rs5__AMUX) 
/* Bidirectional Enable */                                                        
#define Rs5_BIE                    (* (reg8 *) Rs5__BIE)
/* Bit-mask for Aliased Register Access */
#define Rs5_BIT_MASK               (* (reg8 *) Rs5__BIT_MASK)
/* Bypass Enable */
#define Rs5_BYP                    (* (reg8 *) Rs5__BYP)
/* Port wide control signals */                                                   
#define Rs5_CTL                    (* (reg8 *) Rs5__CTL)
/* Drive Modes */
#define Rs5_DM0                    (* (reg8 *) Rs5__DM0) 
#define Rs5_DM1                    (* (reg8 *) Rs5__DM1)
#define Rs5_DM2                    (* (reg8 *) Rs5__DM2) 
/* Input Buffer Disable Override */
#define Rs5_INP_DIS                (* (reg8 *) Rs5__INP_DIS)
/* LCD Common or Segment Drive */
#define Rs5_LCD_COM_SEG            (* (reg8 *) Rs5__LCD_COM_SEG)
/* Enable Segment LCD */
#define Rs5_LCD_EN                 (* (reg8 *) Rs5__LCD_EN)
/* Slew Rate Control */
#define Rs5_SLW                    (* (reg8 *) Rs5__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Rs5_PRTDSI__CAPS_SEL       (* (reg8 *) Rs5__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Rs5_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Rs5__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Rs5_PRTDSI__OE_SEL0        (* (reg8 *) Rs5__PRTDSI__OE_SEL0) 
#define Rs5_PRTDSI__OE_SEL1        (* (reg8 *) Rs5__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Rs5_PRTDSI__OUT_SEL0       (* (reg8 *) Rs5__PRTDSI__OUT_SEL0) 
#define Rs5_PRTDSI__OUT_SEL1       (* (reg8 *) Rs5__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Rs5_PRTDSI__SYNC_OUT       (* (reg8 *) Rs5__PRTDSI__SYNC_OUT) 


#if defined(Rs5__INTSTAT)  /* Interrupt Registers */

    #define Rs5_INTSTAT                (* (reg8 *) Rs5__INTSTAT)
    #define Rs5_SNAP                   (* (reg8 *) Rs5__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Rs5_H */


/* [] END OF FILE */
