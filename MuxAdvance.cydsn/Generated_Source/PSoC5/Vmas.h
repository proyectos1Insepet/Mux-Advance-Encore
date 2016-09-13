/*******************************************************************************
* File Name: Vmas.h  
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

#if !defined(CY_PINS_Vmas_H) /* Pins Vmas_H */
#define CY_PINS_Vmas_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Vmas_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Vmas__PORT == 15 && ((Vmas__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Vmas_Write(uint8 value) ;
void    Vmas_SetDriveMode(uint8 mode) ;
uint8   Vmas_ReadDataReg(void) ;
uint8   Vmas_Read(void) ;
uint8   Vmas_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Vmas_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Vmas_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Vmas_DM_RES_UP          PIN_DM_RES_UP
#define Vmas_DM_RES_DWN         PIN_DM_RES_DWN
#define Vmas_DM_OD_LO           PIN_DM_OD_LO
#define Vmas_DM_OD_HI           PIN_DM_OD_HI
#define Vmas_DM_STRONG          PIN_DM_STRONG
#define Vmas_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Vmas_MASK               Vmas__MASK
#define Vmas_SHIFT              Vmas__SHIFT
#define Vmas_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Vmas_PS                     (* (reg8 *) Vmas__PS)
/* Data Register */
#define Vmas_DR                     (* (reg8 *) Vmas__DR)
/* Port Number */
#define Vmas_PRT_NUM                (* (reg8 *) Vmas__PRT) 
/* Connect to Analog Globals */                                                  
#define Vmas_AG                     (* (reg8 *) Vmas__AG)                       
/* Analog MUX bux enable */
#define Vmas_AMUX                   (* (reg8 *) Vmas__AMUX) 
/* Bidirectional Enable */                                                        
#define Vmas_BIE                    (* (reg8 *) Vmas__BIE)
/* Bit-mask for Aliased Register Access */
#define Vmas_BIT_MASK               (* (reg8 *) Vmas__BIT_MASK)
/* Bypass Enable */
#define Vmas_BYP                    (* (reg8 *) Vmas__BYP)
/* Port wide control signals */                                                   
#define Vmas_CTL                    (* (reg8 *) Vmas__CTL)
/* Drive Modes */
#define Vmas_DM0                    (* (reg8 *) Vmas__DM0) 
#define Vmas_DM1                    (* (reg8 *) Vmas__DM1)
#define Vmas_DM2                    (* (reg8 *) Vmas__DM2) 
/* Input Buffer Disable Override */
#define Vmas_INP_DIS                (* (reg8 *) Vmas__INP_DIS)
/* LCD Common or Segment Drive */
#define Vmas_LCD_COM_SEG            (* (reg8 *) Vmas__LCD_COM_SEG)
/* Enable Segment LCD */
#define Vmas_LCD_EN                 (* (reg8 *) Vmas__LCD_EN)
/* Slew Rate Control */
#define Vmas_SLW                    (* (reg8 *) Vmas__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Vmas_PRTDSI__CAPS_SEL       (* (reg8 *) Vmas__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Vmas_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Vmas__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Vmas_PRTDSI__OE_SEL0        (* (reg8 *) Vmas__PRTDSI__OE_SEL0) 
#define Vmas_PRTDSI__OE_SEL1        (* (reg8 *) Vmas__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Vmas_PRTDSI__OUT_SEL0       (* (reg8 *) Vmas__PRTDSI__OUT_SEL0) 
#define Vmas_PRTDSI__OUT_SEL1       (* (reg8 *) Vmas__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Vmas_PRTDSI__SYNC_OUT       (* (reg8 *) Vmas__PRTDSI__SYNC_OUT) 


#if defined(Vmas__INTSTAT)  /* Interrupt Registers */

    #define Vmas_INTSTAT                (* (reg8 *) Vmas__INTSTAT)
    #define Vmas_SNAP                   (* (reg8 *) Vmas__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Vmas_H */


/* [] END OF FILE */
