/*******************************************************************************
* File Name: Vmenos.h  
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

#if !defined(CY_PINS_Vmenos_H) /* Pins Vmenos_H */
#define CY_PINS_Vmenos_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "Vmenos_aliases.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component cy_pins_v2_10 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 Vmenos__PORT == 15 && ((Vmenos__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

void    Vmenos_Write(uint8 value) ;
void    Vmenos_SetDriveMode(uint8 mode) ;
uint8   Vmenos_ReadDataReg(void) ;
uint8   Vmenos_Read(void) ;
uint8   Vmenos_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define Vmenos_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define Vmenos_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define Vmenos_DM_RES_UP          PIN_DM_RES_UP
#define Vmenos_DM_RES_DWN         PIN_DM_RES_DWN
#define Vmenos_DM_OD_LO           PIN_DM_OD_LO
#define Vmenos_DM_OD_HI           PIN_DM_OD_HI
#define Vmenos_DM_STRONG          PIN_DM_STRONG
#define Vmenos_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define Vmenos_MASK               Vmenos__MASK
#define Vmenos_SHIFT              Vmenos__SHIFT
#define Vmenos_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define Vmenos_PS                     (* (reg8 *) Vmenos__PS)
/* Data Register */
#define Vmenos_DR                     (* (reg8 *) Vmenos__DR)
/* Port Number */
#define Vmenos_PRT_NUM                (* (reg8 *) Vmenos__PRT) 
/* Connect to Analog Globals */                                                  
#define Vmenos_AG                     (* (reg8 *) Vmenos__AG)                       
/* Analog MUX bux enable */
#define Vmenos_AMUX                   (* (reg8 *) Vmenos__AMUX) 
/* Bidirectional Enable */                                                        
#define Vmenos_BIE                    (* (reg8 *) Vmenos__BIE)
/* Bit-mask for Aliased Register Access */
#define Vmenos_BIT_MASK               (* (reg8 *) Vmenos__BIT_MASK)
/* Bypass Enable */
#define Vmenos_BYP                    (* (reg8 *) Vmenos__BYP)
/* Port wide control signals */                                                   
#define Vmenos_CTL                    (* (reg8 *) Vmenos__CTL)
/* Drive Modes */
#define Vmenos_DM0                    (* (reg8 *) Vmenos__DM0) 
#define Vmenos_DM1                    (* (reg8 *) Vmenos__DM1)
#define Vmenos_DM2                    (* (reg8 *) Vmenos__DM2) 
/* Input Buffer Disable Override */
#define Vmenos_INP_DIS                (* (reg8 *) Vmenos__INP_DIS)
/* LCD Common or Segment Drive */
#define Vmenos_LCD_COM_SEG            (* (reg8 *) Vmenos__LCD_COM_SEG)
/* Enable Segment LCD */
#define Vmenos_LCD_EN                 (* (reg8 *) Vmenos__LCD_EN)
/* Slew Rate Control */
#define Vmenos_SLW                    (* (reg8 *) Vmenos__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define Vmenos_PRTDSI__CAPS_SEL       (* (reg8 *) Vmenos__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define Vmenos_PRTDSI__DBL_SYNC_IN    (* (reg8 *) Vmenos__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define Vmenos_PRTDSI__OE_SEL0        (* (reg8 *) Vmenos__PRTDSI__OE_SEL0) 
#define Vmenos_PRTDSI__OE_SEL1        (* (reg8 *) Vmenos__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define Vmenos_PRTDSI__OUT_SEL0       (* (reg8 *) Vmenos__PRTDSI__OUT_SEL0) 
#define Vmenos_PRTDSI__OUT_SEL1       (* (reg8 *) Vmenos__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define Vmenos_PRTDSI__SYNC_OUT       (* (reg8 *) Vmenos__PRTDSI__SYNC_OUT) 


#if defined(Vmenos__INTSTAT)  /* Interrupt Registers */

    #define Vmenos_INTSTAT                (* (reg8 *) Vmenos__INTSTAT)
    #define Vmenos_SNAP                   (* (reg8 *) Vmenos__SNAP)

#endif /* Interrupt Registers */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_Vmenos_H */


/* [] END OF FILE */
