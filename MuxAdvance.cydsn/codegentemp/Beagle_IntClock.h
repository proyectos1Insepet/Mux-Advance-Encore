/*******************************************************************************
* File Name: Beagle_IntClock.h
* Version 2.20
*
*  Description:
*   Provides the function and constant definitions for the clock component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_CLOCK_Beagle_IntClock_H)
#define CY_CLOCK_Beagle_IntClock_H

#include <cytypes.h>
#include <cyfitter.h>


/***************************************
* Conditional Compilation Parameters
***************************************/

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component cy_clock_v2_20 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*        Function Prototypes
***************************************/

void Beagle_IntClock_Start(void) ;
void Beagle_IntClock_Stop(void) ;

#if(CY_PSOC3 || CY_PSOC5LP)
void Beagle_IntClock_StopBlock(void) ;
#endif /* (CY_PSOC3 || CY_PSOC5LP) */

void Beagle_IntClock_StandbyPower(uint8 state) ;
void Beagle_IntClock_SetDividerRegister(uint16 clkDivider, uint8 restart) 
                                ;
uint16 Beagle_IntClock_GetDividerRegister(void) ;
void Beagle_IntClock_SetModeRegister(uint8 modeBitMask) ;
void Beagle_IntClock_ClearModeRegister(uint8 modeBitMask) ;
uint8 Beagle_IntClock_GetModeRegister(void) ;
void Beagle_IntClock_SetSourceRegister(uint8 clkSource) ;
uint8 Beagle_IntClock_GetSourceRegister(void) ;
#if defined(Beagle_IntClock__CFG3)
void Beagle_IntClock_SetPhaseRegister(uint8 clkPhase) ;
uint8 Beagle_IntClock_GetPhaseRegister(void) ;
#endif /* defined(Beagle_IntClock__CFG3) */

#define Beagle_IntClock_Enable()                       Beagle_IntClock_Start()
#define Beagle_IntClock_Disable()                      Beagle_IntClock_Stop()
#define Beagle_IntClock_SetDivider(clkDivider)         Beagle_IntClock_SetDividerRegister(clkDivider, 1u)
#define Beagle_IntClock_SetDividerValue(clkDivider)    Beagle_IntClock_SetDividerRegister((clkDivider) - 1u, 1u)
#define Beagle_IntClock_SetMode(clkMode)               Beagle_IntClock_SetModeRegister(clkMode)
#define Beagle_IntClock_SetSource(clkSource)           Beagle_IntClock_SetSourceRegister(clkSource)
#if defined(Beagle_IntClock__CFG3)
#define Beagle_IntClock_SetPhase(clkPhase)             Beagle_IntClock_SetPhaseRegister(clkPhase)
#define Beagle_IntClock_SetPhaseValue(clkPhase)        Beagle_IntClock_SetPhaseRegister((clkPhase) + 1u)
#endif /* defined(Beagle_IntClock__CFG3) */


/***************************************
*             Registers
***************************************/

/* Register to enable or disable the clock */
#define Beagle_IntClock_CLKEN              (* (reg8 *) Beagle_IntClock__PM_ACT_CFG)
#define Beagle_IntClock_CLKEN_PTR          ((reg8 *) Beagle_IntClock__PM_ACT_CFG)

/* Register to enable or disable the clock */
#define Beagle_IntClock_CLKSTBY            (* (reg8 *) Beagle_IntClock__PM_STBY_CFG)
#define Beagle_IntClock_CLKSTBY_PTR        ((reg8 *) Beagle_IntClock__PM_STBY_CFG)

/* Clock LSB divider configuration register. */
#define Beagle_IntClock_DIV_LSB            (* (reg8 *) Beagle_IntClock__CFG0)
#define Beagle_IntClock_DIV_LSB_PTR        ((reg8 *) Beagle_IntClock__CFG0)
#define Beagle_IntClock_DIV_PTR            ((reg16 *) Beagle_IntClock__CFG0)

/* Clock MSB divider configuration register. */
#define Beagle_IntClock_DIV_MSB            (* (reg8 *) Beagle_IntClock__CFG1)
#define Beagle_IntClock_DIV_MSB_PTR        ((reg8 *) Beagle_IntClock__CFG1)

/* Mode and source configuration register */
#define Beagle_IntClock_MOD_SRC            (* (reg8 *) Beagle_IntClock__CFG2)
#define Beagle_IntClock_MOD_SRC_PTR        ((reg8 *) Beagle_IntClock__CFG2)

#if defined(Beagle_IntClock__CFG3)
/* Analog clock phase configuration register */
#define Beagle_IntClock_PHASE              (* (reg8 *) Beagle_IntClock__CFG3)
#define Beagle_IntClock_PHASE_PTR          ((reg8 *) Beagle_IntClock__CFG3)
#endif /* defined(Beagle_IntClock__CFG3) */


/**************************************
*       Register Constants
**************************************/

/* Power manager register masks */
#define Beagle_IntClock_CLKEN_MASK         Beagle_IntClock__PM_ACT_MSK
#define Beagle_IntClock_CLKSTBY_MASK       Beagle_IntClock__PM_STBY_MSK

/* CFG2 field masks */
#define Beagle_IntClock_SRC_SEL_MSK        Beagle_IntClock__CFG2_SRC_SEL_MASK
#define Beagle_IntClock_MODE_MASK          (~(Beagle_IntClock_SRC_SEL_MSK))

#if defined(Beagle_IntClock__CFG3)
/* CFG3 phase mask */
#define Beagle_IntClock_PHASE_MASK         Beagle_IntClock__CFG3_PHASE_DLY_MASK
#endif /* defined(Beagle_IntClock__CFG3) */

#endif /* CY_CLOCK_Beagle_IntClock_H */


/* [] END OF FILE */
