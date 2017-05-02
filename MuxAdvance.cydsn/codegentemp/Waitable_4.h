/*******************************************************************************
* File Name: Waitable_4.h
* Version 2.70
*
*  Description:
*     Contains the function prototypes and constants available to the timer
*     user module.
*
*   Note:
*     None
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#if !defined(CY_Timer_v2_60_Waitable_4_H)
#define CY_Timer_v2_60_Waitable_4_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h" /* For CyEnterCriticalSection() and CyExitCriticalSection() functions */

extern uint8 Waitable_4_initVar;

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component Timer_v2_70 requires cy_boot v3.0 or later
#endif /* (CY_ PSOC5LP) */


/**************************************
*           Parameter Defaults
**************************************/

#define Waitable_4_Resolution                 16u
#define Waitable_4_UsingFixedFunction         1u
#define Waitable_4_UsingHWCaptureCounter      0u
#define Waitable_4_SoftwareCaptureMode        0u
#define Waitable_4_SoftwareTriggerMode        0u
#define Waitable_4_UsingHWEnable              0u
#define Waitable_4_EnableTriggerMode          0u
#define Waitable_4_InterruptOnCaptureCount    0u
#define Waitable_4_RunModeUsed                0u
#define Waitable_4_ControlRegRemoved          0u

#if defined(Waitable_4_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG)
    #define Waitable_4_UDB_CONTROL_REG_REMOVED            (0u)
#elif  (Waitable_4_UsingFixedFunction)
    #define Waitable_4_UDB_CONTROL_REG_REMOVED            (0u)
#else 
    #define Waitable_4_UDB_CONTROL_REG_REMOVED            (1u)
#endif /* End Waitable_4_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG */


/***************************************
*       Type defines
***************************************/


/**************************************************************************
 * Sleep Wakeup Backup structure for Timer Component
 *************************************************************************/
typedef struct
{
    uint8 TimerEnableState;
    #if(!Waitable_4_UsingFixedFunction)

        uint16 TimerUdb;
        uint8 InterruptMaskValue;
        #if (Waitable_4_UsingHWCaptureCounter)
            uint8 TimerCaptureCounter;
        #endif /* variable declarations for backing up non retention registers in CY_UDB_V1 */

        #if (!Waitable_4_UDB_CONTROL_REG_REMOVED)
            uint8 TimerControlRegister;
        #endif /* variable declaration for backing up enable state of the Timer */
    #endif /* define backup variables only for UDB implementation. Fixed function registers are all retention */

}Waitable_4_backupStruct;


/***************************************
*       Function Prototypes
***************************************/

void    Waitable_4_Start(void) ;
void    Waitable_4_Stop(void) ;

void    Waitable_4_SetInterruptMode(uint8 interruptMode) ;
uint8   Waitable_4_ReadStatusRegister(void) ;
/* Deprecated function. Do not use this in future. Retained for backward compatibility */
#define Waitable_4_GetInterruptSource() Waitable_4_ReadStatusRegister()

#if(!Waitable_4_UDB_CONTROL_REG_REMOVED)
    uint8   Waitable_4_ReadControlRegister(void) ;
    void    Waitable_4_WriteControlRegister(uint8 control) ;
#endif /* (!Waitable_4_UDB_CONTROL_REG_REMOVED) */

uint16  Waitable_4_ReadPeriod(void) ;
void    Waitable_4_WritePeriod(uint16 period) ;
uint16  Waitable_4_ReadCounter(void) ;
void    Waitable_4_WriteCounter(uint16 counter) ;
uint16  Waitable_4_ReadCapture(void) ;
void    Waitable_4_SoftwareCapture(void) ;

#if(!Waitable_4_UsingFixedFunction) /* UDB Prototypes */
    #if (Waitable_4_SoftwareCaptureMode)
        void    Waitable_4_SetCaptureMode(uint8 captureMode) ;
    #endif /* (!Waitable_4_UsingFixedFunction) */

    #if (Waitable_4_SoftwareTriggerMode)
        void    Waitable_4_SetTriggerMode(uint8 triggerMode) ;
    #endif /* (Waitable_4_SoftwareTriggerMode) */

    #if (Waitable_4_EnableTriggerMode)
        void    Waitable_4_EnableTrigger(void) ;
        void    Waitable_4_DisableTrigger(void) ;
    #endif /* (Waitable_4_EnableTriggerMode) */


    #if(Waitable_4_InterruptOnCaptureCount)
        void    Waitable_4_SetInterruptCount(uint8 interruptCount) ;
    #endif /* (Waitable_4_InterruptOnCaptureCount) */

    #if (Waitable_4_UsingHWCaptureCounter)
        void    Waitable_4_SetCaptureCount(uint8 captureCount) ;
        uint8   Waitable_4_ReadCaptureCount(void) ;
    #endif /* (Waitable_4_UsingHWCaptureCounter) */

    void Waitable_4_ClearFIFO(void) ;
#endif /* UDB Prototypes */

/* Sleep Retention APIs */
void Waitable_4_Init(void)          ;
void Waitable_4_Enable(void)        ;
void Waitable_4_SaveConfig(void)    ;
void Waitable_4_RestoreConfig(void) ;
void Waitable_4_Sleep(void)         ;
void Waitable_4_Wakeup(void)        ;


/***************************************
*   Enumerated Types and Parameters
***************************************/

/* Enumerated Type B_Timer__CaptureModes, Used in Capture Mode */
#define Waitable_4__B_TIMER__CM_NONE 0
#define Waitable_4__B_TIMER__CM_RISINGEDGE 1
#define Waitable_4__B_TIMER__CM_FALLINGEDGE 2
#define Waitable_4__B_TIMER__CM_EITHEREDGE 3
#define Waitable_4__B_TIMER__CM_SOFTWARE 4



/* Enumerated Type B_Timer__TriggerModes, Used in Trigger Mode */
#define Waitable_4__B_TIMER__TM_NONE 0x00u
#define Waitable_4__B_TIMER__TM_RISINGEDGE 0x04u
#define Waitable_4__B_TIMER__TM_FALLINGEDGE 0x08u
#define Waitable_4__B_TIMER__TM_EITHEREDGE 0x0Cu
#define Waitable_4__B_TIMER__TM_SOFTWARE 0x10u


/***************************************
*    Initialial Parameter Constants
***************************************/

#define Waitable_4_INIT_PERIOD             249u
#define Waitable_4_INIT_CAPTURE_MODE       ((uint8)((uint8)0u << Waitable_4_CTRL_CAP_MODE_SHIFT))
#define Waitable_4_INIT_TRIGGER_MODE       ((uint8)((uint8)0u << Waitable_4_CTRL_TRIG_MODE_SHIFT))
#if (Waitable_4_UsingFixedFunction)
    #define Waitable_4_INIT_INTERRUPT_MODE (((uint8)((uint8)1u << Waitable_4_STATUS_TC_INT_MASK_SHIFT)) | \
                                                  ((uint8)((uint8)0 << Waitable_4_STATUS_CAPTURE_INT_MASK_SHIFT)))
#else
    #define Waitable_4_INIT_INTERRUPT_MODE (((uint8)((uint8)1u << Waitable_4_STATUS_TC_INT_MASK_SHIFT)) | \
                                                 ((uint8)((uint8)0 << Waitable_4_STATUS_CAPTURE_INT_MASK_SHIFT)) | \
                                                 ((uint8)((uint8)0 << Waitable_4_STATUS_FIFOFULL_INT_MASK_SHIFT)))
#endif /* (Waitable_4_UsingFixedFunction) */
#define Waitable_4_INIT_CAPTURE_COUNT      (2u)
#define Waitable_4_INIT_INT_CAPTURE_COUNT  ((uint8)((uint8)(1u - 1u) << Waitable_4_CTRL_INTCNT_SHIFT))


/***************************************
*           Registers
***************************************/

#if (Waitable_4_UsingFixedFunction) /* Implementation Specific Registers and Register Constants */


    /***************************************
    *    Fixed Function Registers
    ***************************************/

    #define Waitable_4_STATUS         (*(reg8 *) Waitable_4_TimerHW__SR0 )
    /* In Fixed Function Block Status and Mask are the same register */
    #define Waitable_4_STATUS_MASK    (*(reg8 *) Waitable_4_TimerHW__SR0 )
    #define Waitable_4_CONTROL        (*(reg8 *) Waitable_4_TimerHW__CFG0)
    #define Waitable_4_CONTROL2       (*(reg8 *) Waitable_4_TimerHW__CFG1)
    #define Waitable_4_CONTROL2_PTR   ( (reg8 *) Waitable_4_TimerHW__CFG1)
    #define Waitable_4_RT1            (*(reg8 *) Waitable_4_TimerHW__RT1)
    #define Waitable_4_RT1_PTR        ( (reg8 *) Waitable_4_TimerHW__RT1)

    #if (CY_PSOC3 || CY_PSOC5LP)
        #define Waitable_4_CONTROL3       (*(reg8 *) Waitable_4_TimerHW__CFG2)
        #define Waitable_4_CONTROL3_PTR   ( (reg8 *) Waitable_4_TimerHW__CFG2)
    #endif /* (CY_PSOC3 || CY_PSOC5LP) */
    #define Waitable_4_GLOBAL_ENABLE  (*(reg8 *) Waitable_4_TimerHW__PM_ACT_CFG)
    #define Waitable_4_GLOBAL_STBY_ENABLE  (*(reg8 *) Waitable_4_TimerHW__PM_STBY_CFG)

    #define Waitable_4_CAPTURE_LSB         (* (reg16 *) Waitable_4_TimerHW__CAP0 )
    #define Waitable_4_CAPTURE_LSB_PTR       ((reg16 *) Waitable_4_TimerHW__CAP0 )
    #define Waitable_4_PERIOD_LSB          (* (reg16 *) Waitable_4_TimerHW__PER0 )
    #define Waitable_4_PERIOD_LSB_PTR        ((reg16 *) Waitable_4_TimerHW__PER0 )
    #define Waitable_4_COUNTER_LSB         (* (reg16 *) Waitable_4_TimerHW__CNT_CMP0 )
    #define Waitable_4_COUNTER_LSB_PTR       ((reg16 *) Waitable_4_TimerHW__CNT_CMP0 )


    /***************************************
    *    Register Constants
    ***************************************/

    /* Fixed Function Block Chosen */
    #define Waitable_4_BLOCK_EN_MASK                     Waitable_4_TimerHW__PM_ACT_MSK
    #define Waitable_4_BLOCK_STBY_EN_MASK                Waitable_4_TimerHW__PM_STBY_MSK

    /* Control Register Bit Locations */
    /* Interrupt Count - Not valid for Fixed Function Block */
    #define Waitable_4_CTRL_INTCNT_SHIFT                  0x00u
    /* Trigger Polarity - Not valid for Fixed Function Block */
    #define Waitable_4_CTRL_TRIG_MODE_SHIFT               0x00u
    /* Trigger Enable - Not valid for Fixed Function Block */
    #define Waitable_4_CTRL_TRIG_EN_SHIFT                 0x00u
    /* Capture Polarity - Not valid for Fixed Function Block */
    #define Waitable_4_CTRL_CAP_MODE_SHIFT                0x00u
    /* Timer Enable - As defined in Register Map, part of TMRX_CFG0 register */
    #define Waitable_4_CTRL_ENABLE_SHIFT                  0x00u

    /* Control Register Bit Masks */
    #define Waitable_4_CTRL_ENABLE                        ((uint8)((uint8)0x01u << Waitable_4_CTRL_ENABLE_SHIFT))

    /* Control2 Register Bit Masks */
    /* As defined in Register Map, Part of the TMRX_CFG1 register */
    #define Waitable_4_CTRL2_IRQ_SEL_SHIFT                 0x00u
    #define Waitable_4_CTRL2_IRQ_SEL                      ((uint8)((uint8)0x01u << Waitable_4_CTRL2_IRQ_SEL_SHIFT))

    #if (CY_PSOC5A)
        /* Use CFG1 Mode bits to set run mode */
        /* As defined by Verilog Implementation */
        #define Waitable_4_CTRL_MODE_SHIFT                 0x01u
        #define Waitable_4_CTRL_MODE_MASK                 ((uint8)((uint8)0x07u << Waitable_4_CTRL_MODE_SHIFT))
    #endif /* (CY_PSOC5A) */
    #if (CY_PSOC3 || CY_PSOC5LP)
        /* Control3 Register Bit Locations */
        #define Waitable_4_CTRL_RCOD_SHIFT        0x02u
        #define Waitable_4_CTRL_ENBL_SHIFT        0x00u
        #define Waitable_4_CTRL_MODE_SHIFT        0x00u

        /* Control3 Register Bit Masks */
        #define Waitable_4_CTRL_RCOD_MASK  ((uint8)((uint8)0x03u << Waitable_4_CTRL_RCOD_SHIFT)) /* ROD and COD bit masks */
        #define Waitable_4_CTRL_ENBL_MASK  ((uint8)((uint8)0x80u << Waitable_4_CTRL_ENBL_SHIFT)) /* HW_EN bit mask */
        #define Waitable_4_CTRL_MODE_MASK  ((uint8)((uint8)0x03u << Waitable_4_CTRL_MODE_SHIFT)) /* Run mode bit mask */

        #define Waitable_4_CTRL_RCOD       ((uint8)((uint8)0x03u << Waitable_4_CTRL_RCOD_SHIFT))
        #define Waitable_4_CTRL_ENBL       ((uint8)((uint8)0x80u << Waitable_4_CTRL_ENBL_SHIFT))
    #endif /* (CY_PSOC3 || CY_PSOC5LP) */

    /*RT1 Synch Constants: Applicable for PSoC3 and PSoC5LP */
    #define Waitable_4_RT1_SHIFT                       0x04u
    /* Sync TC and CMP bit masks */
    #define Waitable_4_RT1_MASK                        ((uint8)((uint8)0x03u << Waitable_4_RT1_SHIFT))
    #define Waitable_4_SYNC                            ((uint8)((uint8)0x03u << Waitable_4_RT1_SHIFT))
    #define Waitable_4_SYNCDSI_SHIFT                   0x00u
    /* Sync all DSI inputs with Mask  */
    #define Waitable_4_SYNCDSI_MASK                    ((uint8)((uint8)0x0Fu << Waitable_4_SYNCDSI_SHIFT))
    /* Sync all DSI inputs */
    #define Waitable_4_SYNCDSI_EN                      ((uint8)((uint8)0x0Fu << Waitable_4_SYNCDSI_SHIFT))

    #define Waitable_4_CTRL_MODE_PULSEWIDTH            ((uint8)((uint8)0x01u << Waitable_4_CTRL_MODE_SHIFT))
    #define Waitable_4_CTRL_MODE_PERIOD                ((uint8)((uint8)0x02u << Waitable_4_CTRL_MODE_SHIFT))
    #define Waitable_4_CTRL_MODE_CONTINUOUS            ((uint8)((uint8)0x00u << Waitable_4_CTRL_MODE_SHIFT))

    /* Status Register Bit Locations */
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define Waitable_4_STATUS_TC_SHIFT                 0x07u
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define Waitable_4_STATUS_CAPTURE_SHIFT            0x06u
    /* As defined in Register Map, part of TMRX_SR0 register */
    #define Waitable_4_STATUS_TC_INT_MASK_SHIFT        (Waitable_4_STATUS_TC_SHIFT - 0x04u)
    /* As defined in Register Map, part of TMRX_SR0 register, Shared with Compare Status */
    #define Waitable_4_STATUS_CAPTURE_INT_MASK_SHIFT   (Waitable_4_STATUS_CAPTURE_SHIFT - 0x04u)

    /* Status Register Bit Masks */
    #define Waitable_4_STATUS_TC                       ((uint8)((uint8)0x01u << Waitable_4_STATUS_TC_SHIFT))
    #define Waitable_4_STATUS_CAPTURE                  ((uint8)((uint8)0x01u << Waitable_4_STATUS_CAPTURE_SHIFT))
    /* Interrupt Enable Bit-Mask for interrupt on TC */
    #define Waitable_4_STATUS_TC_INT_MASK              ((uint8)((uint8)0x01u << Waitable_4_STATUS_TC_INT_MASK_SHIFT))
    /* Interrupt Enable Bit-Mask for interrupt on Capture */
    #define Waitable_4_STATUS_CAPTURE_INT_MASK         ((uint8)((uint8)0x01u << Waitable_4_STATUS_CAPTURE_INT_MASK_SHIFT))

#else   /* UDB Registers and Register Constants */


    /***************************************
    *           UDB Registers
    ***************************************/

    #define Waitable_4_STATUS              (* (reg8 *) Waitable_4_TimerUDB_rstSts_stsreg__STATUS_REG )
    #define Waitable_4_STATUS_MASK         (* (reg8 *) Waitable_4_TimerUDB_rstSts_stsreg__MASK_REG)
    #define Waitable_4_STATUS_AUX_CTRL     (* (reg8 *) Waitable_4_TimerUDB_rstSts_stsreg__STATUS_AUX_CTL_REG)
    #define Waitable_4_CONTROL             (* (reg8 *) Waitable_4_TimerUDB_sCTRLReg_SyncCtl_ctrlreg__CONTROL_REG )
    
    #if(Waitable_4_Resolution <= 8u) /* 8-bit Timer */
        #define Waitable_4_CAPTURE_LSB         (* (reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Waitable_4_CAPTURE_LSB_PTR       ((reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Waitable_4_PERIOD_LSB          (* (reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Waitable_4_PERIOD_LSB_PTR        ((reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Waitable_4_COUNTER_LSB         (* (reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
        #define Waitable_4_COUNTER_LSB_PTR       ((reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
    #elif(Waitable_4_Resolution <= 16u) /* 8-bit Timer */
        #if(CY_PSOC3) /* 8-bit addres space */
            #define Waitable_4_CAPTURE_LSB         (* (reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Waitable_4_CAPTURE_LSB_PTR       ((reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Waitable_4_PERIOD_LSB          (* (reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Waitable_4_PERIOD_LSB_PTR        ((reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Waitable_4_COUNTER_LSB         (* (reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
            #define Waitable_4_COUNTER_LSB_PTR       ((reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
        #else /* 16-bit address space */
            #define Waitable_4_CAPTURE_LSB         (* (reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__16BIT_F0_REG )
            #define Waitable_4_CAPTURE_LSB_PTR       ((reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__16BIT_F0_REG )
            #define Waitable_4_PERIOD_LSB          (* (reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__16BIT_D0_REG )
            #define Waitable_4_PERIOD_LSB_PTR        ((reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__16BIT_D0_REG )
            #define Waitable_4_COUNTER_LSB         (* (reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__16BIT_A0_REG )
            #define Waitable_4_COUNTER_LSB_PTR       ((reg16 *) Waitable_4_TimerUDB_sT16_timerdp_u0__16BIT_A0_REG )
        #endif /* CY_PSOC3 */
    #elif(Waitable_4_Resolution <= 24u)/* 24-bit Timer */
        #define Waitable_4_CAPTURE_LSB         (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Waitable_4_CAPTURE_LSB_PTR       ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
        #define Waitable_4_PERIOD_LSB          (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Waitable_4_PERIOD_LSB_PTR        ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
        #define Waitable_4_COUNTER_LSB         (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
        #define Waitable_4_COUNTER_LSB_PTR       ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
    #else /* 32-bit Timer */
        #if(CY_PSOC3 || CY_PSOC5) /* 8-bit address space */
            #define Waitable_4_CAPTURE_LSB         (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Waitable_4_CAPTURE_LSB_PTR       ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__F0_REG )
            #define Waitable_4_PERIOD_LSB          (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Waitable_4_PERIOD_LSB_PTR        ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__D0_REG )
            #define Waitable_4_COUNTER_LSB         (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
            #define Waitable_4_COUNTER_LSB_PTR       ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
        #else /* 32-bit address space */
            #define Waitable_4_CAPTURE_LSB         (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__32BIT_F0_REG )
            #define Waitable_4_CAPTURE_LSB_PTR       ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__32BIT_F0_REG )
            #define Waitable_4_PERIOD_LSB          (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__32BIT_D0_REG )
            #define Waitable_4_PERIOD_LSB_PTR        ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__32BIT_D0_REG )
            #define Waitable_4_COUNTER_LSB         (* (reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__32BIT_A0_REG )
            #define Waitable_4_COUNTER_LSB_PTR       ((reg32 *) Waitable_4_TimerUDB_sT16_timerdp_u0__32BIT_A0_REG )
        #endif /* CY_PSOC3 || CY_PSOC5 */ 
    #endif

    #define Waitable_4_COUNTER_LSB_PTR_8BIT       ((reg8 *) Waitable_4_TimerUDB_sT16_timerdp_u0__A0_REG )
    
    #if (Waitable_4_UsingHWCaptureCounter)
        #define Waitable_4_CAP_COUNT              (*(reg8 *) Waitable_4_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define Waitable_4_CAP_COUNT_PTR          ( (reg8 *) Waitable_4_TimerUDB_sCapCount_counter__PERIOD_REG )
        #define Waitable_4_CAPTURE_COUNT_CTRL     (*(reg8 *) Waitable_4_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
        #define Waitable_4_CAPTURE_COUNT_CTRL_PTR ( (reg8 *) Waitable_4_TimerUDB_sCapCount_counter__CONTROL_AUX_CTL_REG )
    #endif /* (Waitable_4_UsingHWCaptureCounter) */


    /***************************************
    *       Register Constants
    ***************************************/

    /* Control Register Bit Locations */
    #define Waitable_4_CTRL_INTCNT_SHIFT              0x00u       /* As defined by Verilog Implementation */
    #define Waitable_4_CTRL_TRIG_MODE_SHIFT           0x02u       /* As defined by Verilog Implementation */
    #define Waitable_4_CTRL_TRIG_EN_SHIFT             0x04u       /* As defined by Verilog Implementation */
    #define Waitable_4_CTRL_CAP_MODE_SHIFT            0x05u       /* As defined by Verilog Implementation */
    #define Waitable_4_CTRL_ENABLE_SHIFT              0x07u       /* As defined by Verilog Implementation */

    /* Control Register Bit Masks */
    #define Waitable_4_CTRL_INTCNT_MASK               ((uint8)((uint8)0x03u << Waitable_4_CTRL_INTCNT_SHIFT))
    #define Waitable_4_CTRL_TRIG_MODE_MASK            ((uint8)((uint8)0x03u << Waitable_4_CTRL_TRIG_MODE_SHIFT))
    #define Waitable_4_CTRL_TRIG_EN                   ((uint8)((uint8)0x01u << Waitable_4_CTRL_TRIG_EN_SHIFT))
    #define Waitable_4_CTRL_CAP_MODE_MASK             ((uint8)((uint8)0x03u << Waitable_4_CTRL_CAP_MODE_SHIFT))
    #define Waitable_4_CTRL_ENABLE                    ((uint8)((uint8)0x01u << Waitable_4_CTRL_ENABLE_SHIFT))

    /* Bit Counter (7-bit) Control Register Bit Definitions */
    /* As defined by the Register map for the AUX Control Register */
    #define Waitable_4_CNTR_ENABLE                    0x20u

    /* Status Register Bit Locations */
    #define Waitable_4_STATUS_TC_SHIFT                0x00u  /* As defined by Verilog Implementation */
    #define Waitable_4_STATUS_CAPTURE_SHIFT           0x01u  /* As defined by Verilog Implementation */
    #define Waitable_4_STATUS_TC_INT_MASK_SHIFT       Waitable_4_STATUS_TC_SHIFT
    #define Waitable_4_STATUS_CAPTURE_INT_MASK_SHIFT  Waitable_4_STATUS_CAPTURE_SHIFT
    #define Waitable_4_STATUS_FIFOFULL_SHIFT          0x02u  /* As defined by Verilog Implementation */
    #define Waitable_4_STATUS_FIFONEMP_SHIFT          0x03u  /* As defined by Verilog Implementation */
    #define Waitable_4_STATUS_FIFOFULL_INT_MASK_SHIFT Waitable_4_STATUS_FIFOFULL_SHIFT

    /* Status Register Bit Masks */
    /* Sticky TC Event Bit-Mask */
    #define Waitable_4_STATUS_TC                      ((uint8)((uint8)0x01u << Waitable_4_STATUS_TC_SHIFT))
    /* Sticky Capture Event Bit-Mask */
    #define Waitable_4_STATUS_CAPTURE                 ((uint8)((uint8)0x01u << Waitable_4_STATUS_CAPTURE_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define Waitable_4_STATUS_TC_INT_MASK             ((uint8)((uint8)0x01u << Waitable_4_STATUS_TC_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define Waitable_4_STATUS_CAPTURE_INT_MASK        ((uint8)((uint8)0x01u << Waitable_4_STATUS_CAPTURE_SHIFT))
    /* NOT-Sticky FIFO Full Bit-Mask */
    #define Waitable_4_STATUS_FIFOFULL                ((uint8)((uint8)0x01u << Waitable_4_STATUS_FIFOFULL_SHIFT))
    /* NOT-Sticky FIFO Not Empty Bit-Mask */
    #define Waitable_4_STATUS_FIFONEMP                ((uint8)((uint8)0x01u << Waitable_4_STATUS_FIFONEMP_SHIFT))
    /* Interrupt Enable Bit-Mask */
    #define Waitable_4_STATUS_FIFOFULL_INT_MASK       ((uint8)((uint8)0x01u << Waitable_4_STATUS_FIFOFULL_SHIFT))

    #define Waitable_4_STATUS_ACTL_INT_EN             0x10u   /* As defined for the ACTL Register */

    /* Datapath Auxillary Control Register definitions */
    #define Waitable_4_AUX_CTRL_FIFO0_CLR             0x01u   /* As defined by Register map */
    #define Waitable_4_AUX_CTRL_FIFO1_CLR             0x02u   /* As defined by Register map */
    #define Waitable_4_AUX_CTRL_FIFO0_LVL             0x04u   /* As defined by Register map */
    #define Waitable_4_AUX_CTRL_FIFO1_LVL             0x08u   /* As defined by Register map */
    #define Waitable_4_STATUS_ACTL_INT_EN_MASK        0x10u   /* As defined for the ACTL Register */

#endif /* Implementation Specific Registers and Register Constants */

#endif  /* CY_Timer_v2_30_Waitable_4_H */


/* [] END OF FILE */
