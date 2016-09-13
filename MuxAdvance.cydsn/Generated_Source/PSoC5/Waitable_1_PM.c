/*******************************************************************************
* File Name: Waitable_1_PM.c
* Version 2.70
*
*  Description:
*     This file provides the power management source code to API for the
*     Timer.
*
*   Note:
*     None
*
*******************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
********************************************************************************/

#include "Waitable_1.h"

static Waitable_1_backupStruct Waitable_1_backup;


/*******************************************************************************
* Function Name: Waitable_1_SaveConfig
********************************************************************************
*
* Summary:
*     Save the current user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Waitable_1_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Waitable_1_SaveConfig(void) 
{
    #if (!Waitable_1_UsingFixedFunction)
        Waitable_1_backup.TimerUdb = Waitable_1_ReadCounter();
        Waitable_1_backup.InterruptMaskValue = Waitable_1_STATUS_MASK;
        #if (Waitable_1_UsingHWCaptureCounter)
            Waitable_1_backup.TimerCaptureCounter = Waitable_1_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Waitable_1_UDB_CONTROL_REG_REMOVED)
            Waitable_1_backup.TimerControlRegister = Waitable_1_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Waitable_1_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Waitable_1_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_1_RestoreConfig(void) 
{   
    #if (!Waitable_1_UsingFixedFunction)

        Waitable_1_WriteCounter(Waitable_1_backup.TimerUdb);
        Waitable_1_STATUS_MASK =Waitable_1_backup.InterruptMaskValue;
        #if (Waitable_1_UsingHWCaptureCounter)
            Waitable_1_SetCaptureCount(Waitable_1_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Waitable_1_UDB_CONTROL_REG_REMOVED)
            Waitable_1_WriteControlRegister(Waitable_1_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Waitable_1_Sleep
********************************************************************************
*
* Summary:
*     Stop and Save the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Waitable_1_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Waitable_1_Sleep(void) 
{
    #if(!Waitable_1_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Waitable_1_CTRL_ENABLE == (Waitable_1_CONTROL & Waitable_1_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Waitable_1_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Waitable_1_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Waitable_1_Stop();
    Waitable_1_SaveConfig();
}


/*******************************************************************************
* Function Name: Waitable_1_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration
*
* Parameters:
*  void
*
* Return:
*  void
*
* Global variables:
*  Waitable_1_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_1_Wakeup(void) 
{
    Waitable_1_RestoreConfig();
    #if(!Waitable_1_UDB_CONTROL_REG_REMOVED)
        if(Waitable_1_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Waitable_1_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
