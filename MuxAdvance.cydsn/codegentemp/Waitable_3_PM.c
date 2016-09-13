/*******************************************************************************
* File Name: Waitable_3_PM.c
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

#include "Waitable_3.h"

static Waitable_3_backupStruct Waitable_3_backup;


/*******************************************************************************
* Function Name: Waitable_3_SaveConfig
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
*  Waitable_3_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Waitable_3_SaveConfig(void) 
{
    #if (!Waitable_3_UsingFixedFunction)
        Waitable_3_backup.TimerUdb = Waitable_3_ReadCounter();
        Waitable_3_backup.InterruptMaskValue = Waitable_3_STATUS_MASK;
        #if (Waitable_3_UsingHWCaptureCounter)
            Waitable_3_backup.TimerCaptureCounter = Waitable_3_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Waitable_3_UDB_CONTROL_REG_REMOVED)
            Waitable_3_backup.TimerControlRegister = Waitable_3_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Waitable_3_RestoreConfig
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
*  Waitable_3_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_3_RestoreConfig(void) 
{   
    #if (!Waitable_3_UsingFixedFunction)

        Waitable_3_WriteCounter(Waitable_3_backup.TimerUdb);
        Waitable_3_STATUS_MASK =Waitable_3_backup.InterruptMaskValue;
        #if (Waitable_3_UsingHWCaptureCounter)
            Waitable_3_SetCaptureCount(Waitable_3_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Waitable_3_UDB_CONTROL_REG_REMOVED)
            Waitable_3_WriteControlRegister(Waitable_3_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Waitable_3_Sleep
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
*  Waitable_3_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Waitable_3_Sleep(void) 
{
    #if(!Waitable_3_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Waitable_3_CTRL_ENABLE == (Waitable_3_CONTROL & Waitable_3_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Waitable_3_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Waitable_3_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Waitable_3_Stop();
    Waitable_3_SaveConfig();
}


/*******************************************************************************
* Function Name: Waitable_3_Wakeup
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
*  Waitable_3_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_3_Wakeup(void) 
{
    Waitable_3_RestoreConfig();
    #if(!Waitable_3_UDB_CONTROL_REG_REMOVED)
        if(Waitable_3_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Waitable_3_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
