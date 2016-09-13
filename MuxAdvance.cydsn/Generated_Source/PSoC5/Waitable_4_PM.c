/*******************************************************************************
* File Name: Waitable_4_PM.c
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

#include "Waitable_4.h"

static Waitable_4_backupStruct Waitable_4_backup;


/*******************************************************************************
* Function Name: Waitable_4_SaveConfig
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
*  Waitable_4_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Waitable_4_SaveConfig(void) 
{
    #if (!Waitable_4_UsingFixedFunction)
        Waitable_4_backup.TimerUdb = Waitable_4_ReadCounter();
        Waitable_4_backup.InterruptMaskValue = Waitable_4_STATUS_MASK;
        #if (Waitable_4_UsingHWCaptureCounter)
            Waitable_4_backup.TimerCaptureCounter = Waitable_4_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Waitable_4_UDB_CONTROL_REG_REMOVED)
            Waitable_4_backup.TimerControlRegister = Waitable_4_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Waitable_4_RestoreConfig
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
*  Waitable_4_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_4_RestoreConfig(void) 
{   
    #if (!Waitable_4_UsingFixedFunction)

        Waitable_4_WriteCounter(Waitable_4_backup.TimerUdb);
        Waitable_4_STATUS_MASK =Waitable_4_backup.InterruptMaskValue;
        #if (Waitable_4_UsingHWCaptureCounter)
            Waitable_4_SetCaptureCount(Waitable_4_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Waitable_4_UDB_CONTROL_REG_REMOVED)
            Waitable_4_WriteControlRegister(Waitable_4_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Waitable_4_Sleep
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
*  Waitable_4_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Waitable_4_Sleep(void) 
{
    #if(!Waitable_4_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Waitable_4_CTRL_ENABLE == (Waitable_4_CONTROL & Waitable_4_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Waitable_4_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Waitable_4_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Waitable_4_Stop();
    Waitable_4_SaveConfig();
}


/*******************************************************************************
* Function Name: Waitable_4_Wakeup
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
*  Waitable_4_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_4_Wakeup(void) 
{
    Waitable_4_RestoreConfig();
    #if(!Waitable_4_UDB_CONTROL_REG_REMOVED)
        if(Waitable_4_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Waitable_4_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
