/*******************************************************************************
* File Name: Waitable_2_PM.c
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

#include "Waitable_2.h"

static Waitable_2_backupStruct Waitable_2_backup;


/*******************************************************************************
* Function Name: Waitable_2_SaveConfig
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
*  Waitable_2_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Waitable_2_SaveConfig(void) 
{
    #if (!Waitable_2_UsingFixedFunction)
        Waitable_2_backup.TimerUdb = Waitable_2_ReadCounter();
        Waitable_2_backup.InterruptMaskValue = Waitable_2_STATUS_MASK;
        #if (Waitable_2_UsingHWCaptureCounter)
            Waitable_2_backup.TimerCaptureCounter = Waitable_2_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Waitable_2_UDB_CONTROL_REG_REMOVED)
            Waitable_2_backup.TimerControlRegister = Waitable_2_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Waitable_2_RestoreConfig
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
*  Waitable_2_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_2_RestoreConfig(void) 
{   
    #if (!Waitable_2_UsingFixedFunction)

        Waitable_2_WriteCounter(Waitable_2_backup.TimerUdb);
        Waitable_2_STATUS_MASK =Waitable_2_backup.InterruptMaskValue;
        #if (Waitable_2_UsingHWCaptureCounter)
            Waitable_2_SetCaptureCount(Waitable_2_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Waitable_2_UDB_CONTROL_REG_REMOVED)
            Waitable_2_WriteControlRegister(Waitable_2_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Waitable_2_Sleep
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
*  Waitable_2_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Waitable_2_Sleep(void) 
{
    #if(!Waitable_2_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Waitable_2_CTRL_ENABLE == (Waitable_2_CONTROL & Waitable_2_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Waitable_2_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Waitable_2_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Waitable_2_Stop();
    Waitable_2_SaveConfig();
}


/*******************************************************************************
* Function Name: Waitable_2_Wakeup
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
*  Waitable_2_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Waitable_2_Wakeup(void) 
{
    Waitable_2_RestoreConfig();
    #if(!Waitable_2_UDB_CONTROL_REG_REMOVED)
        if(Waitable_2_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Waitable_2_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
