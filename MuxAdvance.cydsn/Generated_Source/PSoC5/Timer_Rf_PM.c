/*******************************************************************************
* File Name: Timer_Rf_PM.c
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

#include "Timer_Rf.h"

static Timer_Rf_backupStruct Timer_Rf_backup;


/*******************************************************************************
* Function Name: Timer_Rf_SaveConfig
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
*  Timer_Rf_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void Timer_Rf_SaveConfig(void) 
{
    #if (!Timer_Rf_UsingFixedFunction)
        Timer_Rf_backup.TimerUdb = Timer_Rf_ReadCounter();
        Timer_Rf_backup.InterruptMaskValue = Timer_Rf_STATUS_MASK;
        #if (Timer_Rf_UsingHWCaptureCounter)
            Timer_Rf_backup.TimerCaptureCounter = Timer_Rf_ReadCaptureCount();
        #endif /* Back Up capture counter register  */

        #if(!Timer_Rf_UDB_CONTROL_REG_REMOVED)
            Timer_Rf_backup.TimerControlRegister = Timer_Rf_ReadControlRegister();
        #endif /* Backup the enable state of the Timer component */
    #endif /* Backup non retention registers in UDB implementation. All fixed function registers are retention */
}


/*******************************************************************************
* Function Name: Timer_Rf_RestoreConfig
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
*  Timer_Rf_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void Timer_Rf_RestoreConfig(void) 
{   
    #if (!Timer_Rf_UsingFixedFunction)

        Timer_Rf_WriteCounter(Timer_Rf_backup.TimerUdb);
        Timer_Rf_STATUS_MASK =Timer_Rf_backup.InterruptMaskValue;
        #if (Timer_Rf_UsingHWCaptureCounter)
            Timer_Rf_SetCaptureCount(Timer_Rf_backup.TimerCaptureCounter);
        #endif /* Restore Capture counter register*/

        #if(!Timer_Rf_UDB_CONTROL_REG_REMOVED)
            Timer_Rf_WriteControlRegister(Timer_Rf_backup.TimerControlRegister);
        #endif /* Restore the enable state of the Timer component */
    #endif /* Restore non retention registers in the UDB implementation only */
}


/*******************************************************************************
* Function Name: Timer_Rf_Sleep
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
*  Timer_Rf_backup.TimerEnableState:  Is modified depending on the
*  enable state of the block before entering sleep mode.
*
*******************************************************************************/
void Timer_Rf_Sleep(void) 
{
    #if(!Timer_Rf_UDB_CONTROL_REG_REMOVED)
        /* Save Counter's enable state */
        if(Timer_Rf_CTRL_ENABLE == (Timer_Rf_CONTROL & Timer_Rf_CTRL_ENABLE))
        {
            /* Timer is enabled */
            Timer_Rf_backup.TimerEnableState = 1u;
        }
        else
        {
            /* Timer is disabled */
            Timer_Rf_backup.TimerEnableState = 0u;
        }
    #endif /* Back up enable state from the Timer control register */
    Timer_Rf_Stop();
    Timer_Rf_SaveConfig();
}


/*******************************************************************************
* Function Name: Timer_Rf_Wakeup
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
*  Timer_Rf_backup.enableState:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void Timer_Rf_Wakeup(void) 
{
    Timer_Rf_RestoreConfig();
    #if(!Timer_Rf_UDB_CONTROL_REG_REMOVED)
        if(Timer_Rf_backup.TimerEnableState == 1u)
        {     /* Enable Timer's operation */
                Timer_Rf_Enable();
        } /* Do nothing if Timer was disabled before */
    #endif /* Remove this code section if Control register is removed */
}


/* [] END OF FILE */
