/*******************************************************************************
* File Name: Print_Bill_PM.c
* Version 2.30
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Print_Bill.h"


/***************************************
* Local data allocation
***************************************/

static Print_Bill_BACKUP_STRUCT  Print_Bill_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Print_Bill_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Print_Bill_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Print_Bill_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Print_Bill_CONTROL_REG_REMOVED == 0u)
            Print_Bill_backup.cr = Print_Bill_CONTROL_REG;
        #endif /* End Print_Bill_CONTROL_REG_REMOVED */

        #if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
            Print_Bill_backup.rx_period = Print_Bill_RXBITCTR_PERIOD_REG;
            Print_Bill_backup.rx_mask = Print_Bill_RXSTATUS_MASK_REG;
            #if (Print_Bill_RXHW_ADDRESS_ENABLED)
                Print_Bill_backup.rx_addr1 = Print_Bill_RXADDRESS1_REG;
                Print_Bill_backup.rx_addr2 = Print_Bill_RXADDRESS2_REG;
            #endif /* End Print_Bill_RXHW_ADDRESS_ENABLED */
        #endif /* End Print_Bill_RX_ENABLED | Print_Bill_HD_ENABLED*/

        #if(Print_Bill_TX_ENABLED)
            #if(Print_Bill_TXCLKGEN_DP)
                Print_Bill_backup.tx_clk_ctr = Print_Bill_TXBITCLKGEN_CTR_REG;
                Print_Bill_backup.tx_clk_compl = Print_Bill_TXBITCLKTX_COMPLETE_REG;
            #else
                Print_Bill_backup.tx_period = Print_Bill_TXBITCTR_PERIOD_REG;
            #endif /*End Print_Bill_TXCLKGEN_DP */
            Print_Bill_backup.tx_mask = Print_Bill_TXSTATUS_MASK_REG;
        #endif /*End Print_Bill_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Print_Bill_CONTROL_REG_REMOVED == 0u)
            Print_Bill_backup.cr = Print_Bill_CONTROL_REG;
        #endif /* End Print_Bill_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Print_Bill_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Print_Bill_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Print_Bill_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Print_Bill_CONTROL_REG_REMOVED == 0u)
            Print_Bill_CONTROL_REG = Print_Bill_backup.cr;
        #endif /* End Print_Bill_CONTROL_REG_REMOVED */

        #if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
            Print_Bill_RXBITCTR_PERIOD_REG = Print_Bill_backup.rx_period;
            Print_Bill_RXSTATUS_MASK_REG = Print_Bill_backup.rx_mask;
            #if (Print_Bill_RXHW_ADDRESS_ENABLED)
                Print_Bill_RXADDRESS1_REG = Print_Bill_backup.rx_addr1;
                Print_Bill_RXADDRESS2_REG = Print_Bill_backup.rx_addr2;
            #endif /* End Print_Bill_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */

        #if(Print_Bill_TX_ENABLED)
            #if(Print_Bill_TXCLKGEN_DP)
                Print_Bill_TXBITCLKGEN_CTR_REG = Print_Bill_backup.tx_clk_ctr;
                Print_Bill_TXBITCLKTX_COMPLETE_REG = Print_Bill_backup.tx_clk_compl;
            #else
                Print_Bill_TXBITCTR_PERIOD_REG = Print_Bill_backup.tx_period;
            #endif /*End Print_Bill_TXCLKGEN_DP */
            Print_Bill_TXSTATUS_MASK_REG = Print_Bill_backup.tx_mask;
        #endif /*End Print_Bill_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Print_Bill_CONTROL_REG_REMOVED == 0u)
            Print_Bill_CONTROL_REG = Print_Bill_backup.cr;
        #endif /* End Print_Bill_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Print_Bill_Sleep
********************************************************************************
*
* Summary:
*  Stops and saves the user configuration. Should be called
*  just prior to entering sleep.
*
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Print_Bill_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Print_Bill_Sleep(void)
{

    #if(Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED)
        if((Print_Bill_RXSTATUS_ACTL_REG  & Print_Bill_INT_ENABLE) != 0u)
        {
            Print_Bill_backup.enableState = 1u;
        }
        else
        {
            Print_Bill_backup.enableState = 0u;
        }
    #else
        if((Print_Bill_TXSTATUS_ACTL_REG  & Print_Bill_INT_ENABLE) !=0u)
        {
            Print_Bill_backup.enableState = 1u;
        }
        else
        {
            Print_Bill_backup.enableState = 0u;
        }
    #endif /* End Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED*/

    Print_Bill_Stop();
    Print_Bill_SaveConfig();
}


/*******************************************************************************
* Function Name: Print_Bill_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called
*  just after awaking from sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Print_Bill_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Print_Bill_Wakeup(void)
{
    Print_Bill_RestoreConfig();
    #if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
        Print_Bill_ClearRxBuffer();
    #endif /* End (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */
    #if(Print_Bill_TX_ENABLED || Print_Bill_HD_ENABLED)
        Print_Bill_ClearTxBuffer();
    #endif /* End Print_Bill_TX_ENABLED || Print_Bill_HD_ENABLED */

    if(Print_Bill_backup.enableState != 0u)
    {
        Print_Bill_Enable();
    }
}


/* [] END OF FILE */
