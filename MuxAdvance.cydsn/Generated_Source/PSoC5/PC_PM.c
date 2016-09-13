/*******************************************************************************
* File Name: PC_PM.c
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

#include "PC.h"


/***************************************
* Local data allocation
***************************************/

static PC_BACKUP_STRUCT  PC_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: PC_SaveConfig
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
*  PC_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PC_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(PC_CONTROL_REG_REMOVED == 0u)
            PC_backup.cr = PC_CONTROL_REG;
        #endif /* End PC_CONTROL_REG_REMOVED */

        #if( (PC_RX_ENABLED) || (PC_HD_ENABLED) )
            PC_backup.rx_period = PC_RXBITCTR_PERIOD_REG;
            PC_backup.rx_mask = PC_RXSTATUS_MASK_REG;
            #if (PC_RXHW_ADDRESS_ENABLED)
                PC_backup.rx_addr1 = PC_RXADDRESS1_REG;
                PC_backup.rx_addr2 = PC_RXADDRESS2_REG;
            #endif /* End PC_RXHW_ADDRESS_ENABLED */
        #endif /* End PC_RX_ENABLED | PC_HD_ENABLED*/

        #if(PC_TX_ENABLED)
            #if(PC_TXCLKGEN_DP)
                PC_backup.tx_clk_ctr = PC_TXBITCLKGEN_CTR_REG;
                PC_backup.tx_clk_compl = PC_TXBITCLKTX_COMPLETE_REG;
            #else
                PC_backup.tx_period = PC_TXBITCTR_PERIOD_REG;
            #endif /*End PC_TXCLKGEN_DP */
            PC_backup.tx_mask = PC_TXSTATUS_MASK_REG;
        #endif /*End PC_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(PC_CONTROL_REG_REMOVED == 0u)
            PC_backup.cr = PC_CONTROL_REG;
        #endif /* End PC_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: PC_RestoreConfig
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
*  PC_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PC_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(PC_CONTROL_REG_REMOVED == 0u)
            PC_CONTROL_REG = PC_backup.cr;
        #endif /* End PC_CONTROL_REG_REMOVED */

        #if( (PC_RX_ENABLED) || (PC_HD_ENABLED) )
            PC_RXBITCTR_PERIOD_REG = PC_backup.rx_period;
            PC_RXSTATUS_MASK_REG = PC_backup.rx_mask;
            #if (PC_RXHW_ADDRESS_ENABLED)
                PC_RXADDRESS1_REG = PC_backup.rx_addr1;
                PC_RXADDRESS2_REG = PC_backup.rx_addr2;
            #endif /* End PC_RXHW_ADDRESS_ENABLED */
        #endif  /* End (PC_RX_ENABLED) || (PC_HD_ENABLED) */

        #if(PC_TX_ENABLED)
            #if(PC_TXCLKGEN_DP)
                PC_TXBITCLKGEN_CTR_REG = PC_backup.tx_clk_ctr;
                PC_TXBITCLKTX_COMPLETE_REG = PC_backup.tx_clk_compl;
            #else
                PC_TXBITCTR_PERIOD_REG = PC_backup.tx_period;
            #endif /*End PC_TXCLKGEN_DP */
            PC_TXSTATUS_MASK_REG = PC_backup.tx_mask;
        #endif /*End PC_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(PC_CONTROL_REG_REMOVED == 0u)
            PC_CONTROL_REG = PC_backup.cr;
        #endif /* End PC_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: PC_Sleep
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
*  PC_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PC_Sleep(void)
{

    #if(PC_RX_ENABLED || PC_HD_ENABLED)
        if((PC_RXSTATUS_ACTL_REG  & PC_INT_ENABLE) != 0u)
        {
            PC_backup.enableState = 1u;
        }
        else
        {
            PC_backup.enableState = 0u;
        }
    #else
        if((PC_TXSTATUS_ACTL_REG  & PC_INT_ENABLE) !=0u)
        {
            PC_backup.enableState = 1u;
        }
        else
        {
            PC_backup.enableState = 0u;
        }
    #endif /* End PC_RX_ENABLED || PC_HD_ENABLED*/

    PC_Stop();
    PC_SaveConfig();
}


/*******************************************************************************
* Function Name: PC_Wakeup
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
*  PC_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PC_Wakeup(void)
{
    PC_RestoreConfig();
    #if( (PC_RX_ENABLED) || (PC_HD_ENABLED) )
        PC_ClearRxBuffer();
    #endif /* End (PC_RX_ENABLED) || (PC_HD_ENABLED) */
    #if(PC_TX_ENABLED || PC_HD_ENABLED)
        PC_ClearTxBuffer();
    #endif /* End PC_TX_ENABLED || PC_HD_ENABLED */

    if(PC_backup.enableState != 0u)
    {
        PC_Enable();
    }
}


/* [] END OF FILE */
