/*******************************************************************************
* File Name: Code_Bar_PM.c
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

#include "Code_Bar.h"


/***************************************
* Local data allocation
***************************************/

static Code_Bar_BACKUP_STRUCT  Code_Bar_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Code_Bar_SaveConfig
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
*  Code_Bar_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Code_Bar_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Code_Bar_CONTROL_REG_REMOVED == 0u)
            Code_Bar_backup.cr = Code_Bar_CONTROL_REG;
        #endif /* End Code_Bar_CONTROL_REG_REMOVED */

        #if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
            Code_Bar_backup.rx_period = Code_Bar_RXBITCTR_PERIOD_REG;
            Code_Bar_backup.rx_mask = Code_Bar_RXSTATUS_MASK_REG;
            #if (Code_Bar_RXHW_ADDRESS_ENABLED)
                Code_Bar_backup.rx_addr1 = Code_Bar_RXADDRESS1_REG;
                Code_Bar_backup.rx_addr2 = Code_Bar_RXADDRESS2_REG;
            #endif /* End Code_Bar_RXHW_ADDRESS_ENABLED */
        #endif /* End Code_Bar_RX_ENABLED | Code_Bar_HD_ENABLED*/

        #if(Code_Bar_TX_ENABLED)
            #if(Code_Bar_TXCLKGEN_DP)
                Code_Bar_backup.tx_clk_ctr = Code_Bar_TXBITCLKGEN_CTR_REG;
                Code_Bar_backup.tx_clk_compl = Code_Bar_TXBITCLKTX_COMPLETE_REG;
            #else
                Code_Bar_backup.tx_period = Code_Bar_TXBITCTR_PERIOD_REG;
            #endif /*End Code_Bar_TXCLKGEN_DP */
            Code_Bar_backup.tx_mask = Code_Bar_TXSTATUS_MASK_REG;
        #endif /*End Code_Bar_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Code_Bar_CONTROL_REG_REMOVED == 0u)
            Code_Bar_backup.cr = Code_Bar_CONTROL_REG;
        #endif /* End Code_Bar_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Code_Bar_RestoreConfig
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
*  Code_Bar_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Code_Bar_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Code_Bar_CONTROL_REG_REMOVED == 0u)
            Code_Bar_CONTROL_REG = Code_Bar_backup.cr;
        #endif /* End Code_Bar_CONTROL_REG_REMOVED */

        #if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
            Code_Bar_RXBITCTR_PERIOD_REG = Code_Bar_backup.rx_period;
            Code_Bar_RXSTATUS_MASK_REG = Code_Bar_backup.rx_mask;
            #if (Code_Bar_RXHW_ADDRESS_ENABLED)
                Code_Bar_RXADDRESS1_REG = Code_Bar_backup.rx_addr1;
                Code_Bar_RXADDRESS2_REG = Code_Bar_backup.rx_addr2;
            #endif /* End Code_Bar_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */

        #if(Code_Bar_TX_ENABLED)
            #if(Code_Bar_TXCLKGEN_DP)
                Code_Bar_TXBITCLKGEN_CTR_REG = Code_Bar_backup.tx_clk_ctr;
                Code_Bar_TXBITCLKTX_COMPLETE_REG = Code_Bar_backup.tx_clk_compl;
            #else
                Code_Bar_TXBITCTR_PERIOD_REG = Code_Bar_backup.tx_period;
            #endif /*End Code_Bar_TXCLKGEN_DP */
            Code_Bar_TXSTATUS_MASK_REG = Code_Bar_backup.tx_mask;
        #endif /*End Code_Bar_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Code_Bar_CONTROL_REG_REMOVED == 0u)
            Code_Bar_CONTROL_REG = Code_Bar_backup.cr;
        #endif /* End Code_Bar_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Code_Bar_Sleep
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
*  Code_Bar_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Code_Bar_Sleep(void)
{

    #if(Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED)
        if((Code_Bar_RXSTATUS_ACTL_REG  & Code_Bar_INT_ENABLE) != 0u)
        {
            Code_Bar_backup.enableState = 1u;
        }
        else
        {
            Code_Bar_backup.enableState = 0u;
        }
    #else
        if((Code_Bar_TXSTATUS_ACTL_REG  & Code_Bar_INT_ENABLE) !=0u)
        {
            Code_Bar_backup.enableState = 1u;
        }
        else
        {
            Code_Bar_backup.enableState = 0u;
        }
    #endif /* End Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED*/

    Code_Bar_Stop();
    Code_Bar_SaveConfig();
}


/*******************************************************************************
* Function Name: Code_Bar_Wakeup
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
*  Code_Bar_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Code_Bar_Wakeup(void)
{
    Code_Bar_RestoreConfig();
    #if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
        Code_Bar_ClearRxBuffer();
    #endif /* End (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */
    #if(Code_Bar_TX_ENABLED || Code_Bar_HD_ENABLED)
        Code_Bar_ClearTxBuffer();
    #endif /* End Code_Bar_TX_ENABLED || Code_Bar_HD_ENABLED */

    if(Code_Bar_backup.enableState != 0u)
    {
        Code_Bar_Enable();
    }
}


/* [] END OF FILE */
