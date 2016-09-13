/*******************************************************************************
* File Name: LDC1_PM.c
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

#include "LDC1.h"


/***************************************
* Local data allocation
***************************************/

static LDC1_BACKUP_STRUCT  LDC1_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: LDC1_SaveConfig
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
*  LDC1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LDC1_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(LDC1_CONTROL_REG_REMOVED == 0u)
            LDC1_backup.cr = LDC1_CONTROL_REG;
        #endif /* End LDC1_CONTROL_REG_REMOVED */

        #if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
            LDC1_backup.rx_period = LDC1_RXBITCTR_PERIOD_REG;
            LDC1_backup.rx_mask = LDC1_RXSTATUS_MASK_REG;
            #if (LDC1_RXHW_ADDRESS_ENABLED)
                LDC1_backup.rx_addr1 = LDC1_RXADDRESS1_REG;
                LDC1_backup.rx_addr2 = LDC1_RXADDRESS2_REG;
            #endif /* End LDC1_RXHW_ADDRESS_ENABLED */
        #endif /* End LDC1_RX_ENABLED | LDC1_HD_ENABLED*/

        #if(LDC1_TX_ENABLED)
            #if(LDC1_TXCLKGEN_DP)
                LDC1_backup.tx_clk_ctr = LDC1_TXBITCLKGEN_CTR_REG;
                LDC1_backup.tx_clk_compl = LDC1_TXBITCLKTX_COMPLETE_REG;
            #else
                LDC1_backup.tx_period = LDC1_TXBITCTR_PERIOD_REG;
            #endif /*End LDC1_TXCLKGEN_DP */
            LDC1_backup.tx_mask = LDC1_TXSTATUS_MASK_REG;
        #endif /*End LDC1_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(LDC1_CONTROL_REG_REMOVED == 0u)
            LDC1_backup.cr = LDC1_CONTROL_REG;
        #endif /* End LDC1_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LDC1_RestoreConfig
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
*  LDC1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LDC1_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(LDC1_CONTROL_REG_REMOVED == 0u)
            LDC1_CONTROL_REG = LDC1_backup.cr;
        #endif /* End LDC1_CONTROL_REG_REMOVED */

        #if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
            LDC1_RXBITCTR_PERIOD_REG = LDC1_backup.rx_period;
            LDC1_RXSTATUS_MASK_REG = LDC1_backup.rx_mask;
            #if (LDC1_RXHW_ADDRESS_ENABLED)
                LDC1_RXADDRESS1_REG = LDC1_backup.rx_addr1;
                LDC1_RXADDRESS2_REG = LDC1_backup.rx_addr2;
            #endif /* End LDC1_RXHW_ADDRESS_ENABLED */
        #endif  /* End (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */

        #if(LDC1_TX_ENABLED)
            #if(LDC1_TXCLKGEN_DP)
                LDC1_TXBITCLKGEN_CTR_REG = LDC1_backup.tx_clk_ctr;
                LDC1_TXBITCLKTX_COMPLETE_REG = LDC1_backup.tx_clk_compl;
            #else
                LDC1_TXBITCTR_PERIOD_REG = LDC1_backup.tx_period;
            #endif /*End LDC1_TXCLKGEN_DP */
            LDC1_TXSTATUS_MASK_REG = LDC1_backup.tx_mask;
        #endif /*End LDC1_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(LDC1_CONTROL_REG_REMOVED == 0u)
            LDC1_CONTROL_REG = LDC1_backup.cr;
        #endif /* End LDC1_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LDC1_Sleep
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
*  LDC1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LDC1_Sleep(void)
{

    #if(LDC1_RX_ENABLED || LDC1_HD_ENABLED)
        if((LDC1_RXSTATUS_ACTL_REG  & LDC1_INT_ENABLE) != 0u)
        {
            LDC1_backup.enableState = 1u;
        }
        else
        {
            LDC1_backup.enableState = 0u;
        }
    #else
        if((LDC1_TXSTATUS_ACTL_REG  & LDC1_INT_ENABLE) !=0u)
        {
            LDC1_backup.enableState = 1u;
        }
        else
        {
            LDC1_backup.enableState = 0u;
        }
    #endif /* End LDC1_RX_ENABLED || LDC1_HD_ENABLED*/

    LDC1_Stop();
    LDC1_SaveConfig();
}


/*******************************************************************************
* Function Name: LDC1_Wakeup
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
*  LDC1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LDC1_Wakeup(void)
{
    LDC1_RestoreConfig();
    #if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
        LDC1_ClearRxBuffer();
    #endif /* End (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */
    #if(LDC1_TX_ENABLED || LDC1_HD_ENABLED)
        LDC1_ClearTxBuffer();
    #endif /* End LDC1_TX_ENABLED || LDC1_HD_ENABLED */

    if(LDC1_backup.enableState != 0u)
    {
        LDC1_Enable();
    }
}


/* [] END OF FILE */
