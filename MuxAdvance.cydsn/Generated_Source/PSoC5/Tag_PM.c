/*******************************************************************************
* File Name: Tag_PM.c
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

#include "Tag.h"


/***************************************
* Local data allocation
***************************************/

static Tag_BACKUP_STRUCT  Tag_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Tag_SaveConfig
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
*  Tag_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Tag_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Tag_CONTROL_REG_REMOVED == 0u)
            Tag_backup.cr = Tag_CONTROL_REG;
        #endif /* End Tag_CONTROL_REG_REMOVED */

        #if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
            Tag_backup.rx_period = Tag_RXBITCTR_PERIOD_REG;
            Tag_backup.rx_mask = Tag_RXSTATUS_MASK_REG;
            #if (Tag_RXHW_ADDRESS_ENABLED)
                Tag_backup.rx_addr1 = Tag_RXADDRESS1_REG;
                Tag_backup.rx_addr2 = Tag_RXADDRESS2_REG;
            #endif /* End Tag_RXHW_ADDRESS_ENABLED */
        #endif /* End Tag_RX_ENABLED | Tag_HD_ENABLED*/

        #if(Tag_TX_ENABLED)
            #if(Tag_TXCLKGEN_DP)
                Tag_backup.tx_clk_ctr = Tag_TXBITCLKGEN_CTR_REG;
                Tag_backup.tx_clk_compl = Tag_TXBITCLKTX_COMPLETE_REG;
            #else
                Tag_backup.tx_period = Tag_TXBITCTR_PERIOD_REG;
            #endif /*End Tag_TXCLKGEN_DP */
            Tag_backup.tx_mask = Tag_TXSTATUS_MASK_REG;
        #endif /*End Tag_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Tag_CONTROL_REG_REMOVED == 0u)
            Tag_backup.cr = Tag_CONTROL_REG;
        #endif /* End Tag_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Tag_RestoreConfig
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
*  Tag_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Tag_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Tag_CONTROL_REG_REMOVED == 0u)
            Tag_CONTROL_REG = Tag_backup.cr;
        #endif /* End Tag_CONTROL_REG_REMOVED */

        #if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
            Tag_RXBITCTR_PERIOD_REG = Tag_backup.rx_period;
            Tag_RXSTATUS_MASK_REG = Tag_backup.rx_mask;
            #if (Tag_RXHW_ADDRESS_ENABLED)
                Tag_RXADDRESS1_REG = Tag_backup.rx_addr1;
                Tag_RXADDRESS2_REG = Tag_backup.rx_addr2;
            #endif /* End Tag_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */

        #if(Tag_TX_ENABLED)
            #if(Tag_TXCLKGEN_DP)
                Tag_TXBITCLKGEN_CTR_REG = Tag_backup.tx_clk_ctr;
                Tag_TXBITCLKTX_COMPLETE_REG = Tag_backup.tx_clk_compl;
            #else
                Tag_TXBITCTR_PERIOD_REG = Tag_backup.tx_period;
            #endif /*End Tag_TXCLKGEN_DP */
            Tag_TXSTATUS_MASK_REG = Tag_backup.tx_mask;
        #endif /*End Tag_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Tag_CONTROL_REG_REMOVED == 0u)
            Tag_CONTROL_REG = Tag_backup.cr;
        #endif /* End Tag_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Tag_Sleep
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
*  Tag_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Tag_Sleep(void)
{

    #if(Tag_RX_ENABLED || Tag_HD_ENABLED)
        if((Tag_RXSTATUS_ACTL_REG  & Tag_INT_ENABLE) != 0u)
        {
            Tag_backup.enableState = 1u;
        }
        else
        {
            Tag_backup.enableState = 0u;
        }
    #else
        if((Tag_TXSTATUS_ACTL_REG  & Tag_INT_ENABLE) !=0u)
        {
            Tag_backup.enableState = 1u;
        }
        else
        {
            Tag_backup.enableState = 0u;
        }
    #endif /* End Tag_RX_ENABLED || Tag_HD_ENABLED*/

    Tag_Stop();
    Tag_SaveConfig();
}


/*******************************************************************************
* Function Name: Tag_Wakeup
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
*  Tag_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Tag_Wakeup(void)
{
    Tag_RestoreConfig();
    #if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
        Tag_ClearRxBuffer();
    #endif /* End (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */
    #if(Tag_TX_ENABLED || Tag_HD_ENABLED)
        Tag_ClearTxBuffer();
    #endif /* End Tag_TX_ENABLED || Tag_HD_ENABLED */

    if(Tag_backup.enableState != 0u)
    {
        Tag_Enable();
    }
}


/* [] END OF FILE */
