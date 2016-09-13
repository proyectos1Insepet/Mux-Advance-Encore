/*******************************************************************************
* File Name: Beagle_PM.c
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

#include "Beagle.h"


/***************************************
* Local data allocation
***************************************/

static Beagle_BACKUP_STRUCT  Beagle_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Beagle_SaveConfig
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
*  Beagle_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Beagle_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Beagle_CONTROL_REG_REMOVED == 0u)
            Beagle_backup.cr = Beagle_CONTROL_REG;
        #endif /* End Beagle_CONTROL_REG_REMOVED */

        #if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
            Beagle_backup.rx_period = Beagle_RXBITCTR_PERIOD_REG;
            Beagle_backup.rx_mask = Beagle_RXSTATUS_MASK_REG;
            #if (Beagle_RXHW_ADDRESS_ENABLED)
                Beagle_backup.rx_addr1 = Beagle_RXADDRESS1_REG;
                Beagle_backup.rx_addr2 = Beagle_RXADDRESS2_REG;
            #endif /* End Beagle_RXHW_ADDRESS_ENABLED */
        #endif /* End Beagle_RX_ENABLED | Beagle_HD_ENABLED*/

        #if(Beagle_TX_ENABLED)
            #if(Beagle_TXCLKGEN_DP)
                Beagle_backup.tx_clk_ctr = Beagle_TXBITCLKGEN_CTR_REG;
                Beagle_backup.tx_clk_compl = Beagle_TXBITCLKTX_COMPLETE_REG;
            #else
                Beagle_backup.tx_period = Beagle_TXBITCTR_PERIOD_REG;
            #endif /*End Beagle_TXCLKGEN_DP */
            Beagle_backup.tx_mask = Beagle_TXSTATUS_MASK_REG;
        #endif /*End Beagle_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Beagle_CONTROL_REG_REMOVED == 0u)
            Beagle_backup.cr = Beagle_CONTROL_REG;
        #endif /* End Beagle_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Beagle_RestoreConfig
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
*  Beagle_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Beagle_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Beagle_CONTROL_REG_REMOVED == 0u)
            Beagle_CONTROL_REG = Beagle_backup.cr;
        #endif /* End Beagle_CONTROL_REG_REMOVED */

        #if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
            Beagle_RXBITCTR_PERIOD_REG = Beagle_backup.rx_period;
            Beagle_RXSTATUS_MASK_REG = Beagle_backup.rx_mask;
            #if (Beagle_RXHW_ADDRESS_ENABLED)
                Beagle_RXADDRESS1_REG = Beagle_backup.rx_addr1;
                Beagle_RXADDRESS2_REG = Beagle_backup.rx_addr2;
            #endif /* End Beagle_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */

        #if(Beagle_TX_ENABLED)
            #if(Beagle_TXCLKGEN_DP)
                Beagle_TXBITCLKGEN_CTR_REG = Beagle_backup.tx_clk_ctr;
                Beagle_TXBITCLKTX_COMPLETE_REG = Beagle_backup.tx_clk_compl;
            #else
                Beagle_TXBITCTR_PERIOD_REG = Beagle_backup.tx_period;
            #endif /*End Beagle_TXCLKGEN_DP */
            Beagle_TXSTATUS_MASK_REG = Beagle_backup.tx_mask;
        #endif /*End Beagle_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Beagle_CONTROL_REG_REMOVED == 0u)
            Beagle_CONTROL_REG = Beagle_backup.cr;
        #endif /* End Beagle_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Beagle_Sleep
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
*  Beagle_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Beagle_Sleep(void)
{

    #if(Beagle_RX_ENABLED || Beagle_HD_ENABLED)
        if((Beagle_RXSTATUS_ACTL_REG  & Beagle_INT_ENABLE) != 0u)
        {
            Beagle_backup.enableState = 1u;
        }
        else
        {
            Beagle_backup.enableState = 0u;
        }
    #else
        if((Beagle_TXSTATUS_ACTL_REG  & Beagle_INT_ENABLE) !=0u)
        {
            Beagle_backup.enableState = 1u;
        }
        else
        {
            Beagle_backup.enableState = 0u;
        }
    #endif /* End Beagle_RX_ENABLED || Beagle_HD_ENABLED*/

    Beagle_Stop();
    Beagle_SaveConfig();
}


/*******************************************************************************
* Function Name: Beagle_Wakeup
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
*  Beagle_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Beagle_Wakeup(void)
{
    Beagle_RestoreConfig();
    #if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
        Beagle_ClearRxBuffer();
    #endif /* End (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */
    #if(Beagle_TX_ENABLED || Beagle_HD_ENABLED)
        Beagle_ClearTxBuffer();
    #endif /* End Beagle_TX_ENABLED || Beagle_HD_ENABLED */

    if(Beagle_backup.enableState != 0u)
    {
        Beagle_Enable();
    }
}


/* [] END OF FILE */
