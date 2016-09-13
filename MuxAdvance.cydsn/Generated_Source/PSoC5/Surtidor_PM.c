/*******************************************************************************
* File Name: Surtidor_PM.c
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

#include "Surtidor.h"


/***************************************
* Local data allocation
***************************************/

static Surtidor_BACKUP_STRUCT  Surtidor_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Surtidor_SaveConfig
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
*  Surtidor_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Surtidor_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Surtidor_CONTROL_REG_REMOVED == 0u)
            Surtidor_backup.cr = Surtidor_CONTROL_REG;
        #endif /* End Surtidor_CONTROL_REG_REMOVED */

        #if( (Surtidor_RX_ENABLED) || (Surtidor_HD_ENABLED) )
            Surtidor_backup.rx_period = Surtidor_RXBITCTR_PERIOD_REG;
            Surtidor_backup.rx_mask = Surtidor_RXSTATUS_MASK_REG;
            #if (Surtidor_RXHW_ADDRESS_ENABLED)
                Surtidor_backup.rx_addr1 = Surtidor_RXADDRESS1_REG;
                Surtidor_backup.rx_addr2 = Surtidor_RXADDRESS2_REG;
            #endif /* End Surtidor_RXHW_ADDRESS_ENABLED */
        #endif /* End Surtidor_RX_ENABLED | Surtidor_HD_ENABLED*/

        #if(Surtidor_TX_ENABLED)
            #if(Surtidor_TXCLKGEN_DP)
                Surtidor_backup.tx_clk_ctr = Surtidor_TXBITCLKGEN_CTR_REG;
                Surtidor_backup.tx_clk_compl = Surtidor_TXBITCLKTX_COMPLETE_REG;
            #else
                Surtidor_backup.tx_period = Surtidor_TXBITCTR_PERIOD_REG;
            #endif /*End Surtidor_TXCLKGEN_DP */
            Surtidor_backup.tx_mask = Surtidor_TXSTATUS_MASK_REG;
        #endif /*End Surtidor_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Surtidor_CONTROL_REG_REMOVED == 0u)
            Surtidor_backup.cr = Surtidor_CONTROL_REG;
        #endif /* End Surtidor_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Surtidor_RestoreConfig
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
*  Surtidor_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Surtidor_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Surtidor_CONTROL_REG_REMOVED == 0u)
            Surtidor_CONTROL_REG = Surtidor_backup.cr;
        #endif /* End Surtidor_CONTROL_REG_REMOVED */

        #if( (Surtidor_RX_ENABLED) || (Surtidor_HD_ENABLED) )
            Surtidor_RXBITCTR_PERIOD_REG = Surtidor_backup.rx_period;
            Surtidor_RXSTATUS_MASK_REG = Surtidor_backup.rx_mask;
            #if (Surtidor_RXHW_ADDRESS_ENABLED)
                Surtidor_RXADDRESS1_REG = Surtidor_backup.rx_addr1;
                Surtidor_RXADDRESS2_REG = Surtidor_backup.rx_addr2;
            #endif /* End Surtidor_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Surtidor_RX_ENABLED) || (Surtidor_HD_ENABLED) */

        #if(Surtidor_TX_ENABLED)
            #if(Surtidor_TXCLKGEN_DP)
                Surtidor_TXBITCLKGEN_CTR_REG = Surtidor_backup.tx_clk_ctr;
                Surtidor_TXBITCLKTX_COMPLETE_REG = Surtidor_backup.tx_clk_compl;
            #else
                Surtidor_TXBITCTR_PERIOD_REG = Surtidor_backup.tx_period;
            #endif /*End Surtidor_TXCLKGEN_DP */
            Surtidor_TXSTATUS_MASK_REG = Surtidor_backup.tx_mask;
        #endif /*End Surtidor_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Surtidor_CONTROL_REG_REMOVED == 0u)
            Surtidor_CONTROL_REG = Surtidor_backup.cr;
        #endif /* End Surtidor_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Surtidor_Sleep
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
*  Surtidor_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Surtidor_Sleep(void)
{

    #if(Surtidor_RX_ENABLED || Surtidor_HD_ENABLED)
        if((Surtidor_RXSTATUS_ACTL_REG  & Surtidor_INT_ENABLE) != 0u)
        {
            Surtidor_backup.enableState = 1u;
        }
        else
        {
            Surtidor_backup.enableState = 0u;
        }
    #else
        if((Surtidor_TXSTATUS_ACTL_REG  & Surtidor_INT_ENABLE) !=0u)
        {
            Surtidor_backup.enableState = 1u;
        }
        else
        {
            Surtidor_backup.enableState = 0u;
        }
    #endif /* End Surtidor_RX_ENABLED || Surtidor_HD_ENABLED*/

    Surtidor_Stop();
    Surtidor_SaveConfig();
}


/*******************************************************************************
* Function Name: Surtidor_Wakeup
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
*  Surtidor_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Surtidor_Wakeup(void)
{
    Surtidor_RestoreConfig();
    #if( (Surtidor_RX_ENABLED) || (Surtidor_HD_ENABLED) )
        Surtidor_ClearRxBuffer();
    #endif /* End (Surtidor_RX_ENABLED) || (Surtidor_HD_ENABLED) */
    #if(Surtidor_TX_ENABLED || Surtidor_HD_ENABLED)
        Surtidor_ClearTxBuffer();
    #endif /* End Surtidor_TX_ENABLED || Surtidor_HD_ENABLED */

    if(Surtidor_backup.enableState != 0u)
    {
        Surtidor_Enable();
    }
}


/* [] END OF FILE */
