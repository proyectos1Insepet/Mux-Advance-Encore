/*******************************************************************************
* File Name: Pistola_PM.c
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

#include "Pistola.h"


/***************************************
* Local data allocation
***************************************/

static Pistola_BACKUP_STRUCT  Pistola_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Pistola_SaveConfig
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
*  Pistola_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pistola_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Pistola_CONTROL_REG_REMOVED == 0u)
            Pistola_backup.cr = Pistola_CONTROL_REG;
        #endif /* End Pistola_CONTROL_REG_REMOVED */

        #if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
            Pistola_backup.rx_period = Pistola_RXBITCTR_PERIOD_REG;
            Pistola_backup.rx_mask = Pistola_RXSTATUS_MASK_REG;
            #if (Pistola_RXHW_ADDRESS_ENABLED)
                Pistola_backup.rx_addr1 = Pistola_RXADDRESS1_REG;
                Pistola_backup.rx_addr2 = Pistola_RXADDRESS2_REG;
            #endif /* End Pistola_RXHW_ADDRESS_ENABLED */
        #endif /* End Pistola_RX_ENABLED | Pistola_HD_ENABLED*/

        #if(Pistola_TX_ENABLED)
            #if(Pistola_TXCLKGEN_DP)
                Pistola_backup.tx_clk_ctr = Pistola_TXBITCLKGEN_CTR_REG;
                Pistola_backup.tx_clk_compl = Pistola_TXBITCLKTX_COMPLETE_REG;
            #else
                Pistola_backup.tx_period = Pistola_TXBITCTR_PERIOD_REG;
            #endif /*End Pistola_TXCLKGEN_DP */
            Pistola_backup.tx_mask = Pistola_TXSTATUS_MASK_REG;
        #endif /*End Pistola_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Pistola_CONTROL_REG_REMOVED == 0u)
            Pistola_backup.cr = Pistola_CONTROL_REG;
        #endif /* End Pistola_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Pistola_RestoreConfig
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
*  Pistola_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pistola_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Pistola_CONTROL_REG_REMOVED == 0u)
            Pistola_CONTROL_REG = Pistola_backup.cr;
        #endif /* End Pistola_CONTROL_REG_REMOVED */

        #if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
            Pistola_RXBITCTR_PERIOD_REG = Pistola_backup.rx_period;
            Pistola_RXSTATUS_MASK_REG = Pistola_backup.rx_mask;
            #if (Pistola_RXHW_ADDRESS_ENABLED)
                Pistola_RXADDRESS1_REG = Pistola_backup.rx_addr1;
                Pistola_RXADDRESS2_REG = Pistola_backup.rx_addr2;
            #endif /* End Pistola_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */

        #if(Pistola_TX_ENABLED)
            #if(Pistola_TXCLKGEN_DP)
                Pistola_TXBITCLKGEN_CTR_REG = Pistola_backup.tx_clk_ctr;
                Pistola_TXBITCLKTX_COMPLETE_REG = Pistola_backup.tx_clk_compl;
            #else
                Pistola_TXBITCTR_PERIOD_REG = Pistola_backup.tx_period;
            #endif /*End Pistola_TXCLKGEN_DP */
            Pistola_TXSTATUS_MASK_REG = Pistola_backup.tx_mask;
        #endif /*End Pistola_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Pistola_CONTROL_REG_REMOVED == 0u)
            Pistola_CONTROL_REG = Pistola_backup.cr;
        #endif /* End Pistola_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Pistola_Sleep
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
*  Pistola_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pistola_Sleep(void)
{

    #if(Pistola_RX_ENABLED || Pistola_HD_ENABLED)
        if((Pistola_RXSTATUS_ACTL_REG  & Pistola_INT_ENABLE) != 0u)
        {
            Pistola_backup.enableState = 1u;
        }
        else
        {
            Pistola_backup.enableState = 0u;
        }
    #else
        if((Pistola_TXSTATUS_ACTL_REG  & Pistola_INT_ENABLE) !=0u)
        {
            Pistola_backup.enableState = 1u;
        }
        else
        {
            Pistola_backup.enableState = 0u;
        }
    #endif /* End Pistola_RX_ENABLED || Pistola_HD_ENABLED*/

    Pistola_Stop();
    Pistola_SaveConfig();
}


/*******************************************************************************
* Function Name: Pistola_Wakeup
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
*  Pistola_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pistola_Wakeup(void)
{
    Pistola_RestoreConfig();
    #if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
        Pistola_ClearRxBuffer();
    #endif /* End (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */
    #if(Pistola_TX_ENABLED || Pistola_HD_ENABLED)
        Pistola_ClearTxBuffer();
    #endif /* End Pistola_TX_ENABLED || Pistola_HD_ENABLED */

    if(Pistola_backup.enableState != 0u)
    {
        Pistola_Enable();
    }
}


/* [] END OF FILE */
