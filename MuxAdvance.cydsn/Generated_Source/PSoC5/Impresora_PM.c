/*******************************************************************************
* File Name: Impresora_PM.c
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

#include "Impresora.h"


/***************************************
* Local data allocation
***************************************/

static Impresora_BACKUP_STRUCT  Impresora_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Impresora_SaveConfig
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
*  Impresora_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Impresora_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Impresora_CONTROL_REG_REMOVED == 0u)
            Impresora_backup.cr = Impresora_CONTROL_REG;
        #endif /* End Impresora_CONTROL_REG_REMOVED */

        #if( (Impresora_RX_ENABLED) || (Impresora_HD_ENABLED) )
            Impresora_backup.rx_period = Impresora_RXBITCTR_PERIOD_REG;
            Impresora_backup.rx_mask = Impresora_RXSTATUS_MASK_REG;
            #if (Impresora_RXHW_ADDRESS_ENABLED)
                Impresora_backup.rx_addr1 = Impresora_RXADDRESS1_REG;
                Impresora_backup.rx_addr2 = Impresora_RXADDRESS2_REG;
            #endif /* End Impresora_RXHW_ADDRESS_ENABLED */
        #endif /* End Impresora_RX_ENABLED | Impresora_HD_ENABLED*/

        #if(Impresora_TX_ENABLED)
            #if(Impresora_TXCLKGEN_DP)
                Impresora_backup.tx_clk_ctr = Impresora_TXBITCLKGEN_CTR_REG;
                Impresora_backup.tx_clk_compl = Impresora_TXBITCLKTX_COMPLETE_REG;
            #else
                Impresora_backup.tx_period = Impresora_TXBITCTR_PERIOD_REG;
            #endif /*End Impresora_TXCLKGEN_DP */
            Impresora_backup.tx_mask = Impresora_TXSTATUS_MASK_REG;
        #endif /*End Impresora_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Impresora_CONTROL_REG_REMOVED == 0u)
            Impresora_backup.cr = Impresora_CONTROL_REG;
        #endif /* End Impresora_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Impresora_RestoreConfig
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
*  Impresora_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Impresora_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Impresora_CONTROL_REG_REMOVED == 0u)
            Impresora_CONTROL_REG = Impresora_backup.cr;
        #endif /* End Impresora_CONTROL_REG_REMOVED */

        #if( (Impresora_RX_ENABLED) || (Impresora_HD_ENABLED) )
            Impresora_RXBITCTR_PERIOD_REG = Impresora_backup.rx_period;
            Impresora_RXSTATUS_MASK_REG = Impresora_backup.rx_mask;
            #if (Impresora_RXHW_ADDRESS_ENABLED)
                Impresora_RXADDRESS1_REG = Impresora_backup.rx_addr1;
                Impresora_RXADDRESS2_REG = Impresora_backup.rx_addr2;
            #endif /* End Impresora_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Impresora_RX_ENABLED) || (Impresora_HD_ENABLED) */

        #if(Impresora_TX_ENABLED)
            #if(Impresora_TXCLKGEN_DP)
                Impresora_TXBITCLKGEN_CTR_REG = Impresora_backup.tx_clk_ctr;
                Impresora_TXBITCLKTX_COMPLETE_REG = Impresora_backup.tx_clk_compl;
            #else
                Impresora_TXBITCTR_PERIOD_REG = Impresora_backup.tx_period;
            #endif /*End Impresora_TXCLKGEN_DP */
            Impresora_TXSTATUS_MASK_REG = Impresora_backup.tx_mask;
        #endif /*End Impresora_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Impresora_CONTROL_REG_REMOVED == 0u)
            Impresora_CONTROL_REG = Impresora_backup.cr;
        #endif /* End Impresora_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Impresora_Sleep
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
*  Impresora_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Impresora_Sleep(void)
{

    #if(Impresora_RX_ENABLED || Impresora_HD_ENABLED)
        if((Impresora_RXSTATUS_ACTL_REG  & Impresora_INT_ENABLE) != 0u)
        {
            Impresora_backup.enableState = 1u;
        }
        else
        {
            Impresora_backup.enableState = 0u;
        }
    #else
        if((Impresora_TXSTATUS_ACTL_REG  & Impresora_INT_ENABLE) !=0u)
        {
            Impresora_backup.enableState = 1u;
        }
        else
        {
            Impresora_backup.enableState = 0u;
        }
    #endif /* End Impresora_RX_ENABLED || Impresora_HD_ENABLED*/

    Impresora_Stop();
    Impresora_SaveConfig();
}


/*******************************************************************************
* Function Name: Impresora_Wakeup
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
*  Impresora_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Impresora_Wakeup(void)
{
    Impresora_RestoreConfig();
    #if( (Impresora_RX_ENABLED) || (Impresora_HD_ENABLED) )
        Impresora_ClearRxBuffer();
    #endif /* End (Impresora_RX_ENABLED) || (Impresora_HD_ENABLED) */
    #if(Impresora_TX_ENABLED || Impresora_HD_ENABLED)
        Impresora_ClearTxBuffer();
    #endif /* End Impresora_TX_ENABLED || Impresora_HD_ENABLED */

    if(Impresora_backup.enableState != 0u)
    {
        Impresora_Enable();
    }
}


/* [] END OF FILE */
