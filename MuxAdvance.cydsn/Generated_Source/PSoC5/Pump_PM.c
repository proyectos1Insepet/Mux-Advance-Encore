/*******************************************************************************
* File Name: Pump_PM.c
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

#include "Pump.h"


/***************************************
* Local data allocation
***************************************/

static Pump_BACKUP_STRUCT  Pump_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Pump_SaveConfig
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
*  Pump_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Pump_CONTROL_REG_REMOVED == 0u)
            Pump_backup.cr = Pump_CONTROL_REG;
        #endif /* End Pump_CONTROL_REG_REMOVED */

        #if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
            Pump_backup.rx_period = Pump_RXBITCTR_PERIOD_REG;
            Pump_backup.rx_mask = Pump_RXSTATUS_MASK_REG;
            #if (Pump_RXHW_ADDRESS_ENABLED)
                Pump_backup.rx_addr1 = Pump_RXADDRESS1_REG;
                Pump_backup.rx_addr2 = Pump_RXADDRESS2_REG;
            #endif /* End Pump_RXHW_ADDRESS_ENABLED */
        #endif /* End Pump_RX_ENABLED | Pump_HD_ENABLED*/

        #if(Pump_TX_ENABLED)
            #if(Pump_TXCLKGEN_DP)
                Pump_backup.tx_clk_ctr = Pump_TXBITCLKGEN_CTR_REG;
                Pump_backup.tx_clk_compl = Pump_TXBITCLKTX_COMPLETE_REG;
            #else
                Pump_backup.tx_period = Pump_TXBITCTR_PERIOD_REG;
            #endif /*End Pump_TXCLKGEN_DP */
            Pump_backup.tx_mask = Pump_TXSTATUS_MASK_REG;
        #endif /*End Pump_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Pump_CONTROL_REG_REMOVED == 0u)
            Pump_backup.cr = Pump_CONTROL_REG;
        #endif /* End Pump_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Pump_RestoreConfig
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
*  Pump_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Pump_CONTROL_REG_REMOVED == 0u)
            Pump_CONTROL_REG = Pump_backup.cr;
        #endif /* End Pump_CONTROL_REG_REMOVED */

        #if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
            Pump_RXBITCTR_PERIOD_REG = Pump_backup.rx_period;
            Pump_RXSTATUS_MASK_REG = Pump_backup.rx_mask;
            #if (Pump_RXHW_ADDRESS_ENABLED)
                Pump_RXADDRESS1_REG = Pump_backup.rx_addr1;
                Pump_RXADDRESS2_REG = Pump_backup.rx_addr2;
            #endif /* End Pump_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */

        #if(Pump_TX_ENABLED)
            #if(Pump_TXCLKGEN_DP)
                Pump_TXBITCLKGEN_CTR_REG = Pump_backup.tx_clk_ctr;
                Pump_TXBITCLKTX_COMPLETE_REG = Pump_backup.tx_clk_compl;
            #else
                Pump_TXBITCTR_PERIOD_REG = Pump_backup.tx_period;
            #endif /*End Pump_TXCLKGEN_DP */
            Pump_TXSTATUS_MASK_REG = Pump_backup.tx_mask;
        #endif /*End Pump_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Pump_CONTROL_REG_REMOVED == 0u)
            Pump_CONTROL_REG = Pump_backup.cr;
        #endif /* End Pump_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Pump_Sleep
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
*  Pump_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_Sleep(void)
{

    #if(Pump_RX_ENABLED || Pump_HD_ENABLED)
        if((Pump_RXSTATUS_ACTL_REG  & Pump_INT_ENABLE) != 0u)
        {
            Pump_backup.enableState = 1u;
        }
        else
        {
            Pump_backup.enableState = 0u;
        }
    #else
        if((Pump_TXSTATUS_ACTL_REG  & Pump_INT_ENABLE) !=0u)
        {
            Pump_backup.enableState = 1u;
        }
        else
        {
            Pump_backup.enableState = 0u;
        }
    #endif /* End Pump_RX_ENABLED || Pump_HD_ENABLED*/

    Pump_Stop();
    Pump_SaveConfig();
}


/*******************************************************************************
* Function Name: Pump_Wakeup
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
*  Pump_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_Wakeup(void)
{
    Pump_RestoreConfig();
    #if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
        Pump_ClearRxBuffer();
    #endif /* End (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */
    #if(Pump_TX_ENABLED || Pump_HD_ENABLED)
        Pump_ClearTxBuffer();
    #endif /* End Pump_TX_ENABLED || Pump_HD_ENABLED */

    if(Pump_backup.enableState != 0u)
    {
        Pump_Enable();
    }
}


/* [] END OF FILE */
