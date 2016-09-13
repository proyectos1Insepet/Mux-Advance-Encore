/*******************************************************************************
* File Name: Pump_AL_PM.c
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

#include "Pump_AL.h"


/***************************************
* Local data allocation
***************************************/

static Pump_AL_BACKUP_STRUCT  Pump_AL_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: Pump_AL_SaveConfig
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
*  Pump_AL_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_AL_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(Pump_AL_CONTROL_REG_REMOVED == 0u)
            Pump_AL_backup.cr = Pump_AL_CONTROL_REG;
        #endif /* End Pump_AL_CONTROL_REG_REMOVED */

        #if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
            Pump_AL_backup.rx_period = Pump_AL_RXBITCTR_PERIOD_REG;
            Pump_AL_backup.rx_mask = Pump_AL_RXSTATUS_MASK_REG;
            #if (Pump_AL_RXHW_ADDRESS_ENABLED)
                Pump_AL_backup.rx_addr1 = Pump_AL_RXADDRESS1_REG;
                Pump_AL_backup.rx_addr2 = Pump_AL_RXADDRESS2_REG;
            #endif /* End Pump_AL_RXHW_ADDRESS_ENABLED */
        #endif /* End Pump_AL_RX_ENABLED | Pump_AL_HD_ENABLED*/

        #if(Pump_AL_TX_ENABLED)
            #if(Pump_AL_TXCLKGEN_DP)
                Pump_AL_backup.tx_clk_ctr = Pump_AL_TXBITCLKGEN_CTR_REG;
                Pump_AL_backup.tx_clk_compl = Pump_AL_TXBITCLKTX_COMPLETE_REG;
            #else
                Pump_AL_backup.tx_period = Pump_AL_TXBITCTR_PERIOD_REG;
            #endif /*End Pump_AL_TXCLKGEN_DP */
            Pump_AL_backup.tx_mask = Pump_AL_TXSTATUS_MASK_REG;
        #endif /*End Pump_AL_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(Pump_AL_CONTROL_REG_REMOVED == 0u)
            Pump_AL_backup.cr = Pump_AL_CONTROL_REG;
        #endif /* End Pump_AL_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Pump_AL_RestoreConfig
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
*  Pump_AL_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_AL_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(Pump_AL_CONTROL_REG_REMOVED == 0u)
            Pump_AL_CONTROL_REG = Pump_AL_backup.cr;
        #endif /* End Pump_AL_CONTROL_REG_REMOVED */

        #if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
            Pump_AL_RXBITCTR_PERIOD_REG = Pump_AL_backup.rx_period;
            Pump_AL_RXSTATUS_MASK_REG = Pump_AL_backup.rx_mask;
            #if (Pump_AL_RXHW_ADDRESS_ENABLED)
                Pump_AL_RXADDRESS1_REG = Pump_AL_backup.rx_addr1;
                Pump_AL_RXADDRESS2_REG = Pump_AL_backup.rx_addr2;
            #endif /* End Pump_AL_RXHW_ADDRESS_ENABLED */
        #endif  /* End (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */

        #if(Pump_AL_TX_ENABLED)
            #if(Pump_AL_TXCLKGEN_DP)
                Pump_AL_TXBITCLKGEN_CTR_REG = Pump_AL_backup.tx_clk_ctr;
                Pump_AL_TXBITCLKTX_COMPLETE_REG = Pump_AL_backup.tx_clk_compl;
            #else
                Pump_AL_TXBITCTR_PERIOD_REG = Pump_AL_backup.tx_period;
            #endif /*End Pump_AL_TXCLKGEN_DP */
            Pump_AL_TXSTATUS_MASK_REG = Pump_AL_backup.tx_mask;
        #endif /*End Pump_AL_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(Pump_AL_CONTROL_REG_REMOVED == 0u)
            Pump_AL_CONTROL_REG = Pump_AL_backup.cr;
        #endif /* End Pump_AL_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: Pump_AL_Sleep
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
*  Pump_AL_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_AL_Sleep(void)
{

    #if(Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED)
        if((Pump_AL_RXSTATUS_ACTL_REG  & Pump_AL_INT_ENABLE) != 0u)
        {
            Pump_AL_backup.enableState = 1u;
        }
        else
        {
            Pump_AL_backup.enableState = 0u;
        }
    #else
        if((Pump_AL_TXSTATUS_ACTL_REG  & Pump_AL_INT_ENABLE) !=0u)
        {
            Pump_AL_backup.enableState = 1u;
        }
        else
        {
            Pump_AL_backup.enableState = 0u;
        }
    #endif /* End Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED*/

    Pump_AL_Stop();
    Pump_AL_SaveConfig();
}


/*******************************************************************************
* Function Name: Pump_AL_Wakeup
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
*  Pump_AL_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_AL_Wakeup(void)
{
    Pump_AL_RestoreConfig();
    #if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
        Pump_AL_ClearRxBuffer();
    #endif /* End (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */
    #if(Pump_AL_TX_ENABLED || Pump_AL_HD_ENABLED)
        Pump_AL_ClearTxBuffer();
    #endif /* End Pump_AL_TX_ENABLED || Pump_AL_HD_ENABLED */

    if(Pump_AL_backup.enableState != 0u)
    {
        Pump_AL_Enable();
    }
}


/* [] END OF FILE */
