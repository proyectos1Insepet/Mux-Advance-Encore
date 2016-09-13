/*******************************************************************************
* File Name: LCD2_PM.c
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

#include "LCD2.h"


/***************************************
* Local data allocation
***************************************/

static LCD2_BACKUP_STRUCT  LCD2_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: LCD2_SaveConfig
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
*  LCD2_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD2_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(LCD2_CONTROL_REG_REMOVED == 0u)
            LCD2_backup.cr = LCD2_CONTROL_REG;
        #endif /* End LCD2_CONTROL_REG_REMOVED */

        #if( (LCD2_RX_ENABLED) || (LCD2_HD_ENABLED) )
            LCD2_backup.rx_period = LCD2_RXBITCTR_PERIOD_REG;
            LCD2_backup.rx_mask = LCD2_RXSTATUS_MASK_REG;
            #if (LCD2_RXHW_ADDRESS_ENABLED)
                LCD2_backup.rx_addr1 = LCD2_RXADDRESS1_REG;
                LCD2_backup.rx_addr2 = LCD2_RXADDRESS2_REG;
            #endif /* End LCD2_RXHW_ADDRESS_ENABLED */
        #endif /* End LCD2_RX_ENABLED | LCD2_HD_ENABLED*/

        #if(LCD2_TX_ENABLED)
            #if(LCD2_TXCLKGEN_DP)
                LCD2_backup.tx_clk_ctr = LCD2_TXBITCLKGEN_CTR_REG;
                LCD2_backup.tx_clk_compl = LCD2_TXBITCLKTX_COMPLETE_REG;
            #else
                LCD2_backup.tx_period = LCD2_TXBITCTR_PERIOD_REG;
            #endif /*End LCD2_TXCLKGEN_DP */
            LCD2_backup.tx_mask = LCD2_TXSTATUS_MASK_REG;
        #endif /*End LCD2_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(LCD2_CONTROL_REG_REMOVED == 0u)
            LCD2_backup.cr = LCD2_CONTROL_REG;
        #endif /* End LCD2_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD2_RestoreConfig
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
*  LCD2_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD2_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(LCD2_CONTROL_REG_REMOVED == 0u)
            LCD2_CONTROL_REG = LCD2_backup.cr;
        #endif /* End LCD2_CONTROL_REG_REMOVED */

        #if( (LCD2_RX_ENABLED) || (LCD2_HD_ENABLED) )
            LCD2_RXBITCTR_PERIOD_REG = LCD2_backup.rx_period;
            LCD2_RXSTATUS_MASK_REG = LCD2_backup.rx_mask;
            #if (LCD2_RXHW_ADDRESS_ENABLED)
                LCD2_RXADDRESS1_REG = LCD2_backup.rx_addr1;
                LCD2_RXADDRESS2_REG = LCD2_backup.rx_addr2;
            #endif /* End LCD2_RXHW_ADDRESS_ENABLED */
        #endif  /* End (LCD2_RX_ENABLED) || (LCD2_HD_ENABLED) */

        #if(LCD2_TX_ENABLED)
            #if(LCD2_TXCLKGEN_DP)
                LCD2_TXBITCLKGEN_CTR_REG = LCD2_backup.tx_clk_ctr;
                LCD2_TXBITCLKTX_COMPLETE_REG = LCD2_backup.tx_clk_compl;
            #else
                LCD2_TXBITCTR_PERIOD_REG = LCD2_backup.tx_period;
            #endif /*End LCD2_TXCLKGEN_DP */
            LCD2_TXSTATUS_MASK_REG = LCD2_backup.tx_mask;
        #endif /*End LCD2_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(LCD2_CONTROL_REG_REMOVED == 0u)
            LCD2_CONTROL_REG = LCD2_backup.cr;
        #endif /* End LCD2_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD2_Sleep
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
*  LCD2_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD2_Sleep(void)
{

    #if(LCD2_RX_ENABLED || LCD2_HD_ENABLED)
        if((LCD2_RXSTATUS_ACTL_REG  & LCD2_INT_ENABLE) != 0u)
        {
            LCD2_backup.enableState = 1u;
        }
        else
        {
            LCD2_backup.enableState = 0u;
        }
    #else
        if((LCD2_TXSTATUS_ACTL_REG  & LCD2_INT_ENABLE) !=0u)
        {
            LCD2_backup.enableState = 1u;
        }
        else
        {
            LCD2_backup.enableState = 0u;
        }
    #endif /* End LCD2_RX_ENABLED || LCD2_HD_ENABLED*/

    LCD2_Stop();
    LCD2_SaveConfig();
}


/*******************************************************************************
* Function Name: LCD2_Wakeup
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
*  LCD2_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD2_Wakeup(void)
{
    LCD2_RestoreConfig();
    #if( (LCD2_RX_ENABLED) || (LCD2_HD_ENABLED) )
        LCD2_ClearRxBuffer();
    #endif /* End (LCD2_RX_ENABLED) || (LCD2_HD_ENABLED) */
    #if(LCD2_TX_ENABLED || LCD2_HD_ENABLED)
        LCD2_ClearTxBuffer();
    #endif /* End LCD2_TX_ENABLED || LCD2_HD_ENABLED */

    if(LCD2_backup.enableState != 0u)
    {
        LCD2_Enable();
    }
}


/* [] END OF FILE */
