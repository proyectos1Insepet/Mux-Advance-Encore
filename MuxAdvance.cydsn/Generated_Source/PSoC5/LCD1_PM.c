/*******************************************************************************
* File Name: LCD1_PM.c
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

#include "LCD1.h"


/***************************************
* Local data allocation
***************************************/

static LCD1_BACKUP_STRUCT  LCD1_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: LCD1_SaveConfig
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
*  LCD1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD1_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(LCD1_CONTROL_REG_REMOVED == 0u)
            LCD1_backup.cr = LCD1_CONTROL_REG;
        #endif /* End LCD1_CONTROL_REG_REMOVED */

        #if( (LCD1_RX_ENABLED) || (LCD1_HD_ENABLED) )
            LCD1_backup.rx_period = LCD1_RXBITCTR_PERIOD_REG;
            LCD1_backup.rx_mask = LCD1_RXSTATUS_MASK_REG;
            #if (LCD1_RXHW_ADDRESS_ENABLED)
                LCD1_backup.rx_addr1 = LCD1_RXADDRESS1_REG;
                LCD1_backup.rx_addr2 = LCD1_RXADDRESS2_REG;
            #endif /* End LCD1_RXHW_ADDRESS_ENABLED */
        #endif /* End LCD1_RX_ENABLED | LCD1_HD_ENABLED*/

        #if(LCD1_TX_ENABLED)
            #if(LCD1_TXCLKGEN_DP)
                LCD1_backup.tx_clk_ctr = LCD1_TXBITCLKGEN_CTR_REG;
                LCD1_backup.tx_clk_compl = LCD1_TXBITCLKTX_COMPLETE_REG;
            #else
                LCD1_backup.tx_period = LCD1_TXBITCTR_PERIOD_REG;
            #endif /*End LCD1_TXCLKGEN_DP */
            LCD1_backup.tx_mask = LCD1_TXSTATUS_MASK_REG;
        #endif /*End LCD1_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(LCD1_CONTROL_REG_REMOVED == 0u)
            LCD1_backup.cr = LCD1_CONTROL_REG;
        #endif /* End LCD1_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD1_RestoreConfig
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
*  LCD1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD1_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(LCD1_CONTROL_REG_REMOVED == 0u)
            LCD1_CONTROL_REG = LCD1_backup.cr;
        #endif /* End LCD1_CONTROL_REG_REMOVED */

        #if( (LCD1_RX_ENABLED) || (LCD1_HD_ENABLED) )
            LCD1_RXBITCTR_PERIOD_REG = LCD1_backup.rx_period;
            LCD1_RXSTATUS_MASK_REG = LCD1_backup.rx_mask;
            #if (LCD1_RXHW_ADDRESS_ENABLED)
                LCD1_RXADDRESS1_REG = LCD1_backup.rx_addr1;
                LCD1_RXADDRESS2_REG = LCD1_backup.rx_addr2;
            #endif /* End LCD1_RXHW_ADDRESS_ENABLED */
        #endif  /* End (LCD1_RX_ENABLED) || (LCD1_HD_ENABLED) */

        #if(LCD1_TX_ENABLED)
            #if(LCD1_TXCLKGEN_DP)
                LCD1_TXBITCLKGEN_CTR_REG = LCD1_backup.tx_clk_ctr;
                LCD1_TXBITCLKTX_COMPLETE_REG = LCD1_backup.tx_clk_compl;
            #else
                LCD1_TXBITCTR_PERIOD_REG = LCD1_backup.tx_period;
            #endif /*End LCD1_TXCLKGEN_DP */
            LCD1_TXSTATUS_MASK_REG = LCD1_backup.tx_mask;
        #endif /*End LCD1_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(LCD1_CONTROL_REG_REMOVED == 0u)
            LCD1_CONTROL_REG = LCD1_backup.cr;
        #endif /* End LCD1_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD1_Sleep
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
*  LCD1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD1_Sleep(void)
{

    #if(LCD1_RX_ENABLED || LCD1_HD_ENABLED)
        if((LCD1_RXSTATUS_ACTL_REG  & LCD1_INT_ENABLE) != 0u)
        {
            LCD1_backup.enableState = 1u;
        }
        else
        {
            LCD1_backup.enableState = 0u;
        }
    #else
        if((LCD1_TXSTATUS_ACTL_REG  & LCD1_INT_ENABLE) !=0u)
        {
            LCD1_backup.enableState = 1u;
        }
        else
        {
            LCD1_backup.enableState = 0u;
        }
    #endif /* End LCD1_RX_ENABLED || LCD1_HD_ENABLED*/

    LCD1_Stop();
    LCD1_SaveConfig();
}


/*******************************************************************************
* Function Name: LCD1_Wakeup
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
*  LCD1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD1_Wakeup(void)
{
    LCD1_RestoreConfig();
    #if( (LCD1_RX_ENABLED) || (LCD1_HD_ENABLED) )
        LCD1_ClearRxBuffer();
    #endif /* End (LCD1_RX_ENABLED) || (LCD1_HD_ENABLED) */
    #if(LCD1_TX_ENABLED || LCD1_HD_ENABLED)
        LCD1_ClearTxBuffer();
    #endif /* End LCD1_TX_ENABLED || LCD1_HD_ENABLED */

    if(LCD1_backup.enableState != 0u)
    {
        LCD1_Enable();
    }
}


/* [] END OF FILE */
