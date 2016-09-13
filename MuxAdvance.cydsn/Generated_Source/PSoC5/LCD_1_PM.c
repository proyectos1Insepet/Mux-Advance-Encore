/*******************************************************************************
* File Name: LCD_1_PM.c
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

#include "LCD_1.h"


/***************************************
* Local data allocation
***************************************/

static LCD_1_BACKUP_STRUCT  LCD_1_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: LCD_1_SaveConfig
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
*  LCD_1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_1_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(LCD_1_CONTROL_REG_REMOVED == 0u)
            LCD_1_backup.cr = LCD_1_CONTROL_REG;
        #endif /* End LCD_1_CONTROL_REG_REMOVED */

        #if( (LCD_1_RX_ENABLED) || (LCD_1_HD_ENABLED) )
            LCD_1_backup.rx_period = LCD_1_RXBITCTR_PERIOD_REG;
            LCD_1_backup.rx_mask = LCD_1_RXSTATUS_MASK_REG;
            #if (LCD_1_RXHW_ADDRESS_ENABLED)
                LCD_1_backup.rx_addr1 = LCD_1_RXADDRESS1_REG;
                LCD_1_backup.rx_addr2 = LCD_1_RXADDRESS2_REG;
            #endif /* End LCD_1_RXHW_ADDRESS_ENABLED */
        #endif /* End LCD_1_RX_ENABLED | LCD_1_HD_ENABLED*/

        #if(LCD_1_TX_ENABLED)
            #if(LCD_1_TXCLKGEN_DP)
                LCD_1_backup.tx_clk_ctr = LCD_1_TXBITCLKGEN_CTR_REG;
                LCD_1_backup.tx_clk_compl = LCD_1_TXBITCLKTX_COMPLETE_REG;
            #else
                LCD_1_backup.tx_period = LCD_1_TXBITCTR_PERIOD_REG;
            #endif /*End LCD_1_TXCLKGEN_DP */
            LCD_1_backup.tx_mask = LCD_1_TXSTATUS_MASK_REG;
        #endif /*End LCD_1_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(LCD_1_CONTROL_REG_REMOVED == 0u)
            LCD_1_backup.cr = LCD_1_CONTROL_REG;
        #endif /* End LCD_1_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD_1_RestoreConfig
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
*  LCD_1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_1_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(LCD_1_CONTROL_REG_REMOVED == 0u)
            LCD_1_CONTROL_REG = LCD_1_backup.cr;
        #endif /* End LCD_1_CONTROL_REG_REMOVED */

        #if( (LCD_1_RX_ENABLED) || (LCD_1_HD_ENABLED) )
            LCD_1_RXBITCTR_PERIOD_REG = LCD_1_backup.rx_period;
            LCD_1_RXSTATUS_MASK_REG = LCD_1_backup.rx_mask;
            #if (LCD_1_RXHW_ADDRESS_ENABLED)
                LCD_1_RXADDRESS1_REG = LCD_1_backup.rx_addr1;
                LCD_1_RXADDRESS2_REG = LCD_1_backup.rx_addr2;
            #endif /* End LCD_1_RXHW_ADDRESS_ENABLED */
        #endif  /* End (LCD_1_RX_ENABLED) || (LCD_1_HD_ENABLED) */

        #if(LCD_1_TX_ENABLED)
            #if(LCD_1_TXCLKGEN_DP)
                LCD_1_TXBITCLKGEN_CTR_REG = LCD_1_backup.tx_clk_ctr;
                LCD_1_TXBITCLKTX_COMPLETE_REG = LCD_1_backup.tx_clk_compl;
            #else
                LCD_1_TXBITCTR_PERIOD_REG = LCD_1_backup.tx_period;
            #endif /*End LCD_1_TXCLKGEN_DP */
            LCD_1_TXSTATUS_MASK_REG = LCD_1_backup.tx_mask;
        #endif /*End LCD_1_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(LCD_1_CONTROL_REG_REMOVED == 0u)
            LCD_1_CONTROL_REG = LCD_1_backup.cr;
        #endif /* End LCD_1_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD_1_Sleep
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
*  LCD_1_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_1_Sleep(void)
{

    #if(LCD_1_RX_ENABLED || LCD_1_HD_ENABLED)
        if((LCD_1_RXSTATUS_ACTL_REG  & LCD_1_INT_ENABLE) != 0u)
        {
            LCD_1_backup.enableState = 1u;
        }
        else
        {
            LCD_1_backup.enableState = 0u;
        }
    #else
        if((LCD_1_TXSTATUS_ACTL_REG  & LCD_1_INT_ENABLE) !=0u)
        {
            LCD_1_backup.enableState = 1u;
        }
        else
        {
            LCD_1_backup.enableState = 0u;
        }
    #endif /* End LCD_1_RX_ENABLED || LCD_1_HD_ENABLED*/

    LCD_1_Stop();
    LCD_1_SaveConfig();
}


/*******************************************************************************
* Function Name: LCD_1_Wakeup
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
*  LCD_1_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_1_Wakeup(void)
{
    LCD_1_RestoreConfig();
    #if( (LCD_1_RX_ENABLED) || (LCD_1_HD_ENABLED) )
        LCD_1_ClearRxBuffer();
    #endif /* End (LCD_1_RX_ENABLED) || (LCD_1_HD_ENABLED) */
    #if(LCD_1_TX_ENABLED || LCD_1_HD_ENABLED)
        LCD_1_ClearTxBuffer();
    #endif /* End LCD_1_TX_ENABLED || LCD_1_HD_ENABLED */

    if(LCD_1_backup.enableState != 0u)
    {
        LCD_1_Enable();
    }
}


/* [] END OF FILE */
