/*******************************************************************************
* File Name: LCD_2_PM.c
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

#include "LCD_2.h"


/***************************************
* Local data allocation
***************************************/

static LCD_2_BACKUP_STRUCT  LCD_2_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: LCD_2_SaveConfig
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
*  LCD_2_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_2_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(LCD_2_CONTROL_REG_REMOVED == 0u)
            LCD_2_backup.cr = LCD_2_CONTROL_REG;
        #endif /* End LCD_2_CONTROL_REG_REMOVED */

        #if( (LCD_2_RX_ENABLED) || (LCD_2_HD_ENABLED) )
            LCD_2_backup.rx_period = LCD_2_RXBITCTR_PERIOD_REG;
            LCD_2_backup.rx_mask = LCD_2_RXSTATUS_MASK_REG;
            #if (LCD_2_RXHW_ADDRESS_ENABLED)
                LCD_2_backup.rx_addr1 = LCD_2_RXADDRESS1_REG;
                LCD_2_backup.rx_addr2 = LCD_2_RXADDRESS2_REG;
            #endif /* End LCD_2_RXHW_ADDRESS_ENABLED */
        #endif /* End LCD_2_RX_ENABLED | LCD_2_HD_ENABLED*/

        #if(LCD_2_TX_ENABLED)
            #if(LCD_2_TXCLKGEN_DP)
                LCD_2_backup.tx_clk_ctr = LCD_2_TXBITCLKGEN_CTR_REG;
                LCD_2_backup.tx_clk_compl = LCD_2_TXBITCLKTX_COMPLETE_REG;
            #else
                LCD_2_backup.tx_period = LCD_2_TXBITCTR_PERIOD_REG;
            #endif /*End LCD_2_TXCLKGEN_DP */
            LCD_2_backup.tx_mask = LCD_2_TXSTATUS_MASK_REG;
        #endif /*End LCD_2_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(LCD_2_CONTROL_REG_REMOVED == 0u)
            LCD_2_backup.cr = LCD_2_CONTROL_REG;
        #endif /* End LCD_2_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD_2_RestoreConfig
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
*  LCD_2_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_2_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(LCD_2_CONTROL_REG_REMOVED == 0u)
            LCD_2_CONTROL_REG = LCD_2_backup.cr;
        #endif /* End LCD_2_CONTROL_REG_REMOVED */

        #if( (LCD_2_RX_ENABLED) || (LCD_2_HD_ENABLED) )
            LCD_2_RXBITCTR_PERIOD_REG = LCD_2_backup.rx_period;
            LCD_2_RXSTATUS_MASK_REG = LCD_2_backup.rx_mask;
            #if (LCD_2_RXHW_ADDRESS_ENABLED)
                LCD_2_RXADDRESS1_REG = LCD_2_backup.rx_addr1;
                LCD_2_RXADDRESS2_REG = LCD_2_backup.rx_addr2;
            #endif /* End LCD_2_RXHW_ADDRESS_ENABLED */
        #endif  /* End (LCD_2_RX_ENABLED) || (LCD_2_HD_ENABLED) */

        #if(LCD_2_TX_ENABLED)
            #if(LCD_2_TXCLKGEN_DP)
                LCD_2_TXBITCLKGEN_CTR_REG = LCD_2_backup.tx_clk_ctr;
                LCD_2_TXBITCLKTX_COMPLETE_REG = LCD_2_backup.tx_clk_compl;
            #else
                LCD_2_TXBITCTR_PERIOD_REG = LCD_2_backup.tx_period;
            #endif /*End LCD_2_TXCLKGEN_DP */
            LCD_2_TXSTATUS_MASK_REG = LCD_2_backup.tx_mask;
        #endif /*End LCD_2_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(LCD_2_CONTROL_REG_REMOVED == 0u)
            LCD_2_CONTROL_REG = LCD_2_backup.cr;
        #endif /* End LCD_2_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: LCD_2_Sleep
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
*  LCD_2_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_2_Sleep(void)
{

    #if(LCD_2_RX_ENABLED || LCD_2_HD_ENABLED)
        if((LCD_2_RXSTATUS_ACTL_REG  & LCD_2_INT_ENABLE) != 0u)
        {
            LCD_2_backup.enableState = 1u;
        }
        else
        {
            LCD_2_backup.enableState = 0u;
        }
    #else
        if((LCD_2_TXSTATUS_ACTL_REG  & LCD_2_INT_ENABLE) !=0u)
        {
            LCD_2_backup.enableState = 1u;
        }
        else
        {
            LCD_2_backup.enableState = 0u;
        }
    #endif /* End LCD_2_RX_ENABLED || LCD_2_HD_ENABLED*/

    LCD_2_Stop();
    LCD_2_SaveConfig();
}


/*******************************************************************************
* Function Name: LCD_2_Wakeup
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
*  LCD_2_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD_2_Wakeup(void)
{
    LCD_2_RestoreConfig();
    #if( (LCD_2_RX_ENABLED) || (LCD_2_HD_ENABLED) )
        LCD_2_ClearRxBuffer();
    #endif /* End (LCD_2_RX_ENABLED) || (LCD_2_HD_ENABLED) */
    #if(LCD_2_TX_ENABLED || LCD_2_HD_ENABLED)
        LCD_2_ClearTxBuffer();
    #endif /* End LCD_2_TX_ENABLED || LCD_2_HD_ENABLED */

    if(LCD_2_backup.enableState != 0u)
    {
        LCD_2_Enable();
    }
}


/* [] END OF FILE */
