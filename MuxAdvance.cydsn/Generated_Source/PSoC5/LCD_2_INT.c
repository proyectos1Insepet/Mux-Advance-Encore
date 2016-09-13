/*******************************************************************************
* File Name: LCD_2_INT.c
* Version 2.30
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
* Note:
*  Any unusual or non-standard behavior should be noted here. Other-
*  wise, this section should remain blank.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "LCD_2.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (LCD_2_RX_ENABLED || LCD_2_HD_ENABLED) && \
     (LCD_2_RXBUFFERSIZE > LCD_2_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: LCD_2_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  LCD_2_rxBuffer - RAM buffer pointer for save received data.
    *  LCD_2_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  LCD_2_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  LCD_2_rxBufferOverflow - software overflow flag. Set to one
    *     when LCD_2_rxBufferWrite index overtakes
    *     LCD_2_rxBufferRead index.
    *  LCD_2_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when LCD_2_rxBufferWrite is equal to
    *    LCD_2_rxBufferRead
    *  LCD_2_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  LCD_2_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(LCD_2_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START LCD_2_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = LCD_2_RXSTATUS_REG;

        if((readData & (LCD_2_RX_STS_BREAK | LCD_2_RX_STS_PAR_ERROR |
                        LCD_2_RX_STS_STOP_ERROR | LCD_2_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START LCD_2_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & LCD_2_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (LCD_2_RXHW_ADDRESS_ENABLED)
                if(LCD_2_rxAddressMode == (uint8)LCD_2__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & LCD_2_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & LCD_2_RX_STS_ADDR_MATCH) != 0u)
                        {
                            LCD_2_rxAddressDetected = 1u;
                        }
                        else
                        {
                            LCD_2_rxAddressDetected = 0u;
                        }
                    }

                    readData = LCD_2_RXDATA_REG;
                    if(LCD_2_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        LCD_2_rxBuffer[LCD_2_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    LCD_2_rxBuffer[LCD_2_rxBufferWrite] = LCD_2_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                LCD_2_rxBuffer[LCD_2_rxBufferWrite] = LCD_2_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(LCD_2_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    LCD_2_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                LCD_2_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(LCD_2_rxBufferWrite >= LCD_2_RXBUFFERSIZE)
                {
                    LCD_2_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(LCD_2_rxBufferWrite == LCD_2_rxBufferRead)
                {
                    LCD_2_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(LCD_2_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        LCD_2_RXSTATUS_MASK_REG  &= (uint8)~LCD_2_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(LCD_2_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End LCD_2_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = LCD_2_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LCD_2_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End LCD_2_RX_ENABLED && (LCD_2_RXBUFFERSIZE > LCD_2_FIFO_LENGTH) */


#if(LCD_2_TX_ENABLED && (LCD_2_TXBUFFERSIZE > LCD_2_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: LCD_2_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  LCD_2_txBuffer - RAM buffer pointer for transmit data from.
    *  LCD_2_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  LCD_2_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(LCD_2_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START LCD_2_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((LCD_2_txBufferRead != LCD_2_txBufferWrite) &&
             ((LCD_2_TXSTATUS_REG & LCD_2_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(LCD_2_txBufferRead >= LCD_2_TXBUFFERSIZE)
            {
                LCD_2_txBufferRead = 0u;
            }

            LCD_2_TXDATA_REG = LCD_2_txBuffer[LCD_2_txBufferRead];

            /* Set next pointer. */
            LCD_2_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LCD_2_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End LCD_2_TX_ENABLED && (LCD_2_TXBUFFERSIZE > LCD_2_FIFO_LENGTH) */


/* [] END OF FILE */
