/*******************************************************************************
* File Name: LCD2_INT.c
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

#include "LCD2.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (LCD2_RX_ENABLED || LCD2_HD_ENABLED) && \
     (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: LCD2_RXISR
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
    *  LCD2_rxBuffer - RAM buffer pointer for save received data.
    *  LCD2_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  LCD2_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  LCD2_rxBufferOverflow - software overflow flag. Set to one
    *     when LCD2_rxBufferWrite index overtakes
    *     LCD2_rxBufferRead index.
    *  LCD2_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when LCD2_rxBufferWrite is equal to
    *    LCD2_rxBufferRead
    *  LCD2_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  LCD2_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(LCD2_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START LCD2_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = LCD2_RXSTATUS_REG;

        if((readData & (LCD2_RX_STS_BREAK | LCD2_RX_STS_PAR_ERROR |
                        LCD2_RX_STS_STOP_ERROR | LCD2_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START LCD2_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & LCD2_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (LCD2_RXHW_ADDRESS_ENABLED)
                if(LCD2_rxAddressMode == (uint8)LCD2__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & LCD2_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & LCD2_RX_STS_ADDR_MATCH) != 0u)
                        {
                            LCD2_rxAddressDetected = 1u;
                        }
                        else
                        {
                            LCD2_rxAddressDetected = 0u;
                        }
                    }

                    readData = LCD2_RXDATA_REG;
                    if(LCD2_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        LCD2_rxBuffer[LCD2_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    LCD2_rxBuffer[LCD2_rxBufferWrite] = LCD2_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                LCD2_rxBuffer[LCD2_rxBufferWrite] = LCD2_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(LCD2_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    LCD2_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                LCD2_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(LCD2_rxBufferWrite >= LCD2_RXBUFFERSIZE)
                {
                    LCD2_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(LCD2_rxBufferWrite == LCD2_rxBufferRead)
                {
                    LCD2_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(LCD2_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        LCD2_RXSTATUS_MASK_REG  &= (uint8)~LCD2_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(LCD2_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End LCD2_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = LCD2_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LCD2_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End LCD2_RX_ENABLED && (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH) */


#if(LCD2_TX_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: LCD2_TXISR
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
    *  LCD2_txBuffer - RAM buffer pointer for transmit data from.
    *  LCD2_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  LCD2_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(LCD2_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START LCD2_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((LCD2_txBufferRead != LCD2_txBufferWrite) &&
             ((LCD2_TXSTATUS_REG & LCD2_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(LCD2_txBufferRead >= LCD2_TXBUFFERSIZE)
            {
                LCD2_txBufferRead = 0u;
            }

            LCD2_TXDATA_REG = LCD2_txBuffer[LCD2_txBufferRead];

            /* Set next pointer. */
            LCD2_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LCD2_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End LCD2_TX_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH) */


/* [] END OF FILE */
