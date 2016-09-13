/*******************************************************************************
* File Name: Tag_INT.c
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

#include "Tag.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Tag_RX_ENABLED || Tag_HD_ENABLED) && \
     (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Tag_RXISR
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
    *  Tag_rxBuffer - RAM buffer pointer for save received data.
    *  Tag_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Tag_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Tag_rxBufferOverflow - software overflow flag. Set to one
    *     when Tag_rxBufferWrite index overtakes
    *     Tag_rxBufferRead index.
    *  Tag_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Tag_rxBufferWrite is equal to
    *    Tag_rxBufferRead
    *  Tag_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Tag_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Tag_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Tag_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Tag_RXSTATUS_REG;

        if((readData & (Tag_RX_STS_BREAK | Tag_RX_STS_PAR_ERROR |
                        Tag_RX_STS_STOP_ERROR | Tag_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Tag_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Tag_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Tag_RXHW_ADDRESS_ENABLED)
                if(Tag_rxAddressMode == (uint8)Tag__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Tag_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Tag_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Tag_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Tag_rxAddressDetected = 0u;
                        }
                    }

                    readData = Tag_RXDATA_REG;
                    if(Tag_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Tag_rxBuffer[Tag_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Tag_rxBuffer[Tag_rxBufferWrite] = Tag_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Tag_rxBuffer[Tag_rxBufferWrite] = Tag_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Tag_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Tag_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Tag_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Tag_rxBufferWrite >= Tag_RXBUFFERSIZE)
                {
                    Tag_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Tag_rxBufferWrite == Tag_rxBufferRead)
                {
                    Tag_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Tag_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Tag_RXSTATUS_MASK_REG  &= (uint8)~Tag_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Tag_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Tag_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Tag_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Tag_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Tag_RX_ENABLED && (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH) */


#if(Tag_TX_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Tag_TXISR
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
    *  Tag_txBuffer - RAM buffer pointer for transmit data from.
    *  Tag_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Tag_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Tag_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Tag_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Tag_txBufferRead != Tag_txBufferWrite) &&
             ((Tag_TXSTATUS_REG & Tag_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Tag_txBufferRead >= Tag_TXBUFFERSIZE)
            {
                Tag_txBufferRead = 0u;
            }

            Tag_TXDATA_REG = Tag_txBuffer[Tag_txBufferRead];

            /* Set next pointer. */
            Tag_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Tag_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Tag_TX_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH) */


/* [] END OF FILE */
