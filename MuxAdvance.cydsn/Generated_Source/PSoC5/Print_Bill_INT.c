/*******************************************************************************
* File Name: Print_Bill_INT.c
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

#include "Print_Bill.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED) && \
     (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Print_Bill_RXISR
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
    *  Print_Bill_rxBuffer - RAM buffer pointer for save received data.
    *  Print_Bill_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Print_Bill_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Print_Bill_rxBufferOverflow - software overflow flag. Set to one
    *     when Print_Bill_rxBufferWrite index overtakes
    *     Print_Bill_rxBufferRead index.
    *  Print_Bill_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Print_Bill_rxBufferWrite is equal to
    *    Print_Bill_rxBufferRead
    *  Print_Bill_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Print_Bill_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Print_Bill_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Print_Bill_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Print_Bill_RXSTATUS_REG;

        if((readData & (Print_Bill_RX_STS_BREAK | Print_Bill_RX_STS_PAR_ERROR |
                        Print_Bill_RX_STS_STOP_ERROR | Print_Bill_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Print_Bill_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Print_Bill_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Print_Bill_RXHW_ADDRESS_ENABLED)
                if(Print_Bill_rxAddressMode == (uint8)Print_Bill__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Print_Bill_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Print_Bill_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Print_Bill_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Print_Bill_rxAddressDetected = 0u;
                        }
                    }

                    readData = Print_Bill_RXDATA_REG;
                    if(Print_Bill_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Print_Bill_rxBuffer[Print_Bill_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Print_Bill_rxBuffer[Print_Bill_rxBufferWrite] = Print_Bill_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Print_Bill_rxBuffer[Print_Bill_rxBufferWrite] = Print_Bill_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Print_Bill_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Print_Bill_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Print_Bill_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Print_Bill_rxBufferWrite >= Print_Bill_RXBUFFERSIZE)
                {
                    Print_Bill_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Print_Bill_rxBufferWrite == Print_Bill_rxBufferRead)
                {
                    Print_Bill_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Print_Bill_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Print_Bill_RXSTATUS_MASK_REG  &= (uint8)~Print_Bill_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Print_Bill_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Print_Bill_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Print_Bill_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Print_Bill_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Print_Bill_RX_ENABLED && (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH) */


#if(Print_Bill_TX_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Print_Bill_TXISR
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
    *  Print_Bill_txBuffer - RAM buffer pointer for transmit data from.
    *  Print_Bill_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Print_Bill_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Print_Bill_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Print_Bill_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Print_Bill_txBufferRead != Print_Bill_txBufferWrite) &&
             ((Print_Bill_TXSTATUS_REG & Print_Bill_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Print_Bill_txBufferRead >= Print_Bill_TXBUFFERSIZE)
            {
                Print_Bill_txBufferRead = 0u;
            }

            Print_Bill_TXDATA_REG = Print_Bill_txBuffer[Print_Bill_txBufferRead];

            /* Set next pointer. */
            Print_Bill_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Print_Bill_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Print_Bill_TX_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH) */


/* [] END OF FILE */
