/*******************************************************************************
* File Name: Code_Bar_INT.c
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

#include "Code_Bar.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED) && \
     (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Code_Bar_RXISR
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
    *  Code_Bar_rxBuffer - RAM buffer pointer for save received data.
    *  Code_Bar_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Code_Bar_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Code_Bar_rxBufferOverflow - software overflow flag. Set to one
    *     when Code_Bar_rxBufferWrite index overtakes
    *     Code_Bar_rxBufferRead index.
    *  Code_Bar_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Code_Bar_rxBufferWrite is equal to
    *    Code_Bar_rxBufferRead
    *  Code_Bar_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Code_Bar_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Code_Bar_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Code_Bar_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Code_Bar_RXSTATUS_REG;

        if((readData & (Code_Bar_RX_STS_BREAK | Code_Bar_RX_STS_PAR_ERROR |
                        Code_Bar_RX_STS_STOP_ERROR | Code_Bar_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Code_Bar_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Code_Bar_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Code_Bar_RXHW_ADDRESS_ENABLED)
                if(Code_Bar_rxAddressMode == (uint8)Code_Bar__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Code_Bar_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Code_Bar_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Code_Bar_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Code_Bar_rxAddressDetected = 0u;
                        }
                    }

                    readData = Code_Bar_RXDATA_REG;
                    if(Code_Bar_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Code_Bar_rxBuffer[Code_Bar_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Code_Bar_rxBuffer[Code_Bar_rxBufferWrite] = Code_Bar_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Code_Bar_rxBuffer[Code_Bar_rxBufferWrite] = Code_Bar_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Code_Bar_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Code_Bar_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Code_Bar_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Code_Bar_rxBufferWrite >= Code_Bar_RXBUFFERSIZE)
                {
                    Code_Bar_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Code_Bar_rxBufferWrite == Code_Bar_rxBufferRead)
                {
                    Code_Bar_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Code_Bar_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Code_Bar_RXSTATUS_MASK_REG  &= (uint8)~Code_Bar_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Code_Bar_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Code_Bar_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Code_Bar_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Code_Bar_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Code_Bar_RX_ENABLED && (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH) */


#if(Code_Bar_TX_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Code_Bar_TXISR
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
    *  Code_Bar_txBuffer - RAM buffer pointer for transmit data from.
    *  Code_Bar_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Code_Bar_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Code_Bar_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Code_Bar_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Code_Bar_txBufferRead != Code_Bar_txBufferWrite) &&
             ((Code_Bar_TXSTATUS_REG & Code_Bar_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Code_Bar_txBufferRead >= Code_Bar_TXBUFFERSIZE)
            {
                Code_Bar_txBufferRead = 0u;
            }

            Code_Bar_TXDATA_REG = Code_Bar_txBuffer[Code_Bar_txBufferRead];

            /* Set next pointer. */
            Code_Bar_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Code_Bar_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Code_Bar_TX_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH) */


/* [] END OF FILE */
