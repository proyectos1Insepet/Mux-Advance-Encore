/*******************************************************************************
* File Name: Beagle_INT.c
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

#include "Beagle.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Beagle_RX_ENABLED || Beagle_HD_ENABLED) && \
     (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Beagle_RXISR
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
    *  Beagle_rxBuffer - RAM buffer pointer for save received data.
    *  Beagle_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Beagle_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Beagle_rxBufferOverflow - software overflow flag. Set to one
    *     when Beagle_rxBufferWrite index overtakes
    *     Beagle_rxBufferRead index.
    *  Beagle_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Beagle_rxBufferWrite is equal to
    *    Beagle_rxBufferRead
    *  Beagle_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Beagle_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Beagle_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Beagle_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Beagle_RXSTATUS_REG;

        if((readData & (Beagle_RX_STS_BREAK | Beagle_RX_STS_PAR_ERROR |
                        Beagle_RX_STS_STOP_ERROR | Beagle_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Beagle_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Beagle_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Beagle_RXHW_ADDRESS_ENABLED)
                if(Beagle_rxAddressMode == (uint8)Beagle__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Beagle_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Beagle_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Beagle_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Beagle_rxAddressDetected = 0u;
                        }
                    }

                    readData = Beagle_RXDATA_REG;
                    if(Beagle_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Beagle_rxBuffer[Beagle_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Beagle_rxBuffer[Beagle_rxBufferWrite] = Beagle_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Beagle_rxBuffer[Beagle_rxBufferWrite] = Beagle_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Beagle_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Beagle_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Beagle_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Beagle_rxBufferWrite >= Beagle_RXBUFFERSIZE)
                {
                    Beagle_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Beagle_rxBufferWrite == Beagle_rxBufferRead)
                {
                    Beagle_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Beagle_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Beagle_RXSTATUS_MASK_REG  &= (uint8)~Beagle_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Beagle_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Beagle_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Beagle_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Beagle_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Beagle_RX_ENABLED && (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH) */


#if(Beagle_TX_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Beagle_TXISR
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
    *  Beagle_txBuffer - RAM buffer pointer for transmit data from.
    *  Beagle_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Beagle_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Beagle_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Beagle_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Beagle_txBufferRead != Beagle_txBufferWrite) &&
             ((Beagle_TXSTATUS_REG & Beagle_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Beagle_txBufferRead >= Beagle_TXBUFFERSIZE)
            {
                Beagle_txBufferRead = 0u;
            }

            Beagle_TXDATA_REG = Beagle_txBuffer[Beagle_txBufferRead];

            /* Set next pointer. */
            Beagle_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Beagle_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Beagle_TX_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH) */


/* [] END OF FILE */
