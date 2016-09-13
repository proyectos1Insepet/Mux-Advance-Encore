/*******************************************************************************
* File Name: Impresora_INT.c
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

#include "Impresora.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Impresora_RX_ENABLED || Impresora_HD_ENABLED) && \
     (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Impresora_RXISR
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
    *  Impresora_rxBuffer - RAM buffer pointer for save received data.
    *  Impresora_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Impresora_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Impresora_rxBufferOverflow - software overflow flag. Set to one
    *     when Impresora_rxBufferWrite index overtakes
    *     Impresora_rxBufferRead index.
    *  Impresora_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Impresora_rxBufferWrite is equal to
    *    Impresora_rxBufferRead
    *  Impresora_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Impresora_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Impresora_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Impresora_RXISR_START` */
        
        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Impresora_RXSTATUS_REG;

        if((readData & (Impresora_RX_STS_BREAK | Impresora_RX_STS_PAR_ERROR |
                        Impresora_RX_STS_STOP_ERROR | Impresora_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Impresora_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Impresora_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Impresora_RXHW_ADDRESS_ENABLED)
                if(Impresora_rxAddressMode == (uint8)Impresora__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Impresora_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Impresora_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Impresora_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Impresora_rxAddressDetected = 0u;
                        }
                    }

                    readData = Impresora_RXDATA_REG;
                    if(Impresora_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Impresora_rxBuffer[Impresora_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Impresora_rxBuffer[Impresora_rxBufferWrite] = Impresora_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Impresora_rxBuffer[Impresora_rxBufferWrite] = Impresora_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Impresora_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Impresora_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Impresora_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Impresora_rxBufferWrite >= Impresora_RXBUFFERSIZE)
                {
                    Impresora_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Impresora_rxBufferWrite == Impresora_rxBufferRead)
                {
                    Impresora_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Impresora_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Impresora_RXSTATUS_MASK_REG  &= (uint8)~Impresora_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Impresora_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Impresora_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Impresora_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Impresora_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Impresora_RX_ENABLED && (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH) */


#if(Impresora_TX_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Impresora_TXISR
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
    *  Impresora_txBuffer - RAM buffer pointer for transmit data from.
    *  Impresora_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Impresora_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Impresora_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Impresora_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Impresora_txBufferRead != Impresora_txBufferWrite) &&
             ((Impresora_TXSTATUS_REG & Impresora_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Impresora_txBufferRead >= Impresora_TXBUFFERSIZE)
            {
                Impresora_txBufferRead = 0u;
            }

            Impresora_TXDATA_REG = Impresora_txBuffer[Impresora_txBufferRead];

            /* Set next pointer. */
            Impresora_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Impresora_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Impresora_TX_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH) */


/* [] END OF FILE */
