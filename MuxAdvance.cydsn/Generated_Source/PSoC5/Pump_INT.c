/*******************************************************************************
* File Name: Pump_INT.c
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

#include "Pump.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Pump_RX_ENABLED || Pump_HD_ENABLED) && \
     (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Pump_RXISR
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
    *  Pump_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Pump_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Pump_rxBufferOverflow - software overflow flag. Set to one
    *     when Pump_rxBufferWrite index overtakes
    *     Pump_rxBufferRead index.
    *  Pump_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Pump_rxBufferWrite is equal to
    *    Pump_rxBufferRead
    *  Pump_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Pump_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Pump_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Pump_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Pump_RXSTATUS_REG;

        if((readData & (Pump_RX_STS_BREAK | Pump_RX_STS_PAR_ERROR |
                        Pump_RX_STS_STOP_ERROR | Pump_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Pump_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Pump_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Pump_RXHW_ADDRESS_ENABLED)
                if(Pump_rxAddressMode == (uint8)Pump__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Pump_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Pump_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Pump_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Pump_rxAddressDetected = 0u;
                        }
                    }

                    readData = Pump_RXDATA_REG;
                    if(Pump_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Pump_rxBuffer[Pump_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Pump_rxBuffer[Pump_rxBufferWrite] = Pump_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Pump_rxBuffer[Pump_rxBufferWrite] = Pump_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Pump_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Pump_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Pump_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Pump_rxBufferWrite >= Pump_RXBUFFERSIZE)
                {
                    Pump_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Pump_rxBufferWrite == Pump_rxBufferRead)
                {
                    Pump_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Pump_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Pump_RXSTATUS_MASK_REG  &= (uint8)~Pump_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Pump_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Pump_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Pump_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Pump_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Pump_RX_ENABLED && (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH) */


#if(Pump_TX_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Pump_TXISR
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
    *  Pump_txBuffer - RAM buffer pointer for transmit data from.
    *  Pump_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Pump_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Pump_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Pump_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Pump_txBufferRead != Pump_txBufferWrite) &&
             ((Pump_TXSTATUS_REG & Pump_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Pump_txBufferRead >= Pump_TXBUFFERSIZE)
            {
                Pump_txBufferRead = 0u;
            }

            Pump_TXDATA_REG = Pump_txBuffer[Pump_txBufferRead];

            /* Set next pointer. */
            Pump_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Pump_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Pump_TX_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH) */


/* [] END OF FILE */
