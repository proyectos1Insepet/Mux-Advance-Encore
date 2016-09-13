/*******************************************************************************
* File Name: Pump_PL_INT.c
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

#include "Pump_PL.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Pump_PL_RX_ENABLED || Pump_PL_HD_ENABLED) && \
     (Pump_PL_RXBUFFERSIZE > Pump_PL_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Pump_PL_RXISR
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
    *  Pump_PL_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_PL_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Pump_PL_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Pump_PL_rxBufferOverflow - software overflow flag. Set to one
    *     when Pump_PL_rxBufferWrite index overtakes
    *     Pump_PL_rxBufferRead index.
    *  Pump_PL_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Pump_PL_rxBufferWrite is equal to
    *    Pump_PL_rxBufferRead
    *  Pump_PL_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Pump_PL_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Pump_PL_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Pump_PL_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Pump_PL_RXSTATUS_REG;

        if((readData & (Pump_PL_RX_STS_BREAK | Pump_PL_RX_STS_PAR_ERROR |
                        Pump_PL_RX_STS_STOP_ERROR | Pump_PL_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Pump_PL_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Pump_PL_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Pump_PL_RXHW_ADDRESS_ENABLED)
                if(Pump_PL_rxAddressMode == (uint8)Pump_PL__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Pump_PL_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Pump_PL_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Pump_PL_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Pump_PL_rxAddressDetected = 0u;
                        }
                    }

                    readData = Pump_PL_RXDATA_REG;
                    if(Pump_PL_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Pump_PL_rxBuffer[Pump_PL_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Pump_PL_rxBuffer[Pump_PL_rxBufferWrite] = Pump_PL_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Pump_PL_rxBuffer[Pump_PL_rxBufferWrite] = Pump_PL_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Pump_PL_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Pump_PL_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Pump_PL_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Pump_PL_rxBufferWrite >= Pump_PL_RXBUFFERSIZE)
                {
                    Pump_PL_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Pump_PL_rxBufferWrite == Pump_PL_rxBufferRead)
                {
                    Pump_PL_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Pump_PL_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Pump_PL_RXSTATUS_MASK_REG  &= (uint8)~Pump_PL_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Pump_PL_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Pump_PL_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Pump_PL_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Pump_PL_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Pump_PL_RX_ENABLED && (Pump_PL_RXBUFFERSIZE > Pump_PL_FIFO_LENGTH) */


#if(Pump_PL_TX_ENABLED && (Pump_PL_TXBUFFERSIZE > Pump_PL_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Pump_PL_TXISR
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
    *  Pump_PL_txBuffer - RAM buffer pointer for transmit data from.
    *  Pump_PL_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Pump_PL_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Pump_PL_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Pump_PL_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Pump_PL_txBufferRead != Pump_PL_txBufferWrite) &&
             ((Pump_PL_TXSTATUS_REG & Pump_PL_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Pump_PL_txBufferRead >= Pump_PL_TXBUFFERSIZE)
            {
                Pump_PL_txBufferRead = 0u;
            }

            Pump_PL_TXDATA_REG = Pump_PL_txBuffer[Pump_PL_txBufferRead];

            /* Set next pointer. */
            Pump_PL_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Pump_PL_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Pump_PL_TX_ENABLED && (Pump_PL_TXBUFFERSIZE > Pump_PL_FIFO_LENGTH) */


/* [] END OF FILE */
