/*******************************************************************************
* File Name: Pump_AL_INT.c
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

#include "Pump_AL.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED) && \
     (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Pump_AL_RXISR
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
    *  Pump_AL_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_AL_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  Pump_AL_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  Pump_AL_rxBufferOverflow - software overflow flag. Set to one
    *     when Pump_AL_rxBufferWrite index overtakes
    *     Pump_AL_rxBufferRead index.
    *  Pump_AL_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when Pump_AL_rxBufferWrite is equal to
    *    Pump_AL_rxBufferRead
    *  Pump_AL_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  Pump_AL_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(Pump_AL_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Pump_AL_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = Pump_AL_RXSTATUS_REG;

        if((readData & (Pump_AL_RX_STS_BREAK | Pump_AL_RX_STS_PAR_ERROR |
                        Pump_AL_RX_STS_STOP_ERROR | Pump_AL_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START Pump_AL_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & Pump_AL_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (Pump_AL_RXHW_ADDRESS_ENABLED)
                if(Pump_AL_rxAddressMode == (uint8)Pump_AL__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & Pump_AL_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & Pump_AL_RX_STS_ADDR_MATCH) != 0u)
                        {
                            Pump_AL_rxAddressDetected = 1u;
                        }
                        else
                        {
                            Pump_AL_rxAddressDetected = 0u;
                        }
                    }

                    readData = Pump_AL_RXDATA_REG;
                    if(Pump_AL_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        Pump_AL_rxBuffer[Pump_AL_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    Pump_AL_rxBuffer[Pump_AL_rxBufferWrite] = Pump_AL_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                Pump_AL_rxBuffer[Pump_AL_rxBufferWrite] = Pump_AL_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(Pump_AL_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    Pump_AL_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                Pump_AL_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(Pump_AL_rxBufferWrite >= Pump_AL_RXBUFFERSIZE)
                {
                    Pump_AL_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(Pump_AL_rxBufferWrite == Pump_AL_rxBufferRead)
                {
                    Pump_AL_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(Pump_AL_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        Pump_AL_RXSTATUS_MASK_REG  &= (uint8)~Pump_AL_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(Pump_AL_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End Pump_AL_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = Pump_AL_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Pump_AL_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Pump_AL_RX_ENABLED && (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH) */


#if(Pump_AL_TX_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: Pump_AL_TXISR
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
    *  Pump_AL_txBuffer - RAM buffer pointer for transmit data from.
    *  Pump_AL_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  Pump_AL_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(Pump_AL_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START Pump_AL_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((Pump_AL_txBufferRead != Pump_AL_txBufferWrite) &&
             ((Pump_AL_TXSTATUS_REG & Pump_AL_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(Pump_AL_txBufferRead >= Pump_AL_TXBUFFERSIZE)
            {
                Pump_AL_txBufferRead = 0u;
            }

            Pump_AL_TXDATA_REG = Pump_AL_txBuffer[Pump_AL_txBufferRead];

            /* Set next pointer. */
            Pump_AL_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START Pump_AL_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End Pump_AL_TX_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH) */


/* [] END OF FILE */
