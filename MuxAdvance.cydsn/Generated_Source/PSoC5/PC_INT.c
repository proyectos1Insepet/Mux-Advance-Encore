/*******************************************************************************
* File Name: PC_INT.c
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

#include "PC.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (PC_RX_ENABLED || PC_HD_ENABLED) && \
     (PC_RXBUFFERSIZE > PC_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: PC_RXISR
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
    *  PC_rxBuffer - RAM buffer pointer for save received data.
    *  PC_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  PC_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  PC_rxBufferOverflow - software overflow flag. Set to one
    *     when PC_rxBufferWrite index overtakes
    *     PC_rxBufferRead index.
    *  PC_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when PC_rxBufferWrite is equal to
    *    PC_rxBufferRead
    *  PC_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  PC_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(PC_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START PC_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = PC_RXSTATUS_REG;

        if((readData & (PC_RX_STS_BREAK | PC_RX_STS_PAR_ERROR |
                        PC_RX_STS_STOP_ERROR | PC_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START PC_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & PC_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (PC_RXHW_ADDRESS_ENABLED)
                if(PC_rxAddressMode == (uint8)PC__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & PC_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & PC_RX_STS_ADDR_MATCH) != 0u)
                        {
                            PC_rxAddressDetected = 1u;
                        }
                        else
                        {
                            PC_rxAddressDetected = 0u;
                        }
                    }

                    readData = PC_RXDATA_REG;
                    if(PC_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        PC_rxBuffer[PC_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    PC_rxBuffer[PC_rxBufferWrite] = PC_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                PC_rxBuffer[PC_rxBufferWrite] = PC_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(PC_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    PC_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                PC_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(PC_rxBufferWrite >= PC_RXBUFFERSIZE)
                {
                    PC_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(PC_rxBufferWrite == PC_rxBufferRead)
                {
                    PC_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(PC_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        PC_RXSTATUS_MASK_REG  &= (uint8)~PC_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(PC_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End PC_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = PC_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START PC_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End PC_RX_ENABLED && (PC_RXBUFFERSIZE > PC_FIFO_LENGTH) */


#if(PC_TX_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: PC_TXISR
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
    *  PC_txBuffer - RAM buffer pointer for transmit data from.
    *  PC_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  PC_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(PC_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START PC_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((PC_txBufferRead != PC_txBufferWrite) &&
             ((PC_TXSTATUS_REG & PC_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(PC_txBufferRead >= PC_TXBUFFERSIZE)
            {
                PC_txBufferRead = 0u;
            }

            PC_TXDATA_REG = PC_txBuffer[PC_txBufferRead];

            /* Set next pointer. */
            PC_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START PC_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End PC_TX_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH) */


/* [] END OF FILE */
