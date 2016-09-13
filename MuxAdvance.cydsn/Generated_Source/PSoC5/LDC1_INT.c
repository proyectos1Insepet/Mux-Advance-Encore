/*******************************************************************************
* File Name: LDC1_INT.c
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

#include "LDC1.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (LDC1_RX_ENABLED || LDC1_HD_ENABLED) && \
     (LDC1_RXBUFFERSIZE > LDC1_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: LDC1_RXISR
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
    *  LDC1_rxBuffer - RAM buffer pointer for save received data.
    *  LDC1_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  LDC1_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  LDC1_rxBufferOverflow - software overflow flag. Set to one
    *     when LDC1_rxBufferWrite index overtakes
    *     LDC1_rxBufferRead index.
    *  LDC1_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when LDC1_rxBufferWrite is equal to
    *    LDC1_rxBufferRead
    *  LDC1_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  LDC1_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(LDC1_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START LDC1_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = LDC1_RXSTATUS_REG;

        if((readData & (LDC1_RX_STS_BREAK | LDC1_RX_STS_PAR_ERROR |
                        LDC1_RX_STS_STOP_ERROR | LDC1_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START LDC1_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & LDC1_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (LDC1_RXHW_ADDRESS_ENABLED)
                if(LDC1_rxAddressMode == (uint8)LDC1__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & LDC1_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & LDC1_RX_STS_ADDR_MATCH) != 0u)
                        {
                            LDC1_rxAddressDetected = 1u;
                        }
                        else
                        {
                            LDC1_rxAddressDetected = 0u;
                        }
                    }

                    readData = LDC1_RXDATA_REG;
                    if(LDC1_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        LDC1_rxBuffer[LDC1_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    LDC1_rxBuffer[LDC1_rxBufferWrite] = LDC1_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                LDC1_rxBuffer[LDC1_rxBufferWrite] = LDC1_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(LDC1_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    LDC1_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                LDC1_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(LDC1_rxBufferWrite >= LDC1_RXBUFFERSIZE)
                {
                    LDC1_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(LDC1_rxBufferWrite == LDC1_rxBufferRead)
                {
                    LDC1_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(LDC1_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        LDC1_RXSTATUS_MASK_REG  &= (uint8)~LDC1_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(LDC1_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End LDC1_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = LDC1_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LDC1_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End LDC1_RX_ENABLED && (LDC1_RXBUFFERSIZE > LDC1_FIFO_LENGTH) */


#if(LDC1_TX_ENABLED && (LDC1_TXBUFFERSIZE > LDC1_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: LDC1_TXISR
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
    *  LDC1_txBuffer - RAM buffer pointer for transmit data from.
    *  LDC1_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  LDC1_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(LDC1_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START LDC1_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((LDC1_txBufferRead != LDC1_txBufferWrite) &&
             ((LDC1_TXSTATUS_REG & LDC1_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(LDC1_txBufferRead >= LDC1_TXBUFFERSIZE)
            {
                LDC1_txBufferRead = 0u;
            }

            LDC1_TXDATA_REG = LDC1_txBuffer[LDC1_txBufferRead];

            /* Set next pointer. */
            LDC1_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START LDC1_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End LDC1_TX_ENABLED && (LDC1_TXBUFFERSIZE > LDC1_FIFO_LENGTH) */


/* [] END OF FILE */
