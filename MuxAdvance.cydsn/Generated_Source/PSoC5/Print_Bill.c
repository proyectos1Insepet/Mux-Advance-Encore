/*******************************************************************************
* File Name: Print_Bill.c
* Version 2.30
*
* Description:
*  This file provides all API functionality of the UART component
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "Print_Bill.h"
#include "CyLib.h"
#if(Print_Bill_INTERNAL_CLOCK_USED)
    #include "Print_Bill_IntClock.h"
#endif /* End Print_Bill_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Print_Bill_initVar = 0u;
#if( Print_Bill_TX_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
    volatile uint8 Print_Bill_txBuffer[Print_Bill_TXBUFFERSIZE];
    volatile uint8 Print_Bill_txBufferRead = 0u;
    uint8 Print_Bill_txBufferWrite = 0u;
#endif /* End Print_Bill_TX_ENABLED */
#if( ( Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED ) && \
     (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH) )
    volatile uint8 Print_Bill_rxBuffer[Print_Bill_RXBUFFERSIZE];
    volatile uint8 Print_Bill_rxBufferRead = 0u;
    volatile uint8 Print_Bill_rxBufferWrite = 0u;
    volatile uint8 Print_Bill_rxBufferLoopDetect = 0u;
    volatile uint8 Print_Bill_rxBufferOverflow = 0u;
    #if (Print_Bill_RXHW_ADDRESS_ENABLED)
        volatile uint8 Print_Bill_rxAddressMode = Print_Bill_RXADDRESSMODE;
        volatile uint8 Print_Bill_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Print_Bill_RX_ENABLED */


/*******************************************************************************
* Function Name: Print_Bill_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the UART component.
*  Enable the clock input to enable operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The Print_Bill_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Print_Bill_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Print_Bill_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Print_Bill_initVar == 0u)
    {
        Print_Bill_Init();
        Print_Bill_initVar = 1u;
    }
    Print_Bill_Enable();
}


/*******************************************************************************
* Function Name: Print_Bill_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Print_Bill_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Print_Bill_Init(void) 
{
    #if(Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED)

        #if(Print_Bill_RX_INTERRUPT_ENABLED && (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Print_Bill_RX_VECT_NUM, &Print_Bill_RXISR);
            CyIntSetPriority(Print_Bill_RX_VECT_NUM, Print_Bill_RX_PRIOR_NUM);
        #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */

        #if (Print_Bill_RXHW_ADDRESS_ENABLED)
            Print_Bill_SetRxAddressMode(Print_Bill_RXAddressMode);
            Print_Bill_SetRxAddress1(Print_Bill_RXHWADDRESS1);
            Print_Bill_SetRxAddress2(Print_Bill_RXHWADDRESS2);
        #endif /* End Print_Bill_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Print_Bill_RXBITCTR_PERIOD_REG = Print_Bill_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Print_Bill_RXSTATUS_MASK_REG  = Print_Bill_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED*/

    #if(Print_Bill_TX_ENABLED)
        #if(Print_Bill_TX_INTERRUPT_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Print_Bill_TX_VECT_NUM, &Print_Bill_TXISR);
            CyIntSetPriority(Print_Bill_TX_VECT_NUM, Print_Bill_TX_PRIOR_NUM);
        #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Print_Bill_TXCLKGEN_DP)
            Print_Bill_TXBITCLKGEN_CTR_REG = Print_Bill_BIT_CENTER;
            Print_Bill_TXBITCLKTX_COMPLETE_REG = (Print_Bill_NUMBER_OF_DATA_BITS +
                        Print_Bill_NUMBER_OF_START_BIT) * Print_Bill_OVER_SAMPLE_COUNT;
        #else
            Print_Bill_TXBITCTR_PERIOD_REG = ((Print_Bill_NUMBER_OF_DATA_BITS +
                        Print_Bill_NUMBER_OF_START_BIT) * Print_Bill_OVER_SAMPLE_8) - 1u;
        #endif /* End Print_Bill_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Print_Bill_TX_INTERRUPT_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            Print_Bill_TXSTATUS_MASK_REG = Print_Bill_TX_STS_FIFO_EMPTY;
        #else
            Print_Bill_TXSTATUS_MASK_REG = Print_Bill_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Print_Bill_TX_INTERRUPT_ENABLED*/

    #endif /* End Print_Bill_TX_ENABLED */

    #if(Print_Bill_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Print_Bill_WriteControlRegister( \
            (Print_Bill_ReadControlRegister() & (uint8)~Print_Bill_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Print_Bill_PARITY_TYPE << Print_Bill_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Print_Bill_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Print_Bill_Enable
********************************************************************************
*
* Summary:
*  Enables the UART block operation
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  Print_Bill_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Print_Bill_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Print_Bill_RXBITCTR_CONTROL_REG |= Print_Bill_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Print_Bill_RXSTATUS_ACTL_REG  |= Print_Bill_INT_ENABLE;
        #if(Print_Bill_RX_INTERRUPT_ENABLED && (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            CyIntEnable(Print_Bill_RX_VECT_NUM);
            #if (Print_Bill_RXHW_ADDRESS_ENABLED)
                Print_Bill_rxAddressDetected = 0u;
            #endif /* End Print_Bill_RXHW_ADDRESS_ENABLED */
        #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */
    #endif /* End Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED*/

    #if(Print_Bill_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Print_Bill_TXCLKGEN_DP)
            Print_Bill_TXBITCTR_CONTROL_REG |= Print_Bill_CNTR_ENABLE;
        #endif /* End Print_Bill_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Print_Bill_TXSTATUS_ACTL_REG |= Print_Bill_INT_ENABLE;
        #if(Print_Bill_TX_INTERRUPT_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            CyIntEnable(Print_Bill_TX_VECT_NUM);
        #endif /* End Print_Bill_TX_INTERRUPT_ENABLED*/
     #endif /* End Print_Bill_TX_ENABLED */

    #if(Print_Bill_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Print_Bill_IntClock_Start();
    #endif /* End Print_Bill_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Print_Bill_Stop
********************************************************************************
*
* Summary:
*  Disable the UART component
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Print_Bill_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED)
        Print_Bill_RXBITCTR_CONTROL_REG &= (uint8)~Print_Bill_CNTR_ENABLE;
    #endif /* End Print_Bill_RX_ENABLED */

    #if(Print_Bill_TX_ENABLED)
        #if(!Print_Bill_TXCLKGEN_DP)
            Print_Bill_TXBITCTR_CONTROL_REG &= (uint8)~Print_Bill_CNTR_ENABLE;
        #endif /* End Print_Bill_TXCLKGEN_DP */
    #endif /* Print_Bill_TX_ENABLED */

    #if(Print_Bill_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Print_Bill_IntClock_Stop();
    #endif /* End Print_Bill_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED)
        Print_Bill_RXSTATUS_ACTL_REG  &= (uint8)~Print_Bill_INT_ENABLE;
        #if(Print_Bill_RX_INTERRUPT_ENABLED && (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            Print_Bill_DisableRxInt();
        #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */
    #endif /* End Print_Bill_RX_ENABLED */

    #if(Print_Bill_TX_ENABLED)
        Print_Bill_TXSTATUS_ACTL_REG &= (uint8)~Print_Bill_INT_ENABLE;
        #if(Print_Bill_TX_INTERRUPT_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            Print_Bill_DisableTxInt();
        #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */
    #endif /* End Print_Bill_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Print_Bill_ReadControlRegister
********************************************************************************
*
* Summary:
*  Read the current state of the control register
*
* Parameters:
*  None.
*
* Return:
*  Current state of the control register.
*
*******************************************************************************/
uint8 Print_Bill_ReadControlRegister(void) 
{
    #if( Print_Bill_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Print_Bill_CONTROL_REG);
    #endif /* End Print_Bill_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Print_Bill_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  Print_Bill_WriteControlRegister(uint8 control) 
{
    #if( Print_Bill_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Print_Bill_CONTROL_REG = control;
    #endif /* End Print_Bill_CONTROL_REG_REMOVED */
}


#if(Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED)

    #if(Print_Bill_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Print_Bill_EnableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void Print_Bill_EnableRxInt(void) 
        {
            CyIntEnable(Print_Bill_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Print_Bill_DisableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void Print_Bill_DisableRxInt(void) 
        {
            CyIntDisable(Print_Bill_RX_VECT_NUM);
        }

    #endif /* Print_Bill_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Print_Bill_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  IntSrc:  An or'd combination of the desired status bit masks (defined in
    *           the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void Print_Bill_SetRxInterruptMode(uint8 intSrc) 
    {
        Print_Bill_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Print_Bill_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns data in RX Data register without checking status register to
    *  determine if data is valid
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  Print_Bill_rxBuffer - RAM buffer pointer for save received data.
    *  Print_Bill_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Print_Bill_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Print_Bill_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Print_Bill_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_DisableRxInt();
            #endif /* Print_Bill_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Print_Bill_rxBufferRead;
            loc_rxBufferWrite = Print_Bill_rxBufferWrite;

            if( (Print_Bill_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Print_Bill_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Print_Bill_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Print_Bill_rxBufferRead = loc_rxBufferRead;

                if(Print_Bill_rxBufferLoopDetect != 0u )
                {
                    Print_Bill_rxBufferLoopDetect = 0u;
                    #if( (Print_Bill_RX_INTERRUPT_ENABLED) && (Print_Bill_FLOW_CONTROL != 0u) && \
                         (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Print_Bill_HD_ENABLED )
                            if((Print_Bill_CONTROL_REG & Print_Bill_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Print_Bill_RXSTATUS_MASK_REG  |= Print_Bill_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Print_Bill_RXSTATUS_MASK_REG  |= Print_Bill_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Print_Bill_HD_ENABLED */
                    #endif /* Print_Bill_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Print_Bill_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_EnableRxInt();
            #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */

        #else /* Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Print_Bill_RXDATA_REG;

        #endif /* Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Print_Bill_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the current state of the status register
    *  And detect software buffer overflow.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Global Variables:
    *  Print_Bill_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Print_Bill_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Print_Bill_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Print_Bill_ReadRxStatus(void) 
    {
        uint8 status;

        status = Print_Bill_RXSTATUS_REG & Print_Bill_RX_HW_MASK;

        #if(Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH)
            if( Print_Bill_rxBufferOverflow != 0u )
            {
                status |= Print_Bill_RX_STS_SOFT_BUFF_OVER;
                Print_Bill_rxBufferOverflow = 0u;
            }
        #endif /* Print_Bill_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Print_Bill_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, if data is not available or an error
    *  condition exists, zero is returned; otherwise, character is read and
    *  returned.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  Print_Bill_rxBuffer - RAM buffer pointer for save received data.
    *  Print_Bill_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Print_Bill_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Print_Bill_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Print_Bill_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_DisableRxInt();
            #endif /* Print_Bill_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Print_Bill_rxBufferRead;
            loc_rxBufferWrite = Print_Bill_rxBufferWrite;

            if( (Print_Bill_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Print_Bill_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Print_Bill_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Print_Bill_rxBufferRead = loc_rxBufferRead;

                if(Print_Bill_rxBufferLoopDetect > 0u )
                {
                    Print_Bill_rxBufferLoopDetect = 0u;
                    #if( (Print_Bill_RX_INTERRUPT_ENABLED) && (Print_Bill_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Print_Bill_HD_ENABLED )
                            if((Print_Bill_CONTROL_REG & Print_Bill_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Print_Bill_RXSTATUS_MASK_REG  |= Print_Bill_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Print_Bill_RXSTATUS_MASK_REG  |= Print_Bill_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Print_Bill_HD_ENABLED */
                    #endif /* Print_Bill_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Print_Bill_RXSTATUS_REG;
                if((rxStatus & Print_Bill_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Print_Bill_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Print_Bill_RX_STS_BREAK | Print_Bill_RX_STS_PAR_ERROR |
                                   Print_Bill_RX_STS_STOP_ERROR | Print_Bill_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_EnableRxInt();
            #endif /* Print_Bill_RX_INTERRUPT_ENABLED */

        #else /* Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

            rxStatus =Print_Bill_RXSTATUS_REG;
            if((rxStatus & Print_Bill_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Print_Bill_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Print_Bill_RX_STS_BREAK | Print_Bill_RX_STS_PAR_ERROR |
                               Print_Bill_RX_STS_STOP_ERROR | Print_Bill_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Print_Bill_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Grab the next available byte of data from the recieve FIFO
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains Status Register and LSB contains UART RX data
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 Print_Bill_GetByte(void) 
    {
        return ( ((uint16)Print_Bill_ReadRxStatus() << 8u) | Print_Bill_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Print_Bill_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of bytes left in the RX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Integer count of the number of bytes left
    *  in the RX buffer
    *
    * Global Variables:
    *  Print_Bill_rxBufferWrite - used to calculate left bytes.
    *  Print_Bill_rxBufferRead - used to calculate left bytes.
    *  Print_Bill_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 Print_Bill_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_DisableRxInt();
            #endif /* Print_Bill_RX_INTERRUPT_ENABLED */

            if(Print_Bill_rxBufferRead == Print_Bill_rxBufferWrite)
            {
                if(Print_Bill_rxBufferLoopDetect > 0u)
                {
                    size = Print_Bill_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Print_Bill_rxBufferRead < Print_Bill_rxBufferWrite)
            {
                size = (Print_Bill_rxBufferWrite - Print_Bill_rxBufferRead);
            }
            else
            {
                size = (Print_Bill_RXBUFFERSIZE - Print_Bill_rxBufferRead) + Print_Bill_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_EnableRxInt();
            #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */

        #else /* Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Print_Bill_RXSTATUS_REG & Print_Bill_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Print_Bill_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the RX RAM buffer by setting the read and write pointers both to zero.
    *  Clears hardware RX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_rxBufferWrite - cleared to zero.
    *  Print_Bill_rxBufferRead - cleared to zero.
    *  Print_Bill_rxBufferLoopDetect - cleared to zero.
    *  Print_Bill_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *******************************************************************************/
    void Print_Bill_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Print_Bill_RXDATA_AUX_CTL_REG |=  Print_Bill_RX_FIFO_CLR;
        Print_Bill_RXDATA_AUX_CTL_REG &= (uint8)~Print_Bill_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_DisableRxInt();
            #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */

            Print_Bill_rxBufferRead = 0u;
            Print_Bill_rxBufferWrite = 0u;
            Print_Bill_rxBufferLoopDetect = 0u;
            Print_Bill_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Print_Bill_RX_INTERRUPT_ENABLED)
                Print_Bill_EnableRxInt();
            #endif /* End Print_Bill_RX_INTERRUPT_ENABLED */
        #endif /* End Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Print_Bill_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Print_Bill__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Print_Bill__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Print_Bill__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Print_Bill__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Print_Bill__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Print_Bill_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Print_Bill_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Print_Bill_RXHW_ADDRESS_ENABLED)
            #if(Print_Bill_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Print_Bill_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Print_Bill_CONTROL_REG & (uint8)~Print_Bill_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Print_Bill_CTRL_RXADDR_MODE0_SHIFT);
                Print_Bill_CONTROL_REG = tmpCtrl;
                #if(Print_Bill_RX_INTERRUPT_ENABLED && \
                   (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH) )
                    Print_Bill_rxAddressMode = addressMode;
                    Print_Bill_rxAddressDetected = 0u;
                #endif /* End Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH*/
            #endif /* End Print_Bill_CONTROL_REG_REMOVED */
        #else /* Print_Bill_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Print_Bill_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Print_Bill_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Set the first hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Print_Bill_SetRxAddress1(uint8 address) 

    {
        Print_Bill_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Print_Bill_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Set the second hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Print_Bill_SetRxAddress2(uint8 address) 
    {
        Print_Bill_RXADDRESS2_REG = address;
    }

#endif  /* Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED*/


#if( (Print_Bill_TX_ENABLED) || (Print_Bill_HD_ENABLED) )

    #if(Print_Bill_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Print_Bill_EnableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void Print_Bill_EnableTxInt(void) 
        {
            CyIntEnable(Print_Bill_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Print_Bill_DisableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void Print_Bill_DisableTxInt(void) 
        {
            CyIntDisable(Print_Bill_TX_VECT_NUM);
        }

    #endif /* Print_Bill_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Print_Bill_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  intSrc: An or'd combination of the desired status bit masks (defined in
    *          the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void Print_Bill_SetTxInterruptMode(uint8 intSrc) 
    {
        Print_Bill_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Print_Bill_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Write a byte of data to the Transmit FIFO or TX buffer to be sent when the
    *  bus is available. WriteTxData sends a byte without checking for buffer room
    *  or status. It is up to the user to separately check status.
    *
    * Parameters:
    *  TXDataByte: byte of data to place in the transmit FIFO
    *
    * Return:
    * void
    *
    * Global Variables:
    *  Print_Bill_txBuffer - RAM buffer pointer for save data for transmission
    *  Print_Bill_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Print_Bill_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Print_Bill_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Print_Bill_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Print_Bill_initVar != 0u)
        {
            #if(Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Print_Bill_TX_INTERRUPT_ENABLED)
                    Print_Bill_DisableTxInt();
                #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

                if( (Print_Bill_txBufferRead == Print_Bill_txBufferWrite) &&
                    ((Print_Bill_TXSTATUS_REG & Print_Bill_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Print_Bill_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Print_Bill_txBufferWrite >= Print_Bill_TXBUFFERSIZE)
                    {
                        Print_Bill_txBufferWrite = 0u;
                    }

                    Print_Bill_txBuffer[Print_Bill_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Print_Bill_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Print_Bill_TX_INTERRUPT_ENABLED)
                    Print_Bill_EnableTxInt();
                #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

            #else /* Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Print_Bill_TXDATA_REG = txDataByte;

            #endif /* End Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Print_Bill_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the status register for the component
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the status register which is clear on read. It is up to
    *  the user to handle all bits in this return value accordingly, even if the bit
    *  was not enabled as an interrupt source the event happened and must be handled
    *  accordingly.
    *
    *******************************************************************************/
    uint8 Print_Bill_ReadTxStatus(void) 
    {
        return(Print_Bill_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Print_Bill_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Wait to send byte until TX register or buffer has room.
    *
    * Parameters:
    *  txDataByte: The 8-bit data value to send across the UART.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_txBuffer - RAM buffer pointer for save data for transmission
    *  Print_Bill_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Print_Bill_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Print_Bill_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Print_Bill_PutChar(uint8 txDataByte) 
    {
            #if(Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Print_Bill_TXBUFFERSIZE > Print_Bill_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Print_Bill_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Print_Bill_txBufferWrite;
                    loc_txBufferRead = Print_Bill_txBufferRead;
                    #if ((Print_Bill_TXBUFFERSIZE > Print_Bill_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Print_Bill_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Print_Bill_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Print_Bill_TXSTATUS_REG & Print_Bill_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Print_Bill_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Print_Bill_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Print_Bill_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Print_Bill_TXBUFFERSIZE > Print_Bill_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Print_Bill_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Print_Bill_txBufferWrite = loc_txBufferWrite;
                    #if ((Print_Bill_TXBUFFERSIZE > Print_Bill_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Print_Bill_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

                while((Print_Bill_TXSTATUS_REG & Print_Bill_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Print_Bill_TXDATA_REG = txDataByte;

            #endif /* End Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Print_Bill_PutString
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: char pointer to character string of Data to Send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  This function will block if there is not enough memory to place the whole
    *  string, it will block until the entire string has been written to the
    *  transmit buffer.
    *
    *******************************************************************************/
    void Print_Bill_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Print_Bill_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Print_Bill_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Print_Bill_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of Bytes to be transmitted.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Print_Bill_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Print_Bill_initVar != 0u)
        {
            do
            {
                Print_Bill_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Print_Bill_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Write a character and then carriage return and line feed.
    *
    * Parameters:
    *  txDataByte: uint8 Character to send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Print_Bill_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Print_Bill_initVar != 0u)
        {
            Print_Bill_PutChar(txDataByte);
            Print_Bill_PutChar(0x0Du);
            Print_Bill_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Print_Bill_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of space left in the TX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Integer count of the number of bytes left in the TX buffer
    *
    * Global Variables:
    *  Print_Bill_txBufferWrite - used to calculate left space.
    *  Print_Bill_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Print_Bill_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Print_Bill_TX_INTERRUPT_ENABLED)
                Print_Bill_DisableTxInt();
            #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

            if(Print_Bill_txBufferRead == Print_Bill_txBufferWrite)
            {
                size = 0u;
            }
            else if(Print_Bill_txBufferRead < Print_Bill_txBufferWrite)
            {
                size = (Print_Bill_txBufferWrite - Print_Bill_txBufferRead);
            }
            else
            {
                size = (Print_Bill_TXBUFFERSIZE - Print_Bill_txBufferRead) + Print_Bill_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Print_Bill_TX_INTERRUPT_ENABLED)
                Print_Bill_EnableTxInt();
            #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

        #else /* Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

            size = Print_Bill_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Print_Bill_TX_STS_FIFO_FULL) != 0u)
            {
                size = Print_Bill_FIFO_LENGTH;
            }
            else if((size & Print_Bill_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Print_Bill_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the TX RAM buffer by setting the read and write pointers both to zero.
    *  Clears the hardware TX FIFO.  Any data present in the FIFO will not be sent.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_txBufferWrite - cleared to zero.
    *  Print_Bill_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM buffer will be lost when overwritten.
    *
    *******************************************************************************/
    void Print_Bill_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Print_Bill_TXDATA_AUX_CTL_REG |=  Print_Bill_TX_FIFO_CLR;
        Print_Bill_TXDATA_AUX_CTL_REG &= (uint8)~Print_Bill_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Print_Bill_TX_INTERRUPT_ENABLED)
                Print_Bill_DisableTxInt();
            #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

            Print_Bill_txBufferRead = 0u;
            Print_Bill_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Print_Bill_TX_INTERRUPT_ENABLED)
                Print_Bill_EnableTxInt();
            #endif /* End Print_Bill_TX_INTERRUPT_ENABLED */

        #endif /* End Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Print_Bill_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Write a Break command to the UART
    *
    * Parameters:
    *  uint8 retMode:  Wait mode,
    *   0 - Initialize registers for Break, sends the Break signal and return
    *       imediately.
    *   1 - Wait until Break sending is complete, reinitialize registers to normal
    *       transmission mode then return.
    *   2 - Reinitialize registers to normal transmission mode then return.
    *   3 - both steps: 0 and 1
    *       init registers for Break, send Break signal
    *       wait until Break sending is complete, reinit registers to normal
    *       transmission mode then return.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Print_Bill_initVar - checked to identify that the component has been
    *     initialized.
    *  tx_period - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  Trere are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Funcition will block CPU untill transmition
    *     complete.
    *  2) User may want to use bloking time if UART configured to the low speed
    *     operation
    *     Emample for this case:
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to init and use the interrupt for complete
    *     break operation.
    *     Example for this case:
    *     Init TX interrupt whith "TX - On TX Complete" parameter
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     When interrupt appear with UART_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *   Uses static variable to keep registers configuration.
    *
    *******************************************************************************/
    void Print_Bill_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Print_Bill_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Print_Bill_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Print_Bill_SEND_BREAK) ||
                    (retMode == Print_Bill_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() |
                                                          Print_Bill_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Print_Bill_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Print_Bill_TXSTATUS_REG;
                    }while((tmpStat & Print_Bill_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Print_Bill_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Print_Bill_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Print_Bill_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Print_Bill_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Print_Bill_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Print_Bill_REINIT) ||
                    (retMode == Print_Bill_SEND_WAIT_REINIT) )
                {
                    Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() &
                                                  (uint8)~Print_Bill_CTRL_HD_SEND_BREAK);
                }

            #else /* Print_Bill_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Print_Bill_SEND_BREAK) ||
                    (retMode == Print_Bill_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Print_Bill_PARITY_TYPE != Print_Bill__B_UART__NONE_REVB) || \
                                        (Print_Bill_PARITY_TYPE_SW != 0u) )
                        Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() |
                                                              Print_Bill_CTRL_HD_SEND_BREAK);
                    #endif /* End Print_Bill_PARITY_TYPE != Print_Bill__B_UART__NONE_REVB  */

                    #if(Print_Bill_TXCLKGEN_DP)
                        tx_period = Print_Bill_TXBITCLKTX_COMPLETE_REG;
                        Print_Bill_TXBITCLKTX_COMPLETE_REG = Print_Bill_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Print_Bill_TXBITCTR_PERIOD_REG;
                        Print_Bill_TXBITCTR_PERIOD_REG = Print_Bill_TXBITCTR_BREAKBITS8X;
                    #endif /* End Print_Bill_TXCLKGEN_DP */

                    /* Send zeros*/
                    Print_Bill_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Print_Bill_TXSTATUS_REG;
                    }while((tmpStat & Print_Bill_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Print_Bill_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Print_Bill_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Print_Bill_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Print_Bill_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Print_Bill_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Print_Bill_REINIT) ||
                    (retMode == Print_Bill_SEND_WAIT_REINIT) )
                {

                    #if(Print_Bill_TXCLKGEN_DP)
                        Print_Bill_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Print_Bill_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Print_Bill_TXCLKGEN_DP */

                    #if( (Print_Bill_PARITY_TYPE != Print_Bill__B_UART__NONE_REVB) || \
                         (Print_Bill_PARITY_TYPE_SW != 0u) )
                        Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() &
                                                      (uint8)~Print_Bill_CTRL_HD_SEND_BREAK);
                    #endif /* End Print_Bill_PARITY_TYPE != NONE */
                }
            #endif    /* End Print_Bill_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Print_Bill_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the transmit addressing mode
    *
    * Parameters:
    *  addressMode: 0 -> Space
    *               1 -> Mark
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void Print_Bill_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Print_Bill_CONTROL_REG_REMOVED == 0u )
                Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() |
                                                      Print_Bill_CTRL_MARK);
            #endif /* End Print_Bill_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Print_Bill_CONTROL_REG_REMOVED == 0u )
                Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() &
                                                    (uint8)~Print_Bill_CTRL_MARK);
            #endif /* End Print_Bill_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndPrint_Bill_TX_ENABLED */

#if(Print_Bill_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Print_Bill_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Rx configuration if required and loads the
    *  Tx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Tx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART.
    *
    * Side Effects:
    *  Disable RX interrupt mask, when software buffer has been used.
    *
    *******************************************************************************/
    void Print_Bill_LoadTxConfig(void) 
    {
        #if((Print_Bill_RX_INTERRUPT_ENABLED) && (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Print_Bill_SetRxInterruptMode(0u);
        #endif /* Print_Bill_RX_INTERRUPT_ENABLED */

        Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() | Print_Bill_CTRL_HD_SEND);
        Print_Bill_RXBITCTR_PERIOD_REG = Print_Bill_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Print_Bill_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Print_Bill_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Tx configuration if required and loads the
    *  Rx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Rx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART
    *
    * Side Effects:
    *  Set RX interrupt mask based on customizer settings, when software buffer
    *  has been used.
    *
    *******************************************************************************/
    void Print_Bill_LoadRxConfig(void) 
    {
        Print_Bill_WriteControlRegister(Print_Bill_ReadControlRegister() &
                                                (uint8)~Print_Bill_CTRL_HD_SEND);
        Print_Bill_RXBITCTR_PERIOD_REG = Print_Bill_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Print_Bill_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Print_Bill_RX_INTERRUPT_ENABLED) && (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Print_Bill_SetRxInterruptMode(Print_Bill_INIT_RX_INTERRUPTS_MASK);
        #endif /* Print_Bill_RX_INTERRUPT_ENABLED */
    }

#endif  /* Print_Bill_HD_ENABLED */


/* [] END OF FILE */
