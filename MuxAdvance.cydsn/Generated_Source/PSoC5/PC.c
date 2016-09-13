/*******************************************************************************
* File Name: PC.c
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

#include "PC.h"
#include "CyLib.h"
#if(PC_INTERNAL_CLOCK_USED)
    #include "PC_IntClock.h"
#endif /* End PC_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 PC_initVar = 0u;
#if( PC_TX_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH))
    volatile uint8 PC_txBuffer[PC_TXBUFFERSIZE];
    volatile uint8 PC_txBufferRead = 0u;
    uint8 PC_txBufferWrite = 0u;
#endif /* End PC_TX_ENABLED */
#if( ( PC_RX_ENABLED || PC_HD_ENABLED ) && \
     (PC_RXBUFFERSIZE > PC_FIFO_LENGTH) )
    volatile uint8 PC_rxBuffer[PC_RXBUFFERSIZE];
    volatile uint16 PC_rxBufferRead = 0u;
    volatile uint16 PC_rxBufferWrite = 0u;
    volatile uint8 PC_rxBufferLoopDetect = 0u;
    volatile uint8 PC_rxBufferOverflow = 0u;
    #if (PC_RXHW_ADDRESS_ENABLED)
        volatile uint8 PC_rxAddressMode = PC_RXADDRESSMODE;
        volatile uint8 PC_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End PC_RX_ENABLED */


/*******************************************************************************
* Function Name: PC_Start
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
*  The PC_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the PC_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void PC_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(PC_initVar == 0u)
    {
        PC_Init();
        PC_initVar = 1u;
    }
    PC_Enable();
}


/*******************************************************************************
* Function Name: PC_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  PC_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void PC_Init(void) 
{
    #if(PC_RX_ENABLED || PC_HD_ENABLED)

        #if(PC_RX_INTERRUPT_ENABLED && (PC_RXBUFFERSIZE > PC_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(PC_RX_VECT_NUM, &PC_RXISR);
            CyIntSetPriority(PC_RX_VECT_NUM, PC_RX_PRIOR_NUM);
        #endif /* End PC_RX_INTERRUPT_ENABLED */

        #if (PC_RXHW_ADDRESS_ENABLED)
            PC_SetRxAddressMode(PC_RXAddressMode);
            PC_SetRxAddress1(PC_RXHWADDRESS1);
            PC_SetRxAddress2(PC_RXHWADDRESS2);
        #endif /* End PC_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        PC_RXBITCTR_PERIOD_REG = PC_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        PC_RXSTATUS_MASK_REG  = PC_INIT_RX_INTERRUPTS_MASK;
    #endif /* End PC_RX_ENABLED || PC_HD_ENABLED*/

    #if(PC_TX_ENABLED)
        #if(PC_TX_INTERRUPT_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(PC_TX_VECT_NUM, &PC_TXISR);
            CyIntSetPriority(PC_TX_VECT_NUM, PC_TX_PRIOR_NUM);
        #endif /* End PC_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(PC_TXCLKGEN_DP)
            PC_TXBITCLKGEN_CTR_REG = PC_BIT_CENTER;
            PC_TXBITCLKTX_COMPLETE_REG = (PC_NUMBER_OF_DATA_BITS +
                        PC_NUMBER_OF_START_BIT) * PC_OVER_SAMPLE_COUNT;
        #else
            PC_TXBITCTR_PERIOD_REG = ((PC_NUMBER_OF_DATA_BITS +
                        PC_NUMBER_OF_START_BIT) * PC_OVER_SAMPLE_8) - 1u;
        #endif /* End PC_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(PC_TX_INTERRUPT_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH))
            PC_TXSTATUS_MASK_REG = PC_TX_STS_FIFO_EMPTY;
        #else
            PC_TXSTATUS_MASK_REG = PC_INIT_TX_INTERRUPTS_MASK;
        #endif /*End PC_TX_INTERRUPT_ENABLED*/

    #endif /* End PC_TX_ENABLED */

    #if(PC_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        PC_WriteControlRegister( \
            (PC_ReadControlRegister() & (uint8)~PC_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(PC_PARITY_TYPE << PC_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End PC_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: PC_Enable
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
*  PC_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void PC_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(PC_RX_ENABLED || PC_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        PC_RXBITCTR_CONTROL_REG |= PC_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        PC_RXSTATUS_ACTL_REG  |= PC_INT_ENABLE;
        #if(PC_RX_INTERRUPT_ENABLED && (PC_RXBUFFERSIZE > PC_FIFO_LENGTH))
            CyIntEnable(PC_RX_VECT_NUM);
            #if (PC_RXHW_ADDRESS_ENABLED)
                PC_rxAddressDetected = 0u;
            #endif /* End PC_RXHW_ADDRESS_ENABLED */
        #endif /* End PC_RX_INTERRUPT_ENABLED */
    #endif /* End PC_RX_ENABLED || PC_HD_ENABLED*/

    #if(PC_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!PC_TXCLKGEN_DP)
            PC_TXBITCTR_CONTROL_REG |= PC_CNTR_ENABLE;
        #endif /* End PC_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        PC_TXSTATUS_ACTL_REG |= PC_INT_ENABLE;
        #if(PC_TX_INTERRUPT_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH))
            CyIntEnable(PC_TX_VECT_NUM);
        #endif /* End PC_TX_INTERRUPT_ENABLED*/
     #endif /* End PC_TX_ENABLED */

    #if(PC_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        PC_IntClock_Start();
    #endif /* End PC_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PC_Stop
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
void PC_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(PC_RX_ENABLED || PC_HD_ENABLED)
        PC_RXBITCTR_CONTROL_REG &= (uint8)~PC_CNTR_ENABLE;
    #endif /* End PC_RX_ENABLED */

    #if(PC_TX_ENABLED)
        #if(!PC_TXCLKGEN_DP)
            PC_TXBITCTR_CONTROL_REG &= (uint8)~PC_CNTR_ENABLE;
        #endif /* End PC_TXCLKGEN_DP */
    #endif /* PC_TX_ENABLED */

    #if(PC_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        PC_IntClock_Stop();
    #endif /* End PC_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(PC_RX_ENABLED || PC_HD_ENABLED)
        PC_RXSTATUS_ACTL_REG  &= (uint8)~PC_INT_ENABLE;
        #if(PC_RX_INTERRUPT_ENABLED && (PC_RXBUFFERSIZE > PC_FIFO_LENGTH))
            PC_DisableRxInt();
        #endif /* End PC_RX_INTERRUPT_ENABLED */
    #endif /* End PC_RX_ENABLED */

    #if(PC_TX_ENABLED)
        PC_TXSTATUS_ACTL_REG &= (uint8)~PC_INT_ENABLE;
        #if(PC_TX_INTERRUPT_ENABLED && (PC_TXBUFFERSIZE > PC_FIFO_LENGTH))
            PC_DisableTxInt();
        #endif /* End PC_TX_INTERRUPT_ENABLED */
    #endif /* End PC_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: PC_ReadControlRegister
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
uint8 PC_ReadControlRegister(void) 
{
    #if( PC_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(PC_CONTROL_REG);
    #endif /* End PC_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: PC_WriteControlRegister
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
void  PC_WriteControlRegister(uint8 control) 
{
    #if( PC_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       PC_CONTROL_REG = control;
    #endif /* End PC_CONTROL_REG_REMOVED */
}


#if(PC_RX_ENABLED || PC_HD_ENABLED)

    #if(PC_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: PC_EnableRxInt
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
        void PC_EnableRxInt(void) 
        {
            CyIntEnable(PC_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: PC_DisableRxInt
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
        void PC_DisableRxInt(void) 
        {
            CyIntDisable(PC_RX_VECT_NUM);
        }

    #endif /* PC_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: PC_SetRxInterruptMode
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
    void PC_SetRxInterruptMode(uint8 intSrc) 
    {
        PC_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: PC_ReadRxData
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
    *  PC_rxBuffer - RAM buffer pointer for save received data.
    *  PC_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  PC_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  PC_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 PC_ReadRxData(void) 
    {
        uint8 rxData;

        #if(PC_RXBUFFERSIZE > PC_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_DisableRxInt();
            #endif /* PC_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = PC_rxBufferRead;
            loc_rxBufferWrite = PC_rxBufferWrite;

            if( (PC_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = PC_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= PC_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                PC_rxBufferRead = loc_rxBufferRead;

                if(PC_rxBufferLoopDetect != 0u )
                {
                    PC_rxBufferLoopDetect = 0u;
                    #if( (PC_RX_INTERRUPT_ENABLED) && (PC_FLOW_CONTROL != 0u) && \
                         (PC_RXBUFFERSIZE > PC_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( PC_HD_ENABLED )
                            if((PC_CONTROL_REG & PC_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                PC_RXSTATUS_MASK_REG  |= PC_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            PC_RXSTATUS_MASK_REG  |= PC_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end PC_HD_ENABLED */
                    #endif /* PC_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = PC_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_EnableRxInt();
            #endif /* End PC_RX_INTERRUPT_ENABLED */

        #else /* PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = PC_RXDATA_REG;

        #endif /* PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PC_ReadRxStatus
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
    *  PC_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   PC_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   PC_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 PC_ReadRxStatus(void) 
    {
        uint8 status;

        status = PC_RXSTATUS_REG & PC_RX_HW_MASK;

        #if(PC_RXBUFFERSIZE > PC_FIFO_LENGTH)
            if( PC_rxBufferOverflow != 0u )
            {
                status |= PC_RX_STS_SOFT_BUFF_OVER;
                PC_rxBufferOverflow = 0u;
            }
        #endif /* PC_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: PC_GetChar
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
    *  PC_rxBuffer - RAM buffer pointer for save received data.
    *  PC_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  PC_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  PC_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 PC_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(PC_RXBUFFERSIZE > PC_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_DisableRxInt();
            #endif /* PC_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = PC_rxBufferRead;
            loc_rxBufferWrite = PC_rxBufferWrite;

            if( (PC_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = PC_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= PC_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                PC_rxBufferRead = loc_rxBufferRead;

                if(PC_rxBufferLoopDetect > 0u )
                {
                    PC_rxBufferLoopDetect = 0u;
                    #if( (PC_RX_INTERRUPT_ENABLED) && (PC_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( PC_HD_ENABLED )
                            if((PC_CONTROL_REG & PC_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                PC_RXSTATUS_MASK_REG  |= PC_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            PC_RXSTATUS_MASK_REG  |= PC_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end PC_HD_ENABLED */
                    #endif /* PC_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = PC_RXSTATUS_REG;
                if((rxStatus & PC_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = PC_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (PC_RX_STS_BREAK | PC_RX_STS_PAR_ERROR |
                                   PC_RX_STS_STOP_ERROR | PC_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_EnableRxInt();
            #endif /* PC_RX_INTERRUPT_ENABLED */

        #else /* PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

            rxStatus =PC_RXSTATUS_REG;
            if((rxStatus & PC_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = PC_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (PC_RX_STS_BREAK | PC_RX_STS_PAR_ERROR |
                               PC_RX_STS_STOP_ERROR | PC_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: PC_GetByte
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
    uint16 PC_GetByte(void) 
    {
        return ( ((uint16)PC_ReadRxStatus() << 8u) | PC_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: PC_GetRxBufferSize
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
    *  uint16: Integer count of the number of bytes left
    *  in the RX buffer
    *
    * Global Variables:
    *  PC_rxBufferWrite - used to calculate left bytes.
    *  PC_rxBufferRead - used to calculate left bytes.
    *  PC_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 PC_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(PC_RXBUFFERSIZE > PC_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_DisableRxInt();
            #endif /* PC_RX_INTERRUPT_ENABLED */

            if(PC_rxBufferRead == PC_rxBufferWrite)
            {
                if(PC_rxBufferLoopDetect > 0u)
                {
                    size = PC_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(PC_rxBufferRead < PC_rxBufferWrite)
            {
                size = (PC_rxBufferWrite - PC_rxBufferRead);
            }
            else
            {
                size = (PC_RXBUFFERSIZE - PC_rxBufferRead) + PC_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_EnableRxInt();
            #endif /* End PC_RX_INTERRUPT_ENABLED */

        #else /* PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((PC_RXSTATUS_REG & PC_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: PC_ClearRxBuffer
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
    *  PC_rxBufferWrite - cleared to zero.
    *  PC_rxBufferRead - cleared to zero.
    *  PC_rxBufferLoopDetect - cleared to zero.
    *  PC_rxBufferOverflow - cleared to zero.
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
    void PC_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        PC_RXDATA_AUX_CTL_REG |=  PC_RX_FIFO_CLR;
        PC_RXDATA_AUX_CTL_REG &= (uint8)~PC_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(PC_RXBUFFERSIZE > PC_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_DisableRxInt();
            #endif /* End PC_RX_INTERRUPT_ENABLED */

            PC_rxBufferRead = 0u;
            PC_rxBufferWrite = 0u;
            PC_rxBufferLoopDetect = 0u;
            PC_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(PC_RX_INTERRUPT_ENABLED)
                PC_EnableRxInt();
            #endif /* End PC_RX_INTERRUPT_ENABLED */
        #endif /* End PC_RXBUFFERSIZE > PC_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: PC_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  PC__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  PC__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  PC__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  PC__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  PC__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  PC_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  PC_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void PC_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(PC_RXHW_ADDRESS_ENABLED)
            #if(PC_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* PC_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = PC_CONTROL_REG & (uint8)~PC_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << PC_CTRL_RXADDR_MODE0_SHIFT);
                PC_CONTROL_REG = tmpCtrl;
                #if(PC_RX_INTERRUPT_ENABLED && \
                   (PC_RXBUFFERSIZE > PC_FIFO_LENGTH) )
                    PC_rxAddressMode = addressMode;
                    PC_rxAddressDetected = 0u;
                #endif /* End PC_RXBUFFERSIZE > PC_FIFO_LENGTH*/
            #endif /* End PC_CONTROL_REG_REMOVED */
        #else /* PC_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End PC_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: PC_SetRxAddress1
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
    void PC_SetRxAddress1(uint8 address) 

    {
        PC_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: PC_SetRxAddress2
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
    void PC_SetRxAddress2(uint8 address) 
    {
        PC_RXADDRESS2_REG = address;
    }

#endif  /* PC_RX_ENABLED || PC_HD_ENABLED*/


#if( (PC_TX_ENABLED) || (PC_HD_ENABLED) )

    #if(PC_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: PC_EnableTxInt
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
        void PC_EnableTxInt(void) 
        {
            CyIntEnable(PC_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: PC_DisableTxInt
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
        void PC_DisableTxInt(void) 
        {
            CyIntDisable(PC_TX_VECT_NUM);
        }

    #endif /* PC_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: PC_SetTxInterruptMode
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
    void PC_SetTxInterruptMode(uint8 intSrc) 
    {
        PC_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: PC_WriteTxData
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
    *  PC_txBuffer - RAM buffer pointer for save data for transmission
    *  PC_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  PC_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  PC_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void PC_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(PC_initVar != 0u)
        {
            #if(PC_TXBUFFERSIZE > PC_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(PC_TX_INTERRUPT_ENABLED)
                    PC_DisableTxInt();
                #endif /* End PC_TX_INTERRUPT_ENABLED */

                if( (PC_txBufferRead == PC_txBufferWrite) &&
                    ((PC_TXSTATUS_REG & PC_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    PC_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(PC_txBufferWrite >= PC_TXBUFFERSIZE)
                    {
                        PC_txBufferWrite = 0u;
                    }

                    PC_txBuffer[PC_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    PC_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(PC_TX_INTERRUPT_ENABLED)
                    PC_EnableTxInt();
                #endif /* End PC_TX_INTERRUPT_ENABLED */

            #else /* PC_TXBUFFERSIZE > PC_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                PC_TXDATA_REG = txDataByte;

            #endif /* End PC_TXBUFFERSIZE > PC_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: PC_ReadTxStatus
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
    uint8 PC_ReadTxStatus(void) 
    {
        return(PC_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: PC_PutChar
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
    *  PC_txBuffer - RAM buffer pointer for save data for transmission
    *  PC_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  PC_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  PC_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void PC_PutChar(uint8 txDataByte) 
    {
            #if(PC_TXBUFFERSIZE > PC_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((PC_TXBUFFERSIZE > PC_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(PC_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = PC_txBufferWrite;
                    loc_txBufferRead = PC_txBufferRead;
                    #if ((PC_TXBUFFERSIZE > PC_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(PC_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(PC_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((PC_TXSTATUS_REG & PC_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    PC_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= PC_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    PC_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((PC_TXBUFFERSIZE > PC_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(PC_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    PC_txBufferWrite = loc_txBufferWrite;
                    #if ((PC_TXBUFFERSIZE > PC_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(PC_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* PC_TXBUFFERSIZE > PC_FIFO_LENGTH */

                while((PC_TXSTATUS_REG & PC_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                PC_TXDATA_REG = txDataByte;

            #endif /* End PC_TXBUFFERSIZE > PC_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: PC_PutString
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
    *  PC_initVar - checked to identify that the component has been
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
    void PC_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(PC_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                PC_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: PC_PutArray
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
    *  PC_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void PC_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(PC_initVar != 0u)
        {
            do
            {
                PC_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: PC_PutCRLF
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
    *  PC_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void PC_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(PC_initVar != 0u)
        {
            PC_PutChar(txDataByte);
            PC_PutChar(0x0Du);
            PC_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: PC_GetTxBufferSize
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
    *  PC_txBufferWrite - used to calculate left space.
    *  PC_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 PC_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(PC_TXBUFFERSIZE > PC_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PC_TX_INTERRUPT_ENABLED)
                PC_DisableTxInt();
            #endif /* End PC_TX_INTERRUPT_ENABLED */

            if(PC_txBufferRead == PC_txBufferWrite)
            {
                size = 0u;
            }
            else if(PC_txBufferRead < PC_txBufferWrite)
            {
                size = (PC_txBufferWrite - PC_txBufferRead);
            }
            else
            {
                size = (PC_TXBUFFERSIZE - PC_txBufferRead) + PC_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(PC_TX_INTERRUPT_ENABLED)
                PC_EnableTxInt();
            #endif /* End PC_TX_INTERRUPT_ENABLED */

        #else /* PC_TXBUFFERSIZE > PC_FIFO_LENGTH */

            size = PC_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & PC_TX_STS_FIFO_FULL) != 0u)
            {
                size = PC_FIFO_LENGTH;
            }
            else if((size & PC_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End PC_TXBUFFERSIZE > PC_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: PC_ClearTxBuffer
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
    *  PC_txBufferWrite - cleared to zero.
    *  PC_txBufferRead - cleared to zero.
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
    void PC_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        PC_TXDATA_AUX_CTL_REG |=  PC_TX_FIFO_CLR;
        PC_TXDATA_AUX_CTL_REG &= (uint8)~PC_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(PC_TXBUFFERSIZE > PC_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(PC_TX_INTERRUPT_ENABLED)
                PC_DisableTxInt();
            #endif /* End PC_TX_INTERRUPT_ENABLED */

            PC_txBufferRead = 0u;
            PC_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(PC_TX_INTERRUPT_ENABLED)
                PC_EnableTxInt();
            #endif /* End PC_TX_INTERRUPT_ENABLED */

        #endif /* End PC_TXBUFFERSIZE > PC_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: PC_SendBreak
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
    *  PC_initVar - checked to identify that the component has been
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
    void PC_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(PC_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(PC_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == PC_SEND_BREAK) ||
                    (retMode == PC_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    PC_WriteControlRegister(PC_ReadControlRegister() |
                                                          PC_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    PC_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = PC_TXSTATUS_REG;
                    }while((tmpStat & PC_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == PC_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PC_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = PC_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & PC_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == PC_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PC_REINIT) ||
                    (retMode == PC_SEND_WAIT_REINIT) )
                {
                    PC_WriteControlRegister(PC_ReadControlRegister() &
                                                  (uint8)~PC_CTRL_HD_SEND_BREAK);
                }

            #else /* PC_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == PC_SEND_BREAK) ||
                    (retMode == PC_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (PC_PARITY_TYPE != PC__B_UART__NONE_REVB) || \
                                        (PC_PARITY_TYPE_SW != 0u) )
                        PC_WriteControlRegister(PC_ReadControlRegister() |
                                                              PC_CTRL_HD_SEND_BREAK);
                    #endif /* End PC_PARITY_TYPE != PC__B_UART__NONE_REVB  */

                    #if(PC_TXCLKGEN_DP)
                        tx_period = PC_TXBITCLKTX_COMPLETE_REG;
                        PC_TXBITCLKTX_COMPLETE_REG = PC_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = PC_TXBITCTR_PERIOD_REG;
                        PC_TXBITCTR_PERIOD_REG = PC_TXBITCTR_BREAKBITS8X;
                    #endif /* End PC_TXCLKGEN_DP */

                    /* Send zeros*/
                    PC_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = PC_TXSTATUS_REG;
                    }while((tmpStat & PC_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == PC_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PC_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = PC_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & PC_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == PC_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == PC_REINIT) ||
                    (retMode == PC_SEND_WAIT_REINIT) )
                {

                    #if(PC_TXCLKGEN_DP)
                        PC_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        PC_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End PC_TXCLKGEN_DP */

                    #if( (PC_PARITY_TYPE != PC__B_UART__NONE_REVB) || \
                         (PC_PARITY_TYPE_SW != 0u) )
                        PC_WriteControlRegister(PC_ReadControlRegister() &
                                                      (uint8)~PC_CTRL_HD_SEND_BREAK);
                    #endif /* End PC_PARITY_TYPE != NONE */
                }
            #endif    /* End PC_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: PC_SetTxAddressMode
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
    void PC_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( PC_CONTROL_REG_REMOVED == 0u )
                PC_WriteControlRegister(PC_ReadControlRegister() |
                                                      PC_CTRL_MARK);
            #endif /* End PC_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( PC_CONTROL_REG_REMOVED == 0u )
                PC_WriteControlRegister(PC_ReadControlRegister() &
                                                    (uint8)~PC_CTRL_MARK);
            #endif /* End PC_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndPC_TX_ENABLED */

#if(PC_HD_ENABLED)


    /*******************************************************************************
    * Function Name: PC_LoadTxConfig
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
    void PC_LoadTxConfig(void) 
    {
        #if((PC_RX_INTERRUPT_ENABLED) && (PC_RXBUFFERSIZE > PC_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            PC_SetRxInterruptMode(0u);
        #endif /* PC_RX_INTERRUPT_ENABLED */

        PC_WriteControlRegister(PC_ReadControlRegister() | PC_CTRL_HD_SEND);
        PC_RXBITCTR_PERIOD_REG = PC_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(PC_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: PC_LoadRxConfig
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
    void PC_LoadRxConfig(void) 
    {
        PC_WriteControlRegister(PC_ReadControlRegister() &
                                                (uint8)~PC_CTRL_HD_SEND);
        PC_RXBITCTR_PERIOD_REG = PC_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(PC_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((PC_RX_INTERRUPT_ENABLED) && (PC_RXBUFFERSIZE > PC_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            PC_SetRxInterruptMode(PC_INIT_RX_INTERRUPTS_MASK);
        #endif /* PC_RX_INTERRUPT_ENABLED */
    }

#endif  /* PC_HD_ENABLED */


/* [] END OF FILE */
