/*******************************************************************************
* File Name: Tag.c
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

#include "Tag.h"
#include "CyLib.h"
#if(Tag_INTERNAL_CLOCK_USED)
    #include "Tag_IntClock.h"
#endif /* End Tag_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Tag_initVar = 0u;
#if( Tag_TX_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
    volatile uint8 Tag_txBuffer[Tag_TXBUFFERSIZE];
    volatile uint8 Tag_txBufferRead = 0u;
    uint8 Tag_txBufferWrite = 0u;
#endif /* End Tag_TX_ENABLED */
#if( ( Tag_RX_ENABLED || Tag_HD_ENABLED ) && \
     (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH) )
    volatile uint8 Tag_rxBuffer[Tag_RXBUFFERSIZE];
    volatile uint16 Tag_rxBufferRead = 0u;
    volatile uint16 Tag_rxBufferWrite = 0u;
    volatile uint8 Tag_rxBufferLoopDetect = 0u;
    volatile uint8 Tag_rxBufferOverflow = 0u;
    #if (Tag_RXHW_ADDRESS_ENABLED)
        volatile uint8 Tag_rxAddressMode = Tag_RXADDRESSMODE;
        volatile uint8 Tag_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Tag_RX_ENABLED */


/*******************************************************************************
* Function Name: Tag_Start
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
*  The Tag_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Tag_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Tag_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Tag_initVar == 0u)
    {
        Tag_Init();
        Tag_initVar = 1u;
    }
    Tag_Enable();
}


/*******************************************************************************
* Function Name: Tag_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Tag_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Tag_Init(void) 
{
    #if(Tag_RX_ENABLED || Tag_HD_ENABLED)

        #if(Tag_RX_INTERRUPT_ENABLED && (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Tag_RX_VECT_NUM, &Tag_RXISR);
            CyIntSetPriority(Tag_RX_VECT_NUM, Tag_RX_PRIOR_NUM);
        #endif /* End Tag_RX_INTERRUPT_ENABLED */

        #if (Tag_RXHW_ADDRESS_ENABLED)
            Tag_SetRxAddressMode(Tag_RXAddressMode);
            Tag_SetRxAddress1(Tag_RXHWADDRESS1);
            Tag_SetRxAddress2(Tag_RXHWADDRESS2);
        #endif /* End Tag_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Tag_RXBITCTR_PERIOD_REG = Tag_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Tag_RXSTATUS_MASK_REG  = Tag_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Tag_RX_ENABLED || Tag_HD_ENABLED*/

    #if(Tag_TX_ENABLED)
        #if(Tag_TX_INTERRUPT_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Tag_TX_VECT_NUM, &Tag_TXISR);
            CyIntSetPriority(Tag_TX_VECT_NUM, Tag_TX_PRIOR_NUM);
        #endif /* End Tag_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Tag_TXCLKGEN_DP)
            Tag_TXBITCLKGEN_CTR_REG = Tag_BIT_CENTER;
            Tag_TXBITCLKTX_COMPLETE_REG = (Tag_NUMBER_OF_DATA_BITS +
                        Tag_NUMBER_OF_START_BIT) * Tag_OVER_SAMPLE_COUNT;
        #else
            Tag_TXBITCTR_PERIOD_REG = ((Tag_NUMBER_OF_DATA_BITS +
                        Tag_NUMBER_OF_START_BIT) * Tag_OVER_SAMPLE_8) - 1u;
        #endif /* End Tag_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Tag_TX_INTERRUPT_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
            Tag_TXSTATUS_MASK_REG = Tag_TX_STS_FIFO_EMPTY;
        #else
            Tag_TXSTATUS_MASK_REG = Tag_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Tag_TX_INTERRUPT_ENABLED*/

    #endif /* End Tag_TX_ENABLED */

    #if(Tag_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Tag_WriteControlRegister( \
            (Tag_ReadControlRegister() & (uint8)~Tag_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Tag_PARITY_TYPE << Tag_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Tag_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Tag_Enable
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
*  Tag_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Tag_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Tag_RX_ENABLED || Tag_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Tag_RXBITCTR_CONTROL_REG |= Tag_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Tag_RXSTATUS_ACTL_REG  |= Tag_INT_ENABLE;
        #if(Tag_RX_INTERRUPT_ENABLED && (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH))
            CyIntEnable(Tag_RX_VECT_NUM);
            #if (Tag_RXHW_ADDRESS_ENABLED)
                Tag_rxAddressDetected = 0u;
            #endif /* End Tag_RXHW_ADDRESS_ENABLED */
        #endif /* End Tag_RX_INTERRUPT_ENABLED */
    #endif /* End Tag_RX_ENABLED || Tag_HD_ENABLED*/

    #if(Tag_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Tag_TXCLKGEN_DP)
            Tag_TXBITCTR_CONTROL_REG |= Tag_CNTR_ENABLE;
        #endif /* End Tag_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Tag_TXSTATUS_ACTL_REG |= Tag_INT_ENABLE;
        #if(Tag_TX_INTERRUPT_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
            CyIntEnable(Tag_TX_VECT_NUM);
        #endif /* End Tag_TX_INTERRUPT_ENABLED*/
     #endif /* End Tag_TX_ENABLED */

    #if(Tag_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Tag_IntClock_Start();
    #endif /* End Tag_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Tag_Stop
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
void Tag_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Tag_RX_ENABLED || Tag_HD_ENABLED)
        Tag_RXBITCTR_CONTROL_REG &= (uint8)~Tag_CNTR_ENABLE;
    #endif /* End Tag_RX_ENABLED */

    #if(Tag_TX_ENABLED)
        #if(!Tag_TXCLKGEN_DP)
            Tag_TXBITCTR_CONTROL_REG &= (uint8)~Tag_CNTR_ENABLE;
        #endif /* End Tag_TXCLKGEN_DP */
    #endif /* Tag_TX_ENABLED */

    #if(Tag_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Tag_IntClock_Stop();
    #endif /* End Tag_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Tag_RX_ENABLED || Tag_HD_ENABLED)
        Tag_RXSTATUS_ACTL_REG  &= (uint8)~Tag_INT_ENABLE;
        #if(Tag_RX_INTERRUPT_ENABLED && (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH))
            Tag_DisableRxInt();
        #endif /* End Tag_RX_INTERRUPT_ENABLED */
    #endif /* End Tag_RX_ENABLED */

    #if(Tag_TX_ENABLED)
        Tag_TXSTATUS_ACTL_REG &= (uint8)~Tag_INT_ENABLE;
        #if(Tag_TX_INTERRUPT_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
            Tag_DisableTxInt();
        #endif /* End Tag_TX_INTERRUPT_ENABLED */
    #endif /* End Tag_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Tag_ReadControlRegister
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
uint8 Tag_ReadControlRegister(void) 
{
    #if( Tag_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Tag_CONTROL_REG);
    #endif /* End Tag_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Tag_WriteControlRegister
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
void  Tag_WriteControlRegister(uint8 control) 
{
    #if( Tag_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Tag_CONTROL_REG = control;
    #endif /* End Tag_CONTROL_REG_REMOVED */
}


#if(Tag_RX_ENABLED || Tag_HD_ENABLED)

    #if(Tag_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Tag_EnableRxInt
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
        void Tag_EnableRxInt(void) 
        {
            CyIntEnable(Tag_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Tag_DisableRxInt
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
        void Tag_DisableRxInt(void) 
        {
            CyIntDisable(Tag_RX_VECT_NUM);
        }

    #endif /* Tag_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Tag_SetRxInterruptMode
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
    void Tag_SetRxInterruptMode(uint8 intSrc) 
    {
        Tag_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Tag_ReadRxData
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
    *  Tag_rxBuffer - RAM buffer pointer for save received data.
    *  Tag_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Tag_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Tag_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Tag_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_DisableRxInt();
            #endif /* Tag_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Tag_rxBufferRead;
            loc_rxBufferWrite = Tag_rxBufferWrite;

            if( (Tag_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Tag_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Tag_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Tag_rxBufferRead = loc_rxBufferRead;

                if(Tag_rxBufferLoopDetect != 0u )
                {
                    Tag_rxBufferLoopDetect = 0u;
                    #if( (Tag_RX_INTERRUPT_ENABLED) && (Tag_FLOW_CONTROL != 0u) && \
                         (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Tag_HD_ENABLED )
                            if((Tag_CONTROL_REG & Tag_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Tag_RXSTATUS_MASK_REG  |= Tag_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Tag_RXSTATUS_MASK_REG  |= Tag_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Tag_HD_ENABLED */
                    #endif /* Tag_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Tag_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_EnableRxInt();
            #endif /* End Tag_RX_INTERRUPT_ENABLED */

        #else /* Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Tag_RXDATA_REG;

        #endif /* Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Tag_ReadRxStatus
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
    *  Tag_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Tag_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Tag_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Tag_ReadRxStatus(void) 
    {
        uint8 status;

        status = Tag_RXSTATUS_REG & Tag_RX_HW_MASK;

        #if(Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH)
            if( Tag_rxBufferOverflow != 0u )
            {
                status |= Tag_RX_STS_SOFT_BUFF_OVER;
                Tag_rxBufferOverflow = 0u;
            }
        #endif /* Tag_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Tag_GetChar
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
    *  Tag_rxBuffer - RAM buffer pointer for save received data.
    *  Tag_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Tag_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Tag_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Tag_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_DisableRxInt();
            #endif /* Tag_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Tag_rxBufferRead;
            loc_rxBufferWrite = Tag_rxBufferWrite;

            if( (Tag_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Tag_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Tag_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Tag_rxBufferRead = loc_rxBufferRead;

                if(Tag_rxBufferLoopDetect > 0u )
                {
                    Tag_rxBufferLoopDetect = 0u;
                    #if( (Tag_RX_INTERRUPT_ENABLED) && (Tag_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Tag_HD_ENABLED )
                            if((Tag_CONTROL_REG & Tag_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Tag_RXSTATUS_MASK_REG  |= Tag_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Tag_RXSTATUS_MASK_REG  |= Tag_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Tag_HD_ENABLED */
                    #endif /* Tag_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Tag_RXSTATUS_REG;
                if((rxStatus & Tag_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Tag_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Tag_RX_STS_BREAK | Tag_RX_STS_PAR_ERROR |
                                   Tag_RX_STS_STOP_ERROR | Tag_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_EnableRxInt();
            #endif /* Tag_RX_INTERRUPT_ENABLED */

        #else /* Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

            rxStatus =Tag_RXSTATUS_REG;
            if((rxStatus & Tag_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Tag_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Tag_RX_STS_BREAK | Tag_RX_STS_PAR_ERROR |
                               Tag_RX_STS_STOP_ERROR | Tag_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Tag_GetByte
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
    uint16 Tag_GetByte(void) 
    {
        return ( ((uint16)Tag_ReadRxStatus() << 8u) | Tag_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Tag_GetRxBufferSize
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
    *  Tag_rxBufferWrite - used to calculate left bytes.
    *  Tag_rxBufferRead - used to calculate left bytes.
    *  Tag_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 Tag_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_DisableRxInt();
            #endif /* Tag_RX_INTERRUPT_ENABLED */

            if(Tag_rxBufferRead == Tag_rxBufferWrite)
            {
                if(Tag_rxBufferLoopDetect > 0u)
                {
                    size = Tag_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Tag_rxBufferRead < Tag_rxBufferWrite)
            {
                size = (Tag_rxBufferWrite - Tag_rxBufferRead);
            }
            else
            {
                size = (Tag_RXBUFFERSIZE - Tag_rxBufferRead) + Tag_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_EnableRxInt();
            #endif /* End Tag_RX_INTERRUPT_ENABLED */

        #else /* Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Tag_RXSTATUS_REG & Tag_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Tag_ClearRxBuffer
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
    *  Tag_rxBufferWrite - cleared to zero.
    *  Tag_rxBufferRead - cleared to zero.
    *  Tag_rxBufferLoopDetect - cleared to zero.
    *  Tag_rxBufferOverflow - cleared to zero.
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
    void Tag_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Tag_RXDATA_AUX_CTL_REG |=  Tag_RX_FIFO_CLR;
        Tag_RXDATA_AUX_CTL_REG &= (uint8)~Tag_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_DisableRxInt();
            #endif /* End Tag_RX_INTERRUPT_ENABLED */

            Tag_rxBufferRead = 0u;
            Tag_rxBufferWrite = 0u;
            Tag_rxBufferLoopDetect = 0u;
            Tag_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Tag_RX_INTERRUPT_ENABLED)
                Tag_EnableRxInt();
            #endif /* End Tag_RX_INTERRUPT_ENABLED */
        #endif /* End Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Tag_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Tag__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Tag__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Tag__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Tag__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Tag__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Tag_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Tag_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Tag_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Tag_RXHW_ADDRESS_ENABLED)
            #if(Tag_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Tag_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Tag_CONTROL_REG & (uint8)~Tag_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Tag_CTRL_RXADDR_MODE0_SHIFT);
                Tag_CONTROL_REG = tmpCtrl;
                #if(Tag_RX_INTERRUPT_ENABLED && \
                   (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH) )
                    Tag_rxAddressMode = addressMode;
                    Tag_rxAddressDetected = 0u;
                #endif /* End Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH*/
            #endif /* End Tag_CONTROL_REG_REMOVED */
        #else /* Tag_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Tag_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Tag_SetRxAddress1
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
    void Tag_SetRxAddress1(uint8 address) 

    {
        Tag_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Tag_SetRxAddress2
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
    void Tag_SetRxAddress2(uint8 address) 
    {
        Tag_RXADDRESS2_REG = address;
    }

#endif  /* Tag_RX_ENABLED || Tag_HD_ENABLED*/


#if( (Tag_TX_ENABLED) || (Tag_HD_ENABLED) )

    #if(Tag_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Tag_EnableTxInt
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
        void Tag_EnableTxInt(void) 
        {
            CyIntEnable(Tag_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Tag_DisableTxInt
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
        void Tag_DisableTxInt(void) 
        {
            CyIntDisable(Tag_TX_VECT_NUM);
        }

    #endif /* Tag_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Tag_SetTxInterruptMode
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
    void Tag_SetTxInterruptMode(uint8 intSrc) 
    {
        Tag_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Tag_WriteTxData
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
    *  Tag_txBuffer - RAM buffer pointer for save data for transmission
    *  Tag_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Tag_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Tag_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Tag_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Tag_initVar != 0u)
        {
            #if(Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Tag_TX_INTERRUPT_ENABLED)
                    Tag_DisableTxInt();
                #endif /* End Tag_TX_INTERRUPT_ENABLED */

                if( (Tag_txBufferRead == Tag_txBufferWrite) &&
                    ((Tag_TXSTATUS_REG & Tag_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Tag_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Tag_txBufferWrite >= Tag_TXBUFFERSIZE)
                    {
                        Tag_txBufferWrite = 0u;
                    }

                    Tag_txBuffer[Tag_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Tag_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Tag_TX_INTERRUPT_ENABLED)
                    Tag_EnableTxInt();
                #endif /* End Tag_TX_INTERRUPT_ENABLED */

            #else /* Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Tag_TXDATA_REG = txDataByte;

            #endif /* End Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Tag_ReadTxStatus
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
    uint8 Tag_ReadTxStatus(void) 
    {
        return(Tag_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Tag_PutChar
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
    *  Tag_txBuffer - RAM buffer pointer for save data for transmission
    *  Tag_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Tag_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Tag_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Tag_PutChar(uint8 txDataByte) 
    {
            #if(Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Tag_TXBUFFERSIZE > Tag_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Tag_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Tag_txBufferWrite;
                    loc_txBufferRead = Tag_txBufferRead;
                    #if ((Tag_TXBUFFERSIZE > Tag_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Tag_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Tag_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Tag_TXSTATUS_REG & Tag_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Tag_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Tag_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Tag_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Tag_TXBUFFERSIZE > Tag_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Tag_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Tag_txBufferWrite = loc_txBufferWrite;
                    #if ((Tag_TXBUFFERSIZE > Tag_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Tag_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */

                while((Tag_TXSTATUS_REG & Tag_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Tag_TXDATA_REG = txDataByte;

            #endif /* End Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Tag_PutString
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
    *  Tag_initVar - checked to identify that the component has been
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
    void Tag_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Tag_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Tag_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Tag_PutArray
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
    *  Tag_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Tag_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Tag_initVar != 0u)
        {
            do
            {
                Tag_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Tag_PutCRLF
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
    *  Tag_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Tag_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Tag_initVar != 0u)
        {
            Tag_PutChar(txDataByte);
            Tag_PutChar(0x0Du);
            Tag_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Tag_GetTxBufferSize
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
    *  Tag_txBufferWrite - used to calculate left space.
    *  Tag_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Tag_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Tag_TX_INTERRUPT_ENABLED)
                Tag_DisableTxInt();
            #endif /* End Tag_TX_INTERRUPT_ENABLED */

            if(Tag_txBufferRead == Tag_txBufferWrite)
            {
                size = 0u;
            }
            else if(Tag_txBufferRead < Tag_txBufferWrite)
            {
                size = (Tag_txBufferWrite - Tag_txBufferRead);
            }
            else
            {
                size = (Tag_TXBUFFERSIZE - Tag_txBufferRead) + Tag_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Tag_TX_INTERRUPT_ENABLED)
                Tag_EnableTxInt();
            #endif /* End Tag_TX_INTERRUPT_ENABLED */

        #else /* Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */

            size = Tag_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Tag_TX_STS_FIFO_FULL) != 0u)
            {
                size = Tag_FIFO_LENGTH;
            }
            else if((size & Tag_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Tag_ClearTxBuffer
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
    *  Tag_txBufferWrite - cleared to zero.
    *  Tag_txBufferRead - cleared to zero.
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
    void Tag_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Tag_TXDATA_AUX_CTL_REG |=  Tag_TX_FIFO_CLR;
        Tag_TXDATA_AUX_CTL_REG &= (uint8)~Tag_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Tag_TX_INTERRUPT_ENABLED)
                Tag_DisableTxInt();
            #endif /* End Tag_TX_INTERRUPT_ENABLED */

            Tag_txBufferRead = 0u;
            Tag_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Tag_TX_INTERRUPT_ENABLED)
                Tag_EnableTxInt();
            #endif /* End Tag_TX_INTERRUPT_ENABLED */

        #endif /* End Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Tag_SendBreak
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
    *  Tag_initVar - checked to identify that the component has been
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
    void Tag_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Tag_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Tag_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Tag_SEND_BREAK) ||
                    (retMode == Tag_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Tag_WriteControlRegister(Tag_ReadControlRegister() |
                                                          Tag_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Tag_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Tag_TXSTATUS_REG;
                    }while((tmpStat & Tag_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Tag_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Tag_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Tag_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Tag_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Tag_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Tag_REINIT) ||
                    (retMode == Tag_SEND_WAIT_REINIT) )
                {
                    Tag_WriteControlRegister(Tag_ReadControlRegister() &
                                                  (uint8)~Tag_CTRL_HD_SEND_BREAK);
                }

            #else /* Tag_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Tag_SEND_BREAK) ||
                    (retMode == Tag_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Tag_PARITY_TYPE != Tag__B_UART__NONE_REVB) || \
                                        (Tag_PARITY_TYPE_SW != 0u) )
                        Tag_WriteControlRegister(Tag_ReadControlRegister() |
                                                              Tag_CTRL_HD_SEND_BREAK);
                    #endif /* End Tag_PARITY_TYPE != Tag__B_UART__NONE_REVB  */

                    #if(Tag_TXCLKGEN_DP)
                        tx_period = Tag_TXBITCLKTX_COMPLETE_REG;
                        Tag_TXBITCLKTX_COMPLETE_REG = Tag_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Tag_TXBITCTR_PERIOD_REG;
                        Tag_TXBITCTR_PERIOD_REG = Tag_TXBITCTR_BREAKBITS8X;
                    #endif /* End Tag_TXCLKGEN_DP */

                    /* Send zeros*/
                    Tag_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Tag_TXSTATUS_REG;
                    }while((tmpStat & Tag_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Tag_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Tag_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Tag_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Tag_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Tag_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Tag_REINIT) ||
                    (retMode == Tag_SEND_WAIT_REINIT) )
                {

                    #if(Tag_TXCLKGEN_DP)
                        Tag_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Tag_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Tag_TXCLKGEN_DP */

                    #if( (Tag_PARITY_TYPE != Tag__B_UART__NONE_REVB) || \
                         (Tag_PARITY_TYPE_SW != 0u) )
                        Tag_WriteControlRegister(Tag_ReadControlRegister() &
                                                      (uint8)~Tag_CTRL_HD_SEND_BREAK);
                    #endif /* End Tag_PARITY_TYPE != NONE */
                }
            #endif    /* End Tag_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Tag_SetTxAddressMode
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
    void Tag_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Tag_CONTROL_REG_REMOVED == 0u )
                Tag_WriteControlRegister(Tag_ReadControlRegister() |
                                                      Tag_CTRL_MARK);
            #endif /* End Tag_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Tag_CONTROL_REG_REMOVED == 0u )
                Tag_WriteControlRegister(Tag_ReadControlRegister() &
                                                    (uint8)~Tag_CTRL_MARK);
            #endif /* End Tag_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndTag_TX_ENABLED */

#if(Tag_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Tag_LoadTxConfig
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
    void Tag_LoadTxConfig(void) 
    {
        #if((Tag_RX_INTERRUPT_ENABLED) && (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Tag_SetRxInterruptMode(0u);
        #endif /* Tag_RX_INTERRUPT_ENABLED */

        Tag_WriteControlRegister(Tag_ReadControlRegister() | Tag_CTRL_HD_SEND);
        Tag_RXBITCTR_PERIOD_REG = Tag_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Tag_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Tag_LoadRxConfig
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
    void Tag_LoadRxConfig(void) 
    {
        Tag_WriteControlRegister(Tag_ReadControlRegister() &
                                                (uint8)~Tag_CTRL_HD_SEND);
        Tag_RXBITCTR_PERIOD_REG = Tag_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Tag_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Tag_RX_INTERRUPT_ENABLED) && (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Tag_SetRxInterruptMode(Tag_INIT_RX_INTERRUPTS_MASK);
        #endif /* Tag_RX_INTERRUPT_ENABLED */
    }

#endif  /* Tag_HD_ENABLED */


/* [] END OF FILE */
