/*******************************************************************************
* File Name: Code_Bar.c
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

#include "Code_Bar.h"
#include "CyLib.h"
#if(Code_Bar_INTERNAL_CLOCK_USED)
    #include "Code_Bar_IntClock.h"
#endif /* End Code_Bar_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Code_Bar_initVar = 0u;
#if( Code_Bar_TX_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
    volatile uint8 Code_Bar_txBuffer[Code_Bar_TXBUFFERSIZE];
    volatile uint8 Code_Bar_txBufferRead = 0u;
    uint8 Code_Bar_txBufferWrite = 0u;
#endif /* End Code_Bar_TX_ENABLED */
#if( ( Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED ) && \
     (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH) )
    volatile uint8 Code_Bar_rxBuffer[Code_Bar_RXBUFFERSIZE];
    volatile uint16 Code_Bar_rxBufferRead = 0u;
    volatile uint16 Code_Bar_rxBufferWrite = 0u;
    volatile uint8 Code_Bar_rxBufferLoopDetect = 0u;
    volatile uint8 Code_Bar_rxBufferOverflow = 0u;
    #if (Code_Bar_RXHW_ADDRESS_ENABLED)
        volatile uint8 Code_Bar_rxAddressMode = Code_Bar_RXADDRESSMODE;
        volatile uint8 Code_Bar_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Code_Bar_RX_ENABLED */


/*******************************************************************************
* Function Name: Code_Bar_Start
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
*  The Code_Bar_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Code_Bar_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Code_Bar_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Code_Bar_initVar == 0u)
    {
        Code_Bar_Init();
        Code_Bar_initVar = 1u;
    }
    Code_Bar_Enable();
}


/*******************************************************************************
* Function Name: Code_Bar_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Code_Bar_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Code_Bar_Init(void) 
{
    #if(Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED)

        #if(Code_Bar_RX_INTERRUPT_ENABLED && (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Code_Bar_RX_VECT_NUM, &Code_Bar_RXISR);
            CyIntSetPriority(Code_Bar_RX_VECT_NUM, Code_Bar_RX_PRIOR_NUM);
        #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */

        #if (Code_Bar_RXHW_ADDRESS_ENABLED)
            Code_Bar_SetRxAddressMode(Code_Bar_RXAddressMode);
            Code_Bar_SetRxAddress1(Code_Bar_RXHWADDRESS1);
            Code_Bar_SetRxAddress2(Code_Bar_RXHWADDRESS2);
        #endif /* End Code_Bar_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Code_Bar_RXBITCTR_PERIOD_REG = Code_Bar_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Code_Bar_RXSTATUS_MASK_REG  = Code_Bar_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED*/

    #if(Code_Bar_TX_ENABLED)
        #if(Code_Bar_TX_INTERRUPT_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Code_Bar_TX_VECT_NUM, &Code_Bar_TXISR);
            CyIntSetPriority(Code_Bar_TX_VECT_NUM, Code_Bar_TX_PRIOR_NUM);
        #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Code_Bar_TXCLKGEN_DP)
            Code_Bar_TXBITCLKGEN_CTR_REG = Code_Bar_BIT_CENTER;
            Code_Bar_TXBITCLKTX_COMPLETE_REG = (Code_Bar_NUMBER_OF_DATA_BITS +
                        Code_Bar_NUMBER_OF_START_BIT) * Code_Bar_OVER_SAMPLE_COUNT;
        #else
            Code_Bar_TXBITCTR_PERIOD_REG = ((Code_Bar_NUMBER_OF_DATA_BITS +
                        Code_Bar_NUMBER_OF_START_BIT) * Code_Bar_OVER_SAMPLE_8) - 1u;
        #endif /* End Code_Bar_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Code_Bar_TX_INTERRUPT_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            Code_Bar_TXSTATUS_MASK_REG = Code_Bar_TX_STS_FIFO_EMPTY;
        #else
            Code_Bar_TXSTATUS_MASK_REG = Code_Bar_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Code_Bar_TX_INTERRUPT_ENABLED*/

    #endif /* End Code_Bar_TX_ENABLED */

    #if(Code_Bar_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Code_Bar_WriteControlRegister( \
            (Code_Bar_ReadControlRegister() & (uint8)~Code_Bar_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Code_Bar_PARITY_TYPE << Code_Bar_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Code_Bar_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Code_Bar_Enable
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
*  Code_Bar_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Code_Bar_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Code_Bar_RXBITCTR_CONTROL_REG |= Code_Bar_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Code_Bar_RXSTATUS_ACTL_REG  |= Code_Bar_INT_ENABLE;
        #if(Code_Bar_RX_INTERRUPT_ENABLED && (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            CyIntEnable(Code_Bar_RX_VECT_NUM);
            #if (Code_Bar_RXHW_ADDRESS_ENABLED)
                Code_Bar_rxAddressDetected = 0u;
            #endif /* End Code_Bar_RXHW_ADDRESS_ENABLED */
        #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */
    #endif /* End Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED*/

    #if(Code_Bar_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Code_Bar_TXCLKGEN_DP)
            Code_Bar_TXBITCTR_CONTROL_REG |= Code_Bar_CNTR_ENABLE;
        #endif /* End Code_Bar_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Code_Bar_TXSTATUS_ACTL_REG |= Code_Bar_INT_ENABLE;
        #if(Code_Bar_TX_INTERRUPT_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            CyIntEnable(Code_Bar_TX_VECT_NUM);
        #endif /* End Code_Bar_TX_INTERRUPT_ENABLED*/
     #endif /* End Code_Bar_TX_ENABLED */

    #if(Code_Bar_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Code_Bar_IntClock_Start();
    #endif /* End Code_Bar_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Code_Bar_Stop
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
void Code_Bar_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED)
        Code_Bar_RXBITCTR_CONTROL_REG &= (uint8)~Code_Bar_CNTR_ENABLE;
    #endif /* End Code_Bar_RX_ENABLED */

    #if(Code_Bar_TX_ENABLED)
        #if(!Code_Bar_TXCLKGEN_DP)
            Code_Bar_TXBITCTR_CONTROL_REG &= (uint8)~Code_Bar_CNTR_ENABLE;
        #endif /* End Code_Bar_TXCLKGEN_DP */
    #endif /* Code_Bar_TX_ENABLED */

    #if(Code_Bar_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Code_Bar_IntClock_Stop();
    #endif /* End Code_Bar_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED)
        Code_Bar_RXSTATUS_ACTL_REG  &= (uint8)~Code_Bar_INT_ENABLE;
        #if(Code_Bar_RX_INTERRUPT_ENABLED && (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            Code_Bar_DisableRxInt();
        #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */
    #endif /* End Code_Bar_RX_ENABLED */

    #if(Code_Bar_TX_ENABLED)
        Code_Bar_TXSTATUS_ACTL_REG &= (uint8)~Code_Bar_INT_ENABLE;
        #if(Code_Bar_TX_INTERRUPT_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            Code_Bar_DisableTxInt();
        #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */
    #endif /* End Code_Bar_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Code_Bar_ReadControlRegister
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
uint8 Code_Bar_ReadControlRegister(void) 
{
    #if( Code_Bar_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Code_Bar_CONTROL_REG);
    #endif /* End Code_Bar_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Code_Bar_WriteControlRegister
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
void  Code_Bar_WriteControlRegister(uint8 control) 
{
    #if( Code_Bar_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Code_Bar_CONTROL_REG = control;
    #endif /* End Code_Bar_CONTROL_REG_REMOVED */
}


#if(Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED)

    #if(Code_Bar_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Code_Bar_EnableRxInt
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
        void Code_Bar_EnableRxInt(void) 
        {
            CyIntEnable(Code_Bar_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Code_Bar_DisableRxInt
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
        void Code_Bar_DisableRxInt(void) 
        {
            CyIntDisable(Code_Bar_RX_VECT_NUM);
        }

    #endif /* Code_Bar_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Code_Bar_SetRxInterruptMode
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
    void Code_Bar_SetRxInterruptMode(uint8 intSrc) 
    {
        Code_Bar_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Code_Bar_ReadRxData
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
    *  Code_Bar_rxBuffer - RAM buffer pointer for save received data.
    *  Code_Bar_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Code_Bar_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Code_Bar_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Code_Bar_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_DisableRxInt();
            #endif /* Code_Bar_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Code_Bar_rxBufferRead;
            loc_rxBufferWrite = Code_Bar_rxBufferWrite;

            if( (Code_Bar_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Code_Bar_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Code_Bar_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Code_Bar_rxBufferRead = loc_rxBufferRead;

                if(Code_Bar_rxBufferLoopDetect != 0u )
                {
                    Code_Bar_rxBufferLoopDetect = 0u;
                    #if( (Code_Bar_RX_INTERRUPT_ENABLED) && (Code_Bar_FLOW_CONTROL != 0u) && \
                         (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Code_Bar_HD_ENABLED )
                            if((Code_Bar_CONTROL_REG & Code_Bar_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Code_Bar_RXSTATUS_MASK_REG  |= Code_Bar_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Code_Bar_RXSTATUS_MASK_REG  |= Code_Bar_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Code_Bar_HD_ENABLED */
                    #endif /* Code_Bar_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Code_Bar_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_EnableRxInt();
            #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */

        #else /* Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Code_Bar_RXDATA_REG;

        #endif /* Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Code_Bar_ReadRxStatus
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
    *  Code_Bar_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Code_Bar_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Code_Bar_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Code_Bar_ReadRxStatus(void) 
    {
        uint8 status;

        status = Code_Bar_RXSTATUS_REG & Code_Bar_RX_HW_MASK;

        #if(Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH)
            if( Code_Bar_rxBufferOverflow != 0u )
            {
                status |= Code_Bar_RX_STS_SOFT_BUFF_OVER;
                Code_Bar_rxBufferOverflow = 0u;
            }
        #endif /* Code_Bar_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Code_Bar_GetChar
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
    *  Code_Bar_rxBuffer - RAM buffer pointer for save received data.
    *  Code_Bar_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Code_Bar_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Code_Bar_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Code_Bar_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_DisableRxInt();
            #endif /* Code_Bar_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Code_Bar_rxBufferRead;
            loc_rxBufferWrite = Code_Bar_rxBufferWrite;

            if( (Code_Bar_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Code_Bar_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Code_Bar_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Code_Bar_rxBufferRead = loc_rxBufferRead;

                if(Code_Bar_rxBufferLoopDetect > 0u )
                {
                    Code_Bar_rxBufferLoopDetect = 0u;
                    #if( (Code_Bar_RX_INTERRUPT_ENABLED) && (Code_Bar_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Code_Bar_HD_ENABLED )
                            if((Code_Bar_CONTROL_REG & Code_Bar_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Code_Bar_RXSTATUS_MASK_REG  |= Code_Bar_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Code_Bar_RXSTATUS_MASK_REG  |= Code_Bar_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Code_Bar_HD_ENABLED */
                    #endif /* Code_Bar_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Code_Bar_RXSTATUS_REG;
                if((rxStatus & Code_Bar_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Code_Bar_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Code_Bar_RX_STS_BREAK | Code_Bar_RX_STS_PAR_ERROR |
                                   Code_Bar_RX_STS_STOP_ERROR | Code_Bar_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_EnableRxInt();
            #endif /* Code_Bar_RX_INTERRUPT_ENABLED */

        #else /* Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

            rxStatus =Code_Bar_RXSTATUS_REG;
            if((rxStatus & Code_Bar_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Code_Bar_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Code_Bar_RX_STS_BREAK | Code_Bar_RX_STS_PAR_ERROR |
                               Code_Bar_RX_STS_STOP_ERROR | Code_Bar_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Code_Bar_GetByte
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
    uint16 Code_Bar_GetByte(void) 
    {
        return ( ((uint16)Code_Bar_ReadRxStatus() << 8u) | Code_Bar_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Code_Bar_GetRxBufferSize
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
    *  Code_Bar_rxBufferWrite - used to calculate left bytes.
    *  Code_Bar_rxBufferRead - used to calculate left bytes.
    *  Code_Bar_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 Code_Bar_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_DisableRxInt();
            #endif /* Code_Bar_RX_INTERRUPT_ENABLED */

            if(Code_Bar_rxBufferRead == Code_Bar_rxBufferWrite)
            {
                if(Code_Bar_rxBufferLoopDetect > 0u)
                {
                    size = Code_Bar_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Code_Bar_rxBufferRead < Code_Bar_rxBufferWrite)
            {
                size = (Code_Bar_rxBufferWrite - Code_Bar_rxBufferRead);
            }
            else
            {
                size = (Code_Bar_RXBUFFERSIZE - Code_Bar_rxBufferRead) + Code_Bar_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_EnableRxInt();
            #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */

        #else /* Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Code_Bar_RXSTATUS_REG & Code_Bar_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Code_Bar_ClearRxBuffer
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
    *  Code_Bar_rxBufferWrite - cleared to zero.
    *  Code_Bar_rxBufferRead - cleared to zero.
    *  Code_Bar_rxBufferLoopDetect - cleared to zero.
    *  Code_Bar_rxBufferOverflow - cleared to zero.
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
    void Code_Bar_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Code_Bar_RXDATA_AUX_CTL_REG |=  Code_Bar_RX_FIFO_CLR;
        Code_Bar_RXDATA_AUX_CTL_REG &= (uint8)~Code_Bar_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_DisableRxInt();
            #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */

            Code_Bar_rxBufferRead = 0u;
            Code_Bar_rxBufferWrite = 0u;
            Code_Bar_rxBufferLoopDetect = 0u;
            Code_Bar_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Code_Bar_RX_INTERRUPT_ENABLED)
                Code_Bar_EnableRxInt();
            #endif /* End Code_Bar_RX_INTERRUPT_ENABLED */
        #endif /* End Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Code_Bar_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Code_Bar__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Code_Bar__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Code_Bar__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Code_Bar__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Code_Bar__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Code_Bar_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Code_Bar_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Code_Bar_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Code_Bar_RXHW_ADDRESS_ENABLED)
            #if(Code_Bar_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Code_Bar_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Code_Bar_CONTROL_REG & (uint8)~Code_Bar_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Code_Bar_CTRL_RXADDR_MODE0_SHIFT);
                Code_Bar_CONTROL_REG = tmpCtrl;
                #if(Code_Bar_RX_INTERRUPT_ENABLED && \
                   (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH) )
                    Code_Bar_rxAddressMode = addressMode;
                    Code_Bar_rxAddressDetected = 0u;
                #endif /* End Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH*/
            #endif /* End Code_Bar_CONTROL_REG_REMOVED */
        #else /* Code_Bar_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Code_Bar_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Code_Bar_SetRxAddress1
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
    void Code_Bar_SetRxAddress1(uint8 address) 

    {
        Code_Bar_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Code_Bar_SetRxAddress2
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
    void Code_Bar_SetRxAddress2(uint8 address) 
    {
        Code_Bar_RXADDRESS2_REG = address;
    }

#endif  /* Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED*/


#if( (Code_Bar_TX_ENABLED) || (Code_Bar_HD_ENABLED) )

    #if(Code_Bar_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Code_Bar_EnableTxInt
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
        void Code_Bar_EnableTxInt(void) 
        {
            CyIntEnable(Code_Bar_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Code_Bar_DisableTxInt
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
        void Code_Bar_DisableTxInt(void) 
        {
            CyIntDisable(Code_Bar_TX_VECT_NUM);
        }

    #endif /* Code_Bar_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Code_Bar_SetTxInterruptMode
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
    void Code_Bar_SetTxInterruptMode(uint8 intSrc) 
    {
        Code_Bar_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Code_Bar_WriteTxData
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
    *  Code_Bar_txBuffer - RAM buffer pointer for save data for transmission
    *  Code_Bar_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Code_Bar_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Code_Bar_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Code_Bar_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Code_Bar_initVar != 0u)
        {
            #if(Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Code_Bar_TX_INTERRUPT_ENABLED)
                    Code_Bar_DisableTxInt();
                #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

                if( (Code_Bar_txBufferRead == Code_Bar_txBufferWrite) &&
                    ((Code_Bar_TXSTATUS_REG & Code_Bar_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Code_Bar_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Code_Bar_txBufferWrite >= Code_Bar_TXBUFFERSIZE)
                    {
                        Code_Bar_txBufferWrite = 0u;
                    }

                    Code_Bar_txBuffer[Code_Bar_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Code_Bar_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Code_Bar_TX_INTERRUPT_ENABLED)
                    Code_Bar_EnableTxInt();
                #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

            #else /* Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Code_Bar_TXDATA_REG = txDataByte;

            #endif /* End Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Code_Bar_ReadTxStatus
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
    uint8 Code_Bar_ReadTxStatus(void) 
    {
        return(Code_Bar_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Code_Bar_PutChar
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
    *  Code_Bar_txBuffer - RAM buffer pointer for save data for transmission
    *  Code_Bar_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Code_Bar_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Code_Bar_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Code_Bar_PutChar(uint8 txDataByte) 
    {
            #if(Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Code_Bar_TXBUFFERSIZE > Code_Bar_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Code_Bar_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Code_Bar_txBufferWrite;
                    loc_txBufferRead = Code_Bar_txBufferRead;
                    #if ((Code_Bar_TXBUFFERSIZE > Code_Bar_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Code_Bar_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Code_Bar_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Code_Bar_TXSTATUS_REG & Code_Bar_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Code_Bar_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Code_Bar_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Code_Bar_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Code_Bar_TXBUFFERSIZE > Code_Bar_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Code_Bar_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Code_Bar_txBufferWrite = loc_txBufferWrite;
                    #if ((Code_Bar_TXBUFFERSIZE > Code_Bar_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Code_Bar_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

                while((Code_Bar_TXSTATUS_REG & Code_Bar_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Code_Bar_TXDATA_REG = txDataByte;

            #endif /* End Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Code_Bar_PutString
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
    *  Code_Bar_initVar - checked to identify that the component has been
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
    void Code_Bar_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Code_Bar_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Code_Bar_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Code_Bar_PutArray
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
    *  Code_Bar_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Code_Bar_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Code_Bar_initVar != 0u)
        {
            do
            {
                Code_Bar_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Code_Bar_PutCRLF
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
    *  Code_Bar_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Code_Bar_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Code_Bar_initVar != 0u)
        {
            Code_Bar_PutChar(txDataByte);
            Code_Bar_PutChar(0x0Du);
            Code_Bar_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Code_Bar_GetTxBufferSize
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
    *  Code_Bar_txBufferWrite - used to calculate left space.
    *  Code_Bar_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Code_Bar_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Code_Bar_TX_INTERRUPT_ENABLED)
                Code_Bar_DisableTxInt();
            #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

            if(Code_Bar_txBufferRead == Code_Bar_txBufferWrite)
            {
                size = 0u;
            }
            else if(Code_Bar_txBufferRead < Code_Bar_txBufferWrite)
            {
                size = (Code_Bar_txBufferWrite - Code_Bar_txBufferRead);
            }
            else
            {
                size = (Code_Bar_TXBUFFERSIZE - Code_Bar_txBufferRead) + Code_Bar_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Code_Bar_TX_INTERRUPT_ENABLED)
                Code_Bar_EnableTxInt();
            #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

        #else /* Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

            size = Code_Bar_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Code_Bar_TX_STS_FIFO_FULL) != 0u)
            {
                size = Code_Bar_FIFO_LENGTH;
            }
            else if((size & Code_Bar_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Code_Bar_ClearTxBuffer
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
    *  Code_Bar_txBufferWrite - cleared to zero.
    *  Code_Bar_txBufferRead - cleared to zero.
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
    void Code_Bar_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Code_Bar_TXDATA_AUX_CTL_REG |=  Code_Bar_TX_FIFO_CLR;
        Code_Bar_TXDATA_AUX_CTL_REG &= (uint8)~Code_Bar_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Code_Bar_TX_INTERRUPT_ENABLED)
                Code_Bar_DisableTxInt();
            #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

            Code_Bar_txBufferRead = 0u;
            Code_Bar_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Code_Bar_TX_INTERRUPT_ENABLED)
                Code_Bar_EnableTxInt();
            #endif /* End Code_Bar_TX_INTERRUPT_ENABLED */

        #endif /* End Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Code_Bar_SendBreak
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
    *  Code_Bar_initVar - checked to identify that the component has been
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
    void Code_Bar_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Code_Bar_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Code_Bar_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Code_Bar_SEND_BREAK) ||
                    (retMode == Code_Bar_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() |
                                                          Code_Bar_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Code_Bar_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Code_Bar_TXSTATUS_REG;
                    }while((tmpStat & Code_Bar_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Code_Bar_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Code_Bar_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Code_Bar_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Code_Bar_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Code_Bar_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Code_Bar_REINIT) ||
                    (retMode == Code_Bar_SEND_WAIT_REINIT) )
                {
                    Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() &
                                                  (uint8)~Code_Bar_CTRL_HD_SEND_BREAK);
                }

            #else /* Code_Bar_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Code_Bar_SEND_BREAK) ||
                    (retMode == Code_Bar_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Code_Bar_PARITY_TYPE != Code_Bar__B_UART__NONE_REVB) || \
                                        (Code_Bar_PARITY_TYPE_SW != 0u) )
                        Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() |
                                                              Code_Bar_CTRL_HD_SEND_BREAK);
                    #endif /* End Code_Bar_PARITY_TYPE != Code_Bar__B_UART__NONE_REVB  */

                    #if(Code_Bar_TXCLKGEN_DP)
                        tx_period = Code_Bar_TXBITCLKTX_COMPLETE_REG;
                        Code_Bar_TXBITCLKTX_COMPLETE_REG = Code_Bar_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Code_Bar_TXBITCTR_PERIOD_REG;
                        Code_Bar_TXBITCTR_PERIOD_REG = Code_Bar_TXBITCTR_BREAKBITS8X;
                    #endif /* End Code_Bar_TXCLKGEN_DP */

                    /* Send zeros*/
                    Code_Bar_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Code_Bar_TXSTATUS_REG;
                    }while((tmpStat & Code_Bar_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Code_Bar_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Code_Bar_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Code_Bar_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Code_Bar_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Code_Bar_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Code_Bar_REINIT) ||
                    (retMode == Code_Bar_SEND_WAIT_REINIT) )
                {

                    #if(Code_Bar_TXCLKGEN_DP)
                        Code_Bar_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Code_Bar_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Code_Bar_TXCLKGEN_DP */

                    #if( (Code_Bar_PARITY_TYPE != Code_Bar__B_UART__NONE_REVB) || \
                         (Code_Bar_PARITY_TYPE_SW != 0u) )
                        Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() &
                                                      (uint8)~Code_Bar_CTRL_HD_SEND_BREAK);
                    #endif /* End Code_Bar_PARITY_TYPE != NONE */
                }
            #endif    /* End Code_Bar_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Code_Bar_SetTxAddressMode
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
    void Code_Bar_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Code_Bar_CONTROL_REG_REMOVED == 0u )
                Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() |
                                                      Code_Bar_CTRL_MARK);
            #endif /* End Code_Bar_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Code_Bar_CONTROL_REG_REMOVED == 0u )
                Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() &
                                                    (uint8)~Code_Bar_CTRL_MARK);
            #endif /* End Code_Bar_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndCode_Bar_TX_ENABLED */

#if(Code_Bar_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Code_Bar_LoadTxConfig
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
    void Code_Bar_LoadTxConfig(void) 
    {
        #if((Code_Bar_RX_INTERRUPT_ENABLED) && (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Code_Bar_SetRxInterruptMode(0u);
        #endif /* Code_Bar_RX_INTERRUPT_ENABLED */

        Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() | Code_Bar_CTRL_HD_SEND);
        Code_Bar_RXBITCTR_PERIOD_REG = Code_Bar_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Code_Bar_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Code_Bar_LoadRxConfig
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
    void Code_Bar_LoadRxConfig(void) 
    {
        Code_Bar_WriteControlRegister(Code_Bar_ReadControlRegister() &
                                                (uint8)~Code_Bar_CTRL_HD_SEND);
        Code_Bar_RXBITCTR_PERIOD_REG = Code_Bar_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Code_Bar_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Code_Bar_RX_INTERRUPT_ENABLED) && (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Code_Bar_SetRxInterruptMode(Code_Bar_INIT_RX_INTERRUPTS_MASK);
        #endif /* Code_Bar_RX_INTERRUPT_ENABLED */
    }

#endif  /* Code_Bar_HD_ENABLED */


/* [] END OF FILE */
