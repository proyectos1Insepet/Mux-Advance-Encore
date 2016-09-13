/*******************************************************************************
* File Name: Beagle.c
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

#include "Beagle.h"
#include "CyLib.h"
#if(Beagle_INTERNAL_CLOCK_USED)
    #include "Beagle_IntClock.h"
#endif /* End Beagle_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Beagle_initVar = 0u;
#if( Beagle_TX_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
    volatile uint8 Beagle_txBuffer[Beagle_TXBUFFERSIZE];
    volatile uint8 Beagle_txBufferRead = 0u;
    uint8 Beagle_txBufferWrite = 0u;
#endif /* End Beagle_TX_ENABLED */
#if( ( Beagle_RX_ENABLED || Beagle_HD_ENABLED ) && \
     (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH) )
    volatile uint8 Beagle_rxBuffer[Beagle_RXBUFFERSIZE];
    volatile uint16 Beagle_rxBufferRead = 0u;
    volatile uint16 Beagle_rxBufferWrite = 0u;
    volatile uint8 Beagle_rxBufferLoopDetect = 0u;
    volatile uint8 Beagle_rxBufferOverflow = 0u;
    #if (Beagle_RXHW_ADDRESS_ENABLED)
        volatile uint8 Beagle_rxAddressMode = Beagle_RXADDRESSMODE;
        volatile uint8 Beagle_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Beagle_RX_ENABLED */


/*******************************************************************************
* Function Name: Beagle_Start
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
*  The Beagle_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Beagle_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Beagle_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Beagle_initVar == 0u)
    {
        Beagle_Init();
        Beagle_initVar = 1u;
    }
    Beagle_Enable();
}


/*******************************************************************************
* Function Name: Beagle_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Beagle_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Beagle_Init(void) 
{
    #if(Beagle_RX_ENABLED || Beagle_HD_ENABLED)

        #if(Beagle_RX_INTERRUPT_ENABLED && (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Beagle_RX_VECT_NUM, &Beagle_RXISR);
            CyIntSetPriority(Beagle_RX_VECT_NUM, Beagle_RX_PRIOR_NUM);
        #endif /* End Beagle_RX_INTERRUPT_ENABLED */

        #if (Beagle_RXHW_ADDRESS_ENABLED)
            Beagle_SetRxAddressMode(Beagle_RXAddressMode);
            Beagle_SetRxAddress1(Beagle_RXHWADDRESS1);
            Beagle_SetRxAddress2(Beagle_RXHWADDRESS2);
        #endif /* End Beagle_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Beagle_RXBITCTR_PERIOD_REG = Beagle_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Beagle_RXSTATUS_MASK_REG  = Beagle_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Beagle_RX_ENABLED || Beagle_HD_ENABLED*/

    #if(Beagle_TX_ENABLED)
        #if(Beagle_TX_INTERRUPT_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Beagle_TX_VECT_NUM, &Beagle_TXISR);
            CyIntSetPriority(Beagle_TX_VECT_NUM, Beagle_TX_PRIOR_NUM);
        #endif /* End Beagle_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Beagle_TXCLKGEN_DP)
            Beagle_TXBITCLKGEN_CTR_REG = Beagle_BIT_CENTER;
            Beagle_TXBITCLKTX_COMPLETE_REG = (Beagle_NUMBER_OF_DATA_BITS +
                        Beagle_NUMBER_OF_START_BIT) * Beagle_OVER_SAMPLE_COUNT;
        #else
            Beagle_TXBITCTR_PERIOD_REG = ((Beagle_NUMBER_OF_DATA_BITS +
                        Beagle_NUMBER_OF_START_BIT) * Beagle_OVER_SAMPLE_8) - 1u;
        #endif /* End Beagle_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Beagle_TX_INTERRUPT_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
            Beagle_TXSTATUS_MASK_REG = Beagle_TX_STS_FIFO_EMPTY;
        #else
            Beagle_TXSTATUS_MASK_REG = Beagle_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Beagle_TX_INTERRUPT_ENABLED*/

    #endif /* End Beagle_TX_ENABLED */

    #if(Beagle_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Beagle_WriteControlRegister( \
            (Beagle_ReadControlRegister() & (uint8)~Beagle_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Beagle_PARITY_TYPE << Beagle_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Beagle_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Beagle_Enable
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
*  Beagle_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Beagle_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Beagle_RX_ENABLED || Beagle_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Beagle_RXBITCTR_CONTROL_REG |= Beagle_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Beagle_RXSTATUS_ACTL_REG  |= Beagle_INT_ENABLE;
        #if(Beagle_RX_INTERRUPT_ENABLED && (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH))
            CyIntEnable(Beagle_RX_VECT_NUM);
            #if (Beagle_RXHW_ADDRESS_ENABLED)
                Beagle_rxAddressDetected = 0u;
            #endif /* End Beagle_RXHW_ADDRESS_ENABLED */
        #endif /* End Beagle_RX_INTERRUPT_ENABLED */
    #endif /* End Beagle_RX_ENABLED || Beagle_HD_ENABLED*/

    #if(Beagle_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Beagle_TXCLKGEN_DP)
            Beagle_TXBITCTR_CONTROL_REG |= Beagle_CNTR_ENABLE;
        #endif /* End Beagle_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Beagle_TXSTATUS_ACTL_REG |= Beagle_INT_ENABLE;
        #if(Beagle_TX_INTERRUPT_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
            CyIntEnable(Beagle_TX_VECT_NUM);
        #endif /* End Beagle_TX_INTERRUPT_ENABLED*/
     #endif /* End Beagle_TX_ENABLED */

    #if(Beagle_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Beagle_IntClock_Start();
    #endif /* End Beagle_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Beagle_Stop
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
void Beagle_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Beagle_RX_ENABLED || Beagle_HD_ENABLED)
        Beagle_RXBITCTR_CONTROL_REG &= (uint8)~Beagle_CNTR_ENABLE;
    #endif /* End Beagle_RX_ENABLED */

    #if(Beagle_TX_ENABLED)
        #if(!Beagle_TXCLKGEN_DP)
            Beagle_TXBITCTR_CONTROL_REG &= (uint8)~Beagle_CNTR_ENABLE;
        #endif /* End Beagle_TXCLKGEN_DP */
    #endif /* Beagle_TX_ENABLED */

    #if(Beagle_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Beagle_IntClock_Stop();
    #endif /* End Beagle_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Beagle_RX_ENABLED || Beagle_HD_ENABLED)
        Beagle_RXSTATUS_ACTL_REG  &= (uint8)~Beagle_INT_ENABLE;
        #if(Beagle_RX_INTERRUPT_ENABLED && (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH))
            Beagle_DisableRxInt();
        #endif /* End Beagle_RX_INTERRUPT_ENABLED */
    #endif /* End Beagle_RX_ENABLED */

    #if(Beagle_TX_ENABLED)
        Beagle_TXSTATUS_ACTL_REG &= (uint8)~Beagle_INT_ENABLE;
        #if(Beagle_TX_INTERRUPT_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
            Beagle_DisableTxInt();
        #endif /* End Beagle_TX_INTERRUPT_ENABLED */
    #endif /* End Beagle_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Beagle_ReadControlRegister
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
uint8 Beagle_ReadControlRegister(void) 
{
    #if( Beagle_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Beagle_CONTROL_REG);
    #endif /* End Beagle_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Beagle_WriteControlRegister
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
void  Beagle_WriteControlRegister(uint8 control) 
{
    #if( Beagle_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Beagle_CONTROL_REG = control;
    #endif /* End Beagle_CONTROL_REG_REMOVED */
}


#if(Beagle_RX_ENABLED || Beagle_HD_ENABLED)

    #if(Beagle_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Beagle_EnableRxInt
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
        void Beagle_EnableRxInt(void) 
        {
            CyIntEnable(Beagle_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Beagle_DisableRxInt
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
        void Beagle_DisableRxInt(void) 
        {
            CyIntDisable(Beagle_RX_VECT_NUM);
        }

    #endif /* Beagle_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Beagle_SetRxInterruptMode
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
    void Beagle_SetRxInterruptMode(uint8 intSrc) 
    {
        Beagle_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Beagle_ReadRxData
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
    *  Beagle_rxBuffer - RAM buffer pointer for save received data.
    *  Beagle_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Beagle_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Beagle_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Beagle_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_DisableRxInt();
            #endif /* Beagle_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Beagle_rxBufferRead;
            loc_rxBufferWrite = Beagle_rxBufferWrite;

            if( (Beagle_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Beagle_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Beagle_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Beagle_rxBufferRead = loc_rxBufferRead;

                if(Beagle_rxBufferLoopDetect != 0u )
                {
                    Beagle_rxBufferLoopDetect = 0u;
                    #if( (Beagle_RX_INTERRUPT_ENABLED) && (Beagle_FLOW_CONTROL != 0u) && \
                         (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Beagle_HD_ENABLED )
                            if((Beagle_CONTROL_REG & Beagle_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Beagle_RXSTATUS_MASK_REG  |= Beagle_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Beagle_RXSTATUS_MASK_REG  |= Beagle_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Beagle_HD_ENABLED */
                    #endif /* Beagle_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Beagle_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_EnableRxInt();
            #endif /* End Beagle_RX_INTERRUPT_ENABLED */

        #else /* Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Beagle_RXDATA_REG;

        #endif /* Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Beagle_ReadRxStatus
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
    *  Beagle_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Beagle_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Beagle_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Beagle_ReadRxStatus(void) 
    {
        uint8 status;

        status = Beagle_RXSTATUS_REG & Beagle_RX_HW_MASK;

        #if(Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH)
            if( Beagle_rxBufferOverflow != 0u )
            {
                status |= Beagle_RX_STS_SOFT_BUFF_OVER;
                Beagle_rxBufferOverflow = 0u;
            }
        #endif /* Beagle_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Beagle_GetChar
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
    *  Beagle_rxBuffer - RAM buffer pointer for save received data.
    *  Beagle_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Beagle_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Beagle_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Beagle_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_DisableRxInt();
            #endif /* Beagle_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Beagle_rxBufferRead;
            loc_rxBufferWrite = Beagle_rxBufferWrite;

            if( (Beagle_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Beagle_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Beagle_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Beagle_rxBufferRead = loc_rxBufferRead;

                if(Beagle_rxBufferLoopDetect > 0u )
                {
                    Beagle_rxBufferLoopDetect = 0u;
                    #if( (Beagle_RX_INTERRUPT_ENABLED) && (Beagle_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Beagle_HD_ENABLED )
                            if((Beagle_CONTROL_REG & Beagle_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Beagle_RXSTATUS_MASK_REG  |= Beagle_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Beagle_RXSTATUS_MASK_REG  |= Beagle_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Beagle_HD_ENABLED */
                    #endif /* Beagle_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Beagle_RXSTATUS_REG;
                if((rxStatus & Beagle_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Beagle_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Beagle_RX_STS_BREAK | Beagle_RX_STS_PAR_ERROR |
                                   Beagle_RX_STS_STOP_ERROR | Beagle_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_EnableRxInt();
            #endif /* Beagle_RX_INTERRUPT_ENABLED */

        #else /* Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

            rxStatus =Beagle_RXSTATUS_REG;
            if((rxStatus & Beagle_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Beagle_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Beagle_RX_STS_BREAK | Beagle_RX_STS_PAR_ERROR |
                               Beagle_RX_STS_STOP_ERROR | Beagle_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Beagle_GetByte
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
    uint16 Beagle_GetByte(void) 
    {
        return ( ((uint16)Beagle_ReadRxStatus() << 8u) | Beagle_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Beagle_GetRxBufferSize
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
    *  Beagle_rxBufferWrite - used to calculate left bytes.
    *  Beagle_rxBufferRead - used to calculate left bytes.
    *  Beagle_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 Beagle_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_DisableRxInt();
            #endif /* Beagle_RX_INTERRUPT_ENABLED */

            if(Beagle_rxBufferRead == Beagle_rxBufferWrite)
            {
                if(Beagle_rxBufferLoopDetect > 0u)
                {
                    size = Beagle_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Beagle_rxBufferRead < Beagle_rxBufferWrite)
            {
                size = (Beagle_rxBufferWrite - Beagle_rxBufferRead);
            }
            else
            {
                size = (Beagle_RXBUFFERSIZE - Beagle_rxBufferRead) + Beagle_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_EnableRxInt();
            #endif /* End Beagle_RX_INTERRUPT_ENABLED */

        #else /* Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Beagle_RXSTATUS_REG & Beagle_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Beagle_ClearRxBuffer
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
    *  Beagle_rxBufferWrite - cleared to zero.
    *  Beagle_rxBufferRead - cleared to zero.
    *  Beagle_rxBufferLoopDetect - cleared to zero.
    *  Beagle_rxBufferOverflow - cleared to zero.
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
    void Beagle_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Beagle_RXDATA_AUX_CTL_REG |=  Beagle_RX_FIFO_CLR;
        Beagle_RXDATA_AUX_CTL_REG &= (uint8)~Beagle_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_DisableRxInt();
            #endif /* End Beagle_RX_INTERRUPT_ENABLED */

            Beagle_rxBufferRead = 0u;
            Beagle_rxBufferWrite = 0u;
            Beagle_rxBufferLoopDetect = 0u;
            Beagle_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Beagle_RX_INTERRUPT_ENABLED)
                Beagle_EnableRxInt();
            #endif /* End Beagle_RX_INTERRUPT_ENABLED */
        #endif /* End Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Beagle_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Beagle__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Beagle__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Beagle__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Beagle__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Beagle__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Beagle_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Beagle_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Beagle_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Beagle_RXHW_ADDRESS_ENABLED)
            #if(Beagle_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Beagle_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Beagle_CONTROL_REG & (uint8)~Beagle_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Beagle_CTRL_RXADDR_MODE0_SHIFT);
                Beagle_CONTROL_REG = tmpCtrl;
                #if(Beagle_RX_INTERRUPT_ENABLED && \
                   (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH) )
                    Beagle_rxAddressMode = addressMode;
                    Beagle_rxAddressDetected = 0u;
                #endif /* End Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH*/
            #endif /* End Beagle_CONTROL_REG_REMOVED */
        #else /* Beagle_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Beagle_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Beagle_SetRxAddress1
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
    void Beagle_SetRxAddress1(uint8 address) 

    {
        Beagle_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Beagle_SetRxAddress2
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
    void Beagle_SetRxAddress2(uint8 address) 
    {
        Beagle_RXADDRESS2_REG = address;
    }

#endif  /* Beagle_RX_ENABLED || Beagle_HD_ENABLED*/


#if( (Beagle_TX_ENABLED) || (Beagle_HD_ENABLED) )

    #if(Beagle_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Beagle_EnableTxInt
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
        void Beagle_EnableTxInt(void) 
        {
            CyIntEnable(Beagle_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Beagle_DisableTxInt
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
        void Beagle_DisableTxInt(void) 
        {
            CyIntDisable(Beagle_TX_VECT_NUM);
        }

    #endif /* Beagle_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Beagle_SetTxInterruptMode
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
    void Beagle_SetTxInterruptMode(uint8 intSrc) 
    {
        Beagle_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Beagle_WriteTxData
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
    *  Beagle_txBuffer - RAM buffer pointer for save data for transmission
    *  Beagle_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Beagle_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Beagle_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Beagle_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Beagle_initVar != 0u)
        {
            #if(Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Beagle_TX_INTERRUPT_ENABLED)
                    Beagle_DisableTxInt();
                #endif /* End Beagle_TX_INTERRUPT_ENABLED */

                if( (Beagle_txBufferRead == Beagle_txBufferWrite) &&
                    ((Beagle_TXSTATUS_REG & Beagle_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Beagle_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Beagle_txBufferWrite >= Beagle_TXBUFFERSIZE)
                    {
                        Beagle_txBufferWrite = 0u;
                    }

                    Beagle_txBuffer[Beagle_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Beagle_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Beagle_TX_INTERRUPT_ENABLED)
                    Beagle_EnableTxInt();
                #endif /* End Beagle_TX_INTERRUPT_ENABLED */

            #else /* Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Beagle_TXDATA_REG = txDataByte;

            #endif /* End Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Beagle_ReadTxStatus
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
    uint8 Beagle_ReadTxStatus(void) 
    {
        return(Beagle_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Beagle_PutChar
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
    *  Beagle_txBuffer - RAM buffer pointer for save data for transmission
    *  Beagle_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Beagle_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Beagle_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Beagle_PutChar(uint8 txDataByte) 
    {
            #if(Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Beagle_TXBUFFERSIZE > Beagle_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Beagle_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Beagle_txBufferWrite;
                    loc_txBufferRead = Beagle_txBufferRead;
                    #if ((Beagle_TXBUFFERSIZE > Beagle_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Beagle_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Beagle_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Beagle_TXSTATUS_REG & Beagle_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Beagle_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Beagle_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Beagle_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Beagle_TXBUFFERSIZE > Beagle_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Beagle_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Beagle_txBufferWrite = loc_txBufferWrite;
                    #if ((Beagle_TXBUFFERSIZE > Beagle_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Beagle_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */

                while((Beagle_TXSTATUS_REG & Beagle_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Beagle_TXDATA_REG = txDataByte;

            #endif /* End Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Beagle_PutString
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
    *  Beagle_initVar - checked to identify that the component has been
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
    void Beagle_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Beagle_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Beagle_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Beagle_PutArray
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
    *  Beagle_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Beagle_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Beagle_initVar != 0u)
        {
            do
            {
                Beagle_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Beagle_PutCRLF
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
    *  Beagle_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Beagle_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Beagle_initVar != 0u)
        {
            Beagle_PutChar(txDataByte);
            Beagle_PutChar(0x0Du);
            Beagle_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Beagle_GetTxBufferSize
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
    *  Beagle_txBufferWrite - used to calculate left space.
    *  Beagle_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Beagle_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Beagle_TX_INTERRUPT_ENABLED)
                Beagle_DisableTxInt();
            #endif /* End Beagle_TX_INTERRUPT_ENABLED */

            if(Beagle_txBufferRead == Beagle_txBufferWrite)
            {
                size = 0u;
            }
            else if(Beagle_txBufferRead < Beagle_txBufferWrite)
            {
                size = (Beagle_txBufferWrite - Beagle_txBufferRead);
            }
            else
            {
                size = (Beagle_TXBUFFERSIZE - Beagle_txBufferRead) + Beagle_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Beagle_TX_INTERRUPT_ENABLED)
                Beagle_EnableTxInt();
            #endif /* End Beagle_TX_INTERRUPT_ENABLED */

        #else /* Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */

            size = Beagle_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Beagle_TX_STS_FIFO_FULL) != 0u)
            {
                size = Beagle_FIFO_LENGTH;
            }
            else if((size & Beagle_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Beagle_ClearTxBuffer
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
    *  Beagle_txBufferWrite - cleared to zero.
    *  Beagle_txBufferRead - cleared to zero.
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
    void Beagle_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Beagle_TXDATA_AUX_CTL_REG |=  Beagle_TX_FIFO_CLR;
        Beagle_TXDATA_AUX_CTL_REG &= (uint8)~Beagle_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Beagle_TX_INTERRUPT_ENABLED)
                Beagle_DisableTxInt();
            #endif /* End Beagle_TX_INTERRUPT_ENABLED */

            Beagle_txBufferRead = 0u;
            Beagle_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Beagle_TX_INTERRUPT_ENABLED)
                Beagle_EnableTxInt();
            #endif /* End Beagle_TX_INTERRUPT_ENABLED */

        #endif /* End Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Beagle_SendBreak
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
    *  Beagle_initVar - checked to identify that the component has been
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
    void Beagle_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Beagle_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Beagle_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Beagle_SEND_BREAK) ||
                    (retMode == Beagle_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Beagle_WriteControlRegister(Beagle_ReadControlRegister() |
                                                          Beagle_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Beagle_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Beagle_TXSTATUS_REG;
                    }while((tmpStat & Beagle_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Beagle_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Beagle_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Beagle_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Beagle_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Beagle_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Beagle_REINIT) ||
                    (retMode == Beagle_SEND_WAIT_REINIT) )
                {
                    Beagle_WriteControlRegister(Beagle_ReadControlRegister() &
                                                  (uint8)~Beagle_CTRL_HD_SEND_BREAK);
                }

            #else /* Beagle_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Beagle_SEND_BREAK) ||
                    (retMode == Beagle_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Beagle_PARITY_TYPE != Beagle__B_UART__NONE_REVB) || \
                                        (Beagle_PARITY_TYPE_SW != 0u) )
                        Beagle_WriteControlRegister(Beagle_ReadControlRegister() |
                                                              Beagle_CTRL_HD_SEND_BREAK);
                    #endif /* End Beagle_PARITY_TYPE != Beagle__B_UART__NONE_REVB  */

                    #if(Beagle_TXCLKGEN_DP)
                        tx_period = Beagle_TXBITCLKTX_COMPLETE_REG;
                        Beagle_TXBITCLKTX_COMPLETE_REG = Beagle_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Beagle_TXBITCTR_PERIOD_REG;
                        Beagle_TXBITCTR_PERIOD_REG = Beagle_TXBITCTR_BREAKBITS8X;
                    #endif /* End Beagle_TXCLKGEN_DP */

                    /* Send zeros*/
                    Beagle_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Beagle_TXSTATUS_REG;
                    }while((tmpStat & Beagle_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Beagle_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Beagle_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Beagle_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Beagle_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Beagle_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Beagle_REINIT) ||
                    (retMode == Beagle_SEND_WAIT_REINIT) )
                {

                    #if(Beagle_TXCLKGEN_DP)
                        Beagle_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Beagle_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Beagle_TXCLKGEN_DP */

                    #if( (Beagle_PARITY_TYPE != Beagle__B_UART__NONE_REVB) || \
                         (Beagle_PARITY_TYPE_SW != 0u) )
                        Beagle_WriteControlRegister(Beagle_ReadControlRegister() &
                                                      (uint8)~Beagle_CTRL_HD_SEND_BREAK);
                    #endif /* End Beagle_PARITY_TYPE != NONE */
                }
            #endif    /* End Beagle_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Beagle_SetTxAddressMode
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
    void Beagle_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Beagle_CONTROL_REG_REMOVED == 0u )
                Beagle_WriteControlRegister(Beagle_ReadControlRegister() |
                                                      Beagle_CTRL_MARK);
            #endif /* End Beagle_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Beagle_CONTROL_REG_REMOVED == 0u )
                Beagle_WriteControlRegister(Beagle_ReadControlRegister() &
                                                    (uint8)~Beagle_CTRL_MARK);
            #endif /* End Beagle_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndBeagle_TX_ENABLED */

#if(Beagle_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Beagle_LoadTxConfig
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
    void Beagle_LoadTxConfig(void) 
    {
        #if((Beagle_RX_INTERRUPT_ENABLED) && (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Beagle_SetRxInterruptMode(0u);
        #endif /* Beagle_RX_INTERRUPT_ENABLED */

        Beagle_WriteControlRegister(Beagle_ReadControlRegister() | Beagle_CTRL_HD_SEND);
        Beagle_RXBITCTR_PERIOD_REG = Beagle_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Beagle_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Beagle_LoadRxConfig
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
    void Beagle_LoadRxConfig(void) 
    {
        Beagle_WriteControlRegister(Beagle_ReadControlRegister() &
                                                (uint8)~Beagle_CTRL_HD_SEND);
        Beagle_RXBITCTR_PERIOD_REG = Beagle_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Beagle_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Beagle_RX_INTERRUPT_ENABLED) && (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Beagle_SetRxInterruptMode(Beagle_INIT_RX_INTERRUPTS_MASK);
        #endif /* Beagle_RX_INTERRUPT_ENABLED */
    }

#endif  /* Beagle_HD_ENABLED */


/* [] END OF FILE */
