/*******************************************************************************
* File Name: Pump.c
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

#include "Pump.h"
#include "CyLib.h"
#if(Pump_INTERNAL_CLOCK_USED)
    #include "Pump_IntClock.h"
#endif /* End Pump_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Pump_initVar = 0u;
#if( Pump_TX_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
    volatile uint8 Pump_txBuffer[Pump_TXBUFFERSIZE];
    volatile uint8 Pump_txBufferRead = 0u;
    uint8 Pump_txBufferWrite = 0u;
#endif /* End Pump_TX_ENABLED */
#if( ( Pump_RX_ENABLED || Pump_HD_ENABLED ) && \
     (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH) )
    volatile uint8 Pump_rxBuffer[Pump_RXBUFFERSIZE];
    volatile uint16 Pump_rxBufferRead = 0u;
    volatile uint16 Pump_rxBufferWrite = 0u;
    volatile uint8 Pump_rxBufferLoopDetect = 0u;
    volatile uint8 Pump_rxBufferOverflow = 0u;
    #if (Pump_RXHW_ADDRESS_ENABLED)
        volatile uint8 Pump_rxAddressMode = Pump_RXADDRESSMODE;
        volatile uint8 Pump_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Pump_RX_ENABLED */


/*******************************************************************************
* Function Name: Pump_Start
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
*  The Pump_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Pump_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Pump_initVar == 0u)
    {
        Pump_Init();
        Pump_initVar = 1u;
    }
    Pump_Enable();
}


/*******************************************************************************
* Function Name: Pump_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Pump_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Pump_Init(void) 
{
    #if(Pump_RX_ENABLED || Pump_HD_ENABLED)

        #if(Pump_RX_INTERRUPT_ENABLED && (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Pump_RX_VECT_NUM, &Pump_RXISR);
            CyIntSetPriority(Pump_RX_VECT_NUM, Pump_RX_PRIOR_NUM);
        #endif /* End Pump_RX_INTERRUPT_ENABLED */

        #if (Pump_RXHW_ADDRESS_ENABLED)
            Pump_SetRxAddressMode(Pump_RXAddressMode);
            Pump_SetRxAddress1(Pump_RXHWADDRESS1);
            Pump_SetRxAddress2(Pump_RXHWADDRESS2);
        #endif /* End Pump_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Pump_RXBITCTR_PERIOD_REG = Pump_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Pump_RXSTATUS_MASK_REG  = Pump_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Pump_RX_ENABLED || Pump_HD_ENABLED*/

    #if(Pump_TX_ENABLED)
        #if(Pump_TX_INTERRUPT_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Pump_TX_VECT_NUM, &Pump_TXISR);
            CyIntSetPriority(Pump_TX_VECT_NUM, Pump_TX_PRIOR_NUM);
        #endif /* End Pump_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Pump_TXCLKGEN_DP)
            Pump_TXBITCLKGEN_CTR_REG = Pump_BIT_CENTER;
            Pump_TXBITCLKTX_COMPLETE_REG = (Pump_NUMBER_OF_DATA_BITS +
                        Pump_NUMBER_OF_START_BIT) * Pump_OVER_SAMPLE_COUNT;
        #else
            Pump_TXBITCTR_PERIOD_REG = ((Pump_NUMBER_OF_DATA_BITS +
                        Pump_NUMBER_OF_START_BIT) * Pump_OVER_SAMPLE_8) - 1u;
        #endif /* End Pump_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Pump_TX_INTERRUPT_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
            Pump_TXSTATUS_MASK_REG = Pump_TX_STS_FIFO_EMPTY;
        #else
            Pump_TXSTATUS_MASK_REG = Pump_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Pump_TX_INTERRUPT_ENABLED*/

    #endif /* End Pump_TX_ENABLED */

    #if(Pump_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Pump_WriteControlRegister( \
            (Pump_ReadControlRegister() & (uint8)~Pump_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Pump_PARITY_TYPE << Pump_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Pump_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Pump_Enable
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
*  Pump_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Pump_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Pump_RX_ENABLED || Pump_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Pump_RXBITCTR_CONTROL_REG |= Pump_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Pump_RXSTATUS_ACTL_REG  |= Pump_INT_ENABLE;
        #if(Pump_RX_INTERRUPT_ENABLED && (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH))
            CyIntEnable(Pump_RX_VECT_NUM);
            #if (Pump_RXHW_ADDRESS_ENABLED)
                Pump_rxAddressDetected = 0u;
            #endif /* End Pump_RXHW_ADDRESS_ENABLED */
        #endif /* End Pump_RX_INTERRUPT_ENABLED */
    #endif /* End Pump_RX_ENABLED || Pump_HD_ENABLED*/

    #if(Pump_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Pump_TXCLKGEN_DP)
            Pump_TXBITCTR_CONTROL_REG |= Pump_CNTR_ENABLE;
        #endif /* End Pump_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Pump_TXSTATUS_ACTL_REG |= Pump_INT_ENABLE;
        #if(Pump_TX_INTERRUPT_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
            CyIntEnable(Pump_TX_VECT_NUM);
        #endif /* End Pump_TX_INTERRUPT_ENABLED*/
     #endif /* End Pump_TX_ENABLED */

    #if(Pump_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Pump_IntClock_Start();
    #endif /* End Pump_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Pump_Stop
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
void Pump_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Pump_RX_ENABLED || Pump_HD_ENABLED)
        Pump_RXBITCTR_CONTROL_REG &= (uint8)~Pump_CNTR_ENABLE;
    #endif /* End Pump_RX_ENABLED */

    #if(Pump_TX_ENABLED)
        #if(!Pump_TXCLKGEN_DP)
            Pump_TXBITCTR_CONTROL_REG &= (uint8)~Pump_CNTR_ENABLE;
        #endif /* End Pump_TXCLKGEN_DP */
    #endif /* Pump_TX_ENABLED */

    #if(Pump_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Pump_IntClock_Stop();
    #endif /* End Pump_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Pump_RX_ENABLED || Pump_HD_ENABLED)
        Pump_RXSTATUS_ACTL_REG  &= (uint8)~Pump_INT_ENABLE;
        #if(Pump_RX_INTERRUPT_ENABLED && (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH))
            Pump_DisableRxInt();
        #endif /* End Pump_RX_INTERRUPT_ENABLED */
    #endif /* End Pump_RX_ENABLED */

    #if(Pump_TX_ENABLED)
        Pump_TXSTATUS_ACTL_REG &= (uint8)~Pump_INT_ENABLE;
        #if(Pump_TX_INTERRUPT_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
            Pump_DisableTxInt();
        #endif /* End Pump_TX_INTERRUPT_ENABLED */
    #endif /* End Pump_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Pump_ReadControlRegister
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
uint8 Pump_ReadControlRegister(void) 
{
    #if( Pump_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Pump_CONTROL_REG);
    #endif /* End Pump_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Pump_WriteControlRegister
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
void  Pump_WriteControlRegister(uint8 control) 
{
    #if( Pump_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Pump_CONTROL_REG = control;
    #endif /* End Pump_CONTROL_REG_REMOVED */
}


#if(Pump_RX_ENABLED || Pump_HD_ENABLED)

    #if(Pump_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Pump_EnableRxInt
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
        void Pump_EnableRxInt(void) 
        {
            CyIntEnable(Pump_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Pump_DisableRxInt
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
        void Pump_DisableRxInt(void) 
        {
            CyIntDisable(Pump_RX_VECT_NUM);
        }

    #endif /* Pump_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Pump_SetRxInterruptMode
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
    void Pump_SetRxInterruptMode(uint8 intSrc) 
    {
        Pump_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Pump_ReadRxData
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
    *  Pump_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Pump_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Pump_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Pump_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_DisableRxInt();
            #endif /* Pump_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Pump_rxBufferRead;
            loc_rxBufferWrite = Pump_rxBufferWrite;

            if( (Pump_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Pump_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Pump_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Pump_rxBufferRead = loc_rxBufferRead;

                if(Pump_rxBufferLoopDetect != 0u )
                {
                    Pump_rxBufferLoopDetect = 0u;
                    #if( (Pump_RX_INTERRUPT_ENABLED) && (Pump_FLOW_CONTROL != 0u) && \
                         (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Pump_HD_ENABLED )
                            if((Pump_CONTROL_REG & Pump_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Pump_RXSTATUS_MASK_REG  |= Pump_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Pump_RXSTATUS_MASK_REG  |= Pump_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Pump_HD_ENABLED */
                    #endif /* Pump_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Pump_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_EnableRxInt();
            #endif /* End Pump_RX_INTERRUPT_ENABLED */

        #else /* Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Pump_RXDATA_REG;

        #endif /* Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Pump_ReadRxStatus
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
    *  Pump_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Pump_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Pump_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Pump_ReadRxStatus(void) 
    {
        uint8 status;

        status = Pump_RXSTATUS_REG & Pump_RX_HW_MASK;

        #if(Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH)
            if( Pump_rxBufferOverflow != 0u )
            {
                status |= Pump_RX_STS_SOFT_BUFF_OVER;
                Pump_rxBufferOverflow = 0u;
            }
        #endif /* Pump_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Pump_GetChar
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
    *  Pump_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Pump_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Pump_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Pump_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_DisableRxInt();
            #endif /* Pump_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Pump_rxBufferRead;
            loc_rxBufferWrite = Pump_rxBufferWrite;

            if( (Pump_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Pump_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Pump_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Pump_rxBufferRead = loc_rxBufferRead;

                if(Pump_rxBufferLoopDetect > 0u )
                {
                    Pump_rxBufferLoopDetect = 0u;
                    #if( (Pump_RX_INTERRUPT_ENABLED) && (Pump_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Pump_HD_ENABLED )
                            if((Pump_CONTROL_REG & Pump_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Pump_RXSTATUS_MASK_REG  |= Pump_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Pump_RXSTATUS_MASK_REG  |= Pump_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Pump_HD_ENABLED */
                    #endif /* Pump_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Pump_RXSTATUS_REG;
                if((rxStatus & Pump_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Pump_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Pump_RX_STS_BREAK | Pump_RX_STS_PAR_ERROR |
                                   Pump_RX_STS_STOP_ERROR | Pump_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_EnableRxInt();
            #endif /* Pump_RX_INTERRUPT_ENABLED */

        #else /* Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

            rxStatus =Pump_RXSTATUS_REG;
            if((rxStatus & Pump_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Pump_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Pump_RX_STS_BREAK | Pump_RX_STS_PAR_ERROR |
                               Pump_RX_STS_STOP_ERROR | Pump_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Pump_GetByte
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
    uint16 Pump_GetByte(void) 
    {
        return ( ((uint16)Pump_ReadRxStatus() << 8u) | Pump_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Pump_GetRxBufferSize
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
    *  Pump_rxBufferWrite - used to calculate left bytes.
    *  Pump_rxBufferRead - used to calculate left bytes.
    *  Pump_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 Pump_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_DisableRxInt();
            #endif /* Pump_RX_INTERRUPT_ENABLED */

            if(Pump_rxBufferRead == Pump_rxBufferWrite)
            {
                if(Pump_rxBufferLoopDetect > 0u)
                {
                    size = Pump_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Pump_rxBufferRead < Pump_rxBufferWrite)
            {
                size = (Pump_rxBufferWrite - Pump_rxBufferRead);
            }
            else
            {
                size = (Pump_RXBUFFERSIZE - Pump_rxBufferRead) + Pump_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_EnableRxInt();
            #endif /* End Pump_RX_INTERRUPT_ENABLED */

        #else /* Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Pump_RXSTATUS_REG & Pump_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Pump_ClearRxBuffer
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
    *  Pump_rxBufferWrite - cleared to zero.
    *  Pump_rxBufferRead - cleared to zero.
    *  Pump_rxBufferLoopDetect - cleared to zero.
    *  Pump_rxBufferOverflow - cleared to zero.
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
    void Pump_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Pump_RXDATA_AUX_CTL_REG |=  Pump_RX_FIFO_CLR;
        Pump_RXDATA_AUX_CTL_REG &= (uint8)~Pump_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_DisableRxInt();
            #endif /* End Pump_RX_INTERRUPT_ENABLED */

            Pump_rxBufferRead = 0u;
            Pump_rxBufferWrite = 0u;
            Pump_rxBufferLoopDetect = 0u;
            Pump_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Pump_RX_INTERRUPT_ENABLED)
                Pump_EnableRxInt();
            #endif /* End Pump_RX_INTERRUPT_ENABLED */
        #endif /* End Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Pump_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Pump__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Pump__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Pump__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Pump__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Pump__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Pump_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Pump_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Pump_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Pump_RXHW_ADDRESS_ENABLED)
            #if(Pump_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Pump_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Pump_CONTROL_REG & (uint8)~Pump_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Pump_CTRL_RXADDR_MODE0_SHIFT);
                Pump_CONTROL_REG = tmpCtrl;
                #if(Pump_RX_INTERRUPT_ENABLED && \
                   (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH) )
                    Pump_rxAddressMode = addressMode;
                    Pump_rxAddressDetected = 0u;
                #endif /* End Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH*/
            #endif /* End Pump_CONTROL_REG_REMOVED */
        #else /* Pump_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Pump_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Pump_SetRxAddress1
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
    void Pump_SetRxAddress1(uint8 address) 

    {
        Pump_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Pump_SetRxAddress2
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
    void Pump_SetRxAddress2(uint8 address) 
    {
        Pump_RXADDRESS2_REG = address;
    }

#endif  /* Pump_RX_ENABLED || Pump_HD_ENABLED*/


#if( (Pump_TX_ENABLED) || (Pump_HD_ENABLED) )

    #if(Pump_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Pump_EnableTxInt
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
        void Pump_EnableTxInt(void) 
        {
            CyIntEnable(Pump_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Pump_DisableTxInt
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
        void Pump_DisableTxInt(void) 
        {
            CyIntDisable(Pump_TX_VECT_NUM);
        }

    #endif /* Pump_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Pump_SetTxInterruptMode
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
    void Pump_SetTxInterruptMode(uint8 intSrc) 
    {
        Pump_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Pump_WriteTxData
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
    *  Pump_txBuffer - RAM buffer pointer for save data for transmission
    *  Pump_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Pump_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Pump_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pump_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Pump_initVar != 0u)
        {
            #if(Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Pump_TX_INTERRUPT_ENABLED)
                    Pump_DisableTxInt();
                #endif /* End Pump_TX_INTERRUPT_ENABLED */

                if( (Pump_txBufferRead == Pump_txBufferWrite) &&
                    ((Pump_TXSTATUS_REG & Pump_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Pump_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Pump_txBufferWrite >= Pump_TXBUFFERSIZE)
                    {
                        Pump_txBufferWrite = 0u;
                    }

                    Pump_txBuffer[Pump_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Pump_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Pump_TX_INTERRUPT_ENABLED)
                    Pump_EnableTxInt();
                #endif /* End Pump_TX_INTERRUPT_ENABLED */

            #else /* Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Pump_TXDATA_REG = txDataByte;

            #endif /* End Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Pump_ReadTxStatus
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
    uint8 Pump_ReadTxStatus(void) 
    {
        return(Pump_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Pump_PutChar
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
    *  Pump_txBuffer - RAM buffer pointer for save data for transmission
    *  Pump_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Pump_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Pump_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Pump_PutChar(uint8 txDataByte) 
    {
            #if(Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Pump_TXBUFFERSIZE > Pump_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Pump_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Pump_txBufferWrite;
                    loc_txBufferRead = Pump_txBufferRead;
                    #if ((Pump_TXBUFFERSIZE > Pump_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Pump_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Pump_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Pump_TXSTATUS_REG & Pump_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Pump_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Pump_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Pump_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Pump_TXBUFFERSIZE > Pump_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Pump_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Pump_txBufferWrite = loc_txBufferWrite;
                    #if ((Pump_TXBUFFERSIZE > Pump_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Pump_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */

                while((Pump_TXSTATUS_REG & Pump_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Pump_TXDATA_REG = txDataByte;

            #endif /* End Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Pump_PutString
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
    *  Pump_initVar - checked to identify that the component has been
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
    void Pump_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Pump_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Pump_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Pump_PutArray
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
    *  Pump_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pump_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Pump_initVar != 0u)
        {
            do
            {
                Pump_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Pump_PutCRLF
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
    *  Pump_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pump_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Pump_initVar != 0u)
        {
            Pump_PutChar(txDataByte);
            Pump_PutChar(0x0Du);
            Pump_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Pump_GetTxBufferSize
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
    *  Pump_txBufferWrite - used to calculate left space.
    *  Pump_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Pump_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_TX_INTERRUPT_ENABLED)
                Pump_DisableTxInt();
            #endif /* End Pump_TX_INTERRUPT_ENABLED */

            if(Pump_txBufferRead == Pump_txBufferWrite)
            {
                size = 0u;
            }
            else if(Pump_txBufferRead < Pump_txBufferWrite)
            {
                size = (Pump_txBufferWrite - Pump_txBufferRead);
            }
            else
            {
                size = (Pump_TXBUFFERSIZE - Pump_txBufferRead) + Pump_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Pump_TX_INTERRUPT_ENABLED)
                Pump_EnableTxInt();
            #endif /* End Pump_TX_INTERRUPT_ENABLED */

        #else /* Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */

            size = Pump_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Pump_TX_STS_FIFO_FULL) != 0u)
            {
                size = Pump_FIFO_LENGTH;
            }
            else if((size & Pump_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Pump_ClearTxBuffer
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
    *  Pump_txBufferWrite - cleared to zero.
    *  Pump_txBufferRead - cleared to zero.
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
    void Pump_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Pump_TXDATA_AUX_CTL_REG |=  Pump_TX_FIFO_CLR;
        Pump_TXDATA_AUX_CTL_REG &= (uint8)~Pump_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_TX_INTERRUPT_ENABLED)
                Pump_DisableTxInt();
            #endif /* End Pump_TX_INTERRUPT_ENABLED */

            Pump_txBufferRead = 0u;
            Pump_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Pump_TX_INTERRUPT_ENABLED)
                Pump_EnableTxInt();
            #endif /* End Pump_TX_INTERRUPT_ENABLED */

        #endif /* End Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Pump_SendBreak
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
    *  Pump_initVar - checked to identify that the component has been
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
    void Pump_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Pump_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Pump_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Pump_SEND_BREAK) ||
                    (retMode == Pump_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Pump_WriteControlRegister(Pump_ReadControlRegister() |
                                                          Pump_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Pump_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Pump_TXSTATUS_REG;
                    }while((tmpStat & Pump_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Pump_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Pump_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Pump_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Pump_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_REINIT) ||
                    (retMode == Pump_SEND_WAIT_REINIT) )
                {
                    Pump_WriteControlRegister(Pump_ReadControlRegister() &
                                                  (uint8)~Pump_CTRL_HD_SEND_BREAK);
                }

            #else /* Pump_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Pump_SEND_BREAK) ||
                    (retMode == Pump_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Pump_PARITY_TYPE != Pump__B_UART__NONE_REVB) || \
                                        (Pump_PARITY_TYPE_SW != 0u) )
                        Pump_WriteControlRegister(Pump_ReadControlRegister() |
                                                              Pump_CTRL_HD_SEND_BREAK);
                    #endif /* End Pump_PARITY_TYPE != Pump__B_UART__NONE_REVB  */

                    #if(Pump_TXCLKGEN_DP)
                        tx_period = Pump_TXBITCLKTX_COMPLETE_REG;
                        Pump_TXBITCLKTX_COMPLETE_REG = Pump_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Pump_TXBITCTR_PERIOD_REG;
                        Pump_TXBITCTR_PERIOD_REG = Pump_TXBITCTR_BREAKBITS8X;
                    #endif /* End Pump_TXCLKGEN_DP */

                    /* Send zeros*/
                    Pump_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Pump_TXSTATUS_REG;
                    }while((tmpStat & Pump_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Pump_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Pump_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Pump_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Pump_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_REINIT) ||
                    (retMode == Pump_SEND_WAIT_REINIT) )
                {

                    #if(Pump_TXCLKGEN_DP)
                        Pump_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Pump_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Pump_TXCLKGEN_DP */

                    #if( (Pump_PARITY_TYPE != Pump__B_UART__NONE_REVB) || \
                         (Pump_PARITY_TYPE_SW != 0u) )
                        Pump_WriteControlRegister(Pump_ReadControlRegister() &
                                                      (uint8)~Pump_CTRL_HD_SEND_BREAK);
                    #endif /* End Pump_PARITY_TYPE != NONE */
                }
            #endif    /* End Pump_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Pump_SetTxAddressMode
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
    void Pump_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Pump_CONTROL_REG_REMOVED == 0u )
                Pump_WriteControlRegister(Pump_ReadControlRegister() |
                                                      Pump_CTRL_MARK);
            #endif /* End Pump_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Pump_CONTROL_REG_REMOVED == 0u )
                Pump_WriteControlRegister(Pump_ReadControlRegister() &
                                                    (uint8)~Pump_CTRL_MARK);
            #endif /* End Pump_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndPump_TX_ENABLED */

#if(Pump_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Pump_LoadTxConfig
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
    void Pump_LoadTxConfig(void) 
    {
        #if((Pump_RX_INTERRUPT_ENABLED) && (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Pump_SetRxInterruptMode(0u);
        #endif /* Pump_RX_INTERRUPT_ENABLED */

        Pump_WriteControlRegister(Pump_ReadControlRegister() | Pump_CTRL_HD_SEND);
        Pump_RXBITCTR_PERIOD_REG = Pump_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Pump_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Pump_LoadRxConfig
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
    void Pump_LoadRxConfig(void) 
    {
        Pump_WriteControlRegister(Pump_ReadControlRegister() &
                                                (uint8)~Pump_CTRL_HD_SEND);
        Pump_RXBITCTR_PERIOD_REG = Pump_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Pump_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Pump_RX_INTERRUPT_ENABLED) && (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Pump_SetRxInterruptMode(Pump_INIT_RX_INTERRUPTS_MASK);
        #endif /* Pump_RX_INTERRUPT_ENABLED */
    }

#endif  /* Pump_HD_ENABLED */


/* [] END OF FILE */
