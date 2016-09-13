/*******************************************************************************
* File Name: Pistola.c
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

#include "Pistola.h"
#include "CyLib.h"
#if(Pistola_INTERNAL_CLOCK_USED)
    #include "Pistola_IntClock.h"
#endif /* End Pistola_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Pistola_initVar = 0u;
#if( Pistola_TX_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
    volatile uint8 Pistola_txBuffer[Pistola_TXBUFFERSIZE];
    volatile uint8 Pistola_txBufferRead = 0u;
    uint8 Pistola_txBufferWrite = 0u;
#endif /* End Pistola_TX_ENABLED */
#if( ( Pistola_RX_ENABLED || Pistola_HD_ENABLED ) && \
     (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH) )
    volatile uint8 Pistola_rxBuffer[Pistola_RXBUFFERSIZE];
    volatile uint8 Pistola_rxBufferRead = 0u;
    volatile uint8 Pistola_rxBufferWrite = 0u;
    volatile uint8 Pistola_rxBufferLoopDetect = 0u;
    volatile uint8 Pistola_rxBufferOverflow = 0u;
    #if (Pistola_RXHW_ADDRESS_ENABLED)
        volatile uint8 Pistola_rxAddressMode = Pistola_RXADDRESSMODE;
        volatile uint8 Pistola_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Pistola_RX_ENABLED */


/*******************************************************************************
* Function Name: Pistola_Start
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
*  The Pistola_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Pistola_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pistola_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Pistola_initVar == 0u)
    {
        Pistola_Init();
        Pistola_initVar = 1u;
    }
    Pistola_Enable();
}


/*******************************************************************************
* Function Name: Pistola_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Pistola_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Pistola_Init(void) 
{
    #if(Pistola_RX_ENABLED || Pistola_HD_ENABLED)

        #if(Pistola_RX_INTERRUPT_ENABLED && (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Pistola_RX_VECT_NUM, &Pistola_RXISR);
            CyIntSetPriority(Pistola_RX_VECT_NUM, Pistola_RX_PRIOR_NUM);
        #endif /* End Pistola_RX_INTERRUPT_ENABLED */

        #if (Pistola_RXHW_ADDRESS_ENABLED)
            Pistola_SetRxAddressMode(Pistola_RXAddressMode);
            Pistola_SetRxAddress1(Pistola_RXHWADDRESS1);
            Pistola_SetRxAddress2(Pistola_RXHWADDRESS2);
        #endif /* End Pistola_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Pistola_RXBITCTR_PERIOD_REG = Pistola_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Pistola_RXSTATUS_MASK_REG  = Pistola_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Pistola_RX_ENABLED || Pistola_HD_ENABLED*/

    #if(Pistola_TX_ENABLED)
        #if(Pistola_TX_INTERRUPT_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Pistola_TX_VECT_NUM, &Pistola_TXISR);
            CyIntSetPriority(Pistola_TX_VECT_NUM, Pistola_TX_PRIOR_NUM);
        #endif /* End Pistola_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Pistola_TXCLKGEN_DP)
            Pistola_TXBITCLKGEN_CTR_REG = Pistola_BIT_CENTER;
            Pistola_TXBITCLKTX_COMPLETE_REG = (Pistola_NUMBER_OF_DATA_BITS +
                        Pistola_NUMBER_OF_START_BIT) * Pistola_OVER_SAMPLE_COUNT;
        #else
            Pistola_TXBITCTR_PERIOD_REG = ((Pistola_NUMBER_OF_DATA_BITS +
                        Pistola_NUMBER_OF_START_BIT) * Pistola_OVER_SAMPLE_8) - 1u;
        #endif /* End Pistola_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Pistola_TX_INTERRUPT_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
            Pistola_TXSTATUS_MASK_REG = Pistola_TX_STS_FIFO_EMPTY;
        #else
            Pistola_TXSTATUS_MASK_REG = Pistola_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Pistola_TX_INTERRUPT_ENABLED*/

    #endif /* End Pistola_TX_ENABLED */

    #if(Pistola_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Pistola_WriteControlRegister( \
            (Pistola_ReadControlRegister() & (uint8)~Pistola_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Pistola_PARITY_TYPE << Pistola_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Pistola_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Pistola_Enable
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
*  Pistola_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Pistola_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Pistola_RX_ENABLED || Pistola_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Pistola_RXBITCTR_CONTROL_REG |= Pistola_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Pistola_RXSTATUS_ACTL_REG  |= Pistola_INT_ENABLE;
        #if(Pistola_RX_INTERRUPT_ENABLED && (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH))
            CyIntEnable(Pistola_RX_VECT_NUM);
            #if (Pistola_RXHW_ADDRESS_ENABLED)
                Pistola_rxAddressDetected = 0u;
            #endif /* End Pistola_RXHW_ADDRESS_ENABLED */
        #endif /* End Pistola_RX_INTERRUPT_ENABLED */
    #endif /* End Pistola_RX_ENABLED || Pistola_HD_ENABLED*/

    #if(Pistola_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Pistola_TXCLKGEN_DP)
            Pistola_TXBITCTR_CONTROL_REG |= Pistola_CNTR_ENABLE;
        #endif /* End Pistola_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Pistola_TXSTATUS_ACTL_REG |= Pistola_INT_ENABLE;
        #if(Pistola_TX_INTERRUPT_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
            CyIntEnable(Pistola_TX_VECT_NUM);
        #endif /* End Pistola_TX_INTERRUPT_ENABLED*/
     #endif /* End Pistola_TX_ENABLED */

    #if(Pistola_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Pistola_IntClock_Start();
    #endif /* End Pistola_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Pistola_Stop
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
void Pistola_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Pistola_RX_ENABLED || Pistola_HD_ENABLED)
        Pistola_RXBITCTR_CONTROL_REG &= (uint8)~Pistola_CNTR_ENABLE;
    #endif /* End Pistola_RX_ENABLED */

    #if(Pistola_TX_ENABLED)
        #if(!Pistola_TXCLKGEN_DP)
            Pistola_TXBITCTR_CONTROL_REG &= (uint8)~Pistola_CNTR_ENABLE;
        #endif /* End Pistola_TXCLKGEN_DP */
    #endif /* Pistola_TX_ENABLED */

    #if(Pistola_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Pistola_IntClock_Stop();
    #endif /* End Pistola_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Pistola_RX_ENABLED || Pistola_HD_ENABLED)
        Pistola_RXSTATUS_ACTL_REG  &= (uint8)~Pistola_INT_ENABLE;
        #if(Pistola_RX_INTERRUPT_ENABLED && (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH))
            Pistola_DisableRxInt();
        #endif /* End Pistola_RX_INTERRUPT_ENABLED */
    #endif /* End Pistola_RX_ENABLED */

    #if(Pistola_TX_ENABLED)
        Pistola_TXSTATUS_ACTL_REG &= (uint8)~Pistola_INT_ENABLE;
        #if(Pistola_TX_INTERRUPT_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
            Pistola_DisableTxInt();
        #endif /* End Pistola_TX_INTERRUPT_ENABLED */
    #endif /* End Pistola_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Pistola_ReadControlRegister
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
uint8 Pistola_ReadControlRegister(void) 
{
    #if( Pistola_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Pistola_CONTROL_REG);
    #endif /* End Pistola_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Pistola_WriteControlRegister
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
void  Pistola_WriteControlRegister(uint8 control) 
{
    #if( Pistola_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Pistola_CONTROL_REG = control;
    #endif /* End Pistola_CONTROL_REG_REMOVED */
}


#if(Pistola_RX_ENABLED || Pistola_HD_ENABLED)

    #if(Pistola_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Pistola_EnableRxInt
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
        void Pistola_EnableRxInt(void) 
        {
            CyIntEnable(Pistola_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Pistola_DisableRxInt
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
        void Pistola_DisableRxInt(void) 
        {
            CyIntDisable(Pistola_RX_VECT_NUM);
        }

    #endif /* Pistola_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Pistola_SetRxInterruptMode
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
    void Pistola_SetRxInterruptMode(uint8 intSrc) 
    {
        Pistola_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Pistola_ReadRxData
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
    *  Pistola_rxBuffer - RAM buffer pointer for save received data.
    *  Pistola_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Pistola_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Pistola_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Pistola_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_DisableRxInt();
            #endif /* Pistola_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Pistola_rxBufferRead;
            loc_rxBufferWrite = Pistola_rxBufferWrite;

            if( (Pistola_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Pistola_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Pistola_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Pistola_rxBufferRead = loc_rxBufferRead;

                if(Pistola_rxBufferLoopDetect != 0u )
                {
                    Pistola_rxBufferLoopDetect = 0u;
                    #if( (Pistola_RX_INTERRUPT_ENABLED) && (Pistola_FLOW_CONTROL != 0u) && \
                         (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Pistola_HD_ENABLED )
                            if((Pistola_CONTROL_REG & Pistola_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Pistola_RXSTATUS_MASK_REG  |= Pistola_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Pistola_RXSTATUS_MASK_REG  |= Pistola_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Pistola_HD_ENABLED */
                    #endif /* Pistola_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Pistola_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_EnableRxInt();
            #endif /* End Pistola_RX_INTERRUPT_ENABLED */

        #else /* Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Pistola_RXDATA_REG;

        #endif /* Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Pistola_ReadRxStatus
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
    *  Pistola_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Pistola_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Pistola_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Pistola_ReadRxStatus(void) 
    {
        uint8 status;

        status = Pistola_RXSTATUS_REG & Pistola_RX_HW_MASK;

        #if(Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH)
            if( Pistola_rxBufferOverflow != 0u )
            {
                status |= Pistola_RX_STS_SOFT_BUFF_OVER;
                Pistola_rxBufferOverflow = 0u;
            }
        #endif /* Pistola_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Pistola_GetChar
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
    *  Pistola_rxBuffer - RAM buffer pointer for save received data.
    *  Pistola_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Pistola_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Pistola_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Pistola_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_DisableRxInt();
            #endif /* Pistola_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Pistola_rxBufferRead;
            loc_rxBufferWrite = Pistola_rxBufferWrite;

            if( (Pistola_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Pistola_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Pistola_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Pistola_rxBufferRead = loc_rxBufferRead;

                if(Pistola_rxBufferLoopDetect > 0u )
                {
                    Pistola_rxBufferLoopDetect = 0u;
                    #if( (Pistola_RX_INTERRUPT_ENABLED) && (Pistola_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Pistola_HD_ENABLED )
                            if((Pistola_CONTROL_REG & Pistola_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Pistola_RXSTATUS_MASK_REG  |= Pistola_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Pistola_RXSTATUS_MASK_REG  |= Pistola_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Pistola_HD_ENABLED */
                    #endif /* Pistola_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Pistola_RXSTATUS_REG;
                if((rxStatus & Pistola_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Pistola_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Pistola_RX_STS_BREAK | Pistola_RX_STS_PAR_ERROR |
                                   Pistola_RX_STS_STOP_ERROR | Pistola_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_EnableRxInt();
            #endif /* Pistola_RX_INTERRUPT_ENABLED */

        #else /* Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

            rxStatus =Pistola_RXSTATUS_REG;
            if((rxStatus & Pistola_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Pistola_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Pistola_RX_STS_BREAK | Pistola_RX_STS_PAR_ERROR |
                               Pistola_RX_STS_STOP_ERROR | Pistola_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Pistola_GetByte
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
    uint16 Pistola_GetByte(void) 
    {
        return ( ((uint16)Pistola_ReadRxStatus() << 8u) | Pistola_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Pistola_GetRxBufferSize
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
    *  Pistola_rxBufferWrite - used to calculate left bytes.
    *  Pistola_rxBufferRead - used to calculate left bytes.
    *  Pistola_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 Pistola_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_DisableRxInt();
            #endif /* Pistola_RX_INTERRUPT_ENABLED */

            if(Pistola_rxBufferRead == Pistola_rxBufferWrite)
            {
                if(Pistola_rxBufferLoopDetect > 0u)
                {
                    size = Pistola_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Pistola_rxBufferRead < Pistola_rxBufferWrite)
            {
                size = (Pistola_rxBufferWrite - Pistola_rxBufferRead);
            }
            else
            {
                size = (Pistola_RXBUFFERSIZE - Pistola_rxBufferRead) + Pistola_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_EnableRxInt();
            #endif /* End Pistola_RX_INTERRUPT_ENABLED */

        #else /* Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Pistola_RXSTATUS_REG & Pistola_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Pistola_ClearRxBuffer
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
    *  Pistola_rxBufferWrite - cleared to zero.
    *  Pistola_rxBufferRead - cleared to zero.
    *  Pistola_rxBufferLoopDetect - cleared to zero.
    *  Pistola_rxBufferOverflow - cleared to zero.
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
    void Pistola_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Pistola_RXDATA_AUX_CTL_REG |=  Pistola_RX_FIFO_CLR;
        Pistola_RXDATA_AUX_CTL_REG &= (uint8)~Pistola_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_DisableRxInt();
            #endif /* End Pistola_RX_INTERRUPT_ENABLED */

            Pistola_rxBufferRead = 0u;
            Pistola_rxBufferWrite = 0u;
            Pistola_rxBufferLoopDetect = 0u;
            Pistola_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Pistola_RX_INTERRUPT_ENABLED)
                Pistola_EnableRxInt();
            #endif /* End Pistola_RX_INTERRUPT_ENABLED */
        #endif /* End Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Pistola_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Pistola__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Pistola__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Pistola__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Pistola__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Pistola__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Pistola_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Pistola_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Pistola_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Pistola_RXHW_ADDRESS_ENABLED)
            #if(Pistola_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Pistola_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Pistola_CONTROL_REG & (uint8)~Pistola_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Pistola_CTRL_RXADDR_MODE0_SHIFT);
                Pistola_CONTROL_REG = tmpCtrl;
                #if(Pistola_RX_INTERRUPT_ENABLED && \
                   (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH) )
                    Pistola_rxAddressMode = addressMode;
                    Pistola_rxAddressDetected = 0u;
                #endif /* End Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH*/
            #endif /* End Pistola_CONTROL_REG_REMOVED */
        #else /* Pistola_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Pistola_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Pistola_SetRxAddress1
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
    void Pistola_SetRxAddress1(uint8 address) 

    {
        Pistola_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Pistola_SetRxAddress2
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
    void Pistola_SetRxAddress2(uint8 address) 
    {
        Pistola_RXADDRESS2_REG = address;
    }

#endif  /* Pistola_RX_ENABLED || Pistola_HD_ENABLED*/


#if( (Pistola_TX_ENABLED) || (Pistola_HD_ENABLED) )

    #if(Pistola_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Pistola_EnableTxInt
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
        void Pistola_EnableTxInt(void) 
        {
            CyIntEnable(Pistola_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Pistola_DisableTxInt
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
        void Pistola_DisableTxInt(void) 
        {
            CyIntDisable(Pistola_TX_VECT_NUM);
        }

    #endif /* Pistola_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Pistola_SetTxInterruptMode
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
    void Pistola_SetTxInterruptMode(uint8 intSrc) 
    {
        Pistola_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Pistola_WriteTxData
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
    *  Pistola_txBuffer - RAM buffer pointer for save data for transmission
    *  Pistola_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Pistola_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Pistola_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pistola_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Pistola_initVar != 0u)
        {
            #if(Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Pistola_TX_INTERRUPT_ENABLED)
                    Pistola_DisableTxInt();
                #endif /* End Pistola_TX_INTERRUPT_ENABLED */

                if( (Pistola_txBufferRead == Pistola_txBufferWrite) &&
                    ((Pistola_TXSTATUS_REG & Pistola_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Pistola_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Pistola_txBufferWrite >= Pistola_TXBUFFERSIZE)
                    {
                        Pistola_txBufferWrite = 0u;
                    }

                    Pistola_txBuffer[Pistola_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Pistola_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Pistola_TX_INTERRUPT_ENABLED)
                    Pistola_EnableTxInt();
                #endif /* End Pistola_TX_INTERRUPT_ENABLED */

            #else /* Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Pistola_TXDATA_REG = txDataByte;

            #endif /* End Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Pistola_ReadTxStatus
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
    uint8 Pistola_ReadTxStatus(void) 
    {
        return(Pistola_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Pistola_PutChar
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
    *  Pistola_txBuffer - RAM buffer pointer for save data for transmission
    *  Pistola_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Pistola_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Pistola_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Pistola_PutChar(uint8 txDataByte) 
    {
            #if(Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Pistola_TXBUFFERSIZE > Pistola_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Pistola_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Pistola_txBufferWrite;
                    loc_txBufferRead = Pistola_txBufferRead;
                    #if ((Pistola_TXBUFFERSIZE > Pistola_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Pistola_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Pistola_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Pistola_TXSTATUS_REG & Pistola_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Pistola_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Pistola_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Pistola_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Pistola_TXBUFFERSIZE > Pistola_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Pistola_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Pistola_txBufferWrite = loc_txBufferWrite;
                    #if ((Pistola_TXBUFFERSIZE > Pistola_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Pistola_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */

                while((Pistola_TXSTATUS_REG & Pistola_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Pistola_TXDATA_REG = txDataByte;

            #endif /* End Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Pistola_PutString
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
    *  Pistola_initVar - checked to identify that the component has been
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
    void Pistola_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Pistola_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Pistola_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Pistola_PutArray
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
    *  Pistola_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pistola_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Pistola_initVar != 0u)
        {
            do
            {
                Pistola_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Pistola_PutCRLF
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
    *  Pistola_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pistola_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Pistola_initVar != 0u)
        {
            Pistola_PutChar(txDataByte);
            Pistola_PutChar(0x0Du);
            Pistola_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Pistola_GetTxBufferSize
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
    *  Pistola_txBufferWrite - used to calculate left space.
    *  Pistola_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Pistola_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pistola_TX_INTERRUPT_ENABLED)
                Pistola_DisableTxInt();
            #endif /* End Pistola_TX_INTERRUPT_ENABLED */

            if(Pistola_txBufferRead == Pistola_txBufferWrite)
            {
                size = 0u;
            }
            else if(Pistola_txBufferRead < Pistola_txBufferWrite)
            {
                size = (Pistola_txBufferWrite - Pistola_txBufferRead);
            }
            else
            {
                size = (Pistola_TXBUFFERSIZE - Pistola_txBufferRead) + Pistola_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Pistola_TX_INTERRUPT_ENABLED)
                Pistola_EnableTxInt();
            #endif /* End Pistola_TX_INTERRUPT_ENABLED */

        #else /* Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */

            size = Pistola_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Pistola_TX_STS_FIFO_FULL) != 0u)
            {
                size = Pistola_FIFO_LENGTH;
            }
            else if((size & Pistola_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Pistola_ClearTxBuffer
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
    *  Pistola_txBufferWrite - cleared to zero.
    *  Pistola_txBufferRead - cleared to zero.
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
    void Pistola_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Pistola_TXDATA_AUX_CTL_REG |=  Pistola_TX_FIFO_CLR;
        Pistola_TXDATA_AUX_CTL_REG &= (uint8)~Pistola_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pistola_TX_INTERRUPT_ENABLED)
                Pistola_DisableTxInt();
            #endif /* End Pistola_TX_INTERRUPT_ENABLED */

            Pistola_txBufferRead = 0u;
            Pistola_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Pistola_TX_INTERRUPT_ENABLED)
                Pistola_EnableTxInt();
            #endif /* End Pistola_TX_INTERRUPT_ENABLED */

        #endif /* End Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Pistola_SendBreak
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
    *  Pistola_initVar - checked to identify that the component has been
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
    void Pistola_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Pistola_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Pistola_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Pistola_SEND_BREAK) ||
                    (retMode == Pistola_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Pistola_WriteControlRegister(Pistola_ReadControlRegister() |
                                                          Pistola_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Pistola_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Pistola_TXSTATUS_REG;
                    }while((tmpStat & Pistola_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Pistola_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pistola_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Pistola_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Pistola_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Pistola_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pistola_REINIT) ||
                    (retMode == Pistola_SEND_WAIT_REINIT) )
                {
                    Pistola_WriteControlRegister(Pistola_ReadControlRegister() &
                                                  (uint8)~Pistola_CTRL_HD_SEND_BREAK);
                }

            #else /* Pistola_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Pistola_SEND_BREAK) ||
                    (retMode == Pistola_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Pistola_PARITY_TYPE != Pistola__B_UART__NONE_REVB) || \
                                        (Pistola_PARITY_TYPE_SW != 0u) )
                        Pistola_WriteControlRegister(Pistola_ReadControlRegister() |
                                                              Pistola_CTRL_HD_SEND_BREAK);
                    #endif /* End Pistola_PARITY_TYPE != Pistola__B_UART__NONE_REVB  */

                    #if(Pistola_TXCLKGEN_DP)
                        tx_period = Pistola_TXBITCLKTX_COMPLETE_REG;
                        Pistola_TXBITCLKTX_COMPLETE_REG = Pistola_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Pistola_TXBITCTR_PERIOD_REG;
                        Pistola_TXBITCTR_PERIOD_REG = Pistola_TXBITCTR_BREAKBITS8X;
                    #endif /* End Pistola_TXCLKGEN_DP */

                    /* Send zeros*/
                    Pistola_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Pistola_TXSTATUS_REG;
                    }while((tmpStat & Pistola_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Pistola_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pistola_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Pistola_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Pistola_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Pistola_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pistola_REINIT) ||
                    (retMode == Pistola_SEND_WAIT_REINIT) )
                {

                    #if(Pistola_TXCLKGEN_DP)
                        Pistola_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Pistola_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Pistola_TXCLKGEN_DP */

                    #if( (Pistola_PARITY_TYPE != Pistola__B_UART__NONE_REVB) || \
                         (Pistola_PARITY_TYPE_SW != 0u) )
                        Pistola_WriteControlRegister(Pistola_ReadControlRegister() &
                                                      (uint8)~Pistola_CTRL_HD_SEND_BREAK);
                    #endif /* End Pistola_PARITY_TYPE != NONE */
                }
            #endif    /* End Pistola_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Pistola_SetTxAddressMode
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
    void Pistola_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Pistola_CONTROL_REG_REMOVED == 0u )
                Pistola_WriteControlRegister(Pistola_ReadControlRegister() |
                                                      Pistola_CTRL_MARK);
            #endif /* End Pistola_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Pistola_CONTROL_REG_REMOVED == 0u )
                Pistola_WriteControlRegister(Pistola_ReadControlRegister() &
                                                    (uint8)~Pistola_CTRL_MARK);
            #endif /* End Pistola_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndPistola_TX_ENABLED */

#if(Pistola_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Pistola_LoadTxConfig
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
    void Pistola_LoadTxConfig(void) 
    {
        #if((Pistola_RX_INTERRUPT_ENABLED) && (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Pistola_SetRxInterruptMode(0u);
        #endif /* Pistola_RX_INTERRUPT_ENABLED */

        Pistola_WriteControlRegister(Pistola_ReadControlRegister() | Pistola_CTRL_HD_SEND);
        Pistola_RXBITCTR_PERIOD_REG = Pistola_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Pistola_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Pistola_LoadRxConfig
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
    void Pistola_LoadRxConfig(void) 
    {
        Pistola_WriteControlRegister(Pistola_ReadControlRegister() &
                                                (uint8)~Pistola_CTRL_HD_SEND);
        Pistola_RXBITCTR_PERIOD_REG = Pistola_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Pistola_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Pistola_RX_INTERRUPT_ENABLED) && (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Pistola_SetRxInterruptMode(Pistola_INIT_RX_INTERRUPTS_MASK);
        #endif /* Pistola_RX_INTERRUPT_ENABLED */
    }

#endif  /* Pistola_HD_ENABLED */


/* [] END OF FILE */
