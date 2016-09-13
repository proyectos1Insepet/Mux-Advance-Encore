/*******************************************************************************
* File Name: Surtidor.c
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

#include "Surtidor.h"
#include "CyLib.h"
#if(Surtidor_INTERNAL_CLOCK_USED)
    #include "Surtidor_IntClock.h"
#endif /* End Surtidor_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Surtidor_initVar = 0u;
#if( Surtidor_TX_ENABLED && (Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH))
    volatile uint8 Surtidor_txBuffer[Surtidor_TXBUFFERSIZE];
    volatile uint8 Surtidor_txBufferRead = 0u;
    uint8 Surtidor_txBufferWrite = 0u;
#endif /* End Surtidor_TX_ENABLED */
#if( ( Surtidor_RX_ENABLED || Surtidor_HD_ENABLED ) && \
     (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH) )
    volatile uint8 Surtidor_rxBuffer[Surtidor_RXBUFFERSIZE];
    volatile uint16 Surtidor_rxBufferRead = 0u;
    volatile uint16 Surtidor_rxBufferWrite = 0u;
    volatile uint8 Surtidor_rxBufferLoopDetect = 0u;
    volatile uint8 Surtidor_rxBufferOverflow = 0u;
    #if (Surtidor_RXHW_ADDRESS_ENABLED)
        volatile uint8 Surtidor_rxAddressMode = Surtidor_RXADDRESSMODE;
        volatile uint8 Surtidor_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Surtidor_RX_ENABLED */


/*******************************************************************************
* Function Name: Surtidor_Start
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
*  The Surtidor_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Surtidor_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Surtidor_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Surtidor_initVar == 0u)
    {
        Surtidor_Init();
        Surtidor_initVar = 1u;
    }
    Surtidor_Enable();
}


/*******************************************************************************
* Function Name: Surtidor_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Surtidor_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Surtidor_Init(void) 
{
    #if(Surtidor_RX_ENABLED || Surtidor_HD_ENABLED)

        #if(Surtidor_RX_INTERRUPT_ENABLED && (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Surtidor_RX_VECT_NUM, &Surtidor_RXISR);
            CyIntSetPriority(Surtidor_RX_VECT_NUM, Surtidor_RX_PRIOR_NUM);
        #endif /* End Surtidor_RX_INTERRUPT_ENABLED */

        #if (Surtidor_RXHW_ADDRESS_ENABLED)
            Surtidor_SetRxAddressMode(Surtidor_RXAddressMode);
            Surtidor_SetRxAddress1(Surtidor_RXHWADDRESS1);
            Surtidor_SetRxAddress2(Surtidor_RXHWADDRESS2);
        #endif /* End Surtidor_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Surtidor_RXBITCTR_PERIOD_REG = Surtidor_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Surtidor_RXSTATUS_MASK_REG  = Surtidor_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Surtidor_RX_ENABLED || Surtidor_HD_ENABLED*/

    #if(Surtidor_TX_ENABLED)
        #if(Surtidor_TX_INTERRUPT_ENABLED && (Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Surtidor_TX_VECT_NUM, &Surtidor_TXISR);
            CyIntSetPriority(Surtidor_TX_VECT_NUM, Surtidor_TX_PRIOR_NUM);
        #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Surtidor_TXCLKGEN_DP)
            Surtidor_TXBITCLKGEN_CTR_REG = Surtidor_BIT_CENTER;
            Surtidor_TXBITCLKTX_COMPLETE_REG = (Surtidor_NUMBER_OF_DATA_BITS +
                        Surtidor_NUMBER_OF_START_BIT) * Surtidor_OVER_SAMPLE_COUNT;
        #else
            Surtidor_TXBITCTR_PERIOD_REG = ((Surtidor_NUMBER_OF_DATA_BITS +
                        Surtidor_NUMBER_OF_START_BIT) * Surtidor_OVER_SAMPLE_8) - 1u;
        #endif /* End Surtidor_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Surtidor_TX_INTERRUPT_ENABLED && (Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            Surtidor_TXSTATUS_MASK_REG = Surtidor_TX_STS_FIFO_EMPTY;
        #else
            Surtidor_TXSTATUS_MASK_REG = Surtidor_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Surtidor_TX_INTERRUPT_ENABLED*/

    #endif /* End Surtidor_TX_ENABLED */

    #if(Surtidor_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Surtidor_WriteControlRegister( \
            (Surtidor_ReadControlRegister() & (uint8)~Surtidor_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Surtidor_PARITY_TYPE << Surtidor_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Surtidor_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Surtidor_Enable
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
*  Surtidor_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Surtidor_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Surtidor_RX_ENABLED || Surtidor_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Surtidor_RXBITCTR_CONTROL_REG |= Surtidor_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Surtidor_RXSTATUS_ACTL_REG  |= Surtidor_INT_ENABLE;
        #if(Surtidor_RX_INTERRUPT_ENABLED && (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            CyIntEnable(Surtidor_RX_VECT_NUM);
            #if (Surtidor_RXHW_ADDRESS_ENABLED)
                Surtidor_rxAddressDetected = 0u;
            #endif /* End Surtidor_RXHW_ADDRESS_ENABLED */
        #endif /* End Surtidor_RX_INTERRUPT_ENABLED */
    #endif /* End Surtidor_RX_ENABLED || Surtidor_HD_ENABLED*/

    #if(Surtidor_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Surtidor_TXCLKGEN_DP)
            Surtidor_TXBITCTR_CONTROL_REG |= Surtidor_CNTR_ENABLE;
        #endif /* End Surtidor_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Surtidor_TXSTATUS_ACTL_REG |= Surtidor_INT_ENABLE;
        #if(Surtidor_TX_INTERRUPT_ENABLED && (Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            CyIntEnable(Surtidor_TX_VECT_NUM);
        #endif /* End Surtidor_TX_INTERRUPT_ENABLED*/
     #endif /* End Surtidor_TX_ENABLED */

    #if(Surtidor_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Surtidor_IntClock_Start();
    #endif /* End Surtidor_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Surtidor_Stop
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
void Surtidor_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Surtidor_RX_ENABLED || Surtidor_HD_ENABLED)
        Surtidor_RXBITCTR_CONTROL_REG &= (uint8)~Surtidor_CNTR_ENABLE;
    #endif /* End Surtidor_RX_ENABLED */

    #if(Surtidor_TX_ENABLED)
        #if(!Surtidor_TXCLKGEN_DP)
            Surtidor_TXBITCTR_CONTROL_REG &= (uint8)~Surtidor_CNTR_ENABLE;
        #endif /* End Surtidor_TXCLKGEN_DP */
    #endif /* Surtidor_TX_ENABLED */

    #if(Surtidor_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Surtidor_IntClock_Stop();
    #endif /* End Surtidor_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Surtidor_RX_ENABLED || Surtidor_HD_ENABLED)
        Surtidor_RXSTATUS_ACTL_REG  &= (uint8)~Surtidor_INT_ENABLE;
        #if(Surtidor_RX_INTERRUPT_ENABLED && (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            Surtidor_DisableRxInt();
        #endif /* End Surtidor_RX_INTERRUPT_ENABLED */
    #endif /* End Surtidor_RX_ENABLED */

    #if(Surtidor_TX_ENABLED)
        Surtidor_TXSTATUS_ACTL_REG &= (uint8)~Surtidor_INT_ENABLE;
        #if(Surtidor_TX_INTERRUPT_ENABLED && (Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            Surtidor_DisableTxInt();
        #endif /* End Surtidor_TX_INTERRUPT_ENABLED */
    #endif /* End Surtidor_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Surtidor_ReadControlRegister
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
uint8 Surtidor_ReadControlRegister(void) 
{
    #if( Surtidor_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Surtidor_CONTROL_REG);
    #endif /* End Surtidor_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Surtidor_WriteControlRegister
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
void  Surtidor_WriteControlRegister(uint8 control) 
{
    #if( Surtidor_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Surtidor_CONTROL_REG = control;
    #endif /* End Surtidor_CONTROL_REG_REMOVED */
}


#if(Surtidor_RX_ENABLED || Surtidor_HD_ENABLED)

    #if(Surtidor_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Surtidor_EnableRxInt
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
        void Surtidor_EnableRxInt(void) 
        {
            CyIntEnable(Surtidor_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Surtidor_DisableRxInt
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
        void Surtidor_DisableRxInt(void) 
        {
            CyIntDisable(Surtidor_RX_VECT_NUM);
        }

    #endif /* Surtidor_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Surtidor_SetRxInterruptMode
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
    void Surtidor_SetRxInterruptMode(uint8 intSrc) 
    {
        Surtidor_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Surtidor_ReadRxData
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
    *  Surtidor_rxBuffer - RAM buffer pointer for save received data.
    *  Surtidor_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Surtidor_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Surtidor_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Surtidor_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_DisableRxInt();
            #endif /* Surtidor_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Surtidor_rxBufferRead;
            loc_rxBufferWrite = Surtidor_rxBufferWrite;

            if( (Surtidor_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Surtidor_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Surtidor_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Surtidor_rxBufferRead = loc_rxBufferRead;

                if(Surtidor_rxBufferLoopDetect != 0u )
                {
                    Surtidor_rxBufferLoopDetect = 0u;
                    #if( (Surtidor_RX_INTERRUPT_ENABLED) && (Surtidor_FLOW_CONTROL != 0u) && \
                         (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Surtidor_HD_ENABLED )
                            if((Surtidor_CONTROL_REG & Surtidor_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Surtidor_RXSTATUS_MASK_REG  |= Surtidor_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Surtidor_RXSTATUS_MASK_REG  |= Surtidor_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Surtidor_HD_ENABLED */
                    #endif /* Surtidor_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Surtidor_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_EnableRxInt();
            #endif /* End Surtidor_RX_INTERRUPT_ENABLED */

        #else /* Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Surtidor_RXDATA_REG;

        #endif /* Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Surtidor_ReadRxStatus
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
    *  Surtidor_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Surtidor_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Surtidor_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Surtidor_ReadRxStatus(void) 
    {
        uint8 status;

        status = Surtidor_RXSTATUS_REG & Surtidor_RX_HW_MASK;

        #if(Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH)
            if( Surtidor_rxBufferOverflow != 0u )
            {
                status |= Surtidor_RX_STS_SOFT_BUFF_OVER;
                Surtidor_rxBufferOverflow = 0u;
            }
        #endif /* Surtidor_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Surtidor_GetChar
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
    *  Surtidor_rxBuffer - RAM buffer pointer for save received data.
    *  Surtidor_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Surtidor_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Surtidor_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Surtidor_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_DisableRxInt();
            #endif /* Surtidor_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Surtidor_rxBufferRead;
            loc_rxBufferWrite = Surtidor_rxBufferWrite;

            if( (Surtidor_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Surtidor_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Surtidor_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Surtidor_rxBufferRead = loc_rxBufferRead;

                if(Surtidor_rxBufferLoopDetect > 0u )
                {
                    Surtidor_rxBufferLoopDetect = 0u;
                    #if( (Surtidor_RX_INTERRUPT_ENABLED) && (Surtidor_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Surtidor_HD_ENABLED )
                            if((Surtidor_CONTROL_REG & Surtidor_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Surtidor_RXSTATUS_MASK_REG  |= Surtidor_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Surtidor_RXSTATUS_MASK_REG  |= Surtidor_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Surtidor_HD_ENABLED */
                    #endif /* Surtidor_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Surtidor_RXSTATUS_REG;
                if((rxStatus & Surtidor_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Surtidor_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Surtidor_RX_STS_BREAK | Surtidor_RX_STS_PAR_ERROR |
                                   Surtidor_RX_STS_STOP_ERROR | Surtidor_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_EnableRxInt();
            #endif /* Surtidor_RX_INTERRUPT_ENABLED */

        #else /* Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

            rxStatus =Surtidor_RXSTATUS_REG;
            if((rxStatus & Surtidor_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Surtidor_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Surtidor_RX_STS_BREAK | Surtidor_RX_STS_PAR_ERROR |
                               Surtidor_RX_STS_STOP_ERROR | Surtidor_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Surtidor_GetByte
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
    uint16 Surtidor_GetByte(void) 
    {
        return ( ((uint16)Surtidor_ReadRxStatus() << 8u) | Surtidor_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Surtidor_GetRxBufferSize
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
    *  Surtidor_rxBufferWrite - used to calculate left bytes.
    *  Surtidor_rxBufferRead - used to calculate left bytes.
    *  Surtidor_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 Surtidor_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_DisableRxInt();
            #endif /* Surtidor_RX_INTERRUPT_ENABLED */

            if(Surtidor_rxBufferRead == Surtidor_rxBufferWrite)
            {
                if(Surtidor_rxBufferLoopDetect > 0u)
                {
                    size = Surtidor_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Surtidor_rxBufferRead < Surtidor_rxBufferWrite)
            {
                size = (Surtidor_rxBufferWrite - Surtidor_rxBufferRead);
            }
            else
            {
                size = (Surtidor_RXBUFFERSIZE - Surtidor_rxBufferRead) + Surtidor_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_EnableRxInt();
            #endif /* End Surtidor_RX_INTERRUPT_ENABLED */

        #else /* Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Surtidor_RXSTATUS_REG & Surtidor_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Surtidor_ClearRxBuffer
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
    *  Surtidor_rxBufferWrite - cleared to zero.
    *  Surtidor_rxBufferRead - cleared to zero.
    *  Surtidor_rxBufferLoopDetect - cleared to zero.
    *  Surtidor_rxBufferOverflow - cleared to zero.
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
    void Surtidor_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Surtidor_RXDATA_AUX_CTL_REG |=  Surtidor_RX_FIFO_CLR;
        Surtidor_RXDATA_AUX_CTL_REG &= (uint8)~Surtidor_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_DisableRxInt();
            #endif /* End Surtidor_RX_INTERRUPT_ENABLED */

            Surtidor_rxBufferRead = 0u;
            Surtidor_rxBufferWrite = 0u;
            Surtidor_rxBufferLoopDetect = 0u;
            Surtidor_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Surtidor_RX_INTERRUPT_ENABLED)
                Surtidor_EnableRxInt();
            #endif /* End Surtidor_RX_INTERRUPT_ENABLED */
        #endif /* End Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Surtidor_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Surtidor__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Surtidor__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Surtidor__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Surtidor__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Surtidor__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Surtidor_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Surtidor_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Surtidor_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Surtidor_RXHW_ADDRESS_ENABLED)
            #if(Surtidor_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Surtidor_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Surtidor_CONTROL_REG & (uint8)~Surtidor_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Surtidor_CTRL_RXADDR_MODE0_SHIFT);
                Surtidor_CONTROL_REG = tmpCtrl;
                #if(Surtidor_RX_INTERRUPT_ENABLED && \
                   (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH) )
                    Surtidor_rxAddressMode = addressMode;
                    Surtidor_rxAddressDetected = 0u;
                #endif /* End Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH*/
            #endif /* End Surtidor_CONTROL_REG_REMOVED */
        #else /* Surtidor_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Surtidor_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Surtidor_SetRxAddress1
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
    void Surtidor_SetRxAddress1(uint8 address) 

    {
        Surtidor_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Surtidor_SetRxAddress2
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
    void Surtidor_SetRxAddress2(uint8 address) 
    {
        Surtidor_RXADDRESS2_REG = address;
    }

#endif  /* Surtidor_RX_ENABLED || Surtidor_HD_ENABLED*/


#if( (Surtidor_TX_ENABLED) || (Surtidor_HD_ENABLED) )

    #if(Surtidor_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Surtidor_EnableTxInt
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
        void Surtidor_EnableTxInt(void) 
        {
            CyIntEnable(Surtidor_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Surtidor_DisableTxInt
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
        void Surtidor_DisableTxInt(void) 
        {
            CyIntDisable(Surtidor_TX_VECT_NUM);
        }

    #endif /* Surtidor_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Surtidor_SetTxInterruptMode
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
    void Surtidor_SetTxInterruptMode(uint8 intSrc) 
    {
        Surtidor_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Surtidor_WriteTxData
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
    *  Surtidor_txBuffer - RAM buffer pointer for save data for transmission
    *  Surtidor_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Surtidor_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Surtidor_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Surtidor_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Surtidor_initVar != 0u)
        {
            #if(Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Surtidor_TX_INTERRUPT_ENABLED)
                    Surtidor_DisableTxInt();
                #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

                if( (Surtidor_txBufferRead == Surtidor_txBufferWrite) &&
                    ((Surtidor_TXSTATUS_REG & Surtidor_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Surtidor_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Surtidor_txBufferWrite >= Surtidor_TXBUFFERSIZE)
                    {
                        Surtidor_txBufferWrite = 0u;
                    }

                    Surtidor_txBuffer[Surtidor_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Surtidor_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Surtidor_TX_INTERRUPT_ENABLED)
                    Surtidor_EnableTxInt();
                #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

            #else /* Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Surtidor_TXDATA_REG = txDataByte;

            #endif /* End Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Surtidor_ReadTxStatus
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
    uint8 Surtidor_ReadTxStatus(void) 
    {
        return(Surtidor_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Surtidor_PutChar
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
    *  Surtidor_txBuffer - RAM buffer pointer for save data for transmission
    *  Surtidor_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Surtidor_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Surtidor_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Surtidor_PutChar(uint8 txDataByte) 
    {
            #if(Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Surtidor_TXBUFFERSIZE > Surtidor_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Surtidor_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Surtidor_txBufferWrite;
                    loc_txBufferRead = Surtidor_txBufferRead;
                    #if ((Surtidor_TXBUFFERSIZE > Surtidor_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Surtidor_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Surtidor_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Surtidor_TXSTATUS_REG & Surtidor_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Surtidor_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Surtidor_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Surtidor_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Surtidor_TXBUFFERSIZE > Surtidor_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Surtidor_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Surtidor_txBufferWrite = loc_txBufferWrite;
                    #if ((Surtidor_TXBUFFERSIZE > Surtidor_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Surtidor_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */

                while((Surtidor_TXSTATUS_REG & Surtidor_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Surtidor_TXDATA_REG = txDataByte;

            #endif /* End Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Surtidor_PutString
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
    *  Surtidor_initVar - checked to identify that the component has been
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
    void Surtidor_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Surtidor_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Surtidor_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Surtidor_PutArray
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
    *  Surtidor_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Surtidor_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Surtidor_initVar != 0u)
        {
            do
            {
                Surtidor_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Surtidor_PutCRLF
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
    *  Surtidor_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Surtidor_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Surtidor_initVar != 0u)
        {
            Surtidor_PutChar(txDataByte);
            Surtidor_PutChar(0x0Du);
            Surtidor_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Surtidor_GetTxBufferSize
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
    *  Surtidor_txBufferWrite - used to calculate left space.
    *  Surtidor_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Surtidor_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Surtidor_TX_INTERRUPT_ENABLED)
                Surtidor_DisableTxInt();
            #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

            if(Surtidor_txBufferRead == Surtidor_txBufferWrite)
            {
                size = 0u;
            }
            else if(Surtidor_txBufferRead < Surtidor_txBufferWrite)
            {
                size = (Surtidor_txBufferWrite - Surtidor_txBufferRead);
            }
            else
            {
                size = (Surtidor_TXBUFFERSIZE - Surtidor_txBufferRead) + Surtidor_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Surtidor_TX_INTERRUPT_ENABLED)
                Surtidor_EnableTxInt();
            #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

        #else /* Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */

            size = Surtidor_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Surtidor_TX_STS_FIFO_FULL) != 0u)
            {
                size = Surtidor_FIFO_LENGTH;
            }
            else if((size & Surtidor_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Surtidor_ClearTxBuffer
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
    *  Surtidor_txBufferWrite - cleared to zero.
    *  Surtidor_txBufferRead - cleared to zero.
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
    void Surtidor_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Surtidor_TXDATA_AUX_CTL_REG |=  Surtidor_TX_FIFO_CLR;
        Surtidor_TXDATA_AUX_CTL_REG &= (uint8)~Surtidor_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Surtidor_TX_INTERRUPT_ENABLED)
                Surtidor_DisableTxInt();
            #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

            Surtidor_txBufferRead = 0u;
            Surtidor_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Surtidor_TX_INTERRUPT_ENABLED)
                Surtidor_EnableTxInt();
            #endif /* End Surtidor_TX_INTERRUPT_ENABLED */

        #endif /* End Surtidor_TXBUFFERSIZE > Surtidor_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Surtidor_SendBreak
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
    *  Surtidor_initVar - checked to identify that the component has been
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
    void Surtidor_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Surtidor_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Surtidor_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Surtidor_SEND_BREAK) ||
                    (retMode == Surtidor_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() |
                                                          Surtidor_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Surtidor_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Surtidor_TXSTATUS_REG;
                    }while((tmpStat & Surtidor_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Surtidor_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Surtidor_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Surtidor_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Surtidor_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Surtidor_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Surtidor_REINIT) ||
                    (retMode == Surtidor_SEND_WAIT_REINIT) )
                {
                    Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() &
                                                  (uint8)~Surtidor_CTRL_HD_SEND_BREAK);
                }

            #else /* Surtidor_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Surtidor_SEND_BREAK) ||
                    (retMode == Surtidor_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Surtidor_PARITY_TYPE != Surtidor__B_UART__NONE_REVB) || \
                                        (Surtidor_PARITY_TYPE_SW != 0u) )
                        Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() |
                                                              Surtidor_CTRL_HD_SEND_BREAK);
                    #endif /* End Surtidor_PARITY_TYPE != Surtidor__B_UART__NONE_REVB  */

                    #if(Surtidor_TXCLKGEN_DP)
                        tx_period = Surtidor_TXBITCLKTX_COMPLETE_REG;
                        Surtidor_TXBITCLKTX_COMPLETE_REG = Surtidor_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Surtidor_TXBITCTR_PERIOD_REG;
                        Surtidor_TXBITCTR_PERIOD_REG = Surtidor_TXBITCTR_BREAKBITS8X;
                    #endif /* End Surtidor_TXCLKGEN_DP */

                    /* Send zeros*/
                    Surtidor_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Surtidor_TXSTATUS_REG;
                    }while((tmpStat & Surtidor_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Surtidor_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Surtidor_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Surtidor_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Surtidor_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Surtidor_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Surtidor_REINIT) ||
                    (retMode == Surtidor_SEND_WAIT_REINIT) )
                {

                    #if(Surtidor_TXCLKGEN_DP)
                        Surtidor_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Surtidor_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Surtidor_TXCLKGEN_DP */

                    #if( (Surtidor_PARITY_TYPE != Surtidor__B_UART__NONE_REVB) || \
                         (Surtidor_PARITY_TYPE_SW != 0u) )
                        Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() &
                                                      (uint8)~Surtidor_CTRL_HD_SEND_BREAK);
                    #endif /* End Surtidor_PARITY_TYPE != NONE */
                }
            #endif    /* End Surtidor_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Surtidor_SetTxAddressMode
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
    void Surtidor_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Surtidor_CONTROL_REG_REMOVED == 0u )
                Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() |
                                                      Surtidor_CTRL_MARK);
            #endif /* End Surtidor_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Surtidor_CONTROL_REG_REMOVED == 0u )
                Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() &
                                                    (uint8)~Surtidor_CTRL_MARK);
            #endif /* End Surtidor_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndSurtidor_TX_ENABLED */

#if(Surtidor_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Surtidor_LoadTxConfig
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
    void Surtidor_LoadTxConfig(void) 
    {
        #if((Surtidor_RX_INTERRUPT_ENABLED) && (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Surtidor_SetRxInterruptMode(0u);
        #endif /* Surtidor_RX_INTERRUPT_ENABLED */

        Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() | Surtidor_CTRL_HD_SEND);
        Surtidor_RXBITCTR_PERIOD_REG = Surtidor_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Surtidor_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Surtidor_LoadRxConfig
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
    void Surtidor_LoadRxConfig(void) 
    {
        Surtidor_WriteControlRegister(Surtidor_ReadControlRegister() &
                                                (uint8)~Surtidor_CTRL_HD_SEND);
        Surtidor_RXBITCTR_PERIOD_REG = Surtidor_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Surtidor_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Surtidor_RX_INTERRUPT_ENABLED) && (Surtidor_RXBUFFERSIZE > Surtidor_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Surtidor_SetRxInterruptMode(Surtidor_INIT_RX_INTERRUPTS_MASK);
        #endif /* Surtidor_RX_INTERRUPT_ENABLED */
    }

#endif  /* Surtidor_HD_ENABLED */


/* [] END OF FILE */
