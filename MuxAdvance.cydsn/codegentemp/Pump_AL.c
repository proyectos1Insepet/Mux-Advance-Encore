/*******************************************************************************
* File Name: Pump_AL.c
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

#include "Pump_AL.h"
#include "CyLib.h"
#if(Pump_AL_INTERNAL_CLOCK_USED)
    #include "Pump_AL_IntClock.h"
#endif /* End Pump_AL_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Pump_AL_initVar = 0u;
#if( Pump_AL_TX_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
    volatile uint8 Pump_AL_txBuffer[Pump_AL_TXBUFFERSIZE];
    volatile uint8 Pump_AL_txBufferRead = 0u;
    uint8 Pump_AL_txBufferWrite = 0u;
#endif /* End Pump_AL_TX_ENABLED */
#if( ( Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED ) && \
     (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH) )
    volatile uint8 Pump_AL_rxBuffer[Pump_AL_RXBUFFERSIZE];
    volatile uint16 Pump_AL_rxBufferRead = 0u;
    volatile uint16 Pump_AL_rxBufferWrite = 0u;
    volatile uint8 Pump_AL_rxBufferLoopDetect = 0u;
    volatile uint8 Pump_AL_rxBufferOverflow = 0u;
    #if (Pump_AL_RXHW_ADDRESS_ENABLED)
        volatile uint8 Pump_AL_rxAddressMode = Pump_AL_RXADDRESSMODE;
        volatile uint8 Pump_AL_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Pump_AL_RX_ENABLED */


/*******************************************************************************
* Function Name: Pump_AL_Start
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
*  The Pump_AL_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Pump_AL_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Pump_AL_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Pump_AL_initVar == 0u)
    {
        Pump_AL_Init();
        Pump_AL_initVar = 1u;
    }
    Pump_AL_Enable();
}


/*******************************************************************************
* Function Name: Pump_AL_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Pump_AL_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Pump_AL_Init(void) 
{
    #if(Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED)

        #if(Pump_AL_RX_INTERRUPT_ENABLED && (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Pump_AL_RX_VECT_NUM, &Pump_AL_RXISR);
            CyIntSetPriority(Pump_AL_RX_VECT_NUM, Pump_AL_RX_PRIOR_NUM);
        #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */

        #if (Pump_AL_RXHW_ADDRESS_ENABLED)
            Pump_AL_SetRxAddressMode(Pump_AL_RXAddressMode);
            Pump_AL_SetRxAddress1(Pump_AL_RXHWADDRESS1);
            Pump_AL_SetRxAddress2(Pump_AL_RXHWADDRESS2);
        #endif /* End Pump_AL_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Pump_AL_RXBITCTR_PERIOD_REG = Pump_AL_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Pump_AL_RXSTATUS_MASK_REG  = Pump_AL_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED*/

    #if(Pump_AL_TX_ENABLED)
        #if(Pump_AL_TX_INTERRUPT_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Pump_AL_TX_VECT_NUM, &Pump_AL_TXISR);
            CyIntSetPriority(Pump_AL_TX_VECT_NUM, Pump_AL_TX_PRIOR_NUM);
        #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Pump_AL_TXCLKGEN_DP)
            Pump_AL_TXBITCLKGEN_CTR_REG = Pump_AL_BIT_CENTER;
            Pump_AL_TXBITCLKTX_COMPLETE_REG = (Pump_AL_NUMBER_OF_DATA_BITS +
                        Pump_AL_NUMBER_OF_START_BIT) * Pump_AL_OVER_SAMPLE_COUNT;
        #else
            Pump_AL_TXBITCTR_PERIOD_REG = ((Pump_AL_NUMBER_OF_DATA_BITS +
                        Pump_AL_NUMBER_OF_START_BIT) * Pump_AL_OVER_SAMPLE_8) - 1u;
        #endif /* End Pump_AL_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Pump_AL_TX_INTERRUPT_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            Pump_AL_TXSTATUS_MASK_REG = Pump_AL_TX_STS_FIFO_EMPTY;
        #else
            Pump_AL_TXSTATUS_MASK_REG = Pump_AL_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Pump_AL_TX_INTERRUPT_ENABLED*/

    #endif /* End Pump_AL_TX_ENABLED */

    #if(Pump_AL_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Pump_AL_WriteControlRegister( \
            (Pump_AL_ReadControlRegister() & (uint8)~Pump_AL_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Pump_AL_PARITY_TYPE << Pump_AL_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Pump_AL_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Pump_AL_Enable
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
*  Pump_AL_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Pump_AL_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Pump_AL_RXBITCTR_CONTROL_REG |= Pump_AL_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Pump_AL_RXSTATUS_ACTL_REG  |= Pump_AL_INT_ENABLE;
        #if(Pump_AL_RX_INTERRUPT_ENABLED && (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            CyIntEnable(Pump_AL_RX_VECT_NUM);
            #if (Pump_AL_RXHW_ADDRESS_ENABLED)
                Pump_AL_rxAddressDetected = 0u;
            #endif /* End Pump_AL_RXHW_ADDRESS_ENABLED */
        #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */
    #endif /* End Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED*/

    #if(Pump_AL_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Pump_AL_TXCLKGEN_DP)
            Pump_AL_TXBITCTR_CONTROL_REG |= Pump_AL_CNTR_ENABLE;
        #endif /* End Pump_AL_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Pump_AL_TXSTATUS_ACTL_REG |= Pump_AL_INT_ENABLE;
        #if(Pump_AL_TX_INTERRUPT_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            CyIntEnable(Pump_AL_TX_VECT_NUM);
        #endif /* End Pump_AL_TX_INTERRUPT_ENABLED*/
     #endif /* End Pump_AL_TX_ENABLED */

    #if(Pump_AL_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Pump_AL_IntClock_Start();
    #endif /* End Pump_AL_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Pump_AL_Stop
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
void Pump_AL_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED)
        Pump_AL_RXBITCTR_CONTROL_REG &= (uint8)~Pump_AL_CNTR_ENABLE;
    #endif /* End Pump_AL_RX_ENABLED */

    #if(Pump_AL_TX_ENABLED)
        #if(!Pump_AL_TXCLKGEN_DP)
            Pump_AL_TXBITCTR_CONTROL_REG &= (uint8)~Pump_AL_CNTR_ENABLE;
        #endif /* End Pump_AL_TXCLKGEN_DP */
    #endif /* Pump_AL_TX_ENABLED */

    #if(Pump_AL_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Pump_AL_IntClock_Stop();
    #endif /* End Pump_AL_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED)
        Pump_AL_RXSTATUS_ACTL_REG  &= (uint8)~Pump_AL_INT_ENABLE;
        #if(Pump_AL_RX_INTERRUPT_ENABLED && (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            Pump_AL_DisableRxInt();
        #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */
    #endif /* End Pump_AL_RX_ENABLED */

    #if(Pump_AL_TX_ENABLED)
        Pump_AL_TXSTATUS_ACTL_REG &= (uint8)~Pump_AL_INT_ENABLE;
        #if(Pump_AL_TX_INTERRUPT_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            Pump_AL_DisableTxInt();
        #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */
    #endif /* End Pump_AL_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Pump_AL_ReadControlRegister
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
uint8 Pump_AL_ReadControlRegister(void) 
{
    #if( Pump_AL_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Pump_AL_CONTROL_REG);
    #endif /* End Pump_AL_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Pump_AL_WriteControlRegister
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
void  Pump_AL_WriteControlRegister(uint8 control) 
{
    #if( Pump_AL_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Pump_AL_CONTROL_REG = control;
    #endif /* End Pump_AL_CONTROL_REG_REMOVED */
}


#if(Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED)

    #if(Pump_AL_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Pump_AL_EnableRxInt
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
        void Pump_AL_EnableRxInt(void) 
        {
            CyIntEnable(Pump_AL_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Pump_AL_DisableRxInt
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
        void Pump_AL_DisableRxInt(void) 
        {
            CyIntDisable(Pump_AL_RX_VECT_NUM);
        }

    #endif /* Pump_AL_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Pump_AL_SetRxInterruptMode
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
    void Pump_AL_SetRxInterruptMode(uint8 intSrc) 
    {
        Pump_AL_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Pump_AL_ReadRxData
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
    *  Pump_AL_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_AL_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Pump_AL_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Pump_AL_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Pump_AL_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_DisableRxInt();
            #endif /* Pump_AL_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Pump_AL_rxBufferRead;
            loc_rxBufferWrite = Pump_AL_rxBufferWrite;

            if( (Pump_AL_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Pump_AL_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Pump_AL_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Pump_AL_rxBufferRead = loc_rxBufferRead;

                if(Pump_AL_rxBufferLoopDetect != 0u )
                {
                    Pump_AL_rxBufferLoopDetect = 0u;
                    #if( (Pump_AL_RX_INTERRUPT_ENABLED) && (Pump_AL_FLOW_CONTROL != 0u) && \
                         (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Pump_AL_HD_ENABLED )
                            if((Pump_AL_CONTROL_REG & Pump_AL_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Pump_AL_RXSTATUS_MASK_REG  |= Pump_AL_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Pump_AL_RXSTATUS_MASK_REG  |= Pump_AL_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Pump_AL_HD_ENABLED */
                    #endif /* Pump_AL_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Pump_AL_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_EnableRxInt();
            #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */

        #else /* Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Pump_AL_RXDATA_REG;

        #endif /* Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Pump_AL_ReadRxStatus
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
    *  Pump_AL_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Pump_AL_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Pump_AL_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Pump_AL_ReadRxStatus(void) 
    {
        uint8 status;

        status = Pump_AL_RXSTATUS_REG & Pump_AL_RX_HW_MASK;

        #if(Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH)
            if( Pump_AL_rxBufferOverflow != 0u )
            {
                status |= Pump_AL_RX_STS_SOFT_BUFF_OVER;
                Pump_AL_rxBufferOverflow = 0u;
            }
        #endif /* Pump_AL_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Pump_AL_GetChar
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
    *  Pump_AL_rxBuffer - RAM buffer pointer for save received data.
    *  Pump_AL_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Pump_AL_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Pump_AL_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Pump_AL_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH)
            uint16 loc_rxBufferRead;
            uint16 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_DisableRxInt();
            #endif /* Pump_AL_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Pump_AL_rxBufferRead;
            loc_rxBufferWrite = Pump_AL_rxBufferWrite;

            if( (Pump_AL_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Pump_AL_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Pump_AL_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Pump_AL_rxBufferRead = loc_rxBufferRead;

                if(Pump_AL_rxBufferLoopDetect > 0u )
                {
                    Pump_AL_rxBufferLoopDetect = 0u;
                    #if( (Pump_AL_RX_INTERRUPT_ENABLED) && (Pump_AL_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Pump_AL_HD_ENABLED )
                            if((Pump_AL_CONTROL_REG & Pump_AL_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Pump_AL_RXSTATUS_MASK_REG  |= Pump_AL_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Pump_AL_RXSTATUS_MASK_REG  |= Pump_AL_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Pump_AL_HD_ENABLED */
                    #endif /* Pump_AL_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Pump_AL_RXSTATUS_REG;
                if((rxStatus & Pump_AL_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Pump_AL_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Pump_AL_RX_STS_BREAK | Pump_AL_RX_STS_PAR_ERROR |
                                   Pump_AL_RX_STS_STOP_ERROR | Pump_AL_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_EnableRxInt();
            #endif /* Pump_AL_RX_INTERRUPT_ENABLED */

        #else /* Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

            rxStatus =Pump_AL_RXSTATUS_REG;
            if((rxStatus & Pump_AL_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Pump_AL_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Pump_AL_RX_STS_BREAK | Pump_AL_RX_STS_PAR_ERROR |
                               Pump_AL_RX_STS_STOP_ERROR | Pump_AL_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Pump_AL_GetByte
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
    uint16 Pump_AL_GetByte(void) 
    {
        return ( ((uint16)Pump_AL_ReadRxStatus() << 8u) | Pump_AL_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Pump_AL_GetRxBufferSize
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
    *  Pump_AL_rxBufferWrite - used to calculate left bytes.
    *  Pump_AL_rxBufferRead - used to calculate left bytes.
    *  Pump_AL_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint16 Pump_AL_GetRxBufferSize(void)
                                                            
    {
        uint16 size;

        #if(Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_DisableRxInt();
            #endif /* Pump_AL_RX_INTERRUPT_ENABLED */

            if(Pump_AL_rxBufferRead == Pump_AL_rxBufferWrite)
            {
                if(Pump_AL_rxBufferLoopDetect > 0u)
                {
                    size = Pump_AL_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Pump_AL_rxBufferRead < Pump_AL_rxBufferWrite)
            {
                size = (Pump_AL_rxBufferWrite - Pump_AL_rxBufferRead);
            }
            else
            {
                size = (Pump_AL_RXBUFFERSIZE - Pump_AL_rxBufferRead) + Pump_AL_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_EnableRxInt();
            #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */

        #else /* Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Pump_AL_RXSTATUS_REG & Pump_AL_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Pump_AL_ClearRxBuffer
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
    *  Pump_AL_rxBufferWrite - cleared to zero.
    *  Pump_AL_rxBufferRead - cleared to zero.
    *  Pump_AL_rxBufferLoopDetect - cleared to zero.
    *  Pump_AL_rxBufferOverflow - cleared to zero.
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
    void Pump_AL_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Pump_AL_RXDATA_AUX_CTL_REG |=  Pump_AL_RX_FIFO_CLR;
        Pump_AL_RXDATA_AUX_CTL_REG &= (uint8)~Pump_AL_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_DisableRxInt();
            #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */

            Pump_AL_rxBufferRead = 0u;
            Pump_AL_rxBufferWrite = 0u;
            Pump_AL_rxBufferLoopDetect = 0u;
            Pump_AL_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Pump_AL_RX_INTERRUPT_ENABLED)
                Pump_AL_EnableRxInt();
            #endif /* End Pump_AL_RX_INTERRUPT_ENABLED */
        #endif /* End Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Pump_AL_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Pump_AL__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Pump_AL__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Pump_AL__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Pump_AL__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Pump_AL__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Pump_AL_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Pump_AL_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Pump_AL_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Pump_AL_RXHW_ADDRESS_ENABLED)
            #if(Pump_AL_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Pump_AL_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Pump_AL_CONTROL_REG & (uint8)~Pump_AL_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Pump_AL_CTRL_RXADDR_MODE0_SHIFT);
                Pump_AL_CONTROL_REG = tmpCtrl;
                #if(Pump_AL_RX_INTERRUPT_ENABLED && \
                   (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH) )
                    Pump_AL_rxAddressMode = addressMode;
                    Pump_AL_rxAddressDetected = 0u;
                #endif /* End Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH*/
            #endif /* End Pump_AL_CONTROL_REG_REMOVED */
        #else /* Pump_AL_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Pump_AL_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Pump_AL_SetRxAddress1
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
    void Pump_AL_SetRxAddress1(uint8 address) 

    {
        Pump_AL_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Pump_AL_SetRxAddress2
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
    void Pump_AL_SetRxAddress2(uint8 address) 
    {
        Pump_AL_RXADDRESS2_REG = address;
    }

#endif  /* Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED*/


#if( (Pump_AL_TX_ENABLED) || (Pump_AL_HD_ENABLED) )

    #if(Pump_AL_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Pump_AL_EnableTxInt
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
        void Pump_AL_EnableTxInt(void) 
        {
            CyIntEnable(Pump_AL_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Pump_AL_DisableTxInt
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
        void Pump_AL_DisableTxInt(void) 
        {
            CyIntDisable(Pump_AL_TX_VECT_NUM);
        }

    #endif /* Pump_AL_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Pump_AL_SetTxInterruptMode
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
    void Pump_AL_SetTxInterruptMode(uint8 intSrc) 
    {
        Pump_AL_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Pump_AL_WriteTxData
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
    *  Pump_AL_txBuffer - RAM buffer pointer for save data for transmission
    *  Pump_AL_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Pump_AL_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Pump_AL_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pump_AL_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Pump_AL_initVar != 0u)
        {
            #if(Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Pump_AL_TX_INTERRUPT_ENABLED)
                    Pump_AL_DisableTxInt();
                #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

                if( (Pump_AL_txBufferRead == Pump_AL_txBufferWrite) &&
                    ((Pump_AL_TXSTATUS_REG & Pump_AL_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Pump_AL_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Pump_AL_txBufferWrite >= Pump_AL_TXBUFFERSIZE)
                    {
                        Pump_AL_txBufferWrite = 0u;
                    }

                    Pump_AL_txBuffer[Pump_AL_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Pump_AL_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Pump_AL_TX_INTERRUPT_ENABLED)
                    Pump_AL_EnableTxInt();
                #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

            #else /* Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Pump_AL_TXDATA_REG = txDataByte;

            #endif /* End Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Pump_AL_ReadTxStatus
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
    uint8 Pump_AL_ReadTxStatus(void) 
    {
        return(Pump_AL_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Pump_AL_PutChar
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
    *  Pump_AL_txBuffer - RAM buffer pointer for save data for transmission
    *  Pump_AL_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Pump_AL_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Pump_AL_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Pump_AL_PutChar(uint8 txDataByte) 
    {
            #if(Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Pump_AL_TXBUFFERSIZE > Pump_AL_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Pump_AL_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Pump_AL_txBufferWrite;
                    loc_txBufferRead = Pump_AL_txBufferRead;
                    #if ((Pump_AL_TXBUFFERSIZE > Pump_AL_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Pump_AL_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Pump_AL_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Pump_AL_TXSTATUS_REG & Pump_AL_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Pump_AL_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Pump_AL_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Pump_AL_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Pump_AL_TXBUFFERSIZE > Pump_AL_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Pump_AL_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Pump_AL_txBufferWrite = loc_txBufferWrite;
                    #if ((Pump_AL_TXBUFFERSIZE > Pump_AL_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Pump_AL_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

                while((Pump_AL_TXSTATUS_REG & Pump_AL_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Pump_AL_TXDATA_REG = txDataByte;

            #endif /* End Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Pump_AL_PutString
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
    *  Pump_AL_initVar - checked to identify that the component has been
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
    void Pump_AL_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Pump_AL_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Pump_AL_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Pump_AL_PutArray
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
    *  Pump_AL_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pump_AL_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Pump_AL_initVar != 0u)
        {
            do
            {
                Pump_AL_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Pump_AL_PutCRLF
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
    *  Pump_AL_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Pump_AL_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Pump_AL_initVar != 0u)
        {
            Pump_AL_PutChar(txDataByte);
            Pump_AL_PutChar(0x0Du);
            Pump_AL_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Pump_AL_GetTxBufferSize
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
    *  Pump_AL_txBufferWrite - used to calculate left space.
    *  Pump_AL_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Pump_AL_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_AL_TX_INTERRUPT_ENABLED)
                Pump_AL_DisableTxInt();
            #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

            if(Pump_AL_txBufferRead == Pump_AL_txBufferWrite)
            {
                size = 0u;
            }
            else if(Pump_AL_txBufferRead < Pump_AL_txBufferWrite)
            {
                size = (Pump_AL_txBufferWrite - Pump_AL_txBufferRead);
            }
            else
            {
                size = (Pump_AL_TXBUFFERSIZE - Pump_AL_txBufferRead) + Pump_AL_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Pump_AL_TX_INTERRUPT_ENABLED)
                Pump_AL_EnableTxInt();
            #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

        #else /* Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

            size = Pump_AL_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Pump_AL_TX_STS_FIFO_FULL) != 0u)
            {
                size = Pump_AL_FIFO_LENGTH;
            }
            else if((size & Pump_AL_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Pump_AL_ClearTxBuffer
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
    *  Pump_AL_txBufferWrite - cleared to zero.
    *  Pump_AL_txBufferRead - cleared to zero.
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
    void Pump_AL_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Pump_AL_TXDATA_AUX_CTL_REG |=  Pump_AL_TX_FIFO_CLR;
        Pump_AL_TXDATA_AUX_CTL_REG &= (uint8)~Pump_AL_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Pump_AL_TX_INTERRUPT_ENABLED)
                Pump_AL_DisableTxInt();
            #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

            Pump_AL_txBufferRead = 0u;
            Pump_AL_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Pump_AL_TX_INTERRUPT_ENABLED)
                Pump_AL_EnableTxInt();
            #endif /* End Pump_AL_TX_INTERRUPT_ENABLED */

        #endif /* End Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Pump_AL_SendBreak
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
    *  Pump_AL_initVar - checked to identify that the component has been
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
    void Pump_AL_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Pump_AL_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Pump_AL_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Pump_AL_SEND_BREAK) ||
                    (retMode == Pump_AL_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() |
                                                          Pump_AL_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Pump_AL_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Pump_AL_TXSTATUS_REG;
                    }while((tmpStat & Pump_AL_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Pump_AL_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_AL_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Pump_AL_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Pump_AL_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Pump_AL_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_AL_REINIT) ||
                    (retMode == Pump_AL_SEND_WAIT_REINIT) )
                {
                    Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() &
                                                  (uint8)~Pump_AL_CTRL_HD_SEND_BREAK);
                }

            #else /* Pump_AL_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Pump_AL_SEND_BREAK) ||
                    (retMode == Pump_AL_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Pump_AL_PARITY_TYPE != Pump_AL__B_UART__NONE_REVB) || \
                                        (Pump_AL_PARITY_TYPE_SW != 0u) )
                        Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() |
                                                              Pump_AL_CTRL_HD_SEND_BREAK);
                    #endif /* End Pump_AL_PARITY_TYPE != Pump_AL__B_UART__NONE_REVB  */

                    #if(Pump_AL_TXCLKGEN_DP)
                        tx_period = Pump_AL_TXBITCLKTX_COMPLETE_REG;
                        Pump_AL_TXBITCLKTX_COMPLETE_REG = Pump_AL_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Pump_AL_TXBITCTR_PERIOD_REG;
                        Pump_AL_TXBITCTR_PERIOD_REG = Pump_AL_TXBITCTR_BREAKBITS8X;
                    #endif /* End Pump_AL_TXCLKGEN_DP */

                    /* Send zeros*/
                    Pump_AL_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Pump_AL_TXSTATUS_REG;
                    }while((tmpStat & Pump_AL_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Pump_AL_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_AL_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Pump_AL_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Pump_AL_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Pump_AL_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Pump_AL_REINIT) ||
                    (retMode == Pump_AL_SEND_WAIT_REINIT) )
                {

                    #if(Pump_AL_TXCLKGEN_DP)
                        Pump_AL_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Pump_AL_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Pump_AL_TXCLKGEN_DP */

                    #if( (Pump_AL_PARITY_TYPE != Pump_AL__B_UART__NONE_REVB) || \
                         (Pump_AL_PARITY_TYPE_SW != 0u) )
                        Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() &
                                                      (uint8)~Pump_AL_CTRL_HD_SEND_BREAK);
                    #endif /* End Pump_AL_PARITY_TYPE != NONE */
                }
            #endif    /* End Pump_AL_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Pump_AL_SetTxAddressMode
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
    void Pump_AL_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Pump_AL_CONTROL_REG_REMOVED == 0u )
                Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() |
                                                      Pump_AL_CTRL_MARK);
            #endif /* End Pump_AL_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Pump_AL_CONTROL_REG_REMOVED == 0u )
                Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() &
                                                    (uint8)~Pump_AL_CTRL_MARK);
            #endif /* End Pump_AL_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndPump_AL_TX_ENABLED */

#if(Pump_AL_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Pump_AL_LoadTxConfig
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
    void Pump_AL_LoadTxConfig(void) 
    {
        #if((Pump_AL_RX_INTERRUPT_ENABLED) && (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Pump_AL_SetRxInterruptMode(0u);
        #endif /* Pump_AL_RX_INTERRUPT_ENABLED */

        Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() | Pump_AL_CTRL_HD_SEND);
        Pump_AL_RXBITCTR_PERIOD_REG = Pump_AL_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Pump_AL_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Pump_AL_LoadRxConfig
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
    void Pump_AL_LoadRxConfig(void) 
    {
        Pump_AL_WriteControlRegister(Pump_AL_ReadControlRegister() &
                                                (uint8)~Pump_AL_CTRL_HD_SEND);
        Pump_AL_RXBITCTR_PERIOD_REG = Pump_AL_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Pump_AL_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Pump_AL_RX_INTERRUPT_ENABLED) && (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Pump_AL_SetRxInterruptMode(Pump_AL_INIT_RX_INTERRUPTS_MASK);
        #endif /* Pump_AL_RX_INTERRUPT_ENABLED */
    }

#endif  /* Pump_AL_HD_ENABLED */


/* [] END OF FILE */
