/*******************************************************************************
* File Name: Impresora.c
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

#include "Impresora.h"
#include "CyLib.h"
#if(Impresora_INTERNAL_CLOCK_USED)
    #include "Impresora_IntClock.h"
#endif /* End Impresora_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 Impresora_initVar = 0u;
#if( Impresora_TX_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH))
    volatile uint8 Impresora_txBuffer[Impresora_TXBUFFERSIZE];
    volatile uint8 Impresora_txBufferRead = 0u;
    uint8 Impresora_txBufferWrite = 0u;
#endif /* End Impresora_TX_ENABLED */
#if( ( Impresora_RX_ENABLED || Impresora_HD_ENABLED ) && \
     (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH) )
    volatile uint8 Impresora_rxBuffer[Impresora_RXBUFFERSIZE];
    volatile uint8 Impresora_rxBufferRead = 0u;
    volatile uint8 Impresora_rxBufferWrite = 0u;
    volatile uint8 Impresora_rxBufferLoopDetect = 0u;
    volatile uint8 Impresora_rxBufferOverflow = 0u;
    #if (Impresora_RXHW_ADDRESS_ENABLED)
        volatile uint8 Impresora_rxAddressMode = Impresora_RXADDRESSMODE;
        volatile uint8 Impresora_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End Impresora_RX_ENABLED */


/*******************************************************************************
* Function Name: Impresora_Start
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
*  The Impresora_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the Impresora_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void Impresora_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(Impresora_initVar == 0u)
    {
        Impresora_Init();
        Impresora_initVar = 1u;
    }
    Impresora_Enable();
}


/*******************************************************************************
* Function Name: Impresora_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  Impresora_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void Impresora_Init(void) 
{
    #if(Impresora_RX_ENABLED || Impresora_HD_ENABLED)

        #if(Impresora_RX_INTERRUPT_ENABLED && (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(Impresora_RX_VECT_NUM, &Impresora_RXISR);
            CyIntSetPriority(Impresora_RX_VECT_NUM, Impresora_RX_PRIOR_NUM);
        #endif /* End Impresora_RX_INTERRUPT_ENABLED */

        #if (Impresora_RXHW_ADDRESS_ENABLED)
            Impresora_SetRxAddressMode(Impresora_RXAddressMode);
            Impresora_SetRxAddress1(Impresora_RXHWADDRESS1);
            Impresora_SetRxAddress2(Impresora_RXHWADDRESS2);
        #endif /* End Impresora_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        Impresora_RXBITCTR_PERIOD_REG = Impresora_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        Impresora_RXSTATUS_MASK_REG  = Impresora_INIT_RX_INTERRUPTS_MASK;
    #endif /* End Impresora_RX_ENABLED || Impresora_HD_ENABLED*/

    #if(Impresora_TX_ENABLED)
        #if(Impresora_TX_INTERRUPT_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(Impresora_TX_VECT_NUM, &Impresora_TXISR);
            CyIntSetPriority(Impresora_TX_VECT_NUM, Impresora_TX_PRIOR_NUM);
        #endif /* End Impresora_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(Impresora_TXCLKGEN_DP)
            Impresora_TXBITCLKGEN_CTR_REG = Impresora_BIT_CENTER;
            Impresora_TXBITCLKTX_COMPLETE_REG = (Impresora_NUMBER_OF_DATA_BITS +
                        Impresora_NUMBER_OF_START_BIT) * Impresora_OVER_SAMPLE_COUNT;
        #else
            Impresora_TXBITCTR_PERIOD_REG = ((Impresora_NUMBER_OF_DATA_BITS +
                        Impresora_NUMBER_OF_START_BIT) * Impresora_OVER_SAMPLE_8) - 1u;
        #endif /* End Impresora_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(Impresora_TX_INTERRUPT_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH))
            Impresora_TXSTATUS_MASK_REG = Impresora_TX_STS_FIFO_EMPTY;
        #else
            Impresora_TXSTATUS_MASK_REG = Impresora_INIT_TX_INTERRUPTS_MASK;
        #endif /*End Impresora_TX_INTERRUPT_ENABLED*/

    #endif /* End Impresora_TX_ENABLED */

    #if(Impresora_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        Impresora_WriteControlRegister( \
            (Impresora_ReadControlRegister() & (uint8)~Impresora_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(Impresora_PARITY_TYPE << Impresora_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End Impresora_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: Impresora_Enable
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
*  Impresora_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void Impresora_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(Impresora_RX_ENABLED || Impresora_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        Impresora_RXBITCTR_CONTROL_REG |= Impresora_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        Impresora_RXSTATUS_ACTL_REG  |= Impresora_INT_ENABLE;
        #if(Impresora_RX_INTERRUPT_ENABLED && (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH))
            CyIntEnable(Impresora_RX_VECT_NUM);
            #if (Impresora_RXHW_ADDRESS_ENABLED)
                Impresora_rxAddressDetected = 0u;
            #endif /* End Impresora_RXHW_ADDRESS_ENABLED */
        #endif /* End Impresora_RX_INTERRUPT_ENABLED */
    #endif /* End Impresora_RX_ENABLED || Impresora_HD_ENABLED*/

    #if(Impresora_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!Impresora_TXCLKGEN_DP)
            Impresora_TXBITCTR_CONTROL_REG |= Impresora_CNTR_ENABLE;
        #endif /* End Impresora_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        Impresora_TXSTATUS_ACTL_REG |= Impresora_INT_ENABLE;
        #if(Impresora_TX_INTERRUPT_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH))
            CyIntEnable(Impresora_TX_VECT_NUM);
        #endif /* End Impresora_TX_INTERRUPT_ENABLED*/
     #endif /* End Impresora_TX_ENABLED */

    #if(Impresora_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        Impresora_IntClock_Start();
    #endif /* End Impresora_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Impresora_Stop
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
void Impresora_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(Impresora_RX_ENABLED || Impresora_HD_ENABLED)
        Impresora_RXBITCTR_CONTROL_REG &= (uint8)~Impresora_CNTR_ENABLE;
    #endif /* End Impresora_RX_ENABLED */

    #if(Impresora_TX_ENABLED)
        #if(!Impresora_TXCLKGEN_DP)
            Impresora_TXBITCTR_CONTROL_REG &= (uint8)~Impresora_CNTR_ENABLE;
        #endif /* End Impresora_TXCLKGEN_DP */
    #endif /* Impresora_TX_ENABLED */

    #if(Impresora_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        Impresora_IntClock_Stop();
    #endif /* End Impresora_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(Impresora_RX_ENABLED || Impresora_HD_ENABLED)
        Impresora_RXSTATUS_ACTL_REG  &= (uint8)~Impresora_INT_ENABLE;
        #if(Impresora_RX_INTERRUPT_ENABLED && (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH))
            Impresora_DisableRxInt();
        #endif /* End Impresora_RX_INTERRUPT_ENABLED */
    #endif /* End Impresora_RX_ENABLED */

    #if(Impresora_TX_ENABLED)
        Impresora_TXSTATUS_ACTL_REG &= (uint8)~Impresora_INT_ENABLE;
        #if(Impresora_TX_INTERRUPT_ENABLED && (Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH))
            Impresora_DisableTxInt();
        #endif /* End Impresora_TX_INTERRUPT_ENABLED */
    #endif /* End Impresora_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: Impresora_ReadControlRegister
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
uint8 Impresora_ReadControlRegister(void) 
{
    #if( Impresora_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(Impresora_CONTROL_REG);
    #endif /* End Impresora_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: Impresora_WriteControlRegister
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
void  Impresora_WriteControlRegister(uint8 control) 
{
    #if( Impresora_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       Impresora_CONTROL_REG = control;
    #endif /* End Impresora_CONTROL_REG_REMOVED */
}


#if(Impresora_RX_ENABLED || Impresora_HD_ENABLED)

    #if(Impresora_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Impresora_EnableRxInt
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
        void Impresora_EnableRxInt(void) 
        {
            CyIntEnable(Impresora_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Impresora_DisableRxInt
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
        void Impresora_DisableRxInt(void) 
        {
            CyIntDisable(Impresora_RX_VECT_NUM);
        }

    #endif /* Impresora_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Impresora_SetRxInterruptMode
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
    void Impresora_SetRxInterruptMode(uint8 intSrc) 
    {
        Impresora_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: Impresora_ReadRxData
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
    *  Impresora_rxBuffer - RAM buffer pointer for save received data.
    *  Impresora_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Impresora_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Impresora_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Impresora_ReadRxData(void) 
    {
        uint8 rxData;

        #if(Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_DisableRxInt();
            #endif /* Impresora_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Impresora_rxBufferRead;
            loc_rxBufferWrite = Impresora_rxBufferWrite;

            if( (Impresora_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Impresora_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= Impresora_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Impresora_rxBufferRead = loc_rxBufferRead;

                if(Impresora_rxBufferLoopDetect != 0u )
                {
                    Impresora_rxBufferLoopDetect = 0u;
                    #if( (Impresora_RX_INTERRUPT_ENABLED) && (Impresora_FLOW_CONTROL != 0u) && \
                         (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Impresora_HD_ENABLED )
                            if((Impresora_CONTROL_REG & Impresora_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Impresora_RXSTATUS_MASK_REG  |= Impresora_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Impresora_RXSTATUS_MASK_REG  |= Impresora_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Impresora_HD_ENABLED */
                    #endif /* Impresora_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = Impresora_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_EnableRxInt();
            #endif /* End Impresora_RX_INTERRUPT_ENABLED */

        #else /* Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = Impresora_RXDATA_REG;

        #endif /* Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Impresora_ReadRxStatus
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
    *  Impresora_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   Impresora_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   Impresora_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 Impresora_ReadRxStatus(void) 
    {
        uint8 status;

        status = Impresora_RXSTATUS_REG & Impresora_RX_HW_MASK;

        #if(Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH)
            if( Impresora_rxBufferOverflow != 0u )
            {
                status |= Impresora_RX_STS_SOFT_BUFF_OVER;
                Impresora_rxBufferOverflow = 0u;
            }
        #endif /* Impresora_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: Impresora_GetChar
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
    *  Impresora_rxBuffer - RAM buffer pointer for save received data.
    *  Impresora_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  Impresora_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  Impresora_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 Impresora_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_DisableRxInt();
            #endif /* Impresora_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = Impresora_rxBufferRead;
            loc_rxBufferWrite = Impresora_rxBufferWrite;

            if( (Impresora_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = Impresora_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= Impresora_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                Impresora_rxBufferRead = loc_rxBufferRead;

                if(Impresora_rxBufferLoopDetect > 0u )
                {
                    Impresora_rxBufferLoopDetect = 0u;
                    #if( (Impresora_RX_INTERRUPT_ENABLED) && (Impresora_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( Impresora_HD_ENABLED )
                            if((Impresora_CONTROL_REG & Impresora_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                Impresora_RXSTATUS_MASK_REG  |= Impresora_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            Impresora_RXSTATUS_MASK_REG  |= Impresora_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end Impresora_HD_ENABLED */
                    #endif /* Impresora_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = Impresora_RXSTATUS_REG;
                if((rxStatus & Impresora_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = Impresora_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (Impresora_RX_STS_BREAK | Impresora_RX_STS_PAR_ERROR |
                                   Impresora_RX_STS_STOP_ERROR | Impresora_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_EnableRxInt();
            #endif /* Impresora_RX_INTERRUPT_ENABLED */

        #else /* Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

            rxStatus =Impresora_RXSTATUS_REG;
            if((rxStatus & Impresora_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = Impresora_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (Impresora_RX_STS_BREAK | Impresora_RX_STS_PAR_ERROR |
                               Impresora_RX_STS_STOP_ERROR | Impresora_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: Impresora_GetByte
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
    uint16 Impresora_GetByte(void) 
    {
        return ( ((uint16)Impresora_ReadRxStatus() << 8u) | Impresora_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: Impresora_GetRxBufferSize
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
    *  Impresora_rxBufferWrite - used to calculate left bytes.
    *  Impresora_rxBufferRead - used to calculate left bytes.
    *  Impresora_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 Impresora_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_DisableRxInt();
            #endif /* Impresora_RX_INTERRUPT_ENABLED */

            if(Impresora_rxBufferRead == Impresora_rxBufferWrite)
            {
                if(Impresora_rxBufferLoopDetect > 0u)
                {
                    size = Impresora_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(Impresora_rxBufferRead < Impresora_rxBufferWrite)
            {
                size = (Impresora_rxBufferWrite - Impresora_rxBufferRead);
            }
            else
            {
                size = (Impresora_RXBUFFERSIZE - Impresora_rxBufferRead) + Impresora_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_EnableRxInt();
            #endif /* End Impresora_RX_INTERRUPT_ENABLED */

        #else /* Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((Impresora_RXSTATUS_REG & Impresora_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Impresora_ClearRxBuffer
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
    *  Impresora_rxBufferWrite - cleared to zero.
    *  Impresora_rxBufferRead - cleared to zero.
    *  Impresora_rxBufferLoopDetect - cleared to zero.
    *  Impresora_rxBufferOverflow - cleared to zero.
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
    void Impresora_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        Impresora_RXDATA_AUX_CTL_REG |=  Impresora_RX_FIFO_CLR;
        Impresora_RXDATA_AUX_CTL_REG &= (uint8)~Impresora_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_DisableRxInt();
            #endif /* End Impresora_RX_INTERRUPT_ENABLED */

            Impresora_rxBufferRead = 0u;
            Impresora_rxBufferWrite = 0u;
            Impresora_rxBufferLoopDetect = 0u;
            Impresora_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(Impresora_RX_INTERRUPT_ENABLED)
                Impresora_EnableRxInt();
            #endif /* End Impresora_RX_INTERRUPT_ENABLED */
        #endif /* End Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: Impresora_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  Impresora__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  Impresora__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  Impresora__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  Impresora__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  Impresora__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  Impresora_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  Impresora_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void Impresora_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(Impresora_RXHW_ADDRESS_ENABLED)
            #if(Impresora_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* Impresora_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = Impresora_CONTROL_REG & (uint8)~Impresora_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << Impresora_CTRL_RXADDR_MODE0_SHIFT);
                Impresora_CONTROL_REG = tmpCtrl;
                #if(Impresora_RX_INTERRUPT_ENABLED && \
                   (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH) )
                    Impresora_rxAddressMode = addressMode;
                    Impresora_rxAddressDetected = 0u;
                #endif /* End Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH*/
            #endif /* End Impresora_CONTROL_REG_REMOVED */
        #else /* Impresora_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End Impresora_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: Impresora_SetRxAddress1
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
    void Impresora_SetRxAddress1(uint8 address) 

    {
        Impresora_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: Impresora_SetRxAddress2
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
    void Impresora_SetRxAddress2(uint8 address) 
    {
        Impresora_RXADDRESS2_REG = address;
    }

#endif  /* Impresora_RX_ENABLED || Impresora_HD_ENABLED*/


#if( (Impresora_TX_ENABLED) || (Impresora_HD_ENABLED) )

    #if(Impresora_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: Impresora_EnableTxInt
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
        void Impresora_EnableTxInt(void) 
        {
            CyIntEnable(Impresora_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: Impresora_DisableTxInt
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
        void Impresora_DisableTxInt(void) 
        {
            CyIntDisable(Impresora_TX_VECT_NUM);
        }

    #endif /* Impresora_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: Impresora_SetTxInterruptMode
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
    void Impresora_SetTxInterruptMode(uint8 intSrc) 
    {
        Impresora_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: Impresora_WriteTxData
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
    *  Impresora_txBuffer - RAM buffer pointer for save data for transmission
    *  Impresora_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  Impresora_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  Impresora_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Impresora_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Impresora_initVar != 0u)
        {
            #if(Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(Impresora_TX_INTERRUPT_ENABLED)
                    Impresora_DisableTxInt();
                #endif /* End Impresora_TX_INTERRUPT_ENABLED */

                if( (Impresora_txBufferRead == Impresora_txBufferWrite) &&
                    ((Impresora_TXSTATUS_REG & Impresora_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Impresora_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(Impresora_txBufferWrite >= Impresora_TXBUFFERSIZE)
                    {
                        Impresora_txBufferWrite = 0u;
                    }

                    Impresora_txBuffer[Impresora_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    Impresora_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(Impresora_TX_INTERRUPT_ENABLED)
                    Impresora_EnableTxInt();
                #endif /* End Impresora_TX_INTERRUPT_ENABLED */

            #else /* Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                Impresora_TXDATA_REG = txDataByte;

            #endif /* End Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: Impresora_ReadTxStatus
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
    uint8 Impresora_ReadTxStatus(void) 
    {
        return(Impresora_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: Impresora_PutChar
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
    *  Impresora_txBuffer - RAM buffer pointer for save data for transmission
    *  Impresora_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  Impresora_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  Impresora_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void Impresora_PutChar(uint8 txDataByte) 
    {
            #if(Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((Impresora_TXBUFFERSIZE > Impresora_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(Impresora_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = Impresora_txBufferWrite;
                    loc_txBufferRead = Impresora_txBufferRead;
                    #if ((Impresora_TXBUFFERSIZE > Impresora_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(Impresora_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(Impresora_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((Impresora_TXSTATUS_REG & Impresora_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    Impresora_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= Impresora_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    Impresora_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((Impresora_TXBUFFERSIZE > Impresora_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(Impresora_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    Impresora_txBufferWrite = loc_txBufferWrite;
                    #if ((Impresora_TXBUFFERSIZE > Impresora_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(Impresora_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */

                while((Impresora_TXSTATUS_REG & Impresora_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                Impresora_TXDATA_REG = txDataByte;

            #endif /* End Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Impresora_PutString
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
    *  Impresora_initVar - checked to identify that the component has been
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
    void Impresora_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Impresora_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                Impresora_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: Impresora_PutArray
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
    *  Impresora_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Impresora_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(Impresora_initVar != 0u)
        {
            do
            {
                Impresora_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: Impresora_PutCRLF
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
    *  Impresora_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void Impresora_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(Impresora_initVar != 0u)
        {
            Impresora_PutChar(txDataByte);
            Impresora_PutChar(0x0Du);
            Impresora_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: Impresora_GetTxBufferSize
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
    *  Impresora_txBufferWrite - used to calculate left space.
    *  Impresora_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 Impresora_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Impresora_TX_INTERRUPT_ENABLED)
                Impresora_DisableTxInt();
            #endif /* End Impresora_TX_INTERRUPT_ENABLED */

            if(Impresora_txBufferRead == Impresora_txBufferWrite)
            {
                size = 0u;
            }
            else if(Impresora_txBufferRead < Impresora_txBufferWrite)
            {
                size = (Impresora_txBufferWrite - Impresora_txBufferRead);
            }
            else
            {
                size = (Impresora_TXBUFFERSIZE - Impresora_txBufferRead) + Impresora_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(Impresora_TX_INTERRUPT_ENABLED)
                Impresora_EnableTxInt();
            #endif /* End Impresora_TX_INTERRUPT_ENABLED */

        #else /* Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */

            size = Impresora_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & Impresora_TX_STS_FIFO_FULL) != 0u)
            {
                size = Impresora_FIFO_LENGTH;
            }
            else if((size & Impresora_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: Impresora_ClearTxBuffer
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
    *  Impresora_txBufferWrite - cleared to zero.
    *  Impresora_txBufferRead - cleared to zero.
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
    void Impresora_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        Impresora_TXDATA_AUX_CTL_REG |=  Impresora_TX_FIFO_CLR;
        Impresora_TXDATA_AUX_CTL_REG &= (uint8)~Impresora_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(Impresora_TX_INTERRUPT_ENABLED)
                Impresora_DisableTxInt();
            #endif /* End Impresora_TX_INTERRUPT_ENABLED */

            Impresora_txBufferRead = 0u;
            Impresora_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(Impresora_TX_INTERRUPT_ENABLED)
                Impresora_EnableTxInt();
            #endif /* End Impresora_TX_INTERRUPT_ENABLED */

        #endif /* End Impresora_TXBUFFERSIZE > Impresora_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: Impresora_SendBreak
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
    *  Impresora_initVar - checked to identify that the component has been
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
    void Impresora_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(Impresora_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(Impresora_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == Impresora_SEND_BREAK) ||
                    (retMode == Impresora_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    Impresora_WriteControlRegister(Impresora_ReadControlRegister() |
                                                          Impresora_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    Impresora_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = Impresora_TXSTATUS_REG;
                    }while((tmpStat & Impresora_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Impresora_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Impresora_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Impresora_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Impresora_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Impresora_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Impresora_REINIT) ||
                    (retMode == Impresora_SEND_WAIT_REINIT) )
                {
                    Impresora_WriteControlRegister(Impresora_ReadControlRegister() &
                                                  (uint8)~Impresora_CTRL_HD_SEND_BREAK);
                }

            #else /* Impresora_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == Impresora_SEND_BREAK) ||
                    (retMode == Impresora_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (Impresora_PARITY_TYPE != Impresora__B_UART__NONE_REVB) || \
                                        (Impresora_PARITY_TYPE_SW != 0u) )
                        Impresora_WriteControlRegister(Impresora_ReadControlRegister() |
                                                              Impresora_CTRL_HD_SEND_BREAK);
                    #endif /* End Impresora_PARITY_TYPE != Impresora__B_UART__NONE_REVB  */

                    #if(Impresora_TXCLKGEN_DP)
                        tx_period = Impresora_TXBITCLKTX_COMPLETE_REG;
                        Impresora_TXBITCLKTX_COMPLETE_REG = Impresora_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = Impresora_TXBITCTR_PERIOD_REG;
                        Impresora_TXBITCTR_PERIOD_REG = Impresora_TXBITCTR_BREAKBITS8X;
                    #endif /* End Impresora_TXCLKGEN_DP */

                    /* Send zeros*/
                    Impresora_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = Impresora_TXSTATUS_REG;
                    }while((tmpStat & Impresora_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == Impresora_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Impresora_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = Impresora_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & Impresora_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == Impresora_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == Impresora_REINIT) ||
                    (retMode == Impresora_SEND_WAIT_REINIT) )
                {

                    #if(Impresora_TXCLKGEN_DP)
                        Impresora_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        Impresora_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End Impresora_TXCLKGEN_DP */

                    #if( (Impresora_PARITY_TYPE != Impresora__B_UART__NONE_REVB) || \
                         (Impresora_PARITY_TYPE_SW != 0u) )
                        Impresora_WriteControlRegister(Impresora_ReadControlRegister() &
                                                      (uint8)~Impresora_CTRL_HD_SEND_BREAK);
                    #endif /* End Impresora_PARITY_TYPE != NONE */
                }
            #endif    /* End Impresora_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: Impresora_SetTxAddressMode
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
    void Impresora_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( Impresora_CONTROL_REG_REMOVED == 0u )
                Impresora_WriteControlRegister(Impresora_ReadControlRegister() |
                                                      Impresora_CTRL_MARK);
            #endif /* End Impresora_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( Impresora_CONTROL_REG_REMOVED == 0u )
                Impresora_WriteControlRegister(Impresora_ReadControlRegister() &
                                                    (uint8)~Impresora_CTRL_MARK);
            #endif /* End Impresora_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndImpresora_TX_ENABLED */

#if(Impresora_HD_ENABLED)


    /*******************************************************************************
    * Function Name: Impresora_LoadTxConfig
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
    void Impresora_LoadTxConfig(void) 
    {
        #if((Impresora_RX_INTERRUPT_ENABLED) && (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            Impresora_SetRxInterruptMode(0u);
        #endif /* Impresora_RX_INTERRUPT_ENABLED */

        Impresora_WriteControlRegister(Impresora_ReadControlRegister() | Impresora_CTRL_HD_SEND);
        Impresora_RXBITCTR_PERIOD_REG = Impresora_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Impresora_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: Impresora_LoadRxConfig
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
    void Impresora_LoadRxConfig(void) 
    {
        Impresora_WriteControlRegister(Impresora_ReadControlRegister() &
                                                (uint8)~Impresora_CTRL_HD_SEND);
        Impresora_RXBITCTR_PERIOD_REG = Impresora_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(Impresora_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((Impresora_RX_INTERRUPT_ENABLED) && (Impresora_RXBUFFERSIZE > Impresora_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            Impresora_SetRxInterruptMode(Impresora_INIT_RX_INTERRUPTS_MASK);
        #endif /* Impresora_RX_INTERRUPT_ENABLED */
    }

#endif  /* Impresora_HD_ENABLED */


/* [] END OF FILE */
