/*******************************************************************************
* File Name: LCD2.c
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

#include "LCD2.h"
#include "CyLib.h"
#if(LCD2_INTERNAL_CLOCK_USED)
    #include "LCD2_IntClock.h"
#endif /* End LCD2_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 LCD2_initVar = 0u;
#if( LCD2_TX_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH))
    volatile uint8 LCD2_txBuffer[LCD2_TXBUFFERSIZE];
    volatile uint8 LCD2_txBufferRead = 0u;
    uint8 LCD2_txBufferWrite = 0u;
#endif /* End LCD2_TX_ENABLED */
#if( ( LCD2_RX_ENABLED || LCD2_HD_ENABLED ) && \
     (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH) )
    volatile uint8 LCD2_rxBuffer[LCD2_RXBUFFERSIZE];
    volatile uint8 LCD2_rxBufferRead = 0u;
    volatile uint8 LCD2_rxBufferWrite = 0u;
    volatile uint8 LCD2_rxBufferLoopDetect = 0u;
    volatile uint8 LCD2_rxBufferOverflow = 0u;
    #if (LCD2_RXHW_ADDRESS_ENABLED)
        volatile uint8 LCD2_rxAddressMode = LCD2_RXADDRESSMODE;
        volatile uint8 LCD2_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End LCD2_RX_ENABLED */


/*******************************************************************************
* Function Name: LCD2_Start
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
*  The LCD2_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the LCD2_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LCD2_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(LCD2_initVar == 0u)
    {
        LCD2_Init();
        LCD2_initVar = 1u;
    }
    LCD2_Enable();
}


/*******************************************************************************
* Function Name: LCD2_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  LCD2_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void LCD2_Init(void) 
{
    #if(LCD2_RX_ENABLED || LCD2_HD_ENABLED)

        #if(LCD2_RX_INTERRUPT_ENABLED && (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(LCD2_RX_VECT_NUM, &LCD2_RXISR);
            CyIntSetPriority(LCD2_RX_VECT_NUM, LCD2_RX_PRIOR_NUM);
        #endif /* End LCD2_RX_INTERRUPT_ENABLED */

        #if (LCD2_RXHW_ADDRESS_ENABLED)
            LCD2_SetRxAddressMode(LCD2_RXAddressMode);
            LCD2_SetRxAddress1(LCD2_RXHWADDRESS1);
            LCD2_SetRxAddress2(LCD2_RXHWADDRESS2);
        #endif /* End LCD2_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        LCD2_RXBITCTR_PERIOD_REG = LCD2_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        LCD2_RXSTATUS_MASK_REG  = LCD2_INIT_RX_INTERRUPTS_MASK;
    #endif /* End LCD2_RX_ENABLED || LCD2_HD_ENABLED*/

    #if(LCD2_TX_ENABLED)
        #if(LCD2_TX_INTERRUPT_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(LCD2_TX_VECT_NUM, &LCD2_TXISR);
            CyIntSetPriority(LCD2_TX_VECT_NUM, LCD2_TX_PRIOR_NUM);
        #endif /* End LCD2_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(LCD2_TXCLKGEN_DP)
            LCD2_TXBITCLKGEN_CTR_REG = LCD2_BIT_CENTER;
            LCD2_TXBITCLKTX_COMPLETE_REG = (LCD2_NUMBER_OF_DATA_BITS +
                        LCD2_NUMBER_OF_START_BIT) * LCD2_OVER_SAMPLE_COUNT;
        #else
            LCD2_TXBITCTR_PERIOD_REG = ((LCD2_NUMBER_OF_DATA_BITS +
                        LCD2_NUMBER_OF_START_BIT) * LCD2_OVER_SAMPLE_8) - 1u;
        #endif /* End LCD2_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(LCD2_TX_INTERRUPT_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH))
            LCD2_TXSTATUS_MASK_REG = LCD2_TX_STS_FIFO_EMPTY;
        #else
            LCD2_TXSTATUS_MASK_REG = LCD2_INIT_TX_INTERRUPTS_MASK;
        #endif /*End LCD2_TX_INTERRUPT_ENABLED*/

    #endif /* End LCD2_TX_ENABLED */

    #if(LCD2_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        LCD2_WriteControlRegister( \
            (LCD2_ReadControlRegister() & (uint8)~LCD2_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(LCD2_PARITY_TYPE << LCD2_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End LCD2_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: LCD2_Enable
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
*  LCD2_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void LCD2_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(LCD2_RX_ENABLED || LCD2_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        LCD2_RXBITCTR_CONTROL_REG |= LCD2_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        LCD2_RXSTATUS_ACTL_REG  |= LCD2_INT_ENABLE;
        #if(LCD2_RX_INTERRUPT_ENABLED && (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH))
            CyIntEnable(LCD2_RX_VECT_NUM);
            #if (LCD2_RXHW_ADDRESS_ENABLED)
                LCD2_rxAddressDetected = 0u;
            #endif /* End LCD2_RXHW_ADDRESS_ENABLED */
        #endif /* End LCD2_RX_INTERRUPT_ENABLED */
    #endif /* End LCD2_RX_ENABLED || LCD2_HD_ENABLED*/

    #if(LCD2_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!LCD2_TXCLKGEN_DP)
            LCD2_TXBITCTR_CONTROL_REG |= LCD2_CNTR_ENABLE;
        #endif /* End LCD2_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        LCD2_TXSTATUS_ACTL_REG |= LCD2_INT_ENABLE;
        #if(LCD2_TX_INTERRUPT_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH))
            CyIntEnable(LCD2_TX_VECT_NUM);
        #endif /* End LCD2_TX_INTERRUPT_ENABLED*/
     #endif /* End LCD2_TX_ENABLED */

    #if(LCD2_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        LCD2_IntClock_Start();
    #endif /* End LCD2_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: LCD2_Stop
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
void LCD2_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(LCD2_RX_ENABLED || LCD2_HD_ENABLED)
        LCD2_RXBITCTR_CONTROL_REG &= (uint8)~LCD2_CNTR_ENABLE;
    #endif /* End LCD2_RX_ENABLED */

    #if(LCD2_TX_ENABLED)
        #if(!LCD2_TXCLKGEN_DP)
            LCD2_TXBITCTR_CONTROL_REG &= (uint8)~LCD2_CNTR_ENABLE;
        #endif /* End LCD2_TXCLKGEN_DP */
    #endif /* LCD2_TX_ENABLED */

    #if(LCD2_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        LCD2_IntClock_Stop();
    #endif /* End LCD2_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(LCD2_RX_ENABLED || LCD2_HD_ENABLED)
        LCD2_RXSTATUS_ACTL_REG  &= (uint8)~LCD2_INT_ENABLE;
        #if(LCD2_RX_INTERRUPT_ENABLED && (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH))
            LCD2_DisableRxInt();
        #endif /* End LCD2_RX_INTERRUPT_ENABLED */
    #endif /* End LCD2_RX_ENABLED */

    #if(LCD2_TX_ENABLED)
        LCD2_TXSTATUS_ACTL_REG &= (uint8)~LCD2_INT_ENABLE;
        #if(LCD2_TX_INTERRUPT_ENABLED && (LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH))
            LCD2_DisableTxInt();
        #endif /* End LCD2_TX_INTERRUPT_ENABLED */
    #endif /* End LCD2_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: LCD2_ReadControlRegister
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
uint8 LCD2_ReadControlRegister(void) 
{
    #if( LCD2_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(LCD2_CONTROL_REG);
    #endif /* End LCD2_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: LCD2_WriteControlRegister
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
void  LCD2_WriteControlRegister(uint8 control) 
{
    #if( LCD2_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       LCD2_CONTROL_REG = control;
    #endif /* End LCD2_CONTROL_REG_REMOVED */
}


#if(LCD2_RX_ENABLED || LCD2_HD_ENABLED)

    #if(LCD2_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: LCD2_EnableRxInt
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
        void LCD2_EnableRxInt(void) 
        {
            CyIntEnable(LCD2_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: LCD2_DisableRxInt
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
        void LCD2_DisableRxInt(void) 
        {
            CyIntDisable(LCD2_RX_VECT_NUM);
        }

    #endif /* LCD2_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: LCD2_SetRxInterruptMode
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
    void LCD2_SetRxInterruptMode(uint8 intSrc) 
    {
        LCD2_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: LCD2_ReadRxData
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
    *  LCD2_rxBuffer - RAM buffer pointer for save received data.
    *  LCD2_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  LCD2_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  LCD2_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 LCD2_ReadRxData(void) 
    {
        uint8 rxData;

        #if(LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_DisableRxInt();
            #endif /* LCD2_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = LCD2_rxBufferRead;
            loc_rxBufferWrite = LCD2_rxBufferWrite;

            if( (LCD2_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = LCD2_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= LCD2_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                LCD2_rxBufferRead = loc_rxBufferRead;

                if(LCD2_rxBufferLoopDetect != 0u )
                {
                    LCD2_rxBufferLoopDetect = 0u;
                    #if( (LCD2_RX_INTERRUPT_ENABLED) && (LCD2_FLOW_CONTROL != 0u) && \
                         (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( LCD2_HD_ENABLED )
                            if((LCD2_CONTROL_REG & LCD2_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                LCD2_RXSTATUS_MASK_REG  |= LCD2_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            LCD2_RXSTATUS_MASK_REG  |= LCD2_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end LCD2_HD_ENABLED */
                    #endif /* LCD2_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = LCD2_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_EnableRxInt();
            #endif /* End LCD2_RX_INTERRUPT_ENABLED */

        #else /* LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = LCD2_RXDATA_REG;

        #endif /* LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: LCD2_ReadRxStatus
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
    *  LCD2_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   LCD2_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   LCD2_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 LCD2_ReadRxStatus(void) 
    {
        uint8 status;

        status = LCD2_RXSTATUS_REG & LCD2_RX_HW_MASK;

        #if(LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH)
            if( LCD2_rxBufferOverflow != 0u )
            {
                status |= LCD2_RX_STS_SOFT_BUFF_OVER;
                LCD2_rxBufferOverflow = 0u;
            }
        #endif /* LCD2_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: LCD2_GetChar
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
    *  LCD2_rxBuffer - RAM buffer pointer for save received data.
    *  LCD2_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  LCD2_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  LCD2_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 LCD2_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_DisableRxInt();
            #endif /* LCD2_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = LCD2_rxBufferRead;
            loc_rxBufferWrite = LCD2_rxBufferWrite;

            if( (LCD2_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = LCD2_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= LCD2_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                LCD2_rxBufferRead = loc_rxBufferRead;

                if(LCD2_rxBufferLoopDetect > 0u )
                {
                    LCD2_rxBufferLoopDetect = 0u;
                    #if( (LCD2_RX_INTERRUPT_ENABLED) && (LCD2_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( LCD2_HD_ENABLED )
                            if((LCD2_CONTROL_REG & LCD2_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                LCD2_RXSTATUS_MASK_REG  |= LCD2_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            LCD2_RXSTATUS_MASK_REG  |= LCD2_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end LCD2_HD_ENABLED */
                    #endif /* LCD2_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = LCD2_RXSTATUS_REG;
                if((rxStatus & LCD2_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = LCD2_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (LCD2_RX_STS_BREAK | LCD2_RX_STS_PAR_ERROR |
                                   LCD2_RX_STS_STOP_ERROR | LCD2_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_EnableRxInt();
            #endif /* LCD2_RX_INTERRUPT_ENABLED */

        #else /* LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

            rxStatus =LCD2_RXSTATUS_REG;
            if((rxStatus & LCD2_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = LCD2_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (LCD2_RX_STS_BREAK | LCD2_RX_STS_PAR_ERROR |
                               LCD2_RX_STS_STOP_ERROR | LCD2_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: LCD2_GetByte
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
    uint16 LCD2_GetByte(void) 
    {
        return ( ((uint16)LCD2_ReadRxStatus() << 8u) | LCD2_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: LCD2_GetRxBufferSize
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
    *  LCD2_rxBufferWrite - used to calculate left bytes.
    *  LCD2_rxBufferRead - used to calculate left bytes.
    *  LCD2_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 LCD2_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_DisableRxInt();
            #endif /* LCD2_RX_INTERRUPT_ENABLED */

            if(LCD2_rxBufferRead == LCD2_rxBufferWrite)
            {
                if(LCD2_rxBufferLoopDetect > 0u)
                {
                    size = LCD2_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(LCD2_rxBufferRead < LCD2_rxBufferWrite)
            {
                size = (LCD2_rxBufferWrite - LCD2_rxBufferRead);
            }
            else
            {
                size = (LCD2_RXBUFFERSIZE - LCD2_rxBufferRead) + LCD2_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_EnableRxInt();
            #endif /* End LCD2_RX_INTERRUPT_ENABLED */

        #else /* LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((LCD2_RXSTATUS_REG & LCD2_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: LCD2_ClearRxBuffer
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
    *  LCD2_rxBufferWrite - cleared to zero.
    *  LCD2_rxBufferRead - cleared to zero.
    *  LCD2_rxBufferLoopDetect - cleared to zero.
    *  LCD2_rxBufferOverflow - cleared to zero.
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
    void LCD2_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        LCD2_RXDATA_AUX_CTL_REG |=  LCD2_RX_FIFO_CLR;
        LCD2_RXDATA_AUX_CTL_REG &= (uint8)~LCD2_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_DisableRxInt();
            #endif /* End LCD2_RX_INTERRUPT_ENABLED */

            LCD2_rxBufferRead = 0u;
            LCD2_rxBufferWrite = 0u;
            LCD2_rxBufferLoopDetect = 0u;
            LCD2_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(LCD2_RX_INTERRUPT_ENABLED)
                LCD2_EnableRxInt();
            #endif /* End LCD2_RX_INTERRUPT_ENABLED */
        #endif /* End LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: LCD2_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  LCD2__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  LCD2__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  LCD2__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  LCD2__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  LCD2__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  LCD2_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  LCD2_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void LCD2_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(LCD2_RXHW_ADDRESS_ENABLED)
            #if(LCD2_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* LCD2_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = LCD2_CONTROL_REG & (uint8)~LCD2_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << LCD2_CTRL_RXADDR_MODE0_SHIFT);
                LCD2_CONTROL_REG = tmpCtrl;
                #if(LCD2_RX_INTERRUPT_ENABLED && \
                   (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH) )
                    LCD2_rxAddressMode = addressMode;
                    LCD2_rxAddressDetected = 0u;
                #endif /* End LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH*/
            #endif /* End LCD2_CONTROL_REG_REMOVED */
        #else /* LCD2_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End LCD2_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: LCD2_SetRxAddress1
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
    void LCD2_SetRxAddress1(uint8 address) 

    {
        LCD2_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: LCD2_SetRxAddress2
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
    void LCD2_SetRxAddress2(uint8 address) 
    {
        LCD2_RXADDRESS2_REG = address;
    }

#endif  /* LCD2_RX_ENABLED || LCD2_HD_ENABLED*/


#if( (LCD2_TX_ENABLED) || (LCD2_HD_ENABLED) )

    #if(LCD2_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: LCD2_EnableTxInt
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
        void LCD2_EnableTxInt(void) 
        {
            CyIntEnable(LCD2_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: LCD2_DisableTxInt
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
        void LCD2_DisableTxInt(void) 
        {
            CyIntDisable(LCD2_TX_VECT_NUM);
        }

    #endif /* LCD2_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: LCD2_SetTxInterruptMode
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
    void LCD2_SetTxInterruptMode(uint8 intSrc) 
    {
        LCD2_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: LCD2_WriteTxData
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
    *  LCD2_txBuffer - RAM buffer pointer for save data for transmission
    *  LCD2_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  LCD2_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  LCD2_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void LCD2_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(LCD2_initVar != 0u)
        {
            #if(LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(LCD2_TX_INTERRUPT_ENABLED)
                    LCD2_DisableTxInt();
                #endif /* End LCD2_TX_INTERRUPT_ENABLED */

                if( (LCD2_txBufferRead == LCD2_txBufferWrite) &&
                    ((LCD2_TXSTATUS_REG & LCD2_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    LCD2_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(LCD2_txBufferWrite >= LCD2_TXBUFFERSIZE)
                    {
                        LCD2_txBufferWrite = 0u;
                    }

                    LCD2_txBuffer[LCD2_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    LCD2_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(LCD2_TX_INTERRUPT_ENABLED)
                    LCD2_EnableTxInt();
                #endif /* End LCD2_TX_INTERRUPT_ENABLED */

            #else /* LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                LCD2_TXDATA_REG = txDataByte;

            #endif /* End LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: LCD2_ReadTxStatus
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
    uint8 LCD2_ReadTxStatus(void) 
    {
        return(LCD2_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: LCD2_PutChar
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
    *  LCD2_txBuffer - RAM buffer pointer for save data for transmission
    *  LCD2_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  LCD2_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  LCD2_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void LCD2_PutChar(uint8 txDataByte) 
    {
            #if(LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((LCD2_TXBUFFERSIZE > LCD2_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(LCD2_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = LCD2_txBufferWrite;
                    loc_txBufferRead = LCD2_txBufferRead;
                    #if ((LCD2_TXBUFFERSIZE > LCD2_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(LCD2_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(LCD2_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((LCD2_TXSTATUS_REG & LCD2_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    LCD2_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= LCD2_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    LCD2_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((LCD2_TXBUFFERSIZE > LCD2_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(LCD2_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    LCD2_txBufferWrite = loc_txBufferWrite;
                    #if ((LCD2_TXBUFFERSIZE > LCD2_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(LCD2_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */

                while((LCD2_TXSTATUS_REG & LCD2_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                LCD2_TXDATA_REG = txDataByte;

            #endif /* End LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: LCD2_PutString
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
    *  LCD2_initVar - checked to identify that the component has been
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
    void LCD2_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(LCD2_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                LCD2_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: LCD2_PutArray
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
    *  LCD2_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void LCD2_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(LCD2_initVar != 0u)
        {
            do
            {
                LCD2_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: LCD2_PutCRLF
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
    *  LCD2_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void LCD2_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(LCD2_initVar != 0u)
        {
            LCD2_PutChar(txDataByte);
            LCD2_PutChar(0x0Du);
            LCD2_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: LCD2_GetTxBufferSize
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
    *  LCD2_txBufferWrite - used to calculate left space.
    *  LCD2_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 LCD2_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(LCD2_TX_INTERRUPT_ENABLED)
                LCD2_DisableTxInt();
            #endif /* End LCD2_TX_INTERRUPT_ENABLED */

            if(LCD2_txBufferRead == LCD2_txBufferWrite)
            {
                size = 0u;
            }
            else if(LCD2_txBufferRead < LCD2_txBufferWrite)
            {
                size = (LCD2_txBufferWrite - LCD2_txBufferRead);
            }
            else
            {
                size = (LCD2_TXBUFFERSIZE - LCD2_txBufferRead) + LCD2_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(LCD2_TX_INTERRUPT_ENABLED)
                LCD2_EnableTxInt();
            #endif /* End LCD2_TX_INTERRUPT_ENABLED */

        #else /* LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */

            size = LCD2_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & LCD2_TX_STS_FIFO_FULL) != 0u)
            {
                size = LCD2_FIFO_LENGTH;
            }
            else if((size & LCD2_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: LCD2_ClearTxBuffer
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
    *  LCD2_txBufferWrite - cleared to zero.
    *  LCD2_txBufferRead - cleared to zero.
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
    void LCD2_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        LCD2_TXDATA_AUX_CTL_REG |=  LCD2_TX_FIFO_CLR;
        LCD2_TXDATA_AUX_CTL_REG &= (uint8)~LCD2_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(LCD2_TX_INTERRUPT_ENABLED)
                LCD2_DisableTxInt();
            #endif /* End LCD2_TX_INTERRUPT_ENABLED */

            LCD2_txBufferRead = 0u;
            LCD2_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(LCD2_TX_INTERRUPT_ENABLED)
                LCD2_EnableTxInt();
            #endif /* End LCD2_TX_INTERRUPT_ENABLED */

        #endif /* End LCD2_TXBUFFERSIZE > LCD2_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: LCD2_SendBreak
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
    *  LCD2_initVar - checked to identify that the component has been
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
    void LCD2_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(LCD2_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(LCD2_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == LCD2_SEND_BREAK) ||
                    (retMode == LCD2_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    LCD2_WriteControlRegister(LCD2_ReadControlRegister() |
                                                          LCD2_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    LCD2_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = LCD2_TXSTATUS_REG;
                    }while((tmpStat & LCD2_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == LCD2_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == LCD2_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = LCD2_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & LCD2_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == LCD2_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == LCD2_REINIT) ||
                    (retMode == LCD2_SEND_WAIT_REINIT) )
                {
                    LCD2_WriteControlRegister(LCD2_ReadControlRegister() &
                                                  (uint8)~LCD2_CTRL_HD_SEND_BREAK);
                }

            #else /* LCD2_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == LCD2_SEND_BREAK) ||
                    (retMode == LCD2_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (LCD2_PARITY_TYPE != LCD2__B_UART__NONE_REVB) || \
                                        (LCD2_PARITY_TYPE_SW != 0u) )
                        LCD2_WriteControlRegister(LCD2_ReadControlRegister() |
                                                              LCD2_CTRL_HD_SEND_BREAK);
                    #endif /* End LCD2_PARITY_TYPE != LCD2__B_UART__NONE_REVB  */

                    #if(LCD2_TXCLKGEN_DP)
                        tx_period = LCD2_TXBITCLKTX_COMPLETE_REG;
                        LCD2_TXBITCLKTX_COMPLETE_REG = LCD2_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = LCD2_TXBITCTR_PERIOD_REG;
                        LCD2_TXBITCTR_PERIOD_REG = LCD2_TXBITCTR_BREAKBITS8X;
                    #endif /* End LCD2_TXCLKGEN_DP */

                    /* Send zeros*/
                    LCD2_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = LCD2_TXSTATUS_REG;
                    }while((tmpStat & LCD2_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == LCD2_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == LCD2_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = LCD2_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & LCD2_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == LCD2_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == LCD2_REINIT) ||
                    (retMode == LCD2_SEND_WAIT_REINIT) )
                {

                    #if(LCD2_TXCLKGEN_DP)
                        LCD2_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        LCD2_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End LCD2_TXCLKGEN_DP */

                    #if( (LCD2_PARITY_TYPE != LCD2__B_UART__NONE_REVB) || \
                         (LCD2_PARITY_TYPE_SW != 0u) )
                        LCD2_WriteControlRegister(LCD2_ReadControlRegister() &
                                                      (uint8)~LCD2_CTRL_HD_SEND_BREAK);
                    #endif /* End LCD2_PARITY_TYPE != NONE */
                }
            #endif    /* End LCD2_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: LCD2_SetTxAddressMode
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
    void LCD2_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( LCD2_CONTROL_REG_REMOVED == 0u )
                LCD2_WriteControlRegister(LCD2_ReadControlRegister() |
                                                      LCD2_CTRL_MARK);
            #endif /* End LCD2_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( LCD2_CONTROL_REG_REMOVED == 0u )
                LCD2_WriteControlRegister(LCD2_ReadControlRegister() &
                                                    (uint8)~LCD2_CTRL_MARK);
            #endif /* End LCD2_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndLCD2_TX_ENABLED */

#if(LCD2_HD_ENABLED)


    /*******************************************************************************
    * Function Name: LCD2_LoadTxConfig
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
    void LCD2_LoadTxConfig(void) 
    {
        #if((LCD2_RX_INTERRUPT_ENABLED) && (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            LCD2_SetRxInterruptMode(0u);
        #endif /* LCD2_RX_INTERRUPT_ENABLED */

        LCD2_WriteControlRegister(LCD2_ReadControlRegister() | LCD2_CTRL_HD_SEND);
        LCD2_RXBITCTR_PERIOD_REG = LCD2_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(LCD2_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: LCD2_LoadRxConfig
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
    void LCD2_LoadRxConfig(void) 
    {
        LCD2_WriteControlRegister(LCD2_ReadControlRegister() &
                                                (uint8)~LCD2_CTRL_HD_SEND);
        LCD2_RXBITCTR_PERIOD_REG = LCD2_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(LCD2_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((LCD2_RX_INTERRUPT_ENABLED) && (LCD2_RXBUFFERSIZE > LCD2_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            LCD2_SetRxInterruptMode(LCD2_INIT_RX_INTERRUPTS_MASK);
        #endif /* LCD2_RX_INTERRUPT_ENABLED */
    }

#endif  /* LCD2_HD_ENABLED */


/* [] END OF FILE */
