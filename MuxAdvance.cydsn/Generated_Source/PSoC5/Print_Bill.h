/*******************************************************************************
* File Name: Print_Bill.h
* Version 2.30
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_Print_Bill_H)
#define CY_UART_Print_Bill_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Print_Bill_RX_ENABLED                     (1u)
#define Print_Bill_TX_ENABLED                     (1u)
#define Print_Bill_HD_ENABLED                     (0u)
#define Print_Bill_RX_INTERRUPT_ENABLED           (1u)
#define Print_Bill_TX_INTERRUPT_ENABLED           (0u)
#define Print_Bill_INTERNAL_CLOCK_USED            (1u)
#define Print_Bill_RXHW_ADDRESS_ENABLED           (0u)
#define Print_Bill_OVER_SAMPLE_COUNT              (8u)
#define Print_Bill_PARITY_TYPE                    (0u)
#define Print_Bill_PARITY_TYPE_SW                 (0u)
#define Print_Bill_BREAK_DETECT                   (0u)
#define Print_Bill_BREAK_BITS_TX                  (13u)
#define Print_Bill_BREAK_BITS_RX                  (13u)
#define Print_Bill_TXCLKGEN_DP                    (1u)
#define Print_Bill_USE23POLLING                   (1u)
#define Print_Bill_FLOW_CONTROL                   (0u)
#define Print_Bill_CLK_FREQ                       (0u)
#define Print_Bill_TXBUFFERSIZE                   (4u)
#define Print_Bill_RXBUFFERSIZE                   (20u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Print_Bill_CONTROL_REG_REMOVED            (0u)
#else
    #define Print_Bill_CONTROL_REG_REMOVED            (1u)
#endif /* End Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Print_Bill_backupStruct_
{
    uint8 enableState;

    #if(Print_Bill_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Print_Bill_CONTROL_REG_REMOVED */
    #if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Print_Bill_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Print_Bill_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED)*/

    #if(Print_Bill_TX_ENABLED)
        #if(Print_Bill_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Print_Bill_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Print_Bill_TX_ENABLED */
} Print_Bill_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Print_Bill_Start(void) ;
void Print_Bill_Stop(void) ;
uint8 Print_Bill_ReadControlRegister(void) ;
void Print_Bill_WriteControlRegister(uint8 control) ;

void Print_Bill_Init(void) ;
void Print_Bill_Enable(void) ;
void Print_Bill_SaveConfig(void) ;
void Print_Bill_RestoreConfig(void) ;
void Print_Bill_Sleep(void) ;
void Print_Bill_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )

    #if(Print_Bill_RX_INTERRUPT_ENABLED)
        void  Print_Bill_EnableRxInt(void) ;
        void  Print_Bill_DisableRxInt(void) ;
        CY_ISR_PROTO(Print_Bill_RXISR);
    #endif /* Print_Bill_RX_INTERRUPT_ENABLED */

    void Print_Bill_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Print_Bill_SetRxAddress1(uint8 address) ;
    void Print_Bill_SetRxAddress2(uint8 address) ;

    void  Print_Bill_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Print_Bill_ReadRxData(void) ;
    uint8 Print_Bill_ReadRxStatus(void) ;
    uint8 Print_Bill_GetChar(void) ;
    uint16 Print_Bill_GetByte(void) ;
    uint8 Print_Bill_GetRxBufferSize(void)
                                                            ;
    void Print_Bill_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Print_Bill_GetRxInterruptSource   Print_Bill_ReadRxStatus

#endif /* End (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */

/* Only if TX is enabled */
#if(Print_Bill_TX_ENABLED || Print_Bill_HD_ENABLED)

    #if(Print_Bill_TX_INTERRUPT_ENABLED)
        void Print_Bill_EnableTxInt(void) ;
        void Print_Bill_DisableTxInt(void) ;
        CY_ISR_PROTO(Print_Bill_TXISR);
    #endif /* Print_Bill_TX_INTERRUPT_ENABLED */

    void Print_Bill_SetTxInterruptMode(uint8 intSrc) ;
    void Print_Bill_WriteTxData(uint8 txDataByte) ;
    uint8 Print_Bill_ReadTxStatus(void) ;
    void Print_Bill_PutChar(uint8 txDataByte) ;
    void Print_Bill_PutString(const char8 string[]) ;
    void Print_Bill_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Print_Bill_PutCRLF(uint8 txDataByte) ;
    void Print_Bill_ClearTxBuffer(void) ;
    void Print_Bill_SetTxAddressMode(uint8 addressMode) ;
    void Print_Bill_SendBreak(uint8 retMode) ;
    uint8 Print_Bill_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Print_Bill_PutStringConst         Print_Bill_PutString
    #define Print_Bill_PutArrayConst          Print_Bill_PutArray
    #define Print_Bill_GetTxInterruptSource   Print_Bill_ReadTxStatus

#endif /* End Print_Bill_TX_ENABLED || Print_Bill_HD_ENABLED */

#if(Print_Bill_HD_ENABLED)
    void Print_Bill_LoadRxConfig(void) ;
    void Print_Bill_LoadTxConfig(void) ;
#endif /* End Print_Bill_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Print_Bill) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Print_Bill_CyBtldrCommStart(void) CYSMALL ;
    void    Print_Bill_CyBtldrCommStop(void) CYSMALL ;
    void    Print_Bill_CyBtldrCommReset(void) CYSMALL ;
    cystatus Print_Bill_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Print_Bill_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Print_Bill)
        #define CyBtldrCommStart    Print_Bill_CyBtldrCommStart
        #define CyBtldrCommStop     Print_Bill_CyBtldrCommStop
        #define CyBtldrCommReset    Print_Bill_CyBtldrCommReset
        #define CyBtldrCommWrite    Print_Bill_CyBtldrCommWrite
        #define CyBtldrCommRead     Print_Bill_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Print_Bill) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Print_Bill_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Print_Bill_SET_SPACE                              (0x00u)
#define Print_Bill_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Print_Bill_TX_ENABLED) || (Print_Bill_HD_ENABLED) )
    #if(Print_Bill_TX_INTERRUPT_ENABLED)
        #define Print_Bill_TX_VECT_NUM            (uint8)Print_Bill_TXInternalInterrupt__INTC_NUMBER
        #define Print_Bill_TX_PRIOR_NUM           (uint8)Print_Bill_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Print_Bill_TX_INTERRUPT_ENABLED */
    #if(Print_Bill_TX_ENABLED)
        #define Print_Bill_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Print_Bill_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Print_Bill_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Print_Bill_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Print_Bill_TX_ENABLED */
    #if(Print_Bill_HD_ENABLED)
        #define Print_Bill_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Print_Bill_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Print_Bill_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Print_Bill_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Print_Bill_HD_ENABLED */
    #define Print_Bill_TX_STS_COMPLETE            (uint8)(0x01u << Print_Bill_TX_STS_COMPLETE_SHIFT)
    #define Print_Bill_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Print_Bill_TX_STS_FIFO_EMPTY_SHIFT)
    #define Print_Bill_TX_STS_FIFO_FULL           (uint8)(0x01u << Print_Bill_TX_STS_FIFO_FULL_SHIFT)
    #define Print_Bill_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Print_Bill_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Print_Bill_TX_ENABLED) || (Print_Bill_HD_ENABLED)*/

#if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
    #if(Print_Bill_RX_INTERRUPT_ENABLED)
        #define Print_Bill_RX_VECT_NUM            (uint8)Print_Bill_RXInternalInterrupt__INTC_NUMBER
        #define Print_Bill_RX_PRIOR_NUM           (uint8)Print_Bill_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Print_Bill_RX_INTERRUPT_ENABLED */
    #define Print_Bill_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Print_Bill_RX_STS_BREAK_SHIFT             (0x01u)
    #define Print_Bill_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Print_Bill_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Print_Bill_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Print_Bill_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Print_Bill_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Print_Bill_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Print_Bill_RX_STS_MRKSPC           (uint8)(0x01u << Print_Bill_RX_STS_MRKSPC_SHIFT)
    #define Print_Bill_RX_STS_BREAK            (uint8)(0x01u << Print_Bill_RX_STS_BREAK_SHIFT)
    #define Print_Bill_RX_STS_PAR_ERROR        (uint8)(0x01u << Print_Bill_RX_STS_PAR_ERROR_SHIFT)
    #define Print_Bill_RX_STS_STOP_ERROR       (uint8)(0x01u << Print_Bill_RX_STS_STOP_ERROR_SHIFT)
    #define Print_Bill_RX_STS_OVERRUN          (uint8)(0x01u << Print_Bill_RX_STS_OVERRUN_SHIFT)
    #define Print_Bill_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Print_Bill_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Print_Bill_RX_STS_ADDR_MATCH       (uint8)(0x01u << Print_Bill_RX_STS_ADDR_MATCH_SHIFT)
    #define Print_Bill_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Print_Bill_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Print_Bill_RX_HW_MASK                     (0x7Fu)
#endif /* End (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */

/* Control Register definitions */
#define Print_Bill_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Print_Bill_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Print_Bill_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Print_Bill_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Print_Bill_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Print_Bill_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Print_Bill_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Print_Bill_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Print_Bill_CTRL_HD_SEND               (uint8)(0x01u << Print_Bill_CTRL_HD_SEND_SHIFT)
#define Print_Bill_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Print_Bill_CTRL_HD_SEND_BREAK_SHIFT)
#define Print_Bill_CTRL_MARK                  (uint8)(0x01u << Print_Bill_CTRL_MARK_SHIFT)
#define Print_Bill_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Print_Bill_CTRL_PARITY_TYPE0_SHIFT)
#define Print_Bill_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Print_Bill_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Print_Bill_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Print_Bill_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Print_Bill_SEND_BREAK                         (0x00u)
#define Print_Bill_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Print_Bill_REINIT                             (0x02u)
#define Print_Bill_SEND_WAIT_REINIT                   (0x03u)

#define Print_Bill_OVER_SAMPLE_8                      (8u)
#define Print_Bill_OVER_SAMPLE_16                     (16u)

#define Print_Bill_BIT_CENTER                         (Print_Bill_OVER_SAMPLE_COUNT - 1u)

#define Print_Bill_FIFO_LENGTH                        (4u)
#define Print_Bill_NUMBER_OF_START_BIT                (1u)
#define Print_Bill_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Print_Bill_TXBITCTR_BREAKBITS8X   ((Print_Bill_BREAK_BITS_TX * Print_Bill_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Print_Bill_TXBITCTR_BREAKBITS ((Print_Bill_BREAK_BITS_TX * Print_Bill_OVER_SAMPLE_COUNT) - 1u)

#define Print_Bill_HALF_BIT_COUNT   \
                            (((Print_Bill_OVER_SAMPLE_COUNT / 2u) + (Print_Bill_USE23POLLING * 1u)) - 2u)
#if (Print_Bill_OVER_SAMPLE_COUNT == Print_Bill_OVER_SAMPLE_8)
    #define Print_Bill_HD_TXBITCTR_INIT   (((Print_Bill_BREAK_BITS_TX + \
                            Print_Bill_NUMBER_OF_START_BIT) * Print_Bill_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Print_Bill_RXBITCTR_INIT  ((((Print_Bill_BREAK_BITS_RX + Print_Bill_NUMBER_OF_START_BIT) \
                            * Print_Bill_OVER_SAMPLE_COUNT) + Print_Bill_HALF_BIT_COUNT) - 1u)


#else /* Print_Bill_OVER_SAMPLE_COUNT == Print_Bill_OVER_SAMPLE_16 */
    #define Print_Bill_HD_TXBITCTR_INIT   ((8u * Print_Bill_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Print_Bill_RXBITCTR_INIT      (((7u * Print_Bill_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Print_Bill_HALF_BIT_COUNT)
#endif /* End Print_Bill_OVER_SAMPLE_COUNT */
#define Print_Bill_HD_RXBITCTR_INIT                   Print_Bill_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Print_Bill_initVar;
#if( Print_Bill_TX_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
    extern volatile uint8 Print_Bill_txBuffer[Print_Bill_TXBUFFERSIZE];
    extern volatile uint8 Print_Bill_txBufferRead;
    extern uint8 Print_Bill_txBufferWrite;
#endif /* End Print_Bill_TX_ENABLED */
#if( ( Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED ) && \
     (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH) )
    extern volatile uint8 Print_Bill_rxBuffer[Print_Bill_RXBUFFERSIZE];
    extern volatile uint8 Print_Bill_rxBufferRead;
    extern volatile uint8 Print_Bill_rxBufferWrite;
    extern volatile uint8 Print_Bill_rxBufferLoopDetect;
    extern volatile uint8 Print_Bill_rxBufferOverflow;
    #if (Print_Bill_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Print_Bill_rxAddressMode;
        extern volatile uint8 Print_Bill_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Print_Bill_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Print_Bill__B_UART__AM_SW_BYTE_BYTE 1
#define Print_Bill__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Print_Bill__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Print_Bill__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Print_Bill__B_UART__AM_NONE 0

#define Print_Bill__B_UART__NONE_REVB 0
#define Print_Bill__B_UART__EVEN_REVB 1
#define Print_Bill__B_UART__ODD_REVB 2
#define Print_Bill__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Print_Bill_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Print_Bill_NUMBER_OF_STOP_BITS    (1u)

#if (Print_Bill_RXHW_ADDRESS_ENABLED)
    #define Print_Bill_RXADDRESSMODE      (0u)
    #define Print_Bill_RXHWADDRESS1       (0u)
    #define Print_Bill_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Print_Bill_RXAddressMode      Print_Bill_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Print_Bill_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Print_Bill_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Print_Bill_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Print_Bill_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Print_Bill_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Print_Bill_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Print_Bill_RX_STS_BREAK_SHIFT) \
                                        | (0 << Print_Bill_RX_STS_OVERRUN_SHIFT))

#define Print_Bill_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Print_Bill_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Print_Bill_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Print_Bill_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Print_Bill_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Print_Bill_CONTROL_REG \
                            (* (reg8 *) Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Print_Bill_CONTROL_PTR \
                            (  (reg8 *) Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Print_Bill_TX_ENABLED)
    #define Print_Bill_TXDATA_REG          (* (reg8 *) Print_Bill_BUART_sTX_TxShifter_u0__F0_REG)
    #define Print_Bill_TXDATA_PTR          (  (reg8 *) Print_Bill_BUART_sTX_TxShifter_u0__F0_REG)
    #define Print_Bill_TXDATA_AUX_CTL_REG  (* (reg8 *) Print_Bill_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Print_Bill_TXDATA_AUX_CTL_PTR  (  (reg8 *) Print_Bill_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Print_Bill_TXSTATUS_REG        (* (reg8 *) Print_Bill_BUART_sTX_TxSts__STATUS_REG)
    #define Print_Bill_TXSTATUS_PTR        (  (reg8 *) Print_Bill_BUART_sTX_TxSts__STATUS_REG)
    #define Print_Bill_TXSTATUS_MASK_REG   (* (reg8 *) Print_Bill_BUART_sTX_TxSts__MASK_REG)
    #define Print_Bill_TXSTATUS_MASK_PTR   (  (reg8 *) Print_Bill_BUART_sTX_TxSts__MASK_REG)
    #define Print_Bill_TXSTATUS_ACTL_REG   (* (reg8 *) Print_Bill_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Print_Bill_TXSTATUS_ACTL_PTR   (  (reg8 *) Print_Bill_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Print_Bill_TXCLKGEN_DP)
        #define Print_Bill_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Print_Bill_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Print_Bill_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Print_Bill_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Print_Bill_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Print_Bill_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Print_Bill_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Print_Bill_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Print_Bill_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Print_Bill_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Print_Bill_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Print_Bill_TXCLKGEN_DP */

#endif /* End Print_Bill_TX_ENABLED */

#if(Print_Bill_HD_ENABLED)

    #define Print_Bill_TXDATA_REG             (* (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__F1_REG )
    #define Print_Bill_TXDATA_PTR             (  (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__F1_REG )
    #define Print_Bill_TXDATA_AUX_CTL_REG     (* (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Print_Bill_TXDATA_AUX_CTL_PTR     (  (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Print_Bill_TXSTATUS_REG           (* (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_REG )
    #define Print_Bill_TXSTATUS_PTR           (  (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_REG )
    #define Print_Bill_TXSTATUS_MASK_REG      (* (reg8 *) Print_Bill_BUART_sRX_RxSts__MASK_REG )
    #define Print_Bill_TXSTATUS_MASK_PTR      (  (reg8 *) Print_Bill_BUART_sRX_RxSts__MASK_REG )
    #define Print_Bill_TXSTATUS_ACTL_REG      (* (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Print_Bill_TXSTATUS_ACTL_PTR      (  (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Print_Bill_HD_ENABLED */

#if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
    #define Print_Bill_RXDATA_REG             (* (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__F0_REG )
    #define Print_Bill_RXDATA_PTR             (  (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__F0_REG )
    #define Print_Bill_RXADDRESS1_REG         (* (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__D0_REG )
    #define Print_Bill_RXADDRESS1_PTR         (  (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__D0_REG )
    #define Print_Bill_RXADDRESS2_REG         (* (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__D1_REG )
    #define Print_Bill_RXADDRESS2_PTR         (  (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__D1_REG )
    #define Print_Bill_RXDATA_AUX_CTL_REG     (* (reg8 *) Print_Bill_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Print_Bill_RXBITCTR_PERIOD_REG    (* (reg8 *) Print_Bill_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Print_Bill_RXBITCTR_PERIOD_PTR    (  (reg8 *) Print_Bill_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Print_Bill_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Print_Bill_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Print_Bill_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Print_Bill_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Print_Bill_RXBITCTR_COUNTER_REG   (* (reg8 *) Print_Bill_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Print_Bill_RXBITCTR_COUNTER_PTR   (  (reg8 *) Print_Bill_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Print_Bill_RXSTATUS_REG           (* (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_REG )
    #define Print_Bill_RXSTATUS_PTR           (  (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_REG )
    #define Print_Bill_RXSTATUS_MASK_REG      (* (reg8 *) Print_Bill_BUART_sRX_RxSts__MASK_REG )
    #define Print_Bill_RXSTATUS_MASK_PTR      (  (reg8 *) Print_Bill_BUART_sRX_RxSts__MASK_REG )
    #define Print_Bill_RXSTATUS_ACTL_REG      (* (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Print_Bill_RXSTATUS_ACTL_PTR      (  (reg8 *) Print_Bill_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */

#if(Print_Bill_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Print_Bill_INTCLOCK_CLKEN_REG     (* (reg8 *) Print_Bill_IntClock__PM_ACT_CFG)
    #define Print_Bill_INTCLOCK_CLKEN_PTR     (  (reg8 *) Print_Bill_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Print_Bill_INTCLOCK_CLKEN_MASK    Print_Bill_IntClock__PM_ACT_MSK
#endif /* End Print_Bill_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Print_Bill_TX_ENABLED)
    #define Print_Bill_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Print_Bill_TX_ENABLED */

#if(Print_Bill_HD_ENABLED)
    #define Print_Bill_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Print_Bill_HD_ENABLED */

#if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
    #define Print_Bill_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Print_Bill_initvar                    Print_Bill_initVar

#define Print_Bill_RX_Enabled                 Print_Bill_RX_ENABLED
#define Print_Bill_TX_Enabled                 Print_Bill_TX_ENABLED
#define Print_Bill_HD_Enabled                 Print_Bill_HD_ENABLED
#define Print_Bill_RX_IntInterruptEnabled     Print_Bill_RX_INTERRUPT_ENABLED
#define Print_Bill_TX_IntInterruptEnabled     Print_Bill_TX_INTERRUPT_ENABLED
#define Print_Bill_InternalClockUsed          Print_Bill_INTERNAL_CLOCK_USED
#define Print_Bill_RXHW_Address_Enabled       Print_Bill_RXHW_ADDRESS_ENABLED
#define Print_Bill_OverSampleCount            Print_Bill_OVER_SAMPLE_COUNT
#define Print_Bill_ParityType                 Print_Bill_PARITY_TYPE

#if( Print_Bill_TX_ENABLED && (Print_Bill_TXBUFFERSIZE > Print_Bill_FIFO_LENGTH))
    #define Print_Bill_TXBUFFER               Print_Bill_txBuffer
    #define Print_Bill_TXBUFFERREAD           Print_Bill_txBufferRead
    #define Print_Bill_TXBUFFERWRITE          Print_Bill_txBufferWrite
#endif /* End Print_Bill_TX_ENABLED */
#if( ( Print_Bill_RX_ENABLED || Print_Bill_HD_ENABLED ) && \
     (Print_Bill_RXBUFFERSIZE > Print_Bill_FIFO_LENGTH) )
    #define Print_Bill_RXBUFFER               Print_Bill_rxBuffer
    #define Print_Bill_RXBUFFERREAD           Print_Bill_rxBufferRead
    #define Print_Bill_RXBUFFERWRITE          Print_Bill_rxBufferWrite
    #define Print_Bill_RXBUFFERLOOPDETECT     Print_Bill_rxBufferLoopDetect
    #define Print_Bill_RXBUFFER_OVERFLOW      Print_Bill_rxBufferOverflow
#endif /* End Print_Bill_RX_ENABLED */

#ifdef Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Print_Bill_CONTROL                Print_Bill_CONTROL_REG
#endif /* End Print_Bill_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Print_Bill_TX_ENABLED)
    #define Print_Bill_TXDATA                 Print_Bill_TXDATA_REG
    #define Print_Bill_TXSTATUS               Print_Bill_TXSTATUS_REG
    #define Print_Bill_TXSTATUS_MASK          Print_Bill_TXSTATUS_MASK_REG
    #define Print_Bill_TXSTATUS_ACTL          Print_Bill_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Print_Bill_TXCLKGEN_DP)
        #define Print_Bill_TXBITCLKGEN_CTR        Print_Bill_TXBITCLKGEN_CTR_REG
        #define Print_Bill_TXBITCLKTX_COMPLETE    Print_Bill_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Print_Bill_TXBITCTR_PERIOD        Print_Bill_TXBITCTR_PERIOD_REG
        #define Print_Bill_TXBITCTR_CONTROL       Print_Bill_TXBITCTR_CONTROL_REG
        #define Print_Bill_TXBITCTR_COUNTER       Print_Bill_TXBITCTR_COUNTER_REG
    #endif /* Print_Bill_TXCLKGEN_DP */
#endif /* End Print_Bill_TX_ENABLED */

#if(Print_Bill_HD_ENABLED)
    #define Print_Bill_TXDATA                 Print_Bill_TXDATA_REG
    #define Print_Bill_TXSTATUS               Print_Bill_TXSTATUS_REG
    #define Print_Bill_TXSTATUS_MASK          Print_Bill_TXSTATUS_MASK_REG
    #define Print_Bill_TXSTATUS_ACTL          Print_Bill_TXSTATUS_ACTL_REG
#endif /* End Print_Bill_HD_ENABLED */

#if( (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) )
    #define Print_Bill_RXDATA                 Print_Bill_RXDATA_REG
    #define Print_Bill_RXADDRESS1             Print_Bill_RXADDRESS1_REG
    #define Print_Bill_RXADDRESS2             Print_Bill_RXADDRESS2_REG
    #define Print_Bill_RXBITCTR_PERIOD        Print_Bill_RXBITCTR_PERIOD_REG
    #define Print_Bill_RXBITCTR_CONTROL       Print_Bill_RXBITCTR_CONTROL_REG
    #define Print_Bill_RXBITCTR_COUNTER       Print_Bill_RXBITCTR_COUNTER_REG
    #define Print_Bill_RXSTATUS               Print_Bill_RXSTATUS_REG
    #define Print_Bill_RXSTATUS_MASK          Print_Bill_RXSTATUS_MASK_REG
    #define Print_Bill_RXSTATUS_ACTL          Print_Bill_RXSTATUS_ACTL_REG
#endif /* End  (Print_Bill_RX_ENABLED) || (Print_Bill_HD_ENABLED) */

#if(Print_Bill_INTERNAL_CLOCK_USED)
    #define Print_Bill_INTCLOCK_CLKEN         Print_Bill_INTCLOCK_CLKEN_REG
#endif /* End Print_Bill_INTERNAL_CLOCK_USED */

#define Print_Bill_WAIT_FOR_COMLETE_REINIT    Print_Bill_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Print_Bill_H */


/* [] END OF FILE */
