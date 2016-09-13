/*******************************************************************************
* File Name: Code_Bar.h
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


#if !defined(CY_UART_Code_Bar_H)
#define CY_UART_Code_Bar_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Code_Bar_RX_ENABLED                     (1u)
#define Code_Bar_TX_ENABLED                     (1u)
#define Code_Bar_HD_ENABLED                     (0u)
#define Code_Bar_RX_INTERRUPT_ENABLED           (1u)
#define Code_Bar_TX_INTERRUPT_ENABLED           (0u)
#define Code_Bar_INTERNAL_CLOCK_USED            (1u)
#define Code_Bar_RXHW_ADDRESS_ENABLED           (0u)
#define Code_Bar_OVER_SAMPLE_COUNT              (8u)
#define Code_Bar_PARITY_TYPE                    (0u)
#define Code_Bar_PARITY_TYPE_SW                 (0u)
#define Code_Bar_BREAK_DETECT                   (0u)
#define Code_Bar_BREAK_BITS_TX                  (13u)
#define Code_Bar_BREAK_BITS_RX                  (13u)
#define Code_Bar_TXCLKGEN_DP                    (1u)
#define Code_Bar_USE23POLLING                   (1u)
#define Code_Bar_FLOW_CONTROL                   (0u)
#define Code_Bar_CLK_FREQ                       (0u)
#define Code_Bar_TXBUFFERSIZE                   (4u)
#define Code_Bar_RXBUFFERSIZE                   (256u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Code_Bar_CONTROL_REG_REMOVED            (0u)
#else
    #define Code_Bar_CONTROL_REG_REMOVED            (1u)
#endif /* End Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Code_Bar_backupStruct_
{
    uint8 enableState;

    #if(Code_Bar_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Code_Bar_CONTROL_REG_REMOVED */
    #if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Code_Bar_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Code_Bar_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED)*/

    #if(Code_Bar_TX_ENABLED)
        #if(Code_Bar_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Code_Bar_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Code_Bar_TX_ENABLED */
} Code_Bar_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Code_Bar_Start(void) ;
void Code_Bar_Stop(void) ;
uint8 Code_Bar_ReadControlRegister(void) ;
void Code_Bar_WriteControlRegister(uint8 control) ;

void Code_Bar_Init(void) ;
void Code_Bar_Enable(void) ;
void Code_Bar_SaveConfig(void) ;
void Code_Bar_RestoreConfig(void) ;
void Code_Bar_Sleep(void) ;
void Code_Bar_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )

    #if(Code_Bar_RX_INTERRUPT_ENABLED)
        void  Code_Bar_EnableRxInt(void) ;
        void  Code_Bar_DisableRxInt(void) ;
        CY_ISR_PROTO(Code_Bar_RXISR);
    #endif /* Code_Bar_RX_INTERRUPT_ENABLED */

    void Code_Bar_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Code_Bar_SetRxAddress1(uint8 address) ;
    void Code_Bar_SetRxAddress2(uint8 address) ;

    void  Code_Bar_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Code_Bar_ReadRxData(void) ;
    uint8 Code_Bar_ReadRxStatus(void) ;
    uint8 Code_Bar_GetChar(void) ;
    uint16 Code_Bar_GetByte(void) ;
    uint16 Code_Bar_GetRxBufferSize(void)
                                                            ;
    void Code_Bar_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Code_Bar_GetRxInterruptSource   Code_Bar_ReadRxStatus

#endif /* End (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */

/* Only if TX is enabled */
#if(Code_Bar_TX_ENABLED || Code_Bar_HD_ENABLED)

    #if(Code_Bar_TX_INTERRUPT_ENABLED)
        void Code_Bar_EnableTxInt(void) ;
        void Code_Bar_DisableTxInt(void) ;
        CY_ISR_PROTO(Code_Bar_TXISR);
    #endif /* Code_Bar_TX_INTERRUPT_ENABLED */

    void Code_Bar_SetTxInterruptMode(uint8 intSrc) ;
    void Code_Bar_WriteTxData(uint8 txDataByte) ;
    uint8 Code_Bar_ReadTxStatus(void) ;
    void Code_Bar_PutChar(uint8 txDataByte) ;
    void Code_Bar_PutString(const char8 string[]) ;
    void Code_Bar_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Code_Bar_PutCRLF(uint8 txDataByte) ;
    void Code_Bar_ClearTxBuffer(void) ;
    void Code_Bar_SetTxAddressMode(uint8 addressMode) ;
    void Code_Bar_SendBreak(uint8 retMode) ;
    uint8 Code_Bar_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Code_Bar_PutStringConst         Code_Bar_PutString
    #define Code_Bar_PutArrayConst          Code_Bar_PutArray
    #define Code_Bar_GetTxInterruptSource   Code_Bar_ReadTxStatus

#endif /* End Code_Bar_TX_ENABLED || Code_Bar_HD_ENABLED */

#if(Code_Bar_HD_ENABLED)
    void Code_Bar_LoadRxConfig(void) ;
    void Code_Bar_LoadTxConfig(void) ;
#endif /* End Code_Bar_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Code_Bar) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Code_Bar_CyBtldrCommStart(void) CYSMALL ;
    void    Code_Bar_CyBtldrCommStop(void) CYSMALL ;
    void    Code_Bar_CyBtldrCommReset(void) CYSMALL ;
    cystatus Code_Bar_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Code_Bar_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Code_Bar)
        #define CyBtldrCommStart    Code_Bar_CyBtldrCommStart
        #define CyBtldrCommStop     Code_Bar_CyBtldrCommStop
        #define CyBtldrCommReset    Code_Bar_CyBtldrCommReset
        #define CyBtldrCommWrite    Code_Bar_CyBtldrCommWrite
        #define CyBtldrCommRead     Code_Bar_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Code_Bar) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Code_Bar_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Code_Bar_SET_SPACE                              (0x00u)
#define Code_Bar_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Code_Bar_TX_ENABLED) || (Code_Bar_HD_ENABLED) )
    #if(Code_Bar_TX_INTERRUPT_ENABLED)
        #define Code_Bar_TX_VECT_NUM            (uint8)Code_Bar_TXInternalInterrupt__INTC_NUMBER
        #define Code_Bar_TX_PRIOR_NUM           (uint8)Code_Bar_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Code_Bar_TX_INTERRUPT_ENABLED */
    #if(Code_Bar_TX_ENABLED)
        #define Code_Bar_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Code_Bar_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Code_Bar_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Code_Bar_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Code_Bar_TX_ENABLED */
    #if(Code_Bar_HD_ENABLED)
        #define Code_Bar_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Code_Bar_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Code_Bar_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Code_Bar_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Code_Bar_HD_ENABLED */
    #define Code_Bar_TX_STS_COMPLETE            (uint8)(0x01u << Code_Bar_TX_STS_COMPLETE_SHIFT)
    #define Code_Bar_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Code_Bar_TX_STS_FIFO_EMPTY_SHIFT)
    #define Code_Bar_TX_STS_FIFO_FULL           (uint8)(0x01u << Code_Bar_TX_STS_FIFO_FULL_SHIFT)
    #define Code_Bar_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Code_Bar_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Code_Bar_TX_ENABLED) || (Code_Bar_HD_ENABLED)*/

#if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
    #if(Code_Bar_RX_INTERRUPT_ENABLED)
        #define Code_Bar_RX_VECT_NUM            (uint8)Code_Bar_RXInternalInterrupt__INTC_NUMBER
        #define Code_Bar_RX_PRIOR_NUM           (uint8)Code_Bar_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Code_Bar_RX_INTERRUPT_ENABLED */
    #define Code_Bar_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Code_Bar_RX_STS_BREAK_SHIFT             (0x01u)
    #define Code_Bar_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Code_Bar_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Code_Bar_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Code_Bar_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Code_Bar_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Code_Bar_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Code_Bar_RX_STS_MRKSPC           (uint8)(0x01u << Code_Bar_RX_STS_MRKSPC_SHIFT)
    #define Code_Bar_RX_STS_BREAK            (uint8)(0x01u << Code_Bar_RX_STS_BREAK_SHIFT)
    #define Code_Bar_RX_STS_PAR_ERROR        (uint8)(0x01u << Code_Bar_RX_STS_PAR_ERROR_SHIFT)
    #define Code_Bar_RX_STS_STOP_ERROR       (uint8)(0x01u << Code_Bar_RX_STS_STOP_ERROR_SHIFT)
    #define Code_Bar_RX_STS_OVERRUN          (uint8)(0x01u << Code_Bar_RX_STS_OVERRUN_SHIFT)
    #define Code_Bar_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Code_Bar_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Code_Bar_RX_STS_ADDR_MATCH       (uint8)(0x01u << Code_Bar_RX_STS_ADDR_MATCH_SHIFT)
    #define Code_Bar_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Code_Bar_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Code_Bar_RX_HW_MASK                     (0x7Fu)
#endif /* End (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */

/* Control Register definitions */
#define Code_Bar_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Code_Bar_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Code_Bar_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Code_Bar_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Code_Bar_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Code_Bar_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Code_Bar_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Code_Bar_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Code_Bar_CTRL_HD_SEND               (uint8)(0x01u << Code_Bar_CTRL_HD_SEND_SHIFT)
#define Code_Bar_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Code_Bar_CTRL_HD_SEND_BREAK_SHIFT)
#define Code_Bar_CTRL_MARK                  (uint8)(0x01u << Code_Bar_CTRL_MARK_SHIFT)
#define Code_Bar_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Code_Bar_CTRL_PARITY_TYPE0_SHIFT)
#define Code_Bar_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Code_Bar_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Code_Bar_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Code_Bar_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Code_Bar_SEND_BREAK                         (0x00u)
#define Code_Bar_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Code_Bar_REINIT                             (0x02u)
#define Code_Bar_SEND_WAIT_REINIT                   (0x03u)

#define Code_Bar_OVER_SAMPLE_8                      (8u)
#define Code_Bar_OVER_SAMPLE_16                     (16u)

#define Code_Bar_BIT_CENTER                         (Code_Bar_OVER_SAMPLE_COUNT - 1u)

#define Code_Bar_FIFO_LENGTH                        (4u)
#define Code_Bar_NUMBER_OF_START_BIT                (1u)
#define Code_Bar_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Code_Bar_TXBITCTR_BREAKBITS8X   ((Code_Bar_BREAK_BITS_TX * Code_Bar_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Code_Bar_TXBITCTR_BREAKBITS ((Code_Bar_BREAK_BITS_TX * Code_Bar_OVER_SAMPLE_COUNT) - 1u)

#define Code_Bar_HALF_BIT_COUNT   \
                            (((Code_Bar_OVER_SAMPLE_COUNT / 2u) + (Code_Bar_USE23POLLING * 1u)) - 2u)
#if (Code_Bar_OVER_SAMPLE_COUNT == Code_Bar_OVER_SAMPLE_8)
    #define Code_Bar_HD_TXBITCTR_INIT   (((Code_Bar_BREAK_BITS_TX + \
                            Code_Bar_NUMBER_OF_START_BIT) * Code_Bar_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Code_Bar_RXBITCTR_INIT  ((((Code_Bar_BREAK_BITS_RX + Code_Bar_NUMBER_OF_START_BIT) \
                            * Code_Bar_OVER_SAMPLE_COUNT) + Code_Bar_HALF_BIT_COUNT) - 1u)


#else /* Code_Bar_OVER_SAMPLE_COUNT == Code_Bar_OVER_SAMPLE_16 */
    #define Code_Bar_HD_TXBITCTR_INIT   ((8u * Code_Bar_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Code_Bar_RXBITCTR_INIT      (((7u * Code_Bar_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Code_Bar_HALF_BIT_COUNT)
#endif /* End Code_Bar_OVER_SAMPLE_COUNT */
#define Code_Bar_HD_RXBITCTR_INIT                   Code_Bar_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Code_Bar_initVar;
#if( Code_Bar_TX_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
    extern volatile uint8 Code_Bar_txBuffer[Code_Bar_TXBUFFERSIZE];
    extern volatile uint8 Code_Bar_txBufferRead;
    extern uint8 Code_Bar_txBufferWrite;
#endif /* End Code_Bar_TX_ENABLED */
#if( ( Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED ) && \
     (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH) )
    extern volatile uint8 Code_Bar_rxBuffer[Code_Bar_RXBUFFERSIZE];
    extern volatile uint16 Code_Bar_rxBufferRead;
    extern volatile uint16 Code_Bar_rxBufferWrite;
    extern volatile uint8 Code_Bar_rxBufferLoopDetect;
    extern volatile uint8 Code_Bar_rxBufferOverflow;
    #if (Code_Bar_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Code_Bar_rxAddressMode;
        extern volatile uint8 Code_Bar_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Code_Bar_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Code_Bar__B_UART__AM_SW_BYTE_BYTE 1
#define Code_Bar__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Code_Bar__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Code_Bar__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Code_Bar__B_UART__AM_NONE 0

#define Code_Bar__B_UART__NONE_REVB 0
#define Code_Bar__B_UART__EVEN_REVB 1
#define Code_Bar__B_UART__ODD_REVB 2
#define Code_Bar__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Code_Bar_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Code_Bar_NUMBER_OF_STOP_BITS    (1u)

#if (Code_Bar_RXHW_ADDRESS_ENABLED)
    #define Code_Bar_RXADDRESSMODE      (0u)
    #define Code_Bar_RXHWADDRESS1       (0u)
    #define Code_Bar_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Code_Bar_RXAddressMode      Code_Bar_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Code_Bar_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Code_Bar_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Code_Bar_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Code_Bar_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Code_Bar_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Code_Bar_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Code_Bar_RX_STS_BREAK_SHIFT) \
                                        | (0 << Code_Bar_RX_STS_OVERRUN_SHIFT))

#define Code_Bar_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Code_Bar_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Code_Bar_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Code_Bar_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Code_Bar_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Code_Bar_CONTROL_REG \
                            (* (reg8 *) Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Code_Bar_CONTROL_PTR \
                            (  (reg8 *) Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Code_Bar_TX_ENABLED)
    #define Code_Bar_TXDATA_REG          (* (reg8 *) Code_Bar_BUART_sTX_TxShifter_u0__F0_REG)
    #define Code_Bar_TXDATA_PTR          (  (reg8 *) Code_Bar_BUART_sTX_TxShifter_u0__F0_REG)
    #define Code_Bar_TXDATA_AUX_CTL_REG  (* (reg8 *) Code_Bar_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Code_Bar_TXDATA_AUX_CTL_PTR  (  (reg8 *) Code_Bar_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Code_Bar_TXSTATUS_REG        (* (reg8 *) Code_Bar_BUART_sTX_TxSts__STATUS_REG)
    #define Code_Bar_TXSTATUS_PTR        (  (reg8 *) Code_Bar_BUART_sTX_TxSts__STATUS_REG)
    #define Code_Bar_TXSTATUS_MASK_REG   (* (reg8 *) Code_Bar_BUART_sTX_TxSts__MASK_REG)
    #define Code_Bar_TXSTATUS_MASK_PTR   (  (reg8 *) Code_Bar_BUART_sTX_TxSts__MASK_REG)
    #define Code_Bar_TXSTATUS_ACTL_REG   (* (reg8 *) Code_Bar_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Code_Bar_TXSTATUS_ACTL_PTR   (  (reg8 *) Code_Bar_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Code_Bar_TXCLKGEN_DP)
        #define Code_Bar_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Code_Bar_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Code_Bar_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Code_Bar_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Code_Bar_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Code_Bar_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Code_Bar_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Code_Bar_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Code_Bar_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Code_Bar_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Code_Bar_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Code_Bar_TXCLKGEN_DP */

#endif /* End Code_Bar_TX_ENABLED */

#if(Code_Bar_HD_ENABLED)

    #define Code_Bar_TXDATA_REG             (* (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__F1_REG )
    #define Code_Bar_TXDATA_PTR             (  (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__F1_REG )
    #define Code_Bar_TXDATA_AUX_CTL_REG     (* (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Code_Bar_TXDATA_AUX_CTL_PTR     (  (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Code_Bar_TXSTATUS_REG           (* (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_REG )
    #define Code_Bar_TXSTATUS_PTR           (  (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_REG )
    #define Code_Bar_TXSTATUS_MASK_REG      (* (reg8 *) Code_Bar_BUART_sRX_RxSts__MASK_REG )
    #define Code_Bar_TXSTATUS_MASK_PTR      (  (reg8 *) Code_Bar_BUART_sRX_RxSts__MASK_REG )
    #define Code_Bar_TXSTATUS_ACTL_REG      (* (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Code_Bar_TXSTATUS_ACTL_PTR      (  (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Code_Bar_HD_ENABLED */

#if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
    #define Code_Bar_RXDATA_REG             (* (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__F0_REG )
    #define Code_Bar_RXDATA_PTR             (  (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__F0_REG )
    #define Code_Bar_RXADDRESS1_REG         (* (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__D0_REG )
    #define Code_Bar_RXADDRESS1_PTR         (  (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__D0_REG )
    #define Code_Bar_RXADDRESS2_REG         (* (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__D1_REG )
    #define Code_Bar_RXADDRESS2_PTR         (  (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__D1_REG )
    #define Code_Bar_RXDATA_AUX_CTL_REG     (* (reg8 *) Code_Bar_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Code_Bar_RXBITCTR_PERIOD_REG    (* (reg8 *) Code_Bar_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Code_Bar_RXBITCTR_PERIOD_PTR    (  (reg8 *) Code_Bar_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Code_Bar_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Code_Bar_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Code_Bar_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Code_Bar_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Code_Bar_RXBITCTR_COUNTER_REG   (* (reg8 *) Code_Bar_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Code_Bar_RXBITCTR_COUNTER_PTR   (  (reg8 *) Code_Bar_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Code_Bar_RXSTATUS_REG           (* (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_REG )
    #define Code_Bar_RXSTATUS_PTR           (  (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_REG )
    #define Code_Bar_RXSTATUS_MASK_REG      (* (reg8 *) Code_Bar_BUART_sRX_RxSts__MASK_REG )
    #define Code_Bar_RXSTATUS_MASK_PTR      (  (reg8 *) Code_Bar_BUART_sRX_RxSts__MASK_REG )
    #define Code_Bar_RXSTATUS_ACTL_REG      (* (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Code_Bar_RXSTATUS_ACTL_PTR      (  (reg8 *) Code_Bar_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */

#if(Code_Bar_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Code_Bar_INTCLOCK_CLKEN_REG     (* (reg8 *) Code_Bar_IntClock__PM_ACT_CFG)
    #define Code_Bar_INTCLOCK_CLKEN_PTR     (  (reg8 *) Code_Bar_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Code_Bar_INTCLOCK_CLKEN_MASK    Code_Bar_IntClock__PM_ACT_MSK
#endif /* End Code_Bar_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Code_Bar_TX_ENABLED)
    #define Code_Bar_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Code_Bar_TX_ENABLED */

#if(Code_Bar_HD_ENABLED)
    #define Code_Bar_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Code_Bar_HD_ENABLED */

#if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
    #define Code_Bar_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Code_Bar_initvar                    Code_Bar_initVar

#define Code_Bar_RX_Enabled                 Code_Bar_RX_ENABLED
#define Code_Bar_TX_Enabled                 Code_Bar_TX_ENABLED
#define Code_Bar_HD_Enabled                 Code_Bar_HD_ENABLED
#define Code_Bar_RX_IntInterruptEnabled     Code_Bar_RX_INTERRUPT_ENABLED
#define Code_Bar_TX_IntInterruptEnabled     Code_Bar_TX_INTERRUPT_ENABLED
#define Code_Bar_InternalClockUsed          Code_Bar_INTERNAL_CLOCK_USED
#define Code_Bar_RXHW_Address_Enabled       Code_Bar_RXHW_ADDRESS_ENABLED
#define Code_Bar_OverSampleCount            Code_Bar_OVER_SAMPLE_COUNT
#define Code_Bar_ParityType                 Code_Bar_PARITY_TYPE

#if( Code_Bar_TX_ENABLED && (Code_Bar_TXBUFFERSIZE > Code_Bar_FIFO_LENGTH))
    #define Code_Bar_TXBUFFER               Code_Bar_txBuffer
    #define Code_Bar_TXBUFFERREAD           Code_Bar_txBufferRead
    #define Code_Bar_TXBUFFERWRITE          Code_Bar_txBufferWrite
#endif /* End Code_Bar_TX_ENABLED */
#if( ( Code_Bar_RX_ENABLED || Code_Bar_HD_ENABLED ) && \
     (Code_Bar_RXBUFFERSIZE > Code_Bar_FIFO_LENGTH) )
    #define Code_Bar_RXBUFFER               Code_Bar_rxBuffer
    #define Code_Bar_RXBUFFERREAD           Code_Bar_rxBufferRead
    #define Code_Bar_RXBUFFERWRITE          Code_Bar_rxBufferWrite
    #define Code_Bar_RXBUFFERLOOPDETECT     Code_Bar_rxBufferLoopDetect
    #define Code_Bar_RXBUFFER_OVERFLOW      Code_Bar_rxBufferOverflow
#endif /* End Code_Bar_RX_ENABLED */

#ifdef Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Code_Bar_CONTROL                Code_Bar_CONTROL_REG
#endif /* End Code_Bar_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Code_Bar_TX_ENABLED)
    #define Code_Bar_TXDATA                 Code_Bar_TXDATA_REG
    #define Code_Bar_TXSTATUS               Code_Bar_TXSTATUS_REG
    #define Code_Bar_TXSTATUS_MASK          Code_Bar_TXSTATUS_MASK_REG
    #define Code_Bar_TXSTATUS_ACTL          Code_Bar_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Code_Bar_TXCLKGEN_DP)
        #define Code_Bar_TXBITCLKGEN_CTR        Code_Bar_TXBITCLKGEN_CTR_REG
        #define Code_Bar_TXBITCLKTX_COMPLETE    Code_Bar_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Code_Bar_TXBITCTR_PERIOD        Code_Bar_TXBITCTR_PERIOD_REG
        #define Code_Bar_TXBITCTR_CONTROL       Code_Bar_TXBITCTR_CONTROL_REG
        #define Code_Bar_TXBITCTR_COUNTER       Code_Bar_TXBITCTR_COUNTER_REG
    #endif /* Code_Bar_TXCLKGEN_DP */
#endif /* End Code_Bar_TX_ENABLED */

#if(Code_Bar_HD_ENABLED)
    #define Code_Bar_TXDATA                 Code_Bar_TXDATA_REG
    #define Code_Bar_TXSTATUS               Code_Bar_TXSTATUS_REG
    #define Code_Bar_TXSTATUS_MASK          Code_Bar_TXSTATUS_MASK_REG
    #define Code_Bar_TXSTATUS_ACTL          Code_Bar_TXSTATUS_ACTL_REG
#endif /* End Code_Bar_HD_ENABLED */

#if( (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) )
    #define Code_Bar_RXDATA                 Code_Bar_RXDATA_REG
    #define Code_Bar_RXADDRESS1             Code_Bar_RXADDRESS1_REG
    #define Code_Bar_RXADDRESS2             Code_Bar_RXADDRESS2_REG
    #define Code_Bar_RXBITCTR_PERIOD        Code_Bar_RXBITCTR_PERIOD_REG
    #define Code_Bar_RXBITCTR_CONTROL       Code_Bar_RXBITCTR_CONTROL_REG
    #define Code_Bar_RXBITCTR_COUNTER       Code_Bar_RXBITCTR_COUNTER_REG
    #define Code_Bar_RXSTATUS               Code_Bar_RXSTATUS_REG
    #define Code_Bar_RXSTATUS_MASK          Code_Bar_RXSTATUS_MASK_REG
    #define Code_Bar_RXSTATUS_ACTL          Code_Bar_RXSTATUS_ACTL_REG
#endif /* End  (Code_Bar_RX_ENABLED) || (Code_Bar_HD_ENABLED) */

#if(Code_Bar_INTERNAL_CLOCK_USED)
    #define Code_Bar_INTCLOCK_CLKEN         Code_Bar_INTCLOCK_CLKEN_REG
#endif /* End Code_Bar_INTERNAL_CLOCK_USED */

#define Code_Bar_WAIT_FOR_COMLETE_REINIT    Code_Bar_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Code_Bar_H */


/* [] END OF FILE */
