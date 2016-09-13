/*******************************************************************************
* File Name: Tag.h
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


#if !defined(CY_UART_Tag_H)
#define CY_UART_Tag_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Tag_RX_ENABLED                     (1u)
#define Tag_TX_ENABLED                     (1u)
#define Tag_HD_ENABLED                     (0u)
#define Tag_RX_INTERRUPT_ENABLED           (1u)
#define Tag_TX_INTERRUPT_ENABLED           (0u)
#define Tag_INTERNAL_CLOCK_USED            (1u)
#define Tag_RXHW_ADDRESS_ENABLED           (0u)
#define Tag_OVER_SAMPLE_COUNT              (8u)
#define Tag_PARITY_TYPE                    (0u)
#define Tag_PARITY_TYPE_SW                 (0u)
#define Tag_BREAK_DETECT                   (0u)
#define Tag_BREAK_BITS_TX                  (13u)
#define Tag_BREAK_BITS_RX                  (13u)
#define Tag_TXCLKGEN_DP                    (1u)
#define Tag_USE23POLLING                   (1u)
#define Tag_FLOW_CONTROL                   (0u)
#define Tag_CLK_FREQ                       (0u)
#define Tag_TXBUFFERSIZE                   (4u)
#define Tag_RXBUFFERSIZE                   (256u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Tag_CONTROL_REG_REMOVED            (0u)
#else
    #define Tag_CONTROL_REG_REMOVED            (1u)
#endif /* End Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Tag_backupStruct_
{
    uint8 enableState;

    #if(Tag_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Tag_CONTROL_REG_REMOVED */
    #if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Tag_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Tag_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Tag_RX_ENABLED) || (Tag_HD_ENABLED)*/

    #if(Tag_TX_ENABLED)
        #if(Tag_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Tag_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Tag_TX_ENABLED */
} Tag_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Tag_Start(void) ;
void Tag_Stop(void) ;
uint8 Tag_ReadControlRegister(void) ;
void Tag_WriteControlRegister(uint8 control) ;

void Tag_Init(void) ;
void Tag_Enable(void) ;
void Tag_SaveConfig(void) ;
void Tag_RestoreConfig(void) ;
void Tag_Sleep(void) ;
void Tag_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )

    #if(Tag_RX_INTERRUPT_ENABLED)
        void  Tag_EnableRxInt(void) ;
        void  Tag_DisableRxInt(void) ;
        CY_ISR_PROTO(Tag_RXISR);
    #endif /* Tag_RX_INTERRUPT_ENABLED */

    void Tag_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Tag_SetRxAddress1(uint8 address) ;
    void Tag_SetRxAddress2(uint8 address) ;

    void  Tag_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Tag_ReadRxData(void) ;
    uint8 Tag_ReadRxStatus(void) ;
    uint8 Tag_GetChar(void) ;
    uint16 Tag_GetByte(void) ;
    uint16 Tag_GetRxBufferSize(void)
                                                            ;
    void Tag_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Tag_GetRxInterruptSource   Tag_ReadRxStatus

#endif /* End (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */

/* Only if TX is enabled */
#if(Tag_TX_ENABLED || Tag_HD_ENABLED)

    #if(Tag_TX_INTERRUPT_ENABLED)
        void Tag_EnableTxInt(void) ;
        void Tag_DisableTxInt(void) ;
        CY_ISR_PROTO(Tag_TXISR);
    #endif /* Tag_TX_INTERRUPT_ENABLED */

    void Tag_SetTxInterruptMode(uint8 intSrc) ;
    void Tag_WriteTxData(uint8 txDataByte) ;
    uint8 Tag_ReadTxStatus(void) ;
    void Tag_PutChar(uint8 txDataByte) ;
    void Tag_PutString(const char8 string[]) ;
    void Tag_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Tag_PutCRLF(uint8 txDataByte) ;
    void Tag_ClearTxBuffer(void) ;
    void Tag_SetTxAddressMode(uint8 addressMode) ;
    void Tag_SendBreak(uint8 retMode) ;
    uint8 Tag_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Tag_PutStringConst         Tag_PutString
    #define Tag_PutArrayConst          Tag_PutArray
    #define Tag_GetTxInterruptSource   Tag_ReadTxStatus

#endif /* End Tag_TX_ENABLED || Tag_HD_ENABLED */

#if(Tag_HD_ENABLED)
    void Tag_LoadRxConfig(void) ;
    void Tag_LoadTxConfig(void) ;
#endif /* End Tag_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Tag) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Tag_CyBtldrCommStart(void) CYSMALL ;
    void    Tag_CyBtldrCommStop(void) CYSMALL ;
    void    Tag_CyBtldrCommReset(void) CYSMALL ;
    cystatus Tag_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Tag_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Tag)
        #define CyBtldrCommStart    Tag_CyBtldrCommStart
        #define CyBtldrCommStop     Tag_CyBtldrCommStop
        #define CyBtldrCommReset    Tag_CyBtldrCommReset
        #define CyBtldrCommWrite    Tag_CyBtldrCommWrite
        #define CyBtldrCommRead     Tag_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Tag) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Tag_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Tag_SET_SPACE                              (0x00u)
#define Tag_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Tag_TX_ENABLED) || (Tag_HD_ENABLED) )
    #if(Tag_TX_INTERRUPT_ENABLED)
        #define Tag_TX_VECT_NUM            (uint8)Tag_TXInternalInterrupt__INTC_NUMBER
        #define Tag_TX_PRIOR_NUM           (uint8)Tag_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Tag_TX_INTERRUPT_ENABLED */
    #if(Tag_TX_ENABLED)
        #define Tag_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Tag_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Tag_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Tag_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Tag_TX_ENABLED */
    #if(Tag_HD_ENABLED)
        #define Tag_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Tag_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Tag_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Tag_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Tag_HD_ENABLED */
    #define Tag_TX_STS_COMPLETE            (uint8)(0x01u << Tag_TX_STS_COMPLETE_SHIFT)
    #define Tag_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Tag_TX_STS_FIFO_EMPTY_SHIFT)
    #define Tag_TX_STS_FIFO_FULL           (uint8)(0x01u << Tag_TX_STS_FIFO_FULL_SHIFT)
    #define Tag_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Tag_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Tag_TX_ENABLED) || (Tag_HD_ENABLED)*/

#if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
    #if(Tag_RX_INTERRUPT_ENABLED)
        #define Tag_RX_VECT_NUM            (uint8)Tag_RXInternalInterrupt__INTC_NUMBER
        #define Tag_RX_PRIOR_NUM           (uint8)Tag_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Tag_RX_INTERRUPT_ENABLED */
    #define Tag_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Tag_RX_STS_BREAK_SHIFT             (0x01u)
    #define Tag_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Tag_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Tag_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Tag_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Tag_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Tag_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Tag_RX_STS_MRKSPC           (uint8)(0x01u << Tag_RX_STS_MRKSPC_SHIFT)
    #define Tag_RX_STS_BREAK            (uint8)(0x01u << Tag_RX_STS_BREAK_SHIFT)
    #define Tag_RX_STS_PAR_ERROR        (uint8)(0x01u << Tag_RX_STS_PAR_ERROR_SHIFT)
    #define Tag_RX_STS_STOP_ERROR       (uint8)(0x01u << Tag_RX_STS_STOP_ERROR_SHIFT)
    #define Tag_RX_STS_OVERRUN          (uint8)(0x01u << Tag_RX_STS_OVERRUN_SHIFT)
    #define Tag_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Tag_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Tag_RX_STS_ADDR_MATCH       (uint8)(0x01u << Tag_RX_STS_ADDR_MATCH_SHIFT)
    #define Tag_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Tag_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Tag_RX_HW_MASK                     (0x7Fu)
#endif /* End (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */

/* Control Register definitions */
#define Tag_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Tag_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Tag_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Tag_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Tag_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Tag_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Tag_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Tag_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Tag_CTRL_HD_SEND               (uint8)(0x01u << Tag_CTRL_HD_SEND_SHIFT)
#define Tag_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Tag_CTRL_HD_SEND_BREAK_SHIFT)
#define Tag_CTRL_MARK                  (uint8)(0x01u << Tag_CTRL_MARK_SHIFT)
#define Tag_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Tag_CTRL_PARITY_TYPE0_SHIFT)
#define Tag_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Tag_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Tag_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Tag_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Tag_SEND_BREAK                         (0x00u)
#define Tag_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Tag_REINIT                             (0x02u)
#define Tag_SEND_WAIT_REINIT                   (0x03u)

#define Tag_OVER_SAMPLE_8                      (8u)
#define Tag_OVER_SAMPLE_16                     (16u)

#define Tag_BIT_CENTER                         (Tag_OVER_SAMPLE_COUNT - 1u)

#define Tag_FIFO_LENGTH                        (4u)
#define Tag_NUMBER_OF_START_BIT                (1u)
#define Tag_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Tag_TXBITCTR_BREAKBITS8X   ((Tag_BREAK_BITS_TX * Tag_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Tag_TXBITCTR_BREAKBITS ((Tag_BREAK_BITS_TX * Tag_OVER_SAMPLE_COUNT) - 1u)

#define Tag_HALF_BIT_COUNT   \
                            (((Tag_OVER_SAMPLE_COUNT / 2u) + (Tag_USE23POLLING * 1u)) - 2u)
#if (Tag_OVER_SAMPLE_COUNT == Tag_OVER_SAMPLE_8)
    #define Tag_HD_TXBITCTR_INIT   (((Tag_BREAK_BITS_TX + \
                            Tag_NUMBER_OF_START_BIT) * Tag_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Tag_RXBITCTR_INIT  ((((Tag_BREAK_BITS_RX + Tag_NUMBER_OF_START_BIT) \
                            * Tag_OVER_SAMPLE_COUNT) + Tag_HALF_BIT_COUNT) - 1u)


#else /* Tag_OVER_SAMPLE_COUNT == Tag_OVER_SAMPLE_16 */
    #define Tag_HD_TXBITCTR_INIT   ((8u * Tag_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Tag_RXBITCTR_INIT      (((7u * Tag_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Tag_HALF_BIT_COUNT)
#endif /* End Tag_OVER_SAMPLE_COUNT */
#define Tag_HD_RXBITCTR_INIT                   Tag_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Tag_initVar;
#if( Tag_TX_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
    extern volatile uint8 Tag_txBuffer[Tag_TXBUFFERSIZE];
    extern volatile uint8 Tag_txBufferRead;
    extern uint8 Tag_txBufferWrite;
#endif /* End Tag_TX_ENABLED */
#if( ( Tag_RX_ENABLED || Tag_HD_ENABLED ) && \
     (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH) )
    extern volatile uint8 Tag_rxBuffer[Tag_RXBUFFERSIZE];
    extern volatile uint16 Tag_rxBufferRead;
    extern volatile uint16 Tag_rxBufferWrite;
    extern volatile uint8 Tag_rxBufferLoopDetect;
    extern volatile uint8 Tag_rxBufferOverflow;
    #if (Tag_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Tag_rxAddressMode;
        extern volatile uint8 Tag_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Tag_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Tag__B_UART__AM_SW_BYTE_BYTE 1
#define Tag__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Tag__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Tag__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Tag__B_UART__AM_NONE 0

#define Tag__B_UART__NONE_REVB 0
#define Tag__B_UART__EVEN_REVB 1
#define Tag__B_UART__ODD_REVB 2
#define Tag__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Tag_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Tag_NUMBER_OF_STOP_BITS    (1u)

#if (Tag_RXHW_ADDRESS_ENABLED)
    #define Tag_RXADDRESSMODE      (0u)
    #define Tag_RXHWADDRESS1       (0u)
    #define Tag_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Tag_RXAddressMode      Tag_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Tag_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Tag_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Tag_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Tag_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Tag_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Tag_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Tag_RX_STS_BREAK_SHIFT) \
                                        | (0 << Tag_RX_STS_OVERRUN_SHIFT))

#define Tag_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Tag_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Tag_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Tag_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Tag_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Tag_CONTROL_REG \
                            (* (reg8 *) Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Tag_CONTROL_PTR \
                            (  (reg8 *) Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Tag_TX_ENABLED)
    #define Tag_TXDATA_REG          (* (reg8 *) Tag_BUART_sTX_TxShifter_u0__F0_REG)
    #define Tag_TXDATA_PTR          (  (reg8 *) Tag_BUART_sTX_TxShifter_u0__F0_REG)
    #define Tag_TXDATA_AUX_CTL_REG  (* (reg8 *) Tag_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Tag_TXDATA_AUX_CTL_PTR  (  (reg8 *) Tag_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Tag_TXSTATUS_REG        (* (reg8 *) Tag_BUART_sTX_TxSts__STATUS_REG)
    #define Tag_TXSTATUS_PTR        (  (reg8 *) Tag_BUART_sTX_TxSts__STATUS_REG)
    #define Tag_TXSTATUS_MASK_REG   (* (reg8 *) Tag_BUART_sTX_TxSts__MASK_REG)
    #define Tag_TXSTATUS_MASK_PTR   (  (reg8 *) Tag_BUART_sTX_TxSts__MASK_REG)
    #define Tag_TXSTATUS_ACTL_REG   (* (reg8 *) Tag_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Tag_TXSTATUS_ACTL_PTR   (  (reg8 *) Tag_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Tag_TXCLKGEN_DP)
        #define Tag_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Tag_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Tag_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Tag_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Tag_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Tag_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Tag_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Tag_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Tag_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Tag_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Tag_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Tag_TXCLKGEN_DP */

#endif /* End Tag_TX_ENABLED */

#if(Tag_HD_ENABLED)

    #define Tag_TXDATA_REG             (* (reg8 *) Tag_BUART_sRX_RxShifter_u0__F1_REG )
    #define Tag_TXDATA_PTR             (  (reg8 *) Tag_BUART_sRX_RxShifter_u0__F1_REG )
    #define Tag_TXDATA_AUX_CTL_REG     (* (reg8 *) Tag_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Tag_TXDATA_AUX_CTL_PTR     (  (reg8 *) Tag_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Tag_TXSTATUS_REG           (* (reg8 *) Tag_BUART_sRX_RxSts__STATUS_REG )
    #define Tag_TXSTATUS_PTR           (  (reg8 *) Tag_BUART_sRX_RxSts__STATUS_REG )
    #define Tag_TXSTATUS_MASK_REG      (* (reg8 *) Tag_BUART_sRX_RxSts__MASK_REG )
    #define Tag_TXSTATUS_MASK_PTR      (  (reg8 *) Tag_BUART_sRX_RxSts__MASK_REG )
    #define Tag_TXSTATUS_ACTL_REG      (* (reg8 *) Tag_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Tag_TXSTATUS_ACTL_PTR      (  (reg8 *) Tag_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Tag_HD_ENABLED */

#if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
    #define Tag_RXDATA_REG             (* (reg8 *) Tag_BUART_sRX_RxShifter_u0__F0_REG )
    #define Tag_RXDATA_PTR             (  (reg8 *) Tag_BUART_sRX_RxShifter_u0__F0_REG )
    #define Tag_RXADDRESS1_REG         (* (reg8 *) Tag_BUART_sRX_RxShifter_u0__D0_REG )
    #define Tag_RXADDRESS1_PTR         (  (reg8 *) Tag_BUART_sRX_RxShifter_u0__D0_REG )
    #define Tag_RXADDRESS2_REG         (* (reg8 *) Tag_BUART_sRX_RxShifter_u0__D1_REG )
    #define Tag_RXADDRESS2_PTR         (  (reg8 *) Tag_BUART_sRX_RxShifter_u0__D1_REG )
    #define Tag_RXDATA_AUX_CTL_REG     (* (reg8 *) Tag_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Tag_RXBITCTR_PERIOD_REG    (* (reg8 *) Tag_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Tag_RXBITCTR_PERIOD_PTR    (  (reg8 *) Tag_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Tag_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Tag_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Tag_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Tag_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Tag_RXBITCTR_COUNTER_REG   (* (reg8 *) Tag_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Tag_RXBITCTR_COUNTER_PTR   (  (reg8 *) Tag_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Tag_RXSTATUS_REG           (* (reg8 *) Tag_BUART_sRX_RxSts__STATUS_REG )
    #define Tag_RXSTATUS_PTR           (  (reg8 *) Tag_BUART_sRX_RxSts__STATUS_REG )
    #define Tag_RXSTATUS_MASK_REG      (* (reg8 *) Tag_BUART_sRX_RxSts__MASK_REG )
    #define Tag_RXSTATUS_MASK_PTR      (  (reg8 *) Tag_BUART_sRX_RxSts__MASK_REG )
    #define Tag_RXSTATUS_ACTL_REG      (* (reg8 *) Tag_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Tag_RXSTATUS_ACTL_PTR      (  (reg8 *) Tag_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */

#if(Tag_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Tag_INTCLOCK_CLKEN_REG     (* (reg8 *) Tag_IntClock__PM_ACT_CFG)
    #define Tag_INTCLOCK_CLKEN_PTR     (  (reg8 *) Tag_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Tag_INTCLOCK_CLKEN_MASK    Tag_IntClock__PM_ACT_MSK
#endif /* End Tag_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Tag_TX_ENABLED)
    #define Tag_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Tag_TX_ENABLED */

#if(Tag_HD_ENABLED)
    #define Tag_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Tag_HD_ENABLED */

#if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
    #define Tag_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Tag_initvar                    Tag_initVar

#define Tag_RX_Enabled                 Tag_RX_ENABLED
#define Tag_TX_Enabled                 Tag_TX_ENABLED
#define Tag_HD_Enabled                 Tag_HD_ENABLED
#define Tag_RX_IntInterruptEnabled     Tag_RX_INTERRUPT_ENABLED
#define Tag_TX_IntInterruptEnabled     Tag_TX_INTERRUPT_ENABLED
#define Tag_InternalClockUsed          Tag_INTERNAL_CLOCK_USED
#define Tag_RXHW_Address_Enabled       Tag_RXHW_ADDRESS_ENABLED
#define Tag_OverSampleCount            Tag_OVER_SAMPLE_COUNT
#define Tag_ParityType                 Tag_PARITY_TYPE

#if( Tag_TX_ENABLED && (Tag_TXBUFFERSIZE > Tag_FIFO_LENGTH))
    #define Tag_TXBUFFER               Tag_txBuffer
    #define Tag_TXBUFFERREAD           Tag_txBufferRead
    #define Tag_TXBUFFERWRITE          Tag_txBufferWrite
#endif /* End Tag_TX_ENABLED */
#if( ( Tag_RX_ENABLED || Tag_HD_ENABLED ) && \
     (Tag_RXBUFFERSIZE > Tag_FIFO_LENGTH) )
    #define Tag_RXBUFFER               Tag_rxBuffer
    #define Tag_RXBUFFERREAD           Tag_rxBufferRead
    #define Tag_RXBUFFERWRITE          Tag_rxBufferWrite
    #define Tag_RXBUFFERLOOPDETECT     Tag_rxBufferLoopDetect
    #define Tag_RXBUFFER_OVERFLOW      Tag_rxBufferOverflow
#endif /* End Tag_RX_ENABLED */

#ifdef Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Tag_CONTROL                Tag_CONTROL_REG
#endif /* End Tag_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Tag_TX_ENABLED)
    #define Tag_TXDATA                 Tag_TXDATA_REG
    #define Tag_TXSTATUS               Tag_TXSTATUS_REG
    #define Tag_TXSTATUS_MASK          Tag_TXSTATUS_MASK_REG
    #define Tag_TXSTATUS_ACTL          Tag_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Tag_TXCLKGEN_DP)
        #define Tag_TXBITCLKGEN_CTR        Tag_TXBITCLKGEN_CTR_REG
        #define Tag_TXBITCLKTX_COMPLETE    Tag_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Tag_TXBITCTR_PERIOD        Tag_TXBITCTR_PERIOD_REG
        #define Tag_TXBITCTR_CONTROL       Tag_TXBITCTR_CONTROL_REG
        #define Tag_TXBITCTR_COUNTER       Tag_TXBITCTR_COUNTER_REG
    #endif /* Tag_TXCLKGEN_DP */
#endif /* End Tag_TX_ENABLED */

#if(Tag_HD_ENABLED)
    #define Tag_TXDATA                 Tag_TXDATA_REG
    #define Tag_TXSTATUS               Tag_TXSTATUS_REG
    #define Tag_TXSTATUS_MASK          Tag_TXSTATUS_MASK_REG
    #define Tag_TXSTATUS_ACTL          Tag_TXSTATUS_ACTL_REG
#endif /* End Tag_HD_ENABLED */

#if( (Tag_RX_ENABLED) || (Tag_HD_ENABLED) )
    #define Tag_RXDATA                 Tag_RXDATA_REG
    #define Tag_RXADDRESS1             Tag_RXADDRESS1_REG
    #define Tag_RXADDRESS2             Tag_RXADDRESS2_REG
    #define Tag_RXBITCTR_PERIOD        Tag_RXBITCTR_PERIOD_REG
    #define Tag_RXBITCTR_CONTROL       Tag_RXBITCTR_CONTROL_REG
    #define Tag_RXBITCTR_COUNTER       Tag_RXBITCTR_COUNTER_REG
    #define Tag_RXSTATUS               Tag_RXSTATUS_REG
    #define Tag_RXSTATUS_MASK          Tag_RXSTATUS_MASK_REG
    #define Tag_RXSTATUS_ACTL          Tag_RXSTATUS_ACTL_REG
#endif /* End  (Tag_RX_ENABLED) || (Tag_HD_ENABLED) */

#if(Tag_INTERNAL_CLOCK_USED)
    #define Tag_INTCLOCK_CLKEN         Tag_INTCLOCK_CLKEN_REG
#endif /* End Tag_INTERNAL_CLOCK_USED */

#define Tag_WAIT_FOR_COMLETE_REINIT    Tag_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Tag_H */


/* [] END OF FILE */
