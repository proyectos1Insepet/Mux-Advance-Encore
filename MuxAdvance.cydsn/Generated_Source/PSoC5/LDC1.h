/*******************************************************************************
* File Name: LDC1.h
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


#if !defined(CY_UART_LDC1_H)
#define CY_UART_LDC1_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define LDC1_RX_ENABLED                     (1u)
#define LDC1_TX_ENABLED                     (1u)
#define LDC1_HD_ENABLED                     (0u)
#define LDC1_RX_INTERRUPT_ENABLED           (1u)
#define LDC1_TX_INTERRUPT_ENABLED           (0u)
#define LDC1_INTERNAL_CLOCK_USED            (1u)
#define LDC1_RXHW_ADDRESS_ENABLED           (0u)
#define LDC1_OVER_SAMPLE_COUNT              (8u)
#define LDC1_PARITY_TYPE                    (0u)
#define LDC1_PARITY_TYPE_SW                 (0u)
#define LDC1_BREAK_DETECT                   (0u)
#define LDC1_BREAK_BITS_TX                  (13u)
#define LDC1_BREAK_BITS_RX                  (13u)
#define LDC1_TXCLKGEN_DP                    (1u)
#define LDC1_USE23POLLING                   (1u)
#define LDC1_FLOW_CONTROL                   (0u)
#define LDC1_CLK_FREQ                       (0u)
#define LDC1_TXBUFFERSIZE                   (4u)
#define LDC1_RXBUFFERSIZE                   (8u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define LDC1_CONTROL_REG_REMOVED            (0u)
#else
    #define LDC1_CONTROL_REG_REMOVED            (1u)
#endif /* End LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct LDC1_backupStruct_
{
    uint8 enableState;

    #if(LDC1_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End LDC1_CONTROL_REG_REMOVED */
    #if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (LDC1_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End LDC1_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED)*/

    #if(LDC1_TX_ENABLED)
        #if(LDC1_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End LDC1_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End LDC1_TX_ENABLED */
} LDC1_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void LDC1_Start(void) ;
void LDC1_Stop(void) ;
uint8 LDC1_ReadControlRegister(void) ;
void LDC1_WriteControlRegister(uint8 control) ;

void LDC1_Init(void) ;
void LDC1_Enable(void) ;
void LDC1_SaveConfig(void) ;
void LDC1_RestoreConfig(void) ;
void LDC1_Sleep(void) ;
void LDC1_Wakeup(void) ;

/* Only if RX is enabled */
#if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )

    #if(LDC1_RX_INTERRUPT_ENABLED)
        void  LDC1_EnableRxInt(void) ;
        void  LDC1_DisableRxInt(void) ;
        CY_ISR_PROTO(LDC1_RXISR);
    #endif /* LDC1_RX_INTERRUPT_ENABLED */

    void LDC1_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void LDC1_SetRxAddress1(uint8 address) ;
    void LDC1_SetRxAddress2(uint8 address) ;

    void  LDC1_SetRxInterruptMode(uint8 intSrc) ;
    uint8 LDC1_ReadRxData(void) ;
    uint8 LDC1_ReadRxStatus(void) ;
    uint8 LDC1_GetChar(void) ;
    uint16 LDC1_GetByte(void) ;
    uint8 LDC1_GetRxBufferSize(void)
                                                            ;
    void LDC1_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define LDC1_GetRxInterruptSource   LDC1_ReadRxStatus

#endif /* End (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */

/* Only if TX is enabled */
#if(LDC1_TX_ENABLED || LDC1_HD_ENABLED)

    #if(LDC1_TX_INTERRUPT_ENABLED)
        void LDC1_EnableTxInt(void) ;
        void LDC1_DisableTxInt(void) ;
        CY_ISR_PROTO(LDC1_TXISR);
    #endif /* LDC1_TX_INTERRUPT_ENABLED */

    void LDC1_SetTxInterruptMode(uint8 intSrc) ;
    void LDC1_WriteTxData(uint8 txDataByte) ;
    uint8 LDC1_ReadTxStatus(void) ;
    void LDC1_PutChar(uint8 txDataByte) ;
    void LDC1_PutString(const char8 string[]) ;
    void LDC1_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void LDC1_PutCRLF(uint8 txDataByte) ;
    void LDC1_ClearTxBuffer(void) ;
    void LDC1_SetTxAddressMode(uint8 addressMode) ;
    void LDC1_SendBreak(uint8 retMode) ;
    uint8 LDC1_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define LDC1_PutStringConst         LDC1_PutString
    #define LDC1_PutArrayConst          LDC1_PutArray
    #define LDC1_GetTxInterruptSource   LDC1_ReadTxStatus

#endif /* End LDC1_TX_ENABLED || LDC1_HD_ENABLED */

#if(LDC1_HD_ENABLED)
    void LDC1_LoadRxConfig(void) ;
    void LDC1_LoadTxConfig(void) ;
#endif /* End LDC1_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_LDC1) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    LDC1_CyBtldrCommStart(void) CYSMALL ;
    void    LDC1_CyBtldrCommStop(void) CYSMALL ;
    void    LDC1_CyBtldrCommReset(void) CYSMALL ;
    cystatus LDC1_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus LDC1_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_LDC1)
        #define CyBtldrCommStart    LDC1_CyBtldrCommStart
        #define CyBtldrCommStop     LDC1_CyBtldrCommStop
        #define CyBtldrCommReset    LDC1_CyBtldrCommReset
        #define CyBtldrCommWrite    LDC1_CyBtldrCommWrite
        #define CyBtldrCommRead     LDC1_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_LDC1) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define LDC1_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define LDC1_SET_SPACE                              (0x00u)
#define LDC1_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (LDC1_TX_ENABLED) || (LDC1_HD_ENABLED) )
    #if(LDC1_TX_INTERRUPT_ENABLED)
        #define LDC1_TX_VECT_NUM            (uint8)LDC1_TXInternalInterrupt__INTC_NUMBER
        #define LDC1_TX_PRIOR_NUM           (uint8)LDC1_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* LDC1_TX_INTERRUPT_ENABLED */
    #if(LDC1_TX_ENABLED)
        #define LDC1_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define LDC1_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define LDC1_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define LDC1_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* LDC1_TX_ENABLED */
    #if(LDC1_HD_ENABLED)
        #define LDC1_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define LDC1_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define LDC1_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define LDC1_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* LDC1_HD_ENABLED */
    #define LDC1_TX_STS_COMPLETE            (uint8)(0x01u << LDC1_TX_STS_COMPLETE_SHIFT)
    #define LDC1_TX_STS_FIFO_EMPTY          (uint8)(0x01u << LDC1_TX_STS_FIFO_EMPTY_SHIFT)
    #define LDC1_TX_STS_FIFO_FULL           (uint8)(0x01u << LDC1_TX_STS_FIFO_FULL_SHIFT)
    #define LDC1_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << LDC1_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (LDC1_TX_ENABLED) || (LDC1_HD_ENABLED)*/

#if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
    #if(LDC1_RX_INTERRUPT_ENABLED)
        #define LDC1_RX_VECT_NUM            (uint8)LDC1_RXInternalInterrupt__INTC_NUMBER
        #define LDC1_RX_PRIOR_NUM           (uint8)LDC1_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* LDC1_RX_INTERRUPT_ENABLED */
    #define LDC1_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define LDC1_RX_STS_BREAK_SHIFT             (0x01u)
    #define LDC1_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define LDC1_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define LDC1_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define LDC1_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define LDC1_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define LDC1_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define LDC1_RX_STS_MRKSPC           (uint8)(0x01u << LDC1_RX_STS_MRKSPC_SHIFT)
    #define LDC1_RX_STS_BREAK            (uint8)(0x01u << LDC1_RX_STS_BREAK_SHIFT)
    #define LDC1_RX_STS_PAR_ERROR        (uint8)(0x01u << LDC1_RX_STS_PAR_ERROR_SHIFT)
    #define LDC1_RX_STS_STOP_ERROR       (uint8)(0x01u << LDC1_RX_STS_STOP_ERROR_SHIFT)
    #define LDC1_RX_STS_OVERRUN          (uint8)(0x01u << LDC1_RX_STS_OVERRUN_SHIFT)
    #define LDC1_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << LDC1_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define LDC1_RX_STS_ADDR_MATCH       (uint8)(0x01u << LDC1_RX_STS_ADDR_MATCH_SHIFT)
    #define LDC1_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << LDC1_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define LDC1_RX_HW_MASK                     (0x7Fu)
#endif /* End (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */

/* Control Register definitions */
#define LDC1_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define LDC1_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define LDC1_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define LDC1_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define LDC1_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define LDC1_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define LDC1_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define LDC1_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define LDC1_CTRL_HD_SEND               (uint8)(0x01u << LDC1_CTRL_HD_SEND_SHIFT)
#define LDC1_CTRL_HD_SEND_BREAK         (uint8)(0x01u << LDC1_CTRL_HD_SEND_BREAK_SHIFT)
#define LDC1_CTRL_MARK                  (uint8)(0x01u << LDC1_CTRL_MARK_SHIFT)
#define LDC1_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << LDC1_CTRL_PARITY_TYPE0_SHIFT)
#define LDC1_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << LDC1_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define LDC1_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define LDC1_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define LDC1_SEND_BREAK                         (0x00u)
#define LDC1_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define LDC1_REINIT                             (0x02u)
#define LDC1_SEND_WAIT_REINIT                   (0x03u)

#define LDC1_OVER_SAMPLE_8                      (8u)
#define LDC1_OVER_SAMPLE_16                     (16u)

#define LDC1_BIT_CENTER                         (LDC1_OVER_SAMPLE_COUNT - 1u)

#define LDC1_FIFO_LENGTH                        (4u)
#define LDC1_NUMBER_OF_START_BIT                (1u)
#define LDC1_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define LDC1_TXBITCTR_BREAKBITS8X   ((LDC1_BREAK_BITS_TX * LDC1_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define LDC1_TXBITCTR_BREAKBITS ((LDC1_BREAK_BITS_TX * LDC1_OVER_SAMPLE_COUNT) - 1u)

#define LDC1_HALF_BIT_COUNT   \
                            (((LDC1_OVER_SAMPLE_COUNT / 2u) + (LDC1_USE23POLLING * 1u)) - 2u)
#if (LDC1_OVER_SAMPLE_COUNT == LDC1_OVER_SAMPLE_8)
    #define LDC1_HD_TXBITCTR_INIT   (((LDC1_BREAK_BITS_TX + \
                            LDC1_NUMBER_OF_START_BIT) * LDC1_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define LDC1_RXBITCTR_INIT  ((((LDC1_BREAK_BITS_RX + LDC1_NUMBER_OF_START_BIT) \
                            * LDC1_OVER_SAMPLE_COUNT) + LDC1_HALF_BIT_COUNT) - 1u)


#else /* LDC1_OVER_SAMPLE_COUNT == LDC1_OVER_SAMPLE_16 */
    #define LDC1_HD_TXBITCTR_INIT   ((8u * LDC1_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define LDC1_RXBITCTR_INIT      (((7u * LDC1_OVER_SAMPLE_COUNT) - 1u) + \
                                                      LDC1_HALF_BIT_COUNT)
#endif /* End LDC1_OVER_SAMPLE_COUNT */
#define LDC1_HD_RXBITCTR_INIT                   LDC1_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 LDC1_initVar;
#if( LDC1_TX_ENABLED && (LDC1_TXBUFFERSIZE > LDC1_FIFO_LENGTH))
    extern volatile uint8 LDC1_txBuffer[LDC1_TXBUFFERSIZE];
    extern volatile uint8 LDC1_txBufferRead;
    extern uint8 LDC1_txBufferWrite;
#endif /* End LDC1_TX_ENABLED */
#if( ( LDC1_RX_ENABLED || LDC1_HD_ENABLED ) && \
     (LDC1_RXBUFFERSIZE > LDC1_FIFO_LENGTH) )
    extern volatile uint8 LDC1_rxBuffer[LDC1_RXBUFFERSIZE];
    extern volatile uint8 LDC1_rxBufferRead;
    extern volatile uint8 LDC1_rxBufferWrite;
    extern volatile uint8 LDC1_rxBufferLoopDetect;
    extern volatile uint8 LDC1_rxBufferOverflow;
    #if (LDC1_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 LDC1_rxAddressMode;
        extern volatile uint8 LDC1_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End LDC1_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define LDC1__B_UART__AM_SW_BYTE_BYTE 1
#define LDC1__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define LDC1__B_UART__AM_HW_BYTE_BY_BYTE 3
#define LDC1__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define LDC1__B_UART__AM_NONE 0

#define LDC1__B_UART__NONE_REVB 0
#define LDC1__B_UART__EVEN_REVB 1
#define LDC1__B_UART__ODD_REVB 2
#define LDC1__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define LDC1_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define LDC1_NUMBER_OF_STOP_BITS    (1u)

#if (LDC1_RXHW_ADDRESS_ENABLED)
    #define LDC1_RXADDRESSMODE      (0u)
    #define LDC1_RXHWADDRESS1       (0u)
    #define LDC1_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define LDC1_RXAddressMode      LDC1_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define LDC1_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << LDC1_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << LDC1_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << LDC1_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << LDC1_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << LDC1_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << LDC1_RX_STS_BREAK_SHIFT) \
                                        | (0 << LDC1_RX_STS_OVERRUN_SHIFT))

#define LDC1_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << LDC1_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << LDC1_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << LDC1_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << LDC1_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define LDC1_CONTROL_REG \
                            (* (reg8 *) LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define LDC1_CONTROL_PTR \
                            (  (reg8 *) LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(LDC1_TX_ENABLED)
    #define LDC1_TXDATA_REG          (* (reg8 *) LDC1_BUART_sTX_TxShifter_u0__F0_REG)
    #define LDC1_TXDATA_PTR          (  (reg8 *) LDC1_BUART_sTX_TxShifter_u0__F0_REG)
    #define LDC1_TXDATA_AUX_CTL_REG  (* (reg8 *) LDC1_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define LDC1_TXDATA_AUX_CTL_PTR  (  (reg8 *) LDC1_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define LDC1_TXSTATUS_REG        (* (reg8 *) LDC1_BUART_sTX_TxSts__STATUS_REG)
    #define LDC1_TXSTATUS_PTR        (  (reg8 *) LDC1_BUART_sTX_TxSts__STATUS_REG)
    #define LDC1_TXSTATUS_MASK_REG   (* (reg8 *) LDC1_BUART_sTX_TxSts__MASK_REG)
    #define LDC1_TXSTATUS_MASK_PTR   (  (reg8 *) LDC1_BUART_sTX_TxSts__MASK_REG)
    #define LDC1_TXSTATUS_ACTL_REG   (* (reg8 *) LDC1_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define LDC1_TXSTATUS_ACTL_PTR   (  (reg8 *) LDC1_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(LDC1_TXCLKGEN_DP)
        #define LDC1_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define LDC1_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define LDC1_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define LDC1_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define LDC1_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define LDC1_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define LDC1_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define LDC1_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define LDC1_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define LDC1_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) LDC1_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* LDC1_TXCLKGEN_DP */

#endif /* End LDC1_TX_ENABLED */

#if(LDC1_HD_ENABLED)

    #define LDC1_TXDATA_REG             (* (reg8 *) LDC1_BUART_sRX_RxShifter_u0__F1_REG )
    #define LDC1_TXDATA_PTR             (  (reg8 *) LDC1_BUART_sRX_RxShifter_u0__F1_REG )
    #define LDC1_TXDATA_AUX_CTL_REG     (* (reg8 *) LDC1_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define LDC1_TXDATA_AUX_CTL_PTR     (  (reg8 *) LDC1_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define LDC1_TXSTATUS_REG           (* (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_REG )
    #define LDC1_TXSTATUS_PTR           (  (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_REG )
    #define LDC1_TXSTATUS_MASK_REG      (* (reg8 *) LDC1_BUART_sRX_RxSts__MASK_REG )
    #define LDC1_TXSTATUS_MASK_PTR      (  (reg8 *) LDC1_BUART_sRX_RxSts__MASK_REG )
    #define LDC1_TXSTATUS_ACTL_REG      (* (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define LDC1_TXSTATUS_ACTL_PTR      (  (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End LDC1_HD_ENABLED */

#if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
    #define LDC1_RXDATA_REG             (* (reg8 *) LDC1_BUART_sRX_RxShifter_u0__F0_REG )
    #define LDC1_RXDATA_PTR             (  (reg8 *) LDC1_BUART_sRX_RxShifter_u0__F0_REG )
    #define LDC1_RXADDRESS1_REG         (* (reg8 *) LDC1_BUART_sRX_RxShifter_u0__D0_REG )
    #define LDC1_RXADDRESS1_PTR         (  (reg8 *) LDC1_BUART_sRX_RxShifter_u0__D0_REG )
    #define LDC1_RXADDRESS2_REG         (* (reg8 *) LDC1_BUART_sRX_RxShifter_u0__D1_REG )
    #define LDC1_RXADDRESS2_PTR         (  (reg8 *) LDC1_BUART_sRX_RxShifter_u0__D1_REG )
    #define LDC1_RXDATA_AUX_CTL_REG     (* (reg8 *) LDC1_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define LDC1_RXBITCTR_PERIOD_REG    (* (reg8 *) LDC1_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define LDC1_RXBITCTR_PERIOD_PTR    (  (reg8 *) LDC1_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define LDC1_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) LDC1_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define LDC1_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) LDC1_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define LDC1_RXBITCTR_COUNTER_REG   (* (reg8 *) LDC1_BUART_sRX_RxBitCounter__COUNT_REG )
    #define LDC1_RXBITCTR_COUNTER_PTR   (  (reg8 *) LDC1_BUART_sRX_RxBitCounter__COUNT_REG )

    #define LDC1_RXSTATUS_REG           (* (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_REG )
    #define LDC1_RXSTATUS_PTR           (  (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_REG )
    #define LDC1_RXSTATUS_MASK_REG      (* (reg8 *) LDC1_BUART_sRX_RxSts__MASK_REG )
    #define LDC1_RXSTATUS_MASK_PTR      (  (reg8 *) LDC1_BUART_sRX_RxSts__MASK_REG )
    #define LDC1_RXSTATUS_ACTL_REG      (* (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define LDC1_RXSTATUS_ACTL_PTR      (  (reg8 *) LDC1_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */

#if(LDC1_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define LDC1_INTCLOCK_CLKEN_REG     (* (reg8 *) LDC1_IntClock__PM_ACT_CFG)
    #define LDC1_INTCLOCK_CLKEN_PTR     (  (reg8 *) LDC1_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define LDC1_INTCLOCK_CLKEN_MASK    LDC1_IntClock__PM_ACT_MSK
#endif /* End LDC1_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(LDC1_TX_ENABLED)
    #define LDC1_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End LDC1_TX_ENABLED */

#if(LDC1_HD_ENABLED)
    #define LDC1_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End LDC1_HD_ENABLED */

#if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
    #define LDC1_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define LDC1_initvar                    LDC1_initVar

#define LDC1_RX_Enabled                 LDC1_RX_ENABLED
#define LDC1_TX_Enabled                 LDC1_TX_ENABLED
#define LDC1_HD_Enabled                 LDC1_HD_ENABLED
#define LDC1_RX_IntInterruptEnabled     LDC1_RX_INTERRUPT_ENABLED
#define LDC1_TX_IntInterruptEnabled     LDC1_TX_INTERRUPT_ENABLED
#define LDC1_InternalClockUsed          LDC1_INTERNAL_CLOCK_USED
#define LDC1_RXHW_Address_Enabled       LDC1_RXHW_ADDRESS_ENABLED
#define LDC1_OverSampleCount            LDC1_OVER_SAMPLE_COUNT
#define LDC1_ParityType                 LDC1_PARITY_TYPE

#if( LDC1_TX_ENABLED && (LDC1_TXBUFFERSIZE > LDC1_FIFO_LENGTH))
    #define LDC1_TXBUFFER               LDC1_txBuffer
    #define LDC1_TXBUFFERREAD           LDC1_txBufferRead
    #define LDC1_TXBUFFERWRITE          LDC1_txBufferWrite
#endif /* End LDC1_TX_ENABLED */
#if( ( LDC1_RX_ENABLED || LDC1_HD_ENABLED ) && \
     (LDC1_RXBUFFERSIZE > LDC1_FIFO_LENGTH) )
    #define LDC1_RXBUFFER               LDC1_rxBuffer
    #define LDC1_RXBUFFERREAD           LDC1_rxBufferRead
    #define LDC1_RXBUFFERWRITE          LDC1_rxBufferWrite
    #define LDC1_RXBUFFERLOOPDETECT     LDC1_rxBufferLoopDetect
    #define LDC1_RXBUFFER_OVERFLOW      LDC1_rxBufferOverflow
#endif /* End LDC1_RX_ENABLED */

#ifdef LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define LDC1_CONTROL                LDC1_CONTROL_REG
#endif /* End LDC1_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(LDC1_TX_ENABLED)
    #define LDC1_TXDATA                 LDC1_TXDATA_REG
    #define LDC1_TXSTATUS               LDC1_TXSTATUS_REG
    #define LDC1_TXSTATUS_MASK          LDC1_TXSTATUS_MASK_REG
    #define LDC1_TXSTATUS_ACTL          LDC1_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(LDC1_TXCLKGEN_DP)
        #define LDC1_TXBITCLKGEN_CTR        LDC1_TXBITCLKGEN_CTR_REG
        #define LDC1_TXBITCLKTX_COMPLETE    LDC1_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define LDC1_TXBITCTR_PERIOD        LDC1_TXBITCTR_PERIOD_REG
        #define LDC1_TXBITCTR_CONTROL       LDC1_TXBITCTR_CONTROL_REG
        #define LDC1_TXBITCTR_COUNTER       LDC1_TXBITCTR_COUNTER_REG
    #endif /* LDC1_TXCLKGEN_DP */
#endif /* End LDC1_TX_ENABLED */

#if(LDC1_HD_ENABLED)
    #define LDC1_TXDATA                 LDC1_TXDATA_REG
    #define LDC1_TXSTATUS               LDC1_TXSTATUS_REG
    #define LDC1_TXSTATUS_MASK          LDC1_TXSTATUS_MASK_REG
    #define LDC1_TXSTATUS_ACTL          LDC1_TXSTATUS_ACTL_REG
#endif /* End LDC1_HD_ENABLED */

#if( (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) )
    #define LDC1_RXDATA                 LDC1_RXDATA_REG
    #define LDC1_RXADDRESS1             LDC1_RXADDRESS1_REG
    #define LDC1_RXADDRESS2             LDC1_RXADDRESS2_REG
    #define LDC1_RXBITCTR_PERIOD        LDC1_RXBITCTR_PERIOD_REG
    #define LDC1_RXBITCTR_CONTROL       LDC1_RXBITCTR_CONTROL_REG
    #define LDC1_RXBITCTR_COUNTER       LDC1_RXBITCTR_COUNTER_REG
    #define LDC1_RXSTATUS               LDC1_RXSTATUS_REG
    #define LDC1_RXSTATUS_MASK          LDC1_RXSTATUS_MASK_REG
    #define LDC1_RXSTATUS_ACTL          LDC1_RXSTATUS_ACTL_REG
#endif /* End  (LDC1_RX_ENABLED) || (LDC1_HD_ENABLED) */

#if(LDC1_INTERNAL_CLOCK_USED)
    #define LDC1_INTCLOCK_CLKEN         LDC1_INTCLOCK_CLKEN_REG
#endif /* End LDC1_INTERNAL_CLOCK_USED */

#define LDC1_WAIT_FOR_COMLETE_REINIT    LDC1_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_LDC1_H */


/* [] END OF FILE */
