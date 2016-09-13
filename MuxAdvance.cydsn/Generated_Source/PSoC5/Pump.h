/*******************************************************************************
* File Name: Pump.h
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


#if !defined(CY_UART_Pump_H)
#define CY_UART_Pump_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Pump_RX_ENABLED                     (1u)
#define Pump_TX_ENABLED                     (1u)
#define Pump_HD_ENABLED                     (0u)
#define Pump_RX_INTERRUPT_ENABLED           (1u)
#define Pump_TX_INTERRUPT_ENABLED           (0u)
#define Pump_INTERNAL_CLOCK_USED            (0u)
#define Pump_RXHW_ADDRESS_ENABLED           (0u)
#define Pump_OVER_SAMPLE_COUNT              (8u)
#define Pump_PARITY_TYPE                    (1u)
#define Pump_PARITY_TYPE_SW                 (0u)
#define Pump_BREAK_DETECT                   (0u)
#define Pump_BREAK_BITS_TX                  (13u)
#define Pump_BREAK_BITS_RX                  (13u)
#define Pump_TXCLKGEN_DP                    (1u)
#define Pump_USE23POLLING                   (1u)
#define Pump_FLOW_CONTROL                   (0u)
#define Pump_CLK_FREQ                       (0u)
#define Pump_TXBUFFERSIZE                   (4u)
#define Pump_RXBUFFERSIZE                   (256u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_CONTROL_REG_REMOVED            (0u)
#else
    #define Pump_CONTROL_REG_REMOVED            (1u)
#endif /* End Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Pump_backupStruct_
{
    uint8 enableState;

    #if(Pump_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Pump_CONTROL_REG_REMOVED */
    #if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Pump_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Pump_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Pump_RX_ENABLED) || (Pump_HD_ENABLED)*/

    #if(Pump_TX_ENABLED)
        #if(Pump_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Pump_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Pump_TX_ENABLED */
} Pump_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Pump_Start(void) ;
void Pump_Stop(void) ;
uint8 Pump_ReadControlRegister(void) ;
void Pump_WriteControlRegister(uint8 control) ;

void Pump_Init(void) ;
void Pump_Enable(void) ;
void Pump_SaveConfig(void) ;
void Pump_RestoreConfig(void) ;
void Pump_Sleep(void) ;
void Pump_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )

    #if(Pump_RX_INTERRUPT_ENABLED)
        void  Pump_EnableRxInt(void) ;
        void  Pump_DisableRxInt(void) ;
        CY_ISR_PROTO(Pump_RXISR);
    #endif /* Pump_RX_INTERRUPT_ENABLED */

    void Pump_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Pump_SetRxAddress1(uint8 address) ;
    void Pump_SetRxAddress2(uint8 address) ;

    void  Pump_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Pump_ReadRxData(void) ;
    uint8 Pump_ReadRxStatus(void) ;
    uint8 Pump_GetChar(void) ;
    uint16 Pump_GetByte(void) ;
    uint16 Pump_GetRxBufferSize(void)
                                                            ;
    void Pump_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Pump_GetRxInterruptSource   Pump_ReadRxStatus

#endif /* End (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */

/* Only if TX is enabled */
#if(Pump_TX_ENABLED || Pump_HD_ENABLED)

    #if(Pump_TX_INTERRUPT_ENABLED)
        void Pump_EnableTxInt(void) ;
        void Pump_DisableTxInt(void) ;
        CY_ISR_PROTO(Pump_TXISR);
    #endif /* Pump_TX_INTERRUPT_ENABLED */

    void Pump_SetTxInterruptMode(uint8 intSrc) ;
    void Pump_WriteTxData(uint8 txDataByte) ;
    uint8 Pump_ReadTxStatus(void) ;
    void Pump_PutChar(uint8 txDataByte) ;
    void Pump_PutString(const char8 string[]) ;
    void Pump_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Pump_PutCRLF(uint8 txDataByte) ;
    void Pump_ClearTxBuffer(void) ;
    void Pump_SetTxAddressMode(uint8 addressMode) ;
    void Pump_SendBreak(uint8 retMode) ;
    uint8 Pump_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Pump_PutStringConst         Pump_PutString
    #define Pump_PutArrayConst          Pump_PutArray
    #define Pump_GetTxInterruptSource   Pump_ReadTxStatus

#endif /* End Pump_TX_ENABLED || Pump_HD_ENABLED */

#if(Pump_HD_ENABLED)
    void Pump_LoadRxConfig(void) ;
    void Pump_LoadTxConfig(void) ;
#endif /* End Pump_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Pump_CyBtldrCommStart(void) CYSMALL ;
    void    Pump_CyBtldrCommStop(void) CYSMALL ;
    void    Pump_CyBtldrCommReset(void) CYSMALL ;
    cystatus Pump_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Pump_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump)
        #define CyBtldrCommStart    Pump_CyBtldrCommStart
        #define CyBtldrCommStop     Pump_CyBtldrCommStop
        #define CyBtldrCommReset    Pump_CyBtldrCommReset
        #define CyBtldrCommWrite    Pump_CyBtldrCommWrite
        #define CyBtldrCommRead     Pump_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Pump_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Pump_SET_SPACE                              (0x00u)
#define Pump_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Pump_TX_ENABLED) || (Pump_HD_ENABLED) )
    #if(Pump_TX_INTERRUPT_ENABLED)
        #define Pump_TX_VECT_NUM            (uint8)Pump_TXInternalInterrupt__INTC_NUMBER
        #define Pump_TX_PRIOR_NUM           (uint8)Pump_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pump_TX_INTERRUPT_ENABLED */
    #if(Pump_TX_ENABLED)
        #define Pump_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pump_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pump_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Pump_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pump_TX_ENABLED */
    #if(Pump_HD_ENABLED)
        #define Pump_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pump_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pump_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Pump_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pump_HD_ENABLED */
    #define Pump_TX_STS_COMPLETE            (uint8)(0x01u << Pump_TX_STS_COMPLETE_SHIFT)
    #define Pump_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Pump_TX_STS_FIFO_EMPTY_SHIFT)
    #define Pump_TX_STS_FIFO_FULL           (uint8)(0x01u << Pump_TX_STS_FIFO_FULL_SHIFT)
    #define Pump_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Pump_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Pump_TX_ENABLED) || (Pump_HD_ENABLED)*/

#if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
    #if(Pump_RX_INTERRUPT_ENABLED)
        #define Pump_RX_VECT_NUM            (uint8)Pump_RXInternalInterrupt__INTC_NUMBER
        #define Pump_RX_PRIOR_NUM           (uint8)Pump_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pump_RX_INTERRUPT_ENABLED */
    #define Pump_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Pump_RX_STS_BREAK_SHIFT             (0x01u)
    #define Pump_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Pump_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Pump_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Pump_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Pump_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Pump_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Pump_RX_STS_MRKSPC           (uint8)(0x01u << Pump_RX_STS_MRKSPC_SHIFT)
    #define Pump_RX_STS_BREAK            (uint8)(0x01u << Pump_RX_STS_BREAK_SHIFT)
    #define Pump_RX_STS_PAR_ERROR        (uint8)(0x01u << Pump_RX_STS_PAR_ERROR_SHIFT)
    #define Pump_RX_STS_STOP_ERROR       (uint8)(0x01u << Pump_RX_STS_STOP_ERROR_SHIFT)
    #define Pump_RX_STS_OVERRUN          (uint8)(0x01u << Pump_RX_STS_OVERRUN_SHIFT)
    #define Pump_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Pump_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Pump_RX_STS_ADDR_MATCH       (uint8)(0x01u << Pump_RX_STS_ADDR_MATCH_SHIFT)
    #define Pump_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Pump_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Pump_RX_HW_MASK                     (0x7Fu)
#endif /* End (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */

/* Control Register definitions */
#define Pump_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Pump_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Pump_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Pump_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Pump_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Pump_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Pump_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Pump_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Pump_CTRL_HD_SEND               (uint8)(0x01u << Pump_CTRL_HD_SEND_SHIFT)
#define Pump_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Pump_CTRL_HD_SEND_BREAK_SHIFT)
#define Pump_CTRL_MARK                  (uint8)(0x01u << Pump_CTRL_MARK_SHIFT)
#define Pump_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Pump_CTRL_PARITY_TYPE0_SHIFT)
#define Pump_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Pump_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Pump_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Pump_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Pump_SEND_BREAK                         (0x00u)
#define Pump_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Pump_REINIT                             (0x02u)
#define Pump_SEND_WAIT_REINIT                   (0x03u)

#define Pump_OVER_SAMPLE_8                      (8u)
#define Pump_OVER_SAMPLE_16                     (16u)

#define Pump_BIT_CENTER                         (Pump_OVER_SAMPLE_COUNT - 1u)

#define Pump_FIFO_LENGTH                        (4u)
#define Pump_NUMBER_OF_START_BIT                (1u)
#define Pump_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Pump_TXBITCTR_BREAKBITS8X   ((Pump_BREAK_BITS_TX * Pump_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Pump_TXBITCTR_BREAKBITS ((Pump_BREAK_BITS_TX * Pump_OVER_SAMPLE_COUNT) - 1u)

#define Pump_HALF_BIT_COUNT   \
                            (((Pump_OVER_SAMPLE_COUNT / 2u) + (Pump_USE23POLLING * 1u)) - 2u)
#if (Pump_OVER_SAMPLE_COUNT == Pump_OVER_SAMPLE_8)
    #define Pump_HD_TXBITCTR_INIT   (((Pump_BREAK_BITS_TX + \
                            Pump_NUMBER_OF_START_BIT) * Pump_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Pump_RXBITCTR_INIT  ((((Pump_BREAK_BITS_RX + Pump_NUMBER_OF_START_BIT) \
                            * Pump_OVER_SAMPLE_COUNT) + Pump_HALF_BIT_COUNT) - 1u)


#else /* Pump_OVER_SAMPLE_COUNT == Pump_OVER_SAMPLE_16 */
    #define Pump_HD_TXBITCTR_INIT   ((8u * Pump_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Pump_RXBITCTR_INIT      (((7u * Pump_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Pump_HALF_BIT_COUNT)
#endif /* End Pump_OVER_SAMPLE_COUNT */
#define Pump_HD_RXBITCTR_INIT                   Pump_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Pump_initVar;
#if( Pump_TX_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
    extern volatile uint8 Pump_txBuffer[Pump_TXBUFFERSIZE];
    extern volatile uint8 Pump_txBufferRead;
    extern uint8 Pump_txBufferWrite;
#endif /* End Pump_TX_ENABLED */
#if( ( Pump_RX_ENABLED || Pump_HD_ENABLED ) && \
     (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH) )
    extern volatile uint8 Pump_rxBuffer[Pump_RXBUFFERSIZE];
    extern volatile uint16 Pump_rxBufferRead;
    extern volatile uint16 Pump_rxBufferWrite;
    extern volatile uint8 Pump_rxBufferLoopDetect;
    extern volatile uint8 Pump_rxBufferOverflow;
    #if (Pump_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Pump_rxAddressMode;
        extern volatile uint8 Pump_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Pump_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Pump__B_UART__AM_SW_BYTE_BYTE 1
#define Pump__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Pump__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Pump__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Pump__B_UART__AM_NONE 0

#define Pump__B_UART__NONE_REVB 0
#define Pump__B_UART__EVEN_REVB 1
#define Pump__B_UART__ODD_REVB 2
#define Pump__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Pump_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Pump_NUMBER_OF_STOP_BITS    (1u)

#if (Pump_RXHW_ADDRESS_ENABLED)
    #define Pump_RXADDRESSMODE      (0u)
    #define Pump_RXHWADDRESS1       (0u)
    #define Pump_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Pump_RXAddressMode      Pump_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Pump_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Pump_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Pump_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Pump_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Pump_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Pump_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Pump_RX_STS_BREAK_SHIFT) \
                                        | (0 << Pump_RX_STS_OVERRUN_SHIFT))

#define Pump_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Pump_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Pump_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Pump_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Pump_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_CONTROL_REG \
                            (* (reg8 *) Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Pump_CONTROL_PTR \
                            (  (reg8 *) Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pump_TX_ENABLED)
    #define Pump_TXDATA_REG          (* (reg8 *) Pump_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pump_TXDATA_PTR          (  (reg8 *) Pump_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pump_TXDATA_AUX_CTL_REG  (* (reg8 *) Pump_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_TXDATA_AUX_CTL_PTR  (  (reg8 *) Pump_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_TXSTATUS_REG        (* (reg8 *) Pump_BUART_sTX_TxSts__STATUS_REG)
    #define Pump_TXSTATUS_PTR        (  (reg8 *) Pump_BUART_sTX_TxSts__STATUS_REG)
    #define Pump_TXSTATUS_MASK_REG   (* (reg8 *) Pump_BUART_sTX_TxSts__MASK_REG)
    #define Pump_TXSTATUS_MASK_PTR   (  (reg8 *) Pump_BUART_sTX_TxSts__MASK_REG)
    #define Pump_TXSTATUS_ACTL_REG   (* (reg8 *) Pump_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Pump_TXSTATUS_ACTL_PTR   (  (reg8 *) Pump_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Pump_TXCLKGEN_DP)
        #define Pump_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pump_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pump_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Pump_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Pump_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pump_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pump_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pump_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pump_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Pump_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Pump_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Pump_TXCLKGEN_DP */

#endif /* End Pump_TX_ENABLED */

#if(Pump_HD_ENABLED)

    #define Pump_TXDATA_REG             (* (reg8 *) Pump_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pump_TXDATA_PTR             (  (reg8 *) Pump_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pump_TXDATA_AUX_CTL_REG     (* (reg8 *) Pump_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_TXDATA_AUX_CTL_PTR     (  (reg8 *) Pump_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pump_TXSTATUS_REG           (* (reg8 *) Pump_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_TXSTATUS_PTR           (  (reg8 *) Pump_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_TXSTATUS_MASK_REG      (* (reg8 *) Pump_BUART_sRX_RxSts__MASK_REG )
    #define Pump_TXSTATUS_MASK_PTR      (  (reg8 *) Pump_BUART_sRX_RxSts__MASK_REG )
    #define Pump_TXSTATUS_ACTL_REG      (* (reg8 *) Pump_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pump_TXSTATUS_ACTL_PTR      (  (reg8 *) Pump_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Pump_HD_ENABLED */

#if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
    #define Pump_RXDATA_REG             (* (reg8 *) Pump_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pump_RXDATA_PTR             (  (reg8 *) Pump_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pump_RXADDRESS1_REG         (* (reg8 *) Pump_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pump_RXADDRESS1_PTR         (  (reg8 *) Pump_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pump_RXADDRESS2_REG         (* (reg8 *) Pump_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pump_RXADDRESS2_PTR         (  (reg8 *) Pump_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pump_RXDATA_AUX_CTL_REG     (* (reg8 *) Pump_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pump_RXBITCTR_PERIOD_REG    (* (reg8 *) Pump_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pump_RXBITCTR_PERIOD_PTR    (  (reg8 *) Pump_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pump_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pump_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pump_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pump_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pump_RXBITCTR_COUNTER_REG   (* (reg8 *) Pump_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Pump_RXBITCTR_COUNTER_PTR   (  (reg8 *) Pump_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Pump_RXSTATUS_REG           (* (reg8 *) Pump_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_RXSTATUS_PTR           (  (reg8 *) Pump_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_RXSTATUS_MASK_REG      (* (reg8 *) Pump_BUART_sRX_RxSts__MASK_REG )
    #define Pump_RXSTATUS_MASK_PTR      (  (reg8 *) Pump_BUART_sRX_RxSts__MASK_REG )
    #define Pump_RXSTATUS_ACTL_REG      (* (reg8 *) Pump_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pump_RXSTATUS_ACTL_PTR      (  (reg8 *) Pump_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */

#if(Pump_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Pump_INTCLOCK_CLKEN_REG     (* (reg8 *) Pump_IntClock__PM_ACT_CFG)
    #define Pump_INTCLOCK_CLKEN_PTR     (  (reg8 *) Pump_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Pump_INTCLOCK_CLKEN_MASK    Pump_IntClock__PM_ACT_MSK
#endif /* End Pump_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Pump_TX_ENABLED)
    #define Pump_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Pump_TX_ENABLED */

#if(Pump_HD_ENABLED)
    #define Pump_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Pump_HD_ENABLED */

#if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
    #define Pump_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Pump_initvar                    Pump_initVar

#define Pump_RX_Enabled                 Pump_RX_ENABLED
#define Pump_TX_Enabled                 Pump_TX_ENABLED
#define Pump_HD_Enabled                 Pump_HD_ENABLED
#define Pump_RX_IntInterruptEnabled     Pump_RX_INTERRUPT_ENABLED
#define Pump_TX_IntInterruptEnabled     Pump_TX_INTERRUPT_ENABLED
#define Pump_InternalClockUsed          Pump_INTERNAL_CLOCK_USED
#define Pump_RXHW_Address_Enabled       Pump_RXHW_ADDRESS_ENABLED
#define Pump_OverSampleCount            Pump_OVER_SAMPLE_COUNT
#define Pump_ParityType                 Pump_PARITY_TYPE

#if( Pump_TX_ENABLED && (Pump_TXBUFFERSIZE > Pump_FIFO_LENGTH))
    #define Pump_TXBUFFER               Pump_txBuffer
    #define Pump_TXBUFFERREAD           Pump_txBufferRead
    #define Pump_TXBUFFERWRITE          Pump_txBufferWrite
#endif /* End Pump_TX_ENABLED */
#if( ( Pump_RX_ENABLED || Pump_HD_ENABLED ) && \
     (Pump_RXBUFFERSIZE > Pump_FIFO_LENGTH) )
    #define Pump_RXBUFFER               Pump_rxBuffer
    #define Pump_RXBUFFERREAD           Pump_rxBufferRead
    #define Pump_RXBUFFERWRITE          Pump_rxBufferWrite
    #define Pump_RXBUFFERLOOPDETECT     Pump_rxBufferLoopDetect
    #define Pump_RXBUFFER_OVERFLOW      Pump_rxBufferOverflow
#endif /* End Pump_RX_ENABLED */

#ifdef Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_CONTROL                Pump_CONTROL_REG
#endif /* End Pump_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pump_TX_ENABLED)
    #define Pump_TXDATA                 Pump_TXDATA_REG
    #define Pump_TXSTATUS               Pump_TXSTATUS_REG
    #define Pump_TXSTATUS_MASK          Pump_TXSTATUS_MASK_REG
    #define Pump_TXSTATUS_ACTL          Pump_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Pump_TXCLKGEN_DP)
        #define Pump_TXBITCLKGEN_CTR        Pump_TXBITCLKGEN_CTR_REG
        #define Pump_TXBITCLKTX_COMPLETE    Pump_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Pump_TXBITCTR_PERIOD        Pump_TXBITCTR_PERIOD_REG
        #define Pump_TXBITCTR_CONTROL       Pump_TXBITCTR_CONTROL_REG
        #define Pump_TXBITCTR_COUNTER       Pump_TXBITCTR_COUNTER_REG
    #endif /* Pump_TXCLKGEN_DP */
#endif /* End Pump_TX_ENABLED */

#if(Pump_HD_ENABLED)
    #define Pump_TXDATA                 Pump_TXDATA_REG
    #define Pump_TXSTATUS               Pump_TXSTATUS_REG
    #define Pump_TXSTATUS_MASK          Pump_TXSTATUS_MASK_REG
    #define Pump_TXSTATUS_ACTL          Pump_TXSTATUS_ACTL_REG
#endif /* End Pump_HD_ENABLED */

#if( (Pump_RX_ENABLED) || (Pump_HD_ENABLED) )
    #define Pump_RXDATA                 Pump_RXDATA_REG
    #define Pump_RXADDRESS1             Pump_RXADDRESS1_REG
    #define Pump_RXADDRESS2             Pump_RXADDRESS2_REG
    #define Pump_RXBITCTR_PERIOD        Pump_RXBITCTR_PERIOD_REG
    #define Pump_RXBITCTR_CONTROL       Pump_RXBITCTR_CONTROL_REG
    #define Pump_RXBITCTR_COUNTER       Pump_RXBITCTR_COUNTER_REG
    #define Pump_RXSTATUS               Pump_RXSTATUS_REG
    #define Pump_RXSTATUS_MASK          Pump_RXSTATUS_MASK_REG
    #define Pump_RXSTATUS_ACTL          Pump_RXSTATUS_ACTL_REG
#endif /* End  (Pump_RX_ENABLED) || (Pump_HD_ENABLED) */

#if(Pump_INTERNAL_CLOCK_USED)
    #define Pump_INTCLOCK_CLKEN         Pump_INTCLOCK_CLKEN_REG
#endif /* End Pump_INTERNAL_CLOCK_USED */

#define Pump_WAIT_FOR_COMLETE_REINIT    Pump_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Pump_H */


/* [] END OF FILE */
