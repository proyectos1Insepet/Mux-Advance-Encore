/*******************************************************************************
* File Name: Pistola.h
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


#if !defined(CY_UART_Pistola_H)
#define CY_UART_Pistola_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Pistola_RX_ENABLED                     (1u)
#define Pistola_TX_ENABLED                     (1u)
#define Pistola_HD_ENABLED                     (0u)
#define Pistola_RX_INTERRUPT_ENABLED           (1u)
#define Pistola_TX_INTERRUPT_ENABLED           (0u)
#define Pistola_INTERNAL_CLOCK_USED            (1u)
#define Pistola_RXHW_ADDRESS_ENABLED           (0u)
#define Pistola_OVER_SAMPLE_COUNT              (8u)
#define Pistola_PARITY_TYPE                    (0u)
#define Pistola_PARITY_TYPE_SW                 (0u)
#define Pistola_BREAK_DETECT                   (0u)
#define Pistola_BREAK_BITS_TX                  (13u)
#define Pistola_BREAK_BITS_RX                  (13u)
#define Pistola_TXCLKGEN_DP                    (1u)
#define Pistola_USE23POLLING                   (1u)
#define Pistola_FLOW_CONTROL                   (0u)
#define Pistola_CLK_FREQ                       (0u)
#define Pistola_TXBUFFERSIZE                   (4u)
#define Pistola_RXBUFFERSIZE                   (100u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pistola_CONTROL_REG_REMOVED            (0u)
#else
    #define Pistola_CONTROL_REG_REMOVED            (1u)
#endif /* End Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Pistola_backupStruct_
{
    uint8 enableState;

    #if(Pistola_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Pistola_CONTROL_REG_REMOVED */
    #if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Pistola_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Pistola_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED)*/

    #if(Pistola_TX_ENABLED)
        #if(Pistola_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Pistola_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Pistola_TX_ENABLED */
} Pistola_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Pistola_Start(void) ;
void Pistola_Stop(void) ;
uint8 Pistola_ReadControlRegister(void) ;
void Pistola_WriteControlRegister(uint8 control) ;

void Pistola_Init(void) ;
void Pistola_Enable(void) ;
void Pistola_SaveConfig(void) ;
void Pistola_RestoreConfig(void) ;
void Pistola_Sleep(void) ;
void Pistola_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )

    #if(Pistola_RX_INTERRUPT_ENABLED)
        void  Pistola_EnableRxInt(void) ;
        void  Pistola_DisableRxInt(void) ;
        CY_ISR_PROTO(Pistola_RXISR);
    #endif /* Pistola_RX_INTERRUPT_ENABLED */

    void Pistola_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Pistola_SetRxAddress1(uint8 address) ;
    void Pistola_SetRxAddress2(uint8 address) ;

    void  Pistola_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Pistola_ReadRxData(void) ;
    uint8 Pistola_ReadRxStatus(void) ;
    uint8 Pistola_GetChar(void) ;
    uint16 Pistola_GetByte(void) ;
    uint8 Pistola_GetRxBufferSize(void)
                                                            ;
    void Pistola_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Pistola_GetRxInterruptSource   Pistola_ReadRxStatus

#endif /* End (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */

/* Only if TX is enabled */
#if(Pistola_TX_ENABLED || Pistola_HD_ENABLED)

    #if(Pistola_TX_INTERRUPT_ENABLED)
        void Pistola_EnableTxInt(void) ;
        void Pistola_DisableTxInt(void) ;
        CY_ISR_PROTO(Pistola_TXISR);
    #endif /* Pistola_TX_INTERRUPT_ENABLED */

    void Pistola_SetTxInterruptMode(uint8 intSrc) ;
    void Pistola_WriteTxData(uint8 txDataByte) ;
    uint8 Pistola_ReadTxStatus(void) ;
    void Pistola_PutChar(uint8 txDataByte) ;
    void Pistola_PutString(const char8 string[]) ;
    void Pistola_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Pistola_PutCRLF(uint8 txDataByte) ;
    void Pistola_ClearTxBuffer(void) ;
    void Pistola_SetTxAddressMode(uint8 addressMode) ;
    void Pistola_SendBreak(uint8 retMode) ;
    uint8 Pistola_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Pistola_PutStringConst         Pistola_PutString
    #define Pistola_PutArrayConst          Pistola_PutArray
    #define Pistola_GetTxInterruptSource   Pistola_ReadTxStatus

#endif /* End Pistola_TX_ENABLED || Pistola_HD_ENABLED */

#if(Pistola_HD_ENABLED)
    void Pistola_LoadRxConfig(void) ;
    void Pistola_LoadTxConfig(void) ;
#endif /* End Pistola_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pistola) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Pistola_CyBtldrCommStart(void) CYSMALL ;
    void    Pistola_CyBtldrCommStop(void) CYSMALL ;
    void    Pistola_CyBtldrCommReset(void) CYSMALL ;
    cystatus Pistola_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Pistola_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pistola)
        #define CyBtldrCommStart    Pistola_CyBtldrCommStart
        #define CyBtldrCommStop     Pistola_CyBtldrCommStop
        #define CyBtldrCommReset    Pistola_CyBtldrCommReset
        #define CyBtldrCommWrite    Pistola_CyBtldrCommWrite
        #define CyBtldrCommRead     Pistola_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pistola) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Pistola_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Pistola_SET_SPACE                              (0x00u)
#define Pistola_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Pistola_TX_ENABLED) || (Pistola_HD_ENABLED) )
    #if(Pistola_TX_INTERRUPT_ENABLED)
        #define Pistola_TX_VECT_NUM            (uint8)Pistola_TXInternalInterrupt__INTC_NUMBER
        #define Pistola_TX_PRIOR_NUM           (uint8)Pistola_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pistola_TX_INTERRUPT_ENABLED */
    #if(Pistola_TX_ENABLED)
        #define Pistola_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pistola_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pistola_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Pistola_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pistola_TX_ENABLED */
    #if(Pistola_HD_ENABLED)
        #define Pistola_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pistola_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pistola_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Pistola_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pistola_HD_ENABLED */
    #define Pistola_TX_STS_COMPLETE            (uint8)(0x01u << Pistola_TX_STS_COMPLETE_SHIFT)
    #define Pistola_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Pistola_TX_STS_FIFO_EMPTY_SHIFT)
    #define Pistola_TX_STS_FIFO_FULL           (uint8)(0x01u << Pistola_TX_STS_FIFO_FULL_SHIFT)
    #define Pistola_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Pistola_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Pistola_TX_ENABLED) || (Pistola_HD_ENABLED)*/

#if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
    #if(Pistola_RX_INTERRUPT_ENABLED)
        #define Pistola_RX_VECT_NUM            (uint8)Pistola_RXInternalInterrupt__INTC_NUMBER
        #define Pistola_RX_PRIOR_NUM           (uint8)Pistola_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pistola_RX_INTERRUPT_ENABLED */
    #define Pistola_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Pistola_RX_STS_BREAK_SHIFT             (0x01u)
    #define Pistola_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Pistola_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Pistola_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Pistola_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Pistola_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Pistola_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Pistola_RX_STS_MRKSPC           (uint8)(0x01u << Pistola_RX_STS_MRKSPC_SHIFT)
    #define Pistola_RX_STS_BREAK            (uint8)(0x01u << Pistola_RX_STS_BREAK_SHIFT)
    #define Pistola_RX_STS_PAR_ERROR        (uint8)(0x01u << Pistola_RX_STS_PAR_ERROR_SHIFT)
    #define Pistola_RX_STS_STOP_ERROR       (uint8)(0x01u << Pistola_RX_STS_STOP_ERROR_SHIFT)
    #define Pistola_RX_STS_OVERRUN          (uint8)(0x01u << Pistola_RX_STS_OVERRUN_SHIFT)
    #define Pistola_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Pistola_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Pistola_RX_STS_ADDR_MATCH       (uint8)(0x01u << Pistola_RX_STS_ADDR_MATCH_SHIFT)
    #define Pistola_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Pistola_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Pistola_RX_HW_MASK                     (0x7Fu)
#endif /* End (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */

/* Control Register definitions */
#define Pistola_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Pistola_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Pistola_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Pistola_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Pistola_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Pistola_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Pistola_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Pistola_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Pistola_CTRL_HD_SEND               (uint8)(0x01u << Pistola_CTRL_HD_SEND_SHIFT)
#define Pistola_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Pistola_CTRL_HD_SEND_BREAK_SHIFT)
#define Pistola_CTRL_MARK                  (uint8)(0x01u << Pistola_CTRL_MARK_SHIFT)
#define Pistola_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Pistola_CTRL_PARITY_TYPE0_SHIFT)
#define Pistola_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Pistola_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Pistola_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Pistola_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Pistola_SEND_BREAK                         (0x00u)
#define Pistola_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Pistola_REINIT                             (0x02u)
#define Pistola_SEND_WAIT_REINIT                   (0x03u)

#define Pistola_OVER_SAMPLE_8                      (8u)
#define Pistola_OVER_SAMPLE_16                     (16u)

#define Pistola_BIT_CENTER                         (Pistola_OVER_SAMPLE_COUNT - 1u)

#define Pistola_FIFO_LENGTH                        (4u)
#define Pistola_NUMBER_OF_START_BIT                (1u)
#define Pistola_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Pistola_TXBITCTR_BREAKBITS8X   ((Pistola_BREAK_BITS_TX * Pistola_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Pistola_TXBITCTR_BREAKBITS ((Pistola_BREAK_BITS_TX * Pistola_OVER_SAMPLE_COUNT) - 1u)

#define Pistola_HALF_BIT_COUNT   \
                            (((Pistola_OVER_SAMPLE_COUNT / 2u) + (Pistola_USE23POLLING * 1u)) - 2u)
#if (Pistola_OVER_SAMPLE_COUNT == Pistola_OVER_SAMPLE_8)
    #define Pistola_HD_TXBITCTR_INIT   (((Pistola_BREAK_BITS_TX + \
                            Pistola_NUMBER_OF_START_BIT) * Pistola_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Pistola_RXBITCTR_INIT  ((((Pistola_BREAK_BITS_RX + Pistola_NUMBER_OF_START_BIT) \
                            * Pistola_OVER_SAMPLE_COUNT) + Pistola_HALF_BIT_COUNT) - 1u)


#else /* Pistola_OVER_SAMPLE_COUNT == Pistola_OVER_SAMPLE_16 */
    #define Pistola_HD_TXBITCTR_INIT   ((8u * Pistola_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Pistola_RXBITCTR_INIT      (((7u * Pistola_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Pistola_HALF_BIT_COUNT)
#endif /* End Pistola_OVER_SAMPLE_COUNT */
#define Pistola_HD_RXBITCTR_INIT                   Pistola_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Pistola_initVar;
#if( Pistola_TX_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
    extern volatile uint8 Pistola_txBuffer[Pistola_TXBUFFERSIZE];
    extern volatile uint8 Pistola_txBufferRead;
    extern uint8 Pistola_txBufferWrite;
#endif /* End Pistola_TX_ENABLED */
#if( ( Pistola_RX_ENABLED || Pistola_HD_ENABLED ) && \
     (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH) )
    extern volatile uint8 Pistola_rxBuffer[Pistola_RXBUFFERSIZE];
    extern volatile uint8 Pistola_rxBufferRead;
    extern volatile uint8 Pistola_rxBufferWrite;
    extern volatile uint8 Pistola_rxBufferLoopDetect;
    extern volatile uint8 Pistola_rxBufferOverflow;
    #if (Pistola_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Pistola_rxAddressMode;
        extern volatile uint8 Pistola_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Pistola_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Pistola__B_UART__AM_SW_BYTE_BYTE 1
#define Pistola__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Pistola__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Pistola__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Pistola__B_UART__AM_NONE 0

#define Pistola__B_UART__NONE_REVB 0
#define Pistola__B_UART__EVEN_REVB 1
#define Pistola__B_UART__ODD_REVB 2
#define Pistola__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Pistola_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Pistola_NUMBER_OF_STOP_BITS    (1u)

#if (Pistola_RXHW_ADDRESS_ENABLED)
    #define Pistola_RXADDRESSMODE      (0u)
    #define Pistola_RXHWADDRESS1       (0u)
    #define Pistola_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Pistola_RXAddressMode      Pistola_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Pistola_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Pistola_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Pistola_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Pistola_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Pistola_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Pistola_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Pistola_RX_STS_BREAK_SHIFT) \
                                        | (0 << Pistola_RX_STS_OVERRUN_SHIFT))

#define Pistola_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Pistola_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Pistola_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Pistola_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Pistola_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pistola_CONTROL_REG \
                            (* (reg8 *) Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Pistola_CONTROL_PTR \
                            (  (reg8 *) Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pistola_TX_ENABLED)
    #define Pistola_TXDATA_REG          (* (reg8 *) Pistola_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pistola_TXDATA_PTR          (  (reg8 *) Pistola_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pistola_TXDATA_AUX_CTL_REG  (* (reg8 *) Pistola_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pistola_TXDATA_AUX_CTL_PTR  (  (reg8 *) Pistola_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pistola_TXSTATUS_REG        (* (reg8 *) Pistola_BUART_sTX_TxSts__STATUS_REG)
    #define Pistola_TXSTATUS_PTR        (  (reg8 *) Pistola_BUART_sTX_TxSts__STATUS_REG)
    #define Pistola_TXSTATUS_MASK_REG   (* (reg8 *) Pistola_BUART_sTX_TxSts__MASK_REG)
    #define Pistola_TXSTATUS_MASK_PTR   (  (reg8 *) Pistola_BUART_sTX_TxSts__MASK_REG)
    #define Pistola_TXSTATUS_ACTL_REG   (* (reg8 *) Pistola_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Pistola_TXSTATUS_ACTL_PTR   (  (reg8 *) Pistola_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Pistola_TXCLKGEN_DP)
        #define Pistola_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pistola_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pistola_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Pistola_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Pistola_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pistola_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pistola_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pistola_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pistola_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Pistola_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Pistola_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Pistola_TXCLKGEN_DP */

#endif /* End Pistola_TX_ENABLED */

#if(Pistola_HD_ENABLED)

    #define Pistola_TXDATA_REG             (* (reg8 *) Pistola_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pistola_TXDATA_PTR             (  (reg8 *) Pistola_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pistola_TXDATA_AUX_CTL_REG     (* (reg8 *) Pistola_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Pistola_TXDATA_AUX_CTL_PTR     (  (reg8 *) Pistola_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pistola_TXSTATUS_REG           (* (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_REG )
    #define Pistola_TXSTATUS_PTR           (  (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_REG )
    #define Pistola_TXSTATUS_MASK_REG      (* (reg8 *) Pistola_BUART_sRX_RxSts__MASK_REG )
    #define Pistola_TXSTATUS_MASK_PTR      (  (reg8 *) Pistola_BUART_sRX_RxSts__MASK_REG )
    #define Pistola_TXSTATUS_ACTL_REG      (* (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pistola_TXSTATUS_ACTL_PTR      (  (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Pistola_HD_ENABLED */

#if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
    #define Pistola_RXDATA_REG             (* (reg8 *) Pistola_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pistola_RXDATA_PTR             (  (reg8 *) Pistola_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pistola_RXADDRESS1_REG         (* (reg8 *) Pistola_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pistola_RXADDRESS1_PTR         (  (reg8 *) Pistola_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pistola_RXADDRESS2_REG         (* (reg8 *) Pistola_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pistola_RXADDRESS2_PTR         (  (reg8 *) Pistola_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pistola_RXDATA_AUX_CTL_REG     (* (reg8 *) Pistola_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pistola_RXBITCTR_PERIOD_REG    (* (reg8 *) Pistola_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pistola_RXBITCTR_PERIOD_PTR    (  (reg8 *) Pistola_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pistola_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pistola_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pistola_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pistola_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pistola_RXBITCTR_COUNTER_REG   (* (reg8 *) Pistola_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Pistola_RXBITCTR_COUNTER_PTR   (  (reg8 *) Pistola_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Pistola_RXSTATUS_REG           (* (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_REG )
    #define Pistola_RXSTATUS_PTR           (  (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_REG )
    #define Pistola_RXSTATUS_MASK_REG      (* (reg8 *) Pistola_BUART_sRX_RxSts__MASK_REG )
    #define Pistola_RXSTATUS_MASK_PTR      (  (reg8 *) Pistola_BUART_sRX_RxSts__MASK_REG )
    #define Pistola_RXSTATUS_ACTL_REG      (* (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pistola_RXSTATUS_ACTL_PTR      (  (reg8 *) Pistola_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */

#if(Pistola_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Pistola_INTCLOCK_CLKEN_REG     (* (reg8 *) Pistola_IntClock__PM_ACT_CFG)
    #define Pistola_INTCLOCK_CLKEN_PTR     (  (reg8 *) Pistola_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Pistola_INTCLOCK_CLKEN_MASK    Pistola_IntClock__PM_ACT_MSK
#endif /* End Pistola_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Pistola_TX_ENABLED)
    #define Pistola_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Pistola_TX_ENABLED */

#if(Pistola_HD_ENABLED)
    #define Pistola_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Pistola_HD_ENABLED */

#if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
    #define Pistola_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Pistola_initvar                    Pistola_initVar

#define Pistola_RX_Enabled                 Pistola_RX_ENABLED
#define Pistola_TX_Enabled                 Pistola_TX_ENABLED
#define Pistola_HD_Enabled                 Pistola_HD_ENABLED
#define Pistola_RX_IntInterruptEnabled     Pistola_RX_INTERRUPT_ENABLED
#define Pistola_TX_IntInterruptEnabled     Pistola_TX_INTERRUPT_ENABLED
#define Pistola_InternalClockUsed          Pistola_INTERNAL_CLOCK_USED
#define Pistola_RXHW_Address_Enabled       Pistola_RXHW_ADDRESS_ENABLED
#define Pistola_OverSampleCount            Pistola_OVER_SAMPLE_COUNT
#define Pistola_ParityType                 Pistola_PARITY_TYPE

#if( Pistola_TX_ENABLED && (Pistola_TXBUFFERSIZE > Pistola_FIFO_LENGTH))
    #define Pistola_TXBUFFER               Pistola_txBuffer
    #define Pistola_TXBUFFERREAD           Pistola_txBufferRead
    #define Pistola_TXBUFFERWRITE          Pistola_txBufferWrite
#endif /* End Pistola_TX_ENABLED */
#if( ( Pistola_RX_ENABLED || Pistola_HD_ENABLED ) && \
     (Pistola_RXBUFFERSIZE > Pistola_FIFO_LENGTH) )
    #define Pistola_RXBUFFER               Pistola_rxBuffer
    #define Pistola_RXBUFFERREAD           Pistola_rxBufferRead
    #define Pistola_RXBUFFERWRITE          Pistola_rxBufferWrite
    #define Pistola_RXBUFFERLOOPDETECT     Pistola_rxBufferLoopDetect
    #define Pistola_RXBUFFER_OVERFLOW      Pistola_rxBufferOverflow
#endif /* End Pistola_RX_ENABLED */

#ifdef Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pistola_CONTROL                Pistola_CONTROL_REG
#endif /* End Pistola_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pistola_TX_ENABLED)
    #define Pistola_TXDATA                 Pistola_TXDATA_REG
    #define Pistola_TXSTATUS               Pistola_TXSTATUS_REG
    #define Pistola_TXSTATUS_MASK          Pistola_TXSTATUS_MASK_REG
    #define Pistola_TXSTATUS_ACTL          Pistola_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Pistola_TXCLKGEN_DP)
        #define Pistola_TXBITCLKGEN_CTR        Pistola_TXBITCLKGEN_CTR_REG
        #define Pistola_TXBITCLKTX_COMPLETE    Pistola_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Pistola_TXBITCTR_PERIOD        Pistola_TXBITCTR_PERIOD_REG
        #define Pistola_TXBITCTR_CONTROL       Pistola_TXBITCTR_CONTROL_REG
        #define Pistola_TXBITCTR_COUNTER       Pistola_TXBITCTR_COUNTER_REG
    #endif /* Pistola_TXCLKGEN_DP */
#endif /* End Pistola_TX_ENABLED */

#if(Pistola_HD_ENABLED)
    #define Pistola_TXDATA                 Pistola_TXDATA_REG
    #define Pistola_TXSTATUS               Pistola_TXSTATUS_REG
    #define Pistola_TXSTATUS_MASK          Pistola_TXSTATUS_MASK_REG
    #define Pistola_TXSTATUS_ACTL          Pistola_TXSTATUS_ACTL_REG
#endif /* End Pistola_HD_ENABLED */

#if( (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) )
    #define Pistola_RXDATA                 Pistola_RXDATA_REG
    #define Pistola_RXADDRESS1             Pistola_RXADDRESS1_REG
    #define Pistola_RXADDRESS2             Pistola_RXADDRESS2_REG
    #define Pistola_RXBITCTR_PERIOD        Pistola_RXBITCTR_PERIOD_REG
    #define Pistola_RXBITCTR_CONTROL       Pistola_RXBITCTR_CONTROL_REG
    #define Pistola_RXBITCTR_COUNTER       Pistola_RXBITCTR_COUNTER_REG
    #define Pistola_RXSTATUS               Pistola_RXSTATUS_REG
    #define Pistola_RXSTATUS_MASK          Pistola_RXSTATUS_MASK_REG
    #define Pistola_RXSTATUS_ACTL          Pistola_RXSTATUS_ACTL_REG
#endif /* End  (Pistola_RX_ENABLED) || (Pistola_HD_ENABLED) */

#if(Pistola_INTERNAL_CLOCK_USED)
    #define Pistola_INTCLOCK_CLKEN         Pistola_INTCLOCK_CLKEN_REG
#endif /* End Pistola_INTERNAL_CLOCK_USED */

#define Pistola_WAIT_FOR_COMLETE_REINIT    Pistola_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Pistola_H */


/* [] END OF FILE */
