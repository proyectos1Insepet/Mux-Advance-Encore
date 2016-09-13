/*******************************************************************************
* File Name: Beagle.h
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


#if !defined(CY_UART_Beagle_H)
#define CY_UART_Beagle_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Beagle_RX_ENABLED                     (1u)
#define Beagle_TX_ENABLED                     (1u)
#define Beagle_HD_ENABLED                     (0u)
#define Beagle_RX_INTERRUPT_ENABLED           (1u)
#define Beagle_TX_INTERRUPT_ENABLED           (0u)
#define Beagle_INTERNAL_CLOCK_USED            (1u)
#define Beagle_RXHW_ADDRESS_ENABLED           (0u)
#define Beagle_OVER_SAMPLE_COUNT              (8u)
#define Beagle_PARITY_TYPE                    (0u)
#define Beagle_PARITY_TYPE_SW                 (0u)
#define Beagle_BREAK_DETECT                   (0u)
#define Beagle_BREAK_BITS_TX                  (13u)
#define Beagle_BREAK_BITS_RX                  (13u)
#define Beagle_TXCLKGEN_DP                    (1u)
#define Beagle_USE23POLLING                   (1u)
#define Beagle_FLOW_CONTROL                   (0u)
#define Beagle_CLK_FREQ                       (0u)
#define Beagle_TXBUFFERSIZE                   (4u)
#define Beagle_RXBUFFERSIZE                   (1500u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Beagle_CONTROL_REG_REMOVED            (0u)
#else
    #define Beagle_CONTROL_REG_REMOVED            (1u)
#endif /* End Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Beagle_backupStruct_
{
    uint8 enableState;

    #if(Beagle_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Beagle_CONTROL_REG_REMOVED */
    #if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Beagle_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Beagle_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED)*/

    #if(Beagle_TX_ENABLED)
        #if(Beagle_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Beagle_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Beagle_TX_ENABLED */
} Beagle_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Beagle_Start(void) ;
void Beagle_Stop(void) ;
uint8 Beagle_ReadControlRegister(void) ;
void Beagle_WriteControlRegister(uint8 control) ;

void Beagle_Init(void) ;
void Beagle_Enable(void) ;
void Beagle_SaveConfig(void) ;
void Beagle_RestoreConfig(void) ;
void Beagle_Sleep(void) ;
void Beagle_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )

    #if(Beagle_RX_INTERRUPT_ENABLED)
        void  Beagle_EnableRxInt(void) ;
        void  Beagle_DisableRxInt(void) ;
        CY_ISR_PROTO(Beagle_RXISR);
    #endif /* Beagle_RX_INTERRUPT_ENABLED */

    void Beagle_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Beagle_SetRxAddress1(uint8 address) ;
    void Beagle_SetRxAddress2(uint8 address) ;

    void  Beagle_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Beagle_ReadRxData(void) ;
    uint8 Beagle_ReadRxStatus(void) ;
    uint8 Beagle_GetChar(void) ;
    uint16 Beagle_GetByte(void) ;
    uint16 Beagle_GetRxBufferSize(void)
                                                            ;
    void Beagle_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Beagle_GetRxInterruptSource   Beagle_ReadRxStatus

#endif /* End (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */

/* Only if TX is enabled */
#if(Beagle_TX_ENABLED || Beagle_HD_ENABLED)

    #if(Beagle_TX_INTERRUPT_ENABLED)
        void Beagle_EnableTxInt(void) ;
        void Beagle_DisableTxInt(void) ;
        CY_ISR_PROTO(Beagle_TXISR);
    #endif /* Beagle_TX_INTERRUPT_ENABLED */

    void Beagle_SetTxInterruptMode(uint8 intSrc) ;
    void Beagle_WriteTxData(uint8 txDataByte) ;
    uint8 Beagle_ReadTxStatus(void) ;
    void Beagle_PutChar(uint8 txDataByte) ;
    void Beagle_PutString(const char8 string[]) ;
    void Beagle_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Beagle_PutCRLF(uint8 txDataByte) ;
    void Beagle_ClearTxBuffer(void) ;
    void Beagle_SetTxAddressMode(uint8 addressMode) ;
    void Beagle_SendBreak(uint8 retMode) ;
    uint8 Beagle_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Beagle_PutStringConst         Beagle_PutString
    #define Beagle_PutArrayConst          Beagle_PutArray
    #define Beagle_GetTxInterruptSource   Beagle_ReadTxStatus

#endif /* End Beagle_TX_ENABLED || Beagle_HD_ENABLED */

#if(Beagle_HD_ENABLED)
    void Beagle_LoadRxConfig(void) ;
    void Beagle_LoadTxConfig(void) ;
#endif /* End Beagle_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Beagle) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Beagle_CyBtldrCommStart(void) CYSMALL ;
    void    Beagle_CyBtldrCommStop(void) CYSMALL ;
    void    Beagle_CyBtldrCommReset(void) CYSMALL ;
    cystatus Beagle_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Beagle_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Beagle)
        #define CyBtldrCommStart    Beagle_CyBtldrCommStart
        #define CyBtldrCommStop     Beagle_CyBtldrCommStop
        #define CyBtldrCommReset    Beagle_CyBtldrCommReset
        #define CyBtldrCommWrite    Beagle_CyBtldrCommWrite
        #define CyBtldrCommRead     Beagle_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Beagle) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Beagle_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Beagle_SET_SPACE                              (0x00u)
#define Beagle_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Beagle_TX_ENABLED) || (Beagle_HD_ENABLED) )
    #if(Beagle_TX_INTERRUPT_ENABLED)
        #define Beagle_TX_VECT_NUM            (uint8)Beagle_TXInternalInterrupt__INTC_NUMBER
        #define Beagle_TX_PRIOR_NUM           (uint8)Beagle_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Beagle_TX_INTERRUPT_ENABLED */
    #if(Beagle_TX_ENABLED)
        #define Beagle_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Beagle_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Beagle_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Beagle_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Beagle_TX_ENABLED */
    #if(Beagle_HD_ENABLED)
        #define Beagle_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Beagle_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Beagle_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Beagle_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Beagle_HD_ENABLED */
    #define Beagle_TX_STS_COMPLETE            (uint8)(0x01u << Beagle_TX_STS_COMPLETE_SHIFT)
    #define Beagle_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Beagle_TX_STS_FIFO_EMPTY_SHIFT)
    #define Beagle_TX_STS_FIFO_FULL           (uint8)(0x01u << Beagle_TX_STS_FIFO_FULL_SHIFT)
    #define Beagle_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Beagle_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Beagle_TX_ENABLED) || (Beagle_HD_ENABLED)*/

#if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
    #if(Beagle_RX_INTERRUPT_ENABLED)
        #define Beagle_RX_VECT_NUM            (uint8)Beagle_RXInternalInterrupt__INTC_NUMBER
        #define Beagle_RX_PRIOR_NUM           (uint8)Beagle_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Beagle_RX_INTERRUPT_ENABLED */
    #define Beagle_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Beagle_RX_STS_BREAK_SHIFT             (0x01u)
    #define Beagle_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Beagle_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Beagle_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Beagle_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Beagle_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Beagle_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Beagle_RX_STS_MRKSPC           (uint8)(0x01u << Beagle_RX_STS_MRKSPC_SHIFT)
    #define Beagle_RX_STS_BREAK            (uint8)(0x01u << Beagle_RX_STS_BREAK_SHIFT)
    #define Beagle_RX_STS_PAR_ERROR        (uint8)(0x01u << Beagle_RX_STS_PAR_ERROR_SHIFT)
    #define Beagle_RX_STS_STOP_ERROR       (uint8)(0x01u << Beagle_RX_STS_STOP_ERROR_SHIFT)
    #define Beagle_RX_STS_OVERRUN          (uint8)(0x01u << Beagle_RX_STS_OVERRUN_SHIFT)
    #define Beagle_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Beagle_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Beagle_RX_STS_ADDR_MATCH       (uint8)(0x01u << Beagle_RX_STS_ADDR_MATCH_SHIFT)
    #define Beagle_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Beagle_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Beagle_RX_HW_MASK                     (0x7Fu)
#endif /* End (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */

/* Control Register definitions */
#define Beagle_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Beagle_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Beagle_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Beagle_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Beagle_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Beagle_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Beagle_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Beagle_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Beagle_CTRL_HD_SEND               (uint8)(0x01u << Beagle_CTRL_HD_SEND_SHIFT)
#define Beagle_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Beagle_CTRL_HD_SEND_BREAK_SHIFT)
#define Beagle_CTRL_MARK                  (uint8)(0x01u << Beagle_CTRL_MARK_SHIFT)
#define Beagle_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Beagle_CTRL_PARITY_TYPE0_SHIFT)
#define Beagle_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Beagle_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Beagle_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Beagle_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Beagle_SEND_BREAK                         (0x00u)
#define Beagle_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Beagle_REINIT                             (0x02u)
#define Beagle_SEND_WAIT_REINIT                   (0x03u)

#define Beagle_OVER_SAMPLE_8                      (8u)
#define Beagle_OVER_SAMPLE_16                     (16u)

#define Beagle_BIT_CENTER                         (Beagle_OVER_SAMPLE_COUNT - 1u)

#define Beagle_FIFO_LENGTH                        (4u)
#define Beagle_NUMBER_OF_START_BIT                (1u)
#define Beagle_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Beagle_TXBITCTR_BREAKBITS8X   ((Beagle_BREAK_BITS_TX * Beagle_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Beagle_TXBITCTR_BREAKBITS ((Beagle_BREAK_BITS_TX * Beagle_OVER_SAMPLE_COUNT) - 1u)

#define Beagle_HALF_BIT_COUNT   \
                            (((Beagle_OVER_SAMPLE_COUNT / 2u) + (Beagle_USE23POLLING * 1u)) - 2u)
#if (Beagle_OVER_SAMPLE_COUNT == Beagle_OVER_SAMPLE_8)
    #define Beagle_HD_TXBITCTR_INIT   (((Beagle_BREAK_BITS_TX + \
                            Beagle_NUMBER_OF_START_BIT) * Beagle_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Beagle_RXBITCTR_INIT  ((((Beagle_BREAK_BITS_RX + Beagle_NUMBER_OF_START_BIT) \
                            * Beagle_OVER_SAMPLE_COUNT) + Beagle_HALF_BIT_COUNT) - 1u)


#else /* Beagle_OVER_SAMPLE_COUNT == Beagle_OVER_SAMPLE_16 */
    #define Beagle_HD_TXBITCTR_INIT   ((8u * Beagle_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Beagle_RXBITCTR_INIT      (((7u * Beagle_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Beagle_HALF_BIT_COUNT)
#endif /* End Beagle_OVER_SAMPLE_COUNT */
#define Beagle_HD_RXBITCTR_INIT                   Beagle_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Beagle_initVar;
#if( Beagle_TX_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
    extern volatile uint8 Beagle_txBuffer[Beagle_TXBUFFERSIZE];
    extern volatile uint8 Beagle_txBufferRead;
    extern uint8 Beagle_txBufferWrite;
#endif /* End Beagle_TX_ENABLED */
#if( ( Beagle_RX_ENABLED || Beagle_HD_ENABLED ) && \
     (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH) )
    extern volatile uint8 Beagle_rxBuffer[Beagle_RXBUFFERSIZE];
    extern volatile uint16 Beagle_rxBufferRead;
    extern volatile uint16 Beagle_rxBufferWrite;
    extern volatile uint8 Beagle_rxBufferLoopDetect;
    extern volatile uint8 Beagle_rxBufferOverflow;
    #if (Beagle_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Beagle_rxAddressMode;
        extern volatile uint8 Beagle_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Beagle_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Beagle__B_UART__AM_SW_BYTE_BYTE 1
#define Beagle__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Beagle__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Beagle__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Beagle__B_UART__AM_NONE 0

#define Beagle__B_UART__NONE_REVB 0
#define Beagle__B_UART__EVEN_REVB 1
#define Beagle__B_UART__ODD_REVB 2
#define Beagle__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Beagle_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Beagle_NUMBER_OF_STOP_BITS    (1u)

#if (Beagle_RXHW_ADDRESS_ENABLED)
    #define Beagle_RXADDRESSMODE      (0u)
    #define Beagle_RXHWADDRESS1       (0u)
    #define Beagle_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Beagle_RXAddressMode      Beagle_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Beagle_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Beagle_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Beagle_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Beagle_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Beagle_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Beagle_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Beagle_RX_STS_BREAK_SHIFT) \
                                        | (0 << Beagle_RX_STS_OVERRUN_SHIFT))

#define Beagle_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Beagle_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Beagle_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Beagle_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Beagle_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Beagle_CONTROL_REG \
                            (* (reg8 *) Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Beagle_CONTROL_PTR \
                            (  (reg8 *) Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Beagle_TX_ENABLED)
    #define Beagle_TXDATA_REG          (* (reg8 *) Beagle_BUART_sTX_TxShifter_u0__F0_REG)
    #define Beagle_TXDATA_PTR          (  (reg8 *) Beagle_BUART_sTX_TxShifter_u0__F0_REG)
    #define Beagle_TXDATA_AUX_CTL_REG  (* (reg8 *) Beagle_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Beagle_TXDATA_AUX_CTL_PTR  (  (reg8 *) Beagle_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Beagle_TXSTATUS_REG        (* (reg8 *) Beagle_BUART_sTX_TxSts__STATUS_REG)
    #define Beagle_TXSTATUS_PTR        (  (reg8 *) Beagle_BUART_sTX_TxSts__STATUS_REG)
    #define Beagle_TXSTATUS_MASK_REG   (* (reg8 *) Beagle_BUART_sTX_TxSts__MASK_REG)
    #define Beagle_TXSTATUS_MASK_PTR   (  (reg8 *) Beagle_BUART_sTX_TxSts__MASK_REG)
    #define Beagle_TXSTATUS_ACTL_REG   (* (reg8 *) Beagle_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Beagle_TXSTATUS_ACTL_PTR   (  (reg8 *) Beagle_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Beagle_TXCLKGEN_DP)
        #define Beagle_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Beagle_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Beagle_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Beagle_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Beagle_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Beagle_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Beagle_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Beagle_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Beagle_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Beagle_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Beagle_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Beagle_TXCLKGEN_DP */

#endif /* End Beagle_TX_ENABLED */

#if(Beagle_HD_ENABLED)

    #define Beagle_TXDATA_REG             (* (reg8 *) Beagle_BUART_sRX_RxShifter_u0__F1_REG )
    #define Beagle_TXDATA_PTR             (  (reg8 *) Beagle_BUART_sRX_RxShifter_u0__F1_REG )
    #define Beagle_TXDATA_AUX_CTL_REG     (* (reg8 *) Beagle_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Beagle_TXDATA_AUX_CTL_PTR     (  (reg8 *) Beagle_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Beagle_TXSTATUS_REG           (* (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_REG )
    #define Beagle_TXSTATUS_PTR           (  (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_REG )
    #define Beagle_TXSTATUS_MASK_REG      (* (reg8 *) Beagle_BUART_sRX_RxSts__MASK_REG )
    #define Beagle_TXSTATUS_MASK_PTR      (  (reg8 *) Beagle_BUART_sRX_RxSts__MASK_REG )
    #define Beagle_TXSTATUS_ACTL_REG      (* (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Beagle_TXSTATUS_ACTL_PTR      (  (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Beagle_HD_ENABLED */

#if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
    #define Beagle_RXDATA_REG             (* (reg8 *) Beagle_BUART_sRX_RxShifter_u0__F0_REG )
    #define Beagle_RXDATA_PTR             (  (reg8 *) Beagle_BUART_sRX_RxShifter_u0__F0_REG )
    #define Beagle_RXADDRESS1_REG         (* (reg8 *) Beagle_BUART_sRX_RxShifter_u0__D0_REG )
    #define Beagle_RXADDRESS1_PTR         (  (reg8 *) Beagle_BUART_sRX_RxShifter_u0__D0_REG )
    #define Beagle_RXADDRESS2_REG         (* (reg8 *) Beagle_BUART_sRX_RxShifter_u0__D1_REG )
    #define Beagle_RXADDRESS2_PTR         (  (reg8 *) Beagle_BUART_sRX_RxShifter_u0__D1_REG )
    #define Beagle_RXDATA_AUX_CTL_REG     (* (reg8 *) Beagle_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Beagle_RXBITCTR_PERIOD_REG    (* (reg8 *) Beagle_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Beagle_RXBITCTR_PERIOD_PTR    (  (reg8 *) Beagle_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Beagle_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Beagle_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Beagle_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Beagle_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Beagle_RXBITCTR_COUNTER_REG   (* (reg8 *) Beagle_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Beagle_RXBITCTR_COUNTER_PTR   (  (reg8 *) Beagle_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Beagle_RXSTATUS_REG           (* (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_REG )
    #define Beagle_RXSTATUS_PTR           (  (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_REG )
    #define Beagle_RXSTATUS_MASK_REG      (* (reg8 *) Beagle_BUART_sRX_RxSts__MASK_REG )
    #define Beagle_RXSTATUS_MASK_PTR      (  (reg8 *) Beagle_BUART_sRX_RxSts__MASK_REG )
    #define Beagle_RXSTATUS_ACTL_REG      (* (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Beagle_RXSTATUS_ACTL_PTR      (  (reg8 *) Beagle_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */

#if(Beagle_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Beagle_INTCLOCK_CLKEN_REG     (* (reg8 *) Beagle_IntClock__PM_ACT_CFG)
    #define Beagle_INTCLOCK_CLKEN_PTR     (  (reg8 *) Beagle_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Beagle_INTCLOCK_CLKEN_MASK    Beagle_IntClock__PM_ACT_MSK
#endif /* End Beagle_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Beagle_TX_ENABLED)
    #define Beagle_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Beagle_TX_ENABLED */

#if(Beagle_HD_ENABLED)
    #define Beagle_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Beagle_HD_ENABLED */

#if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
    #define Beagle_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Beagle_initvar                    Beagle_initVar

#define Beagle_RX_Enabled                 Beagle_RX_ENABLED
#define Beagle_TX_Enabled                 Beagle_TX_ENABLED
#define Beagle_HD_Enabled                 Beagle_HD_ENABLED
#define Beagle_RX_IntInterruptEnabled     Beagle_RX_INTERRUPT_ENABLED
#define Beagle_TX_IntInterruptEnabled     Beagle_TX_INTERRUPT_ENABLED
#define Beagle_InternalClockUsed          Beagle_INTERNAL_CLOCK_USED
#define Beagle_RXHW_Address_Enabled       Beagle_RXHW_ADDRESS_ENABLED
#define Beagle_OverSampleCount            Beagle_OVER_SAMPLE_COUNT
#define Beagle_ParityType                 Beagle_PARITY_TYPE

#if( Beagle_TX_ENABLED && (Beagle_TXBUFFERSIZE > Beagle_FIFO_LENGTH))
    #define Beagle_TXBUFFER               Beagle_txBuffer
    #define Beagle_TXBUFFERREAD           Beagle_txBufferRead
    #define Beagle_TXBUFFERWRITE          Beagle_txBufferWrite
#endif /* End Beagle_TX_ENABLED */
#if( ( Beagle_RX_ENABLED || Beagle_HD_ENABLED ) && \
     (Beagle_RXBUFFERSIZE > Beagle_FIFO_LENGTH) )
    #define Beagle_RXBUFFER               Beagle_rxBuffer
    #define Beagle_RXBUFFERREAD           Beagle_rxBufferRead
    #define Beagle_RXBUFFERWRITE          Beagle_rxBufferWrite
    #define Beagle_RXBUFFERLOOPDETECT     Beagle_rxBufferLoopDetect
    #define Beagle_RXBUFFER_OVERFLOW      Beagle_rxBufferOverflow
#endif /* End Beagle_RX_ENABLED */

#ifdef Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Beagle_CONTROL                Beagle_CONTROL_REG
#endif /* End Beagle_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Beagle_TX_ENABLED)
    #define Beagle_TXDATA                 Beagle_TXDATA_REG
    #define Beagle_TXSTATUS               Beagle_TXSTATUS_REG
    #define Beagle_TXSTATUS_MASK          Beagle_TXSTATUS_MASK_REG
    #define Beagle_TXSTATUS_ACTL          Beagle_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Beagle_TXCLKGEN_DP)
        #define Beagle_TXBITCLKGEN_CTR        Beagle_TXBITCLKGEN_CTR_REG
        #define Beagle_TXBITCLKTX_COMPLETE    Beagle_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Beagle_TXBITCTR_PERIOD        Beagle_TXBITCTR_PERIOD_REG
        #define Beagle_TXBITCTR_CONTROL       Beagle_TXBITCTR_CONTROL_REG
        #define Beagle_TXBITCTR_COUNTER       Beagle_TXBITCTR_COUNTER_REG
    #endif /* Beagle_TXCLKGEN_DP */
#endif /* End Beagle_TX_ENABLED */

#if(Beagle_HD_ENABLED)
    #define Beagle_TXDATA                 Beagle_TXDATA_REG
    #define Beagle_TXSTATUS               Beagle_TXSTATUS_REG
    #define Beagle_TXSTATUS_MASK          Beagle_TXSTATUS_MASK_REG
    #define Beagle_TXSTATUS_ACTL          Beagle_TXSTATUS_ACTL_REG
#endif /* End Beagle_HD_ENABLED */

#if( (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) )
    #define Beagle_RXDATA                 Beagle_RXDATA_REG
    #define Beagle_RXADDRESS1             Beagle_RXADDRESS1_REG
    #define Beagle_RXADDRESS2             Beagle_RXADDRESS2_REG
    #define Beagle_RXBITCTR_PERIOD        Beagle_RXBITCTR_PERIOD_REG
    #define Beagle_RXBITCTR_CONTROL       Beagle_RXBITCTR_CONTROL_REG
    #define Beagle_RXBITCTR_COUNTER       Beagle_RXBITCTR_COUNTER_REG
    #define Beagle_RXSTATUS               Beagle_RXSTATUS_REG
    #define Beagle_RXSTATUS_MASK          Beagle_RXSTATUS_MASK_REG
    #define Beagle_RXSTATUS_ACTL          Beagle_RXSTATUS_ACTL_REG
#endif /* End  (Beagle_RX_ENABLED) || (Beagle_HD_ENABLED) */

#if(Beagle_INTERNAL_CLOCK_USED)
    #define Beagle_INTCLOCK_CLKEN         Beagle_INTCLOCK_CLKEN_REG
#endif /* End Beagle_INTERNAL_CLOCK_USED */

#define Beagle_WAIT_FOR_COMLETE_REINIT    Beagle_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Beagle_H */


/* [] END OF FILE */
