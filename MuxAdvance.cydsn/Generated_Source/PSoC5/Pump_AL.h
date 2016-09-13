/*******************************************************************************
* File Name: Pump_AL.h
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


#if !defined(CY_UART_Pump_AL_H)
#define CY_UART_Pump_AL_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Pump_AL_RX_ENABLED                     (1u)
#define Pump_AL_TX_ENABLED                     (1u)
#define Pump_AL_HD_ENABLED                     (0u)
#define Pump_AL_RX_INTERRUPT_ENABLED           (1u)
#define Pump_AL_TX_INTERRUPT_ENABLED           (0u)
#define Pump_AL_INTERNAL_CLOCK_USED            (0u)
#define Pump_AL_RXHW_ADDRESS_ENABLED           (0u)
#define Pump_AL_OVER_SAMPLE_COUNT              (8u)
#define Pump_AL_PARITY_TYPE                    (1u)
#define Pump_AL_PARITY_TYPE_SW                 (0u)
#define Pump_AL_BREAK_DETECT                   (0u)
#define Pump_AL_BREAK_BITS_TX                  (13u)
#define Pump_AL_BREAK_BITS_RX                  (13u)
#define Pump_AL_TXCLKGEN_DP                    (1u)
#define Pump_AL_USE23POLLING                   (1u)
#define Pump_AL_FLOW_CONTROL                   (0u)
#define Pump_AL_CLK_FREQ                       (0u)
#define Pump_AL_TXBUFFERSIZE                   (4u)
#define Pump_AL_RXBUFFERSIZE                   (256u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_AL_CONTROL_REG_REMOVED            (0u)
#else
    #define Pump_AL_CONTROL_REG_REMOVED            (1u)
#endif /* End Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Pump_AL_backupStruct_
{
    uint8 enableState;

    #if(Pump_AL_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Pump_AL_CONTROL_REG_REMOVED */
    #if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Pump_AL_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Pump_AL_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED)*/

    #if(Pump_AL_TX_ENABLED)
        #if(Pump_AL_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Pump_AL_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Pump_AL_TX_ENABLED */
} Pump_AL_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Pump_AL_Start(void) ;
void Pump_AL_Stop(void) ;
uint8 Pump_AL_ReadControlRegister(void) ;
void Pump_AL_WriteControlRegister(uint8 control) ;

void Pump_AL_Init(void) ;
void Pump_AL_Enable(void) ;
void Pump_AL_SaveConfig(void) ;
void Pump_AL_RestoreConfig(void) ;
void Pump_AL_Sleep(void) ;
void Pump_AL_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )

    #if(Pump_AL_RX_INTERRUPT_ENABLED)
        void  Pump_AL_EnableRxInt(void) ;
        void  Pump_AL_DisableRxInt(void) ;
        CY_ISR_PROTO(Pump_AL_RXISR);
    #endif /* Pump_AL_RX_INTERRUPT_ENABLED */

    void Pump_AL_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Pump_AL_SetRxAddress1(uint8 address) ;
    void Pump_AL_SetRxAddress2(uint8 address) ;

    void  Pump_AL_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Pump_AL_ReadRxData(void) ;
    uint8 Pump_AL_ReadRxStatus(void) ;
    uint8 Pump_AL_GetChar(void) ;
    uint16 Pump_AL_GetByte(void) ;
    uint16 Pump_AL_GetRxBufferSize(void)
                                                            ;
    void Pump_AL_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Pump_AL_GetRxInterruptSource   Pump_AL_ReadRxStatus

#endif /* End (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */

/* Only if TX is enabled */
#if(Pump_AL_TX_ENABLED || Pump_AL_HD_ENABLED)

    #if(Pump_AL_TX_INTERRUPT_ENABLED)
        void Pump_AL_EnableTxInt(void) ;
        void Pump_AL_DisableTxInt(void) ;
        CY_ISR_PROTO(Pump_AL_TXISR);
    #endif /* Pump_AL_TX_INTERRUPT_ENABLED */

    void Pump_AL_SetTxInterruptMode(uint8 intSrc) ;
    void Pump_AL_WriteTxData(uint8 txDataByte) ;
    uint8 Pump_AL_ReadTxStatus(void) ;
    void Pump_AL_PutChar(uint8 txDataByte) ;
    void Pump_AL_PutString(const char8 string[]) ;
    void Pump_AL_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Pump_AL_PutCRLF(uint8 txDataByte) ;
    void Pump_AL_ClearTxBuffer(void) ;
    void Pump_AL_SetTxAddressMode(uint8 addressMode) ;
    void Pump_AL_SendBreak(uint8 retMode) ;
    uint8 Pump_AL_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Pump_AL_PutStringConst         Pump_AL_PutString
    #define Pump_AL_PutArrayConst          Pump_AL_PutArray
    #define Pump_AL_GetTxInterruptSource   Pump_AL_ReadTxStatus

#endif /* End Pump_AL_TX_ENABLED || Pump_AL_HD_ENABLED */

#if(Pump_AL_HD_ENABLED)
    void Pump_AL_LoadRxConfig(void) ;
    void Pump_AL_LoadTxConfig(void) ;
#endif /* End Pump_AL_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump_AL) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Pump_AL_CyBtldrCommStart(void) CYSMALL ;
    void    Pump_AL_CyBtldrCommStop(void) CYSMALL ;
    void    Pump_AL_CyBtldrCommReset(void) CYSMALL ;
    cystatus Pump_AL_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Pump_AL_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump_AL)
        #define CyBtldrCommStart    Pump_AL_CyBtldrCommStart
        #define CyBtldrCommStop     Pump_AL_CyBtldrCommStop
        #define CyBtldrCommReset    Pump_AL_CyBtldrCommReset
        #define CyBtldrCommWrite    Pump_AL_CyBtldrCommWrite
        #define CyBtldrCommRead     Pump_AL_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump_AL) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Pump_AL_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Pump_AL_SET_SPACE                              (0x00u)
#define Pump_AL_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Pump_AL_TX_ENABLED) || (Pump_AL_HD_ENABLED) )
    #if(Pump_AL_TX_INTERRUPT_ENABLED)
        #define Pump_AL_TX_VECT_NUM            (uint8)Pump_AL_TXInternalInterrupt__INTC_NUMBER
        #define Pump_AL_TX_PRIOR_NUM           (uint8)Pump_AL_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pump_AL_TX_INTERRUPT_ENABLED */
    #if(Pump_AL_TX_ENABLED)
        #define Pump_AL_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pump_AL_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pump_AL_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Pump_AL_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pump_AL_TX_ENABLED */
    #if(Pump_AL_HD_ENABLED)
        #define Pump_AL_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pump_AL_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pump_AL_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Pump_AL_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pump_AL_HD_ENABLED */
    #define Pump_AL_TX_STS_COMPLETE            (uint8)(0x01u << Pump_AL_TX_STS_COMPLETE_SHIFT)
    #define Pump_AL_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Pump_AL_TX_STS_FIFO_EMPTY_SHIFT)
    #define Pump_AL_TX_STS_FIFO_FULL           (uint8)(0x01u << Pump_AL_TX_STS_FIFO_FULL_SHIFT)
    #define Pump_AL_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Pump_AL_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Pump_AL_TX_ENABLED) || (Pump_AL_HD_ENABLED)*/

#if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
    #if(Pump_AL_RX_INTERRUPT_ENABLED)
        #define Pump_AL_RX_VECT_NUM            (uint8)Pump_AL_RXInternalInterrupt__INTC_NUMBER
        #define Pump_AL_RX_PRIOR_NUM           (uint8)Pump_AL_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pump_AL_RX_INTERRUPT_ENABLED */
    #define Pump_AL_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Pump_AL_RX_STS_BREAK_SHIFT             (0x01u)
    #define Pump_AL_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Pump_AL_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Pump_AL_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Pump_AL_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Pump_AL_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Pump_AL_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Pump_AL_RX_STS_MRKSPC           (uint8)(0x01u << Pump_AL_RX_STS_MRKSPC_SHIFT)
    #define Pump_AL_RX_STS_BREAK            (uint8)(0x01u << Pump_AL_RX_STS_BREAK_SHIFT)
    #define Pump_AL_RX_STS_PAR_ERROR        (uint8)(0x01u << Pump_AL_RX_STS_PAR_ERROR_SHIFT)
    #define Pump_AL_RX_STS_STOP_ERROR       (uint8)(0x01u << Pump_AL_RX_STS_STOP_ERROR_SHIFT)
    #define Pump_AL_RX_STS_OVERRUN          (uint8)(0x01u << Pump_AL_RX_STS_OVERRUN_SHIFT)
    #define Pump_AL_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Pump_AL_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Pump_AL_RX_STS_ADDR_MATCH       (uint8)(0x01u << Pump_AL_RX_STS_ADDR_MATCH_SHIFT)
    #define Pump_AL_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Pump_AL_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Pump_AL_RX_HW_MASK                     (0x7Fu)
#endif /* End (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */

/* Control Register definitions */
#define Pump_AL_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Pump_AL_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Pump_AL_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Pump_AL_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Pump_AL_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Pump_AL_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Pump_AL_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Pump_AL_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Pump_AL_CTRL_HD_SEND               (uint8)(0x01u << Pump_AL_CTRL_HD_SEND_SHIFT)
#define Pump_AL_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Pump_AL_CTRL_HD_SEND_BREAK_SHIFT)
#define Pump_AL_CTRL_MARK                  (uint8)(0x01u << Pump_AL_CTRL_MARK_SHIFT)
#define Pump_AL_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Pump_AL_CTRL_PARITY_TYPE0_SHIFT)
#define Pump_AL_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Pump_AL_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Pump_AL_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Pump_AL_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Pump_AL_SEND_BREAK                         (0x00u)
#define Pump_AL_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Pump_AL_REINIT                             (0x02u)
#define Pump_AL_SEND_WAIT_REINIT                   (0x03u)

#define Pump_AL_OVER_SAMPLE_8                      (8u)
#define Pump_AL_OVER_SAMPLE_16                     (16u)

#define Pump_AL_BIT_CENTER                         (Pump_AL_OVER_SAMPLE_COUNT - 1u)

#define Pump_AL_FIFO_LENGTH                        (4u)
#define Pump_AL_NUMBER_OF_START_BIT                (1u)
#define Pump_AL_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Pump_AL_TXBITCTR_BREAKBITS8X   ((Pump_AL_BREAK_BITS_TX * Pump_AL_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Pump_AL_TXBITCTR_BREAKBITS ((Pump_AL_BREAK_BITS_TX * Pump_AL_OVER_SAMPLE_COUNT) - 1u)

#define Pump_AL_HALF_BIT_COUNT   \
                            (((Pump_AL_OVER_SAMPLE_COUNT / 2u) + (Pump_AL_USE23POLLING * 1u)) - 2u)
#if (Pump_AL_OVER_SAMPLE_COUNT == Pump_AL_OVER_SAMPLE_8)
    #define Pump_AL_HD_TXBITCTR_INIT   (((Pump_AL_BREAK_BITS_TX + \
                            Pump_AL_NUMBER_OF_START_BIT) * Pump_AL_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Pump_AL_RXBITCTR_INIT  ((((Pump_AL_BREAK_BITS_RX + Pump_AL_NUMBER_OF_START_BIT) \
                            * Pump_AL_OVER_SAMPLE_COUNT) + Pump_AL_HALF_BIT_COUNT) - 1u)


#else /* Pump_AL_OVER_SAMPLE_COUNT == Pump_AL_OVER_SAMPLE_16 */
    #define Pump_AL_HD_TXBITCTR_INIT   ((8u * Pump_AL_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Pump_AL_RXBITCTR_INIT      (((7u * Pump_AL_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Pump_AL_HALF_BIT_COUNT)
#endif /* End Pump_AL_OVER_SAMPLE_COUNT */
#define Pump_AL_HD_RXBITCTR_INIT                   Pump_AL_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Pump_AL_initVar;
#if( Pump_AL_TX_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
    extern volatile uint8 Pump_AL_txBuffer[Pump_AL_TXBUFFERSIZE];
    extern volatile uint8 Pump_AL_txBufferRead;
    extern uint8 Pump_AL_txBufferWrite;
#endif /* End Pump_AL_TX_ENABLED */
#if( ( Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED ) && \
     (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH) )
    extern volatile uint8 Pump_AL_rxBuffer[Pump_AL_RXBUFFERSIZE];
    extern volatile uint16 Pump_AL_rxBufferRead;
    extern volatile uint16 Pump_AL_rxBufferWrite;
    extern volatile uint8 Pump_AL_rxBufferLoopDetect;
    extern volatile uint8 Pump_AL_rxBufferOverflow;
    #if (Pump_AL_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Pump_AL_rxAddressMode;
        extern volatile uint8 Pump_AL_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Pump_AL_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Pump_AL__B_UART__AM_SW_BYTE_BYTE 1
#define Pump_AL__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Pump_AL__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Pump_AL__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Pump_AL__B_UART__AM_NONE 0

#define Pump_AL__B_UART__NONE_REVB 0
#define Pump_AL__B_UART__EVEN_REVB 1
#define Pump_AL__B_UART__ODD_REVB 2
#define Pump_AL__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Pump_AL_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Pump_AL_NUMBER_OF_STOP_BITS    (1u)

#if (Pump_AL_RXHW_ADDRESS_ENABLED)
    #define Pump_AL_RXADDRESSMODE      (0u)
    #define Pump_AL_RXHWADDRESS1       (0u)
    #define Pump_AL_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Pump_AL_RXAddressMode      Pump_AL_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Pump_AL_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Pump_AL_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Pump_AL_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Pump_AL_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Pump_AL_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Pump_AL_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Pump_AL_RX_STS_BREAK_SHIFT) \
                                        | (0 << Pump_AL_RX_STS_OVERRUN_SHIFT))

#define Pump_AL_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Pump_AL_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Pump_AL_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Pump_AL_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Pump_AL_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_AL_CONTROL_REG \
                            (* (reg8 *) Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Pump_AL_CONTROL_PTR \
                            (  (reg8 *) Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pump_AL_TX_ENABLED)
    #define Pump_AL_TXDATA_REG          (* (reg8 *) Pump_AL_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pump_AL_TXDATA_PTR          (  (reg8 *) Pump_AL_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pump_AL_TXDATA_AUX_CTL_REG  (* (reg8 *) Pump_AL_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_AL_TXDATA_AUX_CTL_PTR  (  (reg8 *) Pump_AL_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_AL_TXSTATUS_REG        (* (reg8 *) Pump_AL_BUART_sTX_TxSts__STATUS_REG)
    #define Pump_AL_TXSTATUS_PTR        (  (reg8 *) Pump_AL_BUART_sTX_TxSts__STATUS_REG)
    #define Pump_AL_TXSTATUS_MASK_REG   (* (reg8 *) Pump_AL_BUART_sTX_TxSts__MASK_REG)
    #define Pump_AL_TXSTATUS_MASK_PTR   (  (reg8 *) Pump_AL_BUART_sTX_TxSts__MASK_REG)
    #define Pump_AL_TXSTATUS_ACTL_REG   (* (reg8 *) Pump_AL_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Pump_AL_TXSTATUS_ACTL_PTR   (  (reg8 *) Pump_AL_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Pump_AL_TXCLKGEN_DP)
        #define Pump_AL_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pump_AL_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pump_AL_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Pump_AL_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Pump_AL_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pump_AL_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pump_AL_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pump_AL_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pump_AL_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Pump_AL_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Pump_AL_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Pump_AL_TXCLKGEN_DP */

#endif /* End Pump_AL_TX_ENABLED */

#if(Pump_AL_HD_ENABLED)

    #define Pump_AL_TXDATA_REG             (* (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pump_AL_TXDATA_PTR             (  (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pump_AL_TXDATA_AUX_CTL_REG     (* (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_AL_TXDATA_AUX_CTL_PTR     (  (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pump_AL_TXSTATUS_REG           (* (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_AL_TXSTATUS_PTR           (  (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_AL_TXSTATUS_MASK_REG      (* (reg8 *) Pump_AL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_AL_TXSTATUS_MASK_PTR      (  (reg8 *) Pump_AL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_AL_TXSTATUS_ACTL_REG      (* (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pump_AL_TXSTATUS_ACTL_PTR      (  (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Pump_AL_HD_ENABLED */

#if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
    #define Pump_AL_RXDATA_REG             (* (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pump_AL_RXDATA_PTR             (  (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pump_AL_RXADDRESS1_REG         (* (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pump_AL_RXADDRESS1_PTR         (  (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pump_AL_RXADDRESS2_REG         (* (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pump_AL_RXADDRESS2_PTR         (  (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pump_AL_RXDATA_AUX_CTL_REG     (* (reg8 *) Pump_AL_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pump_AL_RXBITCTR_PERIOD_REG    (* (reg8 *) Pump_AL_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pump_AL_RXBITCTR_PERIOD_PTR    (  (reg8 *) Pump_AL_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pump_AL_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pump_AL_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pump_AL_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pump_AL_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pump_AL_RXBITCTR_COUNTER_REG   (* (reg8 *) Pump_AL_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Pump_AL_RXBITCTR_COUNTER_PTR   (  (reg8 *) Pump_AL_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Pump_AL_RXSTATUS_REG           (* (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_AL_RXSTATUS_PTR           (  (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_AL_RXSTATUS_MASK_REG      (* (reg8 *) Pump_AL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_AL_RXSTATUS_MASK_PTR      (  (reg8 *) Pump_AL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_AL_RXSTATUS_ACTL_REG      (* (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pump_AL_RXSTATUS_ACTL_PTR      (  (reg8 *) Pump_AL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */

#if(Pump_AL_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Pump_AL_INTCLOCK_CLKEN_REG     (* (reg8 *) Pump_AL_IntClock__PM_ACT_CFG)
    #define Pump_AL_INTCLOCK_CLKEN_PTR     (  (reg8 *) Pump_AL_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Pump_AL_INTCLOCK_CLKEN_MASK    Pump_AL_IntClock__PM_ACT_MSK
#endif /* End Pump_AL_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Pump_AL_TX_ENABLED)
    #define Pump_AL_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Pump_AL_TX_ENABLED */

#if(Pump_AL_HD_ENABLED)
    #define Pump_AL_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Pump_AL_HD_ENABLED */

#if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
    #define Pump_AL_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Pump_AL_initvar                    Pump_AL_initVar

#define Pump_AL_RX_Enabled                 Pump_AL_RX_ENABLED
#define Pump_AL_TX_Enabled                 Pump_AL_TX_ENABLED
#define Pump_AL_HD_Enabled                 Pump_AL_HD_ENABLED
#define Pump_AL_RX_IntInterruptEnabled     Pump_AL_RX_INTERRUPT_ENABLED
#define Pump_AL_TX_IntInterruptEnabled     Pump_AL_TX_INTERRUPT_ENABLED
#define Pump_AL_InternalClockUsed          Pump_AL_INTERNAL_CLOCK_USED
#define Pump_AL_RXHW_Address_Enabled       Pump_AL_RXHW_ADDRESS_ENABLED
#define Pump_AL_OverSampleCount            Pump_AL_OVER_SAMPLE_COUNT
#define Pump_AL_ParityType                 Pump_AL_PARITY_TYPE

#if( Pump_AL_TX_ENABLED && (Pump_AL_TXBUFFERSIZE > Pump_AL_FIFO_LENGTH))
    #define Pump_AL_TXBUFFER               Pump_AL_txBuffer
    #define Pump_AL_TXBUFFERREAD           Pump_AL_txBufferRead
    #define Pump_AL_TXBUFFERWRITE          Pump_AL_txBufferWrite
#endif /* End Pump_AL_TX_ENABLED */
#if( ( Pump_AL_RX_ENABLED || Pump_AL_HD_ENABLED ) && \
     (Pump_AL_RXBUFFERSIZE > Pump_AL_FIFO_LENGTH) )
    #define Pump_AL_RXBUFFER               Pump_AL_rxBuffer
    #define Pump_AL_RXBUFFERREAD           Pump_AL_rxBufferRead
    #define Pump_AL_RXBUFFERWRITE          Pump_AL_rxBufferWrite
    #define Pump_AL_RXBUFFERLOOPDETECT     Pump_AL_rxBufferLoopDetect
    #define Pump_AL_RXBUFFER_OVERFLOW      Pump_AL_rxBufferOverflow
#endif /* End Pump_AL_RX_ENABLED */

#ifdef Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_AL_CONTROL                Pump_AL_CONTROL_REG
#endif /* End Pump_AL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pump_AL_TX_ENABLED)
    #define Pump_AL_TXDATA                 Pump_AL_TXDATA_REG
    #define Pump_AL_TXSTATUS               Pump_AL_TXSTATUS_REG
    #define Pump_AL_TXSTATUS_MASK          Pump_AL_TXSTATUS_MASK_REG
    #define Pump_AL_TXSTATUS_ACTL          Pump_AL_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Pump_AL_TXCLKGEN_DP)
        #define Pump_AL_TXBITCLKGEN_CTR        Pump_AL_TXBITCLKGEN_CTR_REG
        #define Pump_AL_TXBITCLKTX_COMPLETE    Pump_AL_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Pump_AL_TXBITCTR_PERIOD        Pump_AL_TXBITCTR_PERIOD_REG
        #define Pump_AL_TXBITCTR_CONTROL       Pump_AL_TXBITCTR_CONTROL_REG
        #define Pump_AL_TXBITCTR_COUNTER       Pump_AL_TXBITCTR_COUNTER_REG
    #endif /* Pump_AL_TXCLKGEN_DP */
#endif /* End Pump_AL_TX_ENABLED */

#if(Pump_AL_HD_ENABLED)
    #define Pump_AL_TXDATA                 Pump_AL_TXDATA_REG
    #define Pump_AL_TXSTATUS               Pump_AL_TXSTATUS_REG
    #define Pump_AL_TXSTATUS_MASK          Pump_AL_TXSTATUS_MASK_REG
    #define Pump_AL_TXSTATUS_ACTL          Pump_AL_TXSTATUS_ACTL_REG
#endif /* End Pump_AL_HD_ENABLED */

#if( (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) )
    #define Pump_AL_RXDATA                 Pump_AL_RXDATA_REG
    #define Pump_AL_RXADDRESS1             Pump_AL_RXADDRESS1_REG
    #define Pump_AL_RXADDRESS2             Pump_AL_RXADDRESS2_REG
    #define Pump_AL_RXBITCTR_PERIOD        Pump_AL_RXBITCTR_PERIOD_REG
    #define Pump_AL_RXBITCTR_CONTROL       Pump_AL_RXBITCTR_CONTROL_REG
    #define Pump_AL_RXBITCTR_COUNTER       Pump_AL_RXBITCTR_COUNTER_REG
    #define Pump_AL_RXSTATUS               Pump_AL_RXSTATUS_REG
    #define Pump_AL_RXSTATUS_MASK          Pump_AL_RXSTATUS_MASK_REG
    #define Pump_AL_RXSTATUS_ACTL          Pump_AL_RXSTATUS_ACTL_REG
#endif /* End  (Pump_AL_RX_ENABLED) || (Pump_AL_HD_ENABLED) */

#if(Pump_AL_INTERNAL_CLOCK_USED)
    #define Pump_AL_INTCLOCK_CLKEN         Pump_AL_INTCLOCK_CLKEN_REG
#endif /* End Pump_AL_INTERNAL_CLOCK_USED */

#define Pump_AL_WAIT_FOR_COMLETE_REINIT    Pump_AL_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Pump_AL_H */


/* [] END OF FILE */
