/*******************************************************************************
* File Name: Pump_PL.h
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


#if !defined(CY_UART_Pump_PL_H)
#define CY_UART_Pump_PL_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define Pump_PL_RX_ENABLED                     (1u)
#define Pump_PL_TX_ENABLED                     (1u)
#define Pump_PL_HD_ENABLED                     (0u)
#define Pump_PL_RX_INTERRUPT_ENABLED           (1u)
#define Pump_PL_TX_INTERRUPT_ENABLED           (0u)
#define Pump_PL_INTERNAL_CLOCK_USED            (0u)
#define Pump_PL_RXHW_ADDRESS_ENABLED           (0u)
#define Pump_PL_OVER_SAMPLE_COUNT              (8u)
#define Pump_PL_PARITY_TYPE                    (0u)
#define Pump_PL_PARITY_TYPE_SW                 (0u)
#define Pump_PL_BREAK_DETECT                   (0u)
#define Pump_PL_BREAK_BITS_TX                  (13u)
#define Pump_PL_BREAK_BITS_RX                  (13u)
#define Pump_PL_TXCLKGEN_DP                    (1u)
#define Pump_PL_USE23POLLING                   (1u)
#define Pump_PL_FLOW_CONTROL                   (0u)
#define Pump_PL_CLK_FREQ                       (0u)
#define Pump_PL_TXBUFFERSIZE                   (4u)
#define Pump_PL_RXBUFFERSIZE                   (256u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_PL_CONTROL_REG_REMOVED            (0u)
#else
    #define Pump_PL_CONTROL_REG_REMOVED            (1u)
#endif /* End Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct Pump_PL_backupStruct_
{
    uint8 enableState;

    #if(Pump_PL_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End Pump_PL_CONTROL_REG_REMOVED */
    #if( (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (Pump_PL_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End Pump_PL_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED)*/

    #if(Pump_PL_TX_ENABLED)
        #if(Pump_PL_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End Pump_PL_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End Pump_PL_TX_ENABLED */
} Pump_PL_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void Pump_PL_Start(void) ;
void Pump_PL_Stop(void) ;
uint8 Pump_PL_ReadControlRegister(void) ;
void Pump_PL_WriteControlRegister(uint8 control) ;

void Pump_PL_Init(void) ;
void Pump_PL_Enable(void) ;
void Pump_PL_SaveConfig(void) ;
void Pump_PL_RestoreConfig(void) ;
void Pump_PL_Sleep(void) ;
void Pump_PL_Wakeup(void) ;

/* Only if RX is enabled */
#if( (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) )

    #if(Pump_PL_RX_INTERRUPT_ENABLED)
        void  Pump_PL_EnableRxInt(void) ;
        void  Pump_PL_DisableRxInt(void) ;
        CY_ISR_PROTO(Pump_PL_RXISR);
    #endif /* Pump_PL_RX_INTERRUPT_ENABLED */

    void Pump_PL_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void Pump_PL_SetRxAddress1(uint8 address) ;
    void Pump_PL_SetRxAddress2(uint8 address) ;

    void  Pump_PL_SetRxInterruptMode(uint8 intSrc) ;
    uint8 Pump_PL_ReadRxData(void) ;
    uint8 Pump_PL_ReadRxStatus(void) ;
    uint8 Pump_PL_GetChar(void) ;
    uint16 Pump_PL_GetByte(void) ;
    uint16 Pump_PL_GetRxBufferSize(void)
                                                            ;
    void Pump_PL_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define Pump_PL_GetRxInterruptSource   Pump_PL_ReadRxStatus

#endif /* End (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) */

/* Only if TX is enabled */
#if(Pump_PL_TX_ENABLED || Pump_PL_HD_ENABLED)

    #if(Pump_PL_TX_INTERRUPT_ENABLED)
        void Pump_PL_EnableTxInt(void) ;
        void Pump_PL_DisableTxInt(void) ;
        CY_ISR_PROTO(Pump_PL_TXISR);
    #endif /* Pump_PL_TX_INTERRUPT_ENABLED */

    void Pump_PL_SetTxInterruptMode(uint8 intSrc) ;
    void Pump_PL_WriteTxData(uint8 txDataByte) ;
    uint8 Pump_PL_ReadTxStatus(void) ;
    void Pump_PL_PutChar(uint8 txDataByte) ;
    void Pump_PL_PutString(const char8 string[]) ;
    void Pump_PL_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void Pump_PL_PutCRLF(uint8 txDataByte) ;
    void Pump_PL_ClearTxBuffer(void) ;
    void Pump_PL_SetTxAddressMode(uint8 addressMode) ;
    void Pump_PL_SendBreak(uint8 retMode) ;
    uint8 Pump_PL_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define Pump_PL_PutStringConst         Pump_PL_PutString
    #define Pump_PL_PutArrayConst          Pump_PL_PutArray
    #define Pump_PL_GetTxInterruptSource   Pump_PL_ReadTxStatus

#endif /* End Pump_PL_TX_ENABLED || Pump_PL_HD_ENABLED */

#if(Pump_PL_HD_ENABLED)
    void Pump_PL_LoadRxConfig(void) ;
    void Pump_PL_LoadTxConfig(void) ;
#endif /* End Pump_PL_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump_PL) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    Pump_PL_CyBtldrCommStart(void) CYSMALL ;
    void    Pump_PL_CyBtldrCommStop(void) CYSMALL ;
    void    Pump_PL_CyBtldrCommReset(void) CYSMALL ;
    cystatus Pump_PL_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus Pump_PL_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump_PL)
        #define CyBtldrCommStart    Pump_PL_CyBtldrCommStart
        #define CyBtldrCommStop     Pump_PL_CyBtldrCommStop
        #define CyBtldrCommReset    Pump_PL_CyBtldrCommReset
        #define CyBtldrCommWrite    Pump_PL_CyBtldrCommWrite
        #define CyBtldrCommRead     Pump_PL_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Pump_PL) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define Pump_PL_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define Pump_PL_SET_SPACE                              (0x00u)
#define Pump_PL_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (Pump_PL_TX_ENABLED) || (Pump_PL_HD_ENABLED) )
    #if(Pump_PL_TX_INTERRUPT_ENABLED)
        #define Pump_PL_TX_VECT_NUM            (uint8)Pump_PL_TXInternalInterrupt__INTC_NUMBER
        #define Pump_PL_TX_PRIOR_NUM           (uint8)Pump_PL_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pump_PL_TX_INTERRUPT_ENABLED */
    #if(Pump_PL_TX_ENABLED)
        #define Pump_PL_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pump_PL_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pump_PL_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define Pump_PL_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pump_PL_TX_ENABLED */
    #if(Pump_PL_HD_ENABLED)
        #define Pump_PL_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define Pump_PL_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define Pump_PL_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define Pump_PL_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* Pump_PL_HD_ENABLED */
    #define Pump_PL_TX_STS_COMPLETE            (uint8)(0x01u << Pump_PL_TX_STS_COMPLETE_SHIFT)
    #define Pump_PL_TX_STS_FIFO_EMPTY          (uint8)(0x01u << Pump_PL_TX_STS_FIFO_EMPTY_SHIFT)
    #define Pump_PL_TX_STS_FIFO_FULL           (uint8)(0x01u << Pump_PL_TX_STS_FIFO_FULL_SHIFT)
    #define Pump_PL_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << Pump_PL_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (Pump_PL_TX_ENABLED) || (Pump_PL_HD_ENABLED)*/

#if( (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) )
    #if(Pump_PL_RX_INTERRUPT_ENABLED)
        #define Pump_PL_RX_VECT_NUM            (uint8)Pump_PL_RXInternalInterrupt__INTC_NUMBER
        #define Pump_PL_RX_PRIOR_NUM           (uint8)Pump_PL_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* Pump_PL_RX_INTERRUPT_ENABLED */
    #define Pump_PL_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define Pump_PL_RX_STS_BREAK_SHIFT             (0x01u)
    #define Pump_PL_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define Pump_PL_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define Pump_PL_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define Pump_PL_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define Pump_PL_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define Pump_PL_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define Pump_PL_RX_STS_MRKSPC           (uint8)(0x01u << Pump_PL_RX_STS_MRKSPC_SHIFT)
    #define Pump_PL_RX_STS_BREAK            (uint8)(0x01u << Pump_PL_RX_STS_BREAK_SHIFT)
    #define Pump_PL_RX_STS_PAR_ERROR        (uint8)(0x01u << Pump_PL_RX_STS_PAR_ERROR_SHIFT)
    #define Pump_PL_RX_STS_STOP_ERROR       (uint8)(0x01u << Pump_PL_RX_STS_STOP_ERROR_SHIFT)
    #define Pump_PL_RX_STS_OVERRUN          (uint8)(0x01u << Pump_PL_RX_STS_OVERRUN_SHIFT)
    #define Pump_PL_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << Pump_PL_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define Pump_PL_RX_STS_ADDR_MATCH       (uint8)(0x01u << Pump_PL_RX_STS_ADDR_MATCH_SHIFT)
    #define Pump_PL_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << Pump_PL_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define Pump_PL_RX_HW_MASK                     (0x7Fu)
#endif /* End (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) */

/* Control Register definitions */
#define Pump_PL_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define Pump_PL_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define Pump_PL_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define Pump_PL_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define Pump_PL_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define Pump_PL_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define Pump_PL_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define Pump_PL_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define Pump_PL_CTRL_HD_SEND               (uint8)(0x01u << Pump_PL_CTRL_HD_SEND_SHIFT)
#define Pump_PL_CTRL_HD_SEND_BREAK         (uint8)(0x01u << Pump_PL_CTRL_HD_SEND_BREAK_SHIFT)
#define Pump_PL_CTRL_MARK                  (uint8)(0x01u << Pump_PL_CTRL_MARK_SHIFT)
#define Pump_PL_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << Pump_PL_CTRL_PARITY_TYPE0_SHIFT)
#define Pump_PL_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << Pump_PL_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define Pump_PL_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define Pump_PL_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define Pump_PL_SEND_BREAK                         (0x00u)
#define Pump_PL_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define Pump_PL_REINIT                             (0x02u)
#define Pump_PL_SEND_WAIT_REINIT                   (0x03u)

#define Pump_PL_OVER_SAMPLE_8                      (8u)
#define Pump_PL_OVER_SAMPLE_16                     (16u)

#define Pump_PL_BIT_CENTER                         (Pump_PL_OVER_SAMPLE_COUNT - 1u)

#define Pump_PL_FIFO_LENGTH                        (4u)
#define Pump_PL_NUMBER_OF_START_BIT                (1u)
#define Pump_PL_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define Pump_PL_TXBITCTR_BREAKBITS8X   ((Pump_PL_BREAK_BITS_TX * Pump_PL_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define Pump_PL_TXBITCTR_BREAKBITS ((Pump_PL_BREAK_BITS_TX * Pump_PL_OVER_SAMPLE_COUNT) - 1u)

#define Pump_PL_HALF_BIT_COUNT   \
                            (((Pump_PL_OVER_SAMPLE_COUNT / 2u) + (Pump_PL_USE23POLLING * 1u)) - 2u)
#if (Pump_PL_OVER_SAMPLE_COUNT == Pump_PL_OVER_SAMPLE_8)
    #define Pump_PL_HD_TXBITCTR_INIT   (((Pump_PL_BREAK_BITS_TX + \
                            Pump_PL_NUMBER_OF_START_BIT) * Pump_PL_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define Pump_PL_RXBITCTR_INIT  ((((Pump_PL_BREAK_BITS_RX + Pump_PL_NUMBER_OF_START_BIT) \
                            * Pump_PL_OVER_SAMPLE_COUNT) + Pump_PL_HALF_BIT_COUNT) - 1u)


#else /* Pump_PL_OVER_SAMPLE_COUNT == Pump_PL_OVER_SAMPLE_16 */
    #define Pump_PL_HD_TXBITCTR_INIT   ((8u * Pump_PL_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define Pump_PL_RXBITCTR_INIT      (((7u * Pump_PL_OVER_SAMPLE_COUNT) - 1u) + \
                                                      Pump_PL_HALF_BIT_COUNT)
#endif /* End Pump_PL_OVER_SAMPLE_COUNT */
#define Pump_PL_HD_RXBITCTR_INIT                   Pump_PL_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 Pump_PL_initVar;
#if( Pump_PL_TX_ENABLED && (Pump_PL_TXBUFFERSIZE > Pump_PL_FIFO_LENGTH))
    extern volatile uint8 Pump_PL_txBuffer[Pump_PL_TXBUFFERSIZE];
    extern volatile uint8 Pump_PL_txBufferRead;
    extern uint8 Pump_PL_txBufferWrite;
#endif /* End Pump_PL_TX_ENABLED */
#if( ( Pump_PL_RX_ENABLED || Pump_PL_HD_ENABLED ) && \
     (Pump_PL_RXBUFFERSIZE > Pump_PL_FIFO_LENGTH) )
    extern volatile uint8 Pump_PL_rxBuffer[Pump_PL_RXBUFFERSIZE];
    extern volatile uint16 Pump_PL_rxBufferRead;
    extern volatile uint16 Pump_PL_rxBufferWrite;
    extern volatile uint8 Pump_PL_rxBufferLoopDetect;
    extern volatile uint8 Pump_PL_rxBufferOverflow;
    #if (Pump_PL_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 Pump_PL_rxAddressMode;
        extern volatile uint8 Pump_PL_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End Pump_PL_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define Pump_PL__B_UART__AM_SW_BYTE_BYTE 1
#define Pump_PL__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define Pump_PL__B_UART__AM_HW_BYTE_BY_BYTE 3
#define Pump_PL__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define Pump_PL__B_UART__AM_NONE 0

#define Pump_PL__B_UART__NONE_REVB 0
#define Pump_PL__B_UART__EVEN_REVB 1
#define Pump_PL__B_UART__ODD_REVB 2
#define Pump_PL__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define Pump_PL_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define Pump_PL_NUMBER_OF_STOP_BITS    (1u)

#if (Pump_PL_RXHW_ADDRESS_ENABLED)
    #define Pump_PL_RXADDRESSMODE      (0u)
    #define Pump_PL_RXHWADDRESS1       (0u)
    #define Pump_PL_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define Pump_PL_RXAddressMode      Pump_PL_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define Pump_PL_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << Pump_PL_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << Pump_PL_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << Pump_PL_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << Pump_PL_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << Pump_PL_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << Pump_PL_RX_STS_BREAK_SHIFT) \
                                        | (0 << Pump_PL_RX_STS_OVERRUN_SHIFT))

#define Pump_PL_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << Pump_PL_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << Pump_PL_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << Pump_PL_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << Pump_PL_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_PL_CONTROL_REG \
                            (* (reg8 *) Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define Pump_PL_CONTROL_PTR \
                            (  (reg8 *) Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pump_PL_TX_ENABLED)
    #define Pump_PL_TXDATA_REG          (* (reg8 *) Pump_PL_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pump_PL_TXDATA_PTR          (  (reg8 *) Pump_PL_BUART_sTX_TxShifter_u0__F0_REG)
    #define Pump_PL_TXDATA_AUX_CTL_REG  (* (reg8 *) Pump_PL_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_PL_TXDATA_AUX_CTL_PTR  (  (reg8 *) Pump_PL_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_PL_TXSTATUS_REG        (* (reg8 *) Pump_PL_BUART_sTX_TxSts__STATUS_REG)
    #define Pump_PL_TXSTATUS_PTR        (  (reg8 *) Pump_PL_BUART_sTX_TxSts__STATUS_REG)
    #define Pump_PL_TXSTATUS_MASK_REG   (* (reg8 *) Pump_PL_BUART_sTX_TxSts__MASK_REG)
    #define Pump_PL_TXSTATUS_MASK_PTR   (  (reg8 *) Pump_PL_BUART_sTX_TxSts__MASK_REG)
    #define Pump_PL_TXSTATUS_ACTL_REG   (* (reg8 *) Pump_PL_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define Pump_PL_TXSTATUS_ACTL_PTR   (  (reg8 *) Pump_PL_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(Pump_PL_TXCLKGEN_DP)
        #define Pump_PL_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pump_PL_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define Pump_PL_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define Pump_PL_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define Pump_PL_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pump_PL_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define Pump_PL_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pump_PL_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define Pump_PL_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define Pump_PL_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) Pump_PL_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* Pump_PL_TXCLKGEN_DP */

#endif /* End Pump_PL_TX_ENABLED */

#if(Pump_PL_HD_ENABLED)

    #define Pump_PL_TXDATA_REG             (* (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pump_PL_TXDATA_PTR             (  (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__F1_REG )
    #define Pump_PL_TXDATA_AUX_CTL_REG     (* (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define Pump_PL_TXDATA_AUX_CTL_PTR     (  (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pump_PL_TXSTATUS_REG           (* (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_PL_TXSTATUS_PTR           (  (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_PL_TXSTATUS_MASK_REG      (* (reg8 *) Pump_PL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_PL_TXSTATUS_MASK_PTR      (  (reg8 *) Pump_PL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_PL_TXSTATUS_ACTL_REG      (* (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pump_PL_TXSTATUS_ACTL_PTR      (  (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End Pump_PL_HD_ENABLED */

#if( (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) )
    #define Pump_PL_RXDATA_REG             (* (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pump_PL_RXDATA_PTR             (  (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__F0_REG )
    #define Pump_PL_RXADDRESS1_REG         (* (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pump_PL_RXADDRESS1_PTR         (  (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__D0_REG )
    #define Pump_PL_RXADDRESS2_REG         (* (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pump_PL_RXADDRESS2_PTR         (  (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__D1_REG )
    #define Pump_PL_RXDATA_AUX_CTL_REG     (* (reg8 *) Pump_PL_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define Pump_PL_RXBITCTR_PERIOD_REG    (* (reg8 *) Pump_PL_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pump_PL_RXBITCTR_PERIOD_PTR    (  (reg8 *) Pump_PL_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define Pump_PL_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) Pump_PL_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pump_PL_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) Pump_PL_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define Pump_PL_RXBITCTR_COUNTER_REG   (* (reg8 *) Pump_PL_BUART_sRX_RxBitCounter__COUNT_REG )
    #define Pump_PL_RXBITCTR_COUNTER_PTR   (  (reg8 *) Pump_PL_BUART_sRX_RxBitCounter__COUNT_REG )

    #define Pump_PL_RXSTATUS_REG           (* (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_PL_RXSTATUS_PTR           (  (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_REG )
    #define Pump_PL_RXSTATUS_MASK_REG      (* (reg8 *) Pump_PL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_PL_RXSTATUS_MASK_PTR      (  (reg8 *) Pump_PL_BUART_sRX_RxSts__MASK_REG )
    #define Pump_PL_RXSTATUS_ACTL_REG      (* (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define Pump_PL_RXSTATUS_ACTL_PTR      (  (reg8 *) Pump_PL_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) */

#if(Pump_PL_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define Pump_PL_INTCLOCK_CLKEN_REG     (* (reg8 *) Pump_PL_IntClock__PM_ACT_CFG)
    #define Pump_PL_INTCLOCK_CLKEN_PTR     (  (reg8 *) Pump_PL_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define Pump_PL_INTCLOCK_CLKEN_MASK    Pump_PL_IntClock__PM_ACT_MSK
#endif /* End Pump_PL_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(Pump_PL_TX_ENABLED)
    #define Pump_PL_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End Pump_PL_TX_ENABLED */

#if(Pump_PL_HD_ENABLED)
    #define Pump_PL_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End Pump_PL_HD_ENABLED */

#if( (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) )
    #define Pump_PL_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define Pump_PL_initvar                    Pump_PL_initVar

#define Pump_PL_RX_Enabled                 Pump_PL_RX_ENABLED
#define Pump_PL_TX_Enabled                 Pump_PL_TX_ENABLED
#define Pump_PL_HD_Enabled                 Pump_PL_HD_ENABLED
#define Pump_PL_RX_IntInterruptEnabled     Pump_PL_RX_INTERRUPT_ENABLED
#define Pump_PL_TX_IntInterruptEnabled     Pump_PL_TX_INTERRUPT_ENABLED
#define Pump_PL_InternalClockUsed          Pump_PL_INTERNAL_CLOCK_USED
#define Pump_PL_RXHW_Address_Enabled       Pump_PL_RXHW_ADDRESS_ENABLED
#define Pump_PL_OverSampleCount            Pump_PL_OVER_SAMPLE_COUNT
#define Pump_PL_ParityType                 Pump_PL_PARITY_TYPE

#if( Pump_PL_TX_ENABLED && (Pump_PL_TXBUFFERSIZE > Pump_PL_FIFO_LENGTH))
    #define Pump_PL_TXBUFFER               Pump_PL_txBuffer
    #define Pump_PL_TXBUFFERREAD           Pump_PL_txBufferRead
    #define Pump_PL_TXBUFFERWRITE          Pump_PL_txBufferWrite
#endif /* End Pump_PL_TX_ENABLED */
#if( ( Pump_PL_RX_ENABLED || Pump_PL_HD_ENABLED ) && \
     (Pump_PL_RXBUFFERSIZE > Pump_PL_FIFO_LENGTH) )
    #define Pump_PL_RXBUFFER               Pump_PL_rxBuffer
    #define Pump_PL_RXBUFFERREAD           Pump_PL_rxBufferRead
    #define Pump_PL_RXBUFFERWRITE          Pump_PL_rxBufferWrite
    #define Pump_PL_RXBUFFERLOOPDETECT     Pump_PL_rxBufferLoopDetect
    #define Pump_PL_RXBUFFER_OVERFLOW      Pump_PL_rxBufferOverflow
#endif /* End Pump_PL_RX_ENABLED */

#ifdef Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define Pump_PL_CONTROL                Pump_PL_CONTROL_REG
#endif /* End Pump_PL_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(Pump_PL_TX_ENABLED)
    #define Pump_PL_TXDATA                 Pump_PL_TXDATA_REG
    #define Pump_PL_TXSTATUS               Pump_PL_TXSTATUS_REG
    #define Pump_PL_TXSTATUS_MASK          Pump_PL_TXSTATUS_MASK_REG
    #define Pump_PL_TXSTATUS_ACTL          Pump_PL_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(Pump_PL_TXCLKGEN_DP)
        #define Pump_PL_TXBITCLKGEN_CTR        Pump_PL_TXBITCLKGEN_CTR_REG
        #define Pump_PL_TXBITCLKTX_COMPLETE    Pump_PL_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define Pump_PL_TXBITCTR_PERIOD        Pump_PL_TXBITCTR_PERIOD_REG
        #define Pump_PL_TXBITCTR_CONTROL       Pump_PL_TXBITCTR_CONTROL_REG
        #define Pump_PL_TXBITCTR_COUNTER       Pump_PL_TXBITCTR_COUNTER_REG
    #endif /* Pump_PL_TXCLKGEN_DP */
#endif /* End Pump_PL_TX_ENABLED */

#if(Pump_PL_HD_ENABLED)
    #define Pump_PL_TXDATA                 Pump_PL_TXDATA_REG
    #define Pump_PL_TXSTATUS               Pump_PL_TXSTATUS_REG
    #define Pump_PL_TXSTATUS_MASK          Pump_PL_TXSTATUS_MASK_REG
    #define Pump_PL_TXSTATUS_ACTL          Pump_PL_TXSTATUS_ACTL_REG
#endif /* End Pump_PL_HD_ENABLED */

#if( (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) )
    #define Pump_PL_RXDATA                 Pump_PL_RXDATA_REG
    #define Pump_PL_RXADDRESS1             Pump_PL_RXADDRESS1_REG
    #define Pump_PL_RXADDRESS2             Pump_PL_RXADDRESS2_REG
    #define Pump_PL_RXBITCTR_PERIOD        Pump_PL_RXBITCTR_PERIOD_REG
    #define Pump_PL_RXBITCTR_CONTROL       Pump_PL_RXBITCTR_CONTROL_REG
    #define Pump_PL_RXBITCTR_COUNTER       Pump_PL_RXBITCTR_COUNTER_REG
    #define Pump_PL_RXSTATUS               Pump_PL_RXSTATUS_REG
    #define Pump_PL_RXSTATUS_MASK          Pump_PL_RXSTATUS_MASK_REG
    #define Pump_PL_RXSTATUS_ACTL          Pump_PL_RXSTATUS_ACTL_REG
#endif /* End  (Pump_PL_RX_ENABLED) || (Pump_PL_HD_ENABLED) */

#if(Pump_PL_INTERNAL_CLOCK_USED)
    #define Pump_PL_INTCLOCK_CLKEN         Pump_PL_INTCLOCK_CLKEN_REG
#endif /* End Pump_PL_INTERNAL_CLOCK_USED */

#define Pump_PL_WAIT_FOR_COMLETE_REINIT    Pump_PL_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_Pump_PL_H */


/* [] END OF FILE */
