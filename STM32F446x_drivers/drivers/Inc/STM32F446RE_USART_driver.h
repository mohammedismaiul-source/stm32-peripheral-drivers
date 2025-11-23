/*
 *
 *      Author: moham
 */
#ifndef STM32F446RE_USART_DRIVER_H_
#define STM32F446RE_USART_DRIVER_H_

#include "STM32F446RE.h"

/*
 * USART configuration structure (simple version)
 */
typedef struct
{
    uint8_t Mode;              /* TX / RX / Both                     */
    uint32_t BaudRate;         /* Baud rate                          */
    uint8_t StopBits;          /* Number of stop bits                */
    uint8_t WordLength;        /* 8 or 9 bits                        */
    uint8_t Parity;            /* None / Even / Odd                  */
    uint8_t FlowControl;       /* Flow control options               */

} USART_ConfigSimple_t;

/*
 * USART handle structure
 */
typedef struct
{
    USART_RegDef_t      *Instance;     /* Base address of USART peripheral */
    USART_ConfigSimple_t Config;       /* User settings                    */

    uint8_t  *pTxBuffer;               /* Transmit buffer pointer          */
    uint8_t  *pRxBuffer;               /* Receive buffer pointer           */
    uint32_t TxLen;                    /* Transmission length              */
    uint32_t RxLen;                    /* Reception length                 */

    uint8_t  TxState;                  /* State: busy or ready             */
    uint8_t  RxState;                  /* State: busy or ready             */

} USART_HandleSimple_t;

/*
 * @Mode
 */
#define USART_MODE_TX_ONLY          0u
#define USART_MODE_RX_ONLY          1u
#define USART_MODE_TX_RX            2u

/*
 * @BaudRate
 */
#define USART_BAUD_9600             9600u
#define USART_BAUD_19200            19200u
#define USART_BAUD_38400            38400u
#define USART_BAUD_57600            57600u
#define USART_BAUD_115200           115200u
#define USART_BAUD_230400           230400u
#define USART_BAUD_460800           460800u
#define USART_BAUD_921600           921600u
#define USART_BAUD_2M               2000000u

/*
 * @WordLength
 */
#define USART_WORDLEN_8BIT          0u
#define USART_WORDLEN_9BIT          1u

/*
 * @StopBits
 */
#define USART_STOP_1                0u
#define USART_STOP_0_5              1u
#define USART_STOP_2                2u
#define USART_STOP_1_5              3u

/*
 * @Parity
 */
#define USART_PARITY_NONE           0u
#define USART_PARITY_EVEN           1u
#define USART_PARITY_ODD            2u

/*
 * @FlowControl
 */
#define USART_FLOW_NONE             0u
#define USART_FLOW_CTS              1u
#define USART_FLOW_RTS              2u
#define USART_FLOW_CTS_RTS          3u

/*
 * USART status flags
 */
#define USART_FLAG_TXE              (1u << USART_SR_TXE)
#define USART_FLAG_RXNE             (1u << USART_SR_RXNE)
#define USART_FLAG_TC               (1u << USART_SR_TC)

/*
 * USART states
 */
#define USART_READY                 0u
#define USART_BUSY_TX               1u
#define USART_BUSY_RX               2u

/*
 * USART events for application callback
 */
#define USART_EVENT_TX_DONE         0u
#define USART_EVENT_RX_DONE         1u
#define USART_EVENT_IDLE_LINE       2u
#define USART_EVENT_CTS_SIGNAL      3u
#define USART_EVENT_PARITY_ERR      4u
#define USART_EVENT_FRAME_ERR       5u
#define USART_EVENT_NOISE_ERR       6u
#define USART_EVENT_OVERRUN_ERR     7u

/*
 * Clock control
 */
void USART_EnableClock(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and reset
 */
void USART_InitSimple(USART_HandleSimple_t *pUSARTHandle);
void USART_Reset(USART_RegDef_t *pUSARTx);

/*
 * Blocking send and receive
 */
void USART_SendBlocking(USART_HandleSimple_t *pUSARTHandle,
                        uint8_t *pTxBuffer,
                        uint32_t Len);

void USART_ReceiveBlocking(USART_HandleSimple_t *pUSARTHandle,
                           uint8_t *pRxBuffer,
                           uint32_t Len);

/*
 * Interrupt-based send/receive
 */
uint8_t USART_SendIT(USART_HandleSimple_t *pUSARTHandle,
                     uint8_t *pTxBuffer,
                     uint32_t Len);

uint8_t USART_ReceiveIT(USART_HandleSimple_t *pUSARTHandle,
                        uint8_t *pRxBuffer,
                        uint32_t Len);

/*
 * IRQ configuration & handler
 */
void USART_ConfigIRQ(uint8_t IRQNumber, uint8_t EnOrDi);
void USART_SetIRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandler(USART_HandleSimple_t *pUSARTHandle);

/*
 * Miscellaneous control APIs
 */
uint8_t USART_ReadFlag(USART_RegDef_t *pUSARTx, uint32_t FlagMask);
void    USART_Enable(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void    USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application callback (implemented by user code)
 */
void USART_AppCallback(USART_HandleSimple_t *pUSARTHandle, uint8_t Event);

#endif /* STM32F446RE_USART_DRIVER_H_ */
