
/*
 *
 *      Author: moham
 */
#ifndef STM32F446RE_SPI_DRIVER_H_
#define STM32F446RE_SPI_DRIVER_H_

#include "STM32F446RE.h"

/*
 * Basic configuration structure for SPI peripheral
 */
typedef struct
{
    uint8_t Mode;            /* Master / Slave mode              */
    uint8_t BusConfig;       /* Full duplex / half duplex / RX only */
    uint8_t BaudRate;        /* Clock division factor            */
    uint8_t DataFrameFormat; /* 8-bit or 16-bit frame            */
    uint8_t ClockPolarity;   /* CPOL                             */
    uint8_t ClockPhase;      /* CPHA                             */
    uint8_t SwSlaveManage;   /* Software slave management        */

} SPI_ConfigSimple_t;

/*
 * Handle structure for SPI peripheral
 */
typedef struct
{
    SPI_RegDef_t     *Instance;    /* SPI base address                 */
    SPI_ConfigSimple_t Config;     /* User configuration               */

    uint8_t  *pTxBuf;              /* Tx buffer pointer                */
    uint8_t  *pRxBuf;              /* Rx buffer pointer                */
    uint32_t  TxLen;               /* Tx length                        */
    uint32_t  RxLen;               /* Rx length                        */
    uint8_t   TxState;             /* Tx state                         */
    uint8_t   RxState;             /* Rx state                         */

} SPI_HandleSimple_t;

/*
 * SPI application states
 */
#define SPI_STATE_READY        0u
#define SPI_STATE_BUSY_RX      1u
#define SPI_STATE_BUSY_TX      2u

/*
 * SPI application events
 */
#define SPI_EVENT_TX_DONE      1u
#define SPI_EVENT_RX_DONE      2u
#define SPI_EVENT_OVR_ERROR    3u
#define SPI_EVENT_CRC_ERROR    4u

/*
 * @Mode
 */
#define SPI_MODE_SLAVE         0u
#define SPI_MODE_MASTER        1u

/*
 * @BusConfig
 */
#define SPI_BUS_FULL_DUPLEX        1u
#define SPI_BUS_HALF_DUPLEX        2u
#define SPI_BUS_SIMPLEX_RX_ONLY    3u

/*
 * @BaudRate (maps to BR bits)
 */
#define SPI_BAUD_DIV2          0u
#define SPI_BAUD_DIV4          1u
#define SPI_BAUD_DIV8          2u
#define SPI_BAUD_DIV16         3u
#define SPI_BAUD_DIV32         4u
#define SPI_BAUD_DIV64         5u
#define SPI_BAUD_DIV128        6u
#define SPI_BAUD_DIV256        7u

/*
 * @DataFrameFormat
 */
#define SPI_FRAME_8BIT         0u
#define SPI_FRAME_16BIT        1u

/*
 * @ClockPolarity
 */
#define SPI_POLARITY_LOW       0u
#define SPI_POLARITY_HIGH      1u

/*
 * @ClockPhase
 */
#define SPI_PHASE_FIRST_EDGE   0u
#define SPI_PHASE_SECOND_EDGE  1u

/*
 * @SwSlaveManage
 */
#define SPI_SW_SLAVE_DISABLE   0u
#define SPI_SW_SLAVE_ENABLE    1u

/*
 * SPI status flag definitions
 */
#define SPI_FLAG_TXE           (1u << SPI_SR_TXE)
#define SPI_FLAG_RXNE          (1u << SPI_SR_RXNE)
#define SPI_FLAG_BUSY          (1u << SPI_SR_BSY)


/*
 * Peripheral clock control
 */
void SPI_EnableClock(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void SPI_InitSimple(SPI_HandleSimple_t *pSPIHandle);
void SPI_Reset(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive (blocking)
 */
void SPI_SendBlocking(SPI_RegDef_t *pSPIx,
                      uint8_t *pTxBuffer,
                      uint32_t Length);

void SPI_ReceiveBlocking(SPI_RegDef_t *pSPIx,
                         uint8_t *pRxBuffer,
                         uint32_t Length);

/*
 * Data Send and Receive (interrupt mode)
 */
uint8_t SPI_SendIT(SPI_HandleSimple_t *pSPIHandle,
                   uint8_t *pTxBuffer,
                   uint32_t Length);

uint8_t SPI_ReceiveIT(SPI_HandleSimple_t *pSPIHandle,
                      uint8_t *pRxBuffer,
                      uint32_t Length);

/*
 * IRQ configuration and ISR handling
 */
void SPI_ConfigIRQ(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_SetIRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_HandleSimple_t *pSPIHandle);

/*
 * Other peripheral control APIs
 */
void    SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void    SPI_ConfigSSI(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void    SPI_ConfigSSOE(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_ReadFlag(SPI_RegDef_t *pSPIx, uint32_t FlagMask);
void    SPI_ClearOVR(SPI_RegDef_t *pSPIx);
void    SPI_StopTx(SPI_HandleSimple_t *pSPIHandle);
void    SPI_StopRx(SPI_HandleSimple_t *pSPIHandle);

/*
 * Application callback to be implemented by user code
 */
void SPI_ApplicationCallback(SPI_HandleSimple_t *pSPIHandle, uint8_t AppEvent);

#endif /* STM32F446RE_SPI_DRIVER_H_ */
