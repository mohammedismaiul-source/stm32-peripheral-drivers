

#include "STM32F446RE_SPI_driver.h"

/* ---------- Local helper functions ---------- */

static void spi_handle_txe(SPI_HandleSimple_t *pHandle);
static void spi_handle_rxne(SPI_HandleSimple_t *pHandle);
static void spi_handle_ovr(SPI_HandleSimple_t *pHandle);

/* ---------- Utility flag function ---------- */

uint8_t SPI_ReadFlag(SPI_RegDef_t *pSPIx, uint32_t FlagMask)
{
    if (pSPIx->SR & FlagMask)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/* ---------- Clock control ---------- */

void SPI_EnableClock(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        /* Clock disable not used here */
    }
}

/* ---------- Init / Reset ---------- */

void SPI_InitSimple(SPI_HandleSimple_t *pSPIHandle)
{
    uint32_t temp = 0U;

    /* Enable peripheral clock */
    SPI_EnableClock(pSPIHandle->Instance, ENABLE);

    /* Mode (master/slave) */
    if (pSPIHandle->Config.Mode == SPI_MODE_MASTER)
    {
        temp |= (1U << SPI_CR1_MSTR);
    }

    /* Bus configuration */
    if (pSPIHandle->Config.BusConfig == SPI_BUS_FULL_DUPLEX)
    {
        /* Clear BIDIMODE for full duplex */
        temp &= ~(1U << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->Config.BusConfig == SPI_BUS_HALF_DUPLEX)
    {
        /* Set BIDIMODE for half duplex */
        temp |= (1U << SPI_CR1_BIDIMODE);
    }
    else if (pSPIHandle->Config.BusConfig == SPI_BUS_SIMPLEX_RX_ONLY)
    {
        temp &= ~(1U << SPI_CR1_BIDIMODE);
        temp |= (1U << SPI_CR1_RXONLY);
    }

    /* Baud rate (BR bits) */
    temp |= ((pSPIHandle->Config.BaudRate & 0x7U) << SPI_CR1_BR);

    /* Data frame format */
    if (pSPIHandle->Config.DataFrameFormat == SPI_FRAME_16BIT)
    {
        temp |= (1U << SPI_CR1_DFF);
    }

    /* Clock polarity and phase */
    temp |= ((pSPIHandle->Config.ClockPolarity & 0x1U) << SPI_CR1_CPOL);
    temp |= ((pSPIHandle->Config.ClockPhase   & 0x1U) << SPI_CR1_CPHA);

    /* Software slave management */
    temp |= ((pSPIHandle->Config.SwSlaveManage & 0x1U) << SPI_CR1_SSM);

    pSPIHandle->Instance->CR1 = temp;

    /* Clear states */
    pSPIHandle->TxState = SPI_STATE_READY;
    pSPIHandle->RxState = SPI_STATE_READY;
}

void SPI_Reset(SPI_RegDef_t *pSPIx)
{
    /* Simple reset can be done via RCC reset if needed.
       Left empty in this basic driver. */
    (void)pSPIx;
}

/* ---------- Blocking send / receive ---------- */

void SPI_SendBlocking(SPI_RegDef_t *pSPIx,
                      uint8_t *pTxBuffer,
                      uint32_t Length)
{
    while (Length > 0U)
    {
        /* Wait until TXE is set */
        while (SPI_ReadFlag(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

        if (pSPIx->CR1 & (1U << SPI_CR1_DFF))
        {
            /* 16-bit data frame */
            pSPIx->DR = *((uint16_t *)pTxBuffer);
            Length -= 2U;
            pTxBuffer += 2;
        }
        else
        {
            /* 8-bit data frame */
            pSPIx->DR = *pTxBuffer;
            Length -= 1U;
            pTxBuffer++;
        }
    }
}

void SPI_ReceiveBlocking(SPI_RegDef_t *pSPIx,
                         uint8_t *pRxBuffer,
                         uint32_t Length)
{
    while (Length > 0U)
    {
        /* Wait until RXNE is set */
        while (SPI_ReadFlag(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);

        if (pSPIx->CR1 & (1U << SPI_CR1_DFF))
        {
            /* 16-bit data frame */
            *((uint16_t *)pRxBuffer) = (uint16_t)pSPIx->DR;
            Length -= 2U;
            pRxBuffer += 2;
        }
        else
        {
            /* 8-bit data frame */
            *pRxBuffer = (uint8_t)pSPIx->DR;
            Length -= 1U;
            pRxBuffer++;
        }
    }
}

/* ---------- Interrupt based send / receive ---------- */

uint8_t SPI_SendIT(SPI_HandleSimple_t *pSPIHandle,
                   uint8_t *pTxBuffer,
                   uint32_t Length)
{
    uint8_t state = pSPIHandle->TxState;

    if (state != SPI_STATE_BUSY_TX)
    {
        pSPIHandle->pTxBuf  = pTxBuffer;
        pSPIHandle->TxLen   = Length;
        pSPIHandle->TxState = SPI_STATE_BUSY_TX;

        /* Enable TXE interrupt */
        pSPIHandle->Instance->CR2 |= (1U << SPI_CR2_TXEIE);
    }

    return state;
}

uint8_t SPI_ReceiveIT(SPI_HandleSimple_t *pSPIHandle,
                      uint8_t *pRxBuffer,
                      uint32_t Length)
{
    uint8_t state = pSPIHandle->RxState;

    if (state != SPI_STATE_BUSY_RX)
    {
        pSPIHandle->pRxBuf  = pRxBuffer;
        pSPIHandle->RxLen   = Length;
        pSPIHandle->RxState = SPI_STATE_BUSY_RX;

        /* Enable RXNE interrupt */
        pSPIHandle->Instance->CR2 |= (1U << SPI_CR2_RXNEIE);
    }

    return state;
}

/* ---------- NVIC configuration ---------- */

void SPI_ConfigIRQ(uint8_t IRQNumber, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (IRQNumber <= 31U)
        {
            *NVIC_ISER0 |= (1U << IRQNumber);
        }
        else if (IRQNumber < 64U)
        {
            *NVIC_ISER1 |= (1U << (IRQNumber % 32U));
        }
        else if (IRQNumber < 96U)
        {
            *NVIC_ISER2 |= (1U << (IRQNumber % 64U));
        }
    }
    else
    {
        if (IRQNumber <= 31U)
        {
            *NVIC_ICER0 |= (1U << IRQNumber);
        }
        else if (IRQNumber < 64U)
        {
            *NVIC_ICER1 |= (1U << (IRQNumber % 32U));
        }
        else if (IRQNumber < 96U)
        {
            *NVIC_ICER2 |= (1U << (IRQNumber % 64U));
        }
    }
}

void SPI_SetIRQPriority(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprIndex   = IRQNumber / 4U;
    uint8_t iprSection = IRQNumber % 4U;

    uint8_t shift = (8U * iprSection) + (8U - NO_PR_BITS_IMPLEMENTED);

    NVIC_PR_BASE_ADDR[iprIndex] |= (IRQPriority << shift);
}

/* ---------- IRQ handler ---------- */

void SPI_IRQHandler(SPI_HandleSimple_t *pSPIHandle)
{
    uint8_t temp1, temp2;

    /* TXE */
    temp1 = (pSPIHandle->Instance->SR & (1U << SPI_SR_TXE));
    temp2 = (pSPIHandle->Instance->CR2 & (1U << SPI_CR2_TXEIE));
    if (temp1 && temp2)
    {
        spi_handle_txe(pSPIHandle);
    }

    /* RXNE */
    temp1 = (pSPIHandle->Instance->SR & (1U << SPI_SR_RXNE));
    temp2 = (pSPIHandle->Instance->CR2 & (1U << SPI_CR2_RXNEIE));
    if (temp1 && temp2)
    {
        spi_handle_rxne(pSPIHandle);
    }

    /* OVR */
    temp1 = (pSPIHandle->Instance->SR & (1U << SPI_SR_OVR));
    temp2 = (pSPIHandle->Instance->CR2 & (1U << SPI_CR2_ERRIE));
    if (temp1 && temp2)
    {
        spi_handle_ovr(pSPIHandle);
    }
}

/* ---------- Control APIs ---------- */

void SPI_Enable(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1U << SPI_CR1_SPE);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR1_SPE);
    }
}

void SPI_ConfigSSI(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= (1U << SPI_CR1_SSI);
    }
    else
    {
        pSPIx->CR1 &= ~(1U << SPI_CR1_SSI);
    }
}

void SPI_ConfigSSOE(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pSPIx->CR2 |= (1U << SPI_CR2_SSOE);
    }
    else
    {
        pSPIx->CR2 &= ~(1U << SPI_CR2_SSOE);
    }
}

void SPI_ClearOVR(SPI_RegDef_t *pSPIx)
{
    volatile uint8_t temp;
    temp = (uint8_t)pSPIx->DR;
    temp = (uint8_t)pSPIx->SR;
    (void)temp;
}

void SPI_StopTx(SPI_HandleSimple_t *pSPIHandle)
{
    pSPIHandle->Instance->CR2 &= ~(1U << SPI_CR2_TXEIE);
    pSPIHandle->pTxBuf  = NULL;
    pSPIHandle->TxLen   = 0U;
    pSPIHandle->TxState = SPI_STATE_READY;
}

void SPI_StopRx(SPI_HandleSimple_t *pSPIHandle)
{
    pSPIHandle->Instance->CR2 &= ~(1U << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuf  = NULL;
    pSPIHandle->RxLen   = 0U;
    pSPIHandle->RxState = SPI_STATE_READY;
}

/* ---------- Local interrupt helpers ---------- */

static void spi_handle_txe(SPI_HandleSimple_t *pHandle)
{
    if (pHandle->Instance->CR1 & (1U << SPI_CR1_DFF))
    {
        /* 16-bit frame */
        pHandle->Instance->DR = *((uint16_t *)pHandle->pTxBuf);
        pHandle->TxLen -= 2U;
        pHandle->pTxBuf += 2;
    }
    else
    {
        /* 8-bit frame */
        pHandle->Instance->DR = *pHandle->pTxBuf;
        pHandle->TxLen--;
        pHandle->pTxBuf++;
    }

    if (pHandle->TxLen == 0U)
    {
        SPI_StopTx(pHandle);
        SPI_ApplicationCallback(pHandle, SPI_EVENT_TX_DONE);
    }
}

static void spi_handle_rxne(SPI_HandleSimple_t *pHandle)
{
    if (pHandle->Instance->CR1 & (1U << SPI_CR1_DFF))
    {
        /* 16-bit frame */
        *((uint16_t *)pHandle->pRxBuf) = (uint16_t)pHandle->Instance->DR;
        pHandle->RxLen -= 2U;
        pHandle->pRxBuf += 2;
    }
    else
    {
        /* 8-bit frame */
        *pHandle->pRxBuf = (uint8_t)pHandle->Instance->DR;
        pHandle->RxLen--;
        pHandle->pRxBuf++;
    }

    if (pHandle->RxLen == 0U)
    {
        SPI_StopRx(pHandle);
        SPI_ApplicationCallback(pHandle, SPI_EVENT_RX_DONE);
    }
}

static void spi_handle_ovr(SPI_HandleSimple_t *pHandle)
{
    /* Clear OVR only if not busy in TX */
    if (pHandle->TxState != SPI_STATE_BUSY_TX)
    {
        SPI_ClearOVR(pHandle->Instance);
    }

    SPI_ApplicationCallback(pHandle, SPI_EVENT_OVR_ERROR);
}

/* ---------- Weak application callback ---------- */

__weak void SPI_ApplicationCallback(SPI_HandleSimple_t *pSPIHandle, uint8_t AppEvent)
{
    /* User can override this in application code */
    (void)pSPIHandle;
    (void)AppEvent;
}
