#include "STM32F446RE_USART_driver.h"

/* ----------------------------- Local Helpers ------------------------------ */

static void usart_set_baud(USART_RegDef_t *pUSART, uint32_t baud);
static void usart_handle_txe(USART_HandleBasic_t *h);
static void usart_handle_rxne(USART_HandleBasic_t *h);

/* ----------------------------- Clock Control ------------------------------ */

void USART_EnableClock(USART_RegDef_t *pUSART, uint8_t en)
{
    if (en)
    {
        if (pUSART == USART1)      USART1_PCLK_EN();
        else if (pUSART == USART2) USART2_PCLK_EN();
        else if (pUSART == USART3) USART3_PCLK_EN();
        else if (pUSART == UART4)  UART4_PCLK_EN();
    }
}

/* ----------------------------- Baudrate Setup ----------------------------- */

static void usart_set_baud(USART_RegDef_t *pUSART, uint32_t baud)
{
    uint32_t pclk, usartdiv;
    uint32_t mant, frac, temp = 0;

    if (pUSART == USART1 || pUSART == USART6)
        pclk = RCC_GetPCLK2Value();
    else
        pclk = RCC_GetPCLK1Value();

    if (pUSART->CR1 & (1 << USART_CR1_OVER8))
        usartdiv = (25 * pclk) / (2 * baud);
    else
        usartdiv = (25 * pclk) / (4 * baud);

    mant = usartdiv / 100;
    temp = mant << 4;

    frac = usartdiv - (mant * 100);

    if (pUSART->CR1 & (1 << USART_CR1_OVER8))
        frac = ((frac * 8) + 50) / 100;
    else
        frac = ((frac * 16) + 50) / 100;

    temp |= (frac & 0x0F);

    pUSART->BRR = temp;
}

/* ----------------------------- Initialization ----------------------------- */

void USART_InitBasic(USART_HandleBasic_t *h)
{
    uint32_t temp = 0;

    USART_EnableClock(h->Instance, ENABLE);

    if (h->Config.Mode == USART_MODE_RX)
        temp |= (1 << USART_CR1_RE);
    else if (h->Config.Mode == USART_MODE_TX)
        temp |= (1 << USART_CR1_TE);
    else if (h->Config.Mode == USART_MODE_TXRX)
        temp |= (1 << USART_CR1_RE) | (1 << USART_CR1_TE);

    temp |= h->Config.WordLength << USART_CR1_M;

    if (h->Config.Parity == USART_PARITY_EVEN)
        temp |= (1 << USART_CR1_PCE);
    else if (h->Config.Parity == USART_PARITY_ODD)
        temp |= (1 << USART_CR1_PCE) | (1 << USART_CR1_PS);

    h->Instance->CR1 = temp;

    /* CR2 */
    temp = 0;
    temp |= h->Config.StopBits << USART_CR2_STOP;
    h->Instance->CR2 = temp;

    /* CR3 */
    temp = 0;
    if (h->Config.FlowControl == USART_FLOW_CTS)
        temp |= (1 << USART_CR3_CTSE);
    else if (h->Config.FlowControl == USART_FLOW_RTS)
        temp |= (1 << USART_CR3_RTSE);
    else if (h->Config.FlowControl == USART_FLOW_CTS_RTS)
        temp |= (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE);

    h->Instance->CR3 = temp;

    usart_set_baud(h->Instance, h->Config.Baud);

    h->TxState = USART_READY;
    h->RxState = USART_READY;
}

/* ----------------------------- Enable / Disable ---------------------------- */

void USART_Enable(USART_RegDef_t *pUSART, uint8_t en)
{
    if (en)
        pUSART->CR1 |= (1 << USART_CR1_UE);
    else
        pUSART->CR1 &= ~(1 << USART_CR1_UE);
}

/* ----------------------------- Flag Status -------------------------------- */

uint8_t USART_ReadFlag(USART_RegDef_t *pUSART, uint32_t flag)
{
    return (pUSART->SR & flag) ? 1 : 0;
}

/* ----------------------------- Blocking TX -------------------------------- */

void USART_SendBlocking(USART_HandleBasic_t *h, uint8_t *buf, uint32_t len)
{
    while (len--)
    {
        while (!USART_ReadFlag(h->Instance, USART_FLAG_TXE));

        h->Instance->DR = *buf;
        buf++;
    }

    while (!USART_ReadFlag(h->Instance, USART_FLAG_TC));
}

/* ----------------------------- Blocking RX -------------------------------- */

void USART_ReceiveBlocking(USART_HandleBasic_t *h, uint8_t *buf, uint32_t len)
{
    while (len--)
    {
        while (!USART_ReadFlag(h->Instance, USART_FLAG_RXNE));

        *buf = (uint8_t)(h->Instance->DR & 0xFF);
        buf++;
    }
}

/* ----------------------------- Interrupt TX -------------------------------- */

uint8_t USART_SendIT(USART_HandleBasic_t *h, uint8_t *buf, uint32_t len)
{
    if (h->TxState != USART_BUSY_TX)
    {
        h->pTxBuf = buf;
        h->TxLen  = len;
        h->TxState = USART_BUSY_TX;

        h->Instance->CR1 |= (1 << USART_CR1_TXEIE);
        h->Instance->CR1 |= (1 << USART_CR1_TCIE);

        return USART_READY;
    }
    return USART_BUSY_TX;
}

/* ----------------------------- Interrupt RX -------------------------------- */

uint8_t USART_ReceiveIT(USART_HandleBasic_t *h, uint8_t *buf, uint32_t len)
{
    if (h->RxState != USART_BUSY_RX)
    {
        h->pRxBuf = buf;
        h->RxLen  = len;
        h->RxState = USART_BUSY_RX;

        h->Instance->CR1 |= (1 << USART_CR1_RXNEIE);

        return USART_READY;
    }
    return USART_BUSY_RX;
}

/* ----------------------------- Local Handlers ------------------------------- */

static void usart_handle_txe(USART_HandleBasic_t *h)
{
    if (h->TxLen > 0)
    {
        h->Instance->DR = *h->pTxBuf;
        h->pTxBuf++;
        h->TxLen--;
    }

    if (h->TxLen == 0)
        h->Instance->CR1 &= ~(1 << USART_CR1_TXEIE);
}

static void usart_handle_rxne(USART_HandleBasic_t *h)
{
    *h->pRxBuf = (uint8_t)(h->Instance->DR & 0xFF);

    h->pRxBuf++;
    h->RxLen--;

    if (h->RxLen == 0)
    {
        h->Instance->CR1 &= ~(1 << USART_CR1_RXNEIE);
        h->RxState = USART_READY;

        USART_ApplicationCallback(h, USART_EVENT_RX_DONE);
    }
}

/* ----------------------------- Interrupt Handler ---------------------------- */

void USART_IRQHandler(USART_HandleBasic_t *h)
{
    /* TXE */
    if (USART_ReadFlag(h->Instance, USART_FLAG_TXE) &&
        (h->Instance->CR1 & (1 << USART_CR1_TXEIE)))
    {
        usart_handle_txe(h);
    }

    /* TC */
    if (USART_ReadFlag(h->Instance, USART_FLAG_TC) &&
        (h->Instance->CR1 & (1 << USART_CR1_TCIE)))
    {
        h->Instance->SR &= ~(1 << USART_SR_TC);
        h->TxState = USART_READY;
        USART_ApplicationCallback(h, USART_EVENT_TX_DONE);
    }

    /* RXNE */
    if (USART_ReadFlag(h->Instance, USART_FLAG_RXNE) &&
        (h->Instance->CR1 & (1 << USART_CR1_RXNEIE)))
    {
        usart_handle_rxne(h);
    }
}

/* ----------------------------- Weak Callback -------------------------------- */

__weak void USART_ApplicationCallback(USART_HandleBasic_t *h, uint8_t event)
{
    (void)h;
    (void)event;
}
