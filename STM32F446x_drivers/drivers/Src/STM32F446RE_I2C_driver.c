/*
 * STM32F446RE_I2C_driver.c
 *
 *  Basic I2C driver implementation for STM32F446RE.
 */

#include "STM32F446RE_I2C_driver.h"

/* ---------- Local helper functions (static) ---------- */

static void i2c_generate_start(I2C_RegDef_t *pI2Cx);
static void i2c_send_addr_write(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void i2c_send_addr_read(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
static void i2c_clear_addr_flag(I2C_HandleBasic_t *pHandle);
static void i2c_handle_txe_interrupt(I2C_HandleBasic_t *pHandle);
static void i2c_handle_rxne_interrupt(I2C_HandleBasic_t *pHandle);

/* ---------- Static helpers ---------- */

static void i2c_generate_start(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1U << I2C_CR1_START);
}

static void i2c_send_addr_write(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
    uint8_t addr = (uint8_t)(slaveAddr << 1);
    addr &= ~(1U);
    pI2Cx->DR = addr;
}

static void i2c_send_addr_read(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
    uint8_t addr = (uint8_t)(slaveAddr << 1);
    addr |= 1U;
    pI2Cx->DR = addr;
}

static void i2c_clear_addr_flag(I2C_HandleBasic_t *pHandle)
{
    (void)pHandle->pI2Cx->SR1;
    (void)pHandle->pI2Cx->SR2;
}

/* ---------- Basic utility functions ---------- */

uint8_t I2C_ReadFlag(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    return (pI2Cx->SR1 & FlagName) ? FLAG_SET : FLAG_RESET;
}

void I2C_EnablePeripheral(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1U << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1U << I2C_CR1_PE);
    }
}

void I2C_ConfigAck(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == I2C_ACK_ON)
    {
        pI2Cx->CR1 |= (1U << I2C_CR1_ACK);
    }
    else
    {
        pI2Cx->CR1 &= ~(1U << I2C_CR1_ACK);
    }
}

void I2C_GenerateStop(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1U << I2C_CR1_STOP);
}

/* ---------- Clock control ---------- */

void I2C_ClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        /* Clock disable not used here */
    }
}

/* ---------- Init / Reset ---------- */

void I2C_InitPeripheral(I2C_HandleBasic_t *pI2CHandle)
{
    uint32_t temp = 0U;
    uint32_t pclk1 = 0U;
    uint16_t ccrValue = 0U;

    /* Enable peripheral clock */
    I2C_ClockControl(pI2CHandle->pI2Cx, ENABLE);

    /* Configure ACK */
    if (pI2CHandle->I2C_Config.I2C_AckEnable == I2C_ACK_ON)
    {
        I2C_ConfigAck(pI2CHandle->pI2Cx, I2C_ACK_ON);
    }
    else
    {
        I2C_ConfigAck(pI2CHandle->pI2Cx, I2C_ACK_OFF);
    }

    /* Configure CR2 FREQ (input clock in MHz) */
    pclk1 = RCC_GetPCLK1Value();
    temp = (pclk1 / 1000000U) & 0x3FU;
    pI2CHandle->pI2Cx->CR2 = temp;

    /* Program own address in OAR1 */
    temp = 0U;
    temp |= (pI2CHandle->I2C_Config.I2C_OwnAddress << 1);
    temp |= (1U << 14); /* bit 14 must be 1 for 7-bit addr */
    pI2CHandle->pI2Cx->OAR1 = temp;

    /* Configure CCR register (clock control) */
    temp = 0U;
    if (pI2CHandle->I2C_Config.I2C_ClockSpeed <= I2C_SPEED_STANDARD)
    {
        /* Standard mode (<= 100kHz) */
        ccrValue = (uint16_t)(pclk1 / (2U * pI2CHandle->I2C_Config.I2C_ClockSpeed));
        temp |= (ccrValue & 0x0FFFU);
    }
    else
    {
        /* Fast mode */
        temp |= (1U << 15); /* F/S bit */
        temp |= ((uint32_t)pI2CHandle->I2C_Config.I2C_FastModeDuty << 14);

        if (pI2CHandle->I2C_Config.I2C_FastModeDuty == I2C_DUTY_2)
        {
            ccrValue = (uint16_t)(pclk1 / (3U * pI2CHandle->I2C_Config.I2C_ClockSpeed));
        }
        else
        {
            ccrValue = (uint16_t)(pclk1 / (25U * pI2CHandle->I2C_Config.I2C_ClockSpeed));
        }

        temp |= (ccrValue & 0x0FFFU);
    }
    pI2CHandle->pI2Cx->CCR = temp;

    /* Configure TRISE */
    if (pI2CHandle->I2C_Config.I2C_ClockSpeed <= I2C_SPEED_STANDARD)
    {
        temp = (pclk1 / 1000000U) + 1U;
    }
    else
    {
        temp = (uint32_t)(((pclk1 / 1000000U) * 300U) / 1000U) + 1U;
    }
    pI2CHandle->pI2Cx->TRISE = (temp & 0x3FU);

    pI2CHandle->TxRxState = I2C_STATE_READY;
}

void I2C_Reset(I2C_RegDef_t *pI2Cx)
{
    /* Simple software reset: toggle SWRST bit */
    pI2Cx->CR1 |= (1U << I2C_CR1_SWRST);
    pI2Cx->CR1 &= ~(1U << I2C_CR1_SWRST);
}

/* ---------- Blocking Master APIs ---------- */

void I2C_MasterSendBlocking(I2C_HandleBasic_t *pI2CHandle,
                            uint8_t *pTxbuffer,
                            uint32_t Len,
                            uint8_t SlaveAddr,
                            uint8_t RepeatedStart)
{
    /* 1. Generate START condition */
    i2c_generate_start(pI2CHandle->pI2Cx);

    /* 2. Wait for SB flag */
    while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_SB));

    /* 3. Send address (write) */
    i2c_send_addr_write(pI2CHandle->pI2Cx, SlaveAddr);

    /* 4. Wait for ADDR */
    while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_ADDR));

    /* 5. Clear ADDR */
    i2c_clear_addr_flag(pI2CHandle);

    /* 6. Send all bytes */
    while (Len > 0U)
    {
        while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_TXE));
        pI2CHandle->pI2Cx->DR = *pTxbuffer;
        pTxbuffer++;
        Len--;
    }

    /* Wait for TXE and BTF before STOP */
    while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_TXE));
    while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_BTF));

    if (RepeatedStart == I2C_REPEAT_START_DISABLE)
    {
        I2C_GenerateStop(pI2CHandle->pI2Cx);
    }
}

void I2C_MasterReceiveBlocking(I2C_HandleBasic_t *pI2CHandle,
                               uint8_t *pRxBuffer,
                               uint8_t Len,
                               uint8_t SlaveAddr,
                               uint8_t RepeatedStart)
{
    /* 1. Generate START */
    i2c_generate_start(pI2CHandle->pI2Cx);

    /* 2. Wait SB */
    while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_SB));

    /* 3. Send address for read */
    i2c_send_addr_read(pI2CHandle->pI2Cx, SlaveAddr);

    /* 4. Wait ADDR */
    while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_ADDR));

    if (Len == 1U)
    {
        /* Single-byte reception */
        I2C_ConfigAck(pI2CHandle->pI2Cx, I2C_ACK_OFF);
        i2c_clear_addr_flag(pI2CHandle);

        while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_RXNE));

        if (RepeatedStart == I2C_REPEAT_START_DISABLE)
        {
            I2C_GenerateStop(pI2CHandle->pI2Cx);
        }

        *pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->DR;
    }
    else
    {
        /* Multi-byte reception */
        i2c_clear_addr_flag(pI2CHandle);

        for (uint32_t i = Len; i > 0U; i--)
        {
            while (!I2C_ReadFlag(pI2CHandle->pI2Cx, I2C_STATUS_RXNE));

            if (i == 2U)
            {
                I2C_ConfigAck(pI2CHandle->pI2Cx, I2C_ACK_OFF);
                if (RepeatedStart == I2C_REPEAT_START_DISABLE)
                {
                    I2C_GenerateStop(pI2CHandle->pI2Cx);
                }
            }

            *pRxBuffer = (uint8_t)pI2CHandle->pI2Cx->DR;
            pRxBuffer++;
        }
    }

    /* Re-enable ACK if originally enabled */
    if (pI2CHandle->I2C_Config.I2C_AckEnable == I2C_ACK_ON)
    {
        I2C_ConfigAck(pI2CHandle->pI2Cx, I2C_ACK_ON);
    }
}

/* ---------- Interrupt-based Master APIs ---------- */

uint8_t I2C_MasterSendIT(I2C_HandleBasic_t *pI2CHandle,
                         uint8_t *pTxbuffer,
                         uint32_t Len,
                         uint8_t SlaveAddr,
                         uint8_t RepeatedStart)
{
    uint8_t state = pI2CHandle->TxRxState;

    if ((state != I2C_STATE_BUSY_TX) && (state != I2C_STATE_BUSY_RX))
    {
        pI2CHandle->pTxBuffer     = pTxbuffer;
        pI2CHandle->TxLen         = Len;
        pI2CHandle->TxRxState     = I2C_STATE_BUSY_TX;
        pI2CHandle->DevAddr       = SlaveAddr;
        pI2CHandle->RepeatedStart = RepeatedStart;

        /* Generate START */
        i2c_generate_start(pI2CHandle->pI2Cx);

        /* Enable event / buffer / error interrupts */
        pI2CHandle->pI2Cx->CR2 |= (1U << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1U << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1U << I2C_CR2_ITERREN);
    }

    return state;
}

uint8_t I2C_MasterReceiveIT(I2C_HandleBasic_t *pI2CHandle,
                            uint8_t *pRxBuffer,
                            uint8_t Len,
                            uint8_t SlaveAddr,
                            uint8_t RepeatedStart)
{
    uint8_t state = pI2CHandle->TxRxState;

    if ((state != I2C_STATE_BUSY_TX) && (state != I2C_STATE_BUSY_RX))
    {
        pI2CHandle->pRxBuffer     = pRxBuffer;
        pI2CHandle->RxLen         = Len;
        pI2CHandle->RxSize        = Len;
        pI2CHandle->TxRxState     = I2C_STATE_BUSY_RX;
        pI2CHandle->DevAddr       = SlaveAddr;
        pI2CHandle->RepeatedStart = RepeatedStart;

        i2c_generate_start(pI2CHandle->pI2Cx);

        pI2CHandle->pI2Cx->CR2 |= (1U << I2C_CR2_ITBUFEN);
        pI2CHandle->pI2Cx->CR2 |= (1U << I2C_CR2_ITEVTEN);
        pI2CHandle->pI2Cx->CR2 |= (1U << I2C_CR2_ITERREN);
    }

    return state;
}

/* ---------- Close Rx/Tx (IT mode) ---------- */

void I2C_CloseRx(I2C_HandleBasic_t *pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_STATE_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen     = 0U;
    pI2CHandle->RxSize    = 0U;

    if (pI2CHandle->I2C_Config.I2C_AckEnable == I2C_ACK_ON)
    {
        I2C_ConfigAck(pI2CHandle->pI2Cx, I2C_ACK_ON);
    }
}

void I2C_CloseTx(I2C_HandleBasic_t *pI2CHandle)
{
    pI2CHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1U << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_STATE_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen     = 0U;
}

/* ---------- Simple slave APIs ---------- */

void I2C_SlaveWrite(I2C_RegDef_t *pI2C, uint8_t data)
{
    pI2C->DR = data;
}

uint8_t I2C_SlaveRead(I2C_RegDef_t *pI2C)
{
    return (uint8_t)pI2C->DR;
}

/* ---------- NVIC configuration ---------- */

void I2C_ConfigIRQ(uint8_t IRQNumber, uint8_t EnOrDi)
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

void I2C_SetIRQPriority(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprIndex   = IRQNumber / 4U;
    uint8_t iprSection = IRQNumber % 4U;
    uint8_t shift      = (8U * iprSection) + (8U - NO_PR_BITS_IMPLEMENTED);

    NVIC_PR_BASE_ADDR[iprIndex] |= (IRQPriority << shift);
}

/* ---------- Helper for TXE/RXNE interrupts ---------- */

static void i2c_handle_txe_interrupt(I2C_HandleBasic_t *pHandle)
{
    if (pHandle->TxLen > 0U)
    {
        pHandle->pI2Cx->DR = *(pHandle->pTxBuffer);
        pHandle->pTxBuffer++;
        pHandle->TxLen--;
    }
}

static void i2c_handle_rxne_interrupt(I2C_HandleBasic_t *pHandle)
{
    if (pHandle->RxSize == 1U)
    {
        *pHandle->pRxBuffer = (uint8_t)pHandle->pI2Cx->DR;
        pHandle->RxLen--;
    }
    else if (pHandle->RxSize > 1U)
    {
        if (pHandle->RxLen == 2U)
        {
            I2C_ConfigAck(pHandle->pI2Cx, I2C_ACK_OFF);
        }

        *pHandle->pRxBuffer = (uint8_t)pHandle->pI2Cx->DR;
        pHandle->pRxBuffer++;
        pHandle->RxLen--;
    }

    if (pHandle->RxLen == 0U)
    {
        if (pHandle->RepeatedStart == I2C_REPEAT_START_DISABLE)
        {
            I2C_GenerateStop(pHandle->pI2Cx);
        }

        I2C_CloseRx(pHandle);
        I2C_ApplicationCallback(pHandle, I2C_EVENT_RX_COMPLETE);
    }
}

/* ---------- EV IRQ handler ---------- */

void I2C_HandleEventIRQ(I2C_HandleBasic_t *pI2CHandle)
{
    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1U << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1U << I2C_CR2_ITBUFEN);

    /* SB event */
    temp3 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_SB);
    if (temp1 && temp3)
    {
        if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX)
        {
            i2c_send_addr_write(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if (pI2CHandle->TxRxState == I2C_STATE_BUSY_RX)
        {
            i2c_send_addr_read(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    /* ADDR event */
    temp3 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_ADDR);
    if (temp1 && temp3)
    {
        i2c_clear_addr_flag(pI2CHandle);
    }

    /* BTF event */
    temp3 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_BTF);
    if (temp1 && temp3)
    {
        if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX)
        {
            if ((pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_TXE)) && (pI2CHandle->TxLen == 0U))
            {
                if (pI2CHandle->RepeatedStart == I2C_REPEAT_START_DISABLE)
                {
                    I2C_GenerateStop(pI2CHandle->pI2Cx);
                }

                I2C_CloseTx(pI2CHandle);
                I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_TX_COMPLETE);
            }
        }
    }

    /* STOPF event (slave mode) */
    temp3 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_STOPF);
    if (temp1 && temp3)
    {
        (void)pI2CHandle->pI2Cx->SR1;
        pI2CHandle->pI2Cx->CR1 |= 0x0000U;

        I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_STOP_DETECTED);
    }

    /* TXE event */
    temp3 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_TXE);
    if (temp1 && temp2 && temp3)
    {
        if (pI2CHandle->pI2Cx->SR2 & (1U << I2C_SR2_MSL))
        {
            if (pI2CHandle->TxRxState == I2C_STATE_BUSY_TX)
            {
                i2c_handle_txe_interrupt(pI2CHandle);
            }
        }
        else
        {
            if (pI2CHandle->pI2Cx->SR2 & (1U << I2C_SR2_TRA))
            {
                I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_DATA_REQUEST);
            }
        }
    }

    /* RXNE event */
    temp3 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_RXNE);
    if (temp1 && temp2 && temp3)
    {
        if (pI2CHandle->pI2Cx->SR2 & (1U << I2C_SR2_MSL))
        {
            if (pI2CHandle->TxRxState == I2C_STATE_BUSY_RX)
            {
                i2c_handle_rxne_interrupt(pI2CHandle);
            }
        }
        else
        {
            if (!(pI2CHandle->pI2Cx->SR2 & (1U << I2C_SR2_TRA)))
            {
                I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_DATA_RECEIVE);
            }
        }
    }
}

/* ---------- ER IRQ handler ---------- */

void I2C_HandleErrorIRQ(I2C_HandleBasic_t *pI2CHandle)
{
    uint32_t temp1, temp2;

    temp2 = pI2CHandle->pI2Cx->CR2 & (1U << I2C_CR2_ITERREN);

    /* BERR */
    temp1 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_BERR);
    if (temp1 && temp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_BERR);
        I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_ERROR_BERR);
    }

    /* ARLO */
    temp1 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_ARLO);
    if (temp1 && temp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_ARLO);
        I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_ERROR_ARLO);
    }

    /* AF */
    temp1 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_AF);
    if (temp1 && temp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_AF);
        I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_ERROR_AF);
    }

    /* OVR */
    temp1 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_OVR);
    if (temp1 && temp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_OVR);
        I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_ERROR_OVR);
    }

    /* TIMEOUT */
    temp1 = pI2CHandle->pI2Cx->SR1 & (1U << I2C_SR1_TIMEOUT);
    if (temp1 && temp2)
    {
        pI2CHandle->pI2Cx->SR1 &= ~(1U << I2C_SR1_TIMEOUT);
        I2C_ApplicationCallback(pI2CHandle, I2C_EVENT_ERROR_TIMEOUT);
    }
}

/* ---------- Slave callback enable/disable ---------- */

void I2C_SlaveConfigCallbacks(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR2 |= (1U << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1U << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1U << I2C_CR2_ITERREN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1U << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1U << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1U << I2C_CR2_ITERREN);
    }
}

/* ---------- Weak default callback ---------- */

__weak void I2C_ApplicationCallback(I2C_HandleBasic_t *pI2CHandle, uint8_t AppEvent)
{
    /* User can override this in application code */
    (void)pI2CHandle;
    (void)AppEvent;
}
