

#include <stdio.h>
#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_I2C_driver.h"

/* Flags */
static volatile uint8_t i2cRxDone = 0;

/* Slave address */
#define I2C_SLAVE_ADDR   0x68

/* Master own address */
#define I2C_MASTER_ADDR  0x61

/* Simple delay */
static void small_delay(void)
{
    for (uint32_t i = 0; i < (500000u / 2u); i++)
    {
        __asm__("nop");
    }
}

/* Global I2C handle */
static I2C_HandleSimple_t gI2C1Handle;

/* RX buffer */
static uint8_t rxBuffer[40];

/*
 * PB6 → SCL
 * PB7 → SDA
 */
static void i2c1_gpio_init(void)
{
    GPIO_Handle_t pins;
    memset(&pins, 0, sizeof(pins));

    pins.pGPIOx = GPIOB;
    pins.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    pins.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_OD;
    pins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;
    pins.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;

    /* SCL */
    pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&pins);

    /* SDA */
    pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&pins);
}

static void i2c1_init(void)
{
    memset(&gI2C1Handle, 0, sizeof(gI2C1Handle));

    gI2C1Handle.pI2Cx = I2C1;
    gI2C1Handle.Config.ACKControl  = I2C_ACK_ENABLE;
    gI2C1Handle.Config.DeviceAddr  = I2C_MASTER_ADDR;
    gI2C1Handle.Config.SCLSpeed    = I2C_SCL_SPEED_SM;
    gI2C1Handle.Config.FMDutyCycle = I2C_FM_DUTY_2;

    I2C_InitSimple(&gI2C1Handle);
}

static void button_init(void)
{
    GPIO_Handle_t btn;

    memset(&btn, 0, sizeof(btn));
    btn.pGPIOx = GPIOA;

    btn.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    btn.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;

    GPIO_Init(&btn);
}

int main(void)
{
    uint8_t command = 0;
    uint8_t dataLen = 0;

    printf("Custom I2C Master RX (IT) Example\n");

    button_init();
    i2c1_gpio_init();
    i2c1_init();

    /* Enable interrupts */
    I2C_ConfigIRQ(IRQ_NO_I2C1_EV, ENABLE);
    I2C_ConfigIRQ(IRQ_NO_I2C1_ER, ENABLE);

    /* Enable I2C peripheral */
    I2C_Enable(I2C1, ENABLE);

    /* Enable ACK */
    I2C_ConfigACK(I2C1, ENABLE);

    while (1)
    {
        /* Wait for button */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* ------------------- Step 1: Request length (0x51) ------------------- */
        command = 0x51;

        while (I2C_MasterSendIT(&gI2C1Handle, &command, 1, I2C_SLAVE_ADDR, I2C_ENABLE_SR)
               != I2C_READY);

        while (I2C_MasterReceiveIT(&gI2C1Handle, &dataLen, 1, I2C_SLAVE_ADDR, I2C_ENABLE_SR)
               != I2C_READY);

        /* ------------------- Step 2: Request full data (0x52) ---------------- */
        command = 0x52;

        while (I2C_MasterSendIT(&gI2C1Handle, &command, 1, I2C_SLAVE_ADDR, I2C_ENABLE_SR)
               != I2C_READY);

        while (I2C_MasterReceiveIT(&gI2C1Handle, rxBuffer, dataLen,
                                   I2C_SLAVE_ADDR, I2C_DISABLE_SR)
               != I2C_READY);

        /* Wait until RX finished */
        i2cRxDone = 0;
        while (!i2cRxDone);

        rxBuffer[dataLen] = '\0';

        printf("Received: %s\n", rxBuffer);
    }
}

/* ----------------------------- IRQ HANDLERS -------------------------------- */

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandler(&gI2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandler(&gI2C1Handle);
}

/* ------------------------------ CALLBACK ----------------------------------- */

void I2C_ApplicationCallback(I2C_HandleSimple_t *hI2C, uint8_t evt)
{
    switch (evt)
    {
        case I2C_EVENT_TX_DONE:
            printf("TX completed\n");
            break;

        case I2C_EVENT_RX_DONE:
            printf("RX completed\n");
            i2cRxDone = 1;
            break;

        case I2C_ERROR_ACK_FAIL:
            printf("ACK failure\n");
            I2C_CloseSending(hI2C);
            I2C_Stop(I2C1);
            while (1);
            break;

        default:
            break;
    }
}
