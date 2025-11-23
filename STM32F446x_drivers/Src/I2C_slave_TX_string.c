
#include <stdio.h>
#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_I2C_driver.h"

/* Slave address */
#define SLAVE_ADDR     0x69U

/* Global I2C1 handle */
static I2C_HandleBasic_t gI2C1Handle;

/* Data buffer to send to master */
static uint8_t slaveTxBuffer[32] = "STM32 Slave mode testing..";

/* Simple button delay */
static void small_delay(void)
{
    for (uint32_t i = 0; i < (500000u / 2u); i++)
    {
        __asm__("nop");
    }
}

/*
 * PB6 -> SCL
 * PB7 -> SDA
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

    gI2C1Handle.I2C_Config.I2C_AckEnable    = I2C_ACK_ON;
    gI2C1Handle.I2C_Config.I2C_OwnAddress   = SLAVE_ADDR;
    gI2C1Handle.I2C_Config.I2C_ClockSpeed   = I2C_SPEED_STANDARD;
    gI2C1Handle.I2C_Config.I2C_FastModeDuty = I2C_DUTY_2;

    I2C_InitPeripheral(&gI2C1Handle);
}

static void button_init(void)
{
    GPIO_Handle_t btn;

    memset(&btn, 0, sizeof(btn));
    btn.pGPIOx = GPIOA;

    btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&btn);
}

int main(void)
{
    button_init();
    i2c1_gpio_init();
    i2c1_init();

    /* Configure interrupts */
    I2C_ConfigIRQ(IRQ_NO_I2C1_EV, ENABLE);
    I2C_ConfigIRQ(IRQ_NO_I2C1_ER, ENABLE);

    /* Enable callback events */
    I2C_SlaveConfigCallbacks(I2C1, ENABLE);

    /* Enable peripheral */
    I2C_EnablePeripheral(I2C1, ENABLE);

    /* Enable ACK */
    I2C_ConfigAck(I2C1, I2C_ACK_ON);

    while (1)
    {
        /* Idle loop for slave */
    }
}

/* -------------------- IRQ Handlers ---------------------- */

void I2C1_EV_IRQHandler(void)
{
    I2C_HandleEventIRQ(&gI2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
    I2C_HandleErrorIRQ(&gI2C1Handle);
}

/* -------------------- Application Callback ---------------------- */

void I2C_ApplicationCallback(I2C_HandleBasic_t *pI2C, uint8_t event)
{
    static uint8_t cmdCode = 0;
    static uint8_t byteIndex = 0;

    switch (event)
    {
        case I2C_EVENT_DATA_REQUEST:
            /* Master is asking for data */
            if (cmdCode == 0x51)
            {
                /* Send length */
                I2C_SlaveWrite(pI2C->pI2Cx, (uint8_t)strlen((char *)slaveTxBuffer));
            }
            else if (cmdCode == 0x52)
            {
                /* Send actual string bytes */
                I2C_SlaveWrite(pI2C->pI2Cx, slaveTxBuffer[byteIndex++]);
            }
            break;

        case I2C_EVENT_DATA_RECEIVE:
            /* Master has sent a command */
            cmdCode = I2C_SlaveRead(pI2C->pI2Cx);
            break;

        case I2C_EVENT_ERROR_AF:
            /* Master NACKed → stop sending */
            cmdCode = 0xFF;
            byteIndex = 0;
            break;

        case I2C_EVENT_STOP_DETECTED:
            /* STOP condition — end of transaction */
            break;

        default:
            break;
    }
}
