
#include <stdio.h>
#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_I2C_driver.h"

/* Own address and slave address */
#define I2C_MASTER_ADDRESS   0x61U
#define I2C_SLAVE_ADDRESS    0x68U

/* I2C handle */
static I2C_HandleBasic_t gI2C1Handle;

/* Data to send */
static uint8_t i2cTxMessage[] = "I2C master transmit test message\n";

/* Small delay for button debouncing */
static void small_delay(void)
{
    for (uint32_t i = 0; i < (500000U / 2U); i++)
    {
        __asm__("nop");
    }
}

/*
 * PB6 -> SCL
 * PB7 or PB9 -> SDA
 *
 * Here we use:
 *  PB6 -> SCL
 *  PB9 -> SDA
 */
static void i2c1_pins_init(void)
{
    GPIO_Handle_t i2cPins;

    memset(&i2cPins, 0, sizeof(i2cPins));

    i2cPins.pGPIOx = GPIOB;
    i2cPins.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    i2cPins.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_OD;
    i2cPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;     /* internal pull-up */
    i2cPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    i2cPins.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;

    /* SCL: PB6 */
    i2cPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&i2cPins);

    /* SDA: PB9 */
    i2cPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&i2cPins);
}

/* Configure I2C1 peripheral */
static void i2c1_init(void)
{
    memset(&gI2C1Handle, 0, sizeof(gI2C1Handle));

    gI2C1Handle.pI2Cx = I2C1;

    gI2C1Handle.I2C_Config.I2C_ClockSpeed   = I2C_SPEED_STANDARD;
    gI2C1Handle.I2C_Config.I2C_OwnAddress   = I2C_MASTER_ADDRESS;
    gI2C1Handle.I2C_Config.I2C_AckEnable    = I2C_ACK_ON;
    gI2C1Handle.I2C_Config.I2C_FastModeDuty = I2C_DUTY_2;

    I2C_InitPeripheral(&gI2C1Handle);
}

/* User button: PA0 */
static void button_init(void)
{
    GPIO_Handle_t btn;

    memset(&btn, 0, sizeof(btn));

    btn.pGPIOx = GPIOA;
    btn.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&btn);
}

int main(void)
{
    button_init();
    i2c1_pins_init();
    i2c1_init();

    /* Enable I2C1 peripheral and ACK */
    I2C_EnablePeripheral(I2C1, ENABLE);
    I2C_ConfigAck(I2C1, I2C_ACK_ON);

    while (1)
    {
        /* Wait until button is pressed */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* Send data to slave */
        I2C_MasterSendBlocking(&gI2C1Handle,
                               i2cTxMessage,
                               (uint32_t)strlen((char *)i2cTxMessage),
                               I2C_SLAVE_ADDRESS,
                               I2C_REPEAT_START_DISABLE);
    }

    return 0;
}
