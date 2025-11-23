

#include <stdio.h>
#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_SPI_driver.h"

/* Command codes sent to Arduino */
#define CMD_LED_CTRL        0x50
#define CMD_SENSOR_READ     0x51
#define CMD_LED_STATUS      0x52
#define CMD_PRINT_STRING    0x53
#define CMD_ID_REQUEST      0x54

/* LED control */
#define LED_ON      1
#define LED_OFF     0

/* Arduino analog pins */
#define A0_PIN      0
#define A1_PIN      1
#define A2_PIN      2
#define A3_PIN      3
#define A4_PIN      4

#define ARDUINO_LED_PIN   9

static void small_delay(void)
{
    for (uint32_t i = 0; i < (500000 / 2); i++)
    {
        __asm__("nop");
    }
}

/*
 * PB14 → SPI2_MISO
 * PB15 → SPI2_MOSI
 * PB13 → SPI2_SCK
 * PB12 → SPI2_NSS (HW)
 * AF5
 */
static void spi2_gpio_init(void)
{
    GPIO_Handle_t pin;

    memset(&pin, 0, sizeof(pin));
    pin.pGPIOx = GPIOB;
    pin.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    pin.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    pin.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_PP;
    pin.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
    pin.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;

    /* SCK */
    pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&pin);

    /* MOSI */
    pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&pin);

    /* MISO */
    pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&pin);

    /* NSS */
    pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&pin);
}

static void spi2_init(void)
{
    static SPI_HandleSimple_t spi;

    memset(&spi, 0, sizeof(spi));
    spi.Instance = SPI2;
    spi.Config.Mode            = SPI_MODE_MASTER;
    spi.Config.BusConfig       = SPI_BUS_FULL_DUPLEX;
    spi.Config.BaudRate        = SPI_BAUD_DIV32;
    spi.Config.DataFrameFormat = SPI_FRAME_8BIT;
    spi.Config.ClockPolarity   = SPI_POLARITY_LOW;
    spi.Config.ClockPhase      = SPI_PHASE_FIRST_EDGE;
    spi.Config.SwSlaveManage   = SPI_SW_SLAVE_DISABLE; /* HW NSS */

    SPI_InitSimple(&spi);
}

/* Button: PA0 */
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

/* Simple response check from slave */
static uint8_t verify_ack(uint8_t byte)
{
    return (byte == 0xF5) ? 1u : 0u;
}

int main(void)
{
    uint8_t dummyTx = 0xFF;
    uint8_t dummyRx = 0;
    uint8_t cmd     = 0;
    uint8_t ack     = 0;
    uint8_t args[2];

    printf("Custom SPI Command Handler Running\n");

    button_init();
    spi2_gpio_init();
    spi2_init();

    /* Enable automatic NSS handling */
    SPI_ConfigSSOE(SPI2, ENABLE);

    while (1)
    {
        /* Wait for button press */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* Enable SPI peripheral */
        SPI_Enable(SPI2, ENABLE);

        /* ---------------------------- COMMAND: LED CTRL --------------------------- */
        cmd = CMD_LED_CTRL;
        SPI_SendBlocking(SPI2, &cmd, 1);
        SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

        SPI_SendBlocking(SPI2, &dummyTx, 1);
        SPI_ReceiveBlocking(SPI2, &ack, 1);

        if (verify_ack(ack))
        {
            args[0] = ARDUINO_LED_PIN;
            args[1] = LED_ON;

            SPI_SendBlocking(SPI2, args, 2);
            SPI_ReceiveBlocking(SPI2, args, 2);

            printf("LED Control Command Executed\n");
        }

        /* Button press again */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* ---------------------------- COMMAND: SENSOR READ ----------------------- */
        cmd = CMD_SENSOR_READ;
        SPI_SendBlocking(SPI2, &cmd, 1);
        SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

        SPI_SendBlocking(SPI2, &dummyTx, 1);
        SPI_ReceiveBlocking(SPI2, &ack, 1);

        if (verify_ack(ack))
        {
            args[0] = A0_PIN;

            SPI_SendBlocking(SPI2, args, 1);
            SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

            small_delay();

            SPI_SendBlocking(SPI2, &dummyTx, 1);

            uint8_t adcVal = 0;
            SPI_ReceiveBlocking(SPI2, &adcVal, 1);

            printf("Sensor Value: %d\n", adcVal);
        }

        /* Button again */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* ---------------------------- COMMAND: LED STATUS ------------------------ */
        cmd = CMD_LED_STATUS;
        SPI_SendBlocking(SPI2, &cmd, 1);
        SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

        SPI_SendBlocking(SPI2, &dummyTx, 1);
        SPI_ReceiveBlocking(SPI2, &ack, 1);

        if (verify_ack(ack))
        {
            args[0] = ARDUINO_LED_PIN;

            SPI_SendBlocking(SPI2, args, 1);
            SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

            small_delay();

            SPI_SendBlocking(SPI2, &dummyTx, 1);

            uint8_t ledState = 0;
            SPI_ReceiveBlocking(SPI2, &ledState, 1);

            printf("LED State: %d\n", ledState);
        }

        /* Again button press */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* ---------------------------- COMMAND: PRINT STRING ---------------------- */
        cmd = CMD_PRINT_STRING;
        SPI_SendBlocking(SPI2, &cmd, 1);
        SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

        SPI_SendBlocking(SPI2, &dummyTx, 1);
        SPI_ReceiveBlocking(SPI2, &ack, 1);

        uint8_t msg[] = "Hello from custom SPI!";
        if (verify_ack(ack))
        {
            args[0] = strlen((char *)msg);
            SPI_SendBlocking(SPI2, args, 1);
            SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

            small_delay();

            for (uint32_t i = 0; i < args[0]; i++)
            {
                SPI_SendBlocking(SPI2, &msg[i], 1);
                SPI_ReceiveBlocking(SPI2, &dummyRx, 1);
            }

            printf("Sent string to Arduino\n");
        }

        /* Again button press */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* ---------------------------- COMMAND: ID READ --------------------------- */
        cmd = CMD_ID_REQUEST;
        SPI_SendBlocking(SPI2, &cmd, 1);
        SPI_ReceiveBlocking(SPI2, &dummyRx, 1);

        SPI_SendBlocking(SPI2, &dummyTx, 1);
        SPI_ReceiveBlocking(SPI2, &ack, 1);

        if (verify_ack(ack))
        {
            uint8_t idData[11];
            for (uint32_t i = 0; i < 10; i++)
            {
                SPI_SendBlocking(SPI2, &dummyTx, 1);
                SPI_ReceiveBlocking(SPI2, &idData[i], 1);
            }
            idData[10] = '\0';

            printf("Device ID: %s\n", idData);
        }

        while (SPI_ReadFlag(SPI2, SPI_FLAG_BUSY));
        SPI_Enable(SPI2, DISABLE);

        printf("SPI Session Closed\n");
    }

    return 0;
}
