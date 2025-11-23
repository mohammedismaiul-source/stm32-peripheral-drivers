

#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_SPI_driver.h"

/* Small software delay for button debouncing */
static void app_delay(void)
{
    for (uint32_t i = 0; i < 500000u; i++)
    {
        __asm__("nop");
    }
}

/*
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_SCK
 * PB12 -> SPI2_NSS
 * AF5
 */
static void spi2_gpio_init(void)
{
    GPIO_Handle_t spiPins;

    memset(&spiPins, 0, sizeof(spiPins));

    spiPins.pGPIOx = GPIOB;
    spiPins.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    spiPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    spiPins.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_PP;
    spiPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
    spiPins.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;

    /* SCK */
    spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&spiPins);

    /* MOSI */
    spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&spiPins);

    /* We don't use MISO in this example */
    // spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&spiPins);

    /* NSS */
    spiPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&spiPins);
}

static void spi2_init(void)
{
    static SPI_HandleSimple_t spiHandle;

    memset(&spiHandle, 0, sizeof(spiHandle));

    spiHandle.Instance = SPI2;
    spiHandle.Config.Mode            = SPI_MODE_MASTER;
    spiHandle.Config.BusConfig       = SPI_BUS_FULL_DUPLEX;
    spiHandle.Config.BaudRate        = SPI_BAUD_DIV32;
    spiHandle.Config.DataFrameFormat = SPI_FRAME_8BIT;
    spiHandle.Config.ClockPolarity   = SPI_POLARITY_LOW;
    spiHandle.Config.ClockPhase      = SPI_PHASE_FIRST_EDGE;
    spiHandle.Config.SwSlaveManage   = SPI_SW_SLAVE_DISABLE;  /* HW NSS */

    SPI_InitSimple(&spiHandle);
}

/* Button on PA0 */
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
    char message[] =
            "An Arduino Uno board is suited for beginners, while Arduino Mega "
            "is useful when many I/O pins are required.";

    uint8_t msgLen = (uint8_t)strlen(message);

    button_init();
    spi2_gpio_init();
    spi2_init();

    /* Let hardware manage NSS automatically */
    SPI_ConfigSSOE(SPI2, ENABLE);

    while (1)
    {
        /* Wait for user button press */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        app_delay();

        /* Enable SPI peripheral */
        SPI_Enable(SPI2, ENABLE);

        /* First send length of message */
        SPI_SendBlocking(SPI2, &msgLen, 1);

        /* Then send message bytes */
        SPI_SendBlocking(SPI2, (uint8_t *)message, msgLen);

        /* Wait until SPI not busy */
        while (SPI_ReadFlag(SPI2, SPI_FLAG_BUSY));

        /* Disable SPI2 until next transfer */
        SPI_Enable(SPI2, DISABLE);
    }

    return 0;
}
