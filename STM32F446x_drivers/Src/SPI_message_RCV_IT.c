/*
 * SPI_message_RCV_IT.c
 *
 * Example:
 *  - STM32F446RE as SPI master
 *  - Arduino as SPI slave
 *  - Message received in interrupt mode and printed via SWV / semihosting
 */

#include <stdio.h>
#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_SPI_driver.h"

/* Global SPI handle */
static SPI_HandleSimple_t gSpi2Handle;

/* Simple receive buffer */
#define SPI_RX_MAX_LEN   500u

static char spiRxBuffer[SPI_RX_MAX_LEN];
static volatile char spiRxByte = 0;
static volatile uint8_t spiMsgDone = 0;

/* This flag is set in EXTI handler when slave indicates "data ready" */
static volatile uint8_t spiDataReady = 0;

/* Small delay helper */
static void app_delay(void)
{
    for (uint32_t i = 0; i < (500000u / 2u); i++)
    {
        __asm__("nop");
    }
}

/*
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 -> SPI2_NSS
 * AF5
 */
static void spi2_gpio_init(void)
{
    GPIO_Handle_t pins;

    memset(&pins, 0, sizeof(pins));

    pins.pGPIOx = GPIOB;
    pins.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_ALTFN;
    pins.GPIO_PinConfig.GPIO_PinAltFunMode= 5;
    pins.GPIO_PinConfig.GPIO_PinOPType    = GPIO_OP_TYPE_PP;
    pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    pins.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;

    /* SCLK */
    pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&pins);

    /* MOSI */
    pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&pins);

    /* MISO */
    pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&pins);

    /* NSS */
    pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&pins);
}

static void spi2_init(void)
{
    memset(&gSpi2Handle, 0, sizeof(gSpi2Handle));

    gSpi2Handle.Instance = SPI2;
    gSpi2Handle.Config.Mode            = SPI_MODE_MASTER;
    gSpi2Handle.Config.BusConfig       = SPI_BUS_FULL_DUPLEX;
    gSpi2Handle.Config.BaudRate        = SPI_BAUD_DIV32;
    gSpi2Handle.Config.DataFrameFormat = SPI_FRAME_8BIT;
    gSpi2Handle.Config.ClockPolarity   = SPI_POLARITY_LOW;
    gSpi2Handle.Config.ClockPhase      = SPI_PHASE_FIRST_EDGE;
    gSpi2Handle.Config.SwSlaveManage   = SPI_SW_SLAVE_DISABLE; /* HW NSS */

    SPI_InitSimple(&gSpi2Handle);
}

/* EXTI pin used by slave (Arduino) to signal that data is ready */
static void slave_int_gpio_init(void)
{
    GPIO_Handle_t intPin;

    memset(&intPin, 0, sizeof(intPin));

    intPin.pGPIOx = GPIOD;
    intPin.GPIO_PinConfig.GPIO_PinNumber   = GPIO_PIN_NO_6;
    intPin.GPIO_PinConfig.GPIO_PinMode     = GPIO_MODE_IT_FT;
    intPin.GPIO_PinConfig.GPIO_PinSpeed    = GPIO_SPEED_LOW;
    intPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_Init(&intPin);

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
}

int main(void)
{
    uint8_t dummyByte = 0xFF;

    slave_int_gpio_init();
    spi2_gpio_init();
    spi2_init();

    /* Let hardware manage NSS automatically */
    SPI_ConfigSSOE(SPI2, ENABLE);

    /* SPI2 IRQ in NVIC */
    SPI_ConfigIRQ(IRQ_NO_SPI2, ENABLE);

    while (1)
    {
        spiMsgDone = 0;

        /* Wait until Arduino raises the interrupt pin */
        while (!spiDataReady)
        {
            app_delay();
        }

        /* Avoid re-entrance while message is in progress */
        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

        /* Enable SPI peripheral */
        SPI_Enable(SPI2, ENABLE);

        while (!spiMsgDone)
        {
            /* Start transmit/receive of one dummy byte at a time in IT mode */
            while (SPI_SendIT(&gSpi2Handle, &dummyByte, 1u) == SPI_STATE_BUSY_TX);
            while (SPI_ReceiveIT(&gSpi2Handle, (uint8_t *)&spiRxByte, 1u) == SPI_STATE_BUSY_RX);
        }

        /* Wait until SPI not busy */
        while (SPI_ReadFlag(SPI2, SPI_FLAG_BUSY));

        /* Disable SPI2 until next message */
        SPI_Enable(SPI2, DISABLE);

        printf("Received: %s\n", spiRxBuffer);

        spiDataReady = 0;

        /* Re-enable EXTI interrupt for next message */
        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    }
}

/* SPI2 global interrupt handler */
void SPI2_IRQHandler(void)
{
    SPI_IRQHandler(&gSpi2Handle);
}

/* Callback called from SPI driver */
void SPI_ApplicationCallback(SPI_HandleSimple_t *pSPIHandle, uint8_t event)
{
    static uint32_t index = 0;

    (void)pSPIHandle;

    if (event == SPI_EVENT_RX_DONE)
    {
        spiRxBuffer[index++] = spiRxByte;

        if (spiRxByte == '\0' || index >= SPI_RX_MAX_LEN)
        {
            spiMsgDone = 1;
            spiRxBuffer[index - 1] = '\0';
            index = 0;
        }
    }
}

/* EXTI line 9..5 interrupt handler (slave "data ready" signal) */
void EXTI9_5_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_6);
    spiDataReady = 1;
}
