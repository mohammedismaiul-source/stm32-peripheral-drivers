

#include <stdio.h>
#include <string.h>

#include "STM32F446RE.h"
#include "STM32F446RE_GPIO_driver.h"
#include "STM32F446RE_USART_driver.h"

/* Three different messages sent to Arduino */
static char *gMessages[3] =
{
    "hihihihihihi123",
    "Hello How are you ?",
    "Today is Monday !"
};

/* Buffer for data received from Arduino */
static char gRxBuffer[1024];

/* Global USART2 handle */
static USART_HandleSimple_t gUsart2Handle;

/* Flag to indicate reception complete */
static volatile uint8_t gRxComplete = 0;

/* Provided by semihosting support library */
extern void initialise_monitor_handles(void);

/* Simple delay for button debouncing */
static void small_delay(void)
{
    for (uint32_t i = 0; i < (500000U / 2U); i++)
    {
        __asm__("nop");
    }
}

/* PA2 -> USART2_TX, PA3 -> USART2_RX, AF7 */
static void usart2_gpio_init(void)
{
    GPIO_Handle_t usartPins;

    memset(&usartPins, 0, sizeof(usartPins));

    usartPins.pGPIOx = GPIOA;
    usartPins.GPIO_PinConfig.GPIO_PinMode       = GPIO_MODE_ALTFN;
    usartPins.GPIO_PinConfig.GPIO_PinOPType     = GPIO_OP_TYPE_PP;
    usartPins.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;
    usartPins.GPIO_PinConfig.GPIO_PinSpeed      = GPIO_SPEED_FAST;
    usartPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* TX: PA2 */
    usartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&usartPins);

    /* RX: PA3 */
    usartPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usartPins);
}

/* Configure USART2 peripheral */
static void usart2_init(void)
{
    memset(&gUsart2Handle, 0, sizeof(gUsart2Handle));

    gUsart2Handle.Instance = USART2;
    gUsart2Handle.Config.Mode       = USART_MODE_TX_RX;
    gUsart2Handle.Config.BaudRate   = USART_BAUD_115200;
    gUsart2Handle.Config.StopBits   = USART_STOP_1;
    gUsart2Handle.Config.WordLength = USART_WORDLEN_8BIT;
    gUsart2Handle.Config.Parity     = USART_PARITY_NONE;
    gUsart2Handle.Config.FlowControl= USART_FLOW_NONE;

    USART_InitSimple(&gUsart2Handle);
}

/* Button: PA0, LED: PD12 (not really used in logic, but kept as in original) */
static void button_led_init(void)
{
    GPIO_Handle_t btn;
    GPIO_Handle_t led;

    /* Button on PA0 */
    memset(&btn, 0, sizeof(btn));
    btn.pGPIOx = GPIOA;
    btn.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_0;
    btn.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_IN;
    btn.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;
    btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_Init(&btn);

    /* LED on PD12 */
    memset(&led, 0, sizeof(led));
    led.pGPIOx = GPIOD;
    led.GPIO_PinConfig.GPIO_PinNumber    = GPIO_PIN_NO_12;
    led.GPIO_PinConfig.GPIO_PinMode      = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinSpeed     = GPIO_SPEED_FAST;
    led.GPIO_PinConfig.GPIO_PinOPType    = GPIO_OP_TYPE_OD;
    led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_Init(&led);
}

int main(void)
{
    uint32_t msgIndex = 0;

    initialise_monitor_handles();

    usart2_gpio_init();
    usart2_init();
    button_led_init();

    /* Enable USART2 interrupt in NVIC */
    USART_ConfigIRQ(IRQ_NO_USART2, ENABLE);

    /* Enable USART2 peripheral */
    USART_Enable(USART2, ENABLE);

    printf("UART application is running\n");

    while (1)
    {
        /* Wait for button press */
        while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
        small_delay();

        /* Make sure index stays 0..2 */
        msgIndex = msgIndex % 3U;

        /* Expected reply has same length as transmitted message */
        uint32_t len = (uint32_t)strlen(gMessages[msgIndex]);

        /* Start reception first (interrupt mode) */
        while (USART_ReceiveIT(&gUsart2Handle, (uint8_t *)gRxBuffer, len) != USART_READY);

        /* Send message in blocking mode */
        USART_SendBlocking(&gUsart2Handle,
                           (uint8_t *)gMessages[msgIndex],
                           len);

        printf("Transmitted : %s\n", gMessages[msgIndex]);

        /* Wait until reception completes (set in callback) */
        while (!gRxComplete);

        /* Ensure string termination for printing */
        if (len < sizeof(gRxBuffer) - 1U)
        {
            gRxBuffer[len] = '\0';
        }
        else
        {
            gRxBuffer[sizeof(gRxBuffer) - 1U] = '\0';
        }

        printf("Received    : %s\n", gRxBuffer);

        /* Clear flag and move to next message */
        gRxComplete = 0;
        msgIndex++;
    }

    /* Should never reach here */
    return 0;
}

/* USART2 global IRQ handler */
void USART2_IRQHandler(void)
{
    USART_IRQHandler(&gUsart2Handle);
}

/* Application callback called from driver */
void USART_AppCallback(USART_HandleSimple_t *pUSARTHandle, uint8_t Event)
{
    (void)pUSARTHandle;

    if (Event == USART_EVENT_RX_DONE)
    {
        gRxComplete = 1;
    }
    else if (Event == USART_EVENT_TX_DONE)
    {
        /* Not used in this example */
    }
}
