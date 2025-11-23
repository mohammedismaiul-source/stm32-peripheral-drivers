
#include "GPIO_Driver.h"

#define LOW  0

void EXTI15_10_IRQHandler(void);

void delay(void)
{
	for(uint32_t i=0; i<200000; i++);
}

int main(void)
{
	GPIO_Handle_t LED_Toggle, PushButtonInterrupt;


	// this part is special for GPIO output port A setting
	LED_Toggle.GPIOx = GPIOA;
	LED_Toggle.GPIO_PinConfig.pinNumber = GPIO_PIN_NO_5;
	LED_Toggle.GPIO_PinConfig.pinMode = GPIO_PIN_MODE_OUT;
	LED_Toggle.GPIO_PinConfig.pinOutSpeed = GPIO_HIGH_SPEED;
	LED_Toggle.GPIO_PinConfig.pinOutMode = GPIO_OUTPUT_PP;
	LED_Toggle.GPIO_PinConfig.pinPuPdControl = GPIO_NO_PU_PD;
	GPIO_ClkControl(GPIOA, ENABLE);
	GPIO_Init(&LED_Toggle);


	// this part is special for GPIO input port C setting
	PushButtonInterrupt.GPIOx = GPIOC;
	PushButtonInterrupt.GPIO_PinConfig.pinNumber = GPIO_PIN_NO_13;
	PushButtonInterrupt.GPIO_PinConfig.pinMode = GPIO_INTR_FT;
	PushButtonInterrupt.GPIO_PinConfig.pinOutSpeed = GPIO_HIGH_SPEED;
	PushButtonInterrupt.GPIO_PinConfig.pinPuPdControl = GPIO_NO_PU_PD;
	GPIO_ClkControl(GPIOC, ENABLE);
	GPIO_Init(&PushButtonInterrupt);

	// this part is special for Interrupt input port C setting
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, 0xF, ENABLE);


	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_ToggelOutPin(GPIOA, GPIO_PIN_NO_5);
	GPIO_IRQHandling(GPIO_PIN_NO_13);
}
