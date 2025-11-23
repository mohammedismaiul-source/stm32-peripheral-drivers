

#include "GPIO_Driver.h"

void delay(void)
{
	for(uint32_t i=0; i<500000; i++);
}

int main(void)
{
	GPIO_Handle_t LED_Toggle;

	LED_Toggle.GPIOx = GPIOA;

	LED_Toggle.GPIO_PinConfig.pinNumber = GPIO_PIN_NO_5;
	LED_Toggle.GPIO_PinConfig.pinMode = GPIO_PIN_MODE_OUT;
	LED_Toggle.GPIO_PinConfig.pinOutSpeed = GPIO_HIGH_SPEED;
	LED_Toggle.GPIO_PinConfig.pinOutMode = GPIO_OUTPUT_PP;
	LED_Toggle.GPIO_PinConfig.pinPuPdControl = GPIO_NO_PU_PD;

	GPIO_ClkControl(GPIOA, ENABLE);

	GPIO_Init(&LED_Toggle);

	while(1)
	{
		GPIO_ToggelOutPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}
