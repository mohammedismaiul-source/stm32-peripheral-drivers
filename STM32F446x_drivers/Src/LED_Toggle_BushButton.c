

#include "GPIO_Driver.h"

#define LOW  0

void delay(void)
{
	for(uint32_t i=0; i<200000; i++);
}

int main(void)
{
	GPIO_Handle_t LED_Toggle;
	GPIO_Handle_t PushButton;

	LED_Toggle.GPIOx = GPIOA;
	LED_Toggle.GPIO_PinConfig.pinNumber = GPIO_PIN_NO_5;
	LED_Toggle.GPIO_PinConfig.pinMode = GPIO_PIN_MODE_OUT;
	LED_Toggle.GPIO_PinConfig.pinOutSpeed = GPIO_HIGH_SPEED;
	LED_Toggle.GPIO_PinConfig.pinOutMode = GPIO_OUTPUT_PP;
	LED_Toggle.GPIO_PinConfig.pinPuPdControl = GPIO_NO_PU_PD;
	GPIO_ClkControl(GPIOA, ENABLE);
	GPIO_Init(&LED_Toggle);

	PushButton.GPIOx = GPIOC;
	PushButton.GPIO_PinConfig.pinNumber = GPIO_PIN_NO_13;
	PushButton.GPIO_PinConfig.pinMode = GPIO_PIN_MODE_IN;
	PushButton.GPIO_PinConfig.pinOutSpeed = GPIO_HIGH_SPEED;
	PushButton.GPIO_PinConfig.pinPuPdControl = GPIO_NO_PU_PD;

	GPIO_ClkControl(GPIOC, ENABLE);
	GPIO_Init(&PushButton);

	while(1)
	{
		if(GPIO_ReadFromPin(GPIOC, GPIO_PIN_NO_13) == LOW)
		{
			delay();
			GPIO_ToggelOutPin(GPIOA, GPIO_PIN_NO_5);
		}
	}

	return 0;
}
