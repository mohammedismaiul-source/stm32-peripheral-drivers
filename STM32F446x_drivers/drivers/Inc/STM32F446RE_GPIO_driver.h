/*
 *
 *      Author: moham
 */

#ifndef STM32F446RE_GPIO_DRIVER_H_
#define STM32F446RE_GPIO_DRIVER_H_

#include "STM32F446RE.h"

/*
 * GPIO pin configuration structure
 */
typedef struct
{
    uint8_t GPIO_PinNumber;      /* Pin index 0..15 */
    uint8_t GPIO_PinMode;        /* See @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;       /* See @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl; /* Pull-up / pull-down config */
    uint8_t GPIO_PinOPType;      /* Output type: push-pull / open-drain */
    uint8_t GPIO_PinAltFunMode;  /* Alternate function selection */
} GPIO_PinConfig_t;

/*
 * GPIO handle structure: port base address + pin config
 */
typedef struct
{
    GPIO_RegDef_t   *pGPIOx;      /* Pointer to GPIO port base address */
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0             0
#define GPIO_PIN_NO_1             1
#define GPIO_PIN_NO_2             2
#define GPIO_PIN_NO_3             3
#define GPIO_PIN_NO_4             4
#define GPIO_PIN_NO_5             5
#define GPIO_PIN_NO_6             6
#define GPIO_PIN_NO_7             7
#define GPIO_PIN_NO_8             8
#define GPIO_PIN_NO_9             9
#define GPIO_PIN_NO_10            10
#define GPIO_PIN_NO_11            11
#define GPIO_PIN_NO_12            12
#define GPIO_PIN_NO_13            13
#define GPIO_PIN_NO_14            14
#define GPIO_PIN_NO_15            15

/*
 * @GPIO_PIN_MODES
 * GPIO pin modes
 */
#define GPIO_MODE_IN              0
#define GPIO_MODE_OUT             1
#define GPIO_MODE_ALTFN           2
#define GPIO_MODE_ANALOG          3
#define GPIO_MODE_IT_FT           4  /* Interrupt on falling edge  */
#define GPIO_MODE_IT_RT           5  /* Interrupt on rising edge   */
#define GPIO_MODE_IT_RFT          6  /* Interrupt on both edges    */

/*
 * GPIO output types
 */
#define GPIO_OP_TYPE_PP           0  /* Push-pull  */
#define GPIO_OP_TYPE_OD           1  /* Open-drain */

/*
 * @GPIO_PIN_SPEED
 * GPIO output speeds
 */
#define GPIO_SPEED_LOW            0
#define GPIO_SPEED_MEDIUM         1
#define GPIO_SPEED_FAST           2
#define GPIO_SPEED_HIGH           3

/*
 * GPIO pull-up / pull-down configuration
 */
#define GPIO_NO_PUPD              0
#define GPIO_PIN_PU               1
#define GPIO_PIN_PD               2


/*
 * Clock control for GPIO port
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t  GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteToOutputPin  (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin   (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling       (uint8_t PinNumber);

#endif /* STM32F446RE_GPIO_DRIVER_H_ */
