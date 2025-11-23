/*
 * STM32F446RE_GPIO_driver.c
 *
 *  Basic GPIO driver implementation for STM32F446RE.
 */

#include "STM32F446RE_GPIO_driver.h"

/*
 * Enable or disable clock for a given GPIO port
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
            GPIOI_PCLK_EN();
        }
    }
    else
    {
        /* Clock disable not used in this simple driver */
    }
}

/*
 * Initialize a GPIO pin based on configuration in handle
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t regValue = 0;
    uint8_t  pin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;

    /* 1. Enable clock for the port */
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

    /* 2. Configure mode */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        /* Non-interrupt mode */
        regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2U * pin);

        /* Clear and set MODER bits */
        pGPIOHandle->pGPIOx->MODER &= ~(0x3U << (2U * pin));
        pGPIOHandle->pGPIOx->MODER |= regValue;
    }
    else
    {
        /* Interrupt modes */

        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            /* Falling edge trigger */
            EXTI->FTSR |= (1U << pin);
            EXTI->RTSR &= ~(1U << pin);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            /* Rising edge trigger */
            EXTI->RTSR |= (1U << pin);
            EXTI->FTSR &= ~(1U << pin);
        }
        else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            /* Both edges */
            EXTI->RTSR |= (1U << pin);
            EXTI->FTSR |= (1U << pin);
        }

        /* Select port in SYSCFG_EXTICR */
        uint8_t index   = pin / 4U;
        uint8_t section = pin % 4U;
        uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[index] &= ~(0xFU << (section * 4U));
        SYSCFG->EXTICR[index] |=  (portCode << (section * 4U));

        /* Unmask interrupt line in IMR */
        EXTI->IMR |= (1U << pin);
    }

    /* 3. Configure speed */
    regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2U * pin);
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3U << (2U * pin));
    pGPIOHandle->pGPIOx->OSPEEDR |= regValue;

    /* 4. Configure pull-up / pull-down */
    regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2U * pin);
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3U << (2U * pin));
    pGPIOHandle->pGPIOx->PUPDR |= regValue;

    /* 5. Configure output type */
    regValue = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pin;
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1U << pin);
    pGPIOHandle->pGPIOx->OTYPER |= regValue;

    /* 6. Configure alternate function (if needed) */
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        uint8_t afrIndex   = pin / 8U;
        uint8_t afrSection = pin % 8U;

        pGPIOHandle->pGPIOx->AFR[afrIndex] &= ~(0xFU << (4U * afrSection));
        pGPIOHandle->pGPIOx->AFR[afrIndex] |=
                (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4U * afrSection));
    }
}

/*
 * Reset a GPIO port
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
        GPIOI_REG_RESET();
    }
}

/*
 * Read single input pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t bitValue;

    bitValue = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x1U);

    return bitValue;
}

/*
 * Read full input port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    return (uint16_t)(pGPIOx->IDR);
}

/*
 * Write single output pin
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (1U << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1U << PinNumber);
    }
}

/*
 * Write full output port
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/*
 * Toggle output pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1U << PinNumber);
}

/*
 * Configure NVIC for a given IRQ number (enable/disable)
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31U)
        {
            *NVIC_ISER0 |= (1U << IRQNumber);
        }
        else if (IRQNumber < 64U)
        {
            *NVIC_ISER1 |= (1U << (IRQNumber % 32U));
        }
        else if (IRQNumber < 96U)
        {
            *NVIC_ISER2 |= (1U << (IRQNumber % 64U));
        }
    }
    else
    {
        if (IRQNumber <= 31U)
        {
            *NVIC_ICER0 |= (1U << IRQNumber);
        }
        else if (IRQNumber < 64U)
        {
            *NVIC_ICER1 |= (1U << (IRQNumber % 32U));
        }
        else if (IRQNumber < 96U)
        {
            *NVIC_ICER2 |= (1U << (IRQNumber % 64U));
        }
    }
}

/*
 * Configure priority for given IRQ number
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprIndex   = IRQNumber / 4U;
    uint8_t iprSection = IRQNumber % 4U;

    uint8_t shift = (8U * iprSection) + (8U - NO_PR_BITS_IMPLEMENTED);

    NVIC_PR_BASE_ADDR[iprIndex] |= (IRQPriority << shift);
}

/*
 * Clear pending interrupt for a given pin (EXTI line)
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    if (EXTI->PR & (1U << PinNumber))
    {
        /* Clear pending bit by writing 1 */
        EXTI->PR |= (1U << PinNumber);
    }
}
