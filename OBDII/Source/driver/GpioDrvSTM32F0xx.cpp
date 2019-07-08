/**
 * See the file LICENSE for redistribution information.
 *
 * Copyright (c) 2009-2016 ObdDiag.Net. All rights reserved.
 *
 */

#include "cortexm.h"
#include "GpioDrv.h"

static GPIO_TypeDef* const GPIOPtr[]  = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE };

/**
 * Enabling GPIOx clock
 * @param[in] portNum GPIO number (0..7)
 */
void GPIOConfigure(uint32_t portNum)
{
    RCC->AHBENR |= (1UL << (portNum + 17));
}

/**
 * Setting GPIO pin direction
 * @param[in] portNum GPIO number (0..7)
 * @param[in] pinNum Port pin number
 * @param[in] dir GPIO_DIR_INPUT, GPIO_DIR_OUTPUT
 */
void GPIOSetDir(uint32_t portNum, uint32_t pinNum, uint32_t dir)
{
    GPIO_TypeDef* gpio = GPIOPtr[portNum];
    if (dir == GPIO_OUTPUT) {
        gpio->MODER   &= ~(0x00000003         << pinNum * 2);  // Clear it
        gpio->MODER   |=  (GPIO_Mode_OUT      << pinNum * 2);  // Output mode
        gpio->OTYPER  &= ~(0x00000001         << pinNum);      // Output push-pull by clearing the flag
        gpio->OSPEEDR &= ~(0x00000003         << pinNum * 2);  // Clear it
        gpio->OSPEEDR |=  (GPIO_Speed_Level_2 << pinNum * 2);  // Port output speed Medium 10 MHz
        gpio->PUPDR   &= ~(0x00000003         << pinNum * 2);  // No pull-up or pull-down by clearing the flag
    }
    else {
        gpio->MODER &= ~(0x00000003 << pinNum * 2); // Input mode by clearing the flag
        gpio->PUPDR &= ~(0x00000003 << 2 * pinNum); // No pull-up or pull-down by clearing the flag
    }
}

/**
 * Setting the port pin value
 * @param[in] portNum GPIO number (0..7)
 * @param[in] pinNum Port pin number
 * @param[in] val Port pin value (0 or 1)
 */
void GPIOPinWrite(uint32_t portNum, uint32_t pinNum, uint32_t val)
{
    GPIOPtr[portNum]->BSRR = val ? (0x01 << pinNum) : (0x01 << (pinNum + 16));
}

/**
 * Read port pin
 * @param[in] portNum GPIO number (0..7)
 * @param[in] pinNum Port pin number
 * @return pin value (0 or 1)
 */
uint32_t GPIOPinRead (uint32_t portNum, uint32_t pinNum)
{
    return (GPIOPtr[portNum]->IDR & (0x01 << pinNum)) ? 1 : 0;
}

/**
 * Setting CPU-specific port attributes, like open drain and etc.
 * @param[in] portNum GPIO number (0..7)
 * @param[in] pinNum Port pin number
 * @param[in] val Port attribute
 */
void GPIOPinConfig(uint32_t portNum, uint32_t pinNum, uint32_t val)
{
    if (val == GPIO_OPEN_DRAIN) {
        GPIOPtr[portNum]->OTYPER |= (0x01 << pinNum);
    }
}

/**
 * Configure the GPIO I/O mode
 * @param[in] portNum GPIO number (0..7)
 * @param[in] pinNum Port pin number
 * @param[in] mode Mode for the pin, GPIO_Mode_IN,GPIO_Mode_OUT,GPIO_Mode_AF
 */
void GPIOPinModeConfig(uint32_t portNum, uint32_t pinNum, uint32_t mode)
{
    GPIO_TypeDef* gpio = GPIOPtr[portNum];
    gpio->MODER &= ~(GPIO_MODER_MODER0 << (pinNum * 2));
    gpio->MODER |= (mode << (pinNum * 2));
}

/**
 * Configure the GPIO AF
 * @param[in] portNum GPIO number (0..7)
 * @param[in] pinNum Port pin number
 * @param[in] modeAF AF value for the pin, GPIO_AF_0 - GPIO_AF_5
 */
void GPIOPinAFConfig(uint32_t portNum, uint32_t pinNum, uint32_t modeAF)
{
    GPIO_TypeDef* gpio = GPIOPtr[portNum];
    gpio->AFR[pinNum >> 0x03] &= ~(0x0F << ((pinNum & 0x07) * 4));
    gpio->AFR[pinNum >> 0x03] |= modeAF << ((pinNum & 0x07) * 4);
}
