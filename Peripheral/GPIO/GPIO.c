/**
 * @file GPIO.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for GPIO of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-07
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "GPIO.h"
#include "../RCC/RCC.h"
/******************************************************************************
 * Variables definition
 ******************************************************************************/

/**
*******************************************************************************
* @ Name : GPIO_Set_Mode
* @ Parameters: GPIOn *gpio_x, uint8_t pinx, uint8_t mode
* @ Registers : MODER
* @ Descriptions :
*		- Set GPIO mode for pinx of Port n:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + mode  MODER_INPUT
*                   MODER_OUTPUT
*                   MODER_ALTEMATE
*                   MODER_ANALOG
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
void GPIO_Set_Mode(GPIOn *gpio_x, uint8_t pinx, uint8_t mode)
{
    GPIOn_SET_MODER(gpio_x, pinx, mode);
}

/**
*******************************************************************************
* @ Name : GPIO_GetMode
* @ Parameters: GPIOn *gpio_x, uint8_t pinx
* @ Registers : MODER
* @ Descriptions :
*		- Get current mode in pinx of Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
uint8_t GPIO_GetMode(GPIOn *gpio_x, uint8_t pinx)
{
    uint32_t u32GPIOmode = GPIOn_GET_MODER(gpio_x);
    uint8_t u8Mode = (u32GPIOmode & (0x03 << (pinx * 2))) >> (pinx * 2);
    return u8Mode;
}

/**
*******************************************************************************
* @ Name : GPIO_Set_OutputType
* @ Parameters: GPIOn *gpio_x, uint8_t pinx, uint8_t type
* @ Registers : OTYPER
* @ Descriptions :
*		- If set MODER is Output, this function to set the output type of the pin.
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + type: OTYPER_PUSHPULL
                    OTYPER_OPENDRAIN
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
void GPIO_Set_OutputType(GPIOn *gpio_x, uint8_t pinx, uint8_t type)
{
    GPIOn_SET_OTYPER(gpio_x, pinx, type);
}

/**
*******************************************************************************
* @ Name : GPIO_GetOutputType
* @ Parameters: GPIOn *gpio_x, uint8_t pinx
* @ Registers : OTYPER
* @ Descriptions :
*		- Get current output type in pinx of Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
uint8_t GPIO_GetOutputType(GPIOn *gpio_x, uint8_t pinx)
{
    uint16_t u16OutputTypeRes = GPIOn_GET_OTYPER(gpio_x);
    return (u16OutputTypeRes & (0x01 << pinx)) >> pinx;
}

/**
*******************************************************************************
* @ Name : GPIO_Set_OutputSpeed
* @ Parameters: GPIOn *gpio_x, uint8_t pinx, uint8_t speed
* @ Registers : OSPEEDR
* @ Descriptions :
*		- If set MODER is Output, this function to set the output speed of the pin.
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + speed:    OSPEEDR_LOW,
*                       OSPEEDR_MEDIUM,
*                       OSPEEDR_HIGH,
*                       OSPEEDR_VERYHIGH
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
void GPIO_Set_OutputSpeed(GPIOn *gpio_x, uint8_t pinx, uint8_t speed)
{
    GPIOn_SET_OSPEEDR(gpio_x, pinx, speed);
}

/**
*******************************************************************************
* @ Name : GPIO_GetOutputSpeed
* @ Parameters: GPIOn *gpio_x, uint8_t pinx
* @ Registers : OSPEEDR
* @ Descriptions :
*		- Get current output speed in pinx of Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
uint8_t GPIO_GetOutputSpeed(GPIOn *gpio_x, uint8_t pinx)
{
    uint32_t u32OutputSpeed = GPIOn_GET_OSPEEDR(gpio_x);
    return (u32OutputSpeed & (0x03 << 2 * pinx)) >> (2 * pinx);
}

/**
*******************************************************************************
* @ Name : GPIO_GetInputData
* @ Parameters: GPIOn *gpio_x, uint8_t pinx
* @ Registers : IDR
* @ Descriptions :
*		- Get current input data (HIGH/LOW) in pinx of Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
uint8_t GPIO_GetInputData(GPIOn *gpio_x, uint8_t pinx)
{
    uint16_t u16InputData = GPIOn_GET_IDR(gpio_x);
    return (u16InputData & (0x01 << pinx)) >> pinx;
}

/**
*******************************************************************************
* @ Name : GPIO_SetOutputData
* @ Parameters: GPIOn *gpio_x, uint8_t pinx, uint8_t value
* @ Registers : ODR
* @ Descriptions :
*		- Set value to pinx of Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + value: SET(1) or CLEAR(0)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
void GPIO_SetOutputData(GPIOn *gpio_x, uint8_t pinx, uint8_t value)
{
    if (value == SET)
        GPIOn_SET_ODR(gpio_x, pinx, 0x01);
    else
        GPIOn_CLEAR_ODR(gpio_x, pinx, 0x01);
}

/**
*******************************************************************************
* @ Name : GPIO_GetDataInOutputRes
* @ Parameters: GPIOn *gpio_x, uint8_t pinx
* @ Registers : IDR
* @ Descriptions :
*		- Get current output data of the specified pin:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
uint8_t GPIO_GetDataInOutputRes(GPIOn *gpio_x, uint8_t pinx)
{
    uint16_t u16ODRData = GPIOn_GET_ODR(gpio_x);
    return (u16ODRData & (0x01 << pinx)) >> pinx;
}

/**
*******************************************************************************
* @ Name : GPIO_SettingOutputDataBSRR
* @ Parameters: GPIOn *gpio_x, uint8_t pinx, uint8_t value
* @ Registers : OSPEEDR
* @ Descriptions :
*		- If set MODER is Output, this function to set the output speed of the pin.
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + value:
*               + Higher 16-bit (BRy):
*                   . 0: No action on the corresponding ODRx bit
*                   . 1: Resets the corresponding ODRx bit
*               + Lower 16-bit (BSx):
*                   .0: No action on the corresponding ODRx bit
*                   .1: Sets the corresponding ODRx bit
*           + If both of them are 1, BSx more priority.
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-07
*******************************************************************************
*/
void GPIO_SettingOutputDataBSRR(GPIOn *gpio_x, uint8_t pinx, uint8_t value)
{
    if (value == SET)
        GPIOn_SET_BSRR(gpio_x, (0x01 << pinx));
    else
        GPIOn_SET_BSRR(gpio_x, (0x01 << (pinx + 16)));
}

/**
*******************************************************************************
* @ Name : GPIO_SetInputType
* @ Parameters: GPIOn *gpio_x, uint8_t pinx, uint8_t type
* @ Registers : PUPDR
* @ Descriptions :
*		- Set input type value to pinx of Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + type: PUPDR_NOTHING 
*                   PUPDR_PULLUP  
*                   PUPDR_PULLDOWN
*                   PUPDR_RESERVE 
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-11
*******************************************************************************
*/
void GPIO_SetInputType(GPIOn *gpio_x, uint8_t pinx, uint8_t type)
{
    GPIOn_SET_PUPDR(gpio_x, pinx, type);
}

/**
*******************************************************************************
* @ Name : GPIO_GetInputType
* @ Parameters: GPIOn *gpio_x, uint8_t pinx
* @ Registers : PUPDR
* @ Descriptions :
*		- Get current input type value of pinx in Portn:
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-11
*******************************************************************************
*/
uint8_t GPIO_GetInputType(GPIOn *gpio_x, uint8_t pinx)
{
    uint16_t u16InputTypeData = GPIOn_GET_PUPDR(gpio_x);
    return (u16InputTypeData & (0x01 << 2*pinx)) >> 2*pinx;
}


/**
*******************************************************************************
* @ Name : GPIO_EnableClock
* @ Parameters: GPIOn * gpio_x
* @ Registers : 
* @ Descriptions : APB1ENR,AHB1ENR, APB2ENR
*		- Configuration for GPIOx.
*           + gpiox: GPIO_A -> GPIO_K
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-02-24
*******************************************************************************
*/
void GPIO_EnableClock(GPIOn * gpio_x)
{
    // Enable clock for GPIOx
    if(gpio_x == (GPIOn *)ADDRESS_GPIO_A)
        RCC_EnablePeripheralClock(CLOCK_GPIO_A);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_B)
        RCC_EnablePeripheralClock(CLOCK_GPIO_B);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_C)
        RCC_EnablePeripheralClock(CLOCK_GPIO_C);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_D)
        RCC_EnablePeripheralClock(CLOCK_GPIO_D);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_E)
        RCC_EnablePeripheralClock(CLOCK_GPIO_E);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_F)
        RCC_EnablePeripheralClock(CLOCK_GPIO_F);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_G)
        RCC_EnablePeripheralClock(CLOCK_GPIO_G);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_H)
        RCC_EnablePeripheralClock(CLOCK_GPIO_H);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_I)
        RCC_EnablePeripheralClock(CLOCK_GPIO_I);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_J)
        RCC_EnablePeripheralClock(CLOCK_GPIO_J);
    else if(gpio_x == (GPIOn *)ADDRESS_GPIO_K)
        RCC_EnablePeripheralClock(CLOCK_GPIO_K);
    else
        return;
}

/**
*******************************************************************************
* @ Name : GPIO_Configuration
* @ Parameters: GPIOn * gpio_x, uint8_t pinx, uint8_t mode, uint8_t outtyper, uint8_t speed, uint8_t pull
* @ Registers : MODER, OTYPER, OSPEEDR, PUPDR
* @ Descriptions :
*		- Configuration for GPIOx.
*           + gpiox: GPIO_A -> GPIO_K
*           + pinx: GPIO_PIN0 -> GPIO_PIN15
*           + mode: GPIO_MODE_OUTPUT, GPIO_MODE_INPUT, GPIO_MODE_ALTERNATE, GPIO_MODE_ANALOG
*           + outtyper: OTYPER_PUSHPULL, OTYPER_OPENDRAIN
*           + speed: OSPEEDR_LOW, OSPEEDR_MEDIUM, OSPEEDR_HIGH, OSPEEDR_VERYHIGH
*           + pull: PUPDR_NOTHING, PUPDR_PULLUP, PUPDR_PULLDOWN, PUPDR_RESERVE
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-11
*******************************************************************************
*/
void GPIO_Configuration(GPIOn * gpio_x, uint8_t pinx, uint8_t mode, uint8_t outtyper, uint8_t speed, uint8_t pull)
{
    GPIO_EnableClock(gpio_x);
    GPIO_Set_Mode(gpio_x, pinx, mode);
    GPIO_Set_OutputType(gpio_x, pinx, outtyper);
    GPIO_Set_OutputSpeed(gpio_x, pinx, speed);
    GPIO_SetInputType(gpio_x, pinx, pull);
}


void GPIO_ConfigAlternateFunc(GPIOn * gpio_x, uint8_t pinx, uint8_t AF_x)
{
    GPIOn_SET_MODER(gpio_x, pinx, MODER_ALTEMATE);
    if(pinx < GPIO_PIN8)
        GPIOn_SET_AFRL(gpio_x, pinx, AF_x);
    else
        GPIOn_SET_AFRH(gpio_x, pinx, AF_x);
}


