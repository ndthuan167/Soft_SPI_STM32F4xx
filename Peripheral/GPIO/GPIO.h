/**
 * @file GPIO.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for GPIO(General Purpose Input/Output) of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-10
 * 
 */

/******************************************************************************
* Include Files
******************************************************************************/

#include <stdio.h>
#include <stdint.h>

/*******************************************************************************
* GPIO ADDRESS DEFINITION
*******************************************************************************/
#define ADDRESS_GPIO_A      0x40020000
#define ADDRESS_GPIO_B      0x40020400
#define ADDRESS_GPIO_C      0x40020800
#define ADDRESS_GPIO_D      0x40020C00 
#define ADDRESS_GPIO_E      0x40021000
#define ADDRESS_GPIO_F      0x40021400
#define ADDRESS_GPIO_G      0x40021800
#define ADDRESS_GPIO_H      0x40021C00
#define ADDRESS_GPIO_I      0x40022000
#define ADDRESS_GPIO_J      0x40022400
#define ADDRESS_GPIO_K      0x40022800

/*******************************************************************************
* GPIO REGISTER STRUCTURE DEFINITION
*******************************************************************************/

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
}GPIOn;

/*******************************************************************************
* ENUM DEFINITION
*******************************************************************************/
// enum of number of pin in GPIOn Port
enum
{
    GPIO_PIN0,
    GPIO_PIN1,
    GPIO_PIN2,
    GPIO_PIN3,
    GPIO_PIN4,
    GPIO_PIN5,
    GPIO_PIN6,
    GPIO_PIN7,
    GPIO_PIN8,
    GPIO_PIN9,
    GPIO_PIN10,
    GPIO_PIN11,
    GPIO_PIN12,
    GPIO_PIN13,
    GPIO_PIN14,
    GPIO_PIN15,
};

enum
{
    AF0,
    AF1,
    AF2,
    AF3,
    AF4,
    AF5,
    AF6,
    AF7,
    AF8,
    AF9,
    AF10,
    AF11,
    AF12,
    AF13,
    AF14,
    AF15,
};

/*******************************************************************************
* COMMON DEFINITION
*******************************************************************************/

#define     SET         1
#define     CLEAR    	0

/*******************************************************************************
* MODER REGISTERS DEFINITION
*******************************************************************************/
#define MODER_INPUT         0x00     //00
#define MODER_OUTPUT        0x01     //01
#define MODER_ALTEMATE      0x02     //10
#define MODER_ANALOG        0x03     //11

#define GPIOn_GET_MODER(GPIOn)                      GPIOn->MODER
#define GPIOn_SET_MODER(GPIOn, Pinx, value)         GPIOn->MODER |= (value << 2*Pinx)

/*******************************************************************************
* OTYPER REGISTERS DEFINITION
*******************************************************************************/
#define OTYPER_PUSHPULL         0x00     //0
#define OTYPER_OPENDRAIN        0x01     //1

#define GPIOn_GET_OTYPER(GPIOn)                     GPIOn->OTYPER
#define GPIOn_SET_OTYPER(GPIOn, Pinx, value)        GPIOn->OTYPER |= (value << Pinx)

/*******************************************************************************
* PUPDR REGISTERS DEFINITION
*******************************************************************************/
#define PUPDR_NOTHING           0x00     //00
#define PUPDR_PULLUP            0x01     //01
#define PUPDR_PULLDOWN          0x02     //10
#define PUPDR_RESERVE           0x03     //11

#define GPIOn_GET_PUPDR(GPIOn)                      GPIOn->PUPDR
#define GPIOn_SET_PUPDR(GPIOn, Pinx, value)         GPIOn->PUPDR |= (value << 2*Pinx)

/*******************************************************************************
* OSPEEDR REGISTERS DEFINITION
*******************************************************************************/
#define OSPEEDR_LOW             0x00     //00
#define OSPEEDR_MEDIUM          0x01     //01
#define OSPEEDR_HIGH            0x02     //10
#define OSPEEDR_VERYHIGH        0x03     //11

#define GPIOn_GET_OSPEEDR(GPIOn)                    GPIOn->OSPEEDR
#define GPIOn_SET_OSPEEDR(GPIOn, Pinx, value)       GPIOn->OSPEEDR |= (value << 2*Pinx)

/*******************************************************************************
* IDR REGISTERS DEFINITION
*******************************************************************************/
#define GPIOn_GET_IDR(GPIOn)                        GPIOn->IDR

/*******************************************************************************
* ODR REGISTERS DEFINITION
*******************************************************************************/
#define GPIOn_GET_ODR(GPIOn)                        GPIOn->ODR
#define GPIOn_SET_ODR(GPIOn, Pinx, value)           GPIOn->ODR |= (value << Pinx)
#define GPIOn_CLEAR_ODR(GPIOn, Pinx, value)         GPIOn->ODR &= ~(value << Pinx)

/*******************************************************************************
* BSRR REGISTERS DEFINITION
*******************************************************************************/
#define GPIOn_SET_BSRR(GPIOn, value)                GPIOn->BSRR = (value)

/*******************************************************************************
* AFRL/ AFRH REGISTERS DEFINITION
*******************************************************************************/
#define GPIOn_SET_AFRL(GPIOn, pinx, AFx)                GPIOn->AFRL |= (AFx << (pinx * 4))
#define GPIOn_SET_AFRH(GPIOn, pinx, AFx)                GPIOn->AFRH |= (AFx << ((pinx - 8) * 4))


/*******************************************************************************
* FUNCTIONS DEFINITION
*******************************************************************************/

void GPIO_Set_Mode(GPIOn *gpio_x, uint8_t pinx, uint8_t mode);
uint8_t GPIO_GetMode(GPIOn *gpio_x, uint8_t pinx);

void GPIO_Set_OutputType(GPIOn *gpio_x, uint8_t pinx, uint8_t type);
uint8_t GPIO_GetOutputType(GPIOn *gpio_x, uint8_t pinx);

void GPIO_Set_OutputSpeed(GPIOn *gpio_x, uint8_t pinx, uint8_t speed);
uint8_t GPIO_GetOutputSpeed(GPIOn *gpio_x, uint8_t pinx);

void GPIO_SetInputType(GPIOn *gpio_x, uint8_t pinx, uint8_t type);
uint8_t GPIO_GetInputType(GPIOn *gpio_x, uint8_t pinx);

uint8_t GPIO_GetInputData(GPIOn *gpio_x, uint8_t pinx);

void GPIO_SetOutputData(GPIOn *gpio_x, uint8_t pinx, uint8_t value); 
uint8_t GPIO_GetDataInOutputRes(GPIOn *gpio_x, uint8_t pinx); 

void GPIO_SettingOutputDataBSRR(GPIOn *gpio_x, uint8_t pinx, uint8_t value);

void GPIO_EnableClock(GPIOn * gpio_x);
void GPIO_Configuration(GPIOn * gpio_x, uint8_t pinx, uint8_t mode, uint8_t outtyper, uint8_t speed, uint8_t pull);

void GPIO_ConfigAlternateFunc(GPIOn * gpio_x, uint8_t pinx, uint8_t AF_x);
