/**
 * @file EXTI.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for External interrupt (EXTI) of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-13
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "EXTI.h"
#include "../RCC/RCC.h"
#include "../GPIO/GPIO.h"
#include "../SystemTimer/Systick.h"

/******************************************************************************
 * Variables declaration 
 ******************************************************************************/

EXTIx *_EXTI = (EXTIx*) ADDRESS_EXTI;
SYSCFG *_SYSCFG = (SYSCFG*) ADDRESS_SYSCFG;

GPIOn *gpio_A = (GPIOn *)ADDRESS_GPIO_A;
GPIOn *gpio_B = (GPIOn *)ADDRESS_GPIO_B;
GPIOn *gpio_D = (GPIOn *)ADDRESS_GPIO_D;

volatile uint32_t index = 0;
volatile uint8_t cs_trigger = 0;

/**
*******************************************************************************
* @ Name : SYSCFG_ConfigPortEXTI
* @ Parameters: uint8_t port, uint8_t EXTI_line
* @ Registers : SYSCFG_EXTICRx
* @ Descriptions :
*		- Set port EXTIx coressponding with PINx in System config register.
*           + 4 EXTI_line/ EXTICR register (16-low bit) : CRx = EXTI_line/4
*           + 4-bit / EXTI_line: (EXTI_line % 4) * 4
*           + port: EXTIx_PA_PIN -> EXTIx_PK_PIN
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
void SYSCFG_ConfigPortEXTI(uint8_t port, uint8_t EXTI_line)
{
	RCC_EnablePeripheralClock(CLOCK_SYSCFG);
    SYSCFG_SET_PORT_EXTI(_SYSCFG, EXTI_line / 4, port, EXTI_line);
}

/**
*******************************************************************************
* @ Name : SYSCFG_GetStatusPortEXTI
* @ Parameters: uint8_t EXTI_line
* @ Registers : SYSCFG_EXTICRx
* @ Descriptions :
*		- Get current port EXTIx coressponding with PINx in System config register.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
uint8_t SYSCFG_GetStatusPortEXTI(uint8_t EXTI_line)
{
    return (SYSCFG_GET_PORT_EXTI(_SYSCFG, EXTI_line / 4) >> (EXTI_line % 4) * 4) & 0x0F;
}

/**
*******************************************************************************
* @ Name : EXTI_SettingMaskInterrupt
* @ Parameters: uint8_t EXTI_line, uint8_t mask
* @ Registers : EXTI_IMR
* @ Descriptions :
*		- Setting interrupt mask for EXTI_line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
*           + mask: NOT_MASKED (1), MASKED(0)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
void EXTI_SettingMaskInterrupt(uint8_t EXTI_line, uint8_t mask)
{
    EXTI_SET_IMR(_EXTI, EXTI_line, mask);
}

/**
*******************************************************************************
* @ Name : EXTI_GetValueMaskInterrupt
* @ Parameters: uint8_t EXTI_line
* @ Registers : EXTI_IMR
* @ Descriptions :
*		- Get value of mask interrupt in EXTI_line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
uint8_t EXTI_GetValueMaskInterrupt(uint8_t EXTI_line)
{
    return (EXTI_GET_IMR(_EXTI, EXTI_line) & (1 << EXTI_line)) >> (EXTI_line);
}

/**
*******************************************************************************
* @ Name : EXTI_SettingMaskEventRequest
* @ Parameters: uint8_t EXTI_line, uint8_t mask
* @ Registers : EXTI_EMR
* @ Descriptions :
*		- Setting event request mask for EXTI_line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
*           + mask: NOT_MASKED (1), MASKED(0)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
void EXTI_SettingMaskEventRequest(uint8_t EXTI_line, uint8_t mask)
{
    EXTI_SET_EMR(_EXTI, EXTI_line, mask);
}

/**
*******************************************************************************
* @ Name : EXTI_GetValueMaskEventRequest
* @ Parameters: uint8_t EXTI_line
* @ Registers : EXTI_IMR
* @ Descriptions :
*		- Get value of mask event request in EXTI_line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
uint8_t EXTI_GetValueMaskEventRequest(uint8_t EXTI_line)
{
    return (EXTI_GET_EMR(_EXTI, EXTI_line) & (1 << EXTI_line)) >> (EXTI_line);
}

/**
*******************************************************************************
* @ Name : EXTI_Configuration
* @ Parameters: uint8_t EXTI_port,uint8_t EXTI_line, uint8_t mask_it, uint8_t mask_event, uint8_t trigger_type
* @ Registers : EXTI_RTSR, EXTI_FTSR, EXTI_IMR, EXTI_EMR, EXTI_PR
* @ Descriptions :
*		- Configure EXTI for EXTI_line
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
*           + mask_it, mask_event: NOT_MASKED (1), MASKED(0)
*           + trigger_type: RISING_TRIGGER(0), FALLING_TRIGGER(1)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
void EXTI_Configuration(uint8_t EXTI_port,uint8_t EXTI_line, uint8_t mask_it, uint8_t mask_event, uint8_t trigger_type)
{
    SYSCFG_ConfigPortEXTI(EXTI_port, EXTI_line);     // enable EXTI interrupt line

    if (trigger_type == RISING_TRIGGER)
    {
        EXTI_SET_RTSR(_EXTI, EXTI_line, ENABLE);
        EXTI_SET_FTRS(_EXTI, EXTI_line, DISABLE);
    }
    else if (trigger_type == FALLING_TRIGGER)
    {
        EXTI_SET_RTSR(_EXTI, EXTI_line, DISABLE);
        EXTI_SET_FTRS(_EXTI, EXTI_line, ENABLE);
    }

    EXTI_SettingMaskInterrupt(EXTI_line, mask_it);       // enable interrupt request in line
    EXTI_SettingMaskEventRequest(EXTI_line, mask_event);    
}

/**
*******************************************************************************
* @ Name : EXTI_SetInterruptOccurebySW
* @ Parameters: uint8_t EXTI_line
* @ Registers : EXTI_SWIER
* @ Descriptions :
*		- Set interrupt occure by SW for EXTI_line
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
void EXTI_SetInterruptOccurebySW(uint8_t EXTI_line)
{
    EXTI_SET_SWIER(_EXTI, EXTI_line);
}

/**
*******************************************************************************
* @ Name : EXTI_ClearFlagInterruptInLine
* @ Parameters: uint8_t EXTI_line
* @ Registers : EXTI_PR
* @ Descriptions :
*		- Clear interrupt flag in EXTIx (Pinx) line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
*           + Need to clear this flag to interrupt occure in next time.
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
void EXTI_ClearFlagInterruptInLine(uint8_t EXTI_line)
{
    EXTI_SET_PR(_EXTI, EXTI_line);
}

/**
*******************************************************************************
* @ Name : EXTI_GetFlagInterruptStatus
* @ Parameters: uint8_t EXTI_line
* @ Registers : EXTI_PR
* @ Descriptions :
*		- Get status of interrupt flag in EXTIx (Pinx) line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
uint8_t EXTI_GetFlagInterruptStatus(uint8_t EXTI_line)
{
    return (EXTI_GET_PR(_EXTI, EXTI_line) & (1 << EXTI_line)) >> (EXTI_line);
}

/**
*******************************************************************************
* @ Name : EXTI_GetTriggerStatus
* @ Parameters: uint8_t EXTI_line, uint8_t trigger_type
* @ Registers : EXTI_RTSR, EXTI_FTSR
* @ Descriptions :
*		- Get status of trigger type in EXTIx (Pinx) line.
*           + EXTI_line: EXTI_LINE_0 -> EXTI_LINE_15
*           + trigger_type: RISING_TRIGGER(0), FALLING_TRIGGER(1)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-13
*******************************************************************************
*/
uint8_t EXTI_GetTriggerStatus(uint8_t EXTI_line, uint8_t trigger_type)
{
    if (trigger_type == RISING_TRIGGER)
        return (EXTI_GET_RTSR(_EXTI, EXTI_line) & (1 << EXTI_line)) >> EXTI_line;
    else
        return (EXTI_GET_FTSR(_EXTI, EXTI_line) & (1 << EXTI_line)) >> EXTI_line;
}

/******************************************************************************
 * Interrupt Function 
 ******************************************************************************/

INTERRUPT HandleLED1(void)
{
    GPIO_SettingOutputDataBSRR(gpio_A, GPIO_PIN4, CLEAR);
    GPIO_SettingOutputDataBSRR(gpio_B, GPIO_PIN12, CLEAR);
}

INTERRUPT HandleLED2(void)
{
	GPIO_SettingOutputDataBSRR(gpio_D, GPIO_PIN0, SET);
	for (index = 0; index < 1000000; index++)
		;
	GPIO_SettingOutputDataBSRR(gpio_D, GPIO_PIN0, CLEAR);
	EXTI_ClearFlagInterruptInLine(EXTI_LINE_2);
}


