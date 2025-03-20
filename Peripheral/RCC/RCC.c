/**
 * @file RCC.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for External interrupt (EXTI) of STM32F407VGTx (ARMCortex M4)
 * @date 2025-02-28
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "RCC.h"


/******************************************************************************
 * Variables declaration 
 ******************************************************************************/

RCC * _RCC = (RCC *)ADDRESS_RCC;
PWR * _PWR = (PWR *)ADDRESS_PWR; 
FLASH * _FLASH = (FLASH *)ADDRESS_FLASH;

/**
*******************************************************************************
* @ Name : RCC_SetPLLIndexValue
* @ Parameters: uint8_t PLLM, uint8_t PLLN, uint8_t PLLP, uint8_t PLLQ
* @ Registers : PLLCFGR
* @ Descriptions :
*		- Set value for index of main PLL (PLLM, PLLN, PLLP, PLLQ)
*           + PLLM = / [2, 63]
*           + PLLN = X [50, 432]
*           + PLLP = / [2, 4, 6, 8]
*           + PLLQ = / [2, 4, 6, 8]
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-02-28
*******************************************************************************
*/
void RCC_SetPLLIndexValue(uint8_t PLLM, uint8_t PLLN, uint8_t PLLP, uint8_t PLLQ)
{
    RCC_SET_PLLM(_RCC, PLLM);
    RCC_SET_PLLN(_RCC, PLLN);
    RCC_SET_PLLP(_RCC, PLLP);
    RCC_SET_PLLQ(_RCC, PLLQ);
}


/**
*******************************************************************************
* @ Name : RCC_SettingSystemClockXXMHz
* @ Parameters: void
* @ Registers : CR, CFGR,...
* @ Descriptions :
*		- Configuration for clock
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-02-28
*******************************************************************************
*/
#if defined(USE_SYSCLK_16MHz)
/*
    - Only using HSI source (16Mhz)
    - AHB Prescaler: /1
*/
void RCC_SettingSystemClock16MHz(void)
{
    RCC_SET_HSI(_RCC, ON);
    // while(!(RCC_GET_HSIRDY(_RCC)));

    RCC_SET_AHBCLK_DIV(_RCC, AHBCLK_DIV_0);     // --> HCLK = 24MHz 

// APB1 Clock
#if defined(USE_APB1CLK_16MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB1CLK_8MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_2);
#endif

// APB2 Clock
#if defined(USE_APB2CLK_16MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB2CLK_8MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_2);
#endif
}
#elif defined (USE_SYSCLK_24MHz)
/*
    - Using PLL source from HSI (16MHz)
    - SystemClock = (HSI * PllM x PllN / PllP) = 16 / 8 x 72 / 6 -> 24Mhz
    - AHB Prescaler /1 -> HCLK = 24MHz
*/
void RCC_SettingSystemClock24MHz(void)
{
    RCC_SET_PLLON(_RCC);
    while(!(RCC_GET_PLLRDY(_RCC)));

    RCC_SELECT_PLLSRC(_RCC, PLL_HSI_SOURCE);
    RCC_SetPLLIndexValue(8, 72, 6, 4);
    RCC_SET_AHBCLK_DIV(_RCC, AHBCLK_DIV_0);     // --> HCLK = 24MHz 

// APB1 Clock
#if defined(USE_APB1CLK_24MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB1CLK_12MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB1CLK_6MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_4);
#elif defined(USE_APB1CLK_3MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_8);
#endif

// APB2 Clock
#if defined(USE_APB2CLK_24MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB2CLK_12MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB2CLK_6MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_4);
#elif defined(USE_APB2CLK_3MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_8);
#endif
}

#elif defined(USE_SYSCLK_36MHz)
/*
    - Using PLL source from HSI (16MHz)
    - SystemClock = (HSI * PllM x PllN / PllP) = 16 / 8 x 72 / 2 -> 72Mhz
    - AHB Prescaler /2 -> HCLK = 36MHz
*/
void RCC_SettingSystemClock36MHz(void)
{
    RCC_SET_PLLON(_RCC);
    while(!(RCC_GET_PLLRDY(_RCC)));

    RCC_SELECT_PLLSRC(_RCC, PLL_HSI_SOURCE);
    RCC_SetPLLIndexValue(8, 72, 2, 4);
    RCC_SET_AHBCLK_DIV(_RCC, AHBCLK_DIV_2);

//APB1 Clock
#if defined(USE_APB1CLK_36MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB1CLK_18MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB1CLK_9MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_4);
#endif

//APB2 Clock
#if defined(USE_APB2CLK_36MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB2CLK_18MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB2CLK_9MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_4);
#endif
}
#elif defined(USE_SYSCLK_48MHz)
/*
    - Using PLL source from HSI (16MHz)
    - SystemClock = (HSI * PllM x PllN / PllP) = 16 / 8 x 96 / 4 -> 48Mhz
    - AHB Prescaler /1 -> HCLK = 48MHz
*/
void RCC_SettingSystemClock48MHz(void)
{
    RCC_SET_PLLON(_RCC);
    while(!(RCC_GET_PLLRDY(_RCC)));

    RCC_SELECT_PLLSRC(_RCC, PLL_HSI_SOURCE);
    RCC_SetPLLIndexValue(8, 96, 2, 4);
    RCC_SET_AHBCLK_DIV(_RCC, AHBCLK_DIV_2);

//APB1 Clock
#if defined(USE_APB1CLK_24MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB1CLK_12MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_4);
#endif

//APB2 Clock
#if defined(USE_APB2CLK_48MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB2CLK_24MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB2CLK_12MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_4);
#endif
}
#elif defined(USE_SYSCLK_56MHz)
/*
    - Using PLL source from HSI (16MHz)
    - SystemClock = (HSI * PllM x PllN / PllP) = 16 / 8 x 56 / 2 -> 56Mhz
    - AHB Prescaler /1 -> HCLK = 56MHz
*/
void RCC_SettingSystemClock56MHz(void)
{
    RCC_SET_PLLON(_RCC);
    while(!(RCC_GET_PLLRDY(_RCC)));

    RCC_SELECT_PLLSRC(_RCC, PLL_HSI_SOURCE);
    RCC_SetPLLIndexValue(8, 56, 2, 4);
    RCC_SET_AHBCLK_DIV(_RCC, AHBCLK_DIV_0);

//APB1 Clock
#if defined(USE_APB1CLK_28MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB1CLK_14MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_4);
#endif

//APB2 Clock
#if defined(USE_APB2CLK_56MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB2CLK_28MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB2CLK_14MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_4);
#endif
}

#elif defined(USE_SYSCLK_72MHz)
/*
    - Using PLL source from HSI (16MHz)
    - SystemClock = (HSI * PllM x PllN / PllP) = 16 / 8 x 72 / 2 -> 72Mhz
    - AHB Prescaler /1 -> HCLK = 72MHz
*/
void RCC_SettingSystemClock72MHz(void)
{
    RCC_SET_PLLON(_RCC);
    while(!(RCC_GET_PLLRDY(_RCC)));

    RCC_SELECT_PLLSRC(_RCC, PLL_HSI_SOURCE);
    RCC_SetPLLIndexValue(8, 72, 2, 4);
    RCC_SET_AHBCLK_DIV(_RCC, AHBCLK_DIV_0);

// APB1 Clock
#if defined(USE_APB1CLK_36MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB1CLK_18MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_4);
#elif defined(USE_APB1CLK_9MHz)
    RCC_SET_APB1CLK_DIV(_RCC, APBCLK_DIV_8);
#endif

// APB2 Clock
#if defined(USE_APB2CLK_72MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_0);
#elif defined(USE_APB2CLK_36MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_2);
#elif defined(USE_APB2CLK_18MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_4);
#elif defined(USE_APB2CLK_9MHz)
    RCC_SET_APB2CLK_DIV(_RCC, APBCLK_DIV_8);
#endif

}
#endif

/**
*******************************************************************************
* @ Name : RCC_EnableAnotherSourceForHSE
* @ Parameters: uint8_t Source
* @ Registers : CR
* @ Descriptions :
*		- Select type of sourse when HSE source not work or not stable
*       + CSS: Move to another HSE source
*       + HSEBYP: Move to another source
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-02-28
*******************************************************************************
*/
void RCC_EnableAnotherSourceForHSE(uint8_t Source)
{
    if(RCC_GET_HSE(_RCC) == ON)
    {
        if(Source == CSS_ENABLE)
            RCC_SET_CSSON(_RCC, ON);
        else if(Source == HSE_BYPASS_ENABLE)
            RCC_SET_HSEBYP(_RCC, ON);
    }
}

/**
*******************************************************************************
* @ Name : RCC_SettingMCUOutputClock
* @ Parameters: uint8_t MCOx, uint8_t MCUx_Source, uint8_t MCUx_Prescaler
* @ Registers : CFGR
* @ Descriptions :
*		- Setting MCU output clock
*       + MCOx : MCO1_ENABLE, MCO2_ENABLE
*       + MCUx_Source:
*           . MCO1: OUT1_HSI_CLK, OUT1_LSE_CLK, OUT1_HSE_CLK, OUT1_PLL_CLK
*           . MCO2: SYSCLK_OUT2_CLK, PLLI2S_OUT2_CLK, HSE_OUT2_CLK, PLL_OUT2_CLK
*       + MCUx_Prescaler: / [1 : 5]
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-02-28
*******************************************************************************
*/
void RCC_SettingMCUOutputClock(uint8_t MCOx, uint8_t MCUx_Source, uint8_t MCUx_Prescaler)
{
    if(MCOx == MCO1_ENABLE)
    {
        RCC_SET_MCUCLK_OUT1(_RCC, MCUx_Source);
        RCC_MCUCLK_OUT1_PSC(_RCC, MCUx_Prescaler);
    }

    if(MCOx == MCO2_ENABLE)
    {
        RCC_SET_MCUCLK_OUT2(_RCC, MCUx_Source);
        RCC_MCUCLK_OUT2_PSC(_RCC, MCUx_Prescaler);
    }
}

/**
*******************************************************************************
* @ Name : RCC_EnablePeripheralClock
* @ Parameters: uint8_t Peripheral
* @ Registers : AHB1ENR, APB1ENR, APB2ENR
* @ Descriptions : Enable Clock for peripheral
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-02-28
*******************************************************************************
*/
void RCC_EnablePeripheralClock(uint8_t Peripheral)
{
    uint32_t value = 0;
    if (Peripheral < CLOCK_TIM2)
    {
        value = RCC_GET_AHB1ENR(_RCC);
        value |= (1 << Peripheral);

        RCC_SET_AHB1ENR(_RCC,value);
    }
    else if (Peripheral < CLOCK_TIM1)
    {
        value = RCC_GET_APB1ENR(_RCC);
        value |= (1 << (Peripheral - CLOCK_TIM2));

        RCC_SET_APB1ENR(_RCC, value);
    }
    else
    {
        value = RCC_GET_APB2ENR(_RCC);
        value |= (1 << (Peripheral - CLOCK_TIM1));

        RCC_SET_APB2ENR(_RCC, value);
    }
}
