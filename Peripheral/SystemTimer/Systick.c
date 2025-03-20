/**
 * @file Systick.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for system timer of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-10
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "Systick.h"
#include "../RCC/RCC.h"

#define SYSTICK_CLOCK            16000000
#if defined(USE_SYSCLK_16MHz)
#define SYSTICK_CLOCK            16000000
#elif defined(USE_SYSCLK_24MHz)
#define SYSTICK_CLOCK            24000000
#elif defined(USE_SYSCLK_36MHz)
#define SYSTICK_CLOCK            36000000
#elif defined(USE_SYSCLK_48MHz)
#define SYSTICK_CLOCK            48000000
#elif defined(USE_SYSCLK_56MHz)
#define SYSTICK_CLOCK            56000000
#elif defined(USE_SYSCLK_72MHz)
#define SYSTICK_CLOCK            72000000
#endif

// Systick
SysTick *systick = (SysTick*) ADDRESS_SYSTICK;

/******************************************************************************
 * Vareiables definition
 ******************************************************************************/
volatile uint8_t systick_timer_loop_1ms_IT = 0;

uint8_t systick_timer_loop10ms = 0;
uint8_t systick_timer_loop50ms = 0;
uint8_t systick_timer_loop100ms = 0;
uint8_t systick_timer_loop500ms = 0;
uint8_t systick_timer_loop1s = 0;

typedef struct
{
    uint8_t     systick_timer_1ms       :1;
    uint8_t     systick_timer_10ms      :1;
    uint8_t     systick_timer_50ms      :1;
    uint8_t     systick_timer_100ms     :1;
    uint8_t     systick_timer_500ms     :1;
    uint8_t     systick_timer_1s        :1;
}systick_timer_flag;

systick_timer_flag systick_timer_fl;

/**
*******************************************************************************
* @ Name : SysTickGetCountFlag
* @ Parameters: void
* @ Registers : SYST_CSR
* @ Descriptions :
*		- Get count flag of systick in bit-16 of CSR registers
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2024-07-09
*******************************************************************************
*/
uint8_t SysTickGetCountFlag(void)
{
    uint8_t u8CountFlag;
    uint32_t u32CSRValue = SYST_CSR_GET_COUNTFLAG(systick);
    u8CountFlag = (u32CSRValue & (1 << CSR_COUNTFLAG)) >> CSR_COUNTFLAG;

    return u8CountFlag;
}

/**
*******************************************************************************
* @ Name : SysTickSettingEnableCounter
* @ Parameters: uint8_t value
* @ Registers : SYST_CSR
* @ Descriptions :
*		- Set counter flag of systick in bit-0 of CSR registers
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-09
*******************************************************************************
*/
void SysTickSettingEnableCounter(uint8_t value)
{
    if (value == ENABLE)
        SYST_CSR_SET_ENABLE(systick);
    else
        SYST_CSR_CLEAR_ENABLE(systick);
}

/**
*******************************************************************************
* @ Name : SysTickSettingReloadValue
* @ Parameters: uint8_t systick_interval
* @ Registers : SYST_RVR
* @ Descriptions :
*		- Calculate the reload value base on the systick interval input
*		- Reload value = systick_interval * SYSTICK_CLOCK - 1
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-09
*******************************************************************************
*/
void SysTickSettingReloadValue(float systick_interval)
{
    uint32_t reloadvalue;
    reloadvalue = systick_interval * SYSTICK_CLOCK - 1;
    SYST_RVR_SET_RELOAD_VALUE(systick, reloadvalue);
}

/**
*******************************************************************************
* @ Name : SysTickGetCurrentCounterValue
* @ Parameters: void
* @ Registers : SYST_CVR
* @ Descriptions :
*		- Get current value of counter
* @ Return value : uint32_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-09
*******************************************************************************
*/
uint32_t SysTickGetCurrentCounterValue(void)
{
    return SYST_CVR_GET_CURRENT_COUNTER(systick);
}

/**
*******************************************************************************
* @ Name : InitSystemTimer
* @ Parameters: void
* @ Registers : SYST_CVR
* @ Descriptions :
*		- Init System timer by clear the couter value
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-09
*******************************************************************************
*/
void InitSystemTimer(void)
{
    SYST_CVR_CLEAR_CURRENT_COUNTER(systick);
    SYST_RVR_SET_RELOAD_VALUE(systick, 0x00FFFFFF);
}

/**
*******************************************************************************
* @ Name : SettingSystemTimer
* @ Parameters: uint8_t clksrc, uint8_t tick, uint8_t systick_interval
* @ Registers : SYST_CSR, SYST_RVR
* @ Descriptions :
*		- Setting System timer by setting:
*           + Init Systick: clear counter, set
*           + Clock source : EXTERNAL_CLKSRC (0), PROCESSOR_CLKSRC(1)
*           + Interrupt request exception : ENABLE(1) O : counter = 0 -> SysTick_Handler
                                            DISABLE(0) X
*           + Reload value: SysTickSettingReloadValue()
*           + Enable counter
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-09
*******************************************************************************
*/
void SettingSystemTimer(uint8_t clksrc, uint8_t tick, float systick_interval)
{
    // Init System timer
    InitSystemTimer();

    // Select clock source
    if (clksrc == PROCESSOR_CLKSRC)
        SYST_CSR_SELECT_PROCESSOR_CLKSRC(systick);
    else
        SYST_CSR_SELECT_EXTERNAL_CLKSRC(systick);

    // Enable/Disable interrupt request exception
    if (tick == ENABLE)
        SYST_CSR_ENABLE_INTERRUPT(systick);
    else
        SYST_CSR_DISABLE_INTERRUPT(systick);

    // Setting Reload value
    SysTickSettingReloadValue(systick_interval);

    // Enable counter
    SysTickSettingEnableCounter(ENABLE);
}

/**
*******************************************************************************
* @ Name : LoopSystickTimerSetting
* @ Parameters:
* @ Registers :
* @ Descriptions :
*		- Loop to get the systick_timer flag
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-20
*******************************************************************************
*/
void LoopSystickTimerSetting(void)
{
    *((uint8_t *)&systick_timer_fl) = 0;

    if(systick_timer_loop_1ms_IT)
    {
        systick_timer_loop_1ms_IT = 0;
        systick_timer_fl.systick_timer_1ms = SET;
        systick_timer_loop10ms++;
        if (systick_timer_loop10ms >= 10)
        {
            systick_timer_fl.systick_timer_10ms = SET;
            systick_timer_loop10ms = 0;
            systick_timer_loop50ms++;

            if (systick_timer_loop50ms >= 5)
            {
                systick_timer_fl.systick_timer_50ms = SET;
                systick_timer_loop50ms = 0;
                systick_timer_loop100ms++;

                if (systick_timer_loop100ms >= 2)
                {
                    systick_timer_fl.systick_timer_100ms = SET;
                    systick_timer_loop100ms = 0;
                    systick_timer_loop500ms++;

                    if (systick_timer_loop500ms >= 5)
                    {
                        systick_timer_fl.systick_timer_500ms = SET;
                        systick_timer_loop500ms = 0;
                        systick_timer_loop1s++;

                        if (systick_timer_loop1s >= 2)
                        {
                            systick_timer_fl.systick_timer_1s = SET;
                            systick_timer_loop1s = 0;
                        }
                    }
                }
            }
        }
    }

}

/**
*******************************************************************************
* @ Name : GetFlagTimerSystickXms
* @ Parameters:
* @ Registers :
* @ Descriptions :
*		- Get flag timer (10ms, 50ms, 100ms, 500ms, 1s).
*       - If GetFlagTimerSystickXms() == TRUE, it means time of X is determined.
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167)
* @ date : 2024-07-09
*******************************************************************************
*/


uint8_t GetFlagTimerSystick1ms(void)
{
    return systick_timer_fl.systick_timer_1ms;
}

uint8_t GetFlagTimerSystick10ms(void)
{
    return systick_timer_fl.systick_timer_10ms;
}

uint8_t GetFlagTimerSystick50ms(void)
{
    return systick_timer_fl.systick_timer_50ms;
}

uint8_t GetFlagTimerSystick100ms(void)
{
    return systick_timer_fl.systick_timer_100ms;
}

uint8_t GetFlagTimerSystick500ms(void)
{
    return systick_timer_fl.systick_timer_500ms;
}

uint8_t GetFlagTimerSystick1s(void)
{
    return systick_timer_fl.systick_timer_1s;
}

