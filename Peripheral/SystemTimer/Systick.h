/**
 * @file Systick.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for system timer of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-10
 * 
 */


/******************************************************************************
* Include Files
******************************************************************************/

#include "stdio.h"
#include "stdint.h"

extern volatile uint8_t systick_timer_loop_1ms_IT;

/*******************************************************************************
* SYSTEM TIMER ADDRESS DEFINITION
*******************************************************************************/
#define ADDRESS_SYSTICK 0xE000E010

/*******************************************************************************
* SYSTICK REGISTER STRUCTURE DEFINITION
*******************************************************************************/
typedef struct
{
    volatile uint32_t SYST_CSR;
    volatile uint32_t SYST_RVR;
    volatile uint32_t SYST_CVR;
    volatile uint32_t SYST_CALIB;
} SysTick;

/*******************************************************************************
* ENUM DEFINITION
*******************************************************************************/
// enum of bit order in CSR register
enum
{
    CSR_ENABLE,
    CSR_TICKINT,
    CSR_CLKSOURCE,
    CSR_COUNTFLAG = 16,
};

/*******************************************************************************
* COMMON DEFINITION
*******************************************************************************/

#define     SET                1
#define     CLEAR              0

/*******************************************************************************
* SYST_CSR REGISTERS DEFINITION
*******************************************************************************/

#define     ENABLE             1
#define     DISABLE            0

#define SYST_CSR_SET_ENABLE(SysTick)                SysTick->SYST_CSR |= (1 << CSR_ENABLE)
#define SYST_CSR_CLEAR_ENABLE(SysTick)              SysTick->SYST_CSR &= ~(1 << CSR_ENABLE)

#define SYST_CSR_ENABLE_INTERRUPT(SysTick)          SysTick->SYST_CSR |= (1 << CSR_TICKINT)
#define SYST_CSR_DISABLE_INTERRUPT(SysTick)         SysTick->SYST_CSR &= ~(1 << CSR_TICKINT)

#define SYST_CSR_SELECT_PROCESSOR_CLKSRC(SysTick)   SysTick->SYST_CSR |= (1 << CSR_CLKSOURCE)
#define SYST_CSR_SELECT_EXTERNAL_CLKSRC(SysTick)    SysTick->SYST_CSR &= ~(1 << CSR_CLKSOURCE)

#define PROCESSOR_CLKSRC        1
#define EXTERNAL_CLKSRC         0

#define SYST_CSR_GET_COUNTFLAG(SysTick)             SysTick->SYST_CSR

/*******************************************************************************
* SYST_RVR REGISTERS DEFINITION
*******************************************************************************/

#define SYSTICK_TIMER_1MS      0.001
#define SYSTICK_TIMER_10MS     0.01
#define SYSTICK_TIMER_50MS     0.05
#define SYSTICK_TIMER_100MS    0.1
#define SYSTICK_TIMER_500MS    0.5
#define SYSTICK_TIMER_1S       1

#define SYST_RVR_SET_RELOAD_VALUE(SysTick, value)   SysTick->SYST_RVR = value

/*******************************************************************************
* SYST_CVR REGISTERS DEFINITION
*******************************************************************************/

#define SYST_CVR_CLEAR_CURRENT_COUNTER(SysTick)     SysTick->SYST_CVR = 0x0000
#define SYST_CVR_GET_CURRENT_COUNTER(SysTick)       SysTick->SYST_CVR


/*******************************************************************************
* FUNCTIONS DEFINITION
*******************************************************************************/

uint8_t SysTickGetCountFlag(void);
void SysTickSettingEnableCounter(uint8_t value);

void SysTickSettingReloadValue(float systick_interval);
uint32_t SysTickGetCurrentCounterValue(void);

void SettingSystemTimer(uint8_t clksrc, uint8_t tick, float systick_interval);
void InitSystemTimer(void);

void LoopSystickTimerSetting(void);

uint8_t GetFlagTimerSystick1ms(void);
uint8_t GetFlagTimerSystick10ms(void);
uint8_t GetFlagTimerSystick50ms(void);
uint8_t GetFlagTimerSystick100ms(void);
uint8_t GetFlagTimerSystick500ms(void);
uint8_t GetFlagTimerSystick1s(void);

