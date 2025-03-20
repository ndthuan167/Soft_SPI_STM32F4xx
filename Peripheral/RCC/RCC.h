/**
 * @file RCC.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for Clock configuration (RCC) of STM32F407VGTx (ARMCortex M4)
 * @date 2025-02-27
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include <stdio.h>
#include <stdint.h>

/*******************************************************************************
 * DEFINITION
 *******************************************************************************/

#define USE_SYSCLK_16MHz
// #define USE_SYSCLK_24MHz
// #define USE_SYSCLK_36MHz
// #define USE_SYSCLK_48MHz
// #define USE_SYSCLK_56MHz
// #define USE_SYSCLK_72MHz

#if defined(USE_SYSCLK_16MHz)
#define USE_APB1CLK_16MHz
// #define USE_APB1CLK_8MHz

#define USE_APB2CLK_16MHz
// #define USE_APB2CLK_8MHz

#elif defined(USE_SYSCLK_24MHz)
// #define USE_APB1CLK_24MHz
// #define USE_APB1CLK_12MHz
// #define USE_APB1CLK_6MHz
// #define USE_APB1CLK_3MHz

// #define USE_APB2CLK_24MHz
// #define USE_APB2CLK_12MHz
// #define USE_APB2CLK_6MHz
// #define USE_APB2CLK_3MHz

#elif defined(USE_SYSCLK_36MHz)
#define USE_APB1CLK_36MHz
// #define USE_APB1CLK_18MHz
// #define USE_APB1CLK_9MHz

#define USE_APB2CLK_36MHz
// #define USE_APB2CLK_18MHz
// #define USE_APB2CLK_9MHz

#elif defined(USE_SYSCLK_48MHz)
// #define USE_APB1CLK_24MHz
// #define USE_APB1CLK_12MHz

// #define USE_APB2CLK_48MHz
// #define USE_APB2CLK_24MHz
// #define USE_APB2CLK_12MHz

#elif defined(USE_SYSCLK_56MHz)
// #define USE_APB1CLK_28MHz
// #define USE_APB1CLK_14MHz

// #define USE_APB2CLK_56MHz
// #define USE_APB2CLK_28MHz
// #define USE_APB2CLK_14MHz
#elif defined(USE_SYSCLK_72MHz)
// #define USE_APB1CLK_36MHz
// #define USE_APB1CLK_18MHz
// #define USE_APB1CLK_9MHz

// #define USE_APB2CLK_72MHz
// #define USE_APB2CLK_36MHz
// #define USE_APB2CLK_18MHz
// #define USE_APB2CLK_9MHz
#endif



#define HSI		16000000U
#define LSI		32000000U
#define LSE		32768000U

/*******************************************************************************
 * RCC ADDRESS DEFINITION
 *******************************************************************************/
#define ADDRESS_RCC             0x40023800
#define ADDRESS_PWR				0x40007000
#define ADDRESS_FLASH			0x40023C00

/*******************************************************************************
 * RCC REGISTER STRUCTURE DEFINITION
 *******************************************************************************/

typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t AHB3RSTR;
    uint32_t reserve0;
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
    uint32_t reserve1;
    uint32_t reserve2;
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t AHB3ENR;
    uint32_t reserve3;
	uint32_t APB1ENR;
	uint32_t APB2ENR;
    uint32_t reserve4;
    uint32_t reserve5;
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t AHB3LPENR;
    uint32_t reserve6;
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
    uint32_t reserve7;
    uint32_t reserve8;
	uint32_t BDCR;
	uint32_t CSR;
    uint32_t reserve9;
    uint32_t reserve10;
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
} RCC;


typedef struct
{
	uint32_t CR;
	uint32_t CSR;
} PWR;


typedef struct
{
	uint32_t ACR; // Access control register
	uint32_t KEYR; // Flash key register
	uint32_t OPTKEYR; // Option byte key register
	uint32_t SR; // Status register
	uint32_t CCR; // Control register
	uint32_t OPTR; // Option byte register
} FLASH;


/*******************************************************************************
 * ENUM DEFINITION
 *******************************************************************************/

enum
{
    CLOCK_GPIO_A,
    CLOCK_GPIO_B,
    CLOCK_GPIO_C,
    CLOCK_GPIO_D,
    CLOCK_GPIO_E,
    CLOCK_GPIO_F,
    CLOCK_GPIO_G,
    CLOCK_GPIO_H,
    CLOCK_GPIO_I,
	CLOCK_GPIO_J,
	CLOCK_GPIO_K,
	CLOCK_GPIO_CRC,
	CLOCK_TIM2,
	CLOCK_TIM3,
	CLOCK_TIM4,
	CLOCK_TIM5,
	CLOCK_TIM6,
	CLOCK_TIM7,
	CLOCK_TIM12,
	CLOCK_TIM13,
	CLOCK_TIM14,
	CLOCK_WWDG 		= CLOCK_TIM2 + 11,
	CLOCK_SPI2 		= CLOCK_TIM2 + 14,
	CLOCK_SPI3,
	CLOCK_USART2 	= CLOCK_TIM2 + 17,
	CLOCK_USART3,
	CLOCK_UART4,
	CLOCK_UART5,
	CLOCK_I2C1 		= CLOCK_TIM2 + 21,
	CLOCK_I2C2,
	CLOCK_I2C3,
	CLOCK_CAN1		= CLOCK_TIM2 + 25,
	CLOCK_CAN2,
	CLOCK_PWR 		= CLOCK_TIM2 + 28,
	CLOCK_DAC 		= CLOCK_TIM2 + 29,
	CLOCK_UART7,
	CLOCK_UART8,
	CLOCK_TIM1,
	CLOCK_TIM8,
	CLOCK_USART1 	= CLOCK_TIM1 + 4,
	CLOCK_USART6,
	CLOCK_ADC1 		= CLOCK_TIM1 + 8,
	CLOCK_ADC2,
	CLOCK_ADC3,
	CLOCK_SDIO 		= CLOCK_TIM1 + 11,
	CLOCK_SPI1,
	CLOCK_SPI4 		= CLOCK_TIM1 + 13,
	CLOCK_SYSCFG,
	CLOCK_TIM9 		= CLOCK_TIM1 + 16,
	CLOCK_TIM10,
	CLOCK_TIM11,
	CLOCK_SPI5 		= CLOCK_TIM1 + 20,
	CLOCK_SPI6,
	CLOCK_SAI1,
};

#define 	ON		1
#define 	OFF		0

// When config HSE
#define PWR_SET_VOS_1MODE(_PWR, VOS)	_PWR->CR |= (1 << 14)

// Config Flash to support high speed clock (72MHz) -> Enable Prefetch Buffer, Instruction Cache, Data Cache and Latency 1 wait state.
#define FLASH_CONFIG_FOR_CLOCK(_FLASH)	_FLASH->ACR |= (1 << 8) | (1 << 9) | (1 << 10) | (1 << 0)


/*******************************************************************************
 * CR (Clock Control) REGISTERS DEFINITION
 *******************************************************************************/
// HSI Config
#define RCC_SET_HSI(_RCC, OnOff) 		\
		do { 							\
			if (OnOff == ON) { 			\
				_RCC->CR |= (1 << 0); 	\
			} else if (OnOff == OFF) { 	\
				_RCC->CR &= ~(1 << 0); 	\
			} 							\
		} while (0)

#define RCC_GET_HSI(_RCC) 		((_RCC->CR & (1 << 0)) ? ON : OFF)
#define RCC_GET_HSIRDY(_RCC)	((_RCC->CR & (1 << 1)) ? ON : OFF)

// HSE Config
#define RCC_SET_HSE(_RCC, OnOff)			\
		do {								\
			if(OnOff == ON) {				\
				_RCC-> CR |= (1 << 16);		\
			} else if(OnOff == OFF) {		\
				_RCC->CR &= ~ (1 << 16);	\
			}								\
		} while(0)

#define RCC_GET_HSE(_RCC)				((_RCC-> CR & (1 << 16)) ? ON : OFF)
#define RCC_GET_HSERDY(_RCC)			((_RCC-> CR & (1 << 17)) ? ON : OFF)


enum
{
	CSS_ENABLE,
	HSE_BYPASS_ENABLE,
};

// HSE Bypass -> (Active another External clock in case HSE not work, only Enable when HSE disable)
#define RCC_SET_HSEBYP(_RCC, OnOff)			\
		do { 								\
			if (OnOff == ON) { 				\
				_RCC->CR |= (1 << 18); 		\
			} else if (OnOff == OFF) { 		\
				_RCC->CR &= ~(1 << 18); 	\
			} 								\
		} while (0)

#define RCC_GET_HSEBYP(_RCC)			((_RCC-> CR & (1 << 18)) ? ON : OFF)

// CSS (Clock Security System -> Active another lock automaticaly in case main clock not work or not stable)
#define RCC_SET_CSSON(_RCC, OnOff)			\
		do { 								\
			if (OnOff == ON) { 				\
				_RCC->CR |= (1 << 19); 		\
			} else if (OnOff == OFF) { 		\
				_RCC->CR &= ~(1 << 19); 	\
			} 								\
		} while (0)

#define RCC_GET_CSSON(_RCC)				((_RCC-> CR & (1 << 19)) ? ON : OFF)

// PLL (Main PLL clock enable/Disble)

#define RCC_SET_PLLON(_RCC)				_RCC->CR |= (1 << 24)
#define RCC_GET_PLLRDY(_RCC)			((_RCC-> CR & (1 << 25)) ? ON : OFF)

// PLLI2S (PLL for I2 enable/disable)
#define RCC_SET_PLLI2S(_RCC)			_RCC->CR |= (1 << 26)
#define RCC_GET_PLLI2SRDY(_RCC)			((_RCC-> CR & (1 << 27)) ? ON : OFF)


/*******************************************************************************
 * PLL CONFIGURATION REGISTERS DEFINITION
 *******************************************************************************/
#define RCC_SET_PLLM(_RCC, pllM_val)		_RCC->PLLCFGR |= (pllM_val << 0)
#define RCC_SET_PLLN(_RCC, pllN_val)		_RCC->PLLCFGR |= (pllN_val << 6)
#define RCC_SET_PLLP(_RCC, pllP_val)		_RCC->PLLCFGR |= (pllP_val << 16)
#define RCC_SET_PLLQ(_RCC, pllQ_val)		_RCC->PLLCFGR |= (pllQ_val << 24)

// Select PLL source
#define PLL_HSI_SOURCE	0
#define PLL_HSE_SOURCE	1

#define RCC_SELECT_PLLSRC(_RCC, pll_src)			\
		do{											\
			if(pll_src == PLL_HSE_SOURCE){			\
				_RCC->PLLCFGR |= (1 << 22);			\
			} else if (pll_src == PLL_HSI_SOURCE){	\
				_RCC->PLLCFGR &= ~(1 << 22);		\
			}										\
		}while (0);

#define RCC_GET_PLLSRC_SELECTION(_RCC)		((_RCC-> PLLCFGR & (1 << 2)) ? ON : OFF)

/*******************************************************************************
 * CPGR REGISTERS DEFINITION
 *******************************************************************************/

// Select System Clock source

enum
{
	SYSCLK_HSI,		//00
	SYSCLK_HSE,		//01
	SYSCLK_PLL,		//10
	SYSCLK_NOT,		//11
};

#define RCC_SET_SYSCLK_SOURCE(_RCC, Sysclk_src)		_RCC->CFGR |= (Sysclk_src)	
#define RCC_GET_SYSCLK_SOURCE(_RCC)					(_RCC->CFGR & SYSCLK_NOT)

// AHB Bus prescaler
enum
{
	AHBCLK_DIV_0,
	AHBCLK_DIV_2,
	AHBCLK_DIV_4,
	AHBCLK_DIV_8,
	AHBCLK_DIV_16,
	AHBCLK_DIV_64,
	AHBCLK_DIV_128,
	AHBCLK_DIV_256,
	AHBCLK_DIV_512,
};

/// AHB Clock <= 25MHz when Ethernet is used
#define RCC_SET_AHBCLK_DIV(_RCC, AHBClk_Psc)		_RCC->CFGR |= (AHBClk_Psc << 4)

// APB1 Bus Clock Prescaler
enum
{
	APBCLK_DIV_0,
	APBCLK_DIV_2,
	APBCLK_DIV_4,
	APBCLK_DIV_8,
	APBCLK_DIV_16,
};

#define RCC_SET_APB1CLK_DIV(_RCC, APB1Clk_Psc)		_RCC->CFGR |= (APB1Clk_Psc << 10)
#define RCC_SET_APB2CLK_DIV(_RCC, APB2Clk_Psc)		_RCC->CFGR |= (APB2Clk_Psc << 13)

// RTC Clock base on HSE Prescaler
#define RCC_SET_RTCCLK_DIV(_RCC, RTCClk_Psc)		_RCC->CFGR |= (RTCClk_Psc << 16)

enum
{
	MCO1_ENABLE,
	MCO2_ENABLE,
};

// MCU Clock output 1
enum
{
	OUT1_HSI_CLK,
	OUT1_LSE_CLK,
	OUT1_HSE_CLK,
	OUT1_PLL_CLK,
};
#define RCC_SET_MCUCLK_OUT1(_RCC, MCO1_Src)			_RCC->CFGR |= (MCO1_Src << 21)

// MCU Lock output 1 prescaler
enum
{
	MCUCLK_OUT1_DIV0,
	MCUCLK_OUT1_DIV2 = 4,
	MCUCLK_OUT1_DIV3,
	MCUCLK_OUT1_DIV4,
	MCUCLK_OUT1_DIV5,
};
// I2S Clock
#define RCC_SELECT_I2SCLK_SRC(_RCC, I2SSrc)			_RCC->CFGR |= (I2SSrc << 23)

#define RCC_MCUCLK_OUT1_PSC(_RCC, MCO1_Psc_div)		_RCC->CFGR |= (MCO1_Psc_div << 24)

// MCU Clock output 2
enum
{
	SYSCLK_OUT2_CLK,
	PLLI2S_OUT2_CLK,
	HSE_OUT2_CLK,
	PLL_OUT2_CLK,
};
#define RCC_SET_MCUCLK_OUT2(_RCC, MCO2_Src)			_RCC->CFGR |= (MCO2_Src << 30)

// MCU Lock output 2 prescaler
enum
{
	MCUCLK_OUT2_DIV0,
	MCUCLK_OUT2_DIV2 = 4,
	MCUCLK_OUT2_DIV3,
	MCUCLK_OUT2_DIV4,
	MCUCLK_OUT2_DIV5,
};
#define RCC_MCUCLK_OUT2_PSC(_RCC, MCO1_Psc_div)		_RCC->CFGR |= (MCO1_Psc_div << 27)


/*******************************************************************************
 * AHB1ENR REGISTERS DEFINITION
 *******************************************************************************/
#define RCC_SET_AHB1ENR(_RCC, value)  	_RCC->AHB1ENR = value
#define RCC_GET_AHB1ENR(_RCC)  			_RCC->AHB1ENR

/*******************************************************************************
 * AHB2ENR REGISTERS DEFINITION
 *******************************************************************************/
#define RCC_SET_APB1ENR(_RCC, value)  	_RCC->APB1ENR = value
#define RCC_GET_APB1ENR(_RCC)  			_RCC->APB1ENR

/*******************************************************************************
 * APB2ENR REGISTERS DEFINITION
 *******************************************************************************/
#define RCC_SET_APB2ENR(_RCC, value)  	_RCC->APB2ENR = value
#define RCC_GET_APB2ENR(_RCC)  			_RCC->APB2ENR


/*******************************************************************************
 * FUNCTIONS DEFINITION
 *******************************************************************************/
#if defined(USE_SYSCLK_16MHz)
void RCC_SettingSystemClock16MHz(void);
#elif defined(USE_SYSCLK_24MHz)
void RCC_SettingSystemClock24MHz(void);
#elif defined(USE_SYSCLK_36MHz)
void RCC_SettingSystemClock36MHz(void);
#elif defined(USE_SYSCLK_48MHz)
void RCC_SettingSystemClock48MHz(void);
#elif defined(USE_SYSCLK_56MHz)
void RCC_SettingSystemClock56MHz(void);
#elif defined(USE_SYSCLK_72MHz)
void RCC_SettingSystemClock72MHz(void);
#endif

void RCC_SetPLLIndexValue(uint8_t PLLM, uint8_t PLLN, uint8_t PLLP, uint8_t PLLQ);
void RCC_EnableAnotherSourceForHSE(uint8_t Source);
void RCC_SettingMCUOutputClock(uint8_t MCOx, uint8_t MCUx_Source, uint8_t MCUx_Prescaler);
void RCC_EnablePeripheralClock(uint8_t Peripheral);
