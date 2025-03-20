/**
 * @file NVIC.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for NVIC of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-11
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/

#include "stdio.h"
#include "stdint.h"

/*******************************************************************************
 * SYSTICK REGISTER STRUCTURE DEFINITION
 *******************************************************************************/

typedef struct
{
    volatile uint32_t NVIC_ISER[32];
    volatile uint32_t NVIC_ICER[32];
    volatile uint32_t NVIC_ISPR[32];
    volatile uint32_t NVIC_ICPR[32];
    volatile uint32_t NVIC_IABR[32];
    volatile uint32_t reserve[32];
    volatile uint8_t  NVIC_IPR[240];
} NVIC;

/*******************************************************************************
 * NVIC ADDRESS DEFINITION
 *******************************************************************************/
#define ADDRESS_NVIC 0xE000E100

/*******************************************************************************
 * ENUM DEFINITION
 *******************************************************************************/
// numerical order of IRQHandler
enum
{
    _WWDG_IRQHandler,
    _PVD_IRQHandler,
    _TAMP_STAMP_IRQHandler,
    _RTC_WKUP_IRQHandler,
    _FLASH_IRQHandler,
    _RCC_IRQHandler,
    _EXTI0_IRQHandler,
    _EXTI1_IRQHandler,
    _EXTI2_IRQHandler,
    _EXTI3_IRQHandler,
    _EXTI4_IRQHandler,
    _DMA1_Stream0_IRQHandler,
    _DMA1_Stream1_IRQHandler,
    _DMA1_Stream2_IRQHandler,
    _DMA1_Stream3_IRQHandler,
    _DMA1_Stream4_IRQHandler,
    _DMA1_Stream5_IRQHandler,
    _DMA1_Stream6_IRQHandler,
    _ADC_IRQHandler,
    _CAN1_TX_IRQHandler,
    _CAN1_RX0_IRQHandler,
    _CAN1_RX1_IRQHandler,
    _CAN1_SCE_IRQHandler,
    _EXTI9_5_IRQHandler,
    _TIM1_BRK_TIM9_IRQHandle,
    _TIM1_UP_TIM10_IRQHandle,
    _TIM1_TRG_COM_TIM11_IRQH,
    _TIM1_CC_IRQHandler,
    _TIM2_IRQHandler,
    _TIM3_IRQHandler,
    _TIM4_IRQHandler,
    _I2C1_EV_IRQHandler,
    _I2C1_ER_IRQHandler,
    _I2C2_EV_IRQHandler,
    _I2C2_ER_IRQHandler,
    _SPI1_IRQHandler,
    _SPI2_IRQHandler,
    _USART1_IRQHandler,
    _USART2_IRQHandler,
    _USART3_IRQHandler,
    _EXTI15_10_IRQHandler,
    _RTC_Alarm_IRQHandler,
    _OTG_FS_WKUP_IRQHandler,
    _TIM8_BRK_TIM12_IRQHandl,
    _TIM8_UP_TIM13_IRQHandle,
    _TIM8_TRG_COM_TIM14_IRQH,
    _TIM8_CC_IRQHandler,
    _DMA1_Stream7_IRQHandler,
    _FMC_IRQHandler,
    _SDIO_IRQHandler,
    _TIM5_IRQHandler,
    _SPI3_IRQHandler,
    _UART4_IRQHandler,
    _UART5_IRQHandler,
    _TIM6_DAC_IRQHandler,
    _TIM7_IRQHandler,
    _DMA2_Stream0_IRQHandler,
    _DMA2_Stream1_IRQHandler,
    _DMA2_Stream2_IRQHandler,
    _DMA2_Stream3_IRQHandler,
    _DMA2_Stream4_IRQHandler,
    _ETH_IRQHandler,
    _ETH_WKUP_IRQHandler,
    _CAN2_TX_IRQHandler,
    _CAN2_RX0_IRQHandler,
    _CAN2_RX1_IRQHandler,
    _CAN2_SCE_IRQHandler,
    _OTG_FS_IRQHandler,
    _DMA2_Stream5_IRQHandler,
    _DMA2_Stream6_IRQHandler,
    _DMA2_Stream7_IRQHandler,
    _USART6_IRQHandler,
    _I2C3_EV_IRQHandler,
    _I2C3_ER_IRQHandler,
    _OTG_HS_EP1_OUT_IRQHandl,
    _OTG_HS_EP1_IN_IRQHandle,
    _OTG_HS_WKUP_IRQHandler,
    _OTG_HS_IRQHandler,
    _DCMI_IRQHandler,
    _HASH_RNG_IRQHandler,
    _FPU_IRQHandler,
};

// enum to get current status interrupt.
enum
{
    NVIC_ENABLE_REGISTER,
    NVIC_CLEAR_ENABLE_REGISTER,
    NVIC_SET_PENDING_REGISTGER,
    NVIC_CLEAR_PENDING_REGISTGER,
    NVIC_ACTIVE_BIT_REGISTGER,
    NVIC_PRIORITY_REGISTGER,
};
/*******************************************************************************
 * COMMON DEFINITION
 *******************************************************************************/

#define INT_ENABLE      1
#define INT_DISABLE     0

/*******************************************************************************
 * NVIC_ISER REGISTERS DEFINITION
 *******************************************************************************/

#define NVIC_SET_ENABLE_INTERRUPT(NVIC, IRQNumber)                  NVIC->NVIC_ISER[IRQNumber / 32] |= 1 << (IRQNumber % 32)
#define NVIC_GET_STATUS_ENABLE_INTERRUPT(NVIC, IRQNumber)           NVIC->NVIC_ISER[IRQNumber / 32]

/*******************************************************************************
 * NVIC_ICER REGISTERS DEFINITION
 *******************************************************************************/

#define NVIC_SET_CLEAR_ENABLE_INTERRUPT(NVIC, IRQNumber)            NVIC->NVIC_ICER[IRQNumber / 32] |= 1 << (IRQNumber % 32)
#define NVIC_GET_STATUS_CLEAR_INTERRUPT(NVIC, IRQNumber)            NVIC->NVIC_ICER[IRQNumber / 32]

/*******************************************************************************
 * NVIC_ISPR REGISTERS DEFINITION   
 *******************************************************************************/

#define NVIC_SET_PENDING_INTERRUPT(NVIC, IRQNumber)                 NVIC->NVIC_ISPR[IRQNumber / 32] |= 1 << (IRQNumber % 32)
#define NIVC_GET_STATUS_PENDING_INTERRUPT(NVIC, IRQNumber)          NVIC->NVIC_ISPR[IRQNumber / 32]

/*******************************************************************************
 * NVIC_ICPR REGISTERS DEFINITION
 *******************************************************************************/

#define NVIC_CLEAR_PENDING_INTERRUPT(NVIC, IRQNumber)               NVIC->NVIC_ICPR[IRQNumber / 32] |= 1 << (IRQNumber % 32)
#define NVIC_GET_STATUS_CLEAR_PENDING_INTERRUPT(NVIC, IRQNumber)    NVIC->NVIC_ICPR[IRQNumber / 32]

/*******************************************************************************
 * NVIC_IABR REGISTERS DEFINITION
 *******************************************************************************/

#define NVIC_SET_ACTIVE_INTERRUPT(NVIC, IRQNumber)                  NVIC->NVIC_IABR[IRQNumber / 32] |= 1 << (IRQNumber % 32)
#define NVIC_GET_STATUS_ACTIVE_INTERRUPT(NVIC, IRQNumber)           NVIC->NVIC_IABR[IRQNumber / 32]

/*******************************************************************************
 * NVIC_IPR REGISTERS DEFINITION
 *******************************************************************************/

#define NVIC_SET_PRIORITY_INTERUPT(NVIC, IRQNumber, Priority)       (NVIC->NVIC_IPR[IRQNumber] = (Priority))
#define NVIC_GET_PRIORITY_INTERUPT(NVIC, IRQNumber)                 NVIC->NVIC_IPR[IRQNumber]


/*******************************************************************************
 * FUNCTIONS DEFINITION
 *******************************************************************************/

void NVIC_SetInterruptEnable(uint8_t IQRNumber);
void NVIC_ClearInterruptEnable(uint8_t IQRNumber);
void NVIC_SetPendingInterrupt(uint8_t IRQNumber);
void NVIC_ClearPendingInterrupt(uint8_t IRQNumber);
void NVIC_SetActiveInterrupt(uint8_t IRQNumber);

void NVIC_SetPriority(uint8_t IRQNumber, uint8_t priority);
uint8_t NVIC_GetStatusInterrupt(uint8_t IRQNumber, uint8_t register_read);

void NVIC_Init(uint8_t IRQNumber);
void NVIC_Configuration(uint8_t IRQNumber, uint8_t priority, uint8_t enable);


