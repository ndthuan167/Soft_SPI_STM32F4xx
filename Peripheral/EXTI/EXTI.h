/**
 * @file EXTI.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for EXTI(External interrupt) of STM32F407VGTx (ARMCortex M4)
 * @date 2024-07-12
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

typedef void INTERRUPT;

/*******************************************************************************
 * SYSCFG ADDRESS DEFINITION
 *******************************************************************************/

#define ADDRESS_SYSCFG 0x40013800

/*******************************************************************************
 * SYSCFG REGISTER STRUCTURE DEFINITION
 *******************************************************************************/

typedef struct
{
    volatile uint32_t SYSCFG_MEMRMP;
    volatile uint32_t SYSCFG_PMC;
    volatile uint32_t SYSCFG_EXTICRx[4];
    volatile uint32_t SYSCFG_CMPCR;
} SYSCFG;

/*******************************************************************************
 * EXTI ADDRESS DEFINITION
 *******************************************************************************/

#define ADDRESS_EXTI 0x40013C00

/*******************************************************************************
 * EXTI REGISTER STRUCTURE DEFINITION
 *******************************************************************************/

typedef struct
{
    volatile uint32_t EXTI_IMR;
    volatile uint32_t EXTI_EMR;
    volatile uint32_t EXTI_RTSR;
    volatile uint32_t EXTI_FTSR;
    volatile uint32_t EXTI_SWIER;
    volatile uint32_t EXTI_PR;
} EXTIx;

/*******************************************************************************
 * ENUM DEFINITION
 *******************************************************************************/
enum
{
    EXTIx_PA_PIN,
    EXTIx_PB_PIN,
    EXTIx_PC_PIN,
    EXTIx_PD_PIN,
    EXTIx_PE_PIN,
    EXTIx_PF_PIN,
    EXTIx_PG_PIN,
    EXTIx_PH_PIN,
    EXTIx_PI_PIN,
    EXTIx_PJ_PIN,
    EXTIx_PK_PIN,
};

enum
{
    EXTI_LINE_0,
    EXTI_LINE_1,
    EXTI_LINE_2,
    EXTI_LINE_3,
    EXTI_LINE_4,
    EXTI_LINE_5,
    EXTI_LINE_6,
    EXTI_LINE_7,
    EXTI_LINE_8,
    EXTI_LINE_9,
    EXTI_LINE_10,
    EXTI_LINE_11,
    EXTI_LINE_12,
    EXTI_LINE_13,
    EXTI_LINE_14,
    EXTI_LINE_15,
    EXTI_LINE_16,
    EXTI_LINE_17,
    EXTI_LINE_18,
    EXTI_LINE_19,
    EXTI_LINE_20,
    EXTI_LINE_21,
    EXTI_LINE_22,
};

/*******************************************************************************
 * COMMON DEFINITION
 *******************************************************************************/

#define NOT_MASKED 1
#define MASKED 0

#define ENABLE 1
#define DISABLE 0

#define RISING_TRIGGER 0
#define FALLING_TRIGGER 1

/*******************************************************************************
 * SYSCFG_EXTICRx REGISTERS DEFINITION
 *******************************************************************************/

#define SYSCFG_SET_PORT_EXTI(SYSCFG, CRx, port, EXTI_line)    SYSCFG->SYSCFG_EXTICRx[(CRx)] |= port << (EXTI_line % 4) * 4
#define SYSCFG_GET_PORT_EXTI(SYSCFG, CRx)                     SYSCFG->SYSCFG_EXTICRx[(CRx)]

/*******************************************************************************
 * EXTI_IMR REGISTERS DEFINITION
 *******************************************************************************/

#define EXTI_SET_IMR(EXTI, EXTI_LineX, value)   EXTI->EXTI_IMR |= (value << EXTI_LineX)
#define EXTI_GET_IMR(EXTI, EXTI_LineX)          EXTI->EXTI_IMR

/*******************************************************************************
 * EXTI_EMR REGISTERS DEFINITION
 *******************************************************************************/

#define EXTI_SET_EMR(EXTI, EXTI_LineX, value)   EXTI->EXTI_EMR |= (value << EXTI_LineX)
#define EXTI_GET_EMR(EXTI, EXTI_LineX)          EXTI->EXTI_EMR

/*******************************************************************************
 * EXTI_RTSR REGISTERS DEFINITION
 *******************************************************************************/

#define EXTI_SET_RTSR(EXTI, EXTI_LineX, value)  EXTI->EXTI_RTSR |= (value << EXTI_LineX)
#define EXTI_GET_RTSR(EXTI, EXTI_LineX)         EXTI->EXTI_RTSR

/*******************************************************************************
 * EXTI_FTSR REGISTERS DEFINITION
 *******************************************************************************/
#define EXTI_SET_FTRS(EXTI, EXTI_LineX, value)  EXTI->EXTI_FTSR |= (value << EXTI_LineX)
#define EXTI_GET_FTSR(EXTI, EXTI_LineX)         EXTI->EXTI_FTSR

/*******************************************************************************
 * EXTI_SWIER REGISTERS DEFINITION
 *******************************************************************************/

#define EXTI_SET_SWIER(EXTI, EXTI_LineX)        EXTI->EXTI_SWIER |= (1 << EXTI_LineX)

/*******************************************************************************
 * EXTI_PR REGISTERS DEFINITION
 *******************************************************************************/

#define EXTI_SET_PR(EXTI, EXTI_LineX)           EXTI->EXTI_PR |= (1 << EXTI_LineX)
#define EXTI_GET_PR(EXTI, EXTI_LineX)           EXTI->EXTI_PR

/*******************************************************************************
 * FUNCTIONS DEFINITION
 *******************************************************************************/

// SYSCFG
void SYSCFG_ConfigPortEXTI(uint8_t port, uint8_t EXTI_line);
uint8_t SYSCFG_GetStatusPortEXTI(uint8_t EXTI_line);

// EXTI
void EXTI_Configuration(uint8_t EXTI_port,uint8_t EXTI_line, uint8_t mask_it, uint8_t mask_event, uint8_t trigger_type);
void EXTI_SettingMaskInterrupt(uint8_t EXTI_line, uint8_t mask);
uint8_t EXTI_GetValueMaskInterrupt(uint8_t EXTI_line);
void EXTI_SettingMaskEventRequest(uint8_t EXTI_line, uint8_t mask);
uint8_t EXTI_GetValueMaskEventRequest(uint8_t EXTI_line);
void EXTI_SetInterruptOccurebySW(uint8_t EXTI_line);
void EXTI_ClearFlagInterruptInLine(uint8_t EXTI_line);
uint8_t EXTI_GetFlagInterruptStatus(uint8_t EXTI_line);
uint8_t EXTI_GetTriggerStatus(uint8_t EXTI_line, uint8_t trigger_type);

INTERRUPT HandleLED1(void);
INTERRUPT HandleLED2(void);


