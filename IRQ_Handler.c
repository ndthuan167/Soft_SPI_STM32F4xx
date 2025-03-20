#include "Peripheral/SystemTimer/Systick.h"
#include "Peripheral/USART/USART.h"
#include "Peripheral/EXTI/EXTI.h"

/*******************************************************************************
SYSTICK_HANDLER
*******************************************************************************/
void SysTick_Handler(void)
{
    systick_timer_loop_1ms_IT++;
}


/*******************************************************************************
EXTI_HANDLER
*******************************************************************************/

void EXTI1_IRQHandler()
{
    HandleLED1();
}
