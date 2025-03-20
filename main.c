#include "Thuan_STM32F407VGTx_Driver.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

uint8_t data_TX[] = {0xAA, 0x15, 0x55, 0x07, 0xAB};
uint8_t size = 4;
uint8_t data_RX;
int main()
{

#if defined(USE_SYSCLK_16MHz)
	RCC_SettingSystemClock16MHz();
#elif defined(USE_SYSCLK_24MHz)
	RCC_SettingSystemClock24MHz();
#elif defined(USE_SYSCLK_36MHz)
	RCC_SettingSystemClock36MHz();
#elif defined(USE_SYSCLK_48MHz)
	RCC_SettingSystemClock48MHz();
#elif defined(USE_SYSCLK_56MHz)
	RCC_SettingSystemClock56MHz();
#elif defined(USE_SYSCLK_72MHz)
	RCC_SettingSystemClock72MHz();
#endif

	RCC_EnablePeripheralClock(CLOCK_SYSCFG);
	SettingSystemTimer(PROCESSOR_CLKSRC, ENABLE, SYSTICK_TIMER_1MS);

	// SoftSPI_Master_Configuration(SOFT_SPI2, SAMPLING_TYPE_1_EDGE_RISING, MSB_FIRST);
	SoftSPI_Slave_Configuration(SOFT_SPI1, SAMPLING_TYPE_1_EDGE_RISING, MSB_FIRST);
	GPIO_Configuration(GPIO_A, GPIO_PIN1, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_PULLDOWN);
	EXTI_Configuration(EXTIx_PA_PIN, EXTI_LINE_1, NOT_MASKED, MASKED, RISING_TRIGGER);

	// NVIC configuration for EXTI2
	NVIC_Configuration(_EXTI1_IRQHandler, 0x04, ENABLE);

	// Uasrt2 configuration
	USART_Configuration(USART2, BAUDRATE_9600, DATA_BITS_8, OVERSAMPLING_BY_16, PARITY_DISABLE, EVEN_PARITY);
	NVIC_Configuration(_USART2_IRQHandler, 0x04, ENABLE);

	while (1)
	{
		LoopSystickTimerSetting();	// Systick timer loop

		// SoftSPI_ClockGeneration(SOFT_SPI2);

		// if(size > 0)
		// {
		// 	if(SoftSPI_Master_TransmiterRecevier(SOFT_SPI2, data_TX[4 - size], size) == true)
		// 		size--;
		// }
		// else
		// 	SoftSPI_SlaveDisSelected(SOFT_SPI2);

		if(size > 0)
		{
			if(SoftSPI_Slave_TransmitterReceiver(SOFT_SPI1, data_TX[4 - size], size) == true)
				size--;
		}

	}
}
