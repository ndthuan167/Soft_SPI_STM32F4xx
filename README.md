# Soft_SPI_STM32F4xx
Writing Soft SPI communication structure (bare metal code, not use HAL, CMSIS Library) for STM32F4xx MCU
========================================================================================================================================

## Method: Create GPIO pins for SPI pin (CS, SCK, MISO, MOSI) and System clock (Systick) combine with SPI setting (Sampling type CPOL/CPHA, MSB/LSB,...) to create Soft SPI protocol.

1. Configure SPIx pin as GPIO pin for Master or Slave:
  + Master:
```c
/**
*******************************************************************************
* @ Name : SoftSPI_Master_GPIO_Configuration
* @ Parameters: uint8_t spi_n
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Configuration for GPIO of Soft SPI as Master
*         + CS: Output, SCK: Output, MISO: Input, MOSI: Output
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-19
*******************************************************************************
*/
void SoftSPI_Master_GPIO_Configuration(uint8_t spi_n)
{
    switch (spi_n)
    {
        case SOFT_SPI1:
            RCC_EnablePeripheralClock(CLOCK_GPIO_A);
            // PA4 -> SPI1_CS, PA5 -> SPI1_SCK, PA6 -> SPI1_MISO, PA7 -> SPI1_MOSI
            GPIO_Configuration(gpio_A_SSPI, GPIO_PIN4, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);      // CS
            GPIO_Configuration(gpio_A_SSPI, GPIO_PIN5, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // SCK
            GPIO_Configuration(gpio_A_SSPI, GPIO_PIN6, MODER_INPUT,  OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MISO
            GPIO_Configuration(gpio_A_SSPI, GPIO_PIN7, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MOSI
        break;
        case SOFT_SPI2:
            RCC_EnablePeripheralClock(CLOCK_GPIO_B);
            // PB12 -> SPI2_CS, PB13 -> SPI2_SCK, PB14 -> SPI2_MISO, PB15 -> SPI2_MOSI
            GPIO_Configuration(gpio_B_SSPI, GPIO_PIN12, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);      // CS
            GPIO_Configuration(gpio_B_SSPI, GPIO_PIN13, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // SCK
            GPIO_Configuration(gpio_B_SSPI, GPIO_PIN14, MODER_INPUT,  OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MISO
            GPIO_Configuration(gpio_B_SSPI, GPIO_PIN15, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MOSI
        break;
        case SOFT_SPI3:
            RCC_EnablePeripheralClock(CLOCK_GPIO_A);
            RCC_EnablePeripheralClock(CLOCK_GPIO_C);

            // PA15 -> SPI3_CS, PC10 -> SPI3_SCK, PC11 -> SPI3_MISO, PC12 -> SPI3_MOSI
            GPIO_Configuration(gpio_A_SSPI, GPIO_PIN15, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);      // CS
            GPIO_Configuration(gpio_C_SSPI, GPIO_PIN10, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // SCK
            GPIO_Configuration(gpio_C_SSPI, GPIO_PIN11, MODER_INPUT,  OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MISO
            GPIO_Configuration(gpio_C_SSPI, GPIO_PIN12, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MOSI 
        break;
        default:
        break;
    }
}
```
  ###### + With Slave mode: CS: Input, SCK: Input, MISO: Output, MOSI: Input

2. SCK pin: Using system timer to create clock coressponding with clock on SPIx bus (APB bus). Base on Flag systick timer to set GPIO signal in SCK pin. (In this example, I create a clock with T = 100ms to easily to monitoring)
```c
/**
*******************************************************************************
* @ Name : SoftSPI_ClockGeneration
* @ Parameters: uint8_t spi_n
* @ Registers :
* @ Descriptions :
*		- Create clock signal for SPI base on system clock and GPIO output
*         . Note: example use systick 50ms to generate clock signal ->> (T = 100ms)
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
void SoftSPI_ClockGeneration(uint8_t spi_n)
{
	if(GetFlagTimerSystick50ms())
	{
		clk_div++;
		if(clk_div % 2 == 0)
        {
            switch (spi_n)
            {
                case SOFT_SPI1:
                    GPIO_SettingOutputDataBSRR(gpio_A_SSPI, GPIO_PIN5, CLEAR);
                break;
                case SOFT_SPI2:
                    GPIO_SettingOutputDataBSRR(gpio_B_SSPI, GPIO_PIN13, CLEAR);
                break;
                case SOFT_SPI3:
                    GPIO_SettingOutputDataBSRR(gpio_C_SSPI, GPIO_PIN10, CLEAR);
                break;
                
                default:
                break;
            }
        }
		else
        {
            switch (spi_n)
            {
                case SOFT_SPI1:
                    GPIO_SettingOutputDataBSRR(gpio_A_SSPI, GPIO_PIN5, SET);
                break;
                case SOFT_SPI2:
                    GPIO_SettingOutputDataBSRR(gpio_B_SSPI, GPIO_PIN13, SET);
                break;
                case SOFT_SPI3:
                    GPIO_SettingOutputDataBSRR(gpio_C_SSPI, GPIO_PIN10, SET);
                break;
                
                default:
                break;
            }
        }
	}
}
``` 
3. MOSI / MISO: Base on configuration of Sampling type (CPHA/CPOL) and MSB/LSB value and the signal in SCK pin was created before to set GPIO signal in MISO pin and MOSI pin corresponding with data transmit, receive. In this example, I get data receive and send it to terminal by USART2.
```c
/**
*******************************************************************************
* @ Name : SoftSPI_Master_TransmiterRecevier
* @ Parameters: uint8_t spi_n, uint8_t data, uint8_t size
* @ Registers :
* @ Descriptions :
*		- Transmit data to slave by set the level of MOSI pin 
*       - Receive data from slave by get the level of MISO pin
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-18
*******************************************************************************
*/
bool SoftSPI_Master_TransmiterRecevier(uint8_t spi_n, uint8_t data, uint8_t size)
{
    bool bReturn = false;
    uint8_t cs_status = SoftSPI_GetCSStatus(spi_n);
    uint8_t bin_data[8];
    uint8_t m_RX_bits;

    HexToBinConvert(data, bin_data);  // Convert the hexadecimal value to a binary string

    if(cs_status == CLEAR)  // Slave Selected
    {
        if(sampling_second_edge == true)    // start send data to MOSI after 1 cycle clock if sampling type is second edge
        {
            if(GetFlagTimerSystick100ms())
            {
                sampling_second_edge = false;
            }
        }

        if(sampling_second_edge == false)
        {
            if(SoftSPI_EdgeDetection(spi_n) == first_edge)
            {
                if(first_bit_sent == 0)
                {
                    first_bit_sent = 1;
                    if(bin_data[index_first_bit] == 0x30)    // first bit (MSB))
                    {
                        SoftSPI_SetMOSI(spi_n, CLEAR);
                    }
                    else if(bin_data[index_first_bit] == 0x31)
                    {
                        SoftSPI_SetMOSI(spi_n, SET);
                    }
                }

                // master receiver
                for (j = 0; j < 100; j++){} // delay for stable data
                if (SoftSPI_Master_GetMISOData(spi_n) == SET)   // Read MISO
                    m_RX_bits = 0x01 + '0';
                else
                    m_RX_bits = 0x00 + '0';
                USART_SendData(usart_spi_2, &m_RX_bits, 1);

            }

            if((SoftSPI_FromSecEdgeDetection(spi_n) == second_edge)
            && first_bit_sent == 1)
            {
                from_second_bit = 1;
                if(bin_data[abs(bin_index - index_first_bit)] == 0x30)
                {
                    SoftSPI_SetMOSI(spi_n, CLEAR);
                }
                else if(bin_data[abs(bin_index - index_first_bit)] == 0x31)
                {
                    SoftSPI_SetMOSI(spi_n, SET);
                }
            }

            if(from_second_bit == 1)
            {
                if(GetFlagTimerSystick100ms())
                {
                    bin_index++;
                    index_recevier++;
                    if(bin_index > 8)
                    {
                        bin_index = 1;
                        first_bit_sent = 0;
                        from_second_bit = 0;
                        bReturn = true;
                    }
                }
            }
            else
            {
                if(GetFlagTimerSystick100ms())
                    index_recevier++;
            }

            // Convert RX_data to Hex and send to USART
            if(index_recevier >= 8)
            {
                // USART_SendData(usart_spi_2, &hex_str, strlen(hex_str));
                USART_SendData(usart_spi_2, " ", 1);
                index_recevier = 0;
            }
        }
    }

    return bReturn;
}
```

- The transmiter and receiver start only after the CS pin was set to Low level:

```c
/**
*******************************************************************************
* @ Name : SoftSPI_SlaveSelected
* @ Parameters: uint8_t spi_n
* @ Registers :
* @ Descriptions :
*		- When slave selected, the CS pin will be set to low level corresponding to the SPI
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
void SoftSPI_SlaveSelected(uint8_t spin_n)
{
    switch (spin_n)
    {
        case SOFT_SPI1:
            GPIO_SettingOutputDataBSRR(gpio_A_SSPI, GPIO_PIN4, CLEAR);
        break;
        case SOFT_SPI2:
            GPIO_SettingOutputDataBSRR(gpio_B_SSPI, GPIO_PIN12, CLEAR);
        break;
        case SOFT_SPI3:
            GPIO_SettingOutputDataBSRR(gpio_A_SSPI, GPIO_PIN15, CLEAR);
        break;
    }
}

```

## Example: Communication between 2 MCU by SPI protocol is wrote by Soft SPI method.
- Data send from Master to Slave: 4 bytes : 0x88 > 0x13 > 0xA5 > 0xB1 
- Data send from Slave to Master: 4 bytes : 0x15 > 0x8A > 0x23 > 0xB5

##### -->> I used Proteus to simulate this operation as follow (Push button to set CS to Low to start transmitter and receiver):

![z6426760823605_1a32f0e07f68599ab42ed1a0e7e2a609](https://github.com/user-attachments/assets/fc1b5f3e-180f-4774-ba71-4c7b930ba7c9)

### Result:
![z6426760824713_a8c8d2920b777ad812da7cc9c607a8ed](https://github.com/user-attachments/assets/e4d1ad9c-20c0-4e63-a98b-46133b55e321)
