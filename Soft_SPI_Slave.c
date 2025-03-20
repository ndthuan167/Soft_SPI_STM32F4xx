/**
 * @file Soft_SPI_Slave.c
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Configuration for Soft SPI in STM32F407VGTx (ARMCortex M4) as Slave
 * @date 2025-03-19
 *
 */

/******************************************************************************
 * Include Files
 ******************************************************************************/
#include "Soft_SPI.h"
#include "../RCC/RCC.h"
#include "../GPIO/GPIO.h"
#include "../SystemTimer/Systick.h"
#include "../USART/USART.h"

/******************************************************************************
 * Pointer definition to address of GPIO and USART definition
 ******************************************************************************/
GPIOn *gpio_A_SSPI_S = (GPIOn*) ADDRESS_GPIO_A;
GPIOn *gpio_B_SSPI_S = (GPIOn*) ADDRESS_GPIO_B;
GPIOn *gpio_C_SSPI_S = (GPIOn*) ADDRESS_GPIO_C;
USARTn *usart2_spi_sl = (USARTn*) ADDRESS_USART_2;

/******************************************************************************
 * Variables definition
 ******************************************************************************/
uint8_t bin_index_sl = 1;
uint8_t first_bit_sent_sl = 0;
uint8_t from_second_bit_sl = 0;

uint8_t sl_pre_clk = 0;
uint8_t sl_crr_clk = 0;
uint8_t sl_nxt_pre_clk = 0;
uint8_t sl_nxt_crr_clk = 0;
uint8_t sl_re_crr_clk = 0;
uint8_t sl_re_pre_clk = 0;

bool sl_sampling_second_edge = false;
uint8_t sl_first_edge = RISING_EDGE;
uint8_t sl_second_edge = FALLING_EDGE;
uint8_t sl_index_first_bit;

uint8_t sl_index_recevier = 0;
uint8_t RX_data = 0;
uint16_t i;

/******************************************************************************
 * Private functions definition
 ******************************************************************************/
void SoftSPI_Slave_GPIO_Configuration(uint8_t spi_n);
uint8_t SoftSPI_GetInputCSValue(uint8_t spi_n);
uint8_t SoftSPI_Sl_Transmit_EdgeDetection(uint8_t spin_n);
uint8_t SoftSPI_Sl_Transmit_FromSecEdgeDetection(uint8_t spin_n);
uint8_t SoftSPI_Slave_GetMOSIData(uint8_t spi_n);
void SoftSPI_Slave_SendMISO(uint8_t spin_n, uint8_t value);

/**
*******************************************************************************
* @ Name : SoftSPI_Slave_GPIO_Configuration
* @ Parameters: uint8_t spi_n
* @ Registers : Register of GPIOx
* @ Descriptions :
*		- Configuration for GPIO of Soft SPI as Master
*         + CS: Input, SCK: Input, MISO: Output, MOSI: Input
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-19
*******************************************************************************
*/
void SoftSPI_Slave_GPIO_Configuration(uint8_t spi_n)
{
    switch (spi_n)
    {
        case SOFT_SPI1:
            RCC_EnablePeripheralClock(CLOCK_GPIO_A);
            // PA4 -> SPI1_CS, PA5 -> SPI1_SCK, PA6 -> SPI1_MISO, PA7 -> SPI1_MOSI
            GPIO_Configuration(gpio_A_SSPI_S, GPIO_PIN4, MODER_INPUT,  OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);      // CS
            GPIO_Configuration(gpio_A_SSPI_S, GPIO_PIN5, MODER_INPUT,  OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // SCK
            GPIO_Configuration(gpio_A_SSPI_S, GPIO_PIN6, MODER_OUTPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MISO
            GPIO_Configuration(gpio_A_SSPI_S, GPIO_PIN7, MODER_INPUT,  OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MOSI
        break;
        case SOFT_SPI2:
            RCC_EnablePeripheralClock(CLOCK_GPIO_B);
            // PB12 -> SPI2_CS, PB13 -> SPI2_SCK, PB14 -> SPI2_MISO, PB15 -> SPI2_MOSI
            GPIO_Configuration(gpio_B_SSPI_S, GPIO_PIN12, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);      // CS
            GPIO_Configuration(gpio_B_SSPI_S, GPIO_PIN13, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // SCK
            GPIO_Configuration(gpio_B_SSPI_S, GPIO_PIN14, MODER_OUTPUT,OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MISO
            GPIO_Configuration(gpio_B_SSPI_S, GPIO_PIN15, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MOSI
        break;
        case SOFT_SPI3:
            RCC_EnablePeripheralClock(CLOCK_GPIO_A);
            RCC_EnablePeripheralClock(CLOCK_GPIO_C);

            // PA15 -> SPI3_CS, PC10 -> SPI3_SCK, PC11 -> SPI3_MISO, PC12 -> SPI3_MOSI
            GPIO_Configuration(gpio_A_SSPI_S, GPIO_PIN15, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_LOW, PUPDR_NOTHING);      // CS
            GPIO_Configuration(gpio_C_SSPI_S, GPIO_PIN10, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // SCK
            GPIO_Configuration(gpio_C_SSPI_S, GPIO_PIN11, MODER_OUTPUT,OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MISO
            GPIO_Configuration(gpio_C_SSPI_S, GPIO_PIN12, MODER_INPUT, OTYPER_PUSHPULL, OSPEEDR_VERYHIGH, PUPDR_NOTHING); // MOSI 
        break;
        default:
        break;
    }
}

/**
*******************************************************************************
* @ Name : SoftSPI_GetInputCSValue
* @ Parameters: uint8_t spi_n
* @ Registers :
* @ Descriptions :
*		- Get Slave selected status from CS pin
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
uint8_t SoftSPI_GetInputCSValue(uint8_t spi_n)
{
    uint8_t cs_value;

    switch (spi_n)
    {
        case SOFT_SPI1:
            cs_value = GPIO_GetInputData(gpio_A_SSPI_S, GPIO_PIN4);
            break;
        case SOFT_SPI2:
            cs_value = GPIO_GetInputData(gpio_B_SSPI_S, GPIO_PIN12);
            break;
        case SOFT_SPI3:
            cs_value = GPIO_GetInputData(gpio_A_SSPI_S, GPIO_PIN15);
            break;
        default:
            break;
    }
    return cs_value;
}

/**
*******************************************************************************
* @ Name : SoftSPI_Sl_Transmit_EdgeDetection
* @ Parameters: uint8_t spi_n
* @ Registers :
* @ Descriptions :
*		- Detect the rising and falling edge of first clock signal for transmiting data to Master via MISO
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
uint8_t SoftSPI_Sl_Transmit_EdgeDetection(uint8_t spin_n)
{
    uint8_t edge = NOTHING_EDGE;

    switch (spin_n)
    {
        case SOFT_SPI1:
            sl_crr_clk = GPIO_GetInputData(gpio_A_SSPI_S, GPIO_PIN5);
        break;
        case SOFT_SPI2:
            sl_crr_clk = GPIO_GetInputData(gpio_B_SSPI_S, GPIO_PIN13);
        break;
        case SOFT_SPI3:
            sl_crr_clk = GPIO_GetInputData(gpio_C_SSPI_S, GPIO_PIN10);
        break;
        default:
        break;
    }

    if(sl_crr_clk == 1 && sl_pre_clk == 0)
        edge = RISING_EDGE;
    if(sl_crr_clk == 0 && sl_pre_clk == 1)
        edge = FALLING_EDGE;

    sl_pre_clk = sl_crr_clk;

    return edge;
}

/**
*******************************************************************************
* @ Name : SoftSPI_Sl_Transmit_FromSecEdgeDetection
* @ Parameters: uint8_t spi_n
* @ Registers :
* @ Descriptions :
*		- Detect the rising and falling edge from second clock signal for transmiting data to Master via MISO
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
uint8_t SoftSPI_Sl_Transmit_FromSecEdgeDetection(uint8_t spin_n)
{
    uint8_t edge = NOTHING_EDGE;

    switch (spin_n)
    {
        case SOFT_SPI1:
            sl_nxt_crr_clk = GPIO_GetInputData(gpio_A_SSPI_S, GPIO_PIN5);
        break;
        case SOFT_SPI2:
            sl_nxt_crr_clk = GPIO_GetInputData(gpio_B_SSPI_S, GPIO_PIN13);
        break;
        case SOFT_SPI3:
            sl_nxt_crr_clk = GPIO_GetInputData(gpio_C_SSPI_S, GPIO_PIN10);
        break;
        default:
        break;
    }

    if(sl_nxt_crr_clk == 1 && sl_nxt_pre_clk == 0)
        edge = RISING_EDGE;
    if(sl_nxt_crr_clk == 0 && sl_nxt_pre_clk == 1)
        edge = FALLING_EDGE;

    sl_nxt_pre_clk = sl_nxt_crr_clk;

    return edge;
}

/**
*******************************************************************************
* @ Name : SoftSPI_Slave_SendMISO
* @ Parameters: uint8_t spin_n, uint8_t value
* @ Registers :
* @ Descriptions :
*		- Set the level of MISO pin corresponding to the SPI base on the value
* @ Return value : void
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
void SoftSPI_Slave_SendMISO(uint8_t spin_n, uint8_t value)
{
    switch (spin_n)
    {
    case SOFT_SPI1:
        if(value == SET)
            GPIO_SetOutputData(gpio_A_SSPI_S, GPIO_PIN6, SET);
        else
            GPIO_SetOutputData(gpio_A_SSPI_S, GPIO_PIN6, CLEAR);
        break;
    
    case SOFT_SPI2:
        if(value == SET)
            GPIO_SetOutputData(gpio_B_SSPI_S, GPIO_PIN14, SET);
        else
            GPIO_SetOutputData(gpio_B_SSPI_S, GPIO_PIN14, CLEAR);
        break;
    
    case SOFT_SPI3:
        if(value == SET)
            GPIO_SetOutputData(gpio_C_SSPI_S, GPIO_PIN11, SET);
        else
            GPIO_SetOutputData(gpio_C_SSPI_S, GPIO_PIN11, CLEAR);
        break;
    
    default:
        break;
    }
}

/**
*******************************************************************************
* @ Name : SoftSPI_Slave_GetMOSIData
* @ Parameters: uint8_t spin_n
* @ Registers :
* @ Descriptions :
*		- Get the level of MOSI pin corresponding to the SPI
* @ Return value : uint8_t
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-17
*******************************************************************************
*/
uint8_t SoftSPI_Slave_GetMOSIData(uint8_t spi_n)
{
    uint8_t MOSI_value;
        switch (spi_n)
        {
        case SOFT_SPI1:
            MOSI_value = GPIO_GetInputData(gpio_A_SSPI_S, GPIO_PIN7);
            break;
        
        case SOFT_SPI2:
            MOSI_value = GPIO_GetInputData(gpio_B_SSPI_S, GPIO_PIN15);
            break;
        
        case SOFT_SPI3:
            MOSI_value = GPIO_GetInputData(gpio_C_SSPI_S, GPIO_PIN12);
            break;
        
        default:
            break;
    }

    return MOSI_value;
}

/**
*******************************************************************************
* @ Name : SoftSPI_Slave_TransmitterReceiver
* @ Parameters: uint8_t spi_n, uint8_t data, uint8_t size
* @ Registers :
* @ Descriptions :
*		- Transmit data to Master by set the level of MISO pin 
*       - Receive data from Master by get the level of MOSI pin
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-18
*******************************************************************************
*/
bool SoftSPI_Slave_TransmitterReceiver(uint8_t spi_n, uint8_t data, uint8_t size)
{
    uint8_t get_cs_value = SoftSPI_GetInputCSValue(spi_n);
    uint8_t RX_bits;
    uint8_t bin_data[8];

    bool bReturn = false;

    HexToBinConvert(data, bin_data);

    if(get_cs_value == CLEAR)
    {
        if(sl_sampling_second_edge == true)    // start get data from MOSI after 1 cycle clock if sampling type is second edge
        {
            if(GetFlagTimerSystick100ms())
            {
                sl_sampling_second_edge = false;
            }
        }

        if(sl_sampling_second_edge == false)
        {

            if(SoftSPI_Sl_Transmit_EdgeDetection(spi_n) == sl_first_edge)
            {
                if(first_bit_sent_sl == 0)
                {
                    // slave transmitter
                    first_bit_sent_sl = 1;
                    if(bin_data[sl_index_first_bit] == 0x30)    // first bit (MSB))
                    {
                        SoftSPI_Slave_SendMISO(spi_n, CLEAR);
                    }
                    else if(bin_data[sl_index_first_bit] == 0x31)
                    {
                        SoftSPI_Slave_SendMISO(spi_n, SET);
                    }
                }
                // else
                // {
                for (i = 0; i < 100; i++){} // delay for stable data
                if (SoftSPI_Slave_GetMOSIData(spi_n) == SET) // Read MOSI
                    RX_bits = 0x01 + '0';
                else
                    RX_bits = 0x00 + '0';
                USART_SendData(usart_spi_sl_2, &RX_bits, 1);
                // }
            }

            if((SoftSPI_Sl_Transmit_FromSecEdgeDetection(spi_n) == sl_second_edge)
            && first_bit_sent_sl == 1)
            {
                from_second_bit_sl = 1;
                if(bin_data[abs(bin_index_sl - sl_index_first_bit)] == 0x30)
                {
                    SoftSPI_Slave_SendMISO(spi_n, CLEAR);
                }
                else if(bin_data[abs(bin_index_sl - sl_index_first_bit)] == 0x31)
                {
                    SoftSPI_Slave_SendMISO(spi_n, SET);
                }
            }


            if(from_second_bit_sl == 1)
            {
                if(GetFlagTimerSystick100ms())
                {
                    sl_index_recevier++;
                    bin_index_sl++;
                    if(bin_index_sl > 8)
                    {
                        bin_index_sl = 1;
                        first_bit_sent_sl = 0;
                        from_second_bit_sl = 0;
                        bReturn = true;
                    }
                }
            }
            else
            {
                if(GetFlagTimerSystick100ms())
                    sl_index_recevier++;
            }

            // Convert RX_data to Hex and send to USART
            if(sl_index_recevier >= 8)
            {
                // USART_SendData(usart_spi_sl_2, &hex_str, strlen(hex_str));
                USART_SendData(usart_spi_sl_2, " ", 1);
                sl_index_recevier = 0;
            }


        }

    }
	return bReturn;
}


/**
*******************************************************************************
* @ Name : SoftSPI_Slave_Configuration
* @ Parameters: uint8_t spin_n, uint8_t sampling_type, uint8_t read_data_type
* @ Registers :
* @ Descriptions :
*		- Configuration for Soft SPI as Slave
*         + GPIO configuration for Slave
*         + Sampling type (CPOL, CPHA): SAMPLING_TYPE_1_EDGE_RISING/SAMPLING_TYPE_1_EDGE_FALLING/
                                        SAMPLING_TYPE_2_EDGE_RISING/SAMPLING_TYPE_2_EDGE_FALLING
*         + Read data type (MSB or LSB): MSB_FIRST/LSB_FIRST
* @ Return value : bool
* @ author : Nguyen Dinh Thuan(thuan.nd.167@gmail.com)
* @ date : 2025-03-18
*******************************************************************************
*/
void SoftSPI_Slave_Configuration(uint8_t spin_n, uint8_t sampling_type, uint8_t read_data_type)
{
// GPIO_configuration
    SoftSPI_Slave_GPIO_Configuration(spin_n);

    if(sampling_type == SAMPLING_TYPE_1_EDGE_RISING)
    {
        sl_first_edge = RISING_EDGE;
        sl_second_edge = FALLING_EDGE;
    }
    else if(sampling_type == SAMPLING_TYPE_1_EDGE_FALLING)
    {
        sl_first_edge = FALLING_EDGE;
        sl_second_edge = RISING_EDGE;
    }
    else if (sampling_type == SAMPLING_TYPE_2_EDGE_RISING)
    {
        sl_sampling_second_edge = true;
        sl_first_edge = RISING_EDGE;
        sl_second_edge = FALLING_EDGE;
    }
    else if(sampling_type == SAMPLING_TYPE_2_EDGE_FALLING)
    {
        sl_sampling_second_edge = true;
        sl_first_edge = FALLING_EDGE;
        sl_second_edge = RISING_EDGE;
    }


    if(read_data_type == MSB_FIRST)
        sl_index_first_bit = 0;
    else if(read_data_type == LSB_FIRST)
        sl_index_first_bit = 7;
}



