/**
 * @file USART.h
 * @author Nguyen Dinh Thuan (thuan.nd.167@gmail.com)
 * @brief Some declaration for USART(Universal Synchronous/Asynchronous Receiver Transmiter) of STM32F407VGTx (ARMCortex M4)
 * @date 2024-09-12
 * 
 */

/******************************************************************************
* Include Files
******************************************************************************/

#include <stdio.h>
#include <stdint.h>

/*******************************************************************************
* GPIO ADDRESS DEFINITION
*******************************************************************************/

// UART
#define ADDRESS_UART_4      0x40004C00
#define ADDRESS_UART_5      0x40005000

//USART
#define ADDRESS_USART_1     0x40011000
#define ADDRESS_USART_2     0x40004400
#define ADDRESS_USART_3     0x40004800
#define ADDRESS_USART_6     0x40011400

/*******************************************************************************
* USART REGISTER STRUCTURE DEFINITION
*******************************************************************************/

typedef struct {
    uint32_t SR;       // Status Register
    uint32_t DR;       // Data Register
    uint32_t BRR;      // Baud Rate Register
    uint32_t CR1;      // Control Register 1
    uint32_t CR2;      // Control Register 2
    uint32_t CR3;      // Control Register 3
    uint32_t GTPR;     // Guard Time and Prescaler Register
} USARTn;

extern volatile uint8_t value_re_IT;

/*******************************************************************************
* COMMON DEFINITION
*******************************************************************************/

#define ENABLE  1
#define DISABLE 0

/*******************************************************************************
* SR REGISTERS DEFINITION
*******************************************************************************/

#define DATA_RECEIVED       1
#define DATA_NOT_RECEIVED   0
#define USART_CLEAR_READDATA_FLAG(usartn)       (usartn->SR) &= ~(1 << 5)
#define USART_GET_READDATA_STATUS(usart)        (usart->SR & (1 << 5))


#define DATA_TRANSMITTED    1
#define DATA_NOT_TRANSMITTED 0
#define USART_CLEAR_WRITEDATA_FLAG(usartn)       (usartn->SR) &= ~(1 << 7)
#define USART_GET_WRITEDATA_STATUS(usart)       (usart->SR & (1 << 7))

/*******************************************************************************
* DR REGISTERS DEFINITION
*******************************************************************************/

#define USART_DATA_VALUE(usart, value)               usart->DR = value
#define USART_GET_DATA_VALUE(usart)                 usart->DR

/*******************************************************************************
* BRR REGISTERS DEFINITION
*******************************************************************************/

#define BAUDRATE_9600       (uint32_t)9600
#define BAUDRATE_19200      (uint32_t)19200
#define BAUDRATE_115200     (uint32_t)115200

#define USART_SET_BAUDRATE(usart, baudrate)         usart->BRR = baudrate

/*******************************************************************************
* CR1 REGISTERS DEFINITION
*******************************************************************************/

#define OVERSAMPLING_BY_16  0
#define OVERSAMPLING_BY_8   1   // Not for Smart card mode, IrDA and LIN mode
#define USART_SET_OVERSAMPLING_MODE(usart, over8mode)   usart->CR1 |= over8mode << 15

#define USART_ENABLE_USART(usart)       usart->CR1 |= ENABLE << 13
#define USART_DISABLE_USART(usart)      usart->CR1 &=~ ENABLE << 13

#define DATA_BITS_8          0
#define DATA_BITS_9          1
#define USART_SET_WORD_LENGTH(usart, wordlength)    usart->CR1 |= wordlength << 12

#define PARITY_DISABLE        0
#define PARITY_ENABLE         1
#define USART_SET_PARITY_CONTROL(usart, paritycontrol)  usart->CR1 |= paritycontrol << 10

#define EVEN_PARITY           0
#define ODD_PARITY            1
#define USART_SET_PARITY_TYPE(usart, paritytype)    usart->CR1 |= paritytype << 9


#define USART_TRANSMITER_ENABLE(usart)    usart->CR1 |= ENABLE << 3
#define USART_RECEIVER_ENABLE(usart)       usart->CR1 |= ENABLE << 2

#define USART_ENABLE_RX_INTERRUPT(usart)    usart->CR1 |= ENABLE << 5
#define USART_ENABLE_TX_INTERRUPT(usart)    usart->CR1 |= ENABLE << 7

/*******************************************************************************
* CR2 REGISTERS DEFINITION
*******************************************************************************/

#define STOP_BIT_1      0       // Default for UART
#define STOP_BIT_05     1       // Using when receive data from SmartCard
#define STOP_BIT_2      2       // Using for normal USART, single-wire and modem mode
#define STOP_BIT_15     3       // Using when transmit/receive data in SmartCard mode

#define USART_SET_STOP_BIT(usart, stopbit)      usart->CR2 |= (stopbit << 12)


/*******************************************************************************
* FUNCTIONS DEFINITION
*******************************************************************************/

uint16_t USART_BaudrateCalculator(uint32_t baudrate, float clockfrequency, uint8_t over8);
void USART_Configuration(USARTn* usartn, uint32_t baudrate, uint8_t wordlength, uint8_t over8, uint8_t paritycontrol, uint8_t paritytype);
void USART_SendData(USARTn *usartn,volatile uint8_t *pData, uint16_t Size);
unsigned char USART_ReceiveData(USARTn* usartn);

void USART_ReceiverDataInterruptEnable(USARTn* usartn);
void USART_TransmiterDataInterruptEnable(USARTn* usartn);
