/**
 * See the file LICENSE for redistribution information.
 *
 * Copyright (c) 2009-2017 ObdDiag.Net. All rights reserved.
 *
 */

#include <cstring>
#include "cortexm.h"
#include "EcuUart.h"
#include "GpioDrv.h"	

#define USARTy	USART1

using namespace std;

const int TxPin  = 9;
const int RxPin  = 10;
const int USER_AF = GPIO_AF_1;
#define USARTy_IRQn USART1_IRQn
#define RxPort      GPIOA
#define TxPort      GPIOA

///const uint32_t PinAssign = ((RxPin << 16) + (RxPort * 32)) | ((TxPin << 8)  + (TxPort * 32));

//#define INVERT_OUTPUT // Invert output for simple transistor-based K-line driver
//#define OPEN_DRAIN    // We might have it for Dev board to be totally input level compiant.

/**
 * EcuUart singleton
 * @return The pointer to EcuUart instance
 */
EcuUart* EcuUart::instance()
{
    static EcuUart instance;
    return &instance;;
}

/**
 * Configure UART1
 */
void EcuUart::configure()
{
#ifndef INVERT_OUTPUT
  
    GPIOPinConfig( 0  , RxPin, 0);
#else    
    GPIOPinConfig(RxPort, RxPin, GPIO_HYSTERESIS);
#endif
    
#ifdef OPEN_DRAIN
    GPIOPinConfig( 0 , TxPin, GPIO_OPEN_DRAIN);
#else
    GPIOPinConfig( 0 , TxPin, 0);
#endif

    // GPIO configuration for USART2 signals
    // Select AF mode on Tx/Rx pins
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = (1 << TxPin) | (1 << RxPin);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Connect TxPin to USART_Tx, RxPin to USART_Rx, use alternate function AF1, p165
    GPIO_PinAFConfig(TxPort, TxPin, USER_AF);
    GPIO_PinAFConfig(RxPort, RxPin, USER_AF);
}

/**
 * Use UART ROM API to configuring speed and interrupt for UART1,
 * discard the allocated UART memory block afterwards
 * @parameter[in] speed EcuUart speed
 */
void EcuUart::init(uint32_t speed)
{
    setBitBang(false);
    
    const int UART_MEM_LEN = 40;

    // Allocate UART API block
    uint8_t uartMem[UART_MEM_LEN];

    USART_InitTypeDef USART_InitStruct;
    USART_InitStruct.USART_BaudRate = speed;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USARTy, &USART_InitStruct);

    // Enable USART
    USARTy->CR1 |= USART_CR1_UE; 

    // Enable the USART Receive interrupt
    USART_ITConfig(USARTy, USART_IT_RXNE, ENABLE);
    
    // Clear TC flag 
    USARTy->ICR |= USART_ICR_TCCF;

    NVIC_SetPriority(USARTy_IRQn, 1);
    NVIC_EnableIRQ(USARTy_IRQn);
    
    // Invert output for simple transistor-based K-line driver
#ifdef INVERT_OUTPUT
    USART_InvPinCmd( USARTy , TxPin, ENABLE );
#endif
}

/**
 * Send byte, blocking call pending on UART1 send ready status
 * @parameter[in] byte Byte to sent
 */
void EcuUart::send(uint8_t byte)
{
    while ((USARTy->ISR & USART_FLAG_TXE) == 0 )
        ;
    USART_SendData( USARTy, byte );
}

/**
 * Checking EcuUart receive ready flag
 * @return The flag
 */
bool EcuUart::ready()
{
    return ( USARTy->ISR & USART_FLAG_TXE );
}

/*
 * Reading a byte from USART
 * @return The byte received
 */
uint8_t EcuUart::get()
{
    return USART_ReceiveData( USARTy );
}

/**
 * As USART TX and RX pins are interconnected thru MC33660, sending a byte will always echo it back
 * just wait and read it back.
 * @parameter[in] byte The already sent byte to compare echo with
 * @return The completion status
 */
bool EcuUart::getEcho(uint8_t byte)
{
    const uint32_t EchoTimeout = 20; // Using 20ms echo timeout

    // Use the SysTick to generate the timeout
    SysTick->LOAD = EchoTimeout * (SystemCoreClock / 1000);
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (!ready()) {
        if (SysTick->CTRL & 0x10000) // check timeout
            return false;
    }
    uint8_t echo = get();
    return (echo == byte);
}

/*
 * Turn on/off bing bang-mode for ISO initialization
 * @parameter[in] val Bing-bang mode flag 
 */
void EcuUart::setBitBang(bool val)
{

}

/*
 * Set the USART TX pin status
 * @parameter[in] bit USART TX pin value
 */
void EcuUart::setBit(uint32_t bit)
{
#ifndef INVERT_OUTPUT
    //GPIO_WriteBit( TxPort , TxPin ,  bit );
#else
    // Invert output for simple transistor-based K-line driver
    GPIO_WriteBit( TxPort , TxPin , (bit ? 0 : 1)); 
#endif
}

/**
 * Read USART RX pin status
 * @return pin status, 1 if set, 0 otherwise
 */
uint32_t EcuUart::getBit()
{
    return GPIO_ReadInputDataBit(RxPort, RxPin);
}

/**
 * Clear Framing/Parity errors, if any, by reading RXDATA
 */
void EcuUart::clear()
{
    // Clear FRAMERR and PARITYERR by reading RXDATA
    if ( USART_GetFlagStatus && ( USART_FLAG_FE | USART_FLAG_PE )) {
        USART_ClearFlag( USARTy , ( USART_FLAG_FE | USART_FLAG_PE ));
        USART_ReceiveData( USARTy );
    }
}
