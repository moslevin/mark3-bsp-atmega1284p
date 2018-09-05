/*===========================================================================
     _____        _____        _____        _____
 ___|    _|__  __|_    |__  __|__   |__  __| __  |__  ______
|    \  /  | ||    \      ||     |     ||  |/ /     ||___   |
|     \/   | ||     \     ||     \     ||     \     ||___   |
|__/\__/|__|_||__|\__\  __||__|\__\  __||__|\__\  __||______|
    |_____|      |_____|      |_____|      |_____|

--[Mark3 Realtime Platform]--------------------------------------------------

Copyright (c) 2012 - 2018 m0slevin, all rights reserved.
See license.txt for more information
=========================================================================== */
/**

    @file   drvUART.h

    @brief  Atmega328p serial port driver

 */
#pragma once

#include "kerneltypes.h"
#include "driver.h"
#include "drvUART.h"

//---------------------------------------------------------------------------
// UART defines - user-configurable for different targets
//---------------------------------------------------------------------------
#define UART0_SRA (UCSR0A)
#define UART0_SRB (UCSR0B)
#define UART0_SRC (UCSR0C)
#define UART0_BAUDH (UBRR0H)
#define UART0_BAUDL (UBRR0L)
#define UART0_RXEN (RXEN0)
#define UART0_TXEN (TXEN0)
#define UART0_TXCIE (TXCIE0)
#define UART0_RXCIE (RXCIE0)
#define UART0_UDR (UDR0)
#define UART0_UDRE (UDRE0)
#define UART0_RXC (RXC0)

#define UART0_RX_ISR (USART0_RX_vect)
#define UART0_TX_ISR (USART0_TX_vect)

//---------------------------------------------------------------------------
#define UART1_SRA (UCSR1A)
#define UART1_SRB (UCSR1B)
#define UART1_SRC (UCSR1C)
#define UART1_BAUDH (UBRR1H)
#define UART1_BAUDL (UBRR1L)
#define UART1_RXEN (RXEN1)
#define UART1_TXEN (TXEN1)
#define UART1_TXCIE (TXCIE1)
#define UART1_RXCIE (RXCIE1)
#define UART1_UDR (UDR1)
#define UART1_UDRE (UDRE1)
#define UART1_RXC (RXC1)

#define UART1_RX_ISR (USART1_RX_vect)
#define UART1_TX_ISR (USART1_TX_vect)

#define UART_DEFAULT_BAUD ((uint32_t)57600)

//---------------------------------------------------------------------------

namespace Mark3
{
class ATMegaUART;
//---------------------------------------------------------------------------
typedef struct _UartData_t UartData_t;

//---------------------------------------------------------------------------
/**
 *   Implements a UART driver on the ATMega328p
 */
class ATMegaUART : public UartDriver
{
public:
    virtual int    Init();
    virtual int    Open();
    virtual int    Close();
    virtual size_t Read(void* pu8Data_, size_t uBytes_);
    virtual size_t Write(const void* pu8Data_, size_t uBytes_);
    virtual int
    Control(uint16_t u16EventID_, void* pvDataIn_, size_t uSizeIn_, const void* pvDataOut_, size_t uSizeOut_);

    /**
     *  Called from the transmit complete ISR - implements a
     *  callback/transmit state-machine
     */
    void TxISR();

    /**
     *  Called from the receive-complete ISR - implements a
     *  callback/receive state-machine
     */
    void RxISR();

    /**
     *  @brief GetRxBuffer
     *
     *  Return a pointer to the receive buffer for this UART.
     *
     *  @return pointer to the driver's RX buffer
     */
    uint8_t* GetRxBuffer(void) { return m_pu8RxBuffer; }
    /**
     *  @brief GetTxBuffer
     *
     *  Return a pointer to the transmit buffer for this UART.
     *
     *  @return pointer to the driver's TX buffer
     */
    uint8_t* GetTxBuffer(void) { return m_pu8TxBuffer; }

private:
    void SetBaud(void);
    void StartTx(void);

    size_t m_uTxSize; //!< Size of the TX Buffer
    size_t m_uTxHead; //!< Head index
    size_t m_uTxTail; //!< Tail index

    size_t m_uRxSize; //!< Size of the RX Buffer
    size_t m_uRxHead; //!< Head index
    size_t m_uRxTail; //!< Tail index

    uint8_t* m_pu8RxBuffer; //!< Receive buffer pointer
    uint8_t* m_pu8TxBuffer; //!< Transmit buffer pointer

    uint32_t m_u32BaudRate; //!< Baud rate
    bool     m_bRxOverflow; //!< indicates received overflow
    uint8_t  m_u8Identity;  //!< port number
};

} // namespace Mark3
