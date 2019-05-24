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
===========================================================================*/
/**

    @file   drvUART.cpp

    @brief  Atmega328p serial port driver
*/

#include "kerneltypes.h"
#include "drvATMegaUART.h"
#include "driver.h"
#include "thread.h"
#include "threadport.h"
#include "kerneltimer.h"

#include <avr/io.h>
#include <avr/interrupt.h>

namespace Mark3
{
//---------------------------------------------------------------------------
static ATMegaUART* pclActive0; // Pointer to the active object
static ATMegaUART* pclActive1; // Pointer to the active object

//---------------------------------------------------------------------------
void ATMegaUART::SetBaud(void)
{
    uint16_t u16BaudTemp;
    uint16_t u16PortTemp;

    // Calculate the baud rate from the value in the driver.
    u16BaudTemp = (uint16_t)(((PORT_SYSTEM_FREQ / 16) / m_u32BaudRate) - 1);

    if (m_u8Identity == 0) {
        // Save the current port config registers
        u16PortTemp = UART0_SRB;

        // Clear the registers (disable rx/tx/interrupts)
        UART0_SRB = 0;
        UART0_SRA = 0;

        // Set the baud rate high/low values.
        UART0_BAUDH = (uint8_t)(u16BaudTemp >> 8);
        UART0_BAUDL = (uint8_t)(u16BaudTemp & 0x00FF);

        // Restore the Rx/Tx flags to their previous state
        UART0_SRB = u16PortTemp;
    } else {
        // Save the current port config registers
        u16PortTemp = UART1_SRB;

        // Clear the registers (disable rx/tx/interrupts)
        UART1_SRB = 0;
        UART1_SRA = 0;

        // Set the baud rate high/low values.
        UART1_BAUDH = (uint8_t)(u16BaudTemp >> 8);
        UART1_BAUDL = (uint8_t)(u16BaudTemp & 0x00FF);

        // Restore the Rx/Tx flags to their previous state
        UART1_SRB = u16PortTemp;
    }
}

//---------------------------------------------------------------------------
int ATMegaUART::Init(void)
{
    // Set up the FIFOs
    m_uTxHead     = 0;
    m_uTxTail     = 0;
    m_uRxHead     = 0;
    m_uRxTail     = 0;
    m_bRxOverflow = 0;
    m_u32BaudRate = UART_DEFAULT_BAUD;

    // Clear flags

    SetBaud();
    return 0;
}

//---------------------------------------------------------------------------
int ATMegaUART::Open()
{
    // Enable Rx/Tx + Interrupts
    if (m_u8Identity == 0) {
        UART0_SRB |= (1 << UART0_RXEN) | (1 << UART0_TXEN);
        UART0_SRB |= (1 << UART0_RXCIE) | (1 << UART0_TXCIE);
        // Set frame format: 8 N 1
        UART0_SRC  = 0x06;
        pclActive0 = this;
    } else {
        UART1_SRB |= (1 << UART1_RXEN) | (1 << UART1_TXEN);
        UART1_SRB |= (1 << UART1_RXCIE) | (1 << UART1_TXCIE);
        // Set frame format: 8 N 1
        UART1_SRC  = 0x06;
        pclActive1 = this;
    }

    return 0;
}

//---------------------------------------------------------------------------
int ATMegaUART::Close(void)
{
    // Disable Rx/Tx + Interrupts
    if (m_u8Identity == 0) {
        UART0_SRB &= ~((1 << UART0_RXEN) | (1 << UART0_TXEN));
        UART0_SRB &= ~((1 << UART0_TXCIE) | (1 << UART0_RXCIE));
    } else {
        UART1_SRB &= ~((1 << UART1_RXEN) | (1 << UART1_TXEN));
        UART1_SRB &= ~((1 << UART1_TXCIE) | (1 << UART1_RXCIE));
    }
    return 0;
}

//---------------------------------------------------------------------------
int ATMegaUART::Control(uint16_t u16CmdId_, void* pvIn_, size_t uSizeIn_, const void* pvOut_, size_t uSizeOut_)
{
    switch (static_cast<UartOpcode_t>(u16CmdId_)) {
        case UART_OPCODE_SET_BAUDRATE: {
            uint32_t u32BaudRate = *((uint32_t*)pvIn_);
            m_u32BaudRate        = u32BaudRate;
            SetBaud();
        } break;
        case UART_OPCODE_SET_BUFFERS: {
            m_pu8RxBuffer = (uint8_t*)pvIn_;
            m_pu8TxBuffer = (uint8_t*)pvOut_;
            m_uRxSize     = uSizeIn_;
            m_uTxSize     = uSizeOut_;
        } break;
        case UART_OPCODE_SET_RX_ENABLE: {
            if (m_u8Identity == 0) {
                UART0_SRB |= (1 << UART0_RXEN);
            } else {
                UART1_SRB |= (1 << UART1_RXEN);
            }
        } break;
        case UART_OPCODE_SET_RX_DISABLE: {
            if (m_u8Identity == 0) {
                UART0_SRB &= ~(1 << UART0_RXEN);
            } else {
                UART1_SRB &= ~(1 << UART1_RXEN);
            }
        } break;
        case UART_OPCODE_SET_IDENTITY: {
            m_u8Identity = *((uint8_t*)pvIn_);
        } break;
        default: break;
    }
    return 0;
}

//---------------------------------------------------------------------------
size_t ATMegaUART::Read(void* pvData_, size_t uBytes_)
{
    // Read a string of characters of length N.  Return the number of bytes
    // actually read.  If less than the 1 length, this indicates that
    // the buffer is full and that the app needs to wait.

    size_t uRead   = 0;
    bool   bExit   = 0;
    auto*  pu8Data = reinterpret_cast<uint8_t*>(pvData_);

    for (size_t i = 0; i < uBytes_; i++) {
        // If Tail != Head, there's data in the buffer.
        CriticalSection::Enter();
        if (m_uRxTail != m_uRxHead) {
            // We have room to add the byte, so add it.
            pu8Data[i] = m_pu8RxBuffer[m_uRxTail];

            // Update the buffer head pointer.
            m_uRxTail++;
            if (m_uRxTail >= m_uRxSize) {
                m_uRxTail = 0;
            }
            uRead++;
        } else {
            // Can't do anything else - the buffer is empty
            bExit = 1;
        }
        CriticalSection::Exit();

        // If we have to bail because the buffer is empty, do it now.
        if (bExit == 1) {
            break;
        }
    }
    return uRead;
}

//---------------------------------------------------------------------------
size_t ATMegaUART::Write(const void* pvData_, size_t uSizeOut_)
{
    // Write a string of characters of length N.  Return the number of bytes
    // actually written.  If less than the 1 length, this indicates that
    // the buffer is full and that the app needs to wait.
    size_t uWritten = 0;
    size_t uNext;
    bool   bActivate = 0;
    bool   bExit     = 0;
    auto*  pu8Data   = reinterpret_cast<const uint8_t*>(pvData_);

    // If the head = tail, we need to start sending data out the data ourselves.
    if (m_uTxHead == m_uTxTail) {
        bActivate = 1;
    }

    for (size_t i = 0; i < uSizeOut_; i++) {
        CriticalSection::Enter();
        // Check that head != tail (we have room)
        uNext = (m_uTxHead + 1);
        if (uNext >= m_uTxSize) {
            uNext = 0;
        }

        if (uNext != m_uTxTail) {
            // We have room to add the byte, so add it.
            m_pu8TxBuffer[m_uTxHead] = pu8Data[i];

            // Update the buffer head pointer.
            m_uTxHead = uNext;
            uWritten++;
        } else {
            // Can't do anything - buffer is full
            bExit = 1;
        }
        CriticalSection::Exit();

        // bail if the buffer is full
        if (bExit == 1) {
            break;
        }
    }

    // Activate the transmission if we're currently idle
    if (bActivate == 1) {
        // We know we're idle if we get here.
        CriticalSection::Enter();
        if (m_u8Identity == 0) {
            if (UART0_SRA & (1 << UDRE0)) {
                StartTx();
            }
        } else {
            if (UART1_SRA & (1 << UDRE0)) {
                StartTx();
            }
        }

        CriticalSection::Exit();
    }

    return uWritten;
}

//---------------------------------------------------------------------------
void ATMegaUART::StartTx(void)
{
    // Send the tail byte out.
    uint8_t u8Next;

    CriticalSection::Enter();

    // Send the byte at the tail index
    if (m_u8Identity == 0) {
        UART0_UDR = m_pu8TxBuffer[m_uTxTail];
    } else {
        UART1_UDR = m_pu8TxBuffer[m_uTxTail];
    }

    // Update the tail index
    u8Next = (m_uTxTail + 1);
    if (u8Next >= m_uTxSize) {
        u8Next = 0;
    }
    m_uTxTail = u8Next;

    CriticalSection::Exit();
}

//---------------------------------------------------------------------------
void ATMegaUART::RxISR()
{
    uint8_t u8Temp;
    uint8_t u8Next;

    // Read the byte from the data buffer register
    if (m_u8Identity == 0) {
        u8Temp = UART0_UDR;
    } else {
        u8Temp = UART1_UDR;
    }

    // Check that head != tail (we have room)
    u8Next = (m_uRxHead + 1);
    if (u8Next >= m_uRxSize) {
        u8Next = 0;
    }

    // Always add the byte to the buffer (but flag an error if it's full...)
    m_pu8RxBuffer[m_uRxHead] = u8Temp;

    // Update the buffer head pointer.
    m_uRxHead = u8Next;

    // If the buffer's full, discard the oldest byte in the buffer and flag an error
    if (u8Next == m_uRxTail) {
        // Update the buffer tail pointer
        m_uRxTail = (m_uRxTail + 1);
        if (m_uRxTail >= m_uRxSize) {
            m_uRxTail = 0;
        }

        // Flag an error - the buffer is full
        m_bRxOverflow = 1;
    }
}

//---------------------------------------------------------------------------
ISR(UART0_RX_ISR)
{
    pclActive0->RxISR();
}
ISR(UART1_RX_ISR)
{
    pclActive1->RxISR();
}

//---------------------------------------------------------------------------
void ATMegaUART::TxISR()
{
    // If the head != tail, there's something to send.
    if (m_uTxHead != m_uTxTail) {
        StartTx();
    }
}

//---------------------------------------------------------------------------
ISR(UART0_TX_ISR)
{
    pclActive0->TxISR();
}
ISR(UART1_TX_ISR)
{
    pclActive1->TxISR();
}

} // namespace Mark3
