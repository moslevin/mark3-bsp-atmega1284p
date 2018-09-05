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
#include "drvUARTplus.h"
#include "driver.h"
#include "thread.h"
#include "notify.h"
#include "streamer.h"
#include "threadport.h"
#include "kerneltimer.h"

#include <avr/io.h>
#include <avr/interrupt.h>

namespace Mark3
{
//---------------------------------------------------------------------------
static ATMegaUARTPlus* pclActive0; // Pointer to the active object
static ATMegaUARTPlus* pclActive1; // Pointer to the active object

//---------------------------------------------------------------------------
void ATMegaUARTPlus::SetBaud(uint32_t u32Baud_)
{
    uint16_t u16BaudTemp;
    uint16_t u16PortTemp;

    // Calculate the baud rate from the value in the driver.
    u16BaudTemp = (uint16_t)(((PORT_SYSTEM_FREQ / 16) / u32Baud_) - 1);

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
int ATMegaUARTPlus::Init(void)
{
    SetBaud(UART_DEFAULT_BAUD);

    m_clNotifyIn.Init();
    m_clNotifyOut.Init();

    m_clTimerIn.Init();

    m_bBlocking = true;
    m_bStartTx  = true;
    return 0;
}

//---------------------------------------------------------------------------
int ATMegaUARTPlus::Open()
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
int ATMegaUARTPlus::Close(void)
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
int ATMegaUARTPlus::Control(uint16_t u16CmdId_, void* pvIn_, size_t uSizeIn_, const void* pvOut_, size_t uSizeOut_)
{
    switch (static_cast<UartOpcode_t>(u16CmdId_)) {
        case UART_OPCODE_SET_BAUDRATE: {
            uint32_t u32BaudRate = *((uint32_t*)pvIn_);
            SetBaud(u32BaudRate);
        } break;
        case UART_OPCODE_SET_BUFFERS: {
            uint8_t* pau8In  = (uint8_t*)pvIn_;
            uint8_t* pau8Out = (uint8_t*)pvOut_;
            m_clStreamIn.Init(pau8In, uSizeIn_);
            m_clStreamOut.Init(pau8Out, uSizeOut_);
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
        case UART_OPCODE_SET_BLOCKING: {
            m_bBlocking = true;
        } break;
        case UART_OPCODE_SET_NONBLOCKING: {
            m_bBlocking = false;
        } break;
        default: break;
    }
    return 0;
}

//---------------------------------------------------------------------------
size_t ATMegaUARTPlus::Read(void* pvData_, size_t uSizeIn_)
{
    auto*  pu8Data = reinterpret_cast<uint8_t*>(pvData_);
    size_t uRead   = 0;
    while (uSizeIn_) {
        if (m_clStreamIn.Read(pu8Data)) {
            pu8Data++;
            uSizeIn_--;
            uRead++;
        } else {
            if (m_bBlocking) {
                m_clNotifyIn.Wait(0);
            } else {
                return uRead;
            }
        }
    }
    return uRead;
}

//---------------------------------------------------------------------------
bool ATMegaUARTPlus::StreamOutByte(uint8_t u8In_)
{
    if (!m_clStreamOut.CanWrite()) {
        return false;
    }
    bool bStart = false;
    if (m_clStreamOut.IsEmpty() && m_bStartTx) {
        bStart     = true;
        m_bStartTx = false;
    }
    m_clStreamOut.Write(u8In_);
    if (bStart) {
        StartTx();
    }
    return true;
}

//---------------------------------------------------------------------------
size_t ATMegaUARTPlus::Write(const void* pvData_, size_t uSizeOut_)
{
    auto*  pu8Data  = reinterpret_cast<const uint8_t*>(pvData_);
    size_t uWritten = 0;
    while (uSizeOut_) {
        if (StreamOutByte(*pu8Data)) {
            pu8Data++;
            uSizeOut_--;
            uWritten++;
        } else {
            if (m_bBlocking) {
                m_clNotifyOut.Wait(0);
            } else {
                return uWritten;
            }
        }
    }
    return uWritten;
}

//---------------------------------------------------------------------------
void ATMegaUARTPlus::StartTx()
{
    CS_ENTER();
    uint8_t u8Byte;
    if (m_clStreamOut.Read(&u8Byte)) {
        if (m_u8Identity == 0) {
            UART0_UDR = u8Byte;
        } else {
            UART1_UDR = u8Byte;
        }
    }
    CS_EXIT();
}

//---------------------------------------------------------------------------
void ATMegaUARTPlus::StreamInCallback()
{
    CS_ENTER();
    if (m_clStreamIn.IsEmpty()) {
        m_clTimerIn.Stop();
    }
    CS_EXIT();
    m_clNotifyIn.Signal();
}

//---------------------------------------------------------------------------
static void StreamTimerCallback(Thread* pclOwner_, void* pvData_)
{
    auto* pclThis = reinterpret_cast<ATMegaUARTPlus*>(pvData_);
    pclThis->StreamInCallback();
}

//---------------------------------------------------------------------------
void ATMegaUARTPlus::RxISR()
{
    uint8_t u8Byte;
    if (m_u8Identity == 0) {
        u8Byte = UART0_UDR;
    } else {
        u8Byte = UART1_UDR;
    }

    if (!m_clStreamIn.CanWrite()) {
        return;
    }

    bool bStart = false;
    if (m_clStreamIn.IsEmpty()) {
        bStart = true;
    }
    m_clStreamIn.Write(u8Byte);
    if (bStart) {
        m_clTimerIn.Start(true, 50, StreamTimerCallback, (void*)this);
    }
    if ((u8Byte == 0) || (u8Byte == '\n')) {
        m_clNotifyIn.Signal();
    }
}

//---------------------------------------------------------------------------
void ATMegaUARTPlus::TxISR()
{
    if (!m_clStreamOut.IsEmpty() && !m_bStartTx) {
        StartTx();
    } else {
        m_bStartTx = true;
        m_clNotifyOut.Signal();
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
ISR(UART0_TX_ISR)
{
    pclActive0->TxISR();
}
ISR(UART1_TX_ISR)
{
    pclActive1->TxISR();
}

} // namespace Mark3
