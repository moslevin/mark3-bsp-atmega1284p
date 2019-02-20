#include "mark3.h"
#include "memutil.h"
#include "drvUART.h"
#include "kernelaware.h"
#include "drvATMegaUART.h"

#include <avr/io.h>
#include <avr/sleep.h>

//---------------------------------------------------------------------------
#define UART_SIZE_RX (12) //!< UART RX Buffer size
#define UART_SIZE_TX (12) //!< UART TX Buffer size

using namespace Mark3;

static uint8_t aucRxBuffer[UART_SIZE_RX];
static uint8_t aucTxBuffer[UART_SIZE_TX];

static ATMegaUART clUART; //!< UART device driver object

namespace Mark3
{
namespace UnitTestSupport
{
    void OnInit()
    {
        clUART.SetName("/dev/tty"); //!< Add the serial driver
        clUART.Init();

        DriverList::Add(&clUART);
    }

    void OnStart()
    {
        clUART.SetBuffers(aucRxBuffer, UART_SIZE_RX, aucTxBuffer, UART_SIZE_TX);
        clUART.Open();
    }

    void OnIdle()
    {
        // stub
    }

    void OnExit(int /*rc*/) { KernelAware::ExitSimulator(); }

} // namespace UnitTest
} // namespace Mark3
