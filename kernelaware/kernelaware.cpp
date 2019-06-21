/*===========================================================================
     _____        _____        _____        _____
 ___|    _|__  __|_    |__  __|__   |__  __| __  |__  ______
|    \  /  | ||    \      ||     |     ||  |/ /     ||___   |
|     \/   | ||     \     ||     \     ||     \     ||___   |
|__/\__/|__|_||__|\__\  __||__|\__\  __||__|\__\  __||______|
    |_____|      |_____|      |_____|      |_____|

--[Mark3 Realtime Platform]--------------------------------------------------

Copyright (c) 2012 - 2019 m0slevin, all rights reserved.
See license.txt for more information
===========================================================================*/
/**

    @file   kernelaware.cpp

    @brief  Kernel aware simulation support
*/

#include "kerneltypes.h"
#include "mark3cfg.h"
#include "kernelaware.h"
#include "threadport.h"
#include "criticalsection.h"
#include "criticalguard.h"
//---------------------------------------------------------------------------
/**
 *  This enumeration contains a list of supported commands that can be
 *  executed to invoke a response from a kernel aware host.
 */
typedef enum {
    KA_COMMAND_IDLE = 0,       //!< Null command, does nothing.
    KA_COMMAND_PROFILE_INIT,   //!< Initialize a new profiling session
    KA_COMMAND_PROFILE_START,  //!< Begin a profiling sample
    KA_COMMAND_PROFILE_STOP,   //!< End a profiling sample
    KA_COMMAND_PROFILE_REPORT, //!< Report current profiling session
    KA_COMMAND_EXIT_SIMULATOR, //!< Terminate the host simulator
    KA_COMMAND_TRACE_0,        //!< 0-argument kernel trace
    KA_COMMAND_TRACE_1,        //!< 1-argument kernel trace
    KA_COMMAND_TRACE_2,        //!< 2-argument kernel trace
    KA_COMMAND_PRINT,          //!< Print an arbitrary string of data
    KA_COMMAND_OPEN,           //!< Open a path on the host device filesystem
    KA_COMMAND_CLOSE,          //!< Close a previously opened file on the host
    KA_COMMAND_READ,           //!< Read data from an open file in the host
    KA_COMMAND_WRITE,          //!< Write data to a file on the host
    KA_COMMAND_BLOCKING,       //!< Set blocking/non-blocking mode for an open host file
} KernelAwareCommand_t;

//---------------------------------------------------------------------------
/**
    This structure is used to communicate between the kernel and a kernel-
    aware host.  Its data contents is interpreted differently depending on
    the command executed (by means of setting the g_u8KACommand variable, as
    is done in the command handlers in this module).  As a result, any changes
    to this struct by way of modifying or adding data must be mirrored in the
    kernel-aware simulator.
*/
typedef union {
    volatile uint16_t au16Buffer[5]; //!< Raw binary contents of the struct
                                     /**
                                      * @brief The Profiler struct contains data related to the code-execution
                                      *        profiling functionality provided by a kernel-aware host simluator.
                                      */
    struct {
        volatile const char* szName; //!< Name of the profiling data to report
    } Profiler;
    /**
     * @brief The Trace struct contains data related to the display and output
     *        of kernel-trace strings on a kernel-aware host.
     */
    struct {
        volatile uint16_t u16File; //!< File index
        volatile uint16_t u16Line; //!< Line number
        volatile uint16_t u16Arg1; //!< (optional) argument code
        volatile uint16_t u16Arg2; //!< (optional) argument code
    } Trace;
    /**
     * @brief The Print struct contains data related to the display of arbitrary
     *        null-terminated ASCII strings on the kernel-aware host.
     */
    struct {
        volatile const char* szString; //!< Pointer ot a string (in RAM) to print
    } Print;

    struct {
        volatile const char* szPath;
        volatile bool bRead;
        volatile bool bWrite;
        volatile bool bCreate;
        volatile bool bAppend;
        volatile bool bTruncate;
    } OpenRequest;

    struct {
        volatile int32_t iHostFd;
    } OpenReturn;

    struct {
        volatile int32_t iHostFd;
    } Close;

    struct {
        volatile int32_t iHostFd;
        volatile K_ADDR* paReadBuf;
        volatile K_WORD  u16BytesToRead;
    } Read;

    struct {
        volatile int32_t iBytesRead;
    } ReadReturn;

    struct {
        volatile int32_t iHostFd;
        volatile const K_ADDR* paWriteBuf;
        volatile K_WORD  u16BytesToWrite;
    } Write;

    struct {
        volatile int32_t iBytesWritten;
    } WriteReturn;

    struct {
        volatile int32_t iHostFd;
        volatile bool bBlocking;
    } Blocking;

} KernelAwareData_t;

//---------------------------------------------------------------------------
// Must not live in Mark3 namespace
using namespace Mark3;
volatile bool     g_bIsKernelAware; //!< Will be set to true by a kernel-aware host.
volatile uint8_t  g_u8KACommand;    //!< Kernel-aware simulator command to execute
KernelAwareData_t g_stKAData;       //!< Data structure used to communicate with host.

namespace Mark3
{
//---------------------------------------------------------------------------
void Trace_i(uint16_t u16File_, uint16_t u16Line_, uint16_t u16Arg1_, uint16_t u16Arg2_, KernelAwareCommand_t eCmd_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.Trace.u16File = u16File_;
    g_stKAData.Trace.u16Line = u16Line_;
    g_stKAData.Trace.u16Arg1 = u16Arg1_;
    g_stKAData.Trace.u16Arg2 = u16Arg2_;
    g_u8KACommand            = eCmd_;
}

//---------------------------------------------------------------------------
void KernelAware::ProfileInit(const char* szStr_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.Profiler.szName = szStr_;
    g_u8KACommand              = KA_COMMAND_PROFILE_INIT;
}

//---------------------------------------------------------------------------
void KernelAware::ProfileStart(void)
{
    const auto cg = CriticalGuard{};
    g_u8KACommand = KA_COMMAND_PROFILE_START;
}

//---------------------------------------------------------------------------
void KernelAware::ProfileStop(void)
{
    const auto cg = CriticalGuard{};
    g_u8KACommand = KA_COMMAND_PROFILE_STOP;
}

//---------------------------------------------------------------------------
void KernelAware::ProfileReport(void)
{
    const auto cg = CriticalGuard{};
    g_u8KACommand = KA_COMMAND_PROFILE_REPORT;
}

//---------------------------------------------------------------------------
void KernelAware::ExitSimulator(void)
{
    g_u8KACommand = KA_COMMAND_EXIT_SIMULATOR;
}

//---------------------------------------------------------------------------
void KernelAware::Trace(uint16_t u16File_, uint16_t u16Line_)
{
    const auto cg = CriticalGuard{};
    Trace_i(u16File_, u16Line_, 0, 0, KA_COMMAND_TRACE_0);
}

//---------------------------------------------------------------------------
void KernelAware::Trace(uint16_t u16File_, uint16_t u16Line_, uint16_t u16Arg1_)
{
    const auto cg = CriticalGuard{};
    Trace_i(u16File_, u16Line_, u16Arg1_, 0, KA_COMMAND_TRACE_1);
}
//---------------------------------------------------------------------------
void KernelAware::Trace(uint16_t u16File_, uint16_t u16Line_, uint16_t u16Arg1_, uint16_t u16Arg2_)
{
    const auto cg = CriticalGuard{};
    Trace_i(u16File_, u16Line_, u16Arg1_, u16Arg2_, KA_COMMAND_TRACE_2);
}

//---------------------------------------------------------------------------
void KernelAware::Print(const char* szStr_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.Print.szString = szStr_;
    g_u8KACommand             = KA_COMMAND_PRINT;
}

//---------------------------------------------------------------------------
bool KernelAware::IsSimulatorAware(void)
{
    return g_bIsKernelAware;
}

//---------------------------------------------------------------------------
int32_t KernelAware::Open(const char* szPath_, bool bRead_, bool bWrite_, bool bAppend_, bool bCreate_, bool bTruncate_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.OpenRequest.szPath = szPath_;
    g_stKAData.OpenRequest.bRead = bRead_;
    g_stKAData.OpenRequest.bWrite = bWrite_;
    g_stKAData.OpenRequest.bAppend = bAppend_;
    g_stKAData.OpenRequest.bCreate = bCreate_;
    g_stKAData.OpenRequest.bTruncate = bTruncate_;
    g_u8KACommand = KA_COMMAND_OPEN;

    return g_stKAData.OpenReturn.iHostFd;
}

//---------------------------------------------------------------------------
void KernelAware::Close(int32_t iHostFd_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.Close.iHostFd = iHostFd_;
    g_u8KACommand = KA_COMMAND_CLOSE;
}

//---------------------------------------------------------------------------
K_WORD KernelAware::Read(int32_t iHostFd_, K_ADDR* paReadBuffer_, uint16_t u16BytesToRead_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.Read.iHostFd = iHostFd_;
    g_stKAData.Read.paReadBuf = paReadBuffer_;
    g_stKAData.Read.u16BytesToRead = u16BytesToRead_;
    g_u8KACommand = KA_COMMAND_READ;

    return g_stKAData.ReadReturn.iBytesRead;
}

//---------------------------------------------------------------------------
K_WORD KernelAware::Write(int32_t iHostFd_, const K_ADDR* paWriteBuffer_, uint16_t u16BytesToWrite_)
{
    const auto cg= CriticalGuard{};
    g_stKAData.Write.iHostFd = iHostFd_;
    g_stKAData.Write.paWriteBuf = paWriteBuffer_;
    g_stKAData.Write.u16BytesToWrite = u16BytesToWrite_;
    g_u8KACommand = KA_COMMAND_WRITE;

    return g_stKAData.WriteReturn.iBytesWritten;
}

//---------------------------------------------------------------------------
void KernelAware::SetBlockingMode(int32_t iHostFd_, bool bBlocking_)
{
    const auto cg = CriticalGuard{};
    g_stKAData.Blocking.iHostFd = iHostFd_;
    g_stKAData.Blocking.bBlocking = bBlocking_;
    g_u8KACommand = KA_COMMAND_BLOCKING;
}

} // namespace Mark3
