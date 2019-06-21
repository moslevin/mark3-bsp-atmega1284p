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

    @file   kernelaware.h

    @brief  Kernel aware simulation support
*/
#pragma once

#include "kerneltypes.h"
#include "mark3cfg.h"

namespace Mark3
{
//---------------------------------------------------------------------------
/**
 * @brief The KernelAware class
 *
 * This class contains functions that are used to trigger kernel-aware
 * functionality within a supported simulation environment (i.e. flAVR).
 *
 * These static methods operate on a singleton set of global variables,
 * which are monitored for changes from within the simulator.  The simulator
 * hooks into these variables by looking for the correctly-named symbols in
 * an elf-formatted binary being run and registering callbacks that are called
 * whenever the variables are changed.  On each change of the command variable,
 * the kernel-aware data is analyzed and interpreted appropriately.
 *
 * If these methods are run in an unsupported simulator or on actual hardware
 * the commands generally have no effect (except for the exit-on-reset command,
 * which will result in a jump-to-0 reset).
 */
namespace KernelAware
{
    //---------------------------------------------------------------------------
    /**
     * @brief ProfileInit
     *
     * Initializes the kernel-aware profiler.  This function instructs the
     * kernel-aware simulator to reset its accounting variables, and prepare to
     * start counting profiling data tagged to the given string.  How this is
     * handled is the responsibility of the simulator.
     *
     * @param szStr_ String to use as a tag for the profilng session.
     */
    void ProfileInit(const char* szStr_);

    //---------------------------------------------------------------------------
    /**
     * @brief ProfileStart
     *
     * Instruct the kernel-aware simulator to begin counting cycles towards the
     * current profiling counter.
     *
     */
    void ProfileStart(void);

    //---------------------------------------------------------------------------
    /**
     * @brief ProfileStop
     *
     * Instruct the kernel-aware simulator to end counting cycles relative to the
     * current profiling counter's iteration.
     */
    void ProfileStop(void);

    //---------------------------------------------------------------------------
    /**
     * @brief ProfileReport
     *
     * Instruct the kernel-aware simulator to print a report for its current
     * profiling data.
     *
     */
    void ProfileReport(void);

    //---------------------------------------------------------------------------
    /**
     * @brief ExitSimulator
     *
     * Instruct the kernel-aware simulator to terminate (destroying the virtual
     * CPU).
     *
     */
    void ExitSimulator(void);

    //---------------------------------------------------------------------------
    /**
     * @brief Print
     *
     * Instruct the kernel-aware simulator to print a char string
     *
     * @param szStr_
     */
    void Print(const char* szStr_);

    //---------------------------------------------------------------------------
    /**
     * @brief Trace
     *
     * Insert a kernel trace statement into the kernel-aware simulator's debug
     * data stream.
     *
     * @param u16File_   16-bit code representing the file
     * @param u16Line_   16-bit code representing the line in the file
     */
    void Trace(uint16_t u16File_, uint16_t u16Line_);

    //---------------------------------------------------------------------------
    /**
     * @brief Trace
     *
     * Insert a kernel trace statement into the kernel-aware simulator's debug
     * data stream.
     *
     * @param u16File_   16-bit code representing the file
     * @param u16Line_   16-bit code representing the line in the file
     * @param u16Arg1_   16-bit argument to the format string.
     */
    void Trace(uint16_t u16File_, uint16_t u16Line_, uint16_t u16Arg1_);

    //---------------------------------------------------------------------------
    /**
     * @brief Trace
     *
     * Insert a kernel trace statement into the kernel-aware simulator's debug
     * data stream.
     *
     * @param u16File_   16-bit code representing the file
     * @param u16Line_   16-bit code representing the line in the file
     * @param u16Arg1_   16-bit argument to the format string.
     * @param u16Arg2_   16-bit argument to the format string.
     */
    void Trace(uint16_t u16File_, uint16_t u16Line_, uint16_t u16Arg1_, uint16_t u16Arg2_);

    //---------------------------------------------------------------------------
    /**
     * @brief IsSimulatorAware
     *
     * use this function to determine whether or not the code is running on a
     * simulator that is aware of the kernel.
     *
     * @return true - the application is being run in a kernel-aware simulator.
     *         false - otherwise.
     */
    bool IsSimulatorAware(void);

    //---------------------------------------------------------------------------
    /**
     * @brief Open
     *
     * This function is used to open a file on the host.  This may be a real binary
     * data file, or any other device represented as a file in a filesystem.
     *
     * @param szPath_ Path of the resource in the host filesystem to open
     * @param bRead_ Open file with read access
     * @param bWrite_ Open file with write access (at least one of read/write required)
     * @param bAppend_ Append to existing file
     * @param bCreate_ Create file if it does not exist
     * @param bTruncate_ Truncate existing file if it already exists
     * @return file descriptor returned by the host on opening the file.
     */
    int32_t Open(const char* szPath_, bool bRead_, bool bWrite_, bool bAppend_, bool bCreate_, bool bTruncate_);

    //---------------------------------------------------------------------------
    /**
     * @brief Close
     *
     * Close a previously opened file descriptor in the host environment.
     *
     * @param iHostFd_ File descriptor returned from ::Open().
     */
    void Close(int32_t iHostFd_);

    //---------------------------------------------------------------------------
    /**
     * @brief Read
     *
     * Read data from an open file descriptor on the host.
     *
     * @param iHostFd_  fd of the resource to read
     * @param paReadBuffer_ Buffer to read into
     * @param u16BytesToRead_ Number of bytes to read (max)
     * @return number of bytes actually read
     */
    K_WORD Read(int32_t iHostFd_, K_ADDR* paReadBuffer_, uint16_t u16BytesToRead_);

    //---------------------------------------------------------------------------
    /**
     * @brief Write
     *
     * Write data to an open file descriptor on the host
     *
     * @param iHostFd_ fd of the resource to write
     * @param paWriteBuffer_ Buffer to write into
     * @param u16BytesToWrite_ Number of bytes to write (max)+
     * @return Number of bytes actually written
     */
    K_WORD Write(int32_t iHostFd_, const K_ADDR* paWriteBuffer_, uint16_t u16BytesToWrite_);

    //---------------------------------------------------------------------------
    /**
     * @brief SetBlockingMode
     *
     * Set a file descriptor to use blocking or non-blocking I/O.  Note that blocking
     * I/O in the context of a host simulation likely means that the whole target blocks.
     *
     * @param iHostFd_ fd of the resource to set blocking mode for
     * @param bBlocking_ true - blocking, false - non-blocking
     */
    void SetBlockingMode(int32_t iHostFd_, bool bBlocking_);

} // namespace KernelAware
} // namespace Mark3
