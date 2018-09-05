/*===========================================================================
     _____        _____        _____        _____
 ___|    _|__  __|_    |__  __|__   |__  __| __  |__  ______
|    \  /  | ||    \      ||     |     ||  |/ /     ||___   |
|     \/   | ||     \     ||     \     ||     \     ||___   |
|__/\__/|__|_||__|\__\  __||__|\__\  __||__|\__\  __||______|
    |_____|      |_____|      |_____|      |_____|

--[Mark3 Realtime Platform]--------------------------------------------------

Copyright (c) 2018 m0slevin, all rights reserved.
See license.txt for more information
===========================================================================*/
/**
    @file   ut_support.h
    @brief  Unit test support

    @note derived from https://github.com/arunkuttiyara/cortex_m3_development
 */

namespace Mark3
{
namespace UnitTestSupport
{
    /**
     * Responsible for creating a device registered at "/dev/tty", that operates as
     * standard-output for the test code.
     */
    void OnInit();

    /**
     * Open the output driver for the test interface code.
     */
    void OnStart();

    /**
     * Port-specific idle function implementation
     */
    void OnIdle();

    /**
     * Port-specific function to call at the end of the test function.
     * In simulated environments, this is used to exit the simulator.
     */
    void OnExit(int rc);

} // namespace UnitTest
} // namespace Mark3
