/*
 *  Copyright (c) 2020, The OpenThread Authors.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *   This file includes definitions for the driver manager.
 */

#ifndef DRIVER_MANANGER_HPP_
#define DRIVER_MANANGER_HPP_
#include "driver_hdlc.hpp"
#include "driver_spinel.hpp"
#include "driver_uart.hpp"
#include "openthread-system.h"
#include "platform-config.h"
#include <openthread/platform/uart.h>

namespace ot {
namespace PosixApp {

typedef struct ConnectOption
{
    const char *mName;
    const char *mValue;
} ConnectOption;

class Driver
{
public:
    virtual otError Init(const otPlatformConfig &aPlatformConfig) = 0;

    typedef otError (*InputFunction)(char *aBuffer, uint16_t aLength);
    typedef otError (*OutputFunction)(char *aBuffer, uint16_t aLength);
    typedef otError (*WaitFunction)(void);

    typedef struct Interface
    {
        InputFunction  mInput;
        OutputFunction mOutput;
        WaitFunction   mWait;
    } Interface;

private:
    Interface   mInterface;
    const char *mName;
};

class UpperInterface
{
public:
    virtual otError SetLowerInterface(Driver &aDriver) = 0;
};

class LowerInterface
{
public:
    virtual otError SetUpperInterface(Driver &aDriver) = 0;
};

class DriverManager
{
public:
    /**
     * This constructor initializes the driver manager.
     *
     */
    DriverManager(void);

    /**
     * Initialize the driver manager.
     *
     * @param[in]  aPlatformConfig  Platform configuration structure.
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError Init(const otPlatformConfig &aPlatformConfig);

    /**
     * This method loads the drivers for OT core and RCP communication.
     *
     * @param[in]  aUrl  A pointer to the URL string
     *
     * @retval  OT_ERROR_NONE               Succeeded.
     * @retval  OT_ERROR_BUSY               Failed due to another operation is on going.
     * @retval  OT_ERROR_RESPONSE_TIMEOUT   Failed due to no response received from the transceiver.
     *
     */
    otError LoadRCP(const otRadioUrl *aUrl);
};

} // namespace PosixApp
} // namespace ot

#endif // DRIVER_MANAGER_HPP_
