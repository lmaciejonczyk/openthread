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
 *   This file includes the implementation for the driver manager.
 */

#include "openthread-core-config.h"
#include "openthread-system.h"
#include "platform-posix.h"

#include "driver_manager.hpp"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <syslog.h>
#include <termios.h>
#include <unistd.h>

#include <common/code_utils.hpp>
#include <common/logging.hpp>

namespace ot {
namespace PosixApp {

DriverManager::DriverManager()
{
}

otError DriverManager::Init(const otPlatformConfig &aPlatformConfig)
{
    otError error = OT_ERROR_NONE;

    VerifyOrDie(stat(aPlatformConfig.mRadioUrl.mDevice, &st) == 0, OT_EXIT_INVALID_ARGUMENTS);

exit:
    return error;
}

DriverManager::~DriverManager(void)
{
}

otError LoadRCP(const otPlatformConfig &aPlatformConfig)
{
    otError      error = OT_ERROR_NONE;
    DriverSpinel aDriverSpinel;
    DriverHdlc   aDriverHdlc;
    DriverUart   aDriverUart;

    paraCount = sizeof(aPlatformConfig.mRadioUrl.mParameter) / sizeof(aPlatformConfig.mRadioUrl.mParameter);

    for (int i = 0; i < paraCount; i++)
    {
        if (!strcmp(aPlatformConfig.mRadioUrl.mParameter[i], "spinel"))
        {
            aDriverSpinel.Init(aPlatformConfig);
        }
        if (!strcmp(aPlatformConfig.mRadioUrl.mParameter[i], "hdlc"))
        {
            aDriverHdlc.Init(aPlatformConfig);
        }
        if (!strcmp(aPlatformConfig.mRadioUrl.mParameter[i], "uart"))
        {
            aDriverUart.Init(aPlatformConfig);
        }
    }

    aDriverSpinel.SetLowerInterface(&aDriverHdlc);
    aDriverHdlc.SetLowerInterface(&aDriverUart);

exit:
    return error;
}

} // namespace PosixApp
} // namespace ot
