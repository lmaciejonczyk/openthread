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
 *   This file includes the implementation for the HDLC interface to radio (RCP).
 */

#include "openthread-core-config.h"
#include "platform-posix.h"

#include "driver_hdlc.hpp"

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

#ifndef SOCKET_UTILS_DEFAULT_SHELL
#define SOCKET_UTILS_DEFAULT_SHELL "/bin/sh"
#endif

namespace ot {
namespace PosixApp {

DriverHdlc::DriverHdlc : Drvier(),
                         mUpperInterface(mInput, mOutput, mWait),
                         mHdlcDecoder(mRxFrameBuffer, HandleHdlcFrame, this)
{
}

otError DriverHdlc::Init(const otPlatformConfig &aPlatformConfig)
{
    otError     error = OT_ERROR_NONE;
    struct stat st;

    mUpperInterface.mOutput = &Read;
    mUpperInterface.mInput  = &Write;
    mUpperInterface.mWait   = &WaitForFrame;

exit:
    return error;
}

DriverHdlc::~DriverHdlc(void)
{
    Deinit();
}

void DriverHdlc::Deinit(void)
{
    VerifyOrExit(!mLowerDriver, perror("wait RCP"));

exit:
    return;
}

void DriverHdlc::Decode(const uint8_t *aBuffer, uint16_t aLength)
{
    mHdlcDecoder.Decode(aBuffer, aLength);
}

otError DriverHdlc::SendFrame(const uint8_t *aFrame, uint16_t aLength)
{
    otError                          error = OT_ERROR_NONE;
    Hdlc::FrameBuffer<kMaxFrameSize> encoderBuffer;
    Hdlc::Encoder                    hdlcEncoder(encoderBuffer);

    SuccessOrExit(error = hdlcEncoder.BeginFrame());
    SuccessOrExit(error = hdlcEncoder.Encode(aFrame, aLength));
    SuccessOrExit(error = hdlcEncoder.EndFrame());

    error = Write(encoderBuffer.GetFrame(), encoderBuffer.GetLength());

exit:
    return error;
}

void DriverHdlc::Read(uint8_t *aFrame, uint16_t aLength)
{
    Decode(aFrame, aLength);
}

otError DriverHdlc::Write(const uint8_t *aFrame, uint16_t aLength)
{
    otError error = OT_ERROR_NONE;
#if OPENTHREAD_POSIX_VIRTUAL_TIME
    platformSimSendRadioSpinelWriteEvent(aFrame, aLength);
#else
    while (aLength)
    {
        ssize_t rval = write(mSockFd, aFrame, aLength);

        if (rval > 0)
        {
            aLength -= static_cast<uint16_t>(rval);
            aFrame += static_cast<uint16_t>(rval);
            continue;
        }

        if ((rval < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR))
        {
            DieNow(OT_EXIT_ERROR_ERRNO);
        }

        SuccessOrExit(error = WaitForWritable());
    }

exit:
#endif // OPENTHREAD_POSIX_VIRTUAL_TIME
    return error;
}

otError DriverHdlc::WaitForFrame(struct timeval &aTimeout)
{
    otError error = OT_ERROR_NONE;

#if OPENTHREAD_POSIX_VIRTUAL_TIME
    struct Event event;

    platformSimSendSleepEvent(&aTimeout);
    platformSimReceiveEvent(&event);

    switch (event.mEvent)
    {
    case OT_SIM_EVENT_RADIO_SPINEL_WRITE:
        Decode(event.mData, event.mDataLength);
        break;

    case OT_SIM_EVENT_ALARM_FIRED:
        ExitNow(error = OT_ERROR_RESPONSE_TIMEOUT);
        break;

    default:
        assert(false);
        break;
    }
#else  // OPENTHREAD_POSIX_VIRTUAL_TIME
    fd_set read_fds;
    fd_set error_fds;
    int rval;

    FD_ZERO(&read_fds);
    FD_ZERO(&error_fds);
    FD_SET(mSockFd, &read_fds);
    FD_SET(mSockFd, &error_fds);

    rval = select(mSockFd + 1, &read_fds, NULL, &error_fds, &aTimeout);

    if (rval > 0)
    {
        if (FD_ISSET(mSockFd, &read_fds))
        {
            Read();
        }
        else if (FD_ISSET(mSockFd, &error_fds))
        {
            DieNowWithMessage("NCP error", OT_EXIT_FAILURE);
        }
        else
        {
            DieNow(OT_EXIT_FAILURE);
        }
    }
    else if (rval == 0)
    {
        ExitNow(error = OT_ERROR_RESPONSE_TIMEOUT);
    }
    else if (errno != EINTR)
    {
        DieNowWithMessage("wait response", OT_EXIT_FAILURE);
    }
#endif // OPENTHREAD_POSIX_VIRTUAL_TIME

exit:
    return error;
}

otError DriverHdlc::SetLowerInterface(Driver *aDriver)
{
    otError error = OT_ERROR_NONE;
    if (aDriver)
    {
        mLowerDriver = aDriver;
    }
    else
    {
        error = OT_ERROR_FAILED;
    }

exit:
    return error;
}

void DriverHdlc::HandleHdlcFrame(void *aContext, otError aError)
{
    static_cast<DriverHdlc *>(aContext)->HandleHdlcFrame(aError);
}

void DriverHdlc::HandleHdlcFrame(otError aError)
{
    if (aError == OT_ERROR_NONE)
    {
        mCallbacks.HandleReceivedFrame();
    }
    else
    {
        mRxFrameBuffer.DiscardFrame();
        otLogWarnPlat("Error decoding hdlc frame: %s", otThreadErrorToString(aError));
    }
}

} // namespace PosixApp
} // namespace ot
