#
#  Copyright (c) 2020, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

set(OT_PLATFORM_LIB "openthread-jn5189_plat" "openthread-jn5189_sdk" PARENT_SCOPE)

if(NOT OT_CONFIG)
    set(OT_CONFIG "openthread-core-jn5189-config.h")
    set(OT_CONFIG ${OT_CONFIG} PARENT_SCOPE)
endif()

list(APPEND OT_PLATFORM_DEFINES
    "OPENTHREAD_CORE_CONFIG_PLATFORM_CHECK_FILE=\"openthread-core-jn5189-config-check.h\""
    "MBEDTLS_CONFIG_FILE=\"mbedtls-config.h\""
)
set(OT_PLATFORM_DEFINES ${OT_PLATFORM_DEFINES} PARENT_SCOPE)

list(APPEND OT_PRIVATE_DEFINES "MBEDTLS_USER_CONFIG_FILE=\"jn5189-mbedtls-config.h\"")
list(APPEND OT_PUBLIC_INCLUDES
    "${PROJECT_SOURCE_DIR}/third_party/nxp"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/crypto"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/aes"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/sha"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/rom_apis"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/dk6_jn5180"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/CMSIS/Include"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/uMac/Include"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/Radio_JN5189/Include"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/JennicCommon/Include"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/MicroSpecific/Include"
)
set(OT_PUBLIC_INCLUDES ${OT_PUBLIC_INCLUDES} PARENT_SCOPE)

list(APPEND OT_PLATFORM_DEFINES "OPENTHREAD_PROJECT_CORE_CONFIG_FILE=\"${OT_CONFIG}\"")
string (REPLACE "-pedantic-errors" "" OT_CFLAGS "${OT_CFLAGS}")

add_library(openthread-jn5189_plat
    alarm.c
    diag.c
    entropy.c
    flash.c
    logging.c
    misc.c
    radio.c
    settings_jn5189.c
    system.c
    uart.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_assert.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/crypto/aes_alt.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/crypto/ksdk_mbedtls.c
    $<TARGET_OBJECTS:openthread-platform-utils>
)


add_library(openthread-jn5189_sdk
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/startup_JN5189.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/system_JN5189.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/dk6_jn5180/pin_mux.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/dk6_jn5180/clock_config.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_gpio.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_clock.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_ctimer.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_wtimer.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_flash.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_usart.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_rng.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_flexcomm.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_reset.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_power.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/fsl_debug_console.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/aes/fsl_aes.c
    ${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/drivers/sha/fsl_sha.c
    $<TARGET_OBJECTS:openthread-platform-utils>
)

set_property(TARGET openthread-jn5189_plat PROPERTY C_STANDARD 99)
set_property(TARGET openthread-jn5189_sdk PROPERTY C_STANDARD 99)

target_link_libraries(openthread-jn5189_plat PRIVATE
    openthread-platform-utils
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/libraries/libMiniMac.a"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/libraries/libRadio.a"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/libraries/libMicroSpecific_JN518x.a"
)
target_link_libraries(openthread-jn5189_sdk PRIVATE
    openthread-platform-utils
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/libraries/libMiniMac.a"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/libraries/libRadio.a"
    "${PROJECT_SOURCE_DIR}/third_party/nxp/JN5189/libraries/libMicroSpecific_JN518x.a"
)
target_link_options(openthread-jn5189_plat PUBLIC -T${PROJECT_SOURCE_DIR}/examples/platforms/jn5189/jn5189.ld)
target_link_options(openthread-jn5189_sdk PUBLIC -T${PROJECT_SOURCE_DIR}/examples/platforms/jn5189/jn5189.ld)
target_link_options(openthread-jn5189_plat PUBLIC -Wl,-Map=$<TARGET_PROPERTY:NAME>.map)
target_link_options(openthread-jn5189_sdk PUBLIC -Wl,-Map=$<TARGET_PROPERTY:NAME>.map)

target_compile_definitions(openthread-jn5189_plat
    PUBLIC
        ${OT_PLATFORM_DEFINES}
    PRIVATE
        ${OT_PRIVATE_DEFINES}
        -DCPU_JN518X
        -DCPU_JN518X_REV=2
        -DJENNIC_CHIP_FAMILY_JN518x
        -DJENNIC_CHIP_FAMILY_NAME=_JN518x
        -DSDK_DEBUGCONSOLE=0
)

target_compile_definitions(openthread-jn5189_sdk
    PUBLIC
        ${OT_PLATFORM_DEFINES}
    PRIVATE
        ${OT_PRIVATE_DEFINES}
        -DCPU_JN518X
        -DCPU_JN518X_REV=2
        -DJENNIC_CHIP_FAMILY_JN518x
        -DJENNIC_CHIP_FAMILY_NAME=_JN518x
        -DSDK_DEBUGCONSOLE=0
)

target_compile_options(openthread-jn5189_plat
    PRIVATE
    ${OT_CFLAGS}
    -Wno-unknown-pragmas
    -Wno-sign-compare
    -Wno-unused-function
    -Wno-unused-parameter
    -Wno-empty-body
)
target_compile_options(openthread-jn5189_sdk
    PRIVATE
    ${OT_CFLAGS}
    -Wno-unknown-pragmas
    -Wno-sign-compare
    -Wno-unused-function
    -Wno-unused-parameter
    -Wno-empty-body
)

target_include_directories(openthread-jn5189_plat
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        ${OT_PRIVATE_INCLUDES}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/src/core
        ${PROJECT_SOURCE_DIR}/examples/platforms
        ${PROJECT_SOURCE_DIR}/third_party/mbedtls/repo/include
        ${PROJECT_SOURCE_DIR}/third_party/mbedtls
)
target_include_directories(openthread-jn5189_sdk
    PRIVATE
        ${OT_PUBLIC_INCLUDES}
        ${OT_PRIVATE_INCLUDES}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/src/core
        ${PROJECT_SOURCE_DIR}/examples/platforms
        ${PROJECT_SOURCE_DIR}/third_party/mbedtls/repo/include
        ${PROJECT_SOURCE_DIR}/third_party/mbedtls
)
