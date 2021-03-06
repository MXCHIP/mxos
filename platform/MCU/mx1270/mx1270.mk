#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#


NAME = platform_mx1270

# Host architecture is ARM Cortex M4
HOST_ARCH := Cortex-M4

# Host MCU alias for OpenOCD
HOST_OPENOCD := mx1270

GLOBAL_INCLUDES += \
. \
.. \
../include \
../.. \
../../include \
../../$(TOOLCHAIN_NAME) \
../../$(HOST_ARCH) \
../../$(HOST_ARCH)/CMSIS \
peripherals

# Global flags
GLOBAL_CFLAGS   += $$(CPU_CFLAGS)    $$(ENDIAN_CFLAGS_LITTLE)
GLOBAL_CXXFLAGS += $$(CPU_CXXFLAGS)  $$(ENDIAN_CXXFLAGS_LITTLE)
GLOBAL_ASMFLAGS += $$(CPU_ASMFLAGS)  $$(ENDIAN_ASMFLAGS_LITTLE)
GLOBAL_LDFLAGS  += $$(CPU_LDFLAGS)   $$(ENDIAN_LDFLAGS_LITTLE)
GLOBAL_LDFLAGS  += -L ./platform/MCU/mx1270

# Source files
$(NAME)_SOURCES := \
appstart.c \
../../../MXOS/RTOS/mxos_rtos_common.c \
../../../MXOS/net/LwIP/mxos/mxos_network.c

$(NAME)_INCLUDES := \
../../../MXOS/RTOS/FreeRTOS/mxos

# Libraries
$(NAME)_PREBUILT_LIBRARY := \
libraries/arch_armv7m.a \
libraries/cli.a \
libraries/kernel_init.a \
libraries/libasr_wifi.a \
libraries/mcu_asr5501.a \
libraries/netmgr.a \
libraries/osal_aos.a \
libraries/ulog.a \
libraries/yloop.a \
libraries/board_asr5501.a \
libraries/debug.a \
libraries/lwip.a \
libraries/mxos.a \
libraries/newlib_stub.a \
libraries/rhino.a \
libraries/vfs.a \
libraries/alicrypto.a \
libraries/ls_osa.a

DEFAULT_LINK_SCRIPT := gcc_a0v2.ld

EXTRA_TARGET_MAKEFILES +=  \
mxos/platform/MCU/mx1270/flash_alg.mk \
mxos/platform/MCU/mx1270/image.mk

GLOBAL_DEFINES += CONFIG_MX1270 FreeRTOS_VERSION=\"V9.0.0\"