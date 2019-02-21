#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#


NAME = MX1290

# Host architecture is ARM Cortex M4
HOST_ARCH := Cortex-M4F
HOST_ARCH_M4 = Cortex-M4
# Host MCU alias for OpenOCD
HOST_OPENOCD := MX1290

GLOBAL_DEFINES += __FPU_PRESENT CONFIG_CPU_MX1290

# Global flags
GLOBAL_CFLAGS   += $$(CPU_CFLAGS)    $$(ENDIAN_CFLAGS_LITTLE)
GLOBAL_CXXFLAGS += $$(CPU_CXXFLAGS)  $$(ENDIAN_CXXFLAGS_LITTLE)
GLOBAL_ASMFLAGS += $$(CPU_ASMFLAGS)  $$(ENDIAN_ASMFLAGS_LITTLE)
GLOBAL_LDFLAGS  += $$(CPU_LDFLAGS)   $$(ENDIAN_LDFLAGS_LITTLE)


GLOBAL_LDFLAGS  += -nostartfiles
GLOBAL_LDFLAGS  += -Wl,--defsym,__STACKSIZE__=$$($(RTOS)_START_STACK)
GLOBAL_LDFLAGS  += -L ./platform/MCU/$(NAME)/$(TOOLCHAIN_NAME)

# Components
$(NAME)_COMPONENTS += $(TOOLCHAIN_NAME)

GLOBAL_INCLUDES := . \
                   .. \
                   ../include \
                   ../.. \
                   ../../include \
                   ../../$(TOOLCHAIN_NAME) \
                   ../../$(HOST_ARCH_M4) \
                   ../../$(HOST_ARCH_M4)/CMSIS \
                   interface \
                   peripherals 

$(NAME)_SOURCES := ../../$(HOST_ARCH_M4)/crt0_$(TOOLCHAIN_NAME).c \
                   ../../$(HOST_ARCH_M4)/platform_core.c \
                   ../platform_nsclock.c

$(NAME)_CFLAGS += -Wno-implicit-function-declaration -Wno-unused-variable


ifneq ($(filter $(subst ., ,$(COMPONENTS)),mocOS),)
####################################################################################
# MOC application
####################################################################################
$(NAME)_SOURCES += ../moc_platform_common.c ../platform_retarget.c
$(NAME)_SOURCES += moc/moc_adapter.c
else
####################################################################################
# Building stand-alone image( sub build: spi_flash_write )
####################################################################################
$(NAME)_SOURCES += platform_vector_table.c \
                   GCC/platform_unhandled_isr.c \
                   ../../$(HOST_ARCH_M4)/hardfault_handler.c \
                   ../mxos_platform_common.c \
                   ../platform_retarget.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_pmu.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_pinmux.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_driver.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_clock.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_flash.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_flashc.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_qspi.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_uart.c \
                   peripherals/sdk/src/drivers/mw300/lowlevel/mw300_crc.c \
                   peripherals/sdk/src/drivers/mw300/mdev_iflash.c \
                   peripherals/sdk/src/drivers/common/mdev_crc.c \
                   peripherals/sdk/src/core/util/crc/crc32.c \
                   peripherals/sdk/src/core/util/arc4.c \
                   peripherals/sdk/src/core/partition/partition.c \
                   peripherals/sdk/src/core/mdev/mdev.c \
                   peripherals/sdk/src/core/util/flash.c \
                   peripherals/sdk/src/core/util/boot_flags.c \
                   peripherals/boot2/boot2.c \
                   peripherals/boot2/utils/crc32.c \
                   peripherals/platform_uart.c \
                   peripherals/platform_flash.c \
                   platform_init.c
                   
GLOBAL_INCLUDES += peripherals/boot2
                            
GLOBAL_DEFINES += CONFIG_FLASH_PARTITION_COUNT=10 \
                  ARM_GNU

endif

ifneq ($(filter spi_flash_write, $(APP)),)
####################################################################################
# Building spi_flash_write
####################################################################################

DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_ram$(LINK_SCRIPT_SUFFIX)
GLOBAL_DEFINES      += __JTAG_FLASH_WRITER_DATA_BUFFER_SIZE__=16384

else
ifneq ($(filter $(subst ., ,$(COMPONENTS)),mocOS),)
####################################################################################
# Building standard moc application
####################################################################################

DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_with_moc$(LINK_SCRIPT_SUFFIX)

endif # APP=moc
endif # APP=spi_flash_write

