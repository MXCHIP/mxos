#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME = MiCO_mbed_hal_Interface

MBED_OS_DIR = mbed-os

GLOBAL_INCLUDES := . \
                   .. \
                   ../include \
                   include \
                   peripherals

# Global flags
GLOBAL_CFLAGS   += $$(CPU_CFLAGS)    $$(ENDIAN_CFLAGS_LITTLE)
GLOBAL_CXXFLAGS += $$(CPU_CXXFLAGS)  $$(ENDIAN_CXXFLAGS_LITTLE)
GLOBAL_ASMFLAGS += $$(CPU_ASMFLAGS)  $$(ENDIAN_ASMFLAGS_LITTLE)
GLOBAL_LDFLAGS  += $$(CPU_LDFLAGS)   $$(ENDIAN_LDFLAGS_LITTLE)


#GLOBAL_LDFLAGS  += -Wl,--defsym __STACKSIZE__=$$($(RTOS)_START_STACK)


# Components
$(NAME)_COMPONENTS += mbed-os

# Components, add mbed targets
TARGETS := $(foreach target, $(MBED_TARGETS), TARGET_$(target))
$(eval DIRS := $(shell $(PYTHON) $(LIST_SUB_DIRS_SCRIPT) mxos/platform/mbed/targets))
$(foreach DIR, $(DIRS), $(if $(filter $(notdir $(DIR)), $(TARGETS)), $(eval $(NAME)_COMPONENTS += $(subst \,/,$(DIR))),))
$(foreach target, $(TARGETS), $(eval GLOBAL_DEFINES += $(target)))

# Source files
$(NAME)_SOURCES := mxos_platform_common.c \
                   wlan_platform_common.c \
                   peripherals/platform_adc.c \
                   peripherals/platform_init.c \
                   peripherals/platform_irq.c \
                   peripherals/platform_spi.c \
                   peripherals/platform_rtc.c \
                   peripherals/platform_rng.c \
                   peripherals/platform_gpio.c \
                   peripherals/platform_i2c.c \
                   peripherals/platform_uart.c \
                   peripherals/platform_watchdog.c \
                   peripherals/platform_mcu_powersave.c\
                   peripherals/platform_pwm.c \
                   peripherals/platform_nsclock.c \
                   vcall.c
                   
$(NAME)_SOURCES += ../$(HOST_ARCH)/platform_core.c \
                   ../$(HOST_ARCH)/platform_application.c \
                   ../$(TOOLCHAIN_NAME)/time_newlib.c
                   


