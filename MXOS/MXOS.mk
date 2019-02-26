#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME = MXOS

ifndef USES_BOOTLOADER
USES_BOOTLOADER :=1
endif

$(NAME)_COMPONENTS += MXOS/core
$(NAME)_SOURCES += mxos_main.c core/mxos_config.c

ifneq ($(filter $(subst ., ,$(COMPONENTS)),mocOS mocIP),)
$(NAME)_SOURCES += moc_main.c
endif

$(NAME)_COMPONENTS += MXOS/security \
                      MXOS/system

$(NAME)_COMPONENTS += utilities

GLOBAL_DEFINES += 

# Add MCU component
ifneq ($(MBED_SUPPORT),)
$(NAME)_COMPONENTS += platform/mbed
else
$(NAME)_COMPONENTS += platform/MCU/$(HOST_MCU_FAMILY)
endif

# Easylink Button
$(NAME)_COMPONENTS += drivers/keypad/gpio_button

# Define the default ThreadX and FreeRTOS starting stack sizes
FreeRTOS_START_STACK := 800
ThreadX_START_STACK  := 800
mocOS_START_STACK    := 800


GLOBAL_INCLUDES += . \
                   system \
                   system/include \
                   security
                   
# $(NAME)_CFLAGS  = $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)


