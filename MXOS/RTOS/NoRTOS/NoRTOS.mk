#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME := NoRTOS

GLOBAL_DEFINES += NO_MXOS_RTOS


GLOBAL_INCLUDES := . \
                   ..\
                   portable/GCC/ARM_CM

$(NAME)_SOURCES := rtos.c \
                   ../mxos_rtos_common.c


ifneq ($(MBED_SUPPORT),)
$(NAME)_SOURCES += mbed_main.cpp
else
$(NAME)_SOURCES += mxos_main.c
endif

Cortex-M3_SOURCES  := 
Cortex-M3_INCLUDES := portable/GCC/ARM_CM

Cortex-M4_SOURCES  := 
Cortex-M4_INCLUDES := portable/GCC/ARM_CM

Cortex-M4F_SOURCES  := 
Cortex-M4F_INCLUDES := portable/GCC/ARM_CM

ARM968E-S_SOURCES	:= portable/GCC/ARM968E_S/port.c 
ARM968E-S_INCLUDES 	:= portable/GCC/ARM968E_S

$(NAME)_SOURCES += $($(HOST_ARCH)_SOURCES)
GLOBAL_INCLUDES += $($(HOST_ARCH)_INCLUDES)

