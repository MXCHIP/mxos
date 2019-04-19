#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2017 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME = stm32f412xg

SRC_DIR := $(MBED_OS_PATH)/targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F412xG


$(NAME)_SOURCES := TOOLCHAIN_$(TOOLCHAIN_NAME_MBED)/startup_stm32f412xx.s
                   
GLOBAL_ABS_INCLUDES := $(SRC_DIR) $(SRC_DIR)/device
                   

ifeq ($(APP),bootloader)
####################################################################################
# Building bootloader
####################################################################################

DEFAULT_LINK_SCRIPT += TOOLCHAIN_$(TOOLCHAIN_NAME_MBED)/STM32F412XG_BL_FLASH$(LINK_SCRIPT_SUFFIX)
GLOBAL_DEFINES      += VECT_TAB_OFFSET=0x0

else
ifneq ($(filter spi_flash_write, $(APP)),)
####################################################################################
# Building spi_flash_write
####################################################################################

DEFAULT_LINK_SCRIPT += TOOLCHAIN_$(TOOLCHAIN_NAME_MBED)/STM32F412XG_PROG$(LINK_SCRIPT_SUFFIX)
GLOBAL_DEFINES      += __JTAG_FLASH_WRITER_DATA_BUFFER_SIZE__=16384 \
                       VECT_TAB_SRAM \
                       VECT_TAB_OFFSET=0x5000

else
ifeq ($(USES_BOOTLOADER),1)
####################################################################################
# Building standard application to run with bootloader
####################################################################################

PRE_APP_BUILDS      += bootloader
DEFAULT_LINK_SCRIPT := TOOLCHAIN_$(TOOLCHAIN_NAME_MBED)/STM32F412XG_APP_FLASH$(LINK_SCRIPT_SUFFIX)
ifneq ($(VECT_TAB_OFFSET_APP),)
GLOBAL_DEFINES      += VECT_TAB_OFFSET=$(VECT_TAB_OFFSET_APP)
else
GLOBAL_DEFINES      += VECT_TAB_OFFSET=0x8000
endif

else
####################################################################################
# Building a standalone application (standalone app without bootloader)
####################################################################################

DEFAULT_LINK_SCRIPT := TOOLCHAIN_$(TOOLCHAIN_NAME_MBED)/STM32F412XG$(LINK_SCRIPT_SUFFIX)
GLOBAL_DEFINES      += VECT_TAB_OFFSET=0x0

endif # USES_BOOTLOADER = 1
endif # APP=spi_flash_write
endif # APP=bootloader

