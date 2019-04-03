#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2017 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME = stm32l0

# Host architecture is ARM Cortex M0
HOST_ARCH := Cortex-M0plus

# Host MCU alias for OpenOCD
HOST_OPENOCD := stm32l0

#Add MiCO extended device drivers
$(NAME)_SOURCES := peripherals/platform_flash_mbed.c  \
                   peripherals/platform_eeprom_mbed.c \
                   peripherals/platform_flash.c       \
                   peripherals/platform_ota.c
                   
GLOBAL_INCLUDES := .

###############################################
# Use abslute path to reference mico-os codes #
###############################################
MBED_DRV_DIR := $(MBED_OS_PATH)/targets/TARGET_STM/TARGET_STM32L0
ST_DRV_DIR   := $(MBED_DRV_DIR)/device
ST_DRV_SRC   := $(notdir $(wildcard $(ST_DRV_DIR)/*.c))

$(NAME)_ABS_SOURCES := $(MBED_DRV_DIR)/analogin_device.c \
                       $(MBED_DRV_DIR)/analogout_device.c \
                       $(MBED_DRV_DIR)/flash_api.c \
                       $(MBED_DRV_DIR)/gpio_irq_device.c \
                       $(MBED_DRV_DIR)/pwmout_device.c \
                       $(MBED_DRV_DIR)/serial_device.c \
                       $(MBED_DRV_DIR)/spi_api.c
                   
# Add all files under "device" directory       
$(NAME)_ABS_SOURCES += $(foreach code, $(ST_DRV_SRC), $(addprefix $(ST_DRV_DIR)/,$(code)))

GLOBAL_ABS_INCLUDES := $(MBED_DRV_DIR) $(ST_DRV_DIR)   





                   
