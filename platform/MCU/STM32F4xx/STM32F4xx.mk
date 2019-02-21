#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#


NAME = STM32F4xx

# Host architecture is ARM Cortex M4
HOST_ARCH := Cortex-M4

# Host MCU alias for OpenOCD
HOST_OPENOCD := stm32f4x

GLOBAL_INCLUDES := . \
                   .. \
                   ../include \
                   ../.. \
                   ../../include \
                   ../../$(TOOLCHAIN_NAME) \
                   ../../$(HOST_ARCH) \
                   ../../$(HOST_ARCH)/CMSIS \
                   peripherals

# Global defines
GLOBAL_DEFINES  := USE_STDPERIPH_DRIVER \
                   _STM3x_ \
                   _STM32x_

# Convert the MCU variant into the required STM peripheral library constant
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F405 STM32F415 STM32F407 STM32F417))
GLOBAL_DEFINES += STM32F40_41xxx
endif
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F427 STM32F437))
GLOBAL_DEFINES += STM32F427_437xx
endif
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F429 STM32F439))
GLOBAL_DEFINES += STM32F429_439xx
endif
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F401))
GLOBAL_DEFINES += STM32F401xx
endif
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F411))
GLOBAL_DEFINES += STM32F411xE
endif
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F412))
GLOBAL_DEFINES += STM32F412xG
endif
ifneq (,$(filter $(HOST_MCU_VARIANT), STM32F446))
GLOBAL_DEFINES += STM32F446xx
endif

# Global flags
GLOBAL_CFLAGS   += $$(CPU_CFLAGS)    $$(ENDIAN_CFLAGS_LITTLE)
GLOBAL_CXXFLAGS += $$(CPU_CXXFLAGS)  $$(ENDIAN_CXXFLAGS_LITTLE)
GLOBAL_ASMFLAGS += $$(CPU_ASMFLAGS)  $$(ENDIAN_ASMFLAGS_LITTLE)
GLOBAL_LDFLAGS  += $$(CPU_LDFLAGS)   $$(ENDIAN_LDFLAGS_LITTLE)


GLOBAL_LDFLAGS  += -Wl,--defsym,__STACKSIZE__=$$($(RTOS)_START_STACK)
GLOBAL_LDFLAGS  += -L ./platform/MCU/$(NAME)/$(TOOLCHAIN_NAME)

# Components
$(NAME)_COMPONENTS += $(TOOLCHAIN_NAME)
$(NAME)_COMPONENTS += MCU/STM32F4xx/peripherals
$(NAME)_COMPONENTS += utilities

# Source files
$(NAME)_SOURCES := ../../$(HOST_ARCH)/crt0_$(TOOLCHAIN_NAME).c \
                   ../../$(HOST_ARCH)/platform_core.c \
                   ../../$(HOST_ARCH)/hardfault_handler.c \
                   ../../$(HOST_ARCH)/platform_application.c \
                   ../platform_nsclock.c \
                   ../platform_retarget.c \
                   ../mxos_platform_common.c \
                   ../wlan_platform_common.c \
                   platform_init.c \
                   platform_vector_table.c \
                   GCC/platform_unhandled_isr.c


ifndef NO_WIFI
$(NAME)_SOURCES += wlan_bus_driver/wlan_platform.c \
                   wlan_bus_driver/wlan_bus_$(BUS).c
else
ifdef SHARED_WIFI_SPI_BUS
$(NAME)_SOURCES += wlan_bus_driver/wlan_bus_SPI.c
endif #SHARED_WIFI_SPI_BUS
endif #NO_WIFI

# $(NAME)_CFLAGS = $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)


ifeq ($(APP),bootloader)
####################################################################################
# Building bootloader
####################################################################################

DEFAULT_LINK_SCRIPT += $(TOOLCHAIN_NAME)/bootloader$(LINK_SCRIPT_SUFFIX)
GLOBAL_INCLUDES     += 

else
ifneq ($(filter spi_flash_write, $(APP)),)
####################################################################################
# Building spi_flash_write
####################################################################################

PRE_APP_BUILDS      += bootloader
DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_ram$(LINK_SCRIPT_SUFFIX)
GLOBAL_DEFINES      += __JTAG_FLASH_WRITER_DATA_BUFFER_SIZE__=16384
GLOBAL_INCLUDES     += 

else
ifeq ($(USES_BOOTLOADER),1)
####################################################################################
# Building standard application to run with bootloader
####################################################################################

PRE_APP_BUILDS      += bootloader
DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_with_bootloader$(LINK_SCRIPT_SUFFIX)
GLOBAL_INCLUDES     += 

else
####################################################################################
# Building a standalone application (standalone app without bootloader)
####################################################################################

DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_no_bootloader$(LINK_SCRIPT_SUFFIX)
GLOBAL_INCLUDES     += 

endif # USES_BOOTLOADER = 1
endif # APP=spi_flash_write
endif # APP=bootloader

