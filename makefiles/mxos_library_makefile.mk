#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#
export SOURCE_ROOT ?= ./
export MXOS_OS_PATH := $(SOURCE_ROOT)mxos

LIB_NAME := $(notdir $(LIB_DIR))
LIB_OUT_DIR := $(dir $(LIB_DIR))

ALWAYS_OPTIMISE := 1

BYPASS_LIBRARY_POISON_CHECK=1

include $(LIB_DIR)/$(LIB_NAME)_src.mk

SOURCES := $(addprefix $(LIB_DIR)/,$($(NAME)_SOURCES))
LIBRARY_OUTPUT_DIR := $(LIB_OUT_DIR)

# Standard library defines
CFLAGS += -c -MD -ggdb $(CPU_CFLAGS) $(ENDIAN_CFLAGS_LITTLE) -Wall -fsigned-char -ffunction-sections -fdata-sections -fno-common -std=gnu11 

CFLAGS += $(addprefix -I,$(GLOBAL_INCLUDES)) $(addprefix -D,$(GLOBAL_DEFINES)) $(addprefix -I$(LIB_DIR)/,$($(NAME)_INCLUDES)) $(addprefix -D,$($(NAME)_DEFINES)) $($(NAME)_CFLAGS)

CFLAGS += -I$(MXOS_OS_PATH)/include \
	      -I$(MXOS_OS_PATH)/mxos \
          -I$(MXOS_OS_PATH)/mxos/security \
          -I$(MXOS_OS_PATH)/mxos/security/Sodium/inc \
          -I$(MXOS_OS_PATH)/mxos/security/SRP_6a/inc \
          -I$(MXOS_OS_PATH)/mxos/system \
          -I$(MXOS_OS_PATH)/libraries/drivers \
          -I$(MXOS_OS_PATH)/libraries/drivers/MXOSKit_EXT \
          -I$(MXOS_OS_PATH)/libraries/utilities \
          -I$(MXOS_OS_PATH)/platform \
          -I$(MXOS_OS_PATH)/platform/include \
          -I$(MXOS_OS_PATH)/platform/MCU/include \
          -I$(MXOS_OS_PATH)/platform/$(HOST_ARCH) \
          -I$(MXOS_OS_PATH)/template/includes

include $(MXOS_OS_PATH)/makefiles/mxos_library_build.mk

