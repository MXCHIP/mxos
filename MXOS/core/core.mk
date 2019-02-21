#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME := Lib_MXOS_Kernel

GLOBAL_INCLUDES := 

ifeq ($(MXOS_CORE),)
MXOS_CORE               := MXOS
endif

WLAN_RF_MODULE_LIST := 1062 1088

MODULE_LIST := 3165 3166 3238 3239 3297

ifneq ($(filter $(MODULE),$(WLAN_RF_MODULE_LIST)),)
MXOS_PREBUILT_LIBRARY := $(MXOS_CORE).$(MODULE).$(BUS).$(HOST_ARCH).$(TOOLCHAIN_NAME).a
#MXOS_PREBUILT_LIBRARY := ../../../../../mxos_source/branch_for_merge/output/MXOS.$(MODULE).$(BUS).$(HOST_ARCH).$(TOOLCHAIN_NAME).a
else
ifneq ($(filter $(MODULE),$(MODULE_LIST)),)
MXOS_PREBUILT_LIBRARY := $(MXOS_CORE).$(MODULE).$(TOOLCHAIN_NAME).a
#MXOS_PREBUILT_LIBRARY := ../../../../../mxos_source/branch_for_merge/output/MXOS.$(MODULE).$(TOOLCHAIN_NAME).a
endif
endif


ifneq ($(filter $(subst ., ,$(COMPONENTS)),mocOS mocIP),)
$(info MXOS core based on MOC ! )
$(NAME)_SOURCES += ../../platform/MCU/$(HOST_MCU_FAMILY)/moc/moc_api.c 
                   
GLOBAL_INCLUDES += . \
                   ../../platform/MCU/$(HOST_MCU_FAMILY)/moc
                   
GLOBAL_DEFINES += MOC=1

GLOBAL_LDFLAGS   += $$(CLIB_LDFLAGS_NANO_FLOAT)
else
$(info MXOS core based on pre-build library: ===$(MXOS_PREBUILT_LIBRARY)=== )
ifneq ($(wildcard $(CURDIR)$(MXOS_PREBUILT_LIBRARY)),)
$(NAME)_PREBUILT_LIBRARY := $(MXOS_PREBUILT_LIBRARY)
$(NAME)_SOURCES += rf_driver/wlan_firmware.c
else
# Build from source (MXCHIP internal)
$(info MXOS core based on source code ! $(MXOS_PREBUILT_LIBRARY) is not found! )
endif #ifneq ($(wildcard $(CURDIR)$(MXOS_PREBUILT_LIBRARY)),)
endif #ifneq ($(filter $(MODULE),$(MOC_MODULE_LIST)),)

