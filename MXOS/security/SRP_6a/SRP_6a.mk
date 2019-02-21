#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME := Lib_MXOS_Security_SRP_6a

GLOBAL_INCLUDES:= inc

#SRP-6a
ifneq ($(wildcard $(CURDIR)Lib_SRP6a.$(HOST_ARCH).$(TOOLCHAIN_NAME).release.a),)
$(NAME)_PREBUILT_LIBRARY := Lib_SRP6a.$(HOST_ARCH).$(TOOLCHAIN_NAME).release.a
else
# Build from source
SRC_MAKEFILE := $(CURDIR)SRP_6a_src.mk
ifeq ($(SRC_MAKEFILE), $(wildcard $(SRC_MAKEFILE)))
include $(SRC_MAKEFILE)
endif
endif


