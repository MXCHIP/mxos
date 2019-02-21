#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME := mocOS

VERSION := 1.0.0

$(NAME)_COMPONENTS += MXOS/RTOS/mocOS/mxos

GLOBAL_INCLUDES := ..

# Define some macros to allow for some rtos-specific checks
GLOBAL_DEFINES += RTOS_$(NAME)=1
GLOBAL_DEFINES += $(NAME)_VERSION=$$(SLASH_QUOTE_START)v$(VERSION)$$(SLASH_QUOTE_END)


