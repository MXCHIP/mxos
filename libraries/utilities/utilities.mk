#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

NAME := Lib_Utilities

GLOBAL_INCLUDES += .

$(NAME)_SOURCES := AESUtils.c \
                   LinkListUtils.c \
                   SocketUtils.c \
                   HTTPUtils.c \
                   TimeUtils.c \
                   TLVUtils.c \
                   URLUtils.c
                   
# MXOS source codes
$(NAME)_SOURCES += CheckSumUtils.c \
                   RingBufferUtils.c \
                   StringUtils.c
                   
$(NAME)_COMPONENTS += utilities/json_c

                   
$(NAME)_CFLAGS   += -Wno-char-subscripts



