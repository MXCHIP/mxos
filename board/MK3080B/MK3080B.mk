############################################################################### 
#
#  The MIT License
#  Copyright (c) 2016 MXCHIP Inc.
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy 
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights 
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is furnished
#  to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in
#  all copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
#  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
############################################################################### 

NAME := Board_MK3080

WLAN_CHIP            	:= NONE
WLAN_CHIP_REVISION   	:= NONE
WLAN_CHIP_FAMILY     	:= NONE
WLAN_CHIP_FIRMWARE_VER  := NONE

NO_WIFI_FIRMWARE := YES        

MODULE              	:= 3080B
HOST_MCU_FAMILY      	:= MX1290
HOST_MCU_VARIANT     	:= MX1290
HOST_MCU_PART_NUMBER 	:= 

BUS := MK3080

# Extra build target in mxos_standard_targets.mk, include bootloader, and copy output file to eclipse debug file (copy_output_for_eclipse)
EXTRA_TARGET_MAKEFILES +=  $(MAKEFILES_PATH)/mxos_moc_targets.mk

# Global includes
GLOBAL_INCLUDES  := .

# Global defines
GLOBAL_DEFINES += $$(if $$(NO_CRLF_STDIO_REPLACEMENT),,CRLF_STDIO_REPLACEMENT)
GLOBAL_LDFLAGS += -L $(MXOS_OS_PATH)/board/MK$(MODULE)

# Components
$(NAME)_COMPONENTS += drivers/keypad/gpio_button

# Source files
$(NAME)_SOURCES := mxos_board.c

# MOC configuration
VALID_OSNS_COMBOS   := mocOS@mocIP
VALID_TLS           := mocSSL
MOC_APP_OFFSET      := 0x64000



