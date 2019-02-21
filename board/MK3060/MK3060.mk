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

NAME := Board_MK3060

WLAN_CHIP            	:= MOC108
WLAN_CHIP_REVISION   	:= 1
WLAN_CHIP_FAMILY     	:= MOC108
WLAN_CHIP_FIRMWARE_VER  := 1

MODULE              	:= 3060
HOST_MCU_FAMILY      	:= MOC108
HOST_MCU_VARIANT     	:= MOC108
HOST_MCU_PART_NUMBER 	:= MOC108

BUS := NONE

JTAG := jlink

# Global includes
GLOBAL_INCLUDES  := .

# Components
$(NAME)_COMPONENTS += drivers/spi_flash
$(NAME)_COMPONENTS += drivers/keypad/gpio_button

# Source files
$(NAME)_SOURCES := mxos_board.c
