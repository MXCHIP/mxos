NAME := board_mx1270

MODULE := mx1270
HOST_MCU_FAMILY := mx1270

# Global includes
GLOBAL_INCLUDES  := .

$(NAME)_SOURCES := \
mxos_board.c