NAME := MX3380

MODULE := MX3380
HOST_MCU_FAMILY := RTL8721D

$(NAME)_COMPONENTS += platform/MCU/$(HOST_MCU_FAMILY)

# Global includes
GLOBAL_INCLUDES  := .