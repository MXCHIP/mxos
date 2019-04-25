NAME := OSAL

$(NAME)_SOURCES := \
os/os.c

$(NAME)_INCLUDES := \
. \
../sdk/component/os/freertos/freertos_v8.1.2/Source/include \
../sdk/component/os/freertos/freertos_v8.1.2/Source/portable/GCC/ARM_CM4F \
../sdk/project/realtek_amebaD_cm4_gcc_verification/inc
