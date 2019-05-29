NAME := RTL8721D

# Host architecture is Cortex-M4
HOST_ARCH := Cortex-M33

$(NAME)_PREBUILT_LIBRARY := \
image/objs.a \
image/lib_wlan.a \
image/lib_wps.a \
image/cmse_implib.lib

DEFAULT_LINK_SCRIPT := rlx8721d_img2_ns.ld
$(NAME)_SOURCES := ../../../MXOS/net/LwIP/mxos/mxos_network.c ../../../MXOS/RTOS/mxos_rtos_common.c
$(NAME)_INCLUDES := ../../../MXOS/RTOS/FreeRTOS/mxos

GLOBAL_CFLAGS	+= -ffunction-sections -march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=softfp -mfpu=fpv5-sp-d16 -g -gdwarf-3 -nostartfiles -nodefaultlibs -nostdlib -O2 -D__FPU_PRESENT -gdwarf-3 -fstack-usage -fdata-sections -nostartfiles -nostdlib -Wall -Wpointer-arith -Wstrict-prototypes -Wno-write-strings -Wno-maybe-uninitialized -Wextra -Wno-implicit-fallthrough
GLOBAL_CXXFLAGS	+= $(CPU_CFLAGS)
GLOBAL_ASMFLAGS	+= $(CPU_CFLAGS)
GLOBAL_LDFLAGS	+= -march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=softfp -mfpu=fpv5-sp-d16 -nostartfiles -specs nosys.specs -Wl,--gc-sections -Wl,--warn-section-align -Wl,--build-id=none -Wl,--no-enum-size-warning -Wl,--warn-common -lm -lstdc++
GLOBAL_LDFLAGS	+= -L mxos/platform/MCU/RTL8721D

GLOBAL_INCLUDES := ../include ../../ .
                   
EXTRA_TARGET_MAKEFILES +=  mxos/platform/MCU/RTL8721D/build_helper.mk


GLOBAL_LDFLAGS  += \
-Wl,-wrap,_malloc_r \
-Wl,-wrap,_free_r \
-Wl,-wrap,_realloc_r \
-Wl,-wrap,_calloc_r
