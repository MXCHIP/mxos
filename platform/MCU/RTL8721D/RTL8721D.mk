NAME := RTL8721D

# Host architecture is Cortex-M4
HOST_ARCH := Cortex-M33

KM4_ASDK_PATH := sdk/project/realtek_amebaD_cm4_gcc_verification/asdk/
$(NAME)_PREBUILT_LIBRARY := \
$(KM4_ASDK_PATH)image/objs.a \
$(KM4_ASDK_PATH)lib/application/lib_httpc.a \
$(KM4_ASDK_PATH)lib/application/lib_httpd.a \
$(KM4_ASDK_PATH)lib/application/lib_websocket.a \
$(KM4_ASDK_PATH)lib/application/lib_tftp.a \
$(KM4_ASDK_PATH)lib/application/lib_wlan.a \
$(KM4_ASDK_PATH)lib/application/lib_wps.a \
$(KM4_ASDK_PATH)image/cmse_implib.lib

DEFAULT_LINK_SCRIPT := rlx8721d_img2_ns.ld

GLOBAL_CFLAGS	+= -ffunction-sections -march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=softfp -mfpu=fpv5-sp-d16 -g -gdwarf-3 -nostartfiles -nodefaultlibs -nostdlib -O2 -D__FPU_PRESENT -gdwarf-3 -fstack-usage -fdata-sections -nostartfiles -nostdlib -Wall -Wpointer-arith -Wstrict-prototypes -Wundef -Wno-write-strings -Wno-maybe-uninitialized -Wextra
GLOBAL_CXXFLAGS	+= $(CPU_CFLAGS)
GLOBAL_ASMFLAGS	+= $(CPU_CFLAGS)
GLOBAL_LDFLAGS	+= -march=armv8-m.main+dsp -mthumb -mcmse -mfloat-abi=softfp -mfpu=fpv5-sp-d16 -nostartfiles -specs nosys.specs -Wl,--gc-sections -Wl,--warn-section-align -Wl,--build-id=none -Wl,--no-enum-size-warning -Wl,--warn-common -lm -lstdc++
GLOBAL_LDFLAGS	+= -L mxos/platform/MCU/RTL8721D

EXTRA_TARGET_MAKEFILES +=  mxos/platform/MCU/RTL8721D/build_helper.mk
