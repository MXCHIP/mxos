#
#  UNPUBLISHED PROPRIETARY SOURCE CODE
#  Copyright (c) 2016 MXCHIP Inc.
#
#  The contents of this file may not be disclosed to third parties, copied or
#  duplicated in any form, in whole or in part, without the prior written
#  permission of MXCHIP Corporation.
#

.PHONY: bootloader download_bootloader total download_dct download kill_openocd

EXTRA_PRE_BUILD_TARGETS  += bootloader
EXTRA_POST_BUILD_TARGETS += copy_output_for_eclipse

ifeq (download,$(findstring download,$(MAKECMDGOALS)))
EXTRA_POST_BUILD_TARGETS  += sflash_write_app
OPENOCD_LOG_FILE ?= $(SOURCE_ROOT)build/openocd_log.txt
DOWNLOAD_LOG := >> $(OPENOCD_LOG_FILE)
endif

#SFLASH_WRITER_APP Required by  download_apps
SFLASH_APP_PLATFROM_BUS := $(PLATFORM)

BOOTLOADER_TARGET := sub_build.bootloader@$(PLATFORM)@NoRTOS@release
BOOTLOADER_OUTFILE := $(BUILD_DIR)/$(BOOTLOADER_TARGET)/binary/$(BOOTLOADER_TARGET)
BOOTLOADER_OUTFILE_BIN := $(BUILD_DIR)/$(BOOTLOADER_TARGET)/binary/$(BOOTLOADER_TARGET).bin
BOOTLOADER_LOG_FILE ?= $(BUILD_DIR)/bootloader.log

ifeq (,$(and $(OPENOCD_PATH),$(OPENOCD_FULL_NAME)))
	$(error Path to OpenOCD has not been set using OPENOCD_PATH and OPENOCD_FULL_NAME)
endif

ifneq ($(VERBOSE),1)
BOOTLOADER_REDIRECT	= > $(BOOTLOADER_LOG_FILE)
endif

# Build bootloader itself
ifneq (,$(findstring bootloader, $(BUILD_STRING)))
BOOTLOADER_APP:=1
BUILD_BOOTLOADER:=0
else
# Target "total" not exist, no need to build bootloader
ifneq (total,$(findstring total,$(MAKECMDGOALS)))
BOOTLOADER_APP:=0
BOOTLOADER_SUB_BUILD:=0
else
include $(PLATFORM_DIRECTORY)/gen_standard_images.mk
BOOTLOADER_APP:=0
BOOTLOADER_SUB_BUILD:=1
endif #$(total,$(findstring total,$(MAKECMDGOALS)))
endif #$(findstring bootloader, $(BUILD_STRING))

ifeq ($(BOOTLOADER_SUB_BUILD),1)
bootloader:
	$(QUIET)$(ECHO) Building Bootloader...
	$(QUIET)$(MAKE) -r -f $(SOURCE_ROOT)mxos/makefiles/Makefile $(BOOTLOADER_TARGET) -I$(OUTPUT_DIR)  SFLASH= EXTERNAL_MXOS_GLOBAL_DEFINES=$(EXTERNAL_MXOS_GLOBAL_DEFINES) SUB_BUILD=bootloader $(BOOTLOADER_REDIRECT)
	$(QUIET)$(ECHO) Finished Building Bootloader
	$(QUIET)$(ECHO_BLANK_LINE)

download_bootloader: bootloader display_map_summary sflash_write_app
	$(eval BOOTLOADER_IMAGE_SIZE := $(shell $(PYTHON) $(IMAGE_SIZE_SCRIPT) $(BOOTLOADER_OUTFILE_BIN)))
	$(QUIET)$(ECHO) Downloading bootloader to partition: $(BOOTLOADER_FIRMWARE_PARTITION_TCL) size: $(BOOTLOADER_IMAGE_SIZE) bytes... 
	$(call CONV_SLASHES, $(OPENOCD_FULL_NAME)) -s $(SOURCE_ROOT) -f $(OPENOCD_CFG_PATH)interface/$(JTAG).cfg -f $(OPENOCD_CFG_PATH)$(HOST_OPENOCD)/$(HOST_OPENOCD).cfg -f $(MXOS_OS_PATH)/sub_build/spi_flash_write/sflash_write.tcl -c "sflash_write_file $(BOOTLOADER_OUTFILE_BIN) $(BOOTLOADER_FIRMWARE_PARTITION_TCL) 0x0 $(SFLASH_APP_PLATFROM_BUS) 0" -c shutdown $(DOWNLOAD_LOG) 2>&1 && $(ECHO) Download complete && $(ECHO_BLANK_LINE) || $(ECHO) Download failed	
	
#$(QUIET)$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD)-flash-app.cfg -c "verify_image_checksum $(BOOTLOADER_OUTFILE).stripped.elf" -c shutdown $(DOWNLOAD_LOG) 2>&1 && $(ECHO) No changes detected && $(ECHO_BLANK_LINE) || $(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD)-flash-app.cfg -c "flash write_image erase $(BOOTLOADER_OUTFILE).hex" -c shutdown $(DOWNLOAD_LOG) 2>&1 && $(ECHO) Download complete && $(ECHO_BLANK_LINE) || $(ECHO) "**** OpenOCD failed - ensure you have installed the driver from the drivers directory, and that the debugger is not running **** In Linux this may be due to USB access permissions. In a virtual machine it may be due to USB passthrough settings. Check in the task list that another OpenOCD process is not running. Check that you have the correct target and JTAG device plugged in. ****"

copy_bootloader_output_for_eclipse: build_done
	$(QUIET)$(call MKDIR, $(BUILD_DIR)/eclipse_debug/)
	$(QUIET)$(CP) $(BOOTLOADER_OUTFILE).elf $(BUILD_DIR)/eclipse_debug/last_bootloader.elf


else
ifeq (0,$(BOOTLOADER_APP))
bootloader:
	$(QUIET)$(ECHO) Skipping building bootloader due to \"total\" is not set
	$(QUIET)$(ECHO_BLANK_LINE)
	
download_bootloader: display_map_summary sflash_write_app
	$(QUIET)$(ECHO) Downloading Bootloader ...
	$(QUIET)$(ECHO) Skipping download bootloader due to \"total\" is not set
	$(QUIET)$(ECHO_BLANK_LINE)

else
bootloader:
	@:
	
download_bootloader:
	@:
endif

#copy_bootloader_output_for_eclipse: build_done
#	$(QUIET)$(call MKDIR, $(BUILD_DIR)/eclipse_debug/)
#	$(QUIET)$(CP) $(LINK_OUTPUT_FILE).elf $(BUILD_DIR)/eclipse_debug/last_bootloader.elf

copy_bootloader_output_for_eclipse:
	@:
endif

total:
	@:

download_app: $(STRIPPED_LINK_OUTPUT_FILE) display_map_summary download_bootloader sflash_write_app kill_openocd
	$(eval IMAGE_SIZE := $(shell $(PYTHON) $(IMAGE_SIZE_SCRIPT) $(BIN_OUTPUT_FILE)))
	$(QUIET)$(ECHO) Downloading application to partition: $(APPLICATION_FIRMWARE_PARTITION_TCL) size: $(IMAGE_SIZE) bytes... 
	$(call CONV_SLASHES, $(OPENOCD_FULL_NAME)) -s $(SOURCE_ROOT) -f $(OPENOCD_CFG_PATH)interface/$(JTAG).cfg -f $(OPENOCD_CFG_PATH)$(HOST_OPENOCD)/$(HOST_OPENOCD).cfg -c init -c flash_boot_check -c "flash_program $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.ota$(BIN_OUTPUT_SUFFIX)) 0x13200" -c shutdown $(DOWNLOAD_LOG) 2>&1 && $(ECHO) Download complete && $(ECHO_BLANK_LINE) || $(ECHO) Download failed. See build/openocd_log.txt for detail.

ifeq (download,$(filter download,$(MAKECMDGOALS)))
EXT_IMAGES_DOWNLOAD_DEP := download_app
endif

download: download_app $(if $(findstring total,$(MAKECMDGOALS)), EXT_IMAGE_DOWNLOAD,)

kill_openocd:
	$(KILL_OPENOCD)

$(if $(RTOS),,$(error No RTOS specified. Options are: $(notdir $(wildcard MXOS/RTOS/*))))

run: $(SHOULD_I_WAIT_FOR_DOWNLOAD)
	$(QUIET)$(ECHO) Resetting target
	$(QUIET)$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -c "log_output $(OPENOCD_LOG_FILE)" -s $(SOURCE_ROOT) -f $(OPENOCD_CFG_PATH)interface/$(JTAG).cfg -f $(OPENOCD_CFG_PATH)$(HOST_OPENOCD)/$(HOST_OPENOCD).cfg -c init -c soft_reset_halt -c resume -c shutdown $(DOWNLOAD_LOG) 2>&1 && $(ECHO) Target running

ifeq ($(BOOTLOADER_APP),1)
copy_output_for_eclipse: build_done 
	$(QUIET)$(call MKDIR, $(BUILD_DIR)/eclipse_debug/)
	$(QUIET)$(CP) $(LINK_OUTPUT_FILE) $(BUILD_DIR)/eclipse_debug/last_bootloader.elf
else
copy_output_for_eclipse: build_done copy_bootloader_output_for_eclipse
	$(QUIET)$(call MKDIR, $(BUILD_DIR)/eclipse_debug/)
	$(QUIET)$(CP) $(LINK_OUTPUT_FILE) $(BUILD_DIR)/eclipse_debug/last_built.elf
endif

debug: $(BUILD_STRING) $(SHOULD_I_WAIT_FOR_DOWNLOAD)
	$(QUIET)$(GDB_COMMAND) $(LINK_OUTPUT_FILE) -x .gdbinit_attach



