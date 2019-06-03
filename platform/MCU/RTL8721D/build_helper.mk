EXTRA_POST_BUILD_TARGETS += postbuild

ADD_MD5_SCRIPT := $(SOURCE_ROOT)/mxos/platform/MCU/RTL8721D/scripts/add_md5.py

NM_OUTPUT_FILE := $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.nm)
GEN_COMMON_BIN_OUTPUT_FILE_SCRIPT:= $(SCRIPTS_PATH)/gen_common_bin_output_file.py
ALL_BIN_OUTPUT_FILE :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all$(BIN_OUTPUT_SUFFIX))
OTA_BIN_OUTPUT_FILE :=$(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.ota$(BIN_OUTPUT_SUFFIX))


KM0_BOOT_FILE    := mxos/platform/MCU/RTL8721D/image/km0_boot_all.bin
KM0_BOOT_OFFSET  := 0x0

KM4_BOOT_FILE    := mxos/platform/MCU/RTL8721D/image/km4_boot_all.bin
KM4_BOOT_OFFSET  := 0x4000

ATE_FILE    := mxos/platform/MCU/RTL8721D/image/ate.bin
ATE_OFFSET  := 0x6000

OTA_OFFSET  := 0x188000

ifeq ($(HOST_OS),Win32)

postbuild: build_done
	$(NM) $(LINK_OUTPUT_FILE) | sort > $(NM_OUTPUT_FILE)
	$(OBJCOPY) -j .ram_image2.entry -j .ram_image2.text -j .ram_image2.data -Obinary $(STRIPPED_LINK_OUTPUT_FILE) $(OUTPUT_DIR)/binary/ram_2.bin
	$(OBJCOPY) -j .xip_image2.text -Obinary $(STRIPPED_LINK_OUTPUT_FILE) $(OUTPUT_DIR)/binary/xip_image2.bin
	$(PYTHON) mxos/platform/MCU/RTL8721D/scripts/prepend_header.py $(OUTPUT_DIR)/binary/ram_2.bin __ram_image2_text_start__ $(NM_OUTPUT_FILE)
	$(PYTHON) mxos/platform/MCU/RTL8721D/scripts/prepend_header.py $(OUTPUT_DIR)/binary/xip_image2.bin __flash_text_start__ $(NM_OUTPUT_FILE)
	$(CAT) $(OUTPUT_DIR_WIN)\binary\xip_image2_prepend.bin $(OUTPUT_DIR_WIN)\binary\ram_2_prepend.bin > $(BIN_OUTPUT_FILE_WIN)
	$(PYTHON) mxos/platform/MCU/RTL8721D/scripts/pad.py $(BIN_OUTPUT_FILE)
	$(CAT) .\\mxos\platform\MCU\RTL8721D\image\km0_image2_all.bin $(BIN_OUTPUT_FILE_WIN) > $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin)
else

postbuild: build_done
	$(NM) $(LINK_OUTPUT_FILE) | sort > $(NM_OUTPUT_FILE)
	$(OBJCOPY) -j .ram_image2.entry -j .ram_image2.text -j .ram_image2.data -Obinary $(STRIPPED_LINK_OUTPUT_FILE) $(OUTPUT_DIR)/binary/ram_2.bin
	$(OBJCOPY) -j .xip_image2.text -Obinary $(STRIPPED_LINK_OUTPUT_FILE) $(OUTPUT_DIR)/binary/xip_image2.bin
	$(PYTHON) mxos/platform/MCU/RTL8721D/scripts/prepend_header.py $(OUTPUT_DIR)/binary/ram_2.bin __ram_image2_text_start__ $(NM_OUTPUT_FILE)
	$(PYTHON) mxos/platform/MCU/RTL8721D/scripts/prepend_header.py $(OUTPUT_DIR)/binary/xip_image2.bin __flash_text_start__ $(NM_OUTPUT_FILE)
	$(CAT) $(OUTPUT_DIR)/binary/xip_image2_prepend.bin $(OUTPUT_DIR)/binary/ram_2_prepend.bin > $(BIN_OUTPUT_FILE)
	$(PYTHON) mxos/platform/MCU/RTL8721D/scripts/pad.py $(BIN_OUTPUT_FILE)
	$(CAT) $(SOURCE_ROOT)/mxos/platform/MCU/RTL8721D/image/km0_image2_all.bin $(BIN_OUTPUT_FILE) > $(OTA_BIN_OUTPUT_FILE)

	$(PYTHON) $(ADD_MD5_SCRIPT) $(OTA_BIN_OUTPUT_FILE)
	$(QUIET)$(RM) $(ALL_BIN_OUTPUT_FILE)
	$(PYTHON) $(GEN_COMMON_BIN_OUTPUT_FILE_SCRIPT) -o $(ALL_BIN_OUTPUT_FILE) -f $(KM0_BOOT_OFFSET)   $(KM0_BOOT_FILE)              
	$(PYTHON) $(GEN_COMMON_BIN_OUTPUT_FILE_SCRIPT) -o $(ALL_BIN_OUTPUT_FILE) -f $(KM4_BOOT_OFFSET)   $(KM4_BOOT_FILE)
	$(PYTHON) $(GEN_COMMON_BIN_OUTPUT_FILE_SCRIPT) -o $(ALL_BIN_OUTPUT_FILE) -f $(ATE_OFFSET)        $(ATE_FILE) 
	$(PYTHON) $(GEN_COMMON_BIN_OUTPUT_FILE_SCRIPT) -o $(ALL_BIN_OUTPUT_FILE) -f $(OTA_OFFSET)        $(OTA_BIN_OUTPUT_FILE) 

	$(QUIET)$(call MKDIR, $(BUILD_DIR)/eclipse_debug/)
	$(QUIET)$(CP) $(LINK_OUTPUT_FILE) $(BUILD_DIR)/eclipse_debug/last_built.elf
endif
download: postbuild
ifeq ($(HOST_OS),Win32)
	@echo windows don't support jlink download ...
# echo yes|./mxos/platform/MCU/RTL8721D/flashloader/flashloader.sh $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin) $(OPENOCD_FULL_NAME) >$(OUTPUT_DIR)/flashloader.log 2>&1
else	
	@echo Downloading
	# download total
	#echo yes|./mxos/platform/MCU/RTL8721D/flashloader/flashloader_all.sh $(OTA_BIN_OUTPUT_FILE) $(OPENOCD_FULL_NAME) >$(OUTPUT_DIR)/flashloader.log 2>&1
	# download application only 
	echo yes|./mxos/platform/MCU/RTL8721D/flashloader/flashloader.sh $(OTA_BIN_OUTPUT_FILE) $(OPENOCD_FULL_NAME) >$(OUTPUT_DIR)/flashloader.log 2>&1
endif
