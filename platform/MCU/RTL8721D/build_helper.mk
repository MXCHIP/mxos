EXTRA_POST_BUILD_TARGETS += postbuild

ADD_MD5_SCRIPT := $(SOURCE_ROOT)/mxos/platform/MCU/RTL8721D/scripts/add_md5.py

NM_OUTPUT_FILE := $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.nm)

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
	$(CAT) $(SOURCE_ROOT)/mxos/platform/MCU/RTL8721D/image/km0_image2_all.bin $(BIN_OUTPUT_FILE) > $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin)
endif

	$(CP) $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin) $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.ota.bin)
	$(PYTHON) $(ADD_MD5_SCRIPT) $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.ota.bin)
	
ifeq ($(HOST_OS),Win32)

download: postbuild
	@echo windows don't support jlink download ...
# echo yes|./mxos/platform/MCU/RTL8721D/flashloader/flashloader.sh $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin) $(OPENOCD_FULL_NAME) >$(OUTPUT_DIR)/flashloader.log 2>&1
else	

	@echo Downloading
	echo yes|./mxos/platform/MCU/RTL8721D/flashloader/flashloader.sh $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin) $(OPENOCD_FULL_NAME) >$(OUTPUT_DIR)/flashloader.log 2>&1
endif