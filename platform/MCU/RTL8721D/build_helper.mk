EXTRA_POST_BUILD_TARGETS += postbuild

NM_OUTPUT_FILE := $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.nm)
KM4_ASDK_PATH := mxos/platform/MCU/RTL8721D/sdk/project/realtek_amebaD_cm4_gcc_verification/asdk/

postbuild: build_done
	$(NM) $(LINK_OUTPUT_FILE) | sort > $(NM_OUTPUT_FILE)
	$(OBJCOPY) -j .ram_image2.entry -j .ram_image2.text -j .ram_image2.data -Obinary $(STRIPPED_LINK_OUTPUT_FILE) $(OUTPUT_DIR)/binary/ram_2.bin
	$(OBJCOPY) -j .xip_image2.text -Obinary $(STRIPPED_LINK_OUTPUT_FILE) $(OUTPUT_DIR)/binary/xip_image2.bin
	$(KM4_ASDK_PATH)gnu_utility/prepend_header.sh $(OUTPUT_DIR)/binary/ram_2.bin __ram_image2_text_start__ $(NM_OUTPUT_FILE)
	$(KM4_ASDK_PATH)gnu_utility/prepend_header.sh $(OUTPUT_DIR)/binary/xip_image2.bin __flash_text_start__ $(NM_OUTPUT_FILE)
	cat $(OUTPUT_DIR)/binary/xip_image2_prepend.bin $(OUTPUT_DIR)/binary/ram_2_prepend.bin > $(BIN_OUTPUT_FILE)
	$(KM4_ASDK_PATH)gnu_utility/pad.sh $(BIN_OUTPUT_FILE)
	cat $(KM4_ASDK_PATH)../../realtek_amebaD_cm0_gcc_verification/asdk/image/km0_image2_all.bin $(BIN_OUTPUT_FILE) > $(LINK_OUTPUT_FILE:$(LINK_OUTPUT_SUFFIX)=.all.bin)
