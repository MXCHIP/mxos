download: $(BIN_OUTPUT_FILE) display_map_summary
	$(eval IMAGE_SIZE := $(shell $(PYTHON) $(IMAGE_SIZE_SCRIPT) $(BIN_OUTPUT_FILE)))
	$(QUIET)$(ECHO) Downloading image, size: $(IMAGE_SIZE) bytes... 
	$(PYTHON) mxos/makefiles/scripts/flash_alg_progressbar/flash_alg_write.py -o $(OPENOCD_FULL_NAME) -f $(BIN_OUTPUT_FILE) -a 0x40000 -m mx1270
