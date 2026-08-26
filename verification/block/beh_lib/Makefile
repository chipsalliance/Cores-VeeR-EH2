
# Allow snapshot override
target = default_mt
snapshot = $(target)

VEER_CONFIG = ${RV_ROOT}/configs/veer.config

CONF_PARAMS ?= -set=build_axi4 -set=xlen=64

TEST_DIR = $(RV_ROOT)/verification/block/beh_lib
SNAPSHOT_DIR = $(TEST_DIR)/snapshots/$(snapshot)

INCLUDES = -I$(TEST_DIR)/snapshots/default_mt
SOURCES = $(TEST_DIR)/snapshots/default_mt/common_defines.vh \
		  $(TEST_DIR)/snapshots/default_mt/eh2_pdef.vh \
		  $(RV_ROOT)/design/include/eh2_def.sv \
		  $(RV_ROOT)/design/lib/beh_lib.sv \
		  $(RV_ROOT)/verification/block/beh_lib/rvrangecheck_tb.sv

ifdef DEBUG
 VERILATOR_DEBUG = --trace
endif

# Check for RV_ROOT
ifeq (,$(wildcard ${VEER_CONFIG}))
$(error env var RV_ROOT does not point to a valid dir! Exiting!)
endif

# If define files do not exist, then run veer.config.
$(SNAPSHOT_DIR)/defines.h:
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS)

$(TEST_DIR)/verilator-build: clean $(SNAPSHOT_DIR)/defines.h
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary -Wno-WIDTHTRUNC $(INCLUDES) $(SOURCES) --top-module rvrangecheck_tb
	touch $(TEST_DIR)/verilator-build

rvrangecheck-sim: $(TEST_DIR)/verilator-build
	$(TEST_DIR)/obj_dir/Vrvrangecheck_tb $(VERILATOR_DEBUG)

test-all: rvrangecheck-sim

clean:
	rm -rf $(TEST_DIR)/obj_dir $(SNAPSHOT_DIR)
