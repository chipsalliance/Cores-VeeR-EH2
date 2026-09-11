# Allow snapshot override
target = default_mt
snapshot = $(target)

VEER_CONFIG = ${RV_ROOT}/configs/veer.config

CONF_PARAMS ?= -set=build_axi4

TEST_DIR = $(RV_ROOT)/verification/block/exu
SNAPSHOT_DIR = $(TEST_DIR)/snapshots/$(snapshot)

INCLUDES = -I$(TEST_DIR)/snapshots/default_mt
SOURCES = $(TEST_DIR)/snapshots/default_mt/common_defines.vh \
		  $(TEST_DIR)/snapshots/default_mt/eh2_pdef.vh \
		  $(RV_ROOT)/design/include/eh2_def.sv \
		  $(RV_ROOT)/design/lib/beh_lib.sv \
      	  $(RV_ROOT)/design/exu/eh2_exu_div_ctl.sv \
		  $(RV_ROOT)/verification/block/exu/eh2_exu_div_ctl_tb.sv

ifdef DEBUG
 VERILATOR_DEBUG = --trace
endif

ifdef TB_DEBUG
	DFLAG = -DDEBUG
endif

WFLAGS = -Wno-WIDTH

# Check for RV_ROOT
ifeq (,$(wildcard ${VEER_CONFIG}))
$(error env var RV_ROOT does not point to a valid dir! Exiting!)
endif

# 1-bit cheapsort (non-SRT) division module variant
div-cheapsortq-sim-xlen32: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=0 -set=div_bit=1 -set=xlen=32
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	echo "Running cheapshortq xlen=32 test"
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

div-cheapsortq-sim-xlen64: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=0 -set=div_bit=1 -set=xlen=64
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

# 1-bit SRT division module variant
div-1bit-srt-sim-xlen32: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=1 -set=xlen=32
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

div-1bit-srt-sim-xlen64: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=1 -set=xlen=64
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

# 2-bit SRT division module variant
div-2bit-srt-sim-xlen32: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=2 -set=xlen=32
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

div-2bit-srt-sim-xlen64: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=2 -set=xlen=64
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

# 3-bit SRT division module variant
div-3bit-srt-sim-xlen32: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=3 -set=xlen=32
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

div-3bit-srt-sim-xlen64: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=3 -set=xlen=64
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

# 4-bit SRT division module variant
div-4bit-srt-sim-xlen32: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=4 -set=xlen=32
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

div-4bit-srt-sim-xlen64: clean
	BUILD_PATH=$(SNAPSHOT_DIR) ${VEER_CONFIG} -target=$(target) $(CONF_PARAMS) -set=div_new=1 -set=div_bit=4 -set=xlen=64
	cd $(TEST_DIR) && verilator $(VERILATOR_DEBUG) --timing --binary $(INCLUDES) $(SOURCES) $(WFLAGS) $(DFLAG) --top-module eh2_exu_div_ctl_tb
	$(TEST_DIR)/obj_dir/Veh2_exu_div_ctl_tb $(VERILATOR_DEBUG)

test-all: div-cheapsortq-sim-xlen32 div-cheapsortq-sim-xlen64\
		  div-1bit-srt-sim-xlen32 div-1bit-srt-sim-xlen64 \
		  div-2bit-srt-sim-xlen32 div-2bit-srt-sim-xlen64 \
		  div-3bit-srt-sim-xlen32 div-3bit-srt-sim-xlen64 \
		  div-4bit-srt-sim-xlen32 div-4bit-srt-sim-xlen64

default: test-all

clean:
	rm -rf $(TEST_DIR)/obj_dir $(SNAPSHOT_DIR)
