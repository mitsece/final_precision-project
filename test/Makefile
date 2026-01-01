# Makefile for Verilog simulation
# Supports both RTL and Gate-Level (GL) simulation

# Configuration
TOPLEVEL_LANG = verilog
SIM = icarus
WAVES = 1

# Test module
MODULE = test
TOPLEVEL = tb

# Source files
SRC_DIR = ../src
PROJECT_SOURCES = $(SRC_DIR)/project.v

# For gate-level simulation
GATE_LEVEL_NETLIST = gate_level_netlist.v
GATES ?= no

# Verilog sources
ifeq ($(GATES),yes)
    VERILOG_SOURCES = $(GATE_LEVEL_NETLIST) $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v
    COMPILE_ARGS += -DGL_TEST -DFUNCTIONAL -DUNIT_DELAY=\#1
else
    VERILOG_SOURCES = $(PROJECT_SOURCES) tb.v
endif

# Random seed
RANDOM_SEED ?= 42
PLUSARGS += +RANDOM_SEED=$(RANDOM_SEED)

# Cocotb settings
export COCOTB_REDUCED_LOG_FMT=1

# Include cocotb makefiles
include $(shell cocotb-config --makefiles)/Makefile.sim

# Clean target
clean::
	rm -rf __pycache__ sim_build *.vcd *.vvp results.xml
	rm -f tb.vvp

# Show configuration
.PHONY: show-config
show-config:
	@echo "GATES: $(GATES)"
	@echo "VERILOG_SOURCES: $(VERILOG_SOURCES)"
	@echo "RANDOM_SEED: $(RANDOM_SEED)"
