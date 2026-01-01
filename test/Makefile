# Makefile for Verilog simulation
# Supports both RTL and Gate-Level (GL) simulation

# Configuration
TOPLEVEL = tb
SRC_DIR = ../src
TEST_DIR = .

# Random seed (can be overridden from command line)
RANDOM_SEED ?= 42

# Verilog sources (adjust paths to your design)
RTL_SOURCES = \
	$(SRC_DIR)/your_module.v

# For GL simulation, use the synthesized netlist
GL_SOURCES = \
	../runs/wokwi/results/final/verilog/gl/your_module.v

# Testbench
TB_SOURCES = $(TEST_DIR)/tb.v

# Simulation flags
IVERILOG_FLAGS = -g2012 -Wall
VVP_FLAGS = +RANDOM_SEED=$(RANDOM_SEED)

# Output files
RTL_VVP = tb_rtl.vvp
GL_VVP = tb_gl.vvp
VCD_FILE = tb.vcd

# Targets
.PHONY: all test test_gl clean view help

# Default target
all: test

# RTL simulation
test: $(RTL_VVP)
	@echo "========================================="
	@echo "Running RTL simulation with RANDOM_SEED=$(RANDOM_SEED)"
	@echo "========================================="
	vvp $(RTL_VVP) $(VVP_FLAGS) +trace
	@if [ $$? -eq 0 ]; then \
		echo "✓ RTL simulation PASSED"; \
		exit 0; \
	else \
		echo "✗ RTL simulation FAILED"; \
		exit 1; \
	fi

# Gate-level simulation
test_gl: $(GL_VVP)
	@echo "========================================="
	@echo "Running GL simulation with RANDOM_SEED=$(RANDOM_SEED)"
	@echo "========================================="
	@if [ ! -f "$(GL_SOURCES)" ]; then \
		echo "Error: GL netlist not found at $(GL_SOURCES)"; \
		echo "Run 'make gds' first to generate the netlist"; \
		exit 1; \
	fi
	vvp $(GL_VVP) $(VVP_FLAGS) +trace
	@if [ $$? -eq 0 ]; then \
		echo "✓ GL simulation PASSED"; \
		exit 0; \
	else \
		echo "✗ GL simulation FAILED"; \
		echo "Note: GL failures may indicate timing issues or synthesis problems"; \
		exit 1; \
	fi

# Compile RTL testbench
$(RTL_VVP): $(RTL_SOURCES) $(TB_SOURCES)
	@echo "Compiling RTL testbench..."
	iverilog $(IVERILOG_FLAGS) -o $(RTL_VVP) $(RTL_SOURCES) $(TB_SOURCES)

# Compile GL testbench
$(GL_VVP): $(GL_SOURCES) $(TB_SOURCES)
	@echo "Compiling GL testbench..."
	@if [ ! -f "$(GL_SOURCES)" ]; then \
		echo "Warning: GL sources not found, skipping GL compilation"; \
		exit 1; \
	fi
	iverilog $(IVERILOG_FLAGS) -o $(GL_VVP) $(GL_SOURCES) $(TB_SOURCES) \
		-I../runs/wokwi/results/final/verilog/gl

# View waveform
view: $(VCD_FILE)
	@if command -v gtkwave > /dev/null; then \
		gtkwave $(VCD_FILE) &; \
	else \
		echo "GTKWave not installed. Install with: sudo apt-get install gtkwave"; \
	fi

# Clean build artifacts
clean:
	@echo "Cleaning test artifacts..."
	rm -f $(RTL_VVP) $(GL_VVP) $(VCD_FILE) *.log results.xml

# Help
help:
	@echo "Available targets:"
	@echo "  make test        - Run RTL simulation (default)"
	@echo "  make test_gl     - Run gate-level simulation"
	@echo "  make view        - Open waveform viewer"
	@echo "  make clean       - Remove build artifacts"
	@echo ""
	@echo "Options:"
	@echo "  RANDOM_SEED=N    - Set random seed (default: 42)"
	@echo ""
	@echo "Examples:"
	@echo "  make test RANDOM_SEED=12345"
	@echo "  make test_gl"
