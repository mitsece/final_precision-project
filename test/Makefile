# Makefile for Verilog simulation
# Supports both RTL and Gate-Level (GL) simulation

# Configuration
TOPLEVEL = tb
SRC_DIR = ../src
TEST_DIR = .

# Random seed (can be overridden from command line)
RANDOM_SEED ?= 42

# Verilog sources - Tiny Tapeout project
RTL_SOURCES = \
    $(SRC_DIR)/project.v

# For GL simulation, use the synthesized netlist from Tiny Tapeout
GL_SOURCES = \
    ../runs/wokwi/results/final/verilog/gl/tt_um_precision_farming.v
