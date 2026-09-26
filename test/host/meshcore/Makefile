# Host unit tests for the pure parts of src/meshcore (no Arduino, no radio).
# Usage: make -C test/host/meshcore        (needs a host g++ with C++17)

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Werror
SRC_DIR := ../../../src/meshcore
CONFIG_DIR := ../../../src/core/config
CORE_DIR := ../../../src/core
BUILD := build

TESTS := $(BUILD)/test_radio_policy $(BUILD)/test_settings_rules $(BUILD)/test_tunnel_codec \
         $(BUILD)/test_tunnel_reassembler $(BUILD)/test_tunnel_policy $(BUILD)/test_tunnel_interface

# Pure sources a test links against, beyond its own file.
test_tunnel_reassembler_SOURCES := $(SRC_DIR)/TunnelReassembler.cpp
test_tunnel_policy_SOURCES := $(SRC_DIR)/TunnelPolicy.cpp
test_tunnel_interface_SOURCES := $(SRC_DIR)/TunnelInterface.cpp $(SRC_DIR)/TunnelPolicy.cpp $(SRC_DIR)/TunnelReassembler.cpp

.PHONY: all test clean
all: test

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

.SECONDEXPANSION:
$(BUILD)/%: %.cpp $$(%_SOURCES) $(wildcard $(SRC_DIR)/*.h) $(CONFIG_DIR)/MeshCoreRules.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) -I$(CONFIG_DIR) -I$(CORE_DIR) $< $($*_SOURCES) -o $@

clean:
	rm -rf $(BUILD)
