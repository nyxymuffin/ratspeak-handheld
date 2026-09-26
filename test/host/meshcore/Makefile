# Host unit tests for the pure parts of src/meshcore (no Arduino, no radio).
# Usage: make -C test/host/meshcore        (needs a host g++ with C++17)

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Werror
SRC_DIR := ../../../src/meshcore
CONFIG_DIR := ../../../src/core/config
BUILD := build

TESTS := $(BUILD)/test_radio_policy $(BUILD)/test_settings_rules

.PHONY: all test clean
all: test

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

$(BUILD)/%: %.cpp $(wildcard $(SRC_DIR)/*.h) $(CONFIG_DIR)/MeshCoreRules.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) -I$(CONFIG_DIR) $< -o $@

clean:
	rm -rf $(BUILD)
