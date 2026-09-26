# Host unit tests for the pure parts of src/meshcore (no Arduino, no radio).
# Usage: make -C test/host/meshcore        (needs a host g++ with C++17)

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O1 -Wall -Wextra -Werror
SRC_DIR := ../../../src/meshcore
BUILD := build

TESTS := $(BUILD)/test_radio_policy

.PHONY: all test clean
all: test

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

$(BUILD)/%: %.cpp $(wildcard $(SRC_DIR)/*.h)
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(SRC_DIR) $< -o $@

clean:
	rm -rf $(BUILD)
