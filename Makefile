# Model 8270 pack-mesh: library, simulation, tests.
#   make test    build + run unit tests
#   make sim     build + run the 6-drone scenario
#   make check   tests + sim under ASan/UBSan
#   make audit   grep the flight library for banned constructs
CXX      ?= clang++
STD      := -std=c++17
WARN     := -Wall -Wextra -Wpedantic -Wshadow -Werror
# Flight-library rules: no exceptions, no RTTI.
FLIGHT   := -fno-exceptions -fno-rtti
INC      := -Iinclude
OPT      ?= -O2 -g

LIB_SRCS := $(wildcard src/*.cpp)
HDRS     := $(wildcard include/amc8270/mesh/*.hpp)

BUILD    := build
SAN      := build-san

LIB_OBJS     := $(patsubst src/%.cpp,$(BUILD)/%.o,$(LIB_SRCS))
SAN_LIB_OBJS := $(patsubst src/%.cpp,$(SAN)/%.o,$(LIB_SRCS))

CXXFLAGS     := $(STD) $(WARN) $(FLIGHT) $(INC) $(OPT)
SAN_CXXFLAGS := $(STD) $(WARN) $(FLIGHT) $(INC) -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=undefined

.PHONY: all test sim check audit clean
all: $(BUILD)/mesh_tests $(BUILD)/pack_sim

$(BUILD)/%.o: src/%.cpp $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(SAN)/%.o: src/%.cpp $(HDRS) | $(SAN)
	$(CXX) $(SAN_CXXFLAGS) -c $< -o $@

$(BUILD)/mesh_tests: tests/test_main.cpp $(LIB_OBJS) $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) tests/test_main.cpp $(LIB_OBJS) -o $@

$(BUILD)/pack_sim: sim/pack_sim.cpp $(LIB_OBJS) $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) sim/pack_sim.cpp $(LIB_OBJS) -o $@

$(SAN)/mesh_tests: tests/test_main.cpp $(SAN_LIB_OBJS) $(HDRS) | $(SAN)
	$(CXX) $(SAN_CXXFLAGS) tests/test_main.cpp $(SAN_LIB_OBJS) -o $@

$(SAN)/pack_sim: sim/pack_sim.cpp $(SAN_LIB_OBJS) $(HDRS) | $(SAN)
	$(CXX) $(SAN_CXXFLAGS) sim/pack_sim.cpp $(SAN_LIB_OBJS) -o $@

$(BUILD) $(SAN):
	mkdir -p $@

test: $(BUILD)/mesh_tests
	./$(BUILD)/mesh_tests

sim: $(BUILD)/pack_sim
	./$(BUILD)/pack_sim

check: $(SAN)/mesh_tests $(SAN)/pack_sim
	./$(SAN)/mesh_tests
	./$(SAN)/pack_sim > /dev/null

# The flight library (include/ + src/) must not use these (comments ignored).
BANNED := \b(new|delete|malloc|calloc|realloc|free|throw|try|catch|dynamic_cast|typeid)\b|std::(vector|string|map|unordered_map|list|deque|function|shared_ptr|unique_ptr|make_unique|make_shared)
audit:
	@bad=0; for f in $(HDRS) $(LIB_SRCS); do \
	  hits=$$(sed -e 's://.*$$::' $$f | grep -nE '$(BANNED)' || true); \
	  if [ -n "$$hits" ]; then echo "$$f:"; echo "$$hits"; bad=1; fi; \
	done; \
	if [ $$bad -eq 0 ]; then echo "audit: flight library clean"; else echo "audit: banned construct found"; exit 1; fi

clean:
	rm -rf $(BUILD) $(SAN)
