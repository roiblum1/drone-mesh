# Model 8270 pack-mesh: library, simulation, tests.
#   make test    build + run unit tests
#   make sim     build + run the 6-drone scenario
#   make check   tests + sim under ASan/UBSan
#   make audit   grep the flight library for banned constructs
#   make fuzz    libFuzzer + ASan/UBSan on the decoder and receive path (clang, FUZZ_SECONDS)
#   make fuzz-regress  replay the committed corpus through the harness (any compiler)
#   make corpus  regenerate the seed corpus in fuzz/corpus
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
FUZZ_OUT := build-fuzz

LIB_OBJS     := $(patsubst src/%.cpp,$(BUILD)/%.o,$(LIB_SRCS))
SAN_LIB_OBJS := $(patsubst src/%.cpp,$(SAN)/%.o,$(LIB_SRCS))

CXXFLAGS     := $(STD) $(WARN) $(FLIGHT) $(INC) $(OPT)
SAN_CXXFLAGS := $(STD) $(WARN) $(FLIGHT) $(INC) -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=undefined

.PHONY: all test sim check audit fuzz fuzz-regress corpus clean
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

$(BUILD) $(SAN) $(FUZZ_OUT):
	mkdir -p $@

# ---- fuzzing (fuzz/fuzz_packet.cpp). The library is compiled into the fuzzer
# so libFuzzer's coverage instrumentation covers it.
FUZZ_CXX     ?= clang++
FUZZ_SECONDS ?= 60
FUZZ_FLAGS   := $(STD) $(WARN) $(FLIGHT) $(INC) -O1 -g -fno-omit-frame-pointer -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=undefined

$(FUZZ_OUT)/fuzz_packet: fuzz/fuzz_packet.cpp $(LIB_SRCS) $(HDRS) | $(FUZZ_OUT)
	$(FUZZ_CXX) $(FUZZ_FLAGS) fuzz/fuzz_packet.cpp $(LIB_SRCS) -o $@

$(SAN)/fuzz_replay: fuzz/fuzz_packet.cpp fuzz/fuzz_main.cpp $(SAN_LIB_OBJS) $(HDRS) | $(SAN)
	$(CXX) $(SAN_CXXFLAGS) fuzz/fuzz_packet.cpp fuzz/fuzz_main.cpp $(SAN_LIB_OBJS) -o $@

$(BUILD)/make_corpus: fuzz/make_corpus.cpp $(LIB_OBJS) $(HDRS) | $(BUILD)
	$(CXX) $(CXXFLAGS) fuzz/make_corpus.cpp $(LIB_OBJS) -o $@

test: $(BUILD)/mesh_tests
	./$(BUILD)/mesh_tests

sim: $(BUILD)/pack_sim
	./$(BUILD)/pack_sim

check: $(SAN)/mesh_tests $(SAN)/pack_sim $(SAN)/fuzz_replay
	./$(SAN)/mesh_tests
	./$(SAN)/pack_sim > /dev/null
	./$(SAN)/fuzz_replay fuzz/corpus

# New inputs go to $(FUZZ_OUT)/corpus; fuzz/corpus (the committed seeds) is read only.
fuzz: $(FUZZ_OUT)/fuzz_packet
	mkdir -p $(FUZZ_OUT)/corpus $(FUZZ_OUT)/artifacts
	./$(FUZZ_OUT)/fuzz_packet -max_total_time=$(FUZZ_SECONDS) -dict=fuzz/mesh.dict -timeout=10 \
	  -artifact_prefix=$(FUZZ_OUT)/artifacts/ -print_final_stats=1 $(FUZZ_OUT)/corpus fuzz/corpus

fuzz-regress: $(SAN)/fuzz_replay
	./$(SAN)/fuzz_replay fuzz/corpus

corpus: $(BUILD)/make_corpus
	mkdir -p fuzz/corpus
	./$(BUILD)/make_corpus fuzz/corpus

# The flight library (include/ + src/) must not use these (comments ignored).
BANNED := \b(new|delete|malloc|calloc|realloc|free|throw|try|catch|dynamic_cast|typeid)\b|std::(vector|string|map|unordered_map|list|deque|function|shared_ptr|unique_ptr|make_unique|make_shared)
audit:
	@bad=0; for f in $(HDRS) $(LIB_SRCS); do \
	  hits=$$(sed -e 's://.*$$::' -e 's/= *delete//g' $$f | grep -nE '$(BANNED)' || true); \
	  if [ -n "$$hits" ]; then echo "$$f:"; echo "$$hits"; bad=1; fi; \
	done; \
	if [ $$bad -eq 0 ]; then echo "audit: flight library clean"; else echo "audit: banned construct found"; exit 1; fi

clean:
	rm -rf $(BUILD) $(SAN) $(FUZZ_OUT)
