CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -Wno-unused-parameter -Wno-unused-variable -Wno-sign-compare

INCLUDES = -IPhase-1 -IPhase-2 -IPhase-3 -Iinclude

# --- Source Files ---
PHASE1_SOURCES = $(wildcard ./Phase-1/*.cpp)
PHASE2_SOURCES = $(wildcard ./Phase-2/*.cpp)
PHASE3_SOURCES = $(wildcard ./Phase-3/*.cpp)
PHASE1_LIB = $(filter-out ./Phase-1/main.cpp ./Phase-1/processquery.cpp, $(PHASE1_SOURCES))

# --- Targets ---
.PHONY: all phase1 phase2 phase3 clean

all: phase1 phase2 phase3

# Target for Phase 1 executable
phase1: $(PHASE1_SOURCES) 
	$(CXX) $(CXXFLAGS) $(INCLUDES)  $(PHASE1_SOURCES) -o phase1

# Target for Phase 2 executable
phase2: $(PHASE2_SOURCES) 
	$(CXX) $(CXXFLAGS) $(INCLUDES)  $(PHASE2_SOURCES) $(PHASE1_LIB) -o phase2

# Target for Phase 3 executable
phase3: $(PHASE3_SOURCES) 
	$(CXX) $(CXXFLAGS) $(INCLUDES)  $(PHASE3_SOURCES) $(PHASE1_LIB) -o phase3

clean:
	rm -f phase1 phase2 phase3