CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

INCLUDES = -IPhase-1 -IPhase-2 -IPhase-3 -Iinclude

# --- Source Files ---
PHASE1_SOURCES = $(wildcard Phase-1/*.cpp)
PHASE2_SOURCES = $(wildcard Phase-2/*.cpp)
PHASE3_SOURCES = $(wildcard Phase-3/*.cpp)

# --- Targets ---
.PHONY: all phase1 phase2 phase3 clean

all: phase1 phase2 phase3

# Target for Phase 1 executable
phase1: $(PHASE1_SOURCES) SampleDriver.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -DPHASE=1 SampleDriver.cpp $(PHASE1_SOURCES) -o phase1

# Target for Phase 2 executable
phase2: $(PHASE2_SOURCES) SampleDriver.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -DPHASE=2 SampleDriver.cpp $(PHASE2_SOURCES) -o phase2

# Target for Phase 3 executable
phase3: $(PHASE3_SOURCES) SampleDriver.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -DPHASE=3 SampleDriver.cpp $(PHASE3_SOURCES) -o phase3

clean:
	rm -f phase1 phase2 phase3