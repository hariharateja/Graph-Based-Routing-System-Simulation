# 🗺️ CS293 2025 — Graph-Based Routing System Simulation

## 📖 Overview
This project simulates a **graph-based routing system**, inspired by platforms like Google Maps, Ola, and Zomato.  
It demonstrates how advanced **Data Structures and Algorithms (DSA)** can be applied to efficiently solve large-scale routing problems such as **shortest paths**, **k-nearest neighbors**, and **delivery scheduling**.

The system is implemented in **C++17**, organized in three progressive phases, and designed to be modular, scalable, and performance-oriented.

---

## 🚀 Objectives
- Implement efficient graph algorithms for routing and optimization.  
- Design modular APIs for dynamic graph operations and query processing.  
- Develop heuristic and approximate algorithms for real-world scalability.  
- Simulate delivery optimization using variants of the **Travelling Salesman Problem (TSP)**.

---

## 🧩 Project Phases

### **Phase 1 – Core Graph Algorithms**
Focus: Implement fundamental graph operations and shortest path computations.

#### 🔹 Features
- **Shortest Path Queries**
  - Minimize distance or time
  - Handle dynamic edge updates and constraints
- **K Nearest Neighbors (KNN)**
  - Based on Euclidean distance
  - Based on shortest path distance

#### 🧮 Algorithms Used
- **Dijkstra’s Algorithm**
- **A\*** (for time-dependent shortest paths)
- **KD-Tree / Linear Search** for KNN
- Graph updates using adjacency lists and hash maps

---

### **Phase 2 – Heuristic & Approximate Routing**
Focus: Design faster, approximate, and more diverse routing algorithms.

#### 🔹 Features
- **K Shortest Paths (Exact)** — using Yen’s or Eppstein’s algorithm  
- **K Shortest Paths (Heuristic)** — penalizes overlap with best paths  
- **Approximate Shortest Paths** — optimize for speed within an error tolerance

#### 🧮 Algorithms & Techniques
- Yen’s algorithm (exact k-shortest paths)
- Heuristic cost functions with tunable overlap/distance penalties
- Batch query optimization using multi-threading and priority queues

---

### **Phase 3 – Delivery Scheduling (TSP Variant)**
Focus: Simulate real-world delivery routing similar to **Zomato / Blinkit**.

#### 🔹 Problem
Given:
- `n` delivery agents starting from a depot  
- `m` pickup–delivery pairs  

Goal:
- Minimize **total delivery completion time**  
- Compare with **minimum maximum completion time**

#### 🧮 Techniques
- Nearest Neighbor and Insertion Heuristics  
- **Simulated Annealing / Genetic Algorithm** (for improved optimization)
- Graph preprocessing for Euclidean maps

---

## 📁 Directory Structure
ProjectRoot/
│
├── Phase-1/
│   ├── graph.hpp
│   ├── shortest_path.cpp
│   ├── knn.cpp
│
├── Phase-2/
│   ├── k_shortest_paths.cpp
│   ├── heuristic_routing.cpp
│
├── Phase-3/
│   ├── delivery_scheduler.cpp
│   ├── tsp_heuristics.cpp
│
├── SampleDriver.cpp
├── Makefile
└── Report.pdf
---

## ⚙️ Build & Run

### 🧱 Build
make phase1
make phase2
make phase3
