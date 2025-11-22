import json
import random
import math
import os
from typing import List, Dict, Any
import shutil


# --------------------------------------------------------
# CONSTANTS
# --------------------------------------------------------

POI_TAGS = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
ROAD_TYPES = ["primary", "secondary", "tertiary", "local", "expressway"]
global_edge_id_counter = 10000

# speeds in kmph
avg_speeds = {
    "expressway": [22,23,24,25,26,27,28],
    "primary":[17,18,19,20,21,22],
    "secondary":[11,12,13,14,15,16,17],
    "tertiary":[8,9,10,11,12,13,14],
    "local": [6,7,8,9]
}

# --------------------------------------------------------
# UTILITY: HAVERSINE DISTANCE
# --------------------------------------------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000.0
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


# --------------------------------------------------------
# GRAPH GENERATION 
# --------------------------------------------------------

def generate_synthetic_graph(num_nodes: int, testcase_id: int):
    global global_edge_id_counter

    BASE_LAT, BASE_LON = 19.07, 72.87
    nodes = []

    # Generate nodes
    for i in range(num_nodes):
        lat = BASE_LAT + (random.random() - 0.5) * 0.05
        lon = BASE_LON + (random.random() - 0.5) * 0.05

        pois = []
        if random.random() < 0.35:
            pois = random.sample(POI_TAGS, random.randint(1, 2))

        nodes.append({
            "id": i,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "pois": pois
        })

    # Generate edges
    edges = []
    used_pairs = set()
    node_density = 0.06 if num_nodes <= 100 else 0.04

    for i in range(num_nodes):
        neighbors = max(1, int(num_nodes * node_density))

        for _ in range(neighbors):
            v = random.randint(0, num_nodes - 1)
            if v == i:
                continue

            u, vv = min(i, v), max(i, v)
            if (u, vv) in used_pairs:
                continue
            used_pairs.add((u, vv))

            road_type = random.choice(ROAD_TYPES)

            # Distance
            length = haversine_distance(
                nodes[i]["lat"], nodes[i]["lon"],
                nodes[v]["lat"], nodes[v]["lon"]
            )
            length = max(50.0, min(1500.0, length))

            speed = random.choice(avg_speeds[road_type])
            average_time = length/speed  

            edges.append({
                "id": global_edge_id_counter,
                "u": i,
                "v": v,
                "length": round(length, 3),
                "average_time" : round(average_time,4),
                "road_type": road_type,
                "oneway": random.random() < 0.4
            })
            global_edge_id_counter += 1

    return {
        "meta": {
            "id": f"phase2_graph_{testcase_id}",
            "nodes": num_nodes,
            "description": "Phase-2 auto-generated graph"
        },
        "nodes": nodes,
        "edges": edges
    }


# --------------------------------------------------------
# PHASE-2 QUERY GENERATION
# --------------------------------------------------------

def pick_two_nodes(graph):
    u, v = random.sample(graph["nodes"], 2)
    return u["id"], v["id"]


def make_ksp_exact(graph, qid):
    s, t = pick_two_nodes(graph)
    return {
        "type": "k_shortest_paths",
        "id": qid,
        "source": s,
        "target": t,
        "k": random.randint(2, 7),
        "mode": "distance"
    }


def make_ksp_heuristic(graph, qid):
    s, t = pick_two_nodes(graph)
    return {
        "type": "k_shortest_paths_heuristic",
        "id": qid,
        "source": s,
        "target": t,
        "k": random.randint(2, 7),
        "overlap_threshold": random.randint(20, 80)
    }


def make_approx_sp(graph, qid):
    pairs = []
    for _ in range(random.randint(2, 5)):
        s, t = pick_two_nodes(graph)
        pairs.append({"source": s, "target": t})

    return {
        "type": "approx_shortest_path",
        "id": qid,
        "queries": pairs,
        "time_budget_ms": random.choice([5, 10, 15, 20]),
        "acceptable_error_pct": random.choice([5.0, 10.0, 15.0])
    }


# --------------------------------------------------------
# GENERATE 6 TESTCASES
# --------------------------------------------------------

def generate_phase2_tests(output_dir: str):
    random.seed(42)

    TEST_NODE_SIZES = [50, 75, 100, 150, 300, 500]

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    for i in range(6):
        test_id = i + 1
        folder = os.path.join(output_dir, f"test{test_id}")
        os.makedirs(folder, exist_ok=True)

        num_nodes = TEST_NODE_SIZES[i]

        # ---- GRAPH ----
        graph = generate_synthetic_graph(num_nodes, test_id)
        with open(os.path.join(folder, "graph.json"), "w") as f:
            json.dump(graph, f, indent=2)

        # ---- 12 QUERIES ----
        QUERY_TYPES = ["ksp_exact", "ksp_heuristic", "approx_sp"]
        queries = {"meta": {"id": f"phase2_test_{test_id}"}, "events": []}
        qid = 1

        for _ in range(12):
            qt = random.choice(QUERY_TYPES)
            if qt == "ksp_exact":
                queries["events"].append(make_ksp_exact(graph, qid))
            elif qt == "ksp_heuristic":
                queries["events"].append(make_ksp_heuristic(graph, qid))
            else:
                queries["events"].append(make_approx_sp(graph, qid))
            qid += 1

        with open(os.path.join(folder, "queries.json"), "w") as f:
            json.dump(queries, f, indent=2)

        print(f"✔ Generated test{test_id}")


# --------------------------------------------------------
# MAIN
# --------------------------------------------------------

if __name__ == "__main__":
    generate_phase2_tests("Phase-2/testcases")
    print("All 6 testcases generated in folder: testcases/")
