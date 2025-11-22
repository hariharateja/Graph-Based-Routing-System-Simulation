import json
import random
import math
import os
from typing import List, Dict, Any, Tuple


POI_TAGS: List[str] = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
ROAD_TYPES: List[str] = ["primary", "secondary", "tertiary", "local", "expressway"]

MAX_SPEED_MPS: float = 36.0
SECONDS_PER_DAY: int = 86400
SLOTS_PER_DAY: int = 96
SECONDS_PER_SLOT: int = 900 # 15 minutes

global_edge_id_counter: int = 10000 


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculates the distance between two points in meters using Haversine formula."""
    R_earth = 6371000.0  # Earth radius in meters
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R_earth * c

def generate_speed_profile(road_type: str) -> List[float]:
    """Creates a realistic 96-slot speed profile based on road type characteristics."""
    # Base speeds and deviation ranges (in m/s)
    speed_params = {
        "expressway": (28.0, 7.0),
        "primary": (20.0, 5.0),
        "secondary": (15.0, 4.0),
        "tertiary": (10.0, 3.0),
        "local": (6.0, 2.0)
    }
    
    base_speed, max_dev = speed_params.get(road_type, (15.0, 4.0))

    profile: List[float] = []
    for i in range(SLOTS_PER_DAY):
        hour = i * 15 // 60
        
        if 8 <= hour <= 10 or 17 <= hour <= 19:
            
            speed_factor = random.uniform(0.5, 0.7) 
        else:
            # Normal traffic
            speed_factor = random.uniform(0.8, 1.2)

        speed = base_speed * speed_factor
        
        # Ensure speed is reasonable, avoid negative or zero
        profile.append(max(1.0, round(speed, 2))) 
        
    return profile

def generate_synthetic_graph(num_nodes: int, node_density: float = 0.05, g: int = 0) -> Dict[str, Any]:
    """Generates a graph with random geographic coordinates and guaranteed node connections."""
    
    global global_edge_id_counter
    nodes: List[Dict[str, Any]] = []
    
    # Mumbai area coordinates used for realistic scale
    BASE_LAT, BASE_LON = 19.07, 72.87 
    
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

    edges: List[Dict[str, Any]] = []
    used_connections: set[Tuple[int, int]] = set()

    for i in range(num_nodes):
        
        num_neighbors = max(1, int(num_nodes * node_density))
        
        for _ in range(num_neighbors):
            v = random.randint(0, num_nodes - 1)
            if i == v: continue
            
            u, v_sorted = min(i, v), max(i, v)
            if (u, v_sorted) in used_connections: continue
            used_connections.add((u, v_sorted))
            
            road_type = random.choice(ROAD_TYPES)
            
            # Using Haversine for accurate distance calculation
            length = haversine_distance(nodes[i]["lat"], nodes[i]["lon"], 
                                        nodes[v]["lat"], nodes[v]["lon"])
            
            length = max(50.0, min(1000.0, length))
            
            speed_profile = generate_speed_profile(road_type)
            
            avg_speed_mps = sum(speed_profile) / len(speed_profile)
            average_time = length / avg_speed_mps
            
            
            edges.append({
                "id": global_edge_id_counter,
                "u": i,
                "v": v,
                "length": round(length, 4),
                "average_time": round(average_time, 4),
                "speed_profile": speed_profile,
                "oneway": random.random() < 0.4, # 40% chance of being one-way
                "road_type": road_type
            })
            global_edge_id_counter += 1

    return {
        "meta": {
            "id": f"sample_testcase_{g}",
            "nodes": num_nodes,
            "description": "Instructor-provided map for course project"
        },
        "nodes": nodes,
        "edges": edges
    }


def generate_random_query(graph_data: Dict[str, Any], query_type: str, query_id: int) -> Dict[str, Any]:

    nodes = graph_data["nodes"]
    edges = graph_data["edges"]
    
    if not nodes or not edges:
        return {"id": query_id, "type": query_type, "error": "Graph data is empty"}

    query: Dict[str, Any] = {"id": query_id, "type": query_type}
    
    # Shortest Path Query 
    if query_type == "shortest_path":
        
        src_node, tgt_node = random.sample(nodes, 2)
        
        query["source"] = src_node["id"]
        query["target"] = tgt_node["id"]
        query["mode"] = random.choice(["distance", "time"])
        
        if random.random() < 0.4:
            constraints: Dict[str, Any] = {}
            if random.random() < 0.6: # 60% chance to add forbidden nodes
                size = random.randint(1, max(1, len(nodes) // 10))
                forbidden_nodes = [n["id"] for n in random.sample(nodes, size)]
                constraints["forbidden_nodes"] = forbidden_nodes
            if random.random() < 0.5: # 50% chance to add forbidden roads
                forbidden_roads = random.sample(ROAD_TYPES, random.randint(1, 2))
                constraints["forbidden_road_types"] = forbidden_roads
            
            if constraints:
                query["constraints"] = constraints

    # KNN Query 
    elif query_type == "knn":
        query["poi"] = random.choice(POI_TAGS)
        query_point = random.choice(nodes)
        
        query["query_point"] = {"lat": query_point["lat"], "lon": query_point["lon"]}
        query["metric"] = random.choice(["euclidean", "shortest_path"])
        query["k"] = random.randint(1, max(1, len(nodes) // 20)) # Small k value

    #  Remove Edge 
    elif query_type == "remove_edge":
        query["edge_id"] = random.choice(edges)["id"]

    #  Modify Edge (m)
    elif query_type == "modify_edge":
        edge_to_modify = random.choice(edges)
        query["edge_id"] = edge_to_modify["id"]
        
        patch: Dict[str, Any] = {}
        
        # Randomly choose fields to modify
        if random.random() < 0.4:
            patch["length"] = round(edge_to_modify["length"] * random.uniform(0.8, 1.2), 4)
        if random.random() < 0.4:
            patch["average_time"] = round(edge_to_modify["average_time"] * random.uniform(0.8, 1.2), 4)
        if random.random() < 0.4:
            patch["road_type"] = random.choice(ROAD_TYPES)
        if random.random() < 0.3:
            # Change speed profile
            rt = patch.get("road_type", edge_to_modify["road_type"])
            patch["speed_profile"] = generate_speed_profile(rt)

        if patch and random.random() > 0.05:
         query["patch"] = patch
    else:
        if random.random() < 0.5:
            query["patch"] = {} 
        else:
            pass


    return query


if __name__ == "__main__":
    
    TEST_SCENARIOS = [
        (20, 50), 
        (50 ,200),   
        (100, 300),
        (500, 1500),  
        (1000,2500)
    ]
    

    QUERY_SEQUENCE = [
        "shortest_path", "knn", "remove_edge", "shortest_path", 
        "modify_edge", "shortest_path", "shortest_path", "remove_edge", 
        "knn", "modify_edge", "shortest_path", "shortest_path"
    ]
    
    base_dir = "./tests"
    os.makedirs(base_dir, exist_ok=True)
    i=1
    for r_idx, (num_nodes, num_edges) in enumerate(TEST_SCENARIOS):
        folder_name = f"{base_dir}/test{i}"
        os.makedirs(folder_name, exist_ok=True)
        print(f"Generating Test Case {r_idx + 1}: {num_nodes} nodes, {num_edges} edges...")

        # 1. Generate Graph Data
        graph_data = generate_synthetic_graph(num_nodes,g=i)
        i=i+1
        # 2. Write Graph File
        output_file_g = f"{folder_name}/graph.json"
        with open(output_file_g, "w", encoding="utf-8") as f:
            json.dump(graph_data, f, ensure_ascii=False, indent=2)

        # 3. Generate Queries
        queries = {"meta": {"id": f"qset{r_idx + 1}"}, "events": []}
        
        query_id_counter = 1
        for q_type in QUERY_SEQUENCE:
            query = generate_random_query(graph_data, q_type, query_id_counter)
            queries["events"].append(query)
            query_id_counter += 1
            
        # 4. Write Query File
        output_file_q = f"{folder_name}/queries.json"
        with open(output_file_q, "w", encoding="utf-8") as f:
            json.dump(queries, f, ensure_ascii=False, indent=2)
            
        print(f"  -> Generated {len(queries['events'])} queries successfully.")