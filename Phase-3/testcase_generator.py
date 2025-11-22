import json
import random
import math
import os
from typing import List, Dict, Any, Tuple

# --- CONSTANTS & CONFIG ---
POI_TAGS: List[str] = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
ROAD_TYPES: List[str] = ["primary", "secondary", "tertiary", "local", "expressway"]
MAX_SPEED_MPS: float = 36.0
SLOTS_PER_DAY: int = 96

global_edge_id_counter: int = 10000 

# --- GEOMETRY HELPERS ---

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
    """Creates a realistic 96-slot speed profile based on road type."""
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
        # Rush hour simulation (8-10 AM, 5-7 PM)
        if 8 <= hour <= 10 or 17 <= hour <= 19:
            speed_factor = random.uniform(0.5, 0.7) 
        else:
            speed_factor = random.uniform(0.8, 1.2)
        speed = base_speed * speed_factor
        profile.append(max(1.0, round(speed, 2))) 
    return profile

def project_coordinates(lat: float, lon: float, base_lat: float, base_lon: float) -> Tuple[float, float]:
    """
    Approximates X/Y in meters relative to a base point.
    Essential for the Euclidean Heuristic in C++ to work on Geo-data.
    """
    # 1 deg lat ~= 111km
    # 1 deg lon ~= 111km * cos(lat)
    y = (lat - base_lat) * 111000
    x = (lon - base_lon) * 111000 * math.cos(math.radians(base_lat))
    return round(x, 2), round(y, 2)

# --- GENERATION FUNCTIONS ---

def generate_synthetic_graph(num_nodes: int, node_density: float = 0.05, g_id: int = 0) -> Dict[str, Any]:
    """Generates a graph with random geographic coordinates and guaranteed node connections."""
    global global_edge_id_counter
    nodes: List[Dict[str, Any]] = []
    
    # Center around Mumbai for realism
    BASE_LAT, BASE_LON = 19.07, 72.87 
    
    for i in range(num_nodes):
        # Generate points within ~5km range
        lat = BASE_LAT + (random.random() - 0.5) * 0.05
        lon = BASE_LON + (random.random() - 0.5) * 0.05
        
        # Project to X/Y for VRP Euclidean Heuristic
        x, y = project_coordinates(lat, lon, BASE_LAT, BASE_LON)

        pois = []
        if random.random() < 0.35: 
            pois = random.sample(POI_TAGS, random.randint(1, 2))
            
        nodes.append({
            "id": i,
            "lat": round(lat, 6),
            "lon": round(lon, 6),
            "x": x, # Added for VRP
            "y": y, # Added for VRP
            "pois": pois
        })

    edges: List[Dict[str, Any]] = []
    # Ensure connectivity via a spanning tree approach first
    connected = {0}
    unconnected = list(range(1, num_nodes))
    random.shuffle(unconnected)
    
    # 1. Create Spanning Tree (Guarantees connectivity)
    for u in unconnected:
        v = random.choice(list(connected))
        # Add edge u-v
        length = haversine_distance(nodes[u]["lat"], nodes[u]["lon"], 
                                    nodes[v]["lat"], nodes[v]["lon"])
        road_type = random.choice(ROAD_TYPES)
        speed_profile = generate_speed_profile(road_type)
        avg_speed_mps = sum(speed_profile) / len(speed_profile)
        average_time = length / avg_speed_mps

        edges.append({
            "id": global_edge_id_counter,
            "u": u, "v": v,
            "length": round(length, 4),
            "average_time": round(average_time, 4),
            "speed_profile": speed_profile,
            "oneway": False, 
            "road_type": road_type
        })
        global_edge_id_counter += 1
        # Add reverse for undirected behavior in VRP
        edges.append({
            "id": global_edge_id_counter,
            "u": v, "v": u,
            "length": round(length, 4),
            "average_time": round(average_time, 4),
            "speed_profile": speed_profile,
            "oneway": False, 
            "road_type": road_type
        })
        global_edge_id_counter += 1
        connected.add(u)

    # 2. Add random additional edges based on density
    target_edges = int(num_nodes * (1.0 + node_density * 10))
    
    while len(edges) // 2 < target_edges: # divide by 2 because we add bi-directional
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        if u == v: continue
        
        length = haversine_distance(nodes[u]["lat"], nodes[u]["lon"], 
                                    nodes[v]["lat"], nodes[v]["lon"])
        
        # Don't connect nodes that are too far (keep graph sparse/realistic)
        if length > 1500: continue 

        road_type = random.choice(ROAD_TYPES)
        speed_profile = generate_speed_profile(road_type)
        avg_speed_mps = sum(speed_profile) / len(speed_profile)
        average_time = length / avg_speed_mps

        # Forward
        edges.append({
            "id": global_edge_id_counter,
            "u": u, "v": v,
            "length": round(length, 4),
            "average_time": round(average_time, 4),
            "speed_profile": speed_profile,
            "oneway": False,
            "road_type": road_type
        })
        global_edge_id_counter += 1
        
        # Backward
        edges.append({
            "id": global_edge_id_counter,
            "u": v, "v": u,
            "length": round(length, 4),
            "average_time": round(average_time, 4),
            "speed_profile": speed_profile,
            "oneway": False,
            "road_type": road_type
        })
        global_edge_id_counter += 1

    return {
        "meta": {
            "id": f"testcase_vrp_{g_id}",
            "nodes": num_nodes,
            "description": "VRP Phase 3 Map"
        },
        "nodes": nodes,
        "edges": edges
    }

def generate_vrp_query(num_nodes: int, num_drivers: int, num_orders: int , g :int) -> Dict[str, Any]:
    """Generates the Fleet and Order configuration for Phase 3."""
    
    # Fleet Configuration
    fleet = {
        "depot_node": random.randint(0, num_nodes - 1),
        "num_delievery_guys": num_drivers
    }
    
    # Orders Configuration
    orders = []
    for i in range(num_orders):
        pickup = random.randint(0, num_nodes - 1)
        dropoff = random.randint(0, num_nodes - 1)
        while pickup == dropoff:
            dropoff = random.randint(0, num_nodes - 1)
            
        orders.append({
            "order_id": i,
            "pickup": pickup,
            "dropoff": dropoff,
            # Optional: Add Research fields if needed
            "priority": 3.0 if random.random() < 0.2 else 1.0
        })
        
    # add a stable query id and per-order id values
    # for o in orders:
    #     o["id"] = f"q{g}_order{o['order_id']}"

    return {
        "id": f"qset_{g}",
        "meta": {
            "id": f"qset_{g}"
        },
        "fleet": fleet,
        "orders": orders
    }

# --- MAIN EXECUTION ---

if __name__ == "__main__":
    
    # (Nodes, Drivers, Orders)
    TEST_SCENARIOS = [
        (20, 2, 5),     # Tiny
        (50, 5, 20),    # Small
        (100, 10, 50),  # Medium
        (500, 20, 200), # Large
        (1000, 50, 500) # Stress Test
    ]
    
    base_dir = "./testcases"
    os.makedirs(base_dir, exist_ok=True)
    
    for i, (num_nodes, num_drivers, num_orders) in enumerate(TEST_SCENARIOS):
        scenario_idx = i + 1
        folder_name = f"{base_dir}/test{scenario_idx}"
        os.makedirs(folder_name, exist_ok=True)
        
        print(f"Generating Test Case {scenario_idx}: {num_nodes} Nodes, {num_drivers} Drivers, {num_orders} Orders...")

        # 1. Generate Graph
        graph_data = generate_synthetic_graph(num_nodes, node_density=0.08, g_id=scenario_idx)
        
        output_file_g = f"{folder_name}/graph.json"
        with open(output_file_g, "w", encoding="utf-8") as f:
            json.dump(graph_data, f, ensure_ascii=False, indent=2)

        # 2. Generate VRP Query (Fleet + Orders)
        vrp_query = generate_vrp_query(num_nodes, num_drivers, num_orders , scenario_idx)
        
        output_file_q = f"{folder_name}/queries.json"
        with open(output_file_q, "w", encoding="utf-8") as f:
            json.dump(vrp_query, f, ensure_ascii=False, indent=2)
            
        print(f"  -> Saved to {folder_name}/")