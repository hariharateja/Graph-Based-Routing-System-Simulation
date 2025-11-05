import os
import xml.etree.ElementTree as ET
import json
import random
from math import radians, sin, cos, atan2, sqrt


testcases = [20, 30, 50, 100, 300, 500, 1000, 2500, 5000, 100000]

place_tags = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]

road_tags = ["primary", "secondary", "tertiary", "local", "expressway"]

avg_speeds = {
    "expressway": [22,23,24,25,26,27,28],
    "primary":[17,18,19,20,21,22],
    "secondary":[11,12,13,14,15,16,17],
    "tertiary":[8,9,10,11,12,13,14],
    "local": [6,7,8,9]
}

mode = ["distance", "time"]

metric = ["euclidean", "shortest_path"]

query_order = ["s", "knn", "r", "s", "m", "s", "m", "r", "s", "knn"]


def dist(lat1, lon1, lat2, lon2):
    R_e = 6371000 # from website Adda247
    phi1 = radians(lat1)
    phi2 = radians(lat2)
    dphi = radians(lat2 - lat1)
    dx = radians(lon2 - lon1)
    k = sin(dphi/2)**2 + cos(phi1) * cos(phi2) * sin(dx/2)**2
    return 2 * R_e * atan2(sqrt(k),sqrt(1-k))


def create_graph(osm_file, n, r):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    data = {
        "meta" : {},
        "nodes": [],
        "edges": []
    }

    data["meta"]["id"] = f"sample_testcase_{r}"
    data["meta"]["nodes"] = n
    data["meta"]["description"] = f"sample_set_{r}"

    node_map = {}  # node_id → (lat, lon)
    index = 0
    
    # Parse all <node> elements
    for node in root.findall("node"):
        osm_id = node.attrib["id"]


        if index >= n:
            break  # stop at 'n' nodes


        node_map[osm_id] = (round(float(node.attrib["lat"]), 6), round(float(node.attrib["lon"]), 6))

        node_data = {
            "id": index,
            "lat": round(float(node.attrib["lat"]), 6),
            "lon": round(float(node.attrib["lon"]), 6),
            "pois": []
        }
        
        # if the places are relevant
        for tag in node.findall("tag"):
            if tag.attrib["v"] in place_tags:
                node_data["pois"].append(tag.attrib["v"])

        index += 1
        data["nodes"].append(node_data)

    # Parse all <way> elements
    for way in root.findall("way"):
        way_data = {
            "id": way.attrib.get("id"),
            "u": None,
            "v": None,
            "length": 0.0,
            "average_time": 0.0,
            "oneway": False,
            "road_type": None
        }

        # check if valid edges wrt end nodes
        nd_refs = [nd.attrib["ref"] for nd in way.findall("nd")]
        if len(nd_refs) >= 2:
            u = nd_refs[0]
            v = nd_refs[-1]
        if u in node_map and v in node_map:
            lat1, lon1 = node_map[u]
            lat2, lon2 = node_map[v]
            way_data["length"] = dist(lat1, lon1, lat2, lon2)
        else:
            continue

        # check for tags first since we are finding avg time of edge wrt road_type
        for tag in way.findall("tag"):
            if tag.attrib["k"] == "oneway":
                way_data["oneway"] = True
            if tag.attrib["k"] == "highway" and tag.attrib["v"] in road_tags:
                way_data["road_type"] = tag.attrib["v"]
        
        if way_data["road_type"] == None:
            continue

        # avg_time calculation from speeds map
        speed = random.choice(avg_speeds[way_data["road_type"]])
        way_data["average_time"] = way_data["length"]/ speed

        data["edges"].append(way_data)

    return data


if __name__ == "__main__":
    input_file = "./Maps/map1.osm"

    for r in range(6):
        folder = f"./testcases/test{r+1}"
        os.makedirs(folder, exist_ok=True)

        output_file_g = f"{folder}/graph.json"
        osm_data = create_graph(input_file,testcases[r], r+1)
        with open(output_file_g, "w", encoding="utf-8") as f:
            json.dump(osm_data, f, ensure_ascii=False, indent=2)

        output_file_q = f"{folder}/query.json"
        queries = {}
        queries["meta"]["id"] = f"qset{r+1}"

        for e in range(10):
            query = generate_query(osm_data, query_order[e], e+1)
            queries["events"].append(query)
        with open(output_file_q, "w", encoding="utf-8") as f:
                    json.dump(osm_data, f, ensure_ascii=False, indent=2)

def generate_query(graph_data, type, id):
    
    nodes = graph_data["nodes"]
    edges = graph_data["edges"]

    query = {
        "id": id,
        "type": None;
    }

    if type == "s":
        query["type"] = "shortest_path"
        query["source"] = random.choice(nodes)
        query["target"] = random.choice(nodes)
        query["mode"] = random.choice(mode)

        if random.randint(1,1000)%2:
            query["constraints"] = {}
            if random.randint(1,1000)%2:
                query["constraints"]["forbidden_nodes"] = generate_forbidden_nodes(nodes)
            if random.randint(1,1000)%2:
                query["constraints"]["forbidden_road_types"] = generate_forbidden_road_types(road_tags)
    
    elif type == "knn":
        query["type"] = type
        query["poi"] = random.choice(place_tags)
        query_point = random.choice(nodes)
        query["query_point"]["lat"] = query_point["lat"]
        query["query_point"]["lon"] = query_point["lon"]
        query["metric"] = random.choice(metric)
        query["k"] = random.randint(1, len(nodes)-1)
    
    elif type == "r":
        query["type"] = "remove_edge"
        edge_ids = [e["id"] for e in edges]
        query["edge_id"] = random.choice(edge_ids)

    else:
        query["type"] = "modify_edge"
        edge_ids = [e["id"] for e in edges]
        query["edge_id"] = random.choice(edge_ids)
        
        if random.randint(1, 1000)%2:
            query["patch"] = {}
            if random.randint(1, 1000)%2:
                query["patch"]["length"] = random.randint() + random.rand()
            if random.randint(1, 1000)%2:
                query["patch"]["average_time"] = random.randint() + random.rand()
            if random.randint(1, 1000)%2:
                query["patch"]["road_type"] = random.choice(road_tags)
            if random.randint(1, 1000)%2:
                query["patch"]["speed_profile"] = random





