import xml.etree.ElementTree as ET
import json

def dist(lat1, lon1, lat2, lon2):
    


def parse_osm(osm_file):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    data = {
        "meta" : {},
        "nodes": [],
        "edges": []
    }

    id_map = {}
    node_map = {}  # node_id → (lat, lon)
    index = 0

    place_tags = ["restaurant", "petrol station", "hospital", "pharmacy", "hotel", "atm"]
    road_tags = ["primary", "secondary", "tertiary", "local", "expressway"]



    avg_speeds = {
        "primary":,
        "secondary":,
        "tertiary":,
        "local":,
        "expressway":
    }



    
    # Parse all <node> elements
    for node in root.findall("node"):
        osm_id = node.attrib["id"]




        # further edits required

        if index >= 1e5: # need to make it somewhat randomized
            break  # stop at 100000 nodes



        id_map[osm_id] = index
        node_map[osm_id] = (round(float(node.attrib["lat"]), 6), round(float(node.attrib["lon"]), 6))
        index += 1

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
        
        # avg_time calculation from speeds map
        speed = avg_speeds[way_data["road_type"]]
        way_data["average_time"] = way_data["length"]/ speed

        data["edges"].append(way_data)

    return data


if __name__ == "__main__":
    input_file = "./Maps/map1.osm"
    output_file = "graph.json"

    osm_data = parse_osm(input_file)

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(osm_data, f, ensure_ascii=False, indent=2)
