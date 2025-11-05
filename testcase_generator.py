import xml.etree.ElementTree as ET
import json


def parse_osm(osm_file):
    tree = ET.parse(osm_file)
    root = tree.getroot()

    data = {
        "meta" : {},
        "nodes": [],
        "edges": []
    }

    main_tags = ["amenity", "shop", "tourism", "natural"]

    # Parse all <node> elements
    for node in root.findall("node"):
        node_data = {
            "id": node.attrib.get("id"),
            "lat": node.attrib.get("lat"),
            "lon": node.attrib.get("lon"),
            "pois": []
        }
        # Extract tags within the node (like amenity, name, etc.)
        for tag in node.findall("tag"):
            if tag.attrib["k"] in main_tags:
                node_data["pois"].append(tag.attrib["v"])

        data["nodes"].append(node_data)

    # Parse all <way> elements
    for way in root.findall("way"):
        way_data = {
            "id": way.attrib.get("id"),
            "nodes": [],
            "tags": {}
        }
        for nd in way.findall("nd"):
            way_data["nodes"].append(nd.attrib["ref"])
        for tag in way.findall("tag"):
            way_data["tags"][tag.attrib["k"]] = tag.attrib["v"]

        data["ways"].append(way_data)

    # Parse all <relation> elements
    for rel in root.findall("relation"):
        rel_data = {
            "id": rel.attrib.get("id"),
            "u": None,
            "v": None,
            "oneway": False,
            "average_time": None,
        }
        for member in rel.findall("member"):
            rel_data["members"].append({
                "type": member.attrib.get("type"),
                "ref": member.attrib.get("ref"),
                "role": member.attrib.get("role", "")
            })
        for tag in rel.findall("tag"):
            rel_data["tags"][tag.attrib["k"]] = tag.attrib["v"]

        data["relations"].append(rel_data)

    return data


if __name__ == "__main__":
    input_file = ".osm"   # path to your OSM XML file
    output_file = "output.json"

    osm_data = parse_osm(input_file)

    # ✏️ You can modify 'osm_data' here to your desired JSON format before writing

    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(osm_data, f, ensure_ascii=False, indent=2)

    print(f"Converted {input_file} → {output_file}")
