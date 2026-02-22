import os
import re
import xml.etree.ElementTree as ET

"""
This script converts a URDF file to a MuJoCo MJCF XML format.
Requirements: Meshes folder must always be in the same directory as this file. Otherwise,
input path and output path can be specified in the following global variables.

Run the file as:
cd src/external_pkgs/RoboSuite/robosuite/models/assets/robots/rover2025/
python urdf2mjcfROVER.py
"""

INPUT_URDF_PATH = "input.urdf"
OUTPUT_MJCF_PATH = "robot.xml"

def parse_urdf(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    return root

def create_mjcf_structure():
    mjcf = ET.Element("mujoco", attrib={"model": "converted_urdf"})

    # Default parameters
    default = ET.SubElement(mjcf, "default")
    ET.SubElement(default, "joint", attrib={"limited": "false", "damping": "0.01", "armature": "0.01", "frictionloss": "0.01"})
    ET.SubElement(default, "geom", attrib={"condim": "4", "contype": "1", "conaffinity": "1"})

    # Arm joint default
    arm_joint = ET.SubElement(default, "default", attrib={"class": "arm_joint"})
    ET.SubElement(arm_joint, "joint", attrib={"type": "hinge", "limited": "true"})

    # Collision default
    collision = ET.SubElement(default, "default", attrib={"class": "collision"})
    ET.SubElement(collision, "geom", attrib={"group": "0", "rgba": "0.7 0.7 0.7 1", "type": "mesh"})

    # Visual default
    visual = ET.SubElement(default, "default", attrib={"class": "visual"})
    ET.SubElement(visual, "geom", attrib={"group": "1", "material": "default_material", "type": "mesh"})

    # Compiler setup
    compiler_comment = ET.Comment(" compiler set to radians and use rpy with euler ")
    mjcf.append(compiler_comment)
    ET.SubElement(mjcf, "compiler", attrib={"angle": "radian", "eulerseq": "xyz"})

    # Asset section
    asset_comment = ET.Comment(" mesh names ")
    mjcf.append(asset_comment)
    asset = ET.SubElement(mjcf, "asset")
    ET.SubElement(asset, "material", attrib={"name": "default_visual", "rgba": "0.7 0.7 0.7 1"})
    ET.SubElement(asset, "material", attrib={"name": "default_material", "rgba": "0.7 0.7 0.7 1"})

    return mjcf

def add_meshes_to_asset(asset, urdf_root):
    mesh_files = set()
    for mesh in urdf_root.findall(".//mesh"):
        filename = mesh.attrib.get("filename", "")
        if not filename:
            continue

        base = filename.split("/")[-1]
        name = base.rsplit(".", 1)[0]

        clean_file = filename.replace("package://", "").replace(" ", "_")
        if "/meshes/" in clean_file:
            clean_file = clean_file.split("/meshes/", 1)[1]
            clean_file = os.path.join("meshes", clean_file)

        root, ext = os.path.splitext(clean_file)
        clean_file = root + ext.lower()

        if name not in mesh_files and clean_file:
            mesh_files.add(name)
            ET.SubElement(asset, "mesh", attrib={"name": name, "file": clean_file})
    return mesh_files

def find_child_joints(parent_name, urdf_root):
    joints = []
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        if parent == parent_name:
            joints.append(joint)
    return joints

def extract_origin(element):
    if element is None:
        return ("0 0 0", "0 0 0")
    xyz = element.attrib.get("xyz", "0 0 0")
    rpy = element.attrib.get("rpy", "0 0 0")
    return xyz, rpy

def create_body_recursive(parent_body, link_name, urdf_root, sibling_joint=None):
    link = urdf_root.find(f".//link[@name='{link_name}']")
    if link is None:
        return None

    joint_origin = sibling_joint.find("origin") if sibling_joint is not None else None
    xyz, rpy = extract_origin(joint_origin)

    body_attrib = {"name": link_name, "pos": xyz, "euler": rpy}
    body = ET.SubElement(parent_body, "body", attrib=body_attrib)

    if sibling_joint is not None:
        axis_tag = sibling_joint.find("axis")
        axis = axis_tag.attrib.get("xyz", "0 0 1") if axis_tag is not None else "0 0 1"

        joint_type = sibling_joint.attrib.get("type", "hinge")
        limit_tag = sibling_joint.find("limit")
        if joint_type == "continuous":
            joint_range = "-6.28319 6.28319"
        elif limit_tag is not None:
            lower = limit_tag.attrib.get("lower", "0")
            upper = limit_tag.attrib.get("upper", "0")
            joint_range = f"{lower} {upper}"
        else:
            joint_range = "0 0"

        ET.SubElement(body, "joint", attrib={
            "name": sibling_joint.attrib.get("name", f"{link_name}_joint"),
            "class": "arm_joint",
            "axis": axis,
            "range": joint_range
        })

    for geom_type in ["collision", "visual"]:
        mesh_tag = link.find(f"{geom_type}/geometry/mesh")
        if mesh_tag is not None:
            mesh_file = mesh_tag.attrib["filename"].split("/")[-1].split(".")[0]
            ET.SubElement(body, "geom", attrib={"class": geom_type, "mesh": mesh_file})

    inertial = link.find("inertial")
    if inertial is not None:
        origin = inertial.find("origin")
        mass_tag = inertial.find("mass")
        inertia_tag = inertial.find("inertia")

        pos = origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        mass = mass_tag.attrib.get("value", "1") if mass_tag is not None else "1"
        diaginertia = " ".join([
            inertia_tag.attrib.get("ixx", "0"),
            inertia_tag.attrib.get("iyy", "0"),
            inertia_tag.attrib.get("izz", "0")
        ]) if inertia_tag is not None else "0 0 0"

        ET.SubElement(body, "inertial", attrib={"pos": pos, "mass": mass, "diaginertia": diaginertia})

    for joint in find_child_joints(link_name, urdf_root):
        child_name = joint.find("child").attrib["link"]
        create_body_recursive(body, child_name, urdf_root, joint)

    return body


def find_leaf_links(urdf_root):
    parents = {joint.find("parent").attrib.get("link") for joint in urdf_root.findall("joint")}
    children = {joint.find("child").attrib.get("link") for joint in urdf_root.findall("joint")}
    return sorted(children - parents)


def append_default_hand(mjcf_root, urdf_root):
    leaf_links = find_leaf_links(urdf_root)
    if not leaf_links:
        return

    target_link = leaf_links[0]
    target_body = mjcf_root.find(f".//body[@name='{target_link}']")
    if target_body is None:
        return

    existing = target_body.find(".//body[@name='right_hand']")
    if existing is not None:
        return

    hand_group = ET.SubElement(target_body, "body", attrib={"name": "hand"})
    right_hand = ET.SubElement(hand_group, "body", attrib={
        "name": "right_hand",
        "pos": "0 0.098 0",
        "quat": "0.707 -0.707 0 0"
    })
    ET.SubElement(right_hand, "camera", attrib={
        "mode": "fixed",
        "name": "eye_in_hand",
        "pos": "0.05 0 0",
        "quat": "0 0.707108 0.707108 0",
        "fovy": "75"
    })
    ET.SubElement(right_hand, "site", attrib={
        "name": "grip_site",
        "pos": "0 0.1 0",
        "size": "0.01",
        "rgba": "0 0 1 1"
    })
    ET.SubElement(right_hand, "site", attrib={
        "name": "right_center",
        "pos": "0 0.1 0",
        "size": "0.01",
        "rgba": "1 0 0 1"
    })

def urdf_to_mjcf(urdf_path, output_path):
    urdf_root = parse_urdf(urdf_path)
    mjcf = create_mjcf_structure()
    asset = mjcf.find("asset")
    add_meshes_to_asset(asset, urdf_root)

    # Root body
    worldbody = ET.SubElement(mjcf, "worldbody")

    # Assume first link is base_link
    base_link = urdf_root.find("link")
    if base_link is not None:
        base_name = base_link.attrib["name"]
        create_body_recursive(worldbody, base_name, urdf_root)

    # Actuators
    actuator = ET.SubElement(mjcf, "actuator")
    for joint in urdf_root.findall("joint"):
        name = joint.attrib["name"]
        ET.SubElement(actuator, "motor", attrib={
            "name": name,
            "joint": name,
            "ctrllimited": "true",
            "ctrlrange": "-150 150"
        })

    append_default_hand(mjcf, urdf_root)

    # Save XML with formatting that matches the hand-authored template
    formatted_xml = build_formatted_output(mjcf)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(formatted_xml)
    print(f"MJCF file written to {output_path}")


def indent_tree(elem, level=0, indent_str="    "):
    outer_indent = "\n" + indent_str * level
    inner_indent = outer_indent + indent_str
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = inner_indent
        for index, child in enumerate(elem):
            indent_tree(child, level + 1, indent_str)
            if not child.tail or not child.tail.strip():
                child.tail = inner_indent if index < len(elem) - 1 else outer_indent
        if not elem.tail or not elem.tail.strip():
            elem.tail = outer_indent
    else:
        if not elem.tail or not elem.tail.strip():
            elem.tail = outer_indent


def build_formatted_output(mjcf):
    # Add actuator comment before formatting
    actuator_comment_exists = any(
        isinstance(child.tag, str) is False and child.text and "actuator torques" in child.text
        for child in mjcf
    )
    if not actuator_comment_exists:
        actuator = mjcf.find("actuator")
        if actuator is not None:
            idx = list(mjcf).index(actuator)
            mjcf.insert(idx, ET.Comment(" actuator torques "))

    indent_tree(mjcf)
    xml_body = ET.tostring(mjcf, encoding="unicode")

    xml_body = xml_body.replace("</default>\n    <!-- compiler set", "</default>\n\n    <!-- compiler set")
    xml_body = xml_body.replace("</asset>\n    <worldbody>", "</asset>\n\n    <worldbody>")
    xml_body = xml_body.replace("</worldbody>\n    <!-- actuator torques -->", "</worldbody>\n\n    <!-- actuator torques -->")

    inertial_pattern = re.compile(
        r'(?P<prefix>\n)(?P<indent>[ \t]*)<inertial\s+pos="(?P<pos>[^"]*)"\s+mass="(?P<mass>[^"]*)"\s+diaginertia="(?P<diag>[^"]*)"\s*/>'
    )

    def format_inertial(match):
        prefix = match.group("prefix")
        indent = match.group("indent")
        attr_indent = indent + "    "
        return (
            f'{prefix}{indent}<inertial\n'
            f'{attr_indent}pos="{match.group("pos")}"\n'
            f'{attr_indent}mass="{match.group("mass")}"\n'
            f'{attr_indent}diaginertia="{match.group("diag")}"\n'
            f'{indent}/>'
        )

    xml_body = inertial_pattern.sub(format_inertial, xml_body)

    xml_body = xml_body.strip()
    return "<!-- This file is autogenerated. Feel free to edit. -->\n" + xml_body + "\n"

if __name__ == "__main__":
    urdf_to_mjcf(INPUT_URDF_PATH, OUTPUT_MJCF_PATH)
