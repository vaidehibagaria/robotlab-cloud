#!/usr/bin/env python3
"""
pinocchio_urdf_to_rcm.py
URDF (+optional SDF) -> RCM using Pinocchio for kinematics.

ROS-first behavior:
- By default, this script listens to ROS 2 topics to fetch robot description:
  - /robot_description (URDF XML)
  - /robot_description_semantic (SRDF XML, optional)
- It does NOT fall back to any hard-coded robot.
- If topics are not received within the timeout, it exits with a clear message.

Examples:
    # ROS-only (recommended; no file paths)
    python urdf_to_rcm.py -o ur5_rcm.json

    # Provide explicit URDF path (optional override)
    python urdf_to_rcm.py /path/to/robot.urdf -o robot_rcm.json

    # Provide SDF to merge sensors (optional)
    python urdf_to_rcm.py /path/to/robot.urdf --sdf /path/to/model.sdf -o robot_rcm.json
"""
import argparse
import json
import math
import random
import xml.etree.ElementTree as ET
from pathlib import Path
import os

import numpy as np

# Try import pinocchio; user must have it installed
try:
    import pinocchio as pin
    from pinocchio.robot_wrapper import RobotWrapper
except Exception as e:
    raise SystemExit("pinocchio not available. Install it (conda install -c conda-forge pinocchio) and retry. Error: " + str(e))


# Keyword heuristics (same idea as before)
GRIPPER_KEYWORDS = ["grip", "gripper", "finger", "robotiq", "2f", "parallel", "hand", "preshape"]
# IMPORTANT: Do not include generic link names like base_link/base_footprint to avoid false mobile detection
WHEEL_KEYWORDS = ["wheel", "caster"]
SUCTION_KEYWORDS = ["suction", "vacuum", "pad", "sucker"]

# Semantic inference keywords for environment objects
OBJECT_CLASS_KEYWORDS = {
    "tote": ["tote", "box", "bin", "container", "basket", "crate", "grasp_box", "small_box", "item", "object"],
    "workstation": ["table", "desk", "workstation", "bench", "platform", "surface", "work_area"],
    "shelf": ["shelf", "rack", "storage", "cabinet", "bookcase", "shelving", "storage_unit"],
    "pallet": ["pallet", "skid", "platform", "base", "shipping_pallet"],
    "obstacle": ["wall", "barrier", "fence", "post", "column", "pole", "obstacle", "blocker"],
    "conveyor": ["conveyor", "belt", "transport", "line", "conveyor_belt"],
    "robot": ["robot", "arm", "manipulator", "gripper", "end_effector", "robotic_arm"],
    "sensor": ["camera", "lidar", "laser", "sensor", "detector", "scanner", "vision"],
    "fixture": ["fixture", "jig", "clamp", "holder", "mount", "tooling"],
    "sphere": ["sphere", "ball", "orb", "round", "spherical"],
    "cylinder": ["cylinder", "tube", "pipe", "rod", "cylindrical"],
    "cone": ["cone", "pyramid", "triangular", "conical"],
    "equipment": ["equipment", "machine", "device", "tool", "apparatus"],
    "furniture": ["furniture", "chair", "sofa", "cabinet", "furnishing"]
}

AFFORDANCE_KEYWORDS = {
    "pickable": ["box", "tote", "bin", "container", "grasp", "small", "item", "object", "sphere", "ball", "tool"],
    "placeable": ["table", "desk", "workstation", "shelf", "platform", "surface", "area", "workbench"],
    "movable": ["box", "tote", "bin", "container", "item", "object", "small", "sphere", "ball"],
    "obstacle": ["wall", "barrier", "fence", "post", "column", "pole", "large", "fixed", "immovable"],
    "storage": ["shelf", "rack", "cabinet", "storage", "bookcase", "shelving", "storage_unit"],
    "transport": ["conveyor", "belt", "transport", "line", "conveyor_belt"],
    "climbable": ["ladder", "step", "stairs", "climb", "ascend"],
    "sittable": ["chair", "seat", "bench", "stool", "sitting"],
    "graspable": ["handle", "grip", "grasp", "hold", "grab"],
    "pushable": ["cart", "trolley", "wagon", "push", "moveable"],
    "rotatable": ["wheel", "gear", "rotor", "spinner", "turnable"]
}

ZONE_KEYWORDS = {
    "inbound": ["inbound", "receiving", "dock", "entrance", "input", "delivery"],
    "outbound": ["outbound", "shipping", "dock", "exit", "output", "dispatch"], 
    "storage": ["storage", "warehouse", "shelf", "rack", "inventory", "warehouse_zone"],
    "workstation": ["workstation", "work", "table", "desk", "bench", "processing", "work_area"],
    "quality": ["quality", "inspection", "check", "test", "verify", "qc"],
    "packaging": ["packaging", "pack", "wrap", "seal", "label", "packing"],
    "assembly": ["assembly", "manufacturing", "production", "build", "construct"],
    "office": ["office", "desk", "admin", "administrative", "management"],
    "break_room": ["break", "lunch", "rest", "relaxation", "cafeteria"],
    "loading": ["loading", "unloading", "dock", "bay", "platform"],
    "maintenance": ["maintenance", "repair", "service", "tool_room", "workshop"]
}

def name_has_keywords(name, keywords):
    if not name:
        return False
    name = name.lower()
    return any(k in name for k in keywords)

def infer_object_semantics(obj_id, dimensions, position):
    """
    Automatically infer semantic properties for environment objects based on:
    - Object ID/name patterns
    - Geometric properties (size, shape)
    - Spatial context (position, height)
    """
    obj_id_lower = obj_id.lower() if obj_id else ""
    
    # Infer object class
    object_class = "unknown"
    for class_name, keywords in OBJECT_CLASS_KEYWORDS.items():
        if any(keyword in obj_id_lower for keyword in keywords):
            object_class = class_name
            break
    
    # Infer affordances based on class and size
    affordances = []
    for affordance, keywords in AFFORDANCE_KEYWORDS.items():
        if any(keyword in obj_id_lower for keyword in keywords):
            affordances.append(affordance)
    
    # Size-based affordance inference
    if dimensions and len(dimensions) >= 3:
        width, height, depth = dimensions[0], dimensions[1], dimensions[2]
        volume = width * height * depth
        
        # Small objects are likely pickable/movable
        if volume < 0.01:  # Less than 0.01 m³
            if "pickable" not in affordances:
                affordances.append("pickable")
            if "movable" not in affordances:
                affordances.append("movable")
        
        # Flat, horizontal surfaces are placeable
        if height < 0.1 and width > 0.3 and depth > 0.3:
            if "placeable" not in affordances:
                affordances.append("placeable")
        
        # Large objects are obstacles
        if volume > 1.0:  # Greater than 1 m³
            if "obstacle" not in affordances:
                affordances.append("obstacle")
    
    # Infer zone based on position and name
    zone = "general"
    if position and len(position) >= 3:
        x, y, z = position[0], position[1], position[2]
        
        # Zone inference based on position
        if x < -1.0:
            zone = "inbound"
        elif x > 1.0:
            zone = "outbound"
        elif 0.2 < z < 1.5:  # Mid-height objects
            zone = "workstation"
        elif z > 1.5:  # High objects
            zone = "storage"
    
    # Override zone based on name patterns
    for zone_name, keywords in ZONE_KEYWORDS.items():
        if any(keyword in obj_id_lower for keyword in keywords):
            zone = zone_name
            break
    
    # Infer additional properties
    properties = {}
    if dimensions and len(dimensions) >= 3:
        properties["volume_m3"] = round(dimensions[0] * dimensions[1] * dimensions[2], 4)
        properties["max_dimension_m"] = round(max(dimensions), 3)
        properties["min_dimension_m"] = round(min(dimensions), 3)
    
    if position and len(position) >= 3:
        properties["height_m"] = round(position[2], 3)
    
    # Color inference from name (basic)
    if any(color in obj_id_lower for color in ["red", "blue", "green", "yellow", "black", "white"]):
        for color in ["red", "blue", "green", "yellow", "black", "white"]:
            if color in obj_id_lower:
                properties["color"] = color
                break
    
    # Priority inference
    if "small" in obj_id_lower or "grasp" in obj_id_lower:
        properties["priority"] = "high"
    elif "large" in obj_id_lower or "obstacle" in obj_id_lower:
        properties["priority"] = "low"
    else:
        properties["priority"] = "medium"
    
    return {
        "class": object_class,
        "affordances": list(set(affordances)),  # Remove duplicates
        "zone": zone,
        "properties": properties
    }

def parse_link_names(urdf_path):
    """Parse URDF XML and return a list of link names."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    return [link.attrib.get('name', '') for link in root.findall('link')]

def parse_urdf_core(urdf_path):
    """Parse URDF to extract links, joints, inertials, transmissions, gazebo hints, materials/colors.
    Does NOT record mesh filenames."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Links: inertial, visuals (materials/colors only)
    links = {}
    for link in root.findall('link'):
        lname = link.attrib.get('name')
        inertial = link.find('inertial')
        inert = None
        if inertial is not None:
            mass_tag = inertial.find('mass')
            origin_tag = inertial.find('origin')
            inertia_tag = inertial.find('inertia')
            inert = {
                'mass': float(mass_tag.attrib.get('value', '0')) if mass_tag is not None else None,
                'origin': {
                    'xyz': [float(x) for x in origin_tag.attrib.get('xyz', '0 0 0').split()] if origin_tag is not None else [0.0, 0.0, 0.0],
                    'rpy': [float(x) for x in origin_tag.attrib.get('rpy', '0 0 0').split()] if origin_tag is not None else [0.0, 0.0, 0.0]
                },
                'inertia': {
                    'ixx': float(inertia_tag.attrib.get('ixx', '0')) if inertia_tag is not None else None,
                    'ixy': float(inertia_tag.attrib.get('ixy', '0')) if inertia_tag is not None else None,
                    'ixz': float(inertia_tag.attrib.get('ixz', '0')) if inertia_tag is not None else None,
                    'iyy': float(inertia_tag.attrib.get('iyy', '0')) if inertia_tag is not None else None,
                    'iyz': float(inertia_tag.attrib.get('iyz', '0')) if inertia_tag is not None else None,
                    'izz': float(inertia_tag.attrib.get('izz', '0')) if inertia_tag is not None else None,
                }
            }
        # materials/colors from visuals
        materials = []
        for v in link.findall('visual'):
            mat = v.find('material')
            if mat is not None:
                name = mat.attrib.get('name')
                color_tag = mat.find('color')
                rgba = None
                if color_tag is not None and 'rgba' in color_tag.attrib:
                    rgba = [float(x) for x in color_tag.attrib['rgba'].split()]
                materials.append({'name': name, 'rgba': rgba})
        links[lname] = {'inertial': inert, 'materials': materials}

    # Joints
    joints = {}
    parent_links = set()
    child_links = set()
    for joint in root.findall('joint'):
        jname = joint.attrib.get('name')
        jtype = joint.attrib.get('type')
        parent = joint.find('parent').attrib.get('link') if joint.find('parent') is not None else None
        child = joint.find('child').attrib.get('link') if joint.find('child') is not None else None
        parent_links.add(parent)
        child_links.add(child)
        origin_tag = joint.find('origin')
        axis_tag = joint.find('axis')
        limit_tag = joint.find('limit')
        mimic_tag = joint.find('mimic')
        dyn_tag = joint.find('dynamics')
        joints[jname] = {
            'type': jtype,
            'parent': parent,
            'child': child,
            'origin': {
                'xyz': [float(x) for x in origin_tag.attrib.get('xyz', '0 0 0').split()] if origin_tag is not None else [0.0, 0.0, 0.0],
                'rpy': [float(x) for x in origin_tag.attrib.get('rpy', '0 0 0').split()] if origin_tag is not None else [0.0, 0.0, 0.0],
            },
            'axis': [float(x) for x in axis_tag.attrib.get('xyz', '0 0 0').split()] if axis_tag is not None else None,
            'limits': {
                'lower': float(limit_tag.attrib.get('lower')) if (limit_tag is not None and 'lower' in limit_tag.attrib) else None,
                'upper': float(limit_tag.attrib.get('upper')) if (limit_tag is not None and 'upper' in limit_tag.attrib) else None,
                'velocity': float(limit_tag.attrib.get('velocity')) if (limit_tag is not None and 'velocity' in limit_tag.attrib) else None,
                'effort': float(limit_tag.attrib.get('effort')) if (limit_tag is not None and 'effort' in limit_tag.attrib) else None,
                'initial_position': float(limit_tag.attrib.get('initial_position')) if (limit_tag is not None and 'initial_position' in limit_tag.attrib) else 0.0,
            },
            'mimic': ({
                'joint': mimic_tag.attrib.get('joint'),
                'multiplier': float(mimic_tag.attrib.get('multiplier', '1.0')),
                'offset': float(mimic_tag.attrib.get('offset', '0.0')),
            } if mimic_tag is not None else None),
            'dynamics': ({
                'damping': float(dyn_tag.attrib.get('damping', '0.0')) if dyn_tag is not None else None,
                'friction': float(dyn_tag.attrib.get('friction', '0.0')) if dyn_tag is not None else None,
            }),
        }

    # Find root link
    all_links = set(links.keys())
    root_candidates = list(all_links - child_links)
    root_link = root_candidates[0] if root_candidates else None

    # Transmissions
    transmissions = []
    for tr in root.findall('transmission'):
        tname = tr.attrib.get('name')
        ttype = tr.find('type').text if tr.find('type') is not None else None
        joint_tag = tr.find('joint')
        actuator_tag = tr.find('actuator')
        hi_nodes = tr.findall('.//hardwareInterface')
        hw_ifaces = [n.text for n in hi_nodes if n is not None and n.text]
        transmissions.append({
            'name': tname,
            'type': ttype,
            'joint': joint_tag.attrib.get('name') if joint_tag is not None else None,
            'actuator': actuator_tag.attrib.get('name') if actuator_tag is not None else None,
            'hardware_interfaces': hw_ifaces,
        })

    # Gazebo / ros_control hints
    gazebo = []
    for gz in root.findall('gazebo'):
        ref = gz.attrib.get('reference') if 'reference' in gz.attrib else None
        plugins = []
        for pl in gz.findall('plugin'):
            plugins.append({
                'name': pl.attrib.get('name'),
                'filename': pl.attrib.get('filename'),
                'params': {c.tag: c.text for c in list(pl)}
            })
        gazebo.append({'reference': ref, 'plugins': plugins})

    return {
        'links': links,
        'joints': joints,
        'root_link': root_link,
        'transmissions': transmissions,
        'gazebo': gazebo,
    }

def rpy_to_rot(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # ZYX convention
    Rz = np.array([[cy, -sy, 0],[sy, cy, 0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx

def compose_transform(xyz, rpy):
    R = rpy_to_rot(rpy)
    t = np.array(xyz)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t
    return T

def compute_zero_pose_transforms(core):
    """Compute homogeneous transforms of each link in base (root) frame at zero joint values."""
    joints = core['joints']
    root = core['root_link']
    # build adjacency from parent link to (child link, joint)
    parent_to_children = {}
    for jname, j in joints.items():
        parent_to_children.setdefault(j['parent'], []).append((j['child'], jname))
    link_T = {root: np.eye(4)} if root else {}
    stack = [root] if root else []
    while stack:
        parent = stack.pop()
        for (child, jname) in parent_to_children.get(parent, []):
            j = joints[jname]
            T_joint = compose_transform(j['origin']['xyz'], j['origin']['rpy'])
            link_T[child] = link_T[parent] @ T_joint
            stack.append(child)
    return link_T

def compute_mass_and_com(core, link_T):
    total_mass = 0.0
    weighted_com = np.zeros(3)
    for lname, l in core['links'].items():
        inert = l.get('inertial')
        if not inert or inert.get('mass') in (None, 0):
            continue
        mass = inert['mass']
        # COM in link frame at inertial.origin
        com_local = np.array(inert['origin']['xyz'])
        T = link_T.get(lname, np.eye(4))
        com_world = (T[:3,:3] @ com_local) + T[:3,3]
        weighted_com += mass * com_world
        total_mass += mass
    com = (weighted_com / total_mass).tolist() if total_mass > 0 else None
    return total_mass if total_mass > 0 else None, com

def parse_srdf(srdf_path):
    try:
        tree = ET.parse(srdf_path)
    except Exception:
        return None
    root = tree.getroot()
    groups = []
    for g in root.findall('group'):
        gname = g.attrib.get('name')
        chains = []
        links = []
        joints = []
        for c in g.findall('chain'):
            chains.append({'base_link': c.attrib.get('base_link'), 'tip_link': c.attrib.get('tip_link')})
        for l in g.findall('link'):
            links.append(l.attrib.get('name'))
        for j in g.findall('joint'):
            joints.append(j.attrib.get('name'))
        groups.append({'name': gname, 'chains': chains, 'links': links, 'joints': joints})
    end_effectors = []
    for ee in root.findall('end_effector'):
        end_effectors.append({'name': ee.attrib.get('name'), 'parent_link': ee.attrib.get('parent_link')})
    virtual_joints = []
    for vj in root.findall('virtual_joint'):
        virtual_joints.append({'name': vj.attrib.get('name'), 'type': vj.attrib.get('type'), 'parent_frame': vj.attrib.get('parent_frame'), 'child_link': vj.attrib.get('child_link')})
    passive_joints = [pj.attrib.get('name') for pj in root.findall('passive_joint')]
    return {
        'groups': groups,
        'end_effectors': end_effectors,
        'virtual_joints': virtual_joints,
        'passive_joints': passive_joints,
    }

def detect_sensors(core, link_T):
    sensor_keywords = {
        'lidar': ['lidar','lds','scan','hokuyo','velodyne','rplidar','base_scan'],
        'camera_rgbd': ['realsense','kinect','azure','camera','rgbd','depth'],
        'imu': ['imu'],
    }
    sensors = []
    for lname in core['links'].keys():
        lname_lower = lname.lower()
        detected = None
        for s_type, keys in sensor_keywords.items():
            if any(k in lname_lower for k in keys):
                detected = s_type
                break
        if detected:
            T = link_T.get(lname, np.eye(4))
            pose = {'xyz': T[:3,3].tolist() if T is not None else [0,0,0]}
            sensors.append({'link': lname, 'type': detected, 'pose_in_base': pose, 'fov_hint': None})
    return sensors

def estimate_footprint_radius(link_T):
    if not link_T:
        return None
    pts = np.array([T[:3,3] for T in link_T.values()])
    pts2d = pts[:, :2]
    center = pts2d.mean(axis=0)
    radii = np.linalg.norm(pts2d - center, axis=1)
    return {'center_xy': center.tolist(), 'radius_m': float(radii.max())}

def _parse_sdf_float_list(text, expected_len=None):
    if text is None:
        return None
    try:
        vals = [float(x) for x in text.strip().split()]
        if expected_len is not None and len(vals) != expected_len:
            return None
        return vals
    except Exception:
        return None

def parse_sdf_sensors(sdf_path):
    """Parse SDF model to extract sensors with their parent links and optional poses/topics.

    Returns list of dicts: {link, type, name, pose_sdf:{xyz,rpy}|None, topic|None, source:'sdf'}
    """
    try:
        tree = ET.parse(sdf_path)
        root = tree.getroot()
    except Exception:
        return []

    # SDF may have namespaces; ElementTree findall with .// works regardless for simple tags
    sensors = []
    sensor_type_map = {
        'ray': 'lidar',
        'gpu_ray': 'lidar',
        'lidar': 'lidar',
        'camera': 'camera',
        'depth': 'camera_rgbd',
        'rgbd_camera': 'camera_rgbd',
        'imu': 'imu',
    }

    # Iterate all models → links → sensors
    for model in root.findall('.//model'):
        for link in model.findall('link'):
            lname = link.attrib.get('name', '')
            for sensor in link.findall('sensor'):
                raw_type = (sensor.attrib.get('type') or '').lower()
                s_type = sensor_type_map.get(raw_type)
                if not s_type:
                    continue
                s_name = sensor.attrib.get('name', '')

                pose = None
                pose_tag = sensor.find('pose')
                vals = _parse_sdf_float_list(pose_tag.text if pose_tag is not None else None)
                if vals and len(vals) >= 6:
                    pose = {"xyz": vals[0:3], "rpy": vals[3:6]}

                # Try to find a topic name from plugins (common patterns)
                topic = None
                for pl in sensor.findall('plugin'):
                    # <topicName>foo</topicName>
                    tn = pl.find('topicName')
                    if tn is not None and tn.text:
                        topic = tn.text.strip()
                        break
                    # <ros><namespace>..</namespace><remapping>..</remapping></ros>
                    ros = pl.find('ros')
                    if ros is not None:
                        t = ros.find('topic') or ros.find('remapping')
                        if t is not None and t.text:
                            topic = t.text.strip()
                            break

                sensors.append({
                    'link': lname,
                    'type': s_type,
                    'name': s_name,
                    'pose_sdf': pose,
                    'topic': topic,
                    'source': 'sdf'
                })

    return sensors

def _compose_pose_on_base(link_T, link_name, local_pose):
    """Compose local pose (xyz,rpy) on top of base_T_link to get sensor pose in base frame.
    Returns dict pose_in_base with xyz and rpy if possible, else minimal xyz.
    """
    T_link = link_T.get(link_name)
    if T_link is None:
        return {'xyz': None}
    if not local_pose:
        return {'xyz': T_link[:3,3].tolist()}
    T_local = compose_transform(local_pose.get('xyz', [0,0,0]), local_pose.get('rpy', [0,0,0]))
    T = T_link @ T_local
    # Extract rpy back from rotation if desired (approximate)
    xyz = T[:3,3].tolist()
    return {'xyz': xyz}

def detect_mobile_base_from_urdf_xml(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for joint in root.findall('joint'):
        jname = joint.attrib.get('name','')
        if name_has_keywords(jname, WHEEL_KEYWORDS):
            return True
    for link in root.findall('link'):
        lname = link.attrib.get('name','')
        if name_has_keywords(lname, WHEEL_KEYWORDS):
            return True
    return False

def find_leaf_links_from_xml(urdf_path, max_candidates=6):
    """
    Parse URDF XML and return candidate leaf links that look like end-effectors.
    This version uses name_has_keywords(...) correctly.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    parents = set()
    children = set()
    for joint in root.findall('joint'):
        parent = joint.find('parent').attrib.get('link')
        child = joint.find('child').attrib.get('link')
        parents.add(parent)
        children.add(child)
    # leaf links = links that are not parents of any joint
    leafs = [link.attrib.get('name') for link in root.findall('link') if link.attrib.get('name') not in parents]

    # filter leafs for typical end-effector names using the helper
    ee_keywords = ["ee", "tool", "wrist", "end", "gripper", "hand"]
    cands = [l for l in leafs if name_has_keywords(l, ee_keywords)]

    # fallback: if no obvious EE names, return up to max_candidates leaf links
    if not cands:
        cands = leafs[:max_candidates]
    return cands


def sample_fk_workspace(robot_wrapper, ee_frame_names, n_samples=500):
    """Sample joint configurations and gather ee positions using Pinocchio RobotWrapper."""
    model = robot_wrapper.model
    data = robot_wrapper.data
    q0 = pin.neutral(robot_wrapper.model) if hasattr(pin, 'neutral') else pin.randomConfiguration(model)
    nq = model.nq
    positions = {ee: [] for ee in ee_frame_names}
    # map frame names -> frame ids if frames exist
    frame_map = {}
    for f in model.frames:
        frame_map[f.name] = f.parent
    # Try to use model.getFrameId if frames were created for links
    for ee in ee_frame_names:
        try:
            fid = model.getFrameId(ee)
            frame_map[ee] = fid
        except Exception:
            frame_map[ee] = None
    # get joint limits if available
    lower = getattr(model, 'lowerPositionLimit', None)
    upper = getattr(model, 'upperPositionLimit', None)
    has_limits = (lower is not None and upper is not None)
    for _ in range(n_samples):
        if has_limits:
            q = pin.utils.rand(model.nq) if hasattr(pin.utils, 'rand') else pin.randomConfiguration(model)
            # attempt to sample within limits if arrays exist
            try:
                q = np.array([random.uniform(float(lower[i]), float(upper[i])) for i in range(model.nq)])
            except Exception:
                q = pin.randomConfiguration(model)
        else:
            q = pin.randomConfiguration(model)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        for ee in ee_frame_names:
            try:
                # prefer frame placement if available
                fid = model.getFrameId(ee)
                oMf = data.oMf[fid]
                pos = oMf.translation.tolist() if hasattr(oMf, 'translation') else [float(oMf[0,3]), float(oMf[1,3]), float(oMf[2,3])]
                positions[ee].append(pos)
            except Exception:
                # fallback: try to get joint placement for a joint with same name
                positions[ee].append([0.0, 0.0, 0.0])
    return positions

def positions_to_aabb(poses):
    arr = np.array(poses)
    if arr.size == 0:
        return None
    vmin = arr.min(axis=0).tolist()
    vmax = arr.max(axis=0).tolist()
    vol = float(np.prod(np.maximum(np.array(vmax) - np.array(vmin), 0.0)))
    return {"aabb_min": vmin, "aabb_max": vmax, "volume_estimate_m3": vol}

def build_rcm_with_pinocchio(urdf_path, samples=400, sdf_path: str = None, extra_sensors: list = None):
    urdf_path = str(Path(urdf_path).resolve())
    # RobotWrapper helps loading model + geometry (if available)
    try:
        robot = RobotWrapper.BuildFromURDF(urdf_path, [str(Path(urdf_path).parent)])
        model = robot.model
        data = robot.data
    except Exception:
        # fallback to direct model load
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        robot = None

    # parse xml for leaf links
    mobile = detect_mobile_base_from_urdf_xml(urdf_path)
    ee_candidates = find_leaf_links_from_xml(urdf_path)

    # Exclude wheel-like links from end effector candidates
    ee_candidates = [l for l in ee_candidates if not name_has_keywords(l, WHEEL_KEYWORDS)]

    # No mesh usage for now; omit link bounding boxes
    link_bboxes = {}

    # Parse additional URDF metadata
    core = parse_urdf_core(urdf_path)
    link_T = compute_zero_pose_transforms(core)
    total_mass, com = compute_mass_and_com(core, link_T)
    srdf_info = None
    srdf_path = str(Path(urdf_path).with_suffix('.srdf'))
    if Path(srdf_path).exists():
        srdf_info = parse_srdf(srdf_path)
    # Start with URDF-heuristic sensors
    sensors = detect_sensors(core, link_T)

    # If SDF provided, merge in explicit sensors from SDF (prefer explicit over heuristic)
    sdf_sensors = []
    if sdf_path:
        sdf_sensors = parse_sdf_sensors(sdf_path)
        for s in sdf_sensors:
            pose_in_base = _compose_pose_on_base(link_T, s['link'], s.get('pose_sdf'))
            sensors.append({
                'link': s['link'],
                'type': s['type'],
                'pose_in_base': pose_in_base,
                'topic': s.get('topic'),
                'source': 'sdf'
            })

    # Merge extra sensors from runtime/topic discovery
    if extra_sensors:
        for s in extra_sensors:
            sensors.append(s)

    # Deduplicate sensors by (type, link or topic)
    unique = {}
    for s in sensors:
        key = (s.get('type'), s.get('link'), s.get('topic'))
        if key not in unique:
            unique[key] = s
        else:
            # Prefer SDF-sourced entries
            if unique[key].get('source') != 'sdf' and s.get('source') == 'sdf':
                unique[key] = s
    sensors = list(unique.values())
    footprint = estimate_footprint_radius(link_T)

    # use pinocchio to sample FK if robot wrapper/model available and ee candidates exist
    workspaces = {}
    if ee_candidates:
        try:
            positions = sample_fk_workspace(robot if robot is not None else pin, ee_candidates, n_samples=samples)
            for ee, poses in positions.items():
                workspaces[ee] = positions_to_aabb(poses)
        except Exception as e:
            # if pinocchio FK fails, leave workspaces empty
            workspaces = {}
    # detect gripper by searching for gripper keywords in link names only (no meshes)
    has_gripper = False
    gripper_type = "none"
    evidence = []
    for ln in parse_link_names(urdf_path):
        if name_has_keywords(ln, GRIPPER_KEYWORDS):
            has_gripper = True
            gripper_type = "two_finger_or_parallel"
            evidence.append(f"link_name:{ln}")
            break

    # assemble primitives
    primitives = []
    if mobile:
        primitives.append({
            "primitive_id": "move_base",
            "signature": "move_base(goal:Pose)->Status",
            "preconditions": ["battery_level>0.2"],
            "effects": ["at(goal)"],
            "source": "urdf",
            "confidence": 0.95
        })
        # Low-level locomotion primitives for differential drive bases
        primitives.append({
            "primitive_id": "set_base_twist",
            "signature": "set_base_twist(vx:float, vy:float, wz:float)->Status",
            "preconditions": ["battery_level>0.2"],
            "effects": ["base_twist_set(vx,vy,wz)"],
            "source": "urdf",
            "confidence": 0.9
        })
        primitives.append({
            "primitive_id": "drive_straight",
            "signature": "drive_straight(distance_m:float, speed_mps:float)->Status",
            "preconditions": ["battery_level>0.2"],
            "effects": ["base_translated(distance_m)"],
            "source": "urdf",
            "confidence": 0.85
        })
        primitives.append({
            "primitive_id": "rotate_in_place",
            "signature": "rotate_in_place(yaw_rad:float, yaw_rate_rps:float)->Status",
            "preconditions": ["battery_level>0.2"],
            "effects": ["base_rotated(yaw_rad)"] ,
            "source": "urdf",
            "confidence": 0.85
        })
        primitives.append({
            "primitive_id": "stop",
            "signature": "stop()->Status",
            "preconditions": [],
            "effects": ["base_twist_set(0,0,0)"],
            "source": "urdf",
            "confidence": 0.95
        })
    if ee_candidates:
        primitives.append({
            "primitive_id": "move_arm_to_pose",
            "signature": "move_arm_to_pose(pose:Pose)->Status",
            "preconditions": ["within_workspace(pose)"],
            "effects": ["arm_at(pose)"],
            "source":"urdf",
            "confidence": 0.9
        })
        if has_gripper:
            primitives.append({
                "primitive_id":"grasp_object",
                "signature":"grasp_object(object_id)->Status",
                "preconditions":["object_within_reach","gripper_span_ok"],
                "effects":["object_attached_to_gripper"],
                "source":"urdf",
                "confidence":0.85
            })

    # basic dynamics from URDF inertias (if available)
    mass_known = total_mass is not None

    rcm = {
        "metadata": {"robot_name": model.name if hasattr(model, 'name') else Path(urdf_path).stem, "source": urdf_path, "generator_version":"rcm_pinocchio_v0.1"},
        "links": link_bboxes,
        "locomotion": {"type": "differential" if mobile else "static", "inferred_from": "urdf", "confidence": (0.95 if mobile else 0.85)},
        "end_effectors": ee_candidates,
        "has_gripper": has_gripper,
        "gripper_type": gripper_type,
        "gripper_evidence": evidence,
        "workspaces": workspaces,
        "primitives": primitives,
        "dynamics": {"mass_kg": total_mass if mass_known else None, "center_of_mass_in_base": com if mass_known else None, "mass_confidence": (0.9 if mass_known else 0.2)},
        "joints": core['joints'],
        "transmissions": core['transmissions'],
        "gazebo_plugins": core['gazebo'],
        "default_pose": {j: core['joints'][j]['limits'].get('initial_position', 0.0) for j in core['joints']},
        "kinematics": {
            "root_link": core['root_link'],
            "link_transforms_base": {ln: link_T[ln].tolist() for ln in link_T},
            "srdf": srdf_info
        },
        "sensors": sensors,
        "footprint": footprint,
        "link_semantics": {ln: {"materials": core['links'][ln]['materials']} for ln in core['links']},
        "safety_profile": {"emergency_stop_latency_ms": None, "hard_stops_supported": None, "monitoring_signals": []},
        "provenance": {"urdf_path": urdf_path, **({"sdf_path": str(Path(sdf_path).resolve())} if sdf_path else {})},
        "confidence_overall": 0.75
    }
    return rcm

def autodetect_urdf_sdf_from_ros() -> (str, str):
    """Attempt to autodetect URDF and SDF paths using ROS 2 package index and TURTLEBOT3_MODEL.

    Currently supports TurtleBot3 models out of the box.
    Returns (urdf_path, sdf_path) where either may be None if not found.
    """
    urdf_path = None
    sdf_path = None
    try:
        from ament_index_python.packages import get_package_share_directory
        model = os.environ.get('TURTLEBOT3_MODEL', 'burger').lower()
        # Try turtlebot3_description URDF
        try:
            desc_share = get_package_share_directory('turtlebot3_description')
            candidate_urdf = os.path.join(desc_share, 'urdf', f'turtlebot3_{model}.urdf')
            if Path(candidate_urdf).exists():
                urdf_path = candidate_urdf
        except Exception:
            pass
        # Try turtlebot3_gazebo SDF
        try:
            gz_share = get_package_share_directory('turtlebot3_gazebo')
            candidate_sdf = os.path.join(gz_share, 'models', f'turtlebot3_{model}', 'model.sdf')
            if Path(candidate_sdf).exists():
                sdf_path = candidate_sdf
        except Exception:
            pass
    except Exception:
        # ament_index not available
        pass
    return urdf_path, sdf_path

def collect_runtime_ros_data(timeout_sec: float = 15.0):
    """Collect URDF/SRDF and ROS graph info from ROS 2, and detect sensors.

    Subscribes with Transient Local QoS to catch latched descriptions.
    Returns a tuple:
      (urdf_xml:str|None,
       srdf_xml:str|None,
       runtime_sensors:list,
       topic_names:list[str],
       topics_with_types:list[tuple[str, list[str]]],
       services_with_types:list[tuple[str, list[str]]],
       controllers_via_cm:list[dict])
    """
    urdf_xml = None
    srdf_xml = None
    runtime_sensors = []
    topic_names = []
    topics_with_types = []
    services_with_types = []
    controllers_via_cm = []
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String as RosString
        from rcl_interfaces.srv import GetParameters
        try:
            from controller_manager_msgs.srv import ListControllers as CM_ListControllers
        except Exception:
            CM_ListControllers = None

        class _DescSniffer(Node):
            def __init__(self):
                super().__init__('urdf_to_rcm_sniffer')
                self.urdf_xml = None
                self.srdf_xml = None
                self.controllers = []
                qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                )
                self.create_subscription(RosString, '/robot_description', self._urdf_cb, qos)
                # Many setups do not publish SRDF on a topic, so also try param fetch later
                self.create_subscription(RosString, '/robot_description_semantic', self._srdf_cb, qos)
            def _urdf_cb(self, msg):
                if self.urdf_xml is None and getattr(msg, 'data', None):
                    self.urdf_xml = msg.data
            def _srdf_cb(self, msg):
                if self.srdf_xml is None and getattr(msg, 'data', None):
                    self.srdf_xml = msg.data
            def _try_fetch_param(self, node_name: str, param_name: str, timeout_sec: float = 0.5):
                try:
                    client = self.create_client(GetParameters, f"{node_name}/get_parameters")
                    if not client.wait_for_service(timeout_sec=timeout_sec):
                        return None
                    req = GetParameters.Request()
                    req.names = [param_name]
                    future = client.call_async(req)
                    end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
                    while rclpy.ok() and not future.done() and self.get_clock().now().nanoseconds < end_time:
                        rclpy.spin_once(self, timeout_sec=0.05)
                    if future.done() and future.result() is not None and future.result().values:
                        val = future.result().values[0]
                        # rclpy parameter variant types
                        if hasattr(val, 'string_value') and val.string_value:
                            return val.string_value
                    return None
                except Exception:
                    return None
            def fetch_srdf_from_parameters(self) -> str:
                # Probe common nodes that host robot_description_semantic
                candidates = ['/move_group', '/rviz2', '/planning_scene_monitor']
                # Include every node in graph as fallback
                try:
                    for name in candidates + [n for n in self.get_node_names() if n not in candidates]:
                        val = self._try_fetch_param(name, 'robot_description_semantic', timeout_sec=0.4)
                        if val:
                            return val
                except Exception:
                    pass
                return None
            def query_controller_managers(self, timeout_sec: float = 1.0):
                if CM_ListControllers is None:
                    return []
                services = self.get_service_names_and_types()
                list_ctrl_srvs = [name for (name, types) in services if any('controller_manager_msgs/srv/ListControllers' in t for t in types)]
                discovered = []
                for srv_name in list_ctrl_srvs:
                    try:
                        client = self.create_client(CM_ListControllers, srv_name)
                        if not client.wait_for_service(timeout_sec=0.3):
                            continue
                        req = CM_ListControllers.Request()
                        future = client.call_async(req)
                        # simple spin loop with timeout
                        end_time = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
                        while rclpy.ok() and not future.done() and self.get_clock().now().nanoseconds < end_time:
                            rclpy.spin_once(self, timeout_sec=0.05)
                        if future.done() and future.result() is not None:
                            for c in getattr(future.result(), 'controller', []) or getattr(future.result(), 'controllers', []) or []:
                                try:
                                    discovered.append({
                                        'name': getattr(c, 'name', None),
                                        'type': getattr(c, 'type', None),
                                        'state': getattr(c, 'state', None),
                                        'claimed_interfaces': [getattr(ci, 'interface_name', str(ci)) for ci in (getattr(c, 'claimed_interfaces', []) or [])]
                                    })
                                except Exception:
                                    pass
                    except Exception:
                        continue
                return discovered

        rclpy.init()
        node = _DescSniffer()
        start = node.get_clock().now().nanoseconds
        end_time = start + int(timeout_sec * 1e9)
        # Spin until message received or timeout
        while rclpy.ok() and node.urdf_xml is None and node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Keep spinning briefly to also grab SRDF if available
        srdf_wait_end = node.get_clock().now().nanoseconds + int(2.0 * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < srdf_wait_end:
            if node.srdf_xml is not None:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        urdf_xml = node.urdf_xml
        srdf_xml = node.srdf_xml
        # If SRDF topic not found, try to fetch from parameters
        if srdf_xml is None:
            try:
                srdf_xml = node.fetch_srdf_from_parameters()
            except Exception:
                pass

        # ROS graph snapshot
        topics_with_types = node.get_topic_names_and_types()
        topic_names = [t[0] for t in topics_with_types]
        services_with_types = node.get_service_names_and_types()

        # Try controller_manager listing
        try:
            controllers_via_cm = node.query_controller_managers(timeout_sec=1.0)
        except Exception:
            controllers_via_cm = []

        # Topic-based sensor inference (minimal hints)
        def add_sensor_if_present(topic, s_type):
            if topic in topic_names:
                runtime_sensors.append({'link': None, 'type': s_type, 'pose_in_base': {'xyz': None}, 'topic': topic, 'source': 'ros_runtime'})
        # Common topics
        add_sensor_if_present('/scan', 'lidar')
        add_sensor_if_present('/imu', 'imu')
        # Cameras (search prefixes)
        for name in topic_names:
            if name.endswith('/image') or name.endswith('/image_raw') or '/camera/' in name:
                runtime_sensors.append({'link': None, 'type': 'camera', 'pose_in_base': {'xyz': None}, 'topic': name, 'source': 'ros_runtime'})

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        # If ROS not available or any error, just return what we have
        pass
    return (
        urdf_xml,
        srdf_xml,
        runtime_sensors,
        topic_names,
        topics_with_types,
        services_with_types,
        controllers_via_cm,
    )


def _group_by_type(topics_with_types: list) -> dict:
    by_type = {}
    try:
        for name, types in topics_with_types or []:
            for t in types:
                by_type.setdefault(t, []).append(name)
    except Exception:
        pass
    return by_type


def _infer_ros_interfaces_from_graph(topics_with_types, services_with_types, controllers_via_cm, rcm_after_build: dict) -> dict:
    """Infer interfaces using message types and optional controller_manager data.

    This avoids hardcoding controller names and is namespace-agnostic.
    """
    if not topics_with_types:
        return {}

    interfaces: dict = {}
    topics_by_type = _group_by_type(topics_with_types)

    # Joint states
    joint_state_section = {}
    for tname in topics_by_type.get('sensor_msgs/msg/JointState', []):
        if tname.endswith('/joint_states') or tname == '/joint_states':
            joint_state_section['joint_states'] = tname
            break
    for tname in topics_by_type.get('control_msgs/msg/DynamicJointState', []):
        if tname.endswith('/dynamic_joint_states') or tname == '/dynamic_joint_states':
            joint_state_section['dynamic_joint_states'] = tname
            break
    if joint_state_section:
        interfaces['joint_state'] = joint_state_section

    # Controllers: detect by types, group by base prefix
    controllers = []
    traj_cmds = topics_by_type.get('trajectory_msgs/msg/JointTrajectory', [])
    for cmd_topic in traj_cmds:
        base = '/'.join(cmd_topic.split('/')[:-1]) or '/'
        # find likely state/controller_state topics under same base
        state_topic = None
        ctrl_state_topic = None
        for st in topics_by_type.get('control_msgs/msg/JointTrajectoryControllerState', []):
            if st.startswith(base + '/') or st == base + '/state' or st == base + '/controller_state':
                if st.endswith('/controller_state'):
                    ctrl_state_topic = st
                elif st.endswith('/state'):
                    state_topic = st
        controllers.append({
            'name': base.strip('/').split('/')[-1] or 'trajectory_controller',
            'type': 'trajectory',
            'topics': {
                'command': cmd_topic,
                'state': state_topic,
                'controller_state': ctrl_state_topic
            }
        })

    # Forward position style controllers
    for cmd_topic in topics_by_type.get('std_msgs/msg/Float64MultiArray', []):
        if cmd_topic.endswith('/commands'):
            controllers.append({
                'name': cmd_topic.strip('/').split('/')[-2] if '/' in cmd_topic.strip('/') else 'forward_position_controller',
                'type': 'position',
                'topics': {'command': cmd_topic}
            })

    # Gripper command controller
    for gcmd in topics_by_type.get('control_msgs/msg/GripperCommand', []):
        base = '/'.join(gcmd.split('/')[:-1]) or '/'
        controllers.append({
            'name': base.strip('/').split('/')[-1] or 'gripper_controller',
            'type': 'gripper',
            'topics': {'command': gcmd}
        })

    # Merge with controller_manager info to annotate states/types if present
    if controllers_via_cm:
        by_name = {c.get('name'): c for c in controllers_via_cm if c.get('name')}
        for c in controllers:
            info = by_name.get(c.get('name'))
            if info:
                c['plugin_type'] = info.get('type')
                c['state'] = info.get('state')
                if info.get('claimed_interfaces'):
                    c['claimed_interfaces'] = info['claimed_interfaces']

    if controllers:
        interfaces['controllers'] = controllers

    # Servo (MoveIt Servo) by message types
    servo = {}
    for s in topics_by_type.get('control_msgs/msg/ServoStatus', []):
        servo['status'] = s
        break
    # Common cmd types used by Servo
    # delta joint cmds often std_msgs/Float64MultiArray
    for t in topics_by_type.get('std_msgs/msg/Float64MultiArray', []):
        if 'servo' in t or 'delta_joint' in t:
            servo.setdefault('delta_joint_cmds', t)
            break
    # delta twist cmds often geometry_msgs/TwistStamped
    for t in topics_by_type.get('geometry_msgs/msg/TwistStamped', []):
        if 'servo' in t or 'delta_twist' in t:
            servo.setdefault('delta_twist_cmds', t)
            break
    if servo:
        interfaces['servo'] = servo

    # MoveIt interfaces by types
    moveit = {}
    if topics_by_type.get('moveit_msgs/msg/PlanningScene'):
        moveit['planning_scene'] = topics_by_type['moveit_msgs/msg/PlanningScene'][0]
    if topics_by_type.get('moveit_msgs/msg/MotionPlanRequest'):
        moveit['motion_plan_request'] = topics_by_type['moveit_msgs/msg/MotionPlanRequest'][0]
    if topics_by_type.get('moveit_msgs/msg/DisplayTrajectory'):
        moveit['display_planned_path'] = topics_by_type['moveit_msgs/msg/DisplayTrajectory'][0]
    if topics_by_type.get('moveit_msgs/msg/PlanningSceneWorld'):
        moveit['monitored_planning_scene'] = topics_by_type['moveit_msgs/msg/PlanningSceneWorld'][0]
    # CollisionObject topic (environment updates)
    if topics_by_type.get('moveit_msgs/msg/CollisionObject'):
        moveit['collision_object_topic'] = topics_by_type['moveit_msgs/msg/CollisionObject'][0]
    # Detect GetPlanningScene service from services list
    try:
        if services_with_types:
            for srv_name, srv_types in services_with_types:
                if any('moveit_msgs/srv/GetPlanningScene' in t for t in srv_types):
                    moveit['get_planning_scene_service'] = srv_name
                    break
    except Exception:
        pass
    try:
        srdf_groups = rcm_after_build.get('kinematics', {}).get('srdf', {}).get('groups', [])
        if srdf_groups:
            moveit['groups'] = [g.get('name') for g in srdf_groups if isinstance(g, dict) and g.get('name')]
    except Exception:
        pass
    if moveit:
        interfaces['moveit'] = moveit

    # TF
    tf_section = {}
    if topics_by_type.get('tf2_msgs/msg/TFMessage'):
        # pick first instances
        for name in topics_by_type['tf2_msgs/msg/TFMessage']:
            if name.endswith('/tf') or name == '/tf':
                tf_section['tf'] = name
            if name.endswith('/tf_static') or name == '/tf_static':
                tf_section['tf_static'] = name
        if tf_section:
            interfaces['tf'] = tf_section

    # Clock (optional)
    if topics_by_type.get('rosgraph_msgs/msg/Clock'):
        for name in topics_by_type['rosgraph_msgs/msg/Clock']:
            if name.endswith('/clock') or name == '/clock':
                interfaces['clock'] = name
                break

    return interfaces


def _infer_ros_interfaces_from_topics(topic_names, rcm_after_build: dict) -> dict:
    """Infer generic ROS interfaces/endpoints from topic names.

    Only include sections when corresponding topics exist.
    """
    if not topic_names:
        return {}

    interfaces: dict = {}

    # Joint states
    joint_state_section = {}
    if '/joint_states' in topic_names:
        joint_state_section['joint_states'] = '/joint_states'
    if '/dynamic_joint_states' in topic_names:
        joint_state_section['dynamic_joint_states'] = '/dynamic_joint_states'
    if joint_state_section:
        interfaces['joint_state'] = joint_state_section

    # Controllers
    controllers = []
    # Trajectory controller
    traj_cmd_topics = [t for t in topic_names if t.endswith('/joint_trajectory_controller/joint_trajectory')]
    if traj_cmd_topics:
        name = 'joint_trajectory_controller'
        ns_prefix = ''  # global by default
        controllers.append({
            'name': name,
            'type': 'trajectory',
            'topics': {
                'command': traj_cmd_topics[0],
                'state': next((t for t in topic_names if t.endswith('/joint_trajectory_controller/state')), None),
                'controller_state': next((t for t in topic_names if t.endswith('/joint_trajectory_controller/controller_state')), None)
            }
        })
    # Forward position controller
    if '/forward_position_controller/commands' in topic_names:
        controllers.append({
            'name': 'forward_position_controller',
            'type': 'position',
            'topics': {'command': '/forward_position_controller/commands'}
        })
    if controllers:
        interfaces['controllers'] = controllers

    # Servo (MoveIt Servo)
    servo = {}
    if '/servo_node/delta_joint_cmds' in topic_names:
        servo['delta_joint_cmds'] = '/servo_node/delta_joint_cmds'
    if '/servo_node/delta_twist_cmds' in topic_names:
        servo['delta_twist_cmds'] = '/servo_node/delta_twist_cmds'
    if '/servo_node/status' in topic_names:
        servo['status'] = '/servo_node/status'
    if servo:
        interfaces['servo'] = servo

    # MoveIt interfaces
    moveit = {}
    for key, topic in {
        'planning_scene': '/planning_scene',
        'motion_plan_request': '/motion_plan_request',
        'display_planned_path': '/display_planned_path',
        'monitored_planning_scene': '/monitored_planning_scene'
    }.items():
        if topic in topic_names:
            moveit[key] = topic
    # Include group names if present in built RCM
    try:
        srdf_groups = rcm_after_build.get('kinematics', {}).get('srdf', {}).get('groups', [])
        if srdf_groups:
            moveit['groups'] = [g.get('name') for g in srdf_groups if isinstance(g, dict) and g.get('name')]
    except Exception:
        pass
    if moveit:
        interfaces['moveit'] = moveit

    # TF and clock
    tf_section = {}
    if '/tf' in topic_names:
        tf_section['tf'] = '/tf'
    if '/tf_static' in topic_names:
        tf_section['tf_static'] = '/tf_static'
    if tf_section:
        interfaces['tf'] = tf_section
    if '/clock' in topic_names:
        interfaces['clock'] = '/clock'

    return interfaces

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", nargs='?', help="path to URDF file (optional; defaults to ROS topics)")
    parser.add_argument("--sdf", help="optional path to SDF model file to extract sensors", default=None)
    parser.add_argument("--auto", action='store_true', help="(deprecated) No longer falls back to TurtleBot; ROS topics are used by default")
    parser.add_argument("--from-ros", action='store_true', help="Force fetching descriptions from ROS topics (default behavior if no URDF path provided)")
    parser.add_argument("-o", "--output", default="rcm_pinocchio.json")
    parser.add_argument("--samples", type=int, default=400)
    args = parser.parse_args()

    urdf_path = args.urdf
    sdf_path = args.sdf

    # Deprecated flag notice
    if args.auto:
        print("Note: --auto fallback has been removed. Using ROS topics (or provided URDF path) only.")

    runtime_urdf_xml = None
    runtime_srdf_xml = None
    runtime_sensors = []
    discovered_topic_names: list = []
    discovered_topics_with_types: list = []
    discovered_services_with_types: list = []
    discovered_cm_controllers: list = []

    # If no URDF path is provided, or --from-ros is specified, fetch from ROS topics
    if urdf_path is None or args.from_ros:
        runtime_urdf_xml, runtime_srdf_xml, runtime_sensors, discovered_topic_names, discovered_topics_with_types, discovered_services_with_types, discovered_cm_controllers = collect_runtime_ros_data(timeout_sec=15.0)
        if not runtime_urdf_xml:
            raise SystemExit("Did not receive /robot_description from ROS. Please launch your robot and try again.")
        # Write URDF to temp file
        tmp_urdf = Path(".urdf_runtime_tmp.urdf")
        tmp_urdf.write_text(runtime_urdf_xml)
        urdf_path = str(tmp_urdf.resolve())
        # If SRDF received, write alongside URDF so it is auto-detected by .with_suffix('.srdf')
        if runtime_srdf_xml:
            tmp_srdf = Path(".urdf_runtime_tmp.srdf")
            tmp_srdf.write_text(runtime_srdf_xml)
        else:
            # Ensure we don't reuse stale srdf from previous runs
            tmp_srdf = Path(".urdf_runtime_tmp.srdf")
            if tmp_srdf.exists():
                try:
                    tmp_srdf.unlink()
                except Exception:
                    pass

    # Build RCM (no hard-coded fallbacks)
    rcm = build_rcm_with_pinocchio(urdf_path, samples=args.samples, sdf_path=sdf_path, extra_sensors=runtime_sensors)

    # If MoveIt PlanningScene service is available, query environment objects and include them
    def _fetch_environment_from_moveit(services_with_types, timeout_sec: float = 2.0):
        try:
            import rclpy
            from rclpy.node import Node
            from moveit_msgs.srv import GetPlanningScene
            from moveit_msgs.msg import PlanningSceneComponents
            # Lazily find the service name from the graph snapshot
            service_name = None
            for name, types in (services_with_types or []):
                if any('moveit_msgs/srv/GetPlanningScene' in t for t in types):
                    service_name = name
                    break
            if service_name is None:
                return None

            class _PSClient(Node):
                def __init__(self):
                    super().__init__('rcm_ps_client')
                    self.client = self.create_client(GetPlanningScene, service_name)

            if not rclpy.ok():
                rclpy.init()
                should_shutdown = True
            else:
                should_shutdown = False

            node = _PSClient()
            if not node.client.wait_for_service(timeout_sec=timeout_sec):
                node.destroy_node()
                if should_shutdown:
                    rclpy.shutdown()
                return None
            req = GetPlanningScene.Request()
            # Request world object geometry explicitly so poses/dimensions are populated
            req.components.components = (
                int(getattr(PlanningSceneComponents, 'WORLD_OBJECT_GEOMETRY', 0))
                | int(getattr(PlanningSceneComponents, 'WORLD_OBJECT_NAMES', 0))
            )
            future = node.client.call_async(req)
            end_time = node.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
            while rclpy.ok() and not future.done() and node.get_clock().now().nanoseconds < end_time:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done() or future.result() is None:
                node.destroy_node()
                if should_shutdown:
                    rclpy.shutdown()
                return None
            res = future.result()
            objs = []
            try:
                for co in getattr(res.scene.world, 'collision_objects', []) or []:
                    entry = {
                        'id': getattr(co, 'id', None),
                        'frame_id': getattr(co.header, 'frame_id', 'world') if hasattr(co, 'header') else 'world',
                        'shapes': []
                    }
                    # Extract primitive shapes and poses
                    try:
                        prims = list(getattr(co, 'primitives', []))
                        poses = list(getattr(co, 'primitive_poses', []))
                        # Base pose of the CollisionObject (world frame)
                        base = getattr(co, 'pose', None)
                        for i in range(min(len(prims), len(poses))):
                            p = prims[i]
                            local = poses[i]
                            shape_type = getattr(p, 'type', 0)
                            dims = list(getattr(p, 'dimensions', []))

                            # Compose world pose = base ⊕ local (position only uses yaw-less approx if base missing)
                            if base is not None:
                                # Quaternion multiply: q = qb * ql
                                bx, by, bz, bw = float(base.orientation.x), float(base.orientation.y), float(base.orientation.z), float(base.orientation.w)
                                lx, ly, lz, lw = float(local.orientation.x), float(local.orientation.y), float(local.orientation.z), float(local.orientation.w)
                                qx = bw*lx + bx*lw + by*lz - bz*ly
                                qy = bw*ly - bx*lz + by*lw + bz*lx
                                qz = bw*lz + bx*ly - by*lx + bz*lw
                                qw = bw*lw - bx*lx - by*ly - bz*lz
                                # Rotate local position by base orientation, then add base translation
                                lxv, lyv, lzv = float(local.position.x), float(local.position.y), float(local.position.z)
                                # Rotate vector v by quaternion b: v' = b * v * b_conj
                                # Using optimized form
                                vx, vy, vz = lxv, lyv, lzv
                                ux, uy, uz = bx, by, bz
                                s = bw
                                # t = 2 * u x v
                                tx = 2.0 * (uy*vz - uz*vy)
                                ty = 2.0 * (uz*vx - ux*vz)
                                tz = 2.0 * (ux*vy - uy*vx)
                                rx = vx + s*tx + (uy*tz - uz*ty)
                                ry = vy + s*ty + (uz*tx - ux*tz)
                                rz = vz + s*tz + (ux*ty - uy*tx)
                                px = float(base.position.x) + rx
                                py = float(base.position.y) + ry
                                pz = float(base.position.z) + rz
                            else:
                                qx, qy, qz, qw = float(local.orientation.x), float(local.orientation.y), float(local.orientation.z), float(local.orientation.w)
                                px, py, pz = float(local.position.x), float(local.position.y), float(local.position.z)

                            entry['shapes'].append({
                                'primitive_type': int(shape_type),
                                'dimensions': [float(x) for x in dims],
                                'pose': {
                                    'position': [px, py, pz],
                                    'orientation_xyzw': [qx, qy, qz, qw]
                                }
                            })
                    except Exception:
                        pass
                    
                    # Add semantic inference to the object
                    if entry['shapes']:
                        # Get dimensions and position from the first shape for semantic inference
                        first_shape = entry['shapes'][0]
                        dimensions = first_shape.get('dimensions', [])
                        position = first_shape.get('pose', {}).get('position', [])
                        
                        # Infer semantic properties
                        semantics = infer_object_semantics(entry['id'], dimensions, position)
                        entry.update(semantics)
                    
                    objs.append(entry)
            except Exception:
                objs = []

            node.destroy_node()
            if should_shutdown:
                rclpy.shutdown()
            return {'objects': objs, 'source': 'moveit_get_planning_scene'} if objs else None
        except Exception:
            return None

    env = _fetch_environment_from_moveit(discovered_services_with_types, timeout_sec=2.0)

    # If positions are all zeros, fall back to reading /collision_object topic briefly
    def _all_zero_positions(e: dict) -> bool:
        try:
            for obj in e.get('objects', []):
                for shp in obj.get('shapes', []):
                    pos = shp.get('pose', {}).get('position', [])
                    if any(abs(float(v)) > 1e-6 for v in pos):
                        return False
        except Exception:
            return False
        return True

    if not env or _all_zero_positions(env):
        try:
            import rclpy
            from rclpy.node import Node
            from moveit_msgs.msg import CollisionObject

            class _COCollector(Node):
                def __init__(self):
                    super().__init__('rcm_co_collector')
                    self.received = {}
                    self.sub = self.create_subscription(CollisionObject, '/collision_object', self.cb, 10)
                def cb(self, msg: CollisionObject):
                    try:
                        entry = {
                            'id': msg.id,
                            'frame_id': getattr(msg.header, 'frame_id', 'world'),
                            'shapes': []
                        }
                        prims = list(getattr(msg, 'primitives', []))
                        poses = list(getattr(msg, 'primitive_poses', []))
                        for i in range(min(len(prims), len(poses))):
                            p = prims[i]
                            local = poses[i]
                            dims = list(getattr(p, 'dimensions', []))
                            entry['shapes'].append({
                                'primitive_type': int(getattr(p, 'type', 0)),
                                'dimensions': [float(x) for x in dims],
                                'pose': {
                                    'position': [float(local.position.x), float(local.position.y), float(local.position.z)],
                                    'orientation_xyzw': [float(local.orientation.x), float(local.orientation.y), float(local.orientation.z), float(local.orientation.w)]
                                }
                            })
                        # Add semantic inference to the object
                        if entry['shapes']:
                            # Get dimensions and position from the first shape for semantic inference
                            first_shape = entry['shapes'][0]
                            dimensions = first_shape.get('dimensions', [])
                            position = first_shape.get('pose', {}).get('position', [])
                            
                            # Infer semantic properties
                            semantics = infer_object_semantics(entry['id'], dimensions, position)
                            entry.update(semantics)
                        
                        self.received[msg.id] = entry
                    except Exception:
                        pass

            created = False
            if not rclpy.ok():
                rclpy.init()
                created = True
            node = _COCollector()
            import time
            end = time.time() + 2.0
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(node.received) >= 2:
                    break
            if node.received:
                env = {'objects': list(node.received.values()), 'source': 'collision_object_topic'}
            node.destroy_node()
            if created:
                rclpy.shutdown()
        except Exception:
            pass

    if env:
        rcm['environment'] = env

    # Attach inferred ROS interfaces using graph (preferred), fallback to simple name-based
    ros_if = {}
    if discovered_topics_with_types:
        ros_if = _infer_ros_interfaces_from_graph(discovered_topics_with_types, discovered_services_with_types, discovered_cm_controllers, rcm)
    if not ros_if and discovered_topic_names:
        ros_if = _infer_ros_interfaces_from_topics(discovered_topic_names, rcm)
    if ros_if:
        rcm['ros_interfaces'] = ros_if
    with open(args.output, "w") as f:
        json.dump(rcm, f, indent=2)
    print("RCM written to", args.output)

if __name__ == "__main__":
    main()
