########################################################
## Commandline arguments
########################################################
from dataclasses import dataclass

import tyro


@dataclass
class Args:
    urdf_file: str = "data_rlbench/urdf/close_box/box_base.urdf"


args = tyro.cli(Args)

########################################################
## Normal Code
########################################################
import os
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d


def euler_to_rotation_matrix(rpy):
    """Converts roll, pitch, yaw (RPY) angles to a 3x3 rotation matrix."""
    roll, pitch, yaw = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotation_matrix_to_euler(R):
    """Converts a 3x3 rotation matrix to roll, pitch, yaw angles.

    This uses a standard approach and is valid for non-singular configurations.
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    return np.array([roll, pitch, yaw])


def origin_to_transform(origin_element):
    """Converts an URDF origin element to a 4x4 transformation matrix."""
    T = np.eye(4)
    if origin_element is not None:
        # Translation
        if "xyz" in origin_element.attrib:
            xyz = np.array([float(v) for v in origin_element.get("xyz").split()])
            T[:3, 3] = xyz
        # Rotation (assumed in RPY)
        if "rpy" in origin_element.attrib:
            rpy = np.array([float(v) for v in origin_element.get("rpy").split()])
            R = euler_to_rotation_matrix(rpy)
            T[:3, :3] = R
    return T


def parse_urdf(file_path):
    """Parses a URDF file and returns a list of boxes with combined transformations.

    For each visual element, the final transform is:

        final_transform = link_transform * visual_origin_transform

    Here the link_transform is computed by composing joint transformations.
    """
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Step 1: Gather visual data per link.
    # This dictionary maps link_name to a list of tuples (visual_origin_transform, size)
    link_visuals = {}
    for link in root.findall("link"):
        link_name = link.get("name")
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            visual_transform = origin_to_transform(origin) if origin is not None else np.eye(4)
            geometry = visual.find("geometry")
            if geometry is None:
                continue
            box_elem = geometry.find("box")
            if box_elem is None:
                continue

            size = np.array([float(v) for v in box_elem.get("size").split()])

            if link_name not in link_visuals:
                link_visuals[link_name] = []
            link_visuals[link_name].append((visual_transform, size))

    # Step 2: Compute the transformation for each link by processing joints.
    # We create a dictionary: link_name -> absolute link transformation (4x4)
    link_transforms = {}

    # First, determine the base links (which are not children of any joint):
    joint_children = set()
    for joint in root.findall("joint"):
        child_elem = joint.find("child")
        if child_elem is not None:
            joint_children.add(child_elem.get("link"))

    for link in root.findall("link"):
        link_name = link.get("name")
        # Base links get the identity transform.
        if link_name not in joint_children:
            link_transforms[link_name] = np.eye(4)

    # Process joints (Note: for more complex URDFs, you might need recursion/traversal)
    for joint in root.findall("joint"):
        parent_elem = joint.find("parent")
        child_elem = joint.find("child")
        if parent_elem is None or child_elem is None:
            continue
        parent_link = parent_elem.get("link")
        child_link = child_elem.get("link")

        joint_origin = joint.find("origin")
        joint_transform = origin_to_transform(joint_origin) if joint_origin is not None else np.eye(4)

        # Compose to get child's absolute transform.
        parent_transform = link_transforms.get(parent_link, np.eye(4))
        link_transforms[child_link] = parent_transform @ joint_transform

    # Step 3: Combine link transform with visual transform
    boxes = []
    for link_name, visuals in link_visuals.items():
        # Get the absolute transform of the link, if not defined assume identity.
        base_T = link_transforms.get(link_name, np.eye(4))
        for visual_transform, size in visuals:
            # The final transformation applied to the box
            total_transform = base_T @ visual_transform
            xyz = total_transform[:3, 3]
            rpy = rotation_matrix_to_euler(total_transform[:3, :3])
            boxes.append((xyz, rpy, size))

    return boxes


def create_box(size, xyz, rpy):
    """Creates an Open3D box mesh with the given size and applies the transformation.

    The box is first created with its center at the origin, then the transformation
    (rotation from RPY and translation) is applied.
    """
    box = o3d.geometry.TriangleMesh.create_box(width=size[0], height=size[1], depth=size[2])
    # Move box center to origin
    box.translate([-size[0] / 2, -size[1] / 2, -size[2] / 2])
    box.compute_vertex_normals()

    # Build transformation matrix from xyz and rpy.
    transform = np.eye(4)
    transform[:3, :3] = euler_to_rotation_matrix(rpy)
    transform[:3, 3] = xyz
    box.transform(transform)

    return box


def main(args: Args):
    boxes = parse_urdf(args.urdf_file)
    o3d_boxes = [create_box(size, xyz, rpy) for xyz, rpy, size in boxes]

    for box in o3d_boxes:
        box.paint_uniform_color([0.7, 0.7, 0.7])

    mesh_coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
    o3d.visualization.draw_geometries(o3d_boxes + [mesh_coord])


if __name__ == "__main__":
    main(args)
