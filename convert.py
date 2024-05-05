# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
# Editor: Haoran Geng, Feishi Wang

"""
Utility to convert a URDF into USD format.

Unified Robot Description Format (URDF) is an XML file format used in ROS to describe all elements of
a robot. For more information, see: http://wiki.ros.org/urdf

This script uses the URDF importer extension from Isaac Sim (``omni.isaac.urdf_importer``) to convert a
URDF asset into USD format. It is designed as a convenience script for command-line use. For more
information on the URDF importer, see the documentation for the extension:
https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_urdf.html


positional arguments:
  input               The path to the input URDF file.
  output              The path to store the USD file.

optional arguments:
  -h, --help          show this help message and exit
  --headless          Force display off at all times. (default: False)
  --merge_joints, -m  Consolidate links that are connected by fixed joints. (default: False)
  --fix_base, -f      Fix the base to where it is imported. (default: False)
  --gym, -g           Make the asset instanceable for efficient cloning. (default: False)

"""

############################################
## Launch Isaac Sim Simulator first
############################################

import argparse
import contextlib

# omni-isaac-orbit
from omni.isaac.kit import SimulationApp

def parse_args():
    parser = argparse.ArgumentParser("Utility to convert a URDF into USD format.")
    parser.add_argument("--headless", action="store_true", default=True, help="Force display off at all times.")
    parser.add_argument(
        "--merge_joints",
        "-m",
        action="store_true",
        default=False,
        help="Consolidate links that are connected by fixed joints.",
    )
    parser.add_argument(
        "--fix_base", "-f", action="store_true", default=True, help="Fix the base to where it is imported."
    )
    parser.add_argument(
        "--gym", "-g", action="store_true", default=True, help="Make the asset instanceable for efficient cloning."
    )
    parser.add_argument(
        '--conf', '-c', help='Path to the configuration file.', required=True)
    parser.add_argument(
        '--objs', '-o', nargs='+', help='List of objects to convert.')
    args = parser.parse_args()
    return args

# launch omniverse app
args = parse_args()
config = {"headless": args.headless}
simulation_app = SimulationApp(config)

############################################
## Then convert URDF to USD as usual
############################################

import os
from os.path import join as pjoin
from os.path import exists as pexists
from collections import defaultdict
import xml.etree.ElementTree as ET
import carb
import omni
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.commands
from omni.isaac.core.simulation_context import SimulationContext
from pxr import Usd, UsdPhysics, UsdGeom
from utils import read_yaml, mkdir

_DRIVE_TYPE = {
    "none": 0,
    "position": 1,
    "velocity": 2,
}
"""Mapping from drive name to URDF importer drive number."""

_NORMALS_DIVISION = {
    "catmullClark": 0,
    "loop": 1,
    "bilinear": 2,
    "none": 3,
}
"""Mapping from normals division name to URDF importer normals division number."""

def make_visual_names_unique(xml_string):
    tree = ET.ElementTree(ET.fromstring(xml_string))
    root = tree.getroot()

    # Find all visual elements with a "name" attribute
    elements_with_name = root.findall(".//visual")

    # Count the occurrences of each name
    name_counts = defaultdict(int)
    for element in elements_with_name:
        name = element.get("name")
        name_counts[name] += 1

    # Rename elements with repeated names
    for element in elements_with_name:
        name = element.get("name")
        if name_counts[name] > 1:
            count = name_counts[name]
            new_name = f"{name}{count}"
            element.set("name", new_name)
            name_counts[name] -= 1

    return ET.tostring(root, encoding="unicode")

def single(urdf_path, usd_path):
    # Import URDF config
    _, urdf_config = omni.kit.commands.execute("URDFCreateImportConfig")

    folder_path = os.path.dirname(urdf_path)
    folder_name = folder_path.split('/')[-1]
    # Set URDF config
    # -- stage settings -- dont need to change these.
    urdf_config.set_distance_scale(1.0)
    urdf_config.set_up_vector(0, 0, 1)
    urdf_config.set_create_physics_scene(False)
    urdf_config.set_make_default_prim(True)
    # -- instancing settings
    urdf_config.set_make_instanceable(False)
    urdf_config.set_instanceable_usd_path(f"instanceable/{folder_name}/usd/instanceable_meshes.usd")
    # -- asset settings
    urdf_config.set_density(0.0)
    urdf_config.set_import_inertia_tensor(True)
    urdf_config.set_convex_decomp(False)
    urdf_config.set_subdivision_scheme(_NORMALS_DIVISION["bilinear"])
    # -- physics settings
    urdf_config.set_fix_base(True)
    urdf_config.set_self_collision(False)
    urdf_config.set_merge_fixed_joints(args.merge_joints)
    # -- drive settings
    # note: we set these to none because we want to use the default drive settings.
    urdf_config.set_default_drive_type(_DRIVE_TYPE["none"])
    # urdf_config.set_default_drive_strength(1e7)
    # urdf_config.set_default_position_drive_damping(1e5)

    # Print info
    print("-" * 80)
    print("-" * 80)
    print(f"Input URDF file: {urdf_path}")
    print(f"Saving USD file: {usd_path}")
    print("URDF importer config:")
    for key in dir(urdf_config):
        if not key.startswith("__"):
            try:
                attr = getattr(urdf_config, key)
                # check if attribute is a function
                if callable(attr):
                    continue
                print(f"\t{key}: {attr}")
            except TypeError:
                # this is only the case for subdivison scheme
                pass
    print("-" * 80)
    print("-" * 80)
    
    # Make visual names unique
    xml_str = open(urdf_path, 'r').read()
    xml_str = '<?xml version="1.0"?>' + '\n' + make_visual_names_unique(xml_str)
    urdf_unique_path = urdf_path.replace('.urdf', '_unique.urdf')
    with open(urdf_unique_path, 'w') as f:
        f.write(xml_str)

    # Convert URDF to USD
    omni.kit.commands.execute(
        "URDFParseAndImportFile", urdf_path=urdf_unique_path, import_config=urdf_config, dest_path=usd_path
    )
    
    # Open USD file and reset articulation root
    stage = Usd.Stage.Open(usd_path)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print('[INFO] Removing articulation root for', prim)
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    defaultPrim = stage.GetDefaultPrim()
    print('[INFO] Applying articulation root for', defaultPrim)
    UsdPhysics.ArticulationRootAPI.Apply(defaultPrim)
    stage.Save()

def main():
    urdf_base_dir = 'data/urdf'
    usd_base_dir = 'data/usd'
    objs = args.objs if args.objs else read_yaml(args.conf)['Objects']
    
    all_urdf_paths = []
    all_usd_paths = []
    
    for o in objs:
        urdf_obj_dir = pjoin(urdf_base_dir, o)
        usd_obj_dir = mkdir(pjoin(usd_base_dir, o))
        urdf_names = [name for name in os.listdir(urdf_obj_dir) if name.endswith('.urdf') and not name.endswith('_unique.urdf')]
        usd_names = [name.replace('.urdf', '.usd') for name in urdf_names]
        urdf_paths = [pjoin(urdf_obj_dir, name) for name in urdf_names]
        usd_paths = [pjoin(usd_obj_dir, name) for name in usd_names]
        
        all_urdf_paths += urdf_paths
        all_usd_paths += usd_paths
    
    for idx, (urdf_path, usd_path) in enumerate(zip(all_urdf_paths, all_usd_paths)):
        print("idx: ", idx, f"out of {len(all_usd_paths)}")
        single(urdf_path, usd_path)

if __name__ == "__main__":
    main()
    simulation_app.close()