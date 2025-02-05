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
import asyncio

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
        '--conf', '-c', default='rlbench_objects.yaml')
    parser.add_argument(
        '--objs', '-o', nargs='+', help='List of objects to convert.')
    args = parser.parse_args()
    return args

# launch omniverse app
args = parse_args()
config = {"headless": args.headless}
simulation_app = SimulationApp(config)
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.kit.asset_converter")

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
from pxr import Usd, UsdPhysics, UsdGeom, PhysxSchema
from typing import Optional
# from utils import read_yaml, mkdir
import yaml

def mkdir(path):
    os.makedirs(path, exist_ok=True)
    return path

def read_yaml(path):
    with open(path) as f:
        data = yaml.load(f, Loader=yaml.Loader)
    return data

def write_yaml(path, data):
    with open(path, 'w+') as f:
        yaml.dump(data, f, default_flow_style=False)


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

def safe_get(dict, key_dot_str):
    keys = key_dot_str.split('.')
    data = dict
    for k in keys:
        data = data.get(k, None)
        if data is None:
            return None
    return data

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

def convert_urdf_to_usd(urdf_path, usd_path):
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

async def convert(in_file, out_file, load_materials=False):
    ## Copy from standalone_examples/api/omni.kit.asset_converter/asset_usd_converter.py
    # This import causes conflicts when global
    import omni.kit.asset_converter

    def progress_callback(progress, total_steps):
        pass

    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    converter_context.ignore_materials = not load_materials
    # converter_context.ignore_animation = False
    # converter_context.ignore_cameras = True
    # converter_context.single_mesh = True
    # converter_context.smooth_normals = True
    # converter_context.preview_surface = False
    # converter_context.support_point_instancer = False
    # converter_context.embed_mdl_in_usd = False
    # converter_context.use_meter_as_world_unit = True
    # converter_context.create_world_as_default_root_prim = False
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(in_file, out_file, progress_callback, converter_context)
    success = True
    while True:
        success = await task.wait_until_finished()
        if not success:
            await asyncio.sleep(0.1)
        else:
            break
    return success

def convert_mesh_to_usd(mesh_path, usd_path):
    print(f"Input Mesh file: {mesh_path}")
    print(f"Saving USD file: {usd_path}")
    status = asyncio.get_event_loop().run_until_complete(
        convert(mesh_path, usd_path, True)
    )
    if not status:
        print(f"ERROR Status is {status}")
    print(f"---Added {usd_path}")

def post_process_usd(usd_path, type, collider_type, maxConvexHulls:Optional[int] = None):
    print(f'[INFO] Post processing {usd_path} with type {type} and collider type {collider_type}')
    stage = Usd.Stage.Open(usd_path)
    
    # Remove all rigid body and collision API
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
    
    # Apply collision API
    if type == 'rigid' or type == 'geometry':
        defaultPrim = stage.GetDefaultPrim()
        
        for prim in stage.Traverse():
            UsdPhysics.CollisionAPI.Apply(prim)
            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim)
            if collider_type is None or collider_type == 'convex_hull':
                meshCollisionAPI.CreateApproximationAttr().Set("convexHull")
            elif collider_type == 'convex_decomposition':
                meshCollisionAPI.CreateApproximationAttr().Set("convexDecomposition")
            elif collider_type == 'mesh_simplification':
                # This is buggy in Omniverse Isaac Sim 2023.1.1
                # meshCollisionAPI.CreateApproximationAttr().Set("meshSimplification")
                raise NotImplementedError("Mesh simplification is buggy in Isaac Sim 2023.1.1.")
            elif collider_type == 'triangle_mesh':
                # This is buggy in Omniverse Isaac Sim 2023.1.1
                # meshCollisionAPI.CreateApproximationAttr().Set("none")
                raise NotImplementedError("Triangle mesh is buggy in Isaac Sim 2023.1.1.")
            else:
                raise ValueError("Unknown collider type.")        

    # Apply rigid body API
    if type == 'rigid':
        defaultPrim = stage.GetDefaultPrim()
        UsdPhysics.RigidBodyAPI.Apply(defaultPrim)

    stage.Save()

def main():
    urdf_base_dir = 'data_rlbench/urdf'
    usd_base_dir = 'data_rlbench/usd'
    cfg = read_yaml(args.conf)
    objs = args.objs if args.objs else list(cfg.keys())
    
    all_urdf_paths = []
    all_usd_paths = []
    all_object_names = []
    
    for o in objs:
        urdf_obj_dir = pjoin(urdf_base_dir, o)
        usd_obj_dir = mkdir(pjoin(usd_base_dir, o))
        urdf_names = [name for name in os.listdir(urdf_obj_dir) if 
                      name.endswith('.stl') or (name.endswith('.urdf') and not name.endswith('_unique.urdf'))]
        urdf_names = [name for name in urdf_names if name.removesuffix('.urdf').removesuffix('.stl') in cfg[o]['objects']]
        usd_names = [name.removesuffix('.urdf').removesuffix('.stl') + '.usd' for name in urdf_names]
        urdf_paths = [pjoin(urdf_obj_dir, name) for name in urdf_names]
        usd_paths = [pjoin(usd_obj_dir, name) for name in usd_names]
        
        all_urdf_paths += urdf_paths
        all_usd_paths += usd_paths
        all_object_names += [o] * len(urdf_paths)
    
    for idx, (urdf_path, usd_path, o) in enumerate(zip(all_urdf_paths, all_usd_paths, all_object_names)):
        print("-" * 80)
        print("-" * 80)
        print("idx: ", idx, f"out of {len(all_usd_paths)}")
        if urdf_path.endswith('.stl'):
            convert_mesh_to_usd(urdf_path, usd_path)
        elif urdf_path.endswith('.urdf'):
            convert_urdf_to_usd(urdf_path, usd_path)
        else:
            raise ValueError("Unknown file type.")
        
        name = os.path.splitext(os.path.basename(usd_path))[0]
        try:
            type = cfg[o]['types'][name]
        except:
            type = 'rigid'
        collider_type = safe_get(cfg, f'{o}.collider_types.{name}')
        post_process_usd(usd_path, type, collider_type)
        print("-" * 80)
        print("-" * 80)

if __name__ == "__main__":
    main()
    simulation_app.close()