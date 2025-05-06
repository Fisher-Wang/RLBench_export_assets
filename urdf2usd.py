########################################################
## Commandline arguments
########################################################

from dataclasses import dataclass
from typing import Literal

import tyro


@dataclass
class Args:
    input: str
    output: str | None = None
    make_instanceable: bool = False
    collision_approximation: Literal["convexHull", "convexDecomposition", "none"] = "convexHull"
    """The method used for approximating collision mesh. Set to "none" to not add a collision mesh to the converted mesh."""
    mass: float | None = None
    """The mass (in kg) to assign to the converted asset. If not provided, then no mass is added."""
    headless: bool = False
    """Run with headless could be buggy, see https://github.com/isaac-sim/IsaacLab/issues/1279"""
    exit_on_finish: bool = True
    """Exit the IsaacSim app after the conversion is finished."""


args = tyro.cli(Args)

########################################################
## Launch IsaacLab
########################################################
import argparse

from omni.isaac.lab.app import AppLauncher

parser = argparse.ArgumentParser()
AppLauncher.add_app_launcher_args(parser)
args_isaaclab = parser.parse_args([])
args_isaaclab.headless = args.headless
app_launcher = AppLauncher(args_isaaclab)
simulation_app = app_launcher.app

########################################################
## Normal Code
########################################################
import contextlib
import os
import xml.etree.ElementTree as ET
from collections import defaultdict
from glob import glob
from itertools import chain

import carb
import numpy as np
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.app
import open3d as o3d
import pymeshlab
import yaml
from loguru import logger as log
from omni.isaac.lab.sim.converters import (
    MeshConverter,
    MeshConverterCfg,
    UrdfConverter,
    UrdfConverterCfg,
)
from omni.isaac.lab.sim.schemas import schemas_cfg
from omni.isaac.lab.utils.assets import check_file_path
from pxr import Usd, UsdPhysics
from rich.logging import RichHandler

from utils import ShaderFixer

log.configure(handlers=[{"sink": RichHandler(), "format": "{message}"}])


def convert_obj_to_usd(obj_path, usd_path):
    log.info(f"Converting {obj_path}")

    # check valid file path
    if not os.path.isabs(obj_path):
        obj_path = os.path.abspath(obj_path)
    if not check_file_path(obj_path):
        raise ValueError(f"Invalid mesh file path: {obj_path}")

    usd_path = os.path.abspath(usd_path)

    # Mass properties
    if args.mass is not None:
        mass_props = schemas_cfg.MassPropertiesCfg(mass=args.mass)
        rigid_props = schemas_cfg.RigidBodyPropertiesCfg()
    else:
        mass_props = None
        rigid_props = None

    # Collision properties
    collision_props = schemas_cfg.CollisionPropertiesCfg(collision_enabled=args.collision_approximation != "none")

    # Create Mesh converter config
    mesh_converter_cfg = MeshConverterCfg(
        mass_props=mass_props,
        rigid_props=rigid_props,
        collision_props=collision_props,
        asset_path=obj_path,
        force_usd_conversion=True,
        usd_dir=os.path.dirname(usd_path),
        usd_file_name=os.path.basename(usd_path),
        make_instanceable=args.make_instanceable,
        collision_approximation=args.collision_approximation,
    )

    log.info(f"Conversion configuration: {mesh_converter_cfg}")

    MeshConverter(mesh_converter_cfg)

def make_visual_names_unique(xml_string: str) -> str:
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

def make_urdf_with_unique_visual_names(urdf_path: str) -> str:
    xml_str = open(urdf_path, "r").read()
    xml_str = '<?xml version="1.0"?>' + "\n" + make_visual_names_unique(xml_str)
    urdf_unique_path = urdf_path.replace(".urdf", "_unique.urdf")
    with open(urdf_unique_path, "w") as f:
        f.write(xml_str)
    return urdf_unique_path

def make_obj_without_slashes(obj_path: str) -> str:
    obj_str = open(obj_path, "r").read()
    obj_str = obj_str.replace("/", "")
    obj_clean_path = obj_path.replace(".obj", "_clean.obj")
    with open(obj_clean_path, "w") as f:
        f.write(obj_str)
    return obj_clean_path

def convert_urdf_to_usd(urdf_path, usd_path):
    log.info(f"Converting {urdf_path}")

    # check valid file path
    if not os.path.isabs(urdf_path):
        urdf_path = os.path.abspath(urdf_path)

    urdf_converter_cfg = UrdfConverterCfg(
        asset_path=urdf_path,
        usd_dir=os.path.dirname(usd_path),
        usd_file_name=os.path.basename(usd_path),
        make_instanceable=args.make_instanceable,
        force_usd_conversion=True,
        fix_base=True,
    )
    UrdfConverter(urdf_converter_cfg)

def is_articulation(usd_path: str):
    joint_count = 0
    stage = Usd.Stage.Open(usd_path)
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            joint_count += 1
    return joint_count > 0


def apply_rigidbody_api(usd_path):
    stage = Usd.Stage.Open(usd_path)
    defaultPrim = stage.GetDefaultPrim()
    UsdPhysics.RigidBodyAPI.Apply(defaultPrim)
    stage.Save()


def count_rigid_api(usd_path):
    stage = Usd.Stage.Open(usd_path)
    rigid_api_count = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_api_count += 1
    return rigid_api_count

########################################################
## RLBench Specific
########################################################
def imply_output_path(input_path: str) -> str:
    task_name = os.path.dirname(os.path.relpath(input_path, 'data_rlbench/urdf')).removesuffix('_try')
    obj_name = os.path.basename(input_path).removesuffix('.urdf').removesuffix('.obj')
    return os.path.expanduser(f'~/cod/RoboVerse/roboverse_data/assets/rlbench/{task_name}/{obj_name}/usd/{obj_name}.usd')


def attach_texture_to_mesh(obj_path: str) -> str:
    obj_dir = os.path.dirname(obj_path)
    obj_name = os.path.basename(obj_path).removesuffix('.obj').removesuffix('_clean')

    cfg_paths = glob(f'{obj_dir}/{obj_name}_*.yaml')
    coordinates_list = []
    texture_list = []
    for cfg_path in cfg_paths:
        cfg = yaml.load(open(cfg_path), Loader=yaml.FullLoader)
        if 'texture_savepath' not in cfg:
            continue

        texture_path = cfg['texture_savepath']
        log.info(f'Attaching {texture_path} to {obj_name}')
        coordinates = np.array(cfg['texture_coordinates']).reshape(-1, 2)
        coordinates[:, 1] = 1 - coordinates[:, 1]  # flip y-axis
        texture = o3d.io.read_image(texture_path)

        coordinates_list.append(coordinates)
        texture_list.append(texture)

    if len(texture_list) == 0:
        return obj_path

    ## TODO: Need to concatenate the images when there are multiple textures
    mesh = o3d.io.read_triangle_mesh(obj_path)
    mesh.textures = texture_list
    mesh.triangle_uvs = o3d.utility.Vector2dVector(np.concatenate(coordinates_list, axis=0))
    mesh.triangle_material_ids = o3d.utility.IntVector(list(chain(*[[i] * len(coordinates) for i, coordinates in enumerate(coordinates_list)])))

    obj_textured_path = f'{obj_dir}/{obj_name}_textured.obj'
    o3d.io.write_triangle_mesh(obj_textured_path, mesh)
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(obj_textured_path)
    ms.save_current_mesh(obj_textured_path)
    return obj_textured_path

########################################################
## Main
########################################################
def main():
    if args.output is None:
        args.output = imply_output_path(args.input)

    # Main conversion
    if args.input.endswith(".obj"):
        obj_clean_path = make_obj_without_slashes(args.input)
        obj_textured_path = attach_texture_to_mesh(obj_clean_path)
        convert_obj_to_usd(obj_textured_path, args.output)
        ShaderFixer(args.output).fix_all()
        apply_rigidbody_api(args.output)
    elif args.input.endswith(".urdf"):
        urdf_unique_path = make_urdf_with_unique_visual_names(args.input)
        convert_urdf_to_usd(urdf_unique_path, args.output)
        if not is_articulation(args.output):
            apply_rigidbody_api(args.output)
    log.info(f'Rigid body count: {count_rigid_api(args.output)}')
    log.info(f"Saved USD file to {os.path.abspath(args.output)}")

    os.remove(os.path.join(os.path.dirname(args.output), '.asset_hash'))
    os.remove(os.path.join(os.path.dirname(args.output), 'config.yaml'))

    if args.exit_on_finish:
        return

    # Determine if there is a GUI to update:
    if not args.headless:
        # acquire settings interface
        carb_settings_iface = carb.settings.get_settings()
        # read flag for whether a local GUI is enabled
        local_gui = carb_settings_iface.get("/app/window/enabled")
        # read flag for whether livestreaming GUI is enabled
        livestream_gui = carb_settings_iface.get("/app/livestream/enabled")

        # Simulate scene (if not headless)
        if local_gui or livestream_gui:
            # Open the stage with USD
            stage_utils.open_stage(args.output)
            # Reinitialize the simulation
            app = omni.kit.app.get_app_interface()
            # Run simulation
            with contextlib.suppress(KeyboardInterrupt):
                while app.is_running():
                    # perform step
                    app.update()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
