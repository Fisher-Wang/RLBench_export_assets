########################################################
## Commandline arguments
########################################################

from dataclasses import dataclass
from typing import Literal

import tyro


@dataclass
class Args:
    input: str
    output: str
    make_instanceable: bool = False
    collision_approximation: Literal["convexHull", "convexDecomposition", "none"] = "convexHull"
    """The method used for approximating collision mesh. Set to "none" to not add a collision mesh to the converted mesh."""
    mass: float | None = None
    """The mass (in kg) to assign to the converted asset. If not provided, then no mass is added."""
    headless: bool = True


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

import carb
import omni.isaac.core.utils.stage as stage_utils
import omni.kit.app
from loguru import logger as log
from omni.isaac.lab.sim.converters import MeshConverter, MeshConverterCfg
from omni.isaac.lab.sim.schemas import schemas_cfg
from omni.isaac.lab.utils.assets import check_file_path
from pxr import Usd, UsdPhysics
from rich.logging import RichHandler

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


def apply_rigidbody_api(usd_path):
    stage = Usd.Stage.Open(usd_path)
    defaultPrim = stage.GetDefaultPrim()
    UsdPhysics.RigidBodyAPI.Apply(defaultPrim)
    stage.Save()


def main():
    obj_path = args.input
    usd_path = args.output

    # Main conversion
    convert_obj_to_usd(obj_path, usd_path)
    apply_rigidbody_api(usd_path)
    log.info(f"Saved USD file to {os.path.abspath(usd_path)}")

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
            stage_utils.open_stage(usd_path)
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
