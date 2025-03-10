from __future__ import annotations

import os
from typing import Iterator

import omni
from loguru import logger as log
from pxr import Sdf, Usd, UsdShade


def prim_descendants(root_prim: Usd.Prim) -> Iterator[Usd.Prim]:
    for prim in root_prim.GetAllChildren():
        yield prim
        yield from prim_descendants(prim)

class ShaderFixer:
    def __init__(self, usd_path: str, root_prim_path: str | None = None):
        self.stage: Usd.Stage = Usd.Stage.Open(usd_path)
        self.usd_path = usd_path
        self.root_prim = self.stage.GetPrimAtPath(root_prim_path) if root_prim_path else self.stage.GetDefaultPrim()
        self.pairs: list[tuple[Usd.Prim, UsdShade.Material, UsdShade.Shader]] = []  # obj_prim, material, shader
        self._find_all_targets()

    def _find_all_targets(self):
        for prim in prim_descendants(self.root_prim):
            material = UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterial()
            if material:
                shader = UsdShade.Shader(omni.usd.get_shader_from_material(material, get_prim=True))
                if shader:
                    self.pairs.append((prim, material, shader))
                else:
                    log.warning(f"No shader found for material {material.GetPrim().GetPath()}")

    def _find_unique_shaders(self) -> list[UsdShade.Shader]:
        unique_shaders = {}
        for _, _, shader in self.pairs:
            shader_path = shader.GetPrim().GetPath()
            if shader_path not in unique_shaders:
                unique_shaders[shader_path] = shader
        return list(unique_shaders.values())

    def _fix_single_shader(self, shader: UsdShade.Shader):
        shader_prim = shader.GetPrim()  # often == material_prim + '/Shader'
        log.debug(f"Trying to fix shader {shader_prim.GetPath()}")

        attr = shader_prim.GetAttribute("inputs:diffuse_texture")
        if not attr:
            log.debug("No diffuse_texture attribute found for shader, skipping")
            return

        texture_path = str(attr.Get()).replace("@", "")
        if os.path.isabs(texture_path):
            if os.path.exists(texture_path):
                usd_abs_path = os.path.abspath(self.usd_path)
                texture_abs_path = os.path.join(os.path.dirname(usd_abs_path), texture_path)
                texture_rel_path = os.path.relpath(texture_abs_path, os.path.dirname(self.usd_path))
                attr.Set(Sdf.AssetPath(texture_rel_path))
                log.info(f"Fixed shader {shader_prim.GetPath()}, replace diffuse_texture path from {texture_path} to {texture_rel_path}")
                return
            else:
                log.error(
                    f"diffuse_texture path {texture_path} is absolute and does not exist, please fix manually as"
                    " relative path!"
                )
                return
        else:
            usd_abs_path = os.path.abspath(self.usd_path)
            texture_abs_path = os.path.join(os.path.dirname(usd_abs_path), texture_path)
            if os.path.exists(texture_abs_path):
                log.info(
                    f"Shader {shader_prim.GetPath()} use relative path {texture_path} and can be fixed automatically!"
                )
                return
            else:
                log.error(
                    f"diffuse_texture path {texture_abs_path} (parsed from {texture_path}) does not exist, please fix"
                    " manually as relative path!"
                )
                return

    def fix_all(self):
        for shader in self._find_unique_shaders():
            self._fix_single_shader(shader)
        self.stage.Save()
