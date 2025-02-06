import yaml
import open3d as o3d
import copy
import numpy as np
import functools
import os
from loguru import logger as log
import pymeshlab


task_name = 'basketball_in_hoop'
src_dir = f'data_rlbench/urdf/{task_name}'
obj_names = [x.removesuffix('.obj') for x in os.listdir(src_dir) if x.endswith('.obj') and not x.endswith('_colored.obj') and not x.endswith('_textured.obj')]


def colorize_single_mesh(src_dir, obj_name):
    path = f'{src_dir}/{obj_name}.obj'
    mesh = o3d.io.read_triangle_mesh(path)

    meshes = []
    cfg_idx = 0
    while True:
        cfg_path = f'{src_dir}/{obj_name}_{cfg_idx}.yaml'
        if not os.path.exists(cfg_path):
            break
        
        cfg = yaml.load(open(cfg_path), Loader=yaml.FullLoader)
        indices = np.array(cfg['indices']).reshape(-1, 3)
        colors = np.array(cfg['colors']).reshape(-1, 3)
        vertices = np.array(cfg['vertices']).reshape(-1, 3)
        
        recorded_triangle_indices = np.where((np.asarray(mesh.triangles)[:, None, :] == indices[None,:, :]).all(-1).any(-1))[0]
        triangle_vertice_indices = np.asarray(mesh.triangles)[recorded_triangle_indices].flatten()
        recorded_vertices_indices = np.where((np.isclose(np.asarray(mesh.vertices)[:, None, :], vertices[None,:, :])).all(-1).any(-1))[0]
        vertices_indices = np.unique(np.concatenate([triangle_vertice_indices, recorded_vertices_indices]))
        triangle_indices = np.where((np.asarray(mesh.triangles)[:, None, :] == vertices_indices[None,:, None]).any(1).all(-1))[0]
        
        mesh1 = copy.deepcopy(mesh)
        mesh1.triangles = o3d.utility.Vector3iVector(np.asarray(mesh1.triangles)[triangle_indices])
        mesh1.paint_uniform_color(colors[0])
        meshes.append(mesh1)
        
        log.info(f'Assigned color {colors[0]} to {cfg_idx}th mesh')
        
        cfg_idx += 1

    combined_mesh = functools.reduce(lambda x, y: x + y, meshes)
    o3d.io.write_triangle_mesh(f'{src_dir}/{obj_name}_colored.obj', combined_mesh)


def attach_texture(src_dir, obj_name):
    ## TODO: This function should be merged into the colorize_single_mesh function, so that it can handle the case when there are multiple parts of mesh each with their own texture
    path = f'{src_dir}/{obj_name}.obj'
    mesh = o3d.io.read_triangle_mesh(path)
    cfg_path = f'{src_dir}/{obj_name}_0.yaml'
    cfg = yaml.load(open(cfg_path), Loader=yaml.FullLoader)
    coordinates = np.array(cfg['texture_coordinates']).reshape(-1, 2)
    texture_path = cfg['texture_savepath']
    texture = o3d.io.read_image(texture_path)
    mesh.textures = [texture]
    mesh.triangle_uvs = o3d.utility.Vector2dVector(coordinates)
    o3d.io.write_triangle_mesh(f'{src_dir}/{obj_name}_textured.obj', mesh)
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(f'{src_dir}/{obj_name}_textured.obj')
    ms.save_current_mesh(f'{src_dir}/{obj_name}_textured.obj')


def main():
    for obj_name in obj_names:
        log.info(f'Processing {obj_name}')
        colorize_single_mesh(src_dir, obj_name)
        attach_texture(src_dir, obj_name)


if __name__ == '__main__':
    main()