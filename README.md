# Convert RLBench assets to URDF/USD

## Acknowledgements

The assets in `data_rlbench/ttm` are from [RLBench](https://github.com/stepjam/RLBench), licensed under the [RLBench's license](data_rlbench/ttm/LICENSE).

## Installation

Change the `simURDF.export` function in `lua/simURDF.lua` to the following:

```
function simURDF.export(origModel, fileName, options)
    options = options or 0
    -- options & bit0: transform dummies into red cubes
    -- options & bit1: make visuals and collisions origin frame where parent joint is, if possible
    -- options & bit2: do not reset joint positions to 0
    assert(sim.isHandle(origModel), 'not a valid handle')
    local baseName = fileName
    if fileName:find('%.urdf$') then
        baseName = fileName:sub(1, -6)
    else
        fileName = fileName .. '.urdf'
    end

    -- local model = sim.copyPasteObjects({origModel}, 1 + 2 + 4 + 8 + 16 + 32)[1] -- work with a copy
    local model = origModel
    sim.setModelProperty(model, sim.getModelProperty(origModel))
    if (options & 1) ~= 0 then _S.urdf.replaceDummies(model) end
    if (options & 4) == 0 then
        local l = sim.getObjectsInTree(model, sim.sceneobject_joint)
        for i = 1, #l, 1 do
            if sim.getJointType(l[i]) ~= sim.joint_spherical then
                sim.setJointPosition(l[i], 0)
            end
        end
    end
    _S.urdf.insertAuxShapes(model)
    local info = {baseFile = baseName, base = model, options = options}
    local tree = _S.urdf.newNode({'robot', name = sim.getObjectAlias(model)})
    local dynamicStage = 0
    local mprop = sim.getModelProperty(model)
    if (mprop & sim.modelproperty_not_dynamic) ~= 0 then dynamicStage = 2 end
    local tree = _S.urdf.parseAndCreateMeshFiles(tree, model, -1, -1, dynamicStage, info)
    -- sim.removeModel(model)
    local xml = _S.urdf.toXML(tree)
    local f = io.open(fileName, 'w+')
    f:write(xml)
    f:close()
end
```


## Quick Reference for CoppeliaSim

### Types

#### Object Types

```
sim.object_shape_type: 0
sim.object_joint_type: 1
sim.object_graph_type: 2
sim.object_camera_type: 3
sim.object_dummy_type: 4
sim.object_proximitysensor_type: 5
sim.object_path_type: 8
sim.object_visionsensor_type: 9
sim.object_forcesensor_type: 12
sim.object_light_type: 13
sim.object_octree_type: 15
sim.object_pointcloud_type: 16
```

#### Object Subtypes
```
sim.light_omnidirectional_subtype: 1
sim.light_spot_subtype: 2
sim.light_directional_subtype: 3
sim.joint_revolute_subtype: 10
sim.joint_prismatic_subtype: 11
sim.joint_spherical_subtype: 12
sim.shape_simpleshape_subtype: 20
sim.shape_multishape_subtype: 21
sim.proximitysensor_pyramid_subtype: 30
sim.proximitysensor_cylinder_subtype: 31
sim.proximitysensor_disc_subtype: 32
sim.proximitysensor_cone_subtype: 33
sim.proximitysensor_ray_subtype: 34
```

#### Joint Subtypes
```
sim.joint_revolute_subtype: 10
sim.joint_prismatic_subtype: 11
sim.joint_spherical_subtype: 12
```
