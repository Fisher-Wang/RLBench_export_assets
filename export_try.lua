-----------------------
-- Lua Utils
-----------------------
function find_files(dir, pattern)
    local file_list = {}
    for file in lfs.dir(dir) do
        if string.find(file, pattern) then
            table.insert(file_list, file)
        end
    end
    return file_list
end

function writeYAML(filePath, data)
    local yamlString = lyaml.dump({data})
    local file, err = io.open(filePath, 'w')
    if not file then
        error(string.format("[Error] Failed to open file %s. Error: %s", filePath, err))
    end
    file:write(yamlString)
    file:close()
end

-----------------------
-- CoppeliaSim Utils
-----------------------
function hasChildJoint(handle)
    local childHandles = sim.getObjectsInTree(handle)
    for i = 1, #childHandles do
        local childHandle = childHandles[i]
        local childType = sim.getObjectType(childHandle)
        local parentHandle = sim.getObjectParent(childHandle)
        if childType == sim.object_joint_type and parentHandle == handle then
            return true
        end
    end
    return false
end

function getRootHandle(sceneHandle)
    local childHandles = sim.getObjectsInTree(sceneHandle)
    for i = 1, #childHandles do
        local childHandle = childHandles[i]
        local parentHandle = sim.getObjectParent(childHandle)
        if parentHandle == -1 then
            return childHandle
        end
    end
    return -1
end

function getParentName(handle)
    local parentHandle = sim.getObjectParent(handle)
    if parentHandle == -1 then
        return 'None'
    else
        return sim.getObjectName(parentHandle)
    end
end

function isDynamicallyEnabled(h)
    -- https://forum.coppeliarobotics.com/viewtopic.php?t=10252
    local r = false
    if sim.getObjectType(h) == sim.object_joint_type then
        r = sim.isDynamicallyEnabled(h)
    end
    if sim.getObjectType(h) == sim.object_shape_type then
        if sim.getSimulationState() ~= sim.simulation_stopped then
            if (sim.getObjectInt32Param(h,sim.shapeintparam_respondable) ~= 0) and (sim.getObjectInt32Param(h,sim.shapeintparam_static) == 0) then
                r = true
            end
        end
    end
    return r
end

function getRootHandles(sceneHandle)
    local childHandles = sim.getObjectsInTree(sceneHandle)
    local rootHandles = {}
    for i = 1, #childHandles do
        local childHandle = childHandles[i]
        local parentHandle = sim.getObjectParent(childHandle)
        if parentHandle == -1 then
            table.insert(rootHandles, childHandle)
        end
    end
    return rootHandles
end

function getChildHandles(handle)
    local rst = {}
    local childHandles = sim.getObjectsInTree(handle)
    for i, childHandle in ipairs(childHandles) do
        local parentHandle = sim.getObjectParent(childHandle)
        if parentHandle == handle then
            table.insert(rst, childHandle)
        end
    end
    return rst
end

function handle2name(handle)
    return sim.getObjectName(handle)
end

function handles2names(handles)
    names = {}
    for i, handle in ipairs(handles) do
        table.insert(names, handle2name(handle))
    end
    return names
end

function exportMesh(handle, meshDir)
    -- https://manual.coppeliarobotics.com/en/regularApi/simExportMesh.htm
    name = sim.getObjectName(handle)
    meshFilePath = string.format("%s/%s.stl", meshDir, name)

    local allVertices = {}
    local allIndices = {}

    -- Prepare vertices
    local vertices, indices = sim.getShapeMesh(handle)
    local m = sim.getObjectMatrix(handle)
    for i = 1, #vertices // 3 do
        local v = {vertices[3 * (i - 1) +1], vertices[3 * (i - 1) + 2], vertices[3 * (i - 1) + 3]}
        v = sim.multiplyVector(m, v)
        vertices[3 * (i - 1) + 1] = v[1]
        vertices[3 * (i - 1) + 2] = v[2]
        vertices[3 * (i - 1) + 3] = v[3]
    end

    table.insert(allVertices, vertices)
    table.insert(allIndices, indices)    

    -- 0: OBJ format
    -- 3: TEXT STL format
    -- 4: BINARY STL format
    -- 5: COLLADA format
    -- 6: TEXT PLY format
    -- 7: BINARY PLY format
    sim.exportMesh(4, meshFilePath, 0, 1, allVertices, allIndices)
end

-----------------------
-- Delete non-relavant models
-----------------------

function recursiveKeepShapeAndJoint(handle)
    local name = sim.getObjectName(handle)
    local modelType = sim.getObjectType(handle)
    local parentName = getParentName(handle)
    print(string.format('[DEBUG] Checking model %s. Type: %d. Parent: %s', name, modelType, parentName))

    childHandles = getChildHandles(handle)

    -- Recursively check children
    local hasShapeOrJoint = false
    for i, childHandle in ipairs(childHandles) do
        local childHasShapeOrJoint = recursiveKeepShapeAndJoint(childHandle)
        hasShapeOrJoint = hasShapeOrJoint or childHasShapeOrJoint
    end
    hasShapeOrJoint = hasShapeOrJoint or modelType == sim.object_shape_type or modelType == sim.object_joint_type
    
    -- Delete model if it does not have any shape or joint children, and it is not a shape or joint
    if hasShapeOrJoint then
        return true
    else 
        print(string.format("[INFO] Deleting model %s. Type: %d", name, modelType))
        sim.removeObjects({handle})
        return false
    end
end

function keepShapeAndJoint(sceneHandle)
    rootHandles = getRootHandles(sceneHandle)
    for i, rootHandle in ipairs(rootHandles) do
        recursiveKeepShapeAndJoint(rootHandle)
    end
end

-----------------------
-- Export
-----------------------

function exportWaypoint(handle, filePath)
    local position = sim.getObjectPosition(handle)
    local orientation = sim.getObjectOrientation(handle)
    local pose = sim.getObjectPose(handle)
    data = {
        position = position,
        orientation = orientation,
        pose = pose
    }
    writeYAML(filePath, data)
end

function exportPath(handle, filePath)
    tags = sim.readCustomDataBlockTags(handle)
    print(tags)
    dataBlock = sim.readCustomDataBlock(handle, 'PATH')

    print(dataBlock)
    pathData = sim.unpackDoubleTable(dataBlock)
    print(pathData)
    writeYAML(filePath, {{data=pathData}})
end

function exportJoint(handle, filePath)
    local friction = sim.getEngineFloatParam(sim.bullet_body_friction, handle)
    local jointPosition = sim.getJointPosition(handle)
    local jointTargetPosition = sim.getJointTargetPosition(handle)
    local jointTargetVelocity = sim.getJointTargetVelocity(handle)
    local jointVelocity = sim.getJointVelocity(handle)
    local jointInterval = sim.getJointInterval(handle)
    local jointType = sim.getJointType(handle)
    local jointMode = sim.getJointMode(handle)
    local jointDependency = sim.getJointDependency(handle)
    local jointForce = sim.getJointForce(handle)
    local jointTargetForce = sim.getJointTargetForce(handle)
    data = {
        friction = friction,
        jointPosition = jointPosition,
        jointTargetPosition = jointTargetPosition,
        jointTargetVelocity = jointTargetVelocity,
        jointVelocity = jointVelocity,
        jointInterval = jointInterval,
        jointType = jointType,
        jointMode = jointMode,
        jointDependency = jointDependency,
        jointForce = jointForce,
        jointTargetForce = jointTargetForce
    }
    writeYAML(filePath, data)
end

function recursiveExport(handle, exportDir, exportURDF)
    local name = sim.getObjectName(handle)
    local modelType = sim.getObjectType(handle)
    
    local exportURDFSuccess = false
    -- If is shape, export URDF or Mesh
    if modelType == sim.object_shape_type and exportURDF then
        print(string.format('[INFO] Trying to export model %s. Type: %d', name, modelType))
        local urdfFilePath = string.format("%s/%s.urdf", exportDir, name)
        local status, error = pcall(simURDF.export, handle, urdfFilePath)
        if status then
            exportURDFSuccess = true
            print(string.format("[INFO] Successfully export %s, resetting position and orientation", name))
            sim.setObjectPosition(handle, {0, 0, 0}, sim.handle_world)
            sim.setObjectOrientation(handle, {0, 0, 0}, sim.handle_world)
            local status, error = pcall(simURDF.export, handle, urdfFilePath)
            if status then
                print(string.format("[INFO] Successfully export %s in canonical space", name))
                print(string.format("[INFO] %s position", name), sim.getObjectPosition(handle))
                print(string.format("[INFO] %s orientation", name), sim.getObjectOrientation(handle))
            else
                print(string.format("[ERROR] Failed to export %s as URDF. Error: %s", name, error))
            end
        else
            print(string.format("[INFO] Failed to export %s as URDF. Error: %s", name, error))
            print(string.format("[INFO] Trying to export %s as mesh", name))
            sim.setObjectPosition(handle, {0, 0, 0}, sim.handle_world)
            sim.setObjectOrientation(handle, {0, 0, 0}, sim.handle_world)
            local status, error = pcall(exportMesh, handle, exportDir)
            if status then
                print(string.format("[INFO] Successfully export %s as mesh", name))
            else
                print(string.format("[ERROR] Failed to export %s as mesh. Error: %s", name, error))
            end
        end
    -- If is joint
    elseif modelType == sim.object_joint_type then
        print(string.format('[INFO] Export joint %s', name))
        local yamlFilePath = string.format("%s/%s.yaml", exportDir, name)
        exportJoint(handle, yamlFilePath)

    -- If is waypoint
    elseif modelType == sim.object_dummy_type and string.find(name, 'waypoint') then
        print(string.format('[INFO] Exporting waypoint %s. Type: %d', name, modelType))
        local yamlFilePath = string.format("%s/%s.yaml", exportDir, name)
        exportWaypoint(handle, yamlFilePath)
    -- If is path
    elseif modelType == sim.object_path_type then
        print(string.format('[INFO] Skipping path %s. Type: %d', name, modelType))
        -- print(string.format('[INFO] Exporting path %s. Type: %d', name, modelType))
        -- local yamlFilePath = string.format("%s/%s.yaml", exportDir, name)
        -- exportPath(handle, yamlFilePath)
    else
        print(string.format('[INFO] Skipping model %s. Type: %d', name, modelType))
    end

    -- Recursively export children
    local exportChildURDF = exportURDF and not exportURDFSuccess
    for i, childHandle in ipairs(getChildHandles(handle)) do
        recursiveExport(childHandle, exportDir, exportChildURDF)
    end
end

function loadAndExportURDF(ttmFilePath, urdfDir)
    local sceneHandle = sim.loadModel(ttmFilePath)
    -- keepShapeAndJoint(sceneHandle)
    setRootObjectsToOrigin(sceneHandle)
    recursiveExport(sceneHandle, urdfDir, true)
    return sceneHandle
end

function setRootObjectsToOrigin(sceneHandle)
    -- TODO: Check for task other than close_box
    rootHandles = getRootHandles(sceneHandle)
    for i, rootHandle in ipairs(rootHandles) do
        sim.setObjectPosition(rootHandle, {0, 0, 0}, sim.handle_world)
        sim.setObjectOrientation(rootHandle, {0, 0, 0}, sim.handle_world)
        childHandles = getChildHandles(rootHandle)
        for j, childHandle in ipairs(childHandles) do
            sim.setObjectPosition(childHandle, {0, 0, 0}, sim.handle_world)
            sim.setObjectOrientation(childHandle, {0, 0, 0}, sim.handle_world)
        end
    end
end

-----------------------
-- Main
-----------------------

function sysCall_init()
    -- Import modules
    sim = require('sim')
    simURDF = require('simURDF')
    lfs = require('lfs')
    lyaml = require('lyaml')

    -- Set up file paths
    dataCase = "data_rlbench"
    ttmDir = string.format("/home/fs/cod/try/urdf_preview/%s/ttm", dataCase)
    exportBaseDir = string.format("/home/fs/cod/try/urdf_preview/%s/urdf", dataCase)
    lfs.mkdir(exportBaseDir)

    --! DEBUG
    -- ttmFiles = {'close_box.ttm'}
    -- ttmFiles = {'empty_dishwasher.ttm'}
    -- ttmFiles = {'open_drawer.ttm'}
    -- ttmFiles = {'slide_block_to_target.ttm'}
    -- ttmFiles = {'reach_and_drag.ttm'}
    -- ttmFiles = {'setup_chess.ttm'}
    -- ttmFiles = {'stack_cups.ttm'}
    ttmFiles = {'basketball_in_hoop.ttm'}
    
    -- Iterate over each TTM file and export URDF
    -- ttmFiles = find_files(ttmDir, '.ttm')
    for i, ttmFile in ipairs(ttmFiles) do
        print(string.format("==== Processing %s ====", ttmFile))
        local ttmFilePath = string.format("%s/%s", ttmDir, ttmFile)
        local modelName = string.gsub(ttmFile, "%.ttm$", "")
        exportDir = string.format("%s/%s", exportBaseDir, modelName)
        lfs.mkdir(exportDir)
        
        sceneHandle = loadAndExportURDF(ttmFilePath, exportDir)
        -- local status, error = pcall(loadAndExportURDF, ttmFilePath, urdfDir)
        -- if not status then
        --     print(string.format("[Error] Error occurred while processing %s. Error: %s", ttmFile, error))
        -- end

        -- sim.removeModel(sceneHandle)
        print('')
    end
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- put your cleanup code here
end
