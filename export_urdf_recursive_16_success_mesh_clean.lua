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
    name = sim.getObjectName(handle)
    meshFilePath = string.format("%s/%s.dae", meshDir, name)

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
    sim.exportMesh(5, meshFilePath, 0, 1, allVertices, allIndices)
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

    if hasShapeOrJoint then
        return true
    else 
        -- Delete model if it does not have any shape or joint children, and it is not a shape or joint
        print(string.format("[INFO] Deleting model %s. Type: %d", name, modelType))
        -- sim.removeModel(handle)
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
function recursiveExport(handle, urdfDir)
    local name = sim.getObjectName(handle)
    local modelType = sim.getObjectType(handle)
    local urdfFilePath = string.format("%s/%s.urdf", urdfDir, name)
    
    -- Export URDF if it is a shape
    local exportSuccess = false
    if modelType == sim.object_shape_type then
        print(string.format('[INFO] Trying to export model %s or its children. Type: %d', name, modelType))
        local status, error = pcall(simURDF.export, handle, urdfFilePath)
        if status then
            exportSuccess = true
            print(string.format("[INFO] Successfully export %s", name))
        else
            print(string.format("[DEBUG] Failed to export %s as URDF. Error: %s", name, error))
            -- print(string.format("[INFO] Failed to export %s as URDF. Trying to export it as mesh", name))
            -- exportMesh(handle, urdfDir)
        end
    else
        print(string.format('[INFO] Skipping model %s. Type: %d', name, modelType))
    end

    -- Recursively export children
    if not exportSuccess then
        for i, childHandle in ipairs(getChildHandles(handle)) do
            recursiveExport(childHandle, urdfDir)
        end
    end
end

function loadAndExportURDF(ttmFilePath, urdfDir)
    local sceneHandle = sim.loadModel(ttmFilePath)
    keepShapeAndJoint(sceneHandle)
    recursiveExport(sceneHandle, urdfDir)
    return sceneHandle
end

-----------------------
-- Main
-----------------------

function sysCall_init()
    -- Import modules
    sim = require('sim')
    simURDF = require('simURDF')
    lfs = require('lfs')

    -- Set up file paths
    dataCase = "data"
    ttmDir = string.format("/home/fs/cod/try/urdf_preview/%s/ttm", dataCase)
    exportBaseDir = string.format("/home/fs/cod/try/urdf_preview/%s/urdf", dataCase)
    lfs.mkdir(exportBaseDir)
    
    --! DEBUG
    -- ttmFiles = {'close_box.ttm'}
    -- ttmFiles = {'setup_chess.ttm'}
    -- ttmFiles = {'meat_on_grill.ttm', 'insert_onto_square_peg.ttm', 'put_money_in_safe.ttm', 'wipe_desk.ttm'}
    ttmFiles = {'meat_on_grill.ttm'}
    -- ttmFiles = {'insert_onto_square_peg.ttm'}
    -- ttmFiles = {'put_money_in_safe.ttm'}
    -- ttmFiles = {'wipe_desk.ttm'}
    -- ttmFiles = {'put_money_in_safe.ttm'}
    -- ttmFiles = {'slide_block_to_target.ttm'}
    
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
