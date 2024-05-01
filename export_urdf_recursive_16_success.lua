function find_files(dir, pattern)
    local file_list = {}
    for file in lfs.dir(dir) do
        if string.find(file, pattern) then
            table.insert(file_list, file)
        end
    end
    return file_list
end

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

function recursiveExport(handle, urdfDir)
    name = sim.getObjectName(handle)
    modelType = sim.getObjectType(handle)
    
    local exportSuccess = false
    if modelType == sim.object_shape_type then
        print(string.format('[INFO] Trying to export model %s or its children. Type: %d', name, modelType))
        local urdfFilePath = string.format("%s/%s.urdf", urdfDir, name)
        local status, error = pcall(simURDF.export, handle, urdfFilePath)
        if status then
            exportSuccess = true
            print(string.format("[INFO] Successfully export %s", name))
        else
            print(string.format("[INFO] Failed to export %s. \n[DEBUG] Error: %s", name, error))
        end
    else
        print(string.format('[INFO] Skipping model %s. Type: %d', name, modelType))
    end

    if not exportSuccess then
        for i, childHandle in ipairs(getChildHandles(handle)) do
            recursiveExport(childHandle, urdfDir)
        end
    end
end

function loadAndExportURDF(ttmFilePath, urdfDir)
    local sceneHandle = sim.loadModel(ttmFilePath)
    -- print(handles2names(getRootHandles(sceneHandle)))
    -- for i, rootHandle in ipairs(getRootHandles(sceneHandle)) do
    --     local modelName = handle2name(rootHandle)
    --     print('Model name', modelName)
    --     print('Child models', handles2names(getChildHandles(rootHandle)))
        
    --     recursiveExportURDF(rootHandle, urdfDir)
    -- end
    recursiveExport(sceneHandle, urdfDir)
end

function sysCall_init()
    sim = require('sim')
    simURDF = require('simURDF')
    lfs = require('lfs')

    ttmDir = "/home/fs/cod/try/urdf_preview/data/ttm"
    ttmFiles = find_files(ttmDir, '.ttm')

    --! DEBUG
    ttmFiles = {'close_box.ttm'}
    -- ttmFiles = {'setup_chess.ttm'}

    -- Iterate over each TTM file and export URDF
    for i = 1, #ttmFiles do
        ttmFile = ttmFiles[i]
        print(string.format("==== Processing %s ====", ttmFile))
        local ttmFilePath = string.format("%s/%s", ttmDir, ttmFile)
        local modelName = string.gsub(ttmFile, "%.ttm$", "")
        exportDir = string.format("/home/fs/cod/try/urdf_preview/data/urdf/%s", modelName)
        lfs.mkdir(exportDir)
        
        local status, error = pcall(loadAndExportURDF, ttmFilePath, exportDir)
        if not status then
            print(string.format("[Error] Error occurred while processing %s. Error: %s", ttmFile, error))
        end
        
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
