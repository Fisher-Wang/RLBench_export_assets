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

function getChildHandles(handle)
    -- bit0 set (1): exclude the tree base from the returned array
    -- bit1 set (2): include in the returned array only the object's first children.
    -- If the handle is sceneHandle, return the root handle
    return sim.getObjectsInTree(handle)
end

function recursiveExportChildURDF(handle, urdfDir)
    -- handle: scene handle or object handle
    print('Calling recursiveExportChildURDF')

    local childHandles = getChildHandles(handle)
    for i = 1, #childHandles do
        local childHandle = childHandles[i]
        local childName = sim.getObjectName(childHandle)

        local urdfFilePath = string.format("%s/%s.urdf", urdfDir, childName)
        print(urdfFilePath)
        simURDF.export(childHandle, urdfFilePath)
        -- local status, error = pcall(simURDF.export, childHandle, urdfFilePath)
        -- if not status then
        --     print(string.format("[DEBUG] Failed to export %s. Error: %s", childName, error))
        --     recursiveExportChildURDF(childHandle, urdfDir)
        -- else
        --     print(string.format("[INFO] Successfully export %s", childName))
        -- end
    end
end

function loadAndExportURDF(ttmFilePath, urdfDir)
    local sceneHandle = sim.loadModel(ttmFilePath, urdfDir)
    print(string.format("Loaded scene handle: %s", sceneHandle))
    recursiveExportChildURDF(sceneHandle)
end


-- Function to load TTM file and export URDF
function loadAndExportURDFHeuristic(ttmFilePath, urdfDir)
    -- Load the TTM file
    local sceneHandle = sim.loadModel(ttmFilePath)
    
    if sceneHandle ~= -1 then
        print("Success to load TTM file")
        print(string.format("Loaded scene handle: %s", sceneHandle))

        -- Get the model name from the TTM file name
        local ttmFileName = string.match(ttmFilePath, "([^/\\]+)%.ttm$")
        local modelName = string.gsub(ttmFileName, "%.ttm$", "")
        
        local childHandles = sim.getObjectsInTree(sceneHandle)
        -- Get root object handle
        for i = 1, #childHandles do
            local childHandle = childHandles[i]
            local parentHandle = sim.getObjectParent(childHandle)
            if parentHandle == -1 then
                rootHandle = childHandle
                break
            end
        end

        -- Select candidates for URDF export
        rootHandle = getRootHandle(sceneHandle)
        candidate_handles = {}

        for i = 1, #childHandles do
            local childHandle = childHandles[i]
            local childType = sim.getObjectType(childHandle)
            local parentHandle = sim.getObjectParent(childHandle)
            if childType == sim.object_shape_type and parentHandle == rootHandle and isDynamicallyEnabled(childHandle) then
                table.insert(candidate_handles, childHandle)
            end
        end
        
        if #candidate_handles == 0 then
            for i = 1, #childHandles do
                local childHandle = childHandles[i]
                local childType = sim.getObjectType(childHandle)
                if childType == sim.object_shape_type and isDynamicallyEnabled(childHandle) then
                    table.insert(candidate_handles, childHandle)
                end
            end
        end
        
        -- Export the URDF using simURDF plugin
        if #candidate_handles == 0 then
            print("No candidates found")
        else
            for i = 1, #candidate_handles do
                local childHandle = candidate_handles[i]
                local childName = sim.getObjectName(childHandle)
                local childType = sim.getObjectType(childHandle)
                print(string.format("Child name: %s", childName))
                print(string.format("Child type: %s", childType))
                
                local urdfFilePath = string.format("%s/%s.urdf", urdfDir, childName)
                local result = simURDF.export(childHandle, urdfFilePath)
                
                if result == simURDF.result_ok then
                    print(string.format("Exported URDF: %s", urdfFilePath))
                else
                    print(string.format("Failed to export URDF: %s", urdfFilePath))
                end
            end
        end

        -- Remove the loaded model from the scene
        -- sim.removeModel(sceneHandle)
    else
        print(string.format("Failed to load TTM file: %s", ttmFilePath))
    end
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
        
        -- loadAndExportURDF(ttmFilePath, urdfDir)
        loadAndExportURDFHeuristic(ttmFilePath, exportDir)
        
        -- local status, error = pcall(loadAndExportURDF, ttmFilePath, urdfDir)
        -- if not status then
        --     print(string.format("Error occurred while processing %s. Error: %s", ttmFile, error))
        -- end
        
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
