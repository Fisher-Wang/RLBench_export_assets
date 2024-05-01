function getObjectByName(sceneHandle, name)
    local childHandles = sim.getObjectsInTree(sceneHandle)
    for i, childHandle in ipairs(childHandles) do
        local childName = sim.getObjectName(childHandle)
        if childName == name then
            return childHandle
        end
    end
    return nil
end

function getPoseOnPath(pathHandle, relativeDistance)
    local pos = sim.getPositionOnPath(pathHandle, relativeDistance)
    local ori = sim.getOrientationOnPath(pathHandle, relativeDistance)
    return pos, ori
end

function linspace(start, stop, step)
    local result = {}
    local count = 0
    step = step or 1

    while start + count * step <= stop do
        result[#result + 1] = start + count * step
        count = count + 1
    end

    return result
end

function exportPath(ttmFilePath)
    local sceneHandle = sim.loadModel(ttmFilePath)

    local handle = getObjectByName(sceneHandle, 'waypoint2')
    local name = sim.getObjectName(handle)
    
    local numbers = linspace(0, 1, 0.1)
    for i, number in ipairs(numbers) do
        print('relativeDistance', number)
        local pos, ori = getPoseOnPath(handle, number)
        print(pos, ori)
    end

    return sceneHandle
end

function sysCall_init()
    sim = require('sim')

    dataCase = "data"
    ttmDir = string.format("/home/fs/cod/try/urdf_preview/%s/ttm", dataCase)

    ttmFiles = {'close_box.ttm'}

    for i, ttmFile in ipairs(ttmFiles) do
        print(string.format("==== Processing %s ====", ttmFile))
        local ttmFilePath = string.format("%s/%s", ttmDir, ttmFile)
        
        sceneHandle = exportPath(ttmFilePath)
        print('')
    end

end

function sysCall_actuation()
end

function sysCall_sensing()
end

function sysCall_cleanup()
end
