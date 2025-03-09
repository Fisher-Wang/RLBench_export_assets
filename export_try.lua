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
function getParentName(handle)
    local parentHandle = sim.getObjectParent(handle)
    if parentHandle == -1 then
        return 'None'
    else
        return sim.getObjectName(parentHandle)
    end
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

function exportMesh(handle, meshDir)
    -- https://manual.coppeliarobotics.com/en/regularApi/simExportMesh.htm
    name = sim.getObjectName(handle)
    meshFilePath = string.format("%s/%s.obj", meshDir, name)

    local allVertices = {}
    local allIndices = {}

    -- Export mesh
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
    sim.exportMesh(0, meshFilePath, 0, 1, allVertices, allIndices)

    -- Export texture
    i_viz = 0
    while true do
        data = sim.getShapeViz(handle, i_viz)
        if data == nil then
            break
        end

        local data_save = {
            colors = data.colors,
            indices = data.indices,
            vertices = data.vertices,
        }

        if data.texture ~= nil then
            texture_buffer = data.texture.texture
            texture_resolution = data.texture.resolution
            texture_savepath = string.format("%s/texture_%d.png", meshDir, data.texture.id)
            print(string.format("[INFO] Saving %s texture to %s", name, texture_savepath))
            sim.saveImage(texture_buffer, texture_resolution, 1, texture_savepath, 100)
            data_save.texture_savepath = texture_savepath
            data_save.texture_resolution = texture_resolution
            data_save.texture_coordinates = data.texture.coordinates
        end
        writeYAML(string.format("%s/%s_%d.yaml", meshDir, name, i_viz), data_save)

        i_viz = i_viz + 1
    end
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

function recursiveGetSummary(handle, exportDir, data)
    local name = sim.getObjectName(handle)
    local modelType = sim.getObjectType(handle)

    if modelType == sim.object_shape_type then
        table.insert(data['objects'], name)
    elseif modelType == sim.object_joint_type then
        table.insert(data['joints'], name)
    end

    for i, childHandles in ipairs(getChildHandles(handle)) do
        recursiveGetSummary(childHandles, exportDir, data)
    end
end

function getSummary(sceneHandle, exportDir)
    local data = {}
    data['objects'] = {}
    data['joints'] = {}
    recursiveGetSummary(sceneHandle, exportDir, data)
    writeYAML(string.format("%s/summary.yaml", exportDir), data)
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
        end
        if true then
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
    -- elseif modelType == sim.object_dummy_type and string.find(name, 'waypoint') then
    --     print(string.format('[INFO] Exporting waypoint %s. Type: %d', name, modelType))
    --     local yamlFilePath = string.format("%s/%s.yaml", exportDir, name)
    --     exportWaypoint(handle, yamlFilePath)
    else
        print(string.format('[INFO] Skipping model %s. Type: %d', name, modelType))
    end

    -- Recursively export children
    -- local exportChildURDF = exportURDF and not exportURDFSuccess
    local exportChildURDF = true
    for i, childHandle in ipairs(getChildHandles(handle)) do
        recursiveExport(childHandle, exportDir, exportChildURDF)
    end
end

function loadAndExportURDF(ttmFilePath, urdfDir)
    local sceneHandle = sim.loadModel(ttmFilePath)
    -- keepShapeAndJoint(sceneHandle)
    setRootObjectsToOrigin(sceneHandle)
    recursiveExport(sceneHandle, urdfDir, true)
    -- getSummary(sceneHandle, urdfDir)
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
    ttmDir = string.format("/home/haoran/cod/RLBench_export_assets/%s/ttm", dataCase)
    exportBaseDir = string.format("/home/haoran/cod/RLBench_export_assets/%s/urdf", dataCase)
    lfs.mkdir(exportBaseDir)

    ttmFiles = {
        "basketball_in_hoop.ttm",
        "beat_the_buzz.ttm",
        "block_pyramid.ttm",
        "change_channel.ttm",
        "change_clock.ttm",
        "close_box.ttm",
        "close_door.ttm",
        "close_drawer.ttm",
        "close_fridge.ttm",
        "close_grill.ttm",
        "close_jar.ttm",
        "close_laptop_lid.ttm",
        "close_microwave.ttm",
        "empty_container.ttm",
        "empty_dishwasher.ttm",
        "get_ice_from_fridge.ttm",
        "hang_frame_on_hanger.ttm",
        "hit_ball_with_queue.ttm",
        "hockey.ttm",
        "insert_onto_square_peg.ttm",
        "insert_usb_in_computer.ttm",
        "lamp_off.ttm",
        "lamp_on.ttm",
        "lift_numbered_block.ttm",
        "light_bulb_in.ttm",
        "light_bulb_out.ttm",
        "meat_off_grill.ttm",
        "meat_on_grill.ttm",
        "open_box.ttm",
        "open_door.ttm",
        "open_drawer.ttm",
        "open_fridge.ttm",
        "open_grill.ttm",
        "open_jar.ttm",
        "open_microwave.ttm",
        "open_oven_try.ttm",  -- use modified version to export
        "open_washing_machine.ttm",
        "open_window_try.ttm",  -- use modified version to export
        "open_wine_bottle.ttm",
        "phone_on_base.ttm",
        "pick_and_lift.ttm",
        "pick_and_lift_small.ttm",
        "pick_up_cup.ttm",
        "place_cups_try.ttm",
        "place_shape_in_shape_sorter.ttm",
        "play_jenga.ttm",
        "plug_charger_in_power_supply.ttm",
        "pour_from_cup_to_cup.ttm",
        "press_switch_try.ttm",
        "push_button.ttm",
        "push_buttons.ttm",
        "put_all_groceries_in_cupboard.ttm",
        "put_books_on_bookshelf.ttm",
        "put_bottle_in_fridge_try.ttm",
        "put_groceries_in_cupboard.ttm",
        "put_item_in_drawer.ttm",
        "put_knife_in_knife_block.ttm",
        "put_knife_on_chopping_board.ttm",
        "put_money_in_safe_try.ttm",
        "put_plate_in_colored_dish_rack_try.ttm",
        "put_rubbish_in_bin.ttm",
        "put_shoes_in_box.ttm",
        "put_toilet_roll_on_stand.ttm",
        "put_tray_in_oven.ttm",
        "put_umbrella_in_umbrella_stand.ttm",
        "reach_and_drag.ttm",
        "reach_target.ttm",
        "remove_cups.ttm",
        "scoop_with_spatula.ttm",
        "screw_nail.ttm",
        "set_the_table.ttm",
        "setup_checkers.ttm",
        "setup_chess.ttm",
        "slide_block_to_target.ttm",
        "slide_cabinet_open_and_place_cups.ttm",
        "solve_puzzle.ttm",
        "stack_blocks.ttm",
        "stack_chairs.ttm",
        "stack_cups.ttm",
        "stack_wine.ttm",
        -- "straighten_rope.ttm",  -- spherical joint cannot be exported to URDF
        "sweep_to_dustpan.ttm",
        "take_cup_out_from_cabinet.ttm",
        "take_frame_off_hanger.ttm",
        "take_item_out_of_drawer.ttm",
        "take_lid_off_saucepan.ttm",
        "take_money_out_safe.ttm",
        "take_off_weighing_scales.ttm",
        "take_plate_off_colored_dish_rack.ttm",
        "take_shoes_out_of_box.ttm",
        "take_toilet_roll_off_stand.ttm",
        "take_tray_out_of_oven.ttm",
        "take_umbrella_out_of_umbrella_stand.ttm",
        "take_usb_out_of_computer.ttm",
        "toilet_seat_down.ttm",
        "toilet_seat_up.ttm",
        "turn_oven_on.ttm",
        "turn_tap.ttm",
        "tv_on.ttm",
        "unplug_charger.ttm",
        "water_plants.ttm",
        "weighing_scales.ttm",
        "wipe_desk.ttm",
    }

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

        sim.removeModel(sceneHandle)
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
