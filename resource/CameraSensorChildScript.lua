-- Please add this file to the child script > Threaded onto the CameraSensor on the Thymio, this will listen to the events created by the publisher that publishes on /camera/image_processed path 

function sysCall_init()
    corout=coroutine.create(coroutineMain)
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function sysCall_cleanup()
    if (simROS2) then
        -- Shutdown the ROS2 subscriber when the simulation ends
        simROS2.shutdownSubscriber(processed_subscriber)
    end
end

function coroutineMain()
    if (simROS2) then
        -- Prepare the subscriber for the processed image
        processed_subscriber = simROS2.createSubscription('/camera/image_processed', 'sensor_msgs/msg/Image', 'imageCallback')

        -- Get the handle of the camera sensor
        cameraSensorHandle = sim.getObjectHandle(sim.handle_self)
    else
        sim.addLog(sim.verbosity_errors, "ROS2 interface was not found. Please check your setup.")
    end
end

function imageCallback(msg)
    -- Decode the ROS2 image message to a format that can be used in CoppeliaSim
    local width = msg.width
    local height = msg.height
    local data = msg.data
    local img = {}

    -- Convert the image data from ROS2 message format to CoppeliaSim format
    for i = 1, #data, 3 do
        local r = data[i]
        local g = data[i + 1]
        local b = data[i + 2]
        table.insert(img, r / 255)
        table.insert(img, g / 255)
        table.insert(img, b / 255)
    end

    -- Set the camera sensor image with the processed data
    sim.setVisionSensorImage(cameraSensorHandle, img)
end
