function rosCtx = autosimCreateRosContext(cfg)
    rosCtx = struct();

    nodeName = sprintf('/matlab_autosim_%s', autosimTimestamp());
    node = ros2node(nodeName);

    rosCtx.node = node;
    rosCtx.subState = ros2subscriber(node, cfg.topics.state, 'std_msgs/Int8');
    rosCtx.subPose = ros2subscriber(node, cfg.topics.pose, 'geometry_msgs/Pose');
    rosCtx.subVel = ros2subscriber(node, cfg.topics.vel, 'geometry_msgs/Twist');
    rosCtx.tag_callback_enabled = false;
    rosCtx.tag_cache_key = "";
    tagCallbackRequested = isfield(cfg, 'ros') && isfield(cfg.ros, 'prioritize_tag_callback') && cfg.ros.prioritize_tag_callback;
    if tagCallbackRequested
        tagCacheKey = regexprep(sprintf('autosim_tag_cache_%s', nodeName), '[^a-zA-Z0-9_]', '_');
        autosimInitTagStateCache(tagCacheKey);
        try
            rosCtx.subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray', ...
                @(varargin) autosimTagStateCallback(tagCacheKey, varargin{:}));
            rosCtx.tag_callback_enabled = true;
            rosCtx.tag_cache_key = string(tagCacheKey);
            fprintf('[AUTOSIM] AprilTag callback-priority subscriber enabled on %s\n', cfg.topics.tag_state);
        catch ME
            autosimClearTagStateCache(tagCacheKey);
            rosCtx.subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray');
            warning('[AUTOSIM] AprilTag callback mode unavailable; fallback to polling: %s', ME.message);
        end
    else
        rosCtx.subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray');
    end
    rosCtx.subWind = ros2subscriber(node, cfg.topics.wind_condition, 'std_msgs/Float32MultiArray');

    rosCtx.subImu = [];
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_imu_subscription') && cfg.ros.enable_imu_subscription
        try
            rosCtx.subImu = ros2subscriber(node, cfg.topics.imu, 'sensor_msgs/msg/Imu');
        catch
            rosCtx.subImu = [];
        end
    end

    rosCtx.subBumpers = [];
    rosCtx.bumper_msg_type = "";
    rosCtx.bumper_msg_unsupported = false;
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_bumper_subscription') && cfg.ros.enable_bumper_subscription
        [rosCtx.subBumpers, rosCtx.bumper_msg_type, bumperDiag] = autosimCreateBumperSubscriber(node, cfg, false);
        if isstruct(bumperDiag) && isfield(bumperDiag, 'msg_unsupported')
            rosCtx.bumper_msg_unsupported = logical(bumperDiag.msg_unsupported);
        end
    end

    rosCtx.pubWind = ros2publisher(node, cfg.topics.wind_command, 'std_msgs/Float32MultiArray');
    rosCtx.pubTakeoff = ros2publisher(node, cfg.topics.takeoff_cmd, 'std_msgs/Empty');
    rosCtx.pubLand = ros2publisher(node, cfg.topics.land_cmd, 'std_msgs/Empty');
    rosCtx.pubReset = ros2publisher(node, cfg.topics.reset_cmd, 'std_msgs/Empty');
    rosCtx.pubCmd = ros2publisher(node, cfg.topics.cmd_vel, 'geometry_msgs/Twist');

    rosCtx.msgWind = ros2message(rosCtx.pubWind);
    rosCtx.msgTakeoff = ros2message(rosCtx.pubTakeoff);
    rosCtx.msgLand = ros2message(rosCtx.pubLand);
    rosCtx.msgReset = ros2message(rosCtx.pubReset);
    rosCtx.msgCmd = ros2message(rosCtx.pubCmd);
    rosCtx.cleanupHandles = {rosCtx.msgCmd, rosCtx.msgReset, rosCtx.msgLand, rosCtx.msgTakeoff, rosCtx.msgWind, ...
        rosCtx.pubCmd, rosCtx.pubReset, rosCtx.pubLand, rosCtx.pubTakeoff, rosCtx.pubWind, ...
        rosCtx.subBumpers, rosCtx.subImu, rosCtx.subWind, rosCtx.subTag, rosCtx.subVel, rosCtx.subPose, rosCtx.subState, rosCtx.node};
end


