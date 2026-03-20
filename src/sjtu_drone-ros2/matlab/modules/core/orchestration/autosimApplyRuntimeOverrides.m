function [cfg, info] = autosimApplyRuntimeOverrides(cfg)
    info = struct();
    info.applied = false;
    info.worker_id = cfg.runtime.worker_id;
    info.worker_count = cfg.runtime.worker_count;
    info.domain_id = cfg.runtime.domain_id;
    info.gazebo_port = cfg.runtime.gazebo_port;

    workerId = autosimEnvNumber('AUTOSIM_WORKER_ID', cfg.runtime.worker_id);
    workerCount = autosimEnvNumber('AUTOSIM_WORKER_COUNT', cfg.runtime.worker_count);
    domainId = autosimEnvNumber('AUTOSIM_DOMAIN_ID', cfg.runtime.domain_id);
    gazeboPort = autosimEnvNumber('AUTOSIM_GAZEBO_PORT', cfg.runtime.gazebo_port);
    gpuDevice = autosimEnvNumber('AUTOSIM_GPU_DEVICE', cfg.runtime.gpu_device);
    useGpu = autosimEnvBool('AUTOSIM_ENABLE_GPU', cfg.runtime.use_gpu);

    workerId = max(1, round(workerId));
    workerCount = max(1, round(workerCount));

    cfg.runtime.worker_id = workerId;
    cfg.runtime.worker_count = workerCount;
    cfg.runtime.domain_id = domainId;
    cfg.runtime.gazebo_port = gazeboPort;
    cfg.runtime.gpu_device = gpuDevice;
    cfg.runtime.use_gpu = useGpu;

    if workerCount > 1
        cfg.runtime.drone_namespace = sprintf('/drone_w%02d', workerId);
    else
        cfg.runtime.drone_namespace = '/drone';
    end

    if isfield(cfg, 'scenario') && isstruct(cfg.scenario)
        cfg.scenario.count = max(1, round(autosimEnvNumber('AUTOSIM_SCENARIO_COUNT', cfg.scenario.count)));
    end

    outputRootEnv = strtrim(string(getenv('AUTOSIM_OUTPUT_ROOT')));
    if strlength(outputRootEnv) > 0
        outputRoot = char(outputRootEnv);
        cfg.paths.data_root = fullfile(outputRoot, 'data');
        cfg.paths.log_root = fullfile(outputRoot, 'logs');
        cfg.paths.plot_root = fullfile(outputRoot, 'plots');
        cfg.paths.model_dir = fullfile(outputRoot, 'models');
    end

    workerTag = sprintf('worker_%02d', workerId);
    if workerCount > 1
        cfg.paths.data_root = fullfile(cfg.paths.data_root, workerTag);
        cfg.paths.log_root = fullfile(cfg.paths.log_root, workerTag);
        cfg.paths.plot_root = fullfile(cfg.paths.plot_root, workerTag);
        cfg.paths.model_dir = fullfile(cfg.paths.model_dir, workerTag);
        cfg.paths.run_id = sprintf('%s_w%02d', char(cfg.paths.run_id), workerId);
    end

    cfg.paths.data_dir = fullfile(cfg.paths.data_root, cfg.paths.run_id);
    cfg.paths.plot_dir = fullfile(cfg.paths.plot_root, cfg.paths.run_id);
    cfg.paths.log_dir = fullfile(cfg.paths.log_root, cfg.paths.run_id);
    cfg.paths.lock_file = fullfile(cfg.paths.data_root, sprintf('autosim_w%02d.lock', workerId));

    envItems = strings(0, 1);
    if isfinite(domainId)
        envItems(end+1, 1) = sprintf('export ROS_DOMAIN_ID=%d', round(domainId)); %#ok<AGROW>
    end
    if isfinite(gazeboPort)
        envItems(end+1, 1) = sprintf('export GAZEBO_MASTER_URI=http://127.0.0.1:%d', round(gazeboPort)); %#ok<AGROW>
    end
    if ~isempty(envItems)
        cfg.runtime.launch_env_prefix = strjoin(envItems, ' && ');
    else
        cfg.runtime.launch_env_prefix = "";
    end

    scopeEnv = strtrim(lower(string(getenv('AUTOSIM_CLEANUP_SCOPE'))));
    if scopeEnv == "instance" || scopeEnv == "global"
        cfg.process.cleanup_scope = scopeEnv;
    end

    if autosimEnvBool('AUTOSIM_ENABLE_PROGRESS_PLOT', cfg.visualization.enable_progress_plot)
        cfg.visualization.enable_progress_plot = true;
    else
        cfg.visualization.enable_progress_plot = false;
    end
    if autosimEnvBool('AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ', cfg.visualization.enable_scenario_live_view)
        cfg.visualization.enable_scenario_live_view = true;
    else
        cfg.visualization.enable_scenario_live_view = false;
    end

    droneNs = char(cfg.runtime.drone_namespace);
    cfg.topics.state = [droneNs '/state'];
    cfg.topics.pose = [droneNs '/gt_pose'];
    cfg.topics.vel = [droneNs '/gt_vel'];
    cfg.topics.imu = [droneNs '/imu'];
    cfg.topics.bumpers = [droneNs '/bumper_states'];
    cfg.topics.tag_state = [droneNs '/landing_tag_state'];
    cfg.topics.land_cmd = [droneNs '/land'];
    cfg.topics.takeoff_cmd = [droneNs '/takeoff'];
    cfg.topics.reset_cmd = [droneNs '/reset'];
    cfg.topics.cmd_vel = [droneNs '/cmd_vel'];

    ros2env = [ ...
        'cd /home/j/INCSL/IICC26_ws && ' ...
        'unset LD_LIBRARY_PATH && ' ...
        'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash' ...
    ];
    cfg.shell.land_cli_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        'ros2 topic pub ' cfg.topics.land_cmd ' std_msgs/msg/Empty {} --once' ...
    ];

    info.applied = true;
    info.worker_id = workerId;
    info.worker_count = workerCount;
    info.domain_id = domainId;
    info.gazebo_port = gazeboPort;
end

function n = autosimEnvNumber(name, defaultVal)
    txt = strtrim(getenv(name));
    if isempty(txt)
        n = defaultVal;
        return;
    end
    n = str2double(txt);
    if ~isfinite(n)
        n = defaultVal;
    end
end

function tf = autosimEnvBool(name, defaultVal)
    txt = strtrim(lower(getenv(name)));
    if isempty(txt)
        tf = logical(defaultVal);
        return;
    end
    tf = any(strcmp(txt, {'1', 'true', 'yes', 'y', 'on'}));
end
