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
    multiDroneCount = autosimEnvNumber('AUTOSIM_MULTI_DRONE_COUNT', cfg.runtime.multi_drone_count);
    multiDroneSpacing = autosimEnvNumber('AUTOSIM_MULTI_DRONE_SPACING_M', cfg.runtime.multi_drone_spacing_m);
    primaryDroneIndex = autosimEnvNumber('AUTOSIM_PRIMARY_DRONE_INDEX', cfg.runtime.primary_drone_index);
    multiDronePrefix = strtrim(string(getenv('AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX')));
    if strlength(multiDronePrefix) == 0
        multiDronePrefix = string(cfg.runtime.multi_drone_namespace_prefix);
    end
    spawnTags = autosimEnvBool('AUTOSIM_MULTI_DRONE_SPAWN_TAGS', cfg.runtime.multi_drone_spawn_tags);
    useWorldTagAsFirst = autosimEnvBool('AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST', cfg.runtime.multi_drone_use_world_tag_as_first);

    workerId = max(1, round(workerId));
    workerCount = max(1, round(workerCount));

    cfg.runtime.worker_id = workerId;
    cfg.runtime.worker_count = workerCount;
    cfg.runtime.domain_id = domainId;
    cfg.runtime.gazebo_port = gazeboPort;
    cfg.runtime.gpu_device = gpuDevice;
    cfg.runtime.use_gpu = useGpu;
    cfg.runtime.multi_drone_count = max(1, round(multiDroneCount));
    cfg.runtime.multi_drone_spacing_m = max(0.5, multiDroneSpacing);
    cfg.runtime.primary_drone_index = max(1, round(primaryDroneIndex));
    cfg.runtime.multi_drone_namespace_prefix = char(multiDronePrefix);
    cfg.runtime.multi_drone_spawn_tags = spawnTags;
    cfg.runtime.multi_drone_use_world_tag_as_first = useWorldTagAsFirst;

    if workerCount > 1
        cfg.runtime.drone_namespace = sprintf('/drone_w%02d', workerId);
    elseif cfg.runtime.multi_drone_count > 1
        prefix = regexprep(char(multiDronePrefix), '^/+', '');
        if isempty(prefix)
            prefix = 'drone_w';
        end
        idx = min(cfg.runtime.primary_drone_index, cfg.runtime.multi_drone_count);
        cfg.runtime.primary_drone_index = idx;
        cfg.runtime.drone_namespace = sprintf('/%s%02d', prefix, idx);
    else
        cfg.runtime.drone_namespace = '/drone';
    end

    if isfield(cfg, 'scenario') && isstruct(cfg.scenario)
        cfg.scenario.count = max(1, round(autosimEnvNumber('AUTOSIM_SCENARIO_COUNT', cfg.scenario.count)));
    end

    disableIncrementalTrain = autosimEnvBool('AUTOSIM_DISABLE_INCREMENTAL_TRAIN', false);
    if disableIncrementalTrain && isfield(cfg, 'learning') && isstruct(cfg.learning)
        cfg.learning.enable = false;
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

    schemaTag = autosimSchemaTag(cfg);
    if strlength(schemaTag) > 0
        cfg.paths.model_dir = fullfile(cfg.paths.model_dir, char(schemaTag));
    end

    cfg.paths.data_dir = fullfile(cfg.paths.data_root, cfg.paths.run_id);
    cfg.paths.plot_dir = fullfile(cfg.paths.plot_root, cfg.paths.run_id);
    cfg.paths.log_dir = fullfile(cfg.paths.log_root, cfg.paths.run_id);
    cfg.paths.lock_file = fullfile(cfg.paths.data_root, sprintf('autosim_w%02d.lock', workerId));

    % Keep persistence outputs aligned with runtime-adjusted data directory.
    if isfield(cfg, 'persistence') && isstruct(cfg.persistence)
        cfg.persistence.checkpoint_mat = fullfile(cfg.paths.data_dir, 'autosim_checkpoint_latest.mat');
        cfg.persistence.checkpoint_csv = fullfile(cfg.paths.data_dir, 'autosim_dataset_latest.csv');
        cfg.persistence.trace_csv = fullfile(cfg.paths.data_dir, 'autosim_trace_latest.csv');
    end

    % In multi-worker runs, keep Gazebo headless and disable RViz by default.
    defaultUseGui = true;
    defaultUseRviz = true;
    if workerCount > 1
        defaultUseGui = false;
        defaultUseRviz = false;
    end
    if ~isfield(cfg, 'launch') || ~isstruct(cfg.launch)
        cfg.launch = struct();
    end
    cfg.launch.use_gui = autosimEnvBool('AUTOSIM_USE_GUI', defaultUseGui);
    cfg.launch.use_rviz = autosimEnvBool('AUTOSIM_USE_RVIZ', defaultUseRviz);
    if workerCount > 1 && cfg.launch.use_rviz
        allowParallelRviz = autosimEnvBool('AUTOSIM_ALLOW_PARALLEL_RVIZ', false);
        if ~allowParallelRviz
            cfg.launch.use_rviz = false;
        end
    end
    defaultUseTeleop = false;
    if isfield(cfg.launch, 'use_teleop')
        defaultUseTeleop = logical(cfg.launch.use_teleop);
    end
    cfg.launch.use_teleop = autosimEnvBool('AUTOSIM_USE_TELEOP', defaultUseTeleop);

    envItems = strings(0, 1);
    if isfinite(domainId)
        envItems(end+1, 1) = sprintf('export ROS_DOMAIN_ID=%d', round(domainId)); %#ok<AGROW>
    end
    localhostOnly = autosimEnvBool('AUTOSIM_ROS_LOCALHOST_ONLY', false);
    if localhostOnly
        envItems(end+1, 1) = 'export ROS_LOCALHOST_ONLY=1'; %#ok<AGROW>
    else
        envItems(end+1, 1) = 'export ROS_LOCALHOST_ONLY=0'; %#ok<AGROW>
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
        ros2env ' && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        'ros2 topic pub ' cfg.topics.land_cmd ' std_msgs/msg/Empty {} --once' ...
    ];

    info.applied = true;
    info.worker_id = workerId;
    info.worker_count = workerCount;
    info.domain_id = domainId;
    info.gazebo_port = gazeboPort;
    info.disable_incremental_train = disableIncrementalTrain;
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

function tag = autosimSchemaTag(cfg)
    tag = "";
    if ~isfield(cfg, 'model') || ~isstruct(cfg.model)
        return;
    end
    if ~isfield(cfg.model, 'schema_version')
        return;
    end

    raw = lower(strtrim(string(cfg.model.schema_version)));
    if strlength(raw) == 0
        return;
    end
    cleaned = regexprep(raw, '[^a-z0-9_\-]', '_');
    cleaned = regexprep(cleaned, '_+', '_');
    cleaned = regexprep(cleaned, '^_+|_+$', '');
    if strlength(cleaned) == 0
        cleaned = "default";
    end
    tag = "schema_" + cleaned;
end
