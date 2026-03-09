% auto_landing_pipeline_matlab.m
% End-to-end MATLAB automation pipeline for Gazebo landing scenarios.
% It repeatedly runs ros2 launch, force-stops Gazebo/launch between scenarios,
% labels each scenario outcome from ROS topics with threshold rules,
% accumulates training data, trains a model, stores it with timestamp,
% and runs inference using the latest or a user-specified model file.
%
% Usage:
%   run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/auto_landing_pipeline_matlab.m')
%
% Notes:
% - Requires MATLAB ROS2 support (ros2node/ros2subscriber/ros2publisher).
% - Assumes ROS2 Humble workspace at /home/j/INCSL/IICC26_ws.

clear; clc;

thisDir = fileparts(mfilename('fullpath'));
if ~isempty(thisDir)
    addpath(thisDir);
end

cfg = defaultConfig();
ensureDirectories(cfg);
lockCleanup = acquirePipelineLock(cfg); %#ok<NASGU>

fprintf('\n[PIPELINE] Starting automation at %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('[PIPELINE] Scenario count: %d\n', cfg.scenario.count);

results = repmat(emptyScenarioResult(), 0, 1);
runStatus = "completed";
runError = [];

try
    for i = 1:cfg.scenario.count
        close all;
        fprintf('\n[PIPELINE] Scenario %d/%d\n', i, cfg.scenario.count);

        scenarioCfg = buildScenarioRuntimeConfig(cfg, i);

        cleanupSimulationProcesses(cfg);
        pause(cfg.process.kill_settle_sec);

        launchInfo = startBringupLaunch(cfg, i, scenarioCfg);

        try
            pause(cfg.launch.warmup_sec);
            scenarioResult = runSingleScenario(cfg, scenarioCfg, i);
            scenarioResult.launch_pid = launchInfo.pid;
            scenarioResult.launch_log = launchInfo.log_file;
        catch ME
            scenarioResult = emptyScenarioResult();
            scenarioResult.scenario_id = i;
            scenarioResult.label = "unstable";
            scenarioResult.success = false;
            if isUserInterruptException(ME)
                scenarioResult.failure_reason = "user_interrupt";
                runStatus = "interrupted";
            else
                scenarioResult.failure_reason = "runtime_exception";
            end
            scenarioResult.exception_message = string(ME.message);
            warning('[PIPELINE] Scenario %d exception: %s', i, ME.message);
        end

        results(end+1,1) = scenarioResult; %#ok<SAGROW>
        printRunningLabelStats(results, i, cfg.scenario.count);
        savePipelineCheckpoint(cfg, results, "scenario_end");

        if cfg.process.stop_after_each_scenario
            cleanupSimulationProcesses(cfg, launchInfo.pid);
            pause(cfg.process.kill_settle_sec);
        end

        if runStatus == "interrupted"
            fprintf('[PIPELINE] User interrupt detected. Stopping scenario loop after %d scenario(s).\n', numel(results));
            break;
        end
    end
catch ME
    if isUserInterruptException(ME)
        runStatus = "interrupted";
        warning('[PIPELINE] Interrupted by user: %s', ME.message);
    else
        runStatus = "failed";
        runError = ME;
        warning('[PIPELINE] Fatal pipeline exception: %s', ME.message);
    end
end

cleanupSimulationProcesses(cfg);
pause(cfg.process.kill_settle_sec);

finalizeInfo = finalizePipelineArtifacts(cfg, results, runStatus);

if finalizeInfo.inference_ran && ismember('label', finalizeInfo.summaryTbl.Properties.VariableNames)
    validMask = finalizeInfo.summaryTbl.label == "stable" | finalizeInfo.summaryTbl.label == "unstable";
    if any(validMask)
        gt = finalizeInfo.summaryTbl.label(validMask);
        pr = finalizeInfo.summaryTbl.pred_label(validMask);
        acc = mean(gt == pr);
        fprintf('[PIPELINE] Inference accuracy on collected dataset: %.3f\n', acc);
    end
end

fprintf('[PIPELINE] Completed at %s (status=%s)\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), runStatus);

if ~isempty(runError)
    rethrow(runError);
end


function cfg = defaultConfig()
    cfg = struct();

    ros2env = [ ...
        'cd /home/j/INCSL/IICC26_ws && ' ...
        'unset LD_LIBRARY_PATH && ' ...
        'unset ROS_DOMAIN_ID && ' ...
        'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/opt/ros/humble/lib && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash' ...
    ];

    cfg.paths = struct();
    cfg.paths.ws = '/home/j/INCSL/IICC26_ws';
    cfg.paths.data_dir = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/data';
    cfg.paths.model_dir = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/models';
    cfg.paths.log_dir = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/logs';
    cfg.paths.lock_file = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/data/auto_landing_pipeline.lock';
    cfg.paths.bringup_py_src = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/sjtu_drone_bringup';
    cfg.paths.bringup_py_install = '/home/j/INCSL/IICC26_ws/install/sjtu_drone_bringup/lib/python3.10/site-packages';

    pyPathChain = [ ...
        'export PYTHONPATH=' cfg.paths.bringup_py_install ':' cfg.paths.bringup_py_src ':$PYTHONPATH' ...
    ];

    cfg.launch = struct();
    cfg.launch.command_template = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        pyPathChain ' && ' ...
        'ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py ' ...
        'takeoff_hover_height:=%0.2f ' ...
        'takeoff_vertical_speed:=0.2 ' ...
        'use_gui:=false use_rviz:=true controller:=joystick ' ...
        'use_apriltag:=true apriltag_camera:=/drone/bottom ' ...
        'apriltag_image:=image_raw apriltag_tags:=tags apriltag_type:=umich ' ...
        'apriltag_bridge_topic:=/landing_tag_state ' ...
        'apriltag_use_standalone_detector:=true ' ...
        'apriltag_bridge_use_target_id:=true ' ...
        'apriltag_bridge_target_id:=0' ...
    ];
    cfg.launch.warmup_sec = 10.0;
    cfg.launch.ready_timeout_sec = 15.0;

    cfg.scenario = struct();
    cfg.scenario.count = 50;
    cfg.scenario.duration_sec = 20.0;
    cfg.scenario.post_land_observe_sec = 3.0;
    cfg.scenario.early_stop_after_landing_sec = 1.5;
    cfg.scenario.sample_period_sec = 0.2;
    cfg.scenario.hover_height_min_m = 1.5;
    cfg.scenario.hover_height_max_m = 2.5;

    % Wind randomization per scenario for data diversity.
    cfg.wind = struct();
    cfg.wind.enable = true;
    cfg.wind.update_period_sec = 0.5;
    cfg.wind.speed_min = 0.0;
    cfg.wind.speed_max = 8.0;
    cfg.wind.direction_min = -180.0;
    cfg.wind.direction_max = 180.0;
    cfg.wind.start_delay_after_hover_sec = 2.0;
    cfg.wind.start_require_tag_centered = true;
    cfg.wind.start_tag_center_hold_sec = 1.0;
    cfg.wind.source = "kma_csv"; % "kma_csv" | "random"
    cfg.wind.kma_csv = fullfile(cfg.paths.data_dir, 'kma_seoul_wind_hourly.csv');
    cfg.wind.kma_speed_column = 'wind_speed';
    cfg.wind.kma_direction_column = 'wind_dir';
    cfg.wind.kma_speed_scale = 1.0;
    cfg.wind.kma_direction_offset_deg = 0.0;

    cfg.process = struct();
    cfg.process.stop_after_each_scenario = true;
    cfg.process.kill_settle_sec = 2.0;
    cfg.process.cleanup_verify_timeout_sec = 8.0;

    cfg.persistence = struct();
    cfg.persistence.enable_checkpoint = true;
    cfg.persistence.checkpoint_mat = fullfile(cfg.paths.data_dir, 'landing_dataset_checkpoint_latest.mat');
    cfg.persistence.checkpoint_csv = fullfile(cfg.paths.data_dir, 'landing_dataset_checkpoint_latest.csv');

    cfg.topics = struct();
    cfg.topics.state = '/drone/state';
    cfg.topics.pose = '/drone/gt_pose';
    cfg.topics.vel = '/drone/gt_vel';
    cfg.topics.imu = '/drone/imu';
    cfg.topics.bumpers = '/drone/bumper_states';
    cfg.topics.tag_state = '/landing_tag_state';
    cfg.topics.wind_condition = '/wind_condition';
    cfg.topics.wind_command = '/wind_command';
    cfg.topics.land_cmd = '/drone/land';
    cfg.topics.takeoff_cmd = '/drone/takeoff';
    cfg.topics.cmd_vel = '/drone/cmd_vel';
    cfg.topics.posctrl_cmd = '/drone/posctrl';
    cfg.topics.dronevel_mode_cmd = '/drone/dronevel_mode';

    cfg.ros = struct();
    cfg.ros.receive_timeout_sec = 0.01;
    cfg.ros.enable_bumper_subscription = true;
    cfg.ros.enable_imu_subscription = true;

    cfg.control = struct();
    cfg.control.enable = true;
    cfg.control.takeoff_retry_sec = 1.0;
    cfg.control.takeoff_force_cli_timeout_sec = 4.0;
    cfg.control.takeoff_force_cli_retry_sec = 3.0;
    cfg.control.hover_settle_sec = 2.0;
    cfg.control.flying_altitude_threshold = 0.25;
    cfg.control.target_u = 0.0;
    cfg.control.target_v = -0.08;
    cfg.control.xy_kp = 1.15;
    cfg.control.xy_ki = 0.001;
    cfg.control.xy_kd = 0.08;
    cfg.control.xy_i_limit = 1.0;
    cfg.control.xy_cmd_limit = 0.7;
    cfg.control.xy_map_sign_x_from_v = 1.0;
    cfg.control.xy_map_sign_y_from_u = 1.0;
    cfg.control.xy_control_center_deadband = 0.005;
    cfg.control.tag_predict_horizon_sec = 0.15;
    cfg.control.tag_predict_timeout_sec = 0.6;
    cfg.control.tag_history_len = 20;
    cfg.control.tag_min_predict_samples = 2;
    cfg.control.tag_hold_last_state = true;
    cfg.control.tag_hold_timeout_sec = 0.6;
    cfg.control.use_position_control_mode = true;
    cfg.control.posctrl_enable_only_when_flying = true;
    cfg.control.dronevel_mode_on = false;
    cfg.control.poscmd_xy_rate_scale = 1.0;
    cfg.control.poscmd_xy_rate_scale_landing = 0.7;
    cfg.control.search_min_altitude_margin_m = 0.20;
    cfg.control.landing_descent_rate_mps = 0.07;
    cfg.control.landing_cmd_alt_min_m = 0.10;
    cfg.control.landing_land_trigger_alt_m = 0.22;
    cfg.control.pre_takeoff_require_tag_centered = false;
    cfg.control.pre_takeoff_tag_center_tolerance = 0.005;
    cfg.control.pre_takeoff_tag_center_hold_sec = 1.0;
    cfg.control.enable_landing_vz_damping = true;
    cfg.control.landing_vz_target_mps = -0.02;
    cfg.control.landing_vz_kp = 1.75;
    cfg.control.landing_up_cmd_limit = 0.9;
    cfg.control.landing_vz_eval_window_sec = 1.0;

    cfg.learning = struct();
    cfg.learning.enable = true;
    cfg.learning.tag_lock_error_max = 0.12;
    cfg.learning.tag_lock_hold_sec = 1.2;
    cfg.learning.random_landing_wait_min_sec = 1.0;
    cfg.learning.random_landing_wait_max_sec = 4.0;
    cfg.learning.random_cmd_duration_min_sec = 1.0;
    cfg.learning.random_cmd_duration_max_sec = 3.0;
    cfg.learning.random_xy_cmd_max = 0.35;
    cfg.search = struct();
    cfg.search.enable_spiral_when_tag_lost = true;
    cfg.search.spiral_cmd_max = 0.30;
    cfg.search.spiral_growth_per_sec = 0.08;
    cfg.search.spiral_omega_rad_sec = 1.2;
    cfg.search.spiral_start_radius = 0.04;

    cfg.visualization = struct();
    % Low-memory default for long batch runs.
    cfg.visualization.enable = false;
    cfg.visualization.window_sec = 45.0;
    cfg.visualization.max_points = 500;

    cfg.logging = struct();
    % Disable per-scenario launch log files by default.
    cfg.logging.enable_launch_log_file = false;
    cfg.logging.heartbeat_sec = 2.0;

    cfg.shell = struct();
    cfg.shell.ros2env = ros2env;
    cfg.shell.setup_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        pyPathChain ...
    ];
    cfg.shell.takeoff_cli_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        pyPathChain ' && ' ...
        'timeout 2s ros2 topic pub /drone/takeoff std_msgs/msg/Empty "{}" --once --wait-matching-subscriptions 0' ...
    ];
    cfg.shell.land_cli_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        pyPathChain ' && ' ...
        'ros2 topic pub /drone/land std_msgs/msg/Empty {} --once' ...
    ];

    cfg.thresholds = struct();
    cfg.thresholds.land_state_value = 0;
    cfg.thresholds.landed_altitude_max_m = 0.30;
    cfg.thresholds.final_speed_max_mps = 0.40;
    cfg.thresholds.final_attitude_max_deg = 15.0;
    cfg.thresholds.final_tag_error_max = 0.35;
    cfg.thresholds.final_stability_std_z_max = 0.18;
    cfg.thresholds.final_stability_std_vz_max = 0.22;
    cfg.thresholds.final_stability_std_vz_osc_max = 0.18;
    cfg.thresholds.final_touchdown_accel_rms_max = 1.20;
    cfg.thresholds.bumpers_contact_max = 0;
    cfg.thresholds.final_imu_ang_vel_rms_max = 2.8;
    cfg.thresholds.final_imu_lin_acc_rms_max = 6.0;
    cfg.thresholds.final_contact_force_max_n = 25.0;
    cfg.thresholds.final_arm_force_imbalance_max_n = 18.0;

    cfg.model = struct();
    cfg.model.feature_names = [ ...
        "mean_wind_speed", "max_wind_speed", "mean_abs_roll_deg", "mean_abs_pitch_deg", ...
        "mean_abs_vz", "max_abs_vz", "mean_tag_error", "max_tag_error", ...
        "final_altitude", "final_abs_speed", "final_abs_roll_deg", "final_abs_pitch_deg", ...
        "final_tag_error", "stability_std_z", "stability_std_vz", "stability_std_vz_osc", ...
        "touchdown_accel_rms", "contact_count", ...
        "mean_imu_ang_vel", "max_imu_ang_vel", "mean_imu_lin_acc", "max_imu_lin_acc", ...
        "max_contact_force", "arm_force_imbalance" ...
    ];

    % Inference model selection:
    % - empty model_file => latest model in model_dir
    % - set full path to use specific model
    cfg.inference = struct();
    cfg.inference.model_file = '';

    % Optional runtime override without file edit:
    %   export LANDING_MODEL_FILE=/abs/path/to/landing_model_*.mat
    modelFileEnv = strtrim(getenv('LANDING_MODEL_FILE'));
    if ~isempty(modelFileEnv)
        cfg.inference.model_file = modelFileEnv;
    end
end


function ensureDirectories(cfg)
    dirs = {cfg.paths.data_dir, cfg.paths.model_dir};
    if isfield(cfg, 'logging') && isfield(cfg.logging, 'enable_launch_log_file') && cfg.logging.enable_launch_log_file
        dirs{end+1} = cfg.paths.log_dir; %#ok<AGROW>
    end
    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir')
            mkdir(dirs{i});
        end
    end
end


function lockCleanup = acquirePipelineLock(cfg)
    lockPath = cfg.paths.lock_file;

    if isfile(lockPath)
        try
            txt = strtrim(fileread(lockPath));
            oldPid = str2double(txt);
        catch
            oldPid = nan;
        end

        if isfinite(oldPid) && oldPid > 1
            [st, ~] = system(sprintf('bash -i -c "kill -0 %d >/dev/null 2>&1"', round(oldPid)));
            if st == 0
                error('Another auto_landing_pipeline_matlab instance is running (pid=%d). Stop it before starting a new one.', round(oldPid));
            end
        end
    end

    thisPid = feature('getpid');
    fid = fopen(lockPath, 'w');
    if fid < 0
        error('Failed to create pipeline lock file: %s', lockPath);
    end
    fprintf(fid, '%d\n', round(thisPid));
    fclose(fid);

    lockCleanup = onCleanup(@() releasePipelineLock(lockPath));
end


function releasePipelineLock(lockPath)
    try
        if isfile(lockPath)
            delete(lockPath);
        end
    catch
    end
end


function info = startBringupLaunch(cfg, scenarioId, scenarioCfg)
    [~, preOut] = system('bash -i -c "pgrep -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" | wc -l"');
    preCount = str2double(strtrim(preOut));
    preSnap = getActiveRosProcessSnapshot();
    hasStaleGraph = strlength(strtrim(preSnap)) > 0;

    if (isfinite(preCount) && preCount > 0) || hasStaleGraph
        fprintf('[PIPELINE] Detected stale ROS/Gazebo process(es) before start, forcing cleanup.\n');
        if hasStaleGraph
            fprintf('[PIPELINE] Stale pre-launch snapshot:\n%s\n', preSnap);
        end
        cleanupSimulationProcesses(cfg);
        pause(1.0);

        postCleanupSnap = getActiveRosProcessSnapshot();
        if strlength(strtrim(postCleanupSnap)) > 0
            error('Cleanup did not converge before launch start. Refusing to start new launch.\nRemaining:\n%s', postCleanupSnap);
        end
    end

    logFile = fullfile(cfg.paths.log_dir, sprintf('bringup_s%03d_%s.log', scenarioId, timestampForFile()));

    if nargin < 3 || ~isfield(scenarioCfg, 'hover_height_m') || ~isfinite(scenarioCfg.hover_height_m)
        hoverHeight = 2.5;
    else
        hoverHeight = scenarioCfg.hover_height_m;
    end
    launchCmd = sprintf(cfg.launch.command_template, hoverHeight);
    escapedCmd = shellEscapeDoubleQuotes(launchCmd);

    enableLaunchLog = false;
    if isfield(cfg, 'logging') && isfield(cfg.logging, 'enable_launch_log_file')
        enableLaunchLog = logical(cfg.logging.enable_launch_log_file);
    end

    % Keep launch invocation style consistent with landing_decision_matlab:
    % send a single background shell command via MATLAB system().
    if enableLaunchLog
        escapedLog = shellEscapeDoubleQuotes(logFile);
        bashCmd = sprintf('bash -i -c "%s > \\\"%s\\\" 2>&1 &"', escapedCmd, escapedLog);
    else
        bashCmd = sprintf('bash -i -c "%s >/dev/null 2>&1 &"', escapedCmd);
    end
    fprintf('[PIPELINE] Launch command: %s\n', launchCmd);
    [st, out] = system(bashCmd);
    if st ~= 0
        error('Failed to start launch: %s', out);
    end

    pid = -1;
    [~, pOut] = system('bash -i -c "pgrep -n -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" || true"');
    pTok = regexp(pOut, '(\d+)', 'tokens');
    if ~isempty(pTok)
        pid = str2double(pTok{end}{1});
    end

    if enableLaunchLog
        info = struct('pid', pid, 'log_file', string(logFile));
        fprintf('[PIPELINE] Launch started (pid=%d), log=%s\n', pid, logFile);
    else
        info = struct('pid', pid, 'log_file', "");
        fprintf('[PIPELINE] Launch started (pid=%d), file logging disabled\n', pid);
    end
end


function cleanupSimulationProcesses(cfg, launchPid)
    if nargin < 2
        launchPid = -1;
    end

    if isfinite(launchPid) && launchPid > 1
        killProcessTree(round(launchPid));
    end

    out = '';
    preSnap = getActiveRosProcessSnapshot();
    if strlength(strtrim(preSnap)) > 0
        fprintf('[PIPELINE] Cleanup pre-snapshot:\n%s\n', preSnap);
    end

    for pass = 1:3
        [~, outPass] = system(['bash -i -c "set +m; ' ...
            'pgrep -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" | xargs -r kill -9 || true; ' ...
            'pkill -9 -f \"[r]os2 launch sjtu_drone_bringup\" || true; ' ...
            'pkill -9 -f \"[s]jtu_drone_bringup.launch.py\" || true; ' ...
            'pkill -9 -f \"[c]omponent_container\" || true; ' ...
            'pkill -9 -f \"[a]priltag_detector\" || true; ' ...
            'pkill -9 -f \"[a]priltag_state_bridge\" || true; ' ...
            'pkill -9 -f \"[s]tatic_transform_publisher\" || true; ' ...
            'pkill -9 -f \"[r]obot_state_publisher\" || true; ' ...
            'pkill -9 -f \"[j]oint_state_publisher\" || true; ' ...
            'pkill -9 -f \"[s]pawn_drone\" || true; ' ...
            'pkill -9 -f \"[t]eleop_joystick\" || true; ' ...
            'pkill -9 -f \"[j]oy_node\" || true; ' ...
            'pkill -9 -f \"[g]azebo_wind_plugin_node\" || true; ' ...
            'pkill -9 gzserver || true; ' ...
            'pkill -9 gzclient || true; ' ...
            'pkill -9 -x rviz2 || true; ' ...
            'pkill -9 -f \"[r]viz2\" || true" 2>/dev/null']);

        if ~isempty(strtrim(outPass))
            out = sprintf('%s\n[pass %d]\n%s', out, pass, outPass); %#ok<AGROW>
        end

        pause(0.3);
    end

    % If pattern-based kill missed detached children, kill by discovered PID set.
    killActiveRosProcessTrees();

    refreshRos2Daemon(cfg);

    verifyTimeout = 8.0;
    if isfield(cfg, 'process') && isfield(cfg.process, 'cleanup_verify_timeout_sec') && isfinite(cfg.process.cleanup_verify_timeout_sec)
        verifyTimeout = max(1.0, cfg.process.cleanup_verify_timeout_sec);
    end
    waitForRosProcessCleanup(verifyTimeout);

    postSnap = getActiveRosProcessSnapshot();
    if strlength(strtrim(postSnap)) > 0
        fprintf('[PIPELINE] Cleanup post-snapshot (still alive):\n%s\n', postSnap);
    end

    if ~isempty(strtrim(out))
        fprintf('[PIPELINE] Cleanup output:\n%s\n', out);
    else
        fprintf('[PIPELINE] Cleanup complete (multi-pass kill + daemon refresh).\n');
    end
end


function waitForRosProcessCleanup(timeoutSec)
    t0 = tic;
    while toc(t0) <= timeoutSec
        snap = getActiveRosProcessSnapshot();
        if strlength(strtrim(snap)) == 0
            return;
        end

        killActiveRosProcessTrees();
        pause(0.25);
    end
end


function killActiveRosProcessTrees()
    pids = getActiveRosProcessPids();
    if isempty(pids)
        return;
    end

    for i = 1:numel(pids)
        killProcessTree(pids(i));
    end
end


function pids = getActiveRosProcessPids()
    cmd = ['bash -i -c "' ...
        'pgrep -f \"[r]os2 launch sjtu_drone_bringup|[c]omponent_container|[a]priltag|[j]oint_state_publisher|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]viz2|[j]oy_node|[g]azebo|[g]zserver|[g]zclient|[s]pawn_drone|[g]azebo_wind_plugin_node\" || true"'];
    [~, txt] = system(cmd);
    toks = regexp(txt, '(\d+)', 'tokens');
    if isempty(toks)
        pids = [];
        return;
    end

    pids = zeros(numel(toks), 1);
    for i = 1:numel(toks)
        pids(i) = str2double(toks{i}{1});
    end
    pids = unique(pids(isfinite(pids) & pids > 1));
end


function killProcessTree(pid)
    if ~isfinite(pid) || pid <= 1
        return;
    end

    cmd = sprintf(['bash -i -c "' ...
        'pkill -9 -P %d >/dev/null 2>&1 || true; ' ...
        'pkill -9 -P %d >/dev/null 2>&1 || true; ' ...
        'pkill -9 -P %d >/dev/null 2>&1 || true; ' ...
        'kill -9 %d >/dev/null 2>&1 || true"'], round(pid), round(pid), round(pid), round(pid));
    system(cmd);
end


function refreshRos2Daemon(cfg)
    daemonOk = false;

    if isfield(cfg, 'shell') && isfield(cfg.shell, 'setup_cmd')
        daemonCmd = sprintf('bash -i -c "%s && ros2 daemon stop >/dev/null 2>&1 || true; ros2 daemon start >/dev/null 2>&1 || true"', ...
            shellEscapeDoubleQuotes(cfg.shell.setup_cmd));
        st = system(daemonCmd);
        daemonOk = (st == 0);
    end

    if ~daemonOk
        % Fallback path for cases where workspace setup command fails.
        system('bash -i -c "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; ros2 daemon stop >/dev/null 2>&1 || true; ros2 daemon start >/dev/null 2>&1 || true"');
    end
end


function out = getActiveRosProcessSnapshot()
    cmd = ['bash -i -c "' ...
        'pgrep -af \"[r]os2 launch sjtu_drone_bringup|[c]omponent_container|[a]priltag|[j]oint_state_publisher|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]viz2|[j]oy_node|[g]azebo|[g]zserver|[g]zclient|[s]pawn_drone|[g]azebo_wind_plugin_node\" ' ...
        '| sed -n \"1,120p\" || true"'];
    [~, txt] = system(cmd);
    out = string(txt);
end


function scenarioCfg = buildScenarioRuntimeConfig(cfg, scenarioId)
    scenarioCfg = struct();
    scenarioCfg.id = scenarioId;
    scenarioCfg.hover_height_m = randRange(cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);

    if cfg.wind.enable
        [scenarioCfg.wind_speed, scenarioCfg.wind_dir] = pickScenarioWind(cfg, scenarioId);
    else
        scenarioCfg.wind_speed = 0.0;
        scenarioCfg.wind_dir = 0.0;
    end
end


function result = runSingleScenario(cfg, scenarioCfg, scenarioId)
    nodeName = sprintf('/matlab_auto_landing_s%03d', scenarioId);
    node = ros2node(nodeName);
    nodeCleanup = onCleanup(@() clearNode(node)); %#ok<NASGU>

    subState = ros2subscriber(node, cfg.topics.state, 'std_msgs/Int8');
    subPose = ros2subscriber(node, cfg.topics.pose, 'geometry_msgs/Pose');
    subVel = ros2subscriber(node, cfg.topics.vel, 'geometry_msgs/Twist');
    subImu = [];
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_imu_subscription') && cfg.ros.enable_imu_subscription
        try
            subImu = ros2subscriber(node, cfg.topics.imu, 'sensor_msgs/msg/Imu');
        catch ME
            fprintf('[PIPELINE] IMU subscriber unavailable (disabled for this scenario): %s\n', ME.message);
            subImu = [];
        end
    end
    subBumpers = [];
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_bumper_subscription') && cfg.ros.enable_bumper_subscription
        try
            subBumpers = ros2subscriber(node, cfg.topics.bumpers, 'gazebo_msgs/msg/ContactsState');
        catch ME
            fprintf('[PIPELINE] Bumper subscriber unavailable (disabled for this scenario): %s\n', ME.message);
            subBumpers = [];
        end
    end
    subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray');
    subWind = ros2subscriber(node, cfg.topics.wind_condition, 'std_msgs/Float32MultiArray');

    pubWind = ros2publisher(node, cfg.topics.wind_command, 'std_msgs/Float32MultiArray');
    pubLand = ros2publisher(node, cfg.topics.land_cmd, 'std_msgs/Empty');
    pubTakeoff = ros2publisher(node, cfg.topics.takeoff_cmd, 'std_msgs/Empty');
    pubCmdVel = ros2publisher(node, cfg.topics.cmd_vel, 'geometry_msgs/Twist');
    pubPosCtrl = ros2publisher(node, cfg.topics.posctrl_cmd, 'std_msgs/Bool');
    pubVelMode = ros2publisher(node, cfg.topics.dronevel_mode_cmd, 'std_msgs/Bool');

    msgWind = ros2message(pubWind);
    msgLand = ros2message(pubLand);
    msgTakeoff = ros2message(pubTakeoff);
    msgCmdVel = ros2message(pubCmdVel);
    msgPosCtrl = ros2message(pubPosCtrl);
    msgVelMode = ros2message(pubVelMode);

    sampleN = max(1, floor(cfg.scenario.duration_sec / cfg.scenario.sample_period_sec));
    heartbeatSec = 2.0;
    if isfield(cfg, 'logging') && isfield(cfg.logging, 'heartbeat_sec') && isfinite(cfg.logging.heartbeat_sec)
        heartbeatSec = max(0.2, cfg.logging.heartbeat_sec);
    end
    heartbeatEvery = max(1, round(heartbeatSec / cfg.scenario.sample_period_sec));

    t = zeros(sampleN, 1);
    z = nan(sampleN, 1);
    vz = nan(sampleN, 1);
    speedAbs = nan(sampleN, 1);
    rollDeg = nan(sampleN, 1);
    pitchDeg = nan(sampleN, 1);
    tagErr = nan(sampleN, 1);
    windSpeed = nan(sampleN, 1);
    stateVal = nan(sampleN, 1);
    bumperContact = zeros(sampleN, 1);
    imuAngVel = nan(sampleN, 1);
    imuLinAcc = nan(sampleN, 1);
    contactForce = nan(sampleN, 1);
    armForceFL = nan(sampleN, 1);
    armForceFR = nan(sampleN, 1);
    armForceRL = nan(sampleN, 1);
    armForceRR = nan(sampleN, 1);

    fprintf('[PIPELINE] Scenario %d profile: hover=%.2fm, wind=%.2f m/s, dir=%.1f deg\n', ...
        scenarioId, scenarioCfg.hover_height_m, scenarioCfg.wind_speed, scenarioCfg.wind_dir);

    t0 = tic;
    lastWindCmdT = -inf;
    lastTakeoffCmdT = -inf;
    lastTakeoffCliCmdT = -inf;
    takeoffPublishCount = 0;
    lastCtrlT = 0.0;
    if cfg.control.pre_takeoff_require_tag_centered
        controlPhase = "pre_takeoff_stabilize";
    else
        controlPhase = "takeoff";
    end
    phaseEnterT = 0.0;
    preTakeoffCenterHoldStartT = nan;
    pidX = initPidState();
    pidY = initPidState();

    tagHist = nan(cfg.control.tag_history_len, 2);
    tagHistCount = 0;
    lastTagDetectT = -inf;
    lastTagU = nan;
    lastTagV = nan;
    haveLastTag = false;

    viz = [];
    if cfg.visualization.enable
        viz = initScenarioLivePlots(cfg.visualization.window_sec, cfg.visualization.max_points);
    end

    [readyOk, readyDiag] = waitForScenarioTopicsReady(cfg, cfg.launch.ready_timeout_sec);
    if ~readyOk
        result = emptyScenarioResult();
        result.scenario_id = scenarioId;
        result.label = "unstable";
        result.success = false;
        result.failure_reason = "scenario_topics_not_ready";
        result.exception_message = string(readyDiag);
        fprintf('[PIPELINE] Scenario %d skipped: %s\n', scenarioId, readyDiag);
        return;
    end

    tagLockHoldStartT = nan;
    tagLockAcquired = false;
    landingSent = false;
    landingSentT = nan;
    windArmed = false;
    hoverStartT = nan;
    hoverCenterHoldStartT = nan;
    randomLandingPlanned = false;
    randomLandingStartT = nan;
    randomLandingEndT = nan;
    randomBiasX = 0.0;
    randomBiasY = 0.0;
    landingDescentActive = false;
    targetPoseX = nan;
    targetPoseY = nan;
    targetPoseZ = nan;
    posCtrlConfigured = false;
    tagLostSearchStartT = nan;
    landedHoldStartT = nan;

    for k = 1:sampleN
        tk = toc(t0);
        t(k) = tk;

        if cfg.wind.enable && (tk - lastWindCmdT) >= cfg.wind.update_period_sec
            if windArmed
                msgWind.data = single([scenarioCfg.wind_speed, scenarioCfg.wind_dir]);
            else
                msgWind.data = single([0.0, 0.0]);
            end
            send(pubWind, msgWind);
            lastWindCmdT = tk;
        end

        recvTimeout = 0.01;
        if isfield(cfg, 'ros') && isfield(cfg.ros, 'receive_timeout_sec') && isfinite(cfg.ros.receive_timeout_sec)
            recvTimeout = max(0.0, cfg.ros.receive_timeout_sec);
        end

        poseMsg = tryReceive(subPose, recvTimeout);
        poseXNow = nan;
        poseYNow = nan;
        if ~isempty(poseMsg)
            poseXNow = double(poseMsg.position.x);
            poseYNow = double(poseMsg.position.y);
            z(k) = double(poseMsg.position.z);
            q = poseMsg.orientation;
            [r, p, ~] = quat2eul_local([q.w, q.x, q.y, q.z]);
            rollDeg(k) = abs(rad2deg(r));
            pitchDeg(k) = abs(rad2deg(p));
        end

        velMsg = tryReceive(subVel, recvTimeout);
        if ~isempty(velMsg)
            vx = double(velMsg.linear.x);
            vy = double(velMsg.linear.y);
            vz(k) = double(velMsg.linear.z);
            speedAbs(k) = sqrt(vx*vx + vy*vy + vz(k)*vz(k));
        end

        stateMsg = tryReceive(subState, recvTimeout);
        if ~isempty(stateMsg)
            stateVal(k) = double(stateMsg.data);
        end

        if ~isempty(subImu)
            imuMsg = tryReceive(subImu, recvTimeout);
            if ~isempty(imuMsg)
                [imuAngVel(k), imuLinAcc(k)] = parseImuMetrics(imuMsg);
            end
        end

        bumpMsg = [];
        if ~isempty(subBumpers)
            bumpMsg = tryReceive(subBumpers, recvTimeout);
        end
        if ~isempty(bumpMsg)
            [bumperContact(k), contactForce(k), armForceFL(k), armForceFR(k), armForceRL(k), armForceRR(k)] = parseContactForces(bumpMsg);
        end

        tagMsg = tryReceive(subTag, recvTimeout);
        tagDetected = false;
        uTag = nan;
        vTag = nan;
        if ~isempty(tagMsg)
            tagErr(k) = parseTagErrorFromBridge(tagMsg);
            [tagDetected, uTag, vTag, tagErrNow] = parseTagStateFromBridge(tagMsg);
            if isfinite(tagErrNow)
                tagErr(k) = tagErrNow;
            end
        end

        if tagDetected && isfinite(uTag) && isfinite(vTag)
            [tagHist, tagHistCount] = pushTagCenter(tagHist, tagHistCount, uTag, vTag);
            lastTagU = uTag;
            lastTagV = vTag;
            lastTagDetectT = tk;
            haveLastTag = true;
        elseif cfg.control.tag_hold_last_state && haveLastTag && ((tk - lastTagDetectT) <= cfg.control.tag_hold_timeout_sec)
            tagDetected = true;
            uTag = lastTagU;
            vTag = lastTagV;
            tagErr(k) = sqrt(uTag*uTag + vTag*vTag);
        end

        [predOk, uPred, vPred] = predictTagCenterNorm(tagHist, tagHistCount, uTag, vTag, tk, lastTagDetectT, ...
            cfg.control.tag_predict_horizon_sec, cfg.control.tag_predict_timeout_sec, cfg.scenario.sample_period_sec, cfg.control.tag_min_predict_samples);

        windMsg = tryReceive(subWind, recvTimeout);
        if ~isempty(windMsg)
            ws = parseWindSpeed(windMsg);
            windSpeed(k) = ws;
        end

        % Control state machine: takeoff -> hover_settle -> xy_hold
        stateNow = stateVal(k);
        if isfinite(stateNow)
            isFlying = (stateNow == 1);
        else
            isFlying = isfinite(z(k)) && (z(k) >= cfg.control.flying_altitude_threshold);
        end

        dtCtrl = max(1e-3, tk - lastCtrlT);
        lastCtrlT = tk;
        searchMinAlt = scenarioCfg.hover_height_m + cfg.control.search_min_altitude_margin_m;
        cmdX = 0.0;
        cmdY = 0.0;
        cmdZ = 0.0;
        tagLockReadyNow = false;

        if predOk && isfinite(uPred) && isfinite(vPred)
            tagLockErr = sqrt((uPred - cfg.control.target_u)^2 + (vPred - cfg.control.target_v)^2);
            tagLockReadyNow = tagLockErr <= cfg.learning.tag_lock_error_max;
        elseif tagDetected && isfinite(uTag) && isfinite(vTag)
            tagLockErr = sqrt((uTag - cfg.control.target_u)^2 + (vTag - cfg.control.target_v)^2);
            tagLockReadyNow = tagLockErr <= cfg.learning.tag_lock_error_max;
        end

        if cfg.control.enable
            posctrlEnableCond = true;
            if cfg.control.posctrl_enable_only_when_flying
                posctrlEnableCond = isFlying;
            end

            if cfg.control.use_position_control_mode && ~posCtrlConfigured && posctrlEnableCond
                msgPosCtrl.data = true;
                send(pubPosCtrl, msgPosCtrl);
                msgVelMode.data = logical(cfg.control.dronevel_mode_on);
                send(pubVelMode, msgVelMode);
                posCtrlConfigured = true;

                if isfinite(poseXNow)
                    targetPoseX = poseXNow;
                end
                if isfinite(poseYNow)
                    targetPoseY = poseYNow;
                end
                if isfinite(z(k))
                    targetPoseZ = max(z(k), searchMinAlt);
                else
                    targetPoseZ = searchMinAlt;
                end
            end

            if landingSent
                controlPhase = "landing_observe";
                cmdX = 0.0;
                cmdY = 0.0;
            else
                switch char(controlPhase)
                    case 'pre_takeoff_stabilize'
                        if cfg.control.pre_takeoff_require_tag_centered
                            centerReadyNow = predOk && isfinite(uPred) && isfinite(vPred) && ...
                                (sqrt((uPred - cfg.control.target_u)^2 + (vPred - cfg.control.target_v)^2) <= cfg.control.pre_takeoff_tag_center_tolerance);
                            if centerReadyNow
                                if ~isfinite(preTakeoffCenterHoldStartT)
                                    preTakeoffCenterHoldStartT = tk;
                                end
                            else
                                preTakeoffCenterHoldStartT = nan;
                            end

                            centerHoldReady = isfinite(preTakeoffCenterHoldStartT) && ...
                                ((tk - preTakeoffCenterHoldStartT) >= cfg.control.pre_takeoff_tag_center_hold_sec);
                        else
                            centerHoldReady = true;
                        end

                        if centerHoldReady
                            controlPhase = "takeoff";
                            phaseEnterT = tk;
                        end

                    case 'takeoff'
                        if (tk - lastTakeoffCmdT) >= cfg.control.takeoff_retry_sec
                            send(pubTakeoff, msgTakeoff);
                            lastTakeoffCmdT = tk;
                            takeoffPublishCount = takeoffPublishCount + 1;
                            fprintf('[PIPELINE] s%03d takeoff publish #%d at t=%.1fs\n', scenarioId, takeoffPublishCount, tk);
                        end

                        if ~isFlying && ((tk - phaseEnterT) >= cfg.control.takeoff_force_cli_timeout_sec) && ...
                                ((tk - lastTakeoffCliCmdT) >= cfg.control.takeoff_force_cli_retry_sec)
                            cliCmd = sprintf('bash -i -c "%s"', shellEscapeDoubleQuotes(cfg.shell.takeoff_cli_cmd));
                            [cliSt, cliOut] = system(cliCmd);
                            lastTakeoffCliCmdT = tk;
                            if cliSt == 0 || cliSt == 124
                                fprintf('[PIPELINE] s%03d takeoff CLI fallback sent at t=%.1fs\n', scenarioId, tk);
                            else
                                fprintf('[PIPELINE] s%03d takeoff CLI fallback failed at t=%.1fs (code=%d): %s\n', scenarioId, tk, cliSt, strtrim(cliOut));
                            end
                        end

                        if isFlying
                            controlPhase = "hover_settle";
                            phaseEnterT = tk;
                            hoverStartT = tk;
                            hoverCenterHoldStartT = nan;
                            pidX = initPidState();
                            pidY = initPidState();
                        end

                    case 'hover_settle'
                        if ~isFlying
                            controlPhase = "takeoff";
                            phaseEnterT = tk;
                        elseif (tk - phaseEnterT) >= cfg.control.hover_settle_sec
                            controlPhase = "xy_hold";
                            phaseEnterT = tk;
                        end

                    case 'xy_hold'
                        if ~isFlying
                            controlPhase = "takeoff";
                            phaseEnterT = tk;
                            hoverStartT = nan;
                            hoverCenterHoldStartT = nan;
                            pidX = initPidState();
                            pidY = initPidState();
                            tagLostSearchStartT = nan;
                        else
                            usePred = predOk && isfinite(uPred) && isfinite(vPred);
                            useNow = tagDetected && isfinite(uTag) && isfinite(vTag);

                            if usePred
                                uCtrl = uPred;
                                vCtrl = vPred;
                            elseif useNow
                                uCtrl = uTag;
                                vCtrl = vTag;
                            else
                                uCtrl = nan;
                                vCtrl = nan;
                            end

                            if isfinite(uCtrl) && isfinite(vCtrl)
                                tagLostSearchStartT = nan;
                                errU = cfg.control.target_u - uCtrl;
                                errV = cfg.control.target_v - vCtrl;

                                centeredCtrl = sqrt(errU^2 + errV^2) <= cfg.control.xy_control_center_deadband;

                                [ux, pidX] = pidStep(errV, dtCtrl, pidX, cfg.control.xy_kp, cfg.control.xy_ki, cfg.control.xy_kd, cfg.control.xy_i_limit, cfg.control.xy_cmd_limit);
                                [uy, pidY] = pidStep(errU, dtCtrl, pidY, cfg.control.xy_kp, cfg.control.xy_ki, cfg.control.xy_kd, cfg.control.xy_i_limit, cfg.control.xy_cmd_limit);

                                cmdX = cfg.control.xy_map_sign_x_from_v * ux;
                                cmdY = cfg.control.xy_map_sign_y_from_u * uy;

                                if centeredCtrl
                                    cmdX = 0.0;
                                    cmdY = 0.0;
                                end
                            else
                                pidX = initPidState();
                                pidY = initPidState();

                                if cfg.search.enable_spiral_when_tag_lost
                                    if ~isfinite(tagLostSearchStartT)
                                        tagLostSearchStartT = tk;
                                    end

                                    tSearch = max(0.0, tk - tagLostSearchStartT);
                                    rSearch = cfg.search.spiral_start_radius + cfg.search.spiral_growth_per_sec * tSearch;
                                    rSearch = min(rSearch, cfg.search.spiral_cmd_max);
                                    th = cfg.search.spiral_omega_rad_sec * tSearch;

                                    cmdX = clampValue(rSearch * cos(th), -abs(cfg.search.spiral_cmd_max), abs(cfg.search.spiral_cmd_max));
                                    cmdY = clampValue(rSearch * sin(th), -abs(cfg.search.spiral_cmd_max), abs(cfg.search.spiral_cmd_max));
                                end
                            end
                        end
                end
            end

            if cfg.learning.enable && isFlying && (controlPhase == "xy_hold")
                if tagLockReadyNow
                    if ~isfinite(tagLockHoldStartT)
                        tagLockHoldStartT = tk;
                    end
                else
                    tagLockHoldStartT = nan;
                end

                if ~tagLockAcquired && isfinite(tagLockHoldStartT) && ((tk - tagLockHoldStartT) >= cfg.learning.tag_lock_hold_sec)
                    tagLockAcquired = true;

                    waitSec = randRange(cfg.learning.random_landing_wait_min_sec, cfg.learning.random_landing_wait_max_sec);
                    durSec = randRange(cfg.learning.random_cmd_duration_min_sec, cfg.learning.random_cmd_duration_max_sec);
                    randomBiasX = randRange(-cfg.learning.random_xy_cmd_max, cfg.learning.random_xy_cmd_max);
                    randomBiasY = randRange(-cfg.learning.random_xy_cmd_max, cfg.learning.random_xy_cmd_max);

                    randomLandingStartT = tk + waitSec;
                    randomLandingEndT = randomLandingStartT + durSec;
                    randomLandingPlanned = true;

                    fprintf('[PIPELINE] s%03d tag lock acquired: random landing in %.1fs (duration %.1fs, bias=(%.2f,%.2f))\n', ...
                        scenarioId, waitSec, durSec, randomBiasX, randomBiasY);
                end

                if randomLandingPlanned && ~landingSent
                    if tk >= randomLandingStartT && tk < randomLandingEndT
                        cmdX = clampValue(cmdX + randomBiasX, -abs(cfg.control.xy_cmd_limit), abs(cfg.control.xy_cmd_limit));
                        cmdY = clampValue(cmdY + randomBiasY, -abs(cfg.control.xy_cmd_limit), abs(cfg.control.xy_cmd_limit));
                    elseif tk >= randomLandingEndT
                        landingDescentActive = true;
                        fprintf('[PIPELINE] s%03d smooth landing descent started at t=%.1fs\n', scenarioId, tk);
                    end
                end
            end

            hoverDelayOk = isfinite(hoverStartT) && ((tk - hoverStartT) >= cfg.wind.start_delay_after_hover_sec);
            if cfg.wind.start_require_tag_centered
                centerForWind = (controlPhase == "xy_hold") && isFlying && tagLockReadyNow;
                if centerForWind
                    if ~isfinite(hoverCenterHoldStartT)
                        hoverCenterHoldStartT = tk;
                    end
                else
                    hoverCenterHoldStartT = nan;
                end
                hoverCenterReady = isfinite(hoverCenterHoldStartT) && ((tk - hoverCenterHoldStartT) >= cfg.wind.start_tag_center_hold_sec);
            else
                hoverCenterReady = true;
            end

            if ~windArmed && cfg.wind.enable && hoverDelayOk && hoverCenterReady
                windArmed = true;
                fprintf('[PIPELINE] s%03d wind enabled after hover+%.1fs and tag-center hold %.1fs at t=%.1fs\n', ...
                    scenarioId, cfg.wind.start_delay_after_hover_sec, cfg.wind.start_tag_center_hold_sec, tk);
            end

            if isFlying
                if landingSent
                    cmdX = 0.0;
                    cmdY = 0.0;

                    % During auto-landing, apply mild upward damping when descent is too fast.
                    if (~cfg.control.use_position_control_mode) && cfg.control.enable_landing_vz_damping && isfinite(vz(k))
                        vzErr = cfg.control.landing_vz_target_mps - vz(k);
                        cmdZ = clampValue(cfg.control.landing_vz_kp * vzErr, 0.0, abs(cfg.control.landing_up_cmd_limit));
                    end
                end

                usePosCtrlNow = cfg.control.use_position_control_mode && posCtrlConfigured;

                if usePosCtrlNow
                    if ~isfinite(targetPoseX) && isfinite(poseXNow)
                        targetPoseX = poseXNow;
                    end
                    if ~isfinite(targetPoseY) && isfinite(poseYNow)
                        targetPoseY = poseYNow;
                    end
                    if ~isfinite(targetPoseZ) && isfinite(z(k))
                        targetPoseZ = max(z(k), searchMinAlt);
                    end

                    xyScale = cfg.control.poscmd_xy_rate_scale;
                    if landingDescentActive
                        xyScale = cfg.control.poscmd_xy_rate_scale_landing;
                    end
                    if isfinite(targetPoseX)
                        targetPoseX = targetPoseX + cmdX * dtCtrl * xyScale;
                    end
                    if isfinite(targetPoseY)
                        targetPoseY = targetPoseY + cmdY * dtCtrl * xyScale;
                    end

                    if landingDescentActive && ~landingSent
                        if isfinite(targetPoseZ)
                            targetPoseZ = targetPoseZ - cfg.control.landing_descent_rate_mps * dtCtrl;
                            targetPoseZ = max(cfg.control.landing_cmd_alt_min_m, targetPoseZ);
                        end

                        if isfinite(z(k)) && (z(k) <= cfg.control.landing_land_trigger_alt_m)
                            landCmd = sprintf('bash -i -c "%s"', shellEscapeDoubleQuotes(cfg.shell.land_cli_cmd));
                            [landSt, landOut] = system(landCmd);
                            if landSt ~= 0
                                fprintf('[PIPELINE] s%03d land CLI failed at t=%.1fs (code=%d): %s\n', scenarioId, tk, landSt, strtrim(landOut));
                            end
                            landingSent = true;
                            landingSentT = tk;
                            fprintf('[PIPELINE] s%03d final land command sent at t=%.1fs (z=%.2f)\n', scenarioId, tk, z(k));
                        end
                    elseif ~landingSent && isfinite(targetPoseZ)
                        % Keep enough camera standoff while searching/aliging the landing tag.
                        targetPoseZ = max(targetPoseZ, searchMinAlt);
                    end

                    if ~isfinite(targetPoseX), targetPoseX = 0.0; end
                    if ~isfinite(targetPoseY), targetPoseY = 0.0; end
                    if ~isfinite(targetPoseZ), targetPoseZ = searchMinAlt; end
                    msgCmdVel.linear.x = targetPoseX;
                    msgCmdVel.linear.y = targetPoseY;
                    msgCmdVel.linear.z = targetPoseZ;
                else
                    msgCmdVel.linear.x = cmdX;
                    msgCmdVel.linear.y = cmdY;
                    msgCmdVel.linear.z = cmdZ;
                end
                msgCmdVel.angular.x = 0.0;
                msgCmdVel.angular.y = 0.0;
                msgCmdVel.angular.z = 0.0;
                send(pubCmdVel, msgCmdVel);
            end
        end

        if cfg.visualization.enable
            viz = updateScenarioLivePlots(viz, tk, z(k), tagErr(k), cmdX, cmdY, string(controlPhase), isFlying, tagDetected, predOk);
        end

        if mod(k, heartbeatEvery) == 0
            fprintf('[PIPELINE] s%03d t=%.1fs phase=%s fly=%d tag=%d pred=%d tkofPub=%d cmd=(%.2f,%.2f) z=%.2f\n', ...
                scenarioId, tk, char(controlPhase), isFlying, tagDetected, predOk, takeoffPublishCount, cmdX, cmdY, z(k));
        end

        if landingSent
            landedByState = isfinite(stateNow) && (stateNow == cfg.thresholds.land_state_value);
            evalWindowSamples = max(3, round(cfg.control.landing_vz_eval_window_sec / cfg.scenario.sample_period_sec));
            recentVz = tailVec(vz(1:k), evalWindowSamples);
            [vzTrendNow, vzOscStdNow, vzAccelRmsNow] = calcVzMotionMetrics(recentVz, cfg.scenario.sample_period_sec);
            landedByPose = isfinite(z(k)) && ...
                (z(k) <= (cfg.thresholds.landed_altitude_max_m + 0.05)) && ...
                isfinite(vzTrendNow) && (abs(vzTrendNow) <= max(cfg.thresholds.final_speed_max_mps, 0.12)) && ...
                isfinite(vzOscStdNow) && (vzOscStdNow <= cfg.thresholds.final_stability_std_vz_osc_max) && ...
                isfinite(vzAccelRmsNow) && (vzAccelRmsNow <= cfg.thresholds.final_touchdown_accel_rms_max);

            if landedByState || landedByPose
                if ~isfinite(landedHoldStartT)
                    landedHoldStartT = tk;
                end
            else
                landedHoldStartT = nan;
            end

            if isfinite(landedHoldStartT) && ((tk - landedHoldStartT) >= cfg.scenario.early_stop_after_landing_sec)
                fprintf('[PIPELINE] s%03d landing confirmed for %.1fs, ending scenario loop early at t=%.1fs\n', ...
                    scenarioId, cfg.scenario.early_stop_after_landing_sec, tk);
                break;
            end
        end

        pause(cfg.scenario.sample_period_sec);
    end

    % Neutralize motion command, then trigger land command and watch post-landing window.
    msgCmdVel.linear.x = 0.0;
    msgCmdVel.linear.y = 0.0;
    msgCmdVel.linear.z = 0.0;
    msgCmdVel.angular.x = 0.0;
    msgCmdVel.angular.y = 0.0;
    msgCmdVel.angular.z = 0.0;
    send(pubCmdVel, msgCmdVel);

    if ~landingSent
        landCmd = sprintf('bash -i -c "%s"', shellEscapeDoubleQuotes(cfg.shell.land_cli_cmd));
        [landSt, landOut] = system(landCmd);
        if landSt ~= 0
            fprintf('[PIPELINE] s%03d final land CLI failed (code=%d): %s\n', scenarioId, landSt, strtrim(landOut));
        end
        landingSent = true;
        landingSentT = toc(t0);
    end

    postN = max(1, floor(cfg.scenario.post_land_observe_sec / cfg.scenario.sample_period_sec));
    for m = 1:postN
        k = sampleN + m;
        poseMsg = tryReceive(subPose, recvTimeout);
        if ~isempty(poseMsg)
            z(end+1,1) = double(poseMsg.position.z); %#ok<AGROW>
            q = poseMsg.orientation;
            [r, p, ~] = quat2eul_local([q.w, q.x, q.y, q.z]);
            rollDeg(end+1,1) = abs(rad2deg(r)); %#ok<AGROW>
            pitchDeg(end+1,1) = abs(rad2deg(p)); %#ok<AGROW>
        else
            z(end+1,1) = nan; %#ok<AGROW>
            rollDeg(end+1,1) = nan; %#ok<AGROW>
            pitchDeg(end+1,1) = nan; %#ok<AGROW>
        end

        velMsg = tryReceive(subVel, recvTimeout);
        if ~isempty(velMsg)
            vx = double(velMsg.linear.x);
            vy = double(velMsg.linear.y);
            vz(end+1,1) = double(velMsg.linear.z); %#ok<AGROW>
            speedAbs(end+1,1) = sqrt(vx*vx + vy*vy + vz(end)^2); %#ok<AGROW>
        else
            vz(end+1,1) = nan; %#ok<AGROW>
            speedAbs(end+1,1) = nan; %#ok<AGROW>
        end

        stateMsg = tryReceive(subState, recvTimeout);
        if ~isempty(stateMsg)
            stateVal(end+1,1) = double(stateMsg.data); %#ok<AGROW>
        else
            stateVal(end+1,1) = nan; %#ok<AGROW>
        end

        if ~isempty(subImu)
            imuMsg = tryReceive(subImu, recvTimeout);
            if ~isempty(imuMsg)
                [angNow, accNow] = parseImuMetrics(imuMsg);
                imuAngVel(end+1,1) = angNow; %#ok<AGROW>
                imuLinAcc(end+1,1) = accNow; %#ok<AGROW>
            else
                imuAngVel(end+1,1) = nan; %#ok<AGROW>
                imuLinAcc(end+1,1) = nan; %#ok<AGROW>
            end
        else
            imuAngVel(end+1,1) = nan; %#ok<AGROW>
            imuLinAcc(end+1,1) = nan; %#ok<AGROW>
        end

        tagMsg = tryReceive(subTag, recvTimeout);
        if ~isempty(tagMsg)
            tagErr(end+1,1) = parseTagErrorFromBridge(tagMsg); %#ok<AGROW>
        else
            tagErr(end+1,1) = nan; %#ok<AGROW>
        end

        bumpMsg = [];
        if ~isempty(subBumpers)
            bumpMsg = tryReceive(subBumpers, recvTimeout);
        end
        if ~isempty(bumpMsg)
            [cNow, fNow, flNow, frNow, rlNow, rrNow] = parseContactForces(bumpMsg);
            bumperContact(end+1,1) = cNow; %#ok<AGROW>
            contactForce(end+1,1) = fNow; %#ok<AGROW>
            armForceFL(end+1,1) = flNow; %#ok<AGROW>
            armForceFR(end+1,1) = frNow; %#ok<AGROW>
            armForceRL(end+1,1) = rlNow; %#ok<AGROW>
            armForceRR(end+1,1) = rrNow; %#ok<AGROW>
        else
            bumperContact(end+1,1) = 0; %#ok<AGROW>
            contactForce(end+1,1) = nan; %#ok<AGROW>
            armForceFL(end+1,1) = nan; %#ok<AGROW>
            armForceFR(end+1,1) = nan; %#ok<AGROW>
            armForceRL(end+1,1) = nan; %#ok<AGROW>
            armForceRR(end+1,1) = nan; %#ok<AGROW>
        end

        windMsg = tryReceive(subWind, recvTimeout);
        if ~isempty(windMsg)
            windSpeed(end+1,1) = parseWindSpeed(windMsg); %#ok<AGROW>
        else
            windSpeed(end+1,1) = nan; %#ok<AGROW>
        end

        pause(cfg.scenario.sample_period_sec);
    end

    if cfg.control.use_position_control_mode
        msgPosCtrl.data = false;
        send(pubPosCtrl, msgPosCtrl);
    end

    result = summarizeAndLabelScenario(cfg, scenarioId, scenarioCfg, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, bumperContact, ...
        imuAngVel, imuLinAcc, contactForce, armForceFL, armForceFR, armForceRL, armForceRR);
    result.tag_lock_acquired = tagLockAcquired;
    result.landing_cmd_time = landingSentT;
    result.random_bias_x = randomBiasX;
    result.random_bias_y = randomBiasY;
end


function out = summarizeAndLabelScenario(cfg, scenarioId, scenarioCfg, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, bumperContact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR)
    out = emptyScenarioResult();
    out.scenario_id = scenarioId;
    out.hover_height_cmd = scenarioCfg.hover_height_m;
    out.wind_speed_cmd = scenarioCfg.wind_speed;
    out.wind_dir_cmd = scenarioCfg.wind_dir;

    out.mean_wind_speed = nanmeanSafe(windSpeed);
    out.max_wind_speed = nanmaxSafe(windSpeed);
    out.mean_abs_roll_deg = nanmeanSafe(abs(rollDeg));
    out.mean_abs_pitch_deg = nanmeanSafe(abs(pitchDeg));
    out.mean_abs_vz = nanmeanSafe(abs(vz));
    out.max_abs_vz = nanmaxSafe(abs(vz));
    out.mean_tag_error = nanmeanSafe(tagErr);
    out.max_tag_error = nanmaxSafe(tagErr);

    out.final_altitude = nanlast(z);
    out.landing_height_m = out.final_altitude;
    out.final_abs_speed = nanlast(speedAbs);
    out.final_abs_roll_deg = nanlast(abs(rollDeg));
    out.final_abs_pitch_deg = nanlast(abs(pitchDeg));
    out.final_tag_error = nanlast(tagErr);

    finalWindow = max(3, round(5.0 / cfg.scenario.sample_period_sec));
    touchdownIdx = find(isfinite(z) & (z <= (cfg.thresholds.landed_altitude_max_m + 0.02)), 1, 'first');
    touchdownPostFinite = 0;
    if ~isempty(touchdownIdx)
        settleSkip = round(1.0 / cfg.scenario.sample_period_sec);
        segStart = min(numel(z), touchdownIdx + max(0, settleSkip));
        zStableSrc = z(segStart:end);
        vzStableSrc = vz(segStart:end);
        touchdownPostFinite = sum(isfinite(zStableSrc) & isfinite(vzStableSrc));
    else
        zStableSrc = tailVec(z, finalWindow);
        vzStableSrc = tailVec(vz, finalWindow);
    end

    if sum(isfinite(zStableSrc)) < 3
        if ~isempty(touchdownIdx)
            idx0 = max(1, touchdownIdx - finalWindow + 1);
            zStableSrc = z(idx0:touchdownIdx);
        else
            zStableSrc = tailVec(z, finalWindow);
        end
    end
    if sum(isfinite(vzStableSrc)) < 3
        if ~isempty(touchdownIdx)
            idx0 = max(1, touchdownIdx - finalWindow + 1);
            vzStableSrc = vz(idx0:touchdownIdx);
        else
            vzStableSrc = tailVec(vz, finalWindow);
        end
    end

    out.stability_std_z = nanstdSafe(zStableSrc);
    out.stability_std_vz = nanstdSafe(vzStableSrc);
    [~, out.stability_std_vz_osc, out.touchdown_accel_rms] = calcVzMotionMetrics(vzStableSrc, cfg.scenario.sample_period_sec);

    out.contact_count = sum(bumperContact > 0);
    out.mean_imu_ang_vel = nanmeanSafe(imuAngVel);
    out.max_imu_ang_vel = nanmaxSafe(imuAngVel);
    out.mean_imu_lin_acc = nanmeanSafe(imuLinAcc);
    out.max_imu_lin_acc = nanmaxSafe(imuLinAcc);
    out.max_contact_force = nanmaxSafe(contactForce);
    out.arm_force_fl_mean = nanmeanSafe(armFL);
    out.arm_force_fr_mean = nanmeanSafe(armFR);
    out.arm_force_rl_mean = nanmeanSafe(armRL);
    out.arm_force_rr_mean = nanmeanSafe(armRR);
    out.arm_force_imbalance = nanmaxSafe([abs(armFL-armFR); abs(armRL-armRR)]);

    stateFinal = nanlast(stateVal);
    out.final_state = stateFinal;

    c = cfg.thresholds;
    condState = isfinite(stateFinal) && stateFinal == c.land_state_value;
    condAlt = isfinite(out.final_altitude) && out.final_altitude <= c.landed_altitude_max_m;
    condSpeed = isfinite(out.final_abs_speed) && out.final_abs_speed <= c.final_speed_max_mps;
    condRoll = isfinite(out.final_abs_roll_deg) && out.final_abs_roll_deg <= c.final_attitude_max_deg;
    condPitch = isfinite(out.final_abs_pitch_deg) && out.final_abs_pitch_deg <= c.final_attitude_max_deg;
    condTag = (~isfinite(out.final_tag_error)) || (out.final_tag_error <= c.final_tag_error_max);
    condStdZ = isfinite(out.stability_std_z) && out.stability_std_z <= c.final_stability_std_z_max;
    condStdVz = isfinite(out.stability_std_vz) && out.stability_std_vz <= c.final_stability_std_vz_max;
    condVzOsc = isfinite(out.stability_std_vz_osc) && out.stability_std_vz_osc <= c.final_stability_std_vz_osc_max;
    condAccel = isfinite(out.touchdown_accel_rms) && out.touchdown_accel_rms <= c.final_touchdown_accel_rms_max;
    condImuAng = (~isfinite(out.max_imu_ang_vel)) || (out.max_imu_ang_vel <= c.final_imu_ang_vel_rms_max);
    condImuAcc = (~isfinite(out.max_imu_lin_acc)) || (out.max_imu_lin_acc <= c.final_imu_lin_acc_rms_max);
    condContactForce = (~isfinite(out.max_contact_force)) || (out.max_contact_force <= c.final_contact_force_max_n);
    condArmBalance = (~isfinite(out.arm_force_imbalance)) || (out.arm_force_imbalance <= c.final_arm_force_imbalance_max_n);
    % If touchdown was detected but there are too few post-touchdown samples,
    % avoid penalizing stability metrics dominated by pre-touchdown trajectory.
    if ~isempty(touchdownIdx) && touchdownPostFinite < 3
        condStdZ = true;
        condStdVz = true;
        condVzOsc = true;
        condAccel = true;
    end
    % Ground contact is expected after successful landing.
    if condState && condAlt
        condContact = true;
    else
        condContact = out.contact_count <= c.bumpers_contact_max;
    end

    passAll = condState && condAlt && condSpeed && condRoll && condPitch && condTag && condStdZ && condStdVz && condVzOsc && condAccel && condContact && ...
        condImuAng && condImuAcc && condContactForce && condArmBalance;

    if passAll
        out.label = "stable";
        out.success = true;
        out.failure_reason = "";
    else
        out.label = "unstable";
        out.success = false;
        out.failure_reason = buildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz, condVzOsc, condAccel, condContact, ...
            condImuAng, condImuAcc, condContactForce, condArmBalance);
    end

    fprintf('[PIPELINE] Scenario %d label=%s reason=%s | final(z=%.3f, v=%.3f, roll=%.1f, pitch=%.1f, tag=%.3f)\n', ...
        scenarioId, out.label, out.failure_reason, out.final_altitude, out.final_abs_speed, ...
        out.final_abs_roll_deg, out.final_abs_pitch_deg, out.final_tag_error);
end


function reason = buildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz, condVzOsc, condAccel, condContact, condImuAng, condImuAcc, condContactForce, condArmBalance)
    parts = strings(0,1);
    if ~condState, parts(end+1,1) = "state_not_landed"; end %#ok<AGROW>
    if ~condAlt, parts(end+1,1) = "altitude_high"; end %#ok<AGROW>
    if ~condSpeed, parts(end+1,1) = "speed_high"; end %#ok<AGROW>
    if ~condRoll, parts(end+1,1) = "roll_high"; end %#ok<AGROW>
    if ~condPitch, parts(end+1,1) = "pitch_high"; end %#ok<AGROW>
    if ~condTag, parts(end+1,1) = "tag_error_high"; end %#ok<AGROW>
    if ~condStdZ, parts(end+1,1) = "z_unstable"; end %#ok<AGROW>
    if ~condStdVz, parts(end+1,1) = "vz_unstable"; end %#ok<AGROW>
    if ~condVzOsc, parts(end+1,1) = "vz_oscillation_high"; end %#ok<AGROW>
    if ~condAccel, parts(end+1,1) = "touchdown_accel_high"; end %#ok<AGROW>
    if ~condContact, parts(end+1,1) = "contact_detected"; end %#ok<AGROW>
    if ~condImuAng, parts(end+1,1) = "imu_angular_rate_high"; end %#ok<AGROW>
    if ~condImuAcc, parts(end+1,1) = "imu_linear_accel_high"; end %#ok<AGROW>
    if ~condContactForce, parts(end+1,1) = "contact_force_high"; end %#ok<AGROW>
    if ~condArmBalance, parts(end+1,1) = "arm_force_imbalance"; end %#ok<AGROW>

    if isempty(parts)
        reason = "unknown";
    else
        reason = strjoin(parts, ';');
    end
end


function tbl = toSummaryTable(results)
    if isempty(results)
        tbl = table();
        return;
    end

    tbl = struct2table(results);

    wanted = {
        'scenario_id','label','success','failure_reason', ...
        'tag_lock_acquired','landing_cmd_time','random_bias_x','random_bias_y', ...
        'hover_height_cmd','landing_height_m', ...
        'wind_speed_cmd','wind_dir_cmd', ...
        'mean_wind_speed','max_wind_speed', ...
        'mean_abs_roll_deg','mean_abs_pitch_deg', ...
        'mean_abs_vz','max_abs_vz', ...
        'mean_tag_error','max_tag_error', ...
        'final_altitude','final_abs_speed','final_abs_roll_deg','final_abs_pitch_deg','final_tag_error', ...
        'stability_std_z','stability_std_vz','stability_std_vz_osc','touchdown_accel_rms','contact_count', ...
        'mean_imu_ang_vel','max_imu_ang_vel','mean_imu_lin_acc','max_imu_lin_acc', ...
        'max_contact_force','arm_force_fl_mean','arm_force_fr_mean','arm_force_rl_mean','arm_force_rr_mean','arm_force_imbalance', ...
        'final_state', ...
        'launch_pid','launch_log','exception_message'
    };

    cols = intersect(wanted, tbl.Properties.VariableNames, 'stable');
    tbl = tbl(:, cols);
end


function modelInfo = trainAndSaveModel(cfg, tbl)
    featNames = cellstr(cfg.model.feature_names);

    valid = (tbl.label == "stable") | (tbl.label == "unstable");
    if sum(valid) < 2
        error('Not enough labeled samples for training. Need at least 2 rows.');
    end

    trainTbl = tbl(valid, :);
    X = zeros(height(trainTbl), numel(featNames));
    for i = 1:numel(featNames)
        col = featNames{i};
        if ismember(col, trainTbl.Properties.VariableNames)
            X(:,i) = toNumericVector(trainTbl.(col));
        else
            error('Missing feature column for training: %s', col);
        end
    end

    y = string(trainTbl.label);
    y(y ~= "stable") = "unstable";

    model = trainGaussianNB(X, y, cfg.model.feature_names);

    ts = timestampForFile();
    modelPath = fullfile(cfg.paths.model_dir, sprintf('landing_model_%s.mat', ts));
    save(modelPath, 'model');

    modelInfo = struct('model_path', string(modelPath), 'n_samples', numel(y));
end


function modelInfo = savePlaceholderModel(cfg, tbl, reason)
    feat = cfg.model.feature_names;
    nFeat = numel(feat);

    model = struct();
    model.kind = "gaussian_nb";
    model.class_names = ["stable"; "unstable"];
    model.feature_names = feat;
    model.mu = zeros(2, nFeat);
    model.sigma2 = ones(2, nFeat);
    model.prior = [0.5; 0.5];
    model.placeholder = true;
    model.placeholder_reason = string(reason);
    model.created_at = string(datetime('now'));
    model.n_rows = height(tbl);

    ts = timestampForFile();
    modelPath = fullfile(cfg.paths.model_dir, sprintf('landing_model_%s_placeholder.mat', ts));
    save(modelPath, 'model');

    modelInfo = struct('model_path', string(modelPath), 'n_samples', 0, 'placeholder', true, 'reason', string(reason));
end


function finalizeInfo = finalizePipelineArtifacts(cfg, results, runStatus)
    summaryTbl = toSummaryTable(results);
    ts = timestampForFile();
    tag = lower(char(runStatus));

    summaryPathMat = fullfile(cfg.paths.data_dir, sprintf('landing_dataset_%s_%s.mat', ts, tag));
    summaryPathCsv = fullfile(cfg.paths.data_dir, sprintf('landing_dataset_%s_%s.csv', ts, tag));
    save(summaryPathMat, 'results', 'summaryTbl', 'cfg', 'runStatus');
    writetable(summaryTbl, summaryPathCsv);

    fprintf('\n[PIPELINE] Dataset saved:\n  %s\n  %s\n', summaryPathMat, summaryPathCsv);

    valid = false(height(summaryTbl), 1);
    if ismember('label', summaryTbl.Properties.VariableNames)
        valid = (summaryTbl.label == "stable") | (summaryTbl.label == "unstable");
    end

    try
        if sum(valid) >= 2
            modelInfo = trainAndSaveModel(cfg, summaryTbl);
            fprintf('[PIPELINE] Training model saved: %s\n', modelInfo.model_path);
        else
            modelInfo = savePlaceholderModel(cfg, summaryTbl, "not_enough_labeled_samples");
            fprintf('[PIPELINE] Placeholder model saved: %s\n', modelInfo.model_path);
        end
    catch ME
        warning('[PIPELINE] Training failed, saving placeholder model: %s', ME.message);
        modelInfo = savePlaceholderModel(cfg, summaryTbl, "training_exception");
        fprintf('[PIPELINE] Placeholder model saved: %s\n', modelInfo.model_path);
    end

    inferModel = [];
    inferModelPath = "";
    predCsvPath = "";
    inferenceRan = false;
    try
        cfgInfer = cfg;
        cfgInfer.inference.model_file = char(modelInfo.model_path);
        [inferModel, inferModelPath] = loadModelForInference(cfgInfer);
        [predLabels, predScores] = inferOnTable(summaryTbl, inferModel);

        summaryTbl.pred_label = string(predLabels);
        summaryTbl.pred_score = predScores;

        predCsvPath = fullfile(cfg.paths.data_dir, sprintf('landing_inference_%s_%s.csv', ts, tag));
        writetable(summaryTbl, predCsvPath);
        inferenceRan = true;

        fprintf('[PIPELINE] Inference model used: %s\n', inferModelPath);
        fprintf('[PIPELINE] Inference result csv: %s\n', predCsvPath);
    catch ME
        warning('[PIPELINE] Inference skipped: %s', ME.message);
    end

    finalizeInfo = struct();
    finalizeInfo.summaryTbl = summaryTbl;
    finalizeInfo.summary_mat_path = string(summaryPathMat);
    finalizeInfo.summary_csv_path = string(summaryPathCsv);
    finalizeInfo.modelInfo = modelInfo;
    finalizeInfo.inferModel = inferModel;
    finalizeInfo.inferModelPath = string(inferModelPath);
    finalizeInfo.predCsvPath = string(predCsvPath);
    finalizeInfo.inference_ran = inferenceRan;
end


function savePipelineCheckpoint(cfg, results, reasonTag)
    if ~isfield(cfg, 'persistence') || ~isfield(cfg.persistence, 'enable_checkpoint') || ~cfg.persistence.enable_checkpoint
        return;
    end

    try
        summaryTbl = toSummaryTable(results);
        checkpoint = struct();
        checkpoint.reason = string(reasonTag);
        checkpoint.timestamp = string(datetime('now'));
        checkpoint.n_rows = height(summaryTbl);

        save(cfg.persistence.checkpoint_mat, 'results', 'summaryTbl', 'checkpoint');
        writetable(summaryTbl, cfg.persistence.checkpoint_csv);
    catch ME
        warning('[PIPELINE] Checkpoint save failed: %s', ME.message);
    end
end


function printRunningLabelStats(results, idxNow, totalCount)
    if isempty(results)
        return;
    end

    labels = strings(numel(results), 1);
    for i = 1:numel(results)
        if isfield(results(i), 'label')
            labels(i) = string(results(i).label);
        end
    end

    nStable = sum(labels == "stable");
    nUnstable = sum(labels == "unstable");
    nValid = nStable + nUnstable;

    if nValid > 0
        stableRatio = nStable / nValid;
        unstableRatio = nUnstable / nValid;
        fprintf('[PIPELINE] Label stats after scenario %d/%d: stable=%d (%.1f%%), unstable=%d (%.1f%%)\n', ...
            idxNow, totalCount, nStable, 100.0*stableRatio, nUnstable, 100.0*unstableRatio);
    else
        fprintf('[PIPELINE] Label stats after scenario %d/%d: no valid labels yet\n', idxNow, totalCount);
    end
end


function [model, modelPath] = loadModelForInference(cfg)
    if isfield(cfg.inference, 'model_file') && ~isempty(cfg.inference.model_file)
        modelPath = string(cfg.inference.model_file);
        if ~isfile(modelPath)
            error('Specified model does not exist: %s', modelPath);
        end
    else
        dd = dir(fullfile(cfg.paths.model_dir, 'landing_model_*.mat'));
        if isempty(dd)
            error('No model file found in %s', cfg.paths.model_dir);
        end
        [~, idx] = max([dd.datenum]);
        modelPath = string(fullfile(dd(idx).folder, dd(idx).name));
    end

    S = load(modelPath);
    if ~isfield(S, 'model')
        error('Invalid model file: %s', modelPath);
    end
    model = S.model;
end


function [predLabel, predScore] = inferOnTable(tbl, model)
    featNames = cellstr(model.feature_names);
    X = zeros(height(tbl), numel(featNames));
    for i = 1:numel(featNames)
        col = featNames{i};
        if ismember(col, tbl.Properties.VariableNames)
            X(:,i) = toNumericVector(tbl.(col));
        else
            X(:,i) = 0.0;
        end
    end

    [predLabel, predScore] = predictGaussianNB(model, X);
end


function model = trainGaussianNB(X, y, featureNames)
    cls = unique(y);
    nClass = numel(cls);
    nFeat = size(X, 2);

    mu = zeros(nClass, nFeat);
    sigma2 = ones(nClass, nFeat);
    prior = zeros(nClass, 1);

    X = sanitizeFeatures(X);

    for i = 1:nClass
        mask = (y == cls(i));
        Xi = X(mask, :);
        prior(i) = max(mean(mask), eps);
        mu(i,:) = mean(Xi, 1);
        sigma2(i,:) = var(Xi, 0, 1);
        sigma2(i, sigma2(i,:) < 1e-6) = 1e-6;
    end

    model = struct();
    model.kind = "gaussian_nb";
    model.class_names = cls;
    model.feature_names = featureNames;
    model.mu = mu;
    model.sigma2 = sigma2;
    model.prior = prior;
    model.created_at = string(datetime('now'));
end


function [predLabel, predScore] = predictGaussianNB(model, X)
    X = sanitizeFeatures(X);
    n = size(X, 1);
    c = numel(model.class_names);
    logp = zeros(n, c);

    for i = 1:c
        mu = model.mu(i,:);
        s2 = model.sigma2(i,:);
        lp = -0.5 * sum(log(2*pi*s2) + ((X - mu).^2) ./ s2, 2);
        lp = lp + log(model.prior(i));
        logp(:,i) = lp;
    end

    [mx, idx] = max(logp, [], 2);
    predLabel = strings(n,1);
    for k = 1:n
        predLabel(k) = model.class_names(idx(k));
    end

    lse = zeros(n,1);
    for k = 1:n
        a = logp(k,:) - mx(k);
        lse(k) = mx(k) + log(sum(exp(a)));
    end
    predScore = exp(mx - lse);
end


function x = sanitizeFeatures(x)
    x(~isfinite(x)) = 0.0;
end


function y = toNumericVector(x)
    if isnumeric(x)
        y = double(x);
    elseif islogical(x)
        y = double(x);
    elseif isstring(x)
        y = double(str2double(x));
        y(~isfinite(y)) = 0.0;
    elseif iscell(x)
        y = zeros(numel(x),1);
        for i = 1:numel(x)
            if isnumeric(x{i})
                y(i) = double(x{i});
            else
                v = str2double(string(x{i}));
                if ~isfinite(v)
                    v = 0.0;
                end
                y(i) = v;
            end
        end
    else
        y = zeros(numel(x),1);
    end

    if isrow(y)
        y = y.';
    end

    y(~isfinite(y)) = 0.0;
end


function [ok, diagText] = waitForScenarioTopicsReady(cfg, timeoutSec)
    ok = false;
    diagText = "";
    t0 = tic;

    while toc(t0) <= timeoutSec
        [posePub, ~] = getTopicCounts(cfg.topics.pose, cfg.shell.setup_cmd);
        [statePub, ~] = getTopicCounts(cfg.topics.state, cfg.shell.setup_cmd);
        [~, takeoffSub] = getTopicCounts(cfg.topics.takeoff_cmd, cfg.shell.setup_cmd);

        if posePub > 0 && statePub > 0 && takeoffSub > 0
            ok = true;
            diagText = sprintf('ready posePub=%d statePub=%d takeoffSub=%d', posePub, statePub, takeoffSub);
            return;
        end

        pause(0.5);
    end

    [posePub, poseSub] = getTopicCounts(cfg.topics.pose, cfg.shell.setup_cmd);
    [statePub, stateSub] = getTopicCounts(cfg.topics.state, cfg.shell.setup_cmd);
    [takeoffPub, takeoffSub] = getTopicCounts(cfg.topics.takeoff_cmd, cfg.shell.setup_cmd);
    diagText = sprintf('not ready after %.1fs | pose(pub=%d sub=%d) state(pub=%d sub=%d) takeoff(pub=%d sub=%d)', ...
        timeoutSec, posePub, poseSub, statePub, stateSub, takeoffPub, takeoffSub);
end


function [pubCount, subCount] = getTopicCounts(topicName, setupCmd)
    pubCount = 0;
    subCount = 0;

    cmd = sprintf('bash -i -c "%s && ros2 topic info %s 2>/dev/null"', ...
        shellEscapeDoubleQuotes(setupCmd), shellEscapeDoubleQuotes(topicName));
    [st, out] = system(cmd);
    if st ~= 0
        return;
    end

    pTok = regexp(out, 'Publisher count:\s*(\d+)', 'tokens', 'once');
    sTok = regexp(out, 'Subscription count:\s*(\d+)', 'tokens', 'once');

    if ~isempty(pTok)
        pubCount = str2double(pTok{1});
        if ~isfinite(pubCount), pubCount = 0; end
    end
    if ~isempty(sTok)
        subCount = str2double(sTok{1});
        if ~isfinite(subCount), subCount = 0; end
    end
end


function x = tryReceive(sub, timeout)
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end


function ws = parseWindSpeed(msg)
    ws = nan;
    try
        d = double(msg.data);
        if ~isempty(d)
            ws = d(1);
        end
    catch
    end
end


function [windSpeed, windDir] = pickScenarioWind(cfg, scenarioId)
    windSpeed = cfg.wind.speed_min + rand() * (cfg.wind.speed_max - cfg.wind.speed_min);
    windDir = cfg.wind.direction_min + rand() * (cfg.wind.direction_max - cfg.wind.direction_min);

    src = "random";
    if isfield(cfg.wind, 'source')
        src = string(cfg.wind.source);
    end
    if src ~= "kma_csv"
        return;
    end

    profile = getKmaWindProfile(cfg);
    if isempty(profile) || isempty(profile.speed)
        return;
    end

    idx = mod(max(1, scenarioId) - 1, numel(profile.speed)) + 1;
    windSpeed = profile.speed(idx);
    windDir = profile.dir(idx);
end


function profile = getKmaWindProfile(cfg)
    persistent cachedPath cachedProfile

    profile = [];
    if ~isfield(cfg.wind, 'kma_csv')
        return;
    end

    csvPath = char(cfg.wind.kma_csv);
    if isempty(csvPath) || ~isfile(csvPath)
        return;
    end

    if ~isempty(cachedPath) && strcmp(cachedPath, csvPath) && ~isempty(cachedProfile)
        profile = cachedProfile;
        return;
    end

    try
        T = readtable(csvPath);
    catch
        return;
    end

    if isempty(T) || width(T) < 2
        return;
    end

    speedIdx = findColumnIndex(T.Properties.VariableNames, cfg.wind.kma_speed_column, {'wind_speed','speed','ws','windspd'});
    dirIdx = findColumnIndex(T.Properties.VariableNames, cfg.wind.kma_direction_column, {'wind_dir','direction','wd','winddir'});
    if speedIdx < 1 || dirIdx < 1
        return;
    end

    speed = toNumericVector(T{:, speedIdx});
    dir = toNumericVector(T{:, dirIdx});

    mask = isfinite(speed) & isfinite(dir);
    speed = speed(mask);
    dir = dir(mask);
    if isempty(speed)
        return;
    end

    if isfield(cfg.wind, 'kma_speed_scale') && isfinite(cfg.wind.kma_speed_scale)
        speed = speed * cfg.wind.kma_speed_scale;
    end
    if isfield(cfg.wind, 'kma_direction_offset_deg') && isfinite(cfg.wind.kma_direction_offset_deg)
        dir = dir + cfg.wind.kma_direction_offset_deg;
    end

    speed = max(0.0, speed);
    dir = mod(dir + 180.0, 360.0) - 180.0;

    profile = struct('speed', speed(:), 'dir', dir(:));
    cachedPath = csvPath;
    cachedProfile = profile;
end


function idx = findColumnIndex(varNames, preferredName, fallbackNames)
    idx = -1;
    names = lower(string(varNames));

    if ~isempty(preferredName)
        hit = find(names == lower(string(preferredName)), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end

    for i = 1:numel(fallbackNames)
        hit = find(names == lower(string(fallbackNames{i})), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end
end


function [angVelNorm, linAccNorm] = parseImuMetrics(msg)
    angVelNorm = nan;
    linAccNorm = nan;

    try
        av = msg.angular_velocity;
        angVelNorm = sqrt(double(av.x)^2 + double(av.y)^2 + double(av.z)^2);
    catch
    end

    try
        la = msg.linear_acceleration;
        linAccNorm = sqrt(double(la.x)^2 + double(la.y)^2 + double(la.z)^2);
    catch
    end
end


function [hasContact, totalForce, fFL, fFR, fRL, fRR] = parseContactForces(msg)
    hasContact = 0;
    totalForce = nan;
    fFL = nan;
    fFR = nan;
    fRL = nan;
    fRR = nan;

    try
        states = msg.states;
    catch
        return;
    end
    if isempty(states)
        return;
    end

    hasContact = 1;
    totalForce = 0.0;
    fFL = 0.0;
    fFR = 0.0;
    fRL = 0.0;
    fRR = 0.0;

    for i = 1:numel(states)
        st = states(i);
        namePair = "";
        try
            namePair = string(st.collision1_name) + " " + string(st.collision2_name);
        catch
        end
        key = contactArmKey(namePair);

        forceSum = 0.0;
        try
            wrenches = st.wrenches;
            for j = 1:numel(wrenches)
                wj = wrenches(j);
                fx = double(wj.force.x);
                fy = double(wj.force.y);
                fz = double(wj.force.z);
                forceSum = forceSum + sqrt(fx*fx + fy*fy + fz*fz);
            end
        catch
            forceSum = 0.0;
        end

        totalForce = totalForce + forceSum;
        switch key
            case "fl"
                fFL = fFL + forceSum;
            case "fr"
                fFR = fFR + forceSum;
            case "rl"
                fRL = fRL + forceSum;
            case "rr"
                fRR = fRR + forceSum;
        end
    end
end


function key = contactArmKey(nameText)
    s = lower(char(nameText));
    key = "";
    if contains(s, 'front_left') || contains(s, 'left_front') || contains(s, 'arm_fl') || contains(s, 'fl_')
        key = "fl";
    elseif contains(s, 'front_right') || contains(s, 'right_front') || contains(s, 'arm_fr') || contains(s, 'fr_')
        key = "fr";
    elseif contains(s, 'rear_left') || contains(s, 'back_left') || contains(s, 'left_rear') || contains(s, 'arm_rl') || contains(s, 'rl_')
        key = "rl";
    elseif contains(s, 'rear_right') || contains(s, 'back_right') || contains(s, 'right_rear') || contains(s, 'arm_rr') || contains(s, 'rr_')
        key = "rr";
    end
end


function err = parseTagErrorFromBridge(msg)
    % bridge format: [detected, tag_id, cx_px, cy_px, area_px2, margin, num_tags]
    err = nan;
    try
        d = double(msg.data);
        if numel(d) < 4
            return;
        end

        detected = d(1) > 0.5;
        if ~detected
            return;
        end

        cx = d(3);
        cy = d(4);

        w = 640.0;
        h = 480.0;
        u = (cx - w/2.0) / (w/2.0);
        v = (cy - h/2.0) / (h/2.0);
        err = sqrt(u*u + v*v);
    catch
    end
end


function [detected, u, v, err] = parseTagStateFromBridge(msg)
    detected = false;
    u = nan;
    v = nan;
    err = nan;

    try
        d = double(msg.data);
        if numel(d) < 4
            return;
        end

        detected = d(1) > 0.5;
        if ~detected
            return;
        end

        cx = d(3);
        cy = d(4);
        w = 640.0;
        h = 480.0;

        u = (cx - w/2.0) / (w/2.0);
        v = (cy - h/2.0) / (h/2.0);
        err = sqrt(u*u + v*v);
    catch
    end
end


function [hist, count] = pushTagCenter(hist, count, u, v)
    if ~isfinite(u) || ~isfinite(v)
        return;
    end
    hist(1:end-1, :) = hist(2:end, :);
    hist(end, :) = [u, v];
    count = min(size(hist, 1), count + 1);
end


function [ok, uPred, vPred] = predictTagCenterNorm(hist, count, uNow, vNow, tNow, lastDetectT, horizonSec, timeoutSec, samplePeriodSec, minSamples)
    ok = false;
    uPred = nan;
    vPred = nan;

    if isfinite(uNow) && isfinite(vNow)
        uPred = uNow;
        vPred = vNow;
        ok = true;
    end

    if count < max(2, minSamples)
        return;
    end
    if (tNow - lastDetectT) > timeoutSec
        return;
    end

    rows = hist(end-count+1:end, :);
    rows = rows(all(isfinite(rows), 2), :);
    if size(rows, 1) < max(2, minSamples)
        return;
    end

    p2 = rows(end, :);
    p1 = rows(end-1, :);
    dt = max(samplePeriodSec, 1e-3);
    v = (p2 - p1) ./ dt;
    pPred = p2 + v .* horizonSec;

    uPred = pPred(1);
    vPred = pPred(2);
    ok = isfinite(uPred) && isfinite(vPred);
end


function st = initPidState()
    st = struct('integral', 0.0, 'prev_error', 0.0, 'initialized', false);
end


function [u, st] = pidStep(err, dt, st, kp, ki, kd, i_limit, out_limit)
    if ~st.initialized
        st.prev_error = err;
        st.initialized = true;
    end

    st.integral = clampValue(st.integral + err * dt, -abs(i_limit), abs(i_limit));
    derr = (err - st.prev_error) / max(dt, 1e-6);
    st.prev_error = err;

    u = kp * err + ki * st.integral + kd * derr;
    u = clampValue(u, -abs(out_limit), abs(out_limit));
end


function y = clampValue(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end


function x = randRange(a, b)
    lo = min(a, b);
    hi = max(a, b);
    x = lo + rand() * (hi - lo);
end


function viz = initScenarioLivePlots(windowSec, maxPoints)
    viz = struct();
    viz.window_sec = max(8.0, windowSec);
    if nargin < 2 || ~isfinite(maxPoints)
        maxPoints = 500;
    end
    maxPoints = max(100, round(maxPoints));
    viz.fig = figure('Name', 'Auto Landing Pipeline Monitor', 'NumberTitle', 'off');
    tl = tiledlayout(viz.fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    viz.altLine = animatedline(ax1, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4, 'MaximumNumPoints', maxPoints);
    title(ax1, 'Altitude z (m)');
    ylabel(ax1, 'm');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    viz.tagErrLine = animatedline(ax2, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4, 'MaximumNumPoints', maxPoints);
    title(ax2, 'Tag Error (normalized)');
    ylabel(ax2, 'error');
    grid(ax2, 'on');

    ax3 = nexttile(tl, 3);
    viz.cmdXLine = animatedline(ax3, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.3, 'MaximumNumPoints', maxPoints);
    viz.cmdYLine = animatedline(ax3, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.3, 'MaximumNumPoints', maxPoints);
    title(ax3, 'PID Command (cmd\_vel x/y)');
    xlabel(ax3, 'time (s)');
    ylabel(ax3, 'cmd');
    legend(ax3, {'cmd x','cmd y'}, 'Location', 'best');
    grid(ax3, 'on');

    viz.statusText = annotation(viz.fig, 'textbox', [0.66 0.92 0.33 0.07], ...
        'String', '', 'FitBoxToText', 'off', 'EdgeColor', 'none', 'HorizontalAlignment', 'right');
    viz.axes = [ax1, ax2, ax3];
end


function viz = updateScenarioLivePlots(viz, tNow, z, tagErr, cmdX, cmdY, phase, isFlying, tagDetected, predOk)
    if isempty(viz) || ~isfield(viz, 'fig') || ~isgraphics(viz.fig)
        return;
    end

    if isfinite(z)
        addpoints(viz.altLine, tNow, z);
    end
    if isfinite(tagErr)
        addpoints(viz.tagErrLine, tNow, tagErr);
    end
    addpoints(viz.cmdXLine, tNow, cmdX);
    addpoints(viz.cmdYLine, tNow, cmdY);

    xmin = max(0, tNow - viz.window_sec);
    xmax = max(viz.window_sec, tNow);
    for i = 1:numel(viz.axes)
        xlim(viz.axes(i), [xmin, xmax]);
    end

    viz.statusText.String = sprintf('phase=%s | flying=%d | tag=%d | pred=%d | t=%.1fs', char(phase), isFlying, tagDetected, predOk, tNow);
    drawnow limitrate nocallbacks;
end


function [roll,pitch,yaw] = quat2eul_local(qwxyz)
    w = qwxyz(1); x = qwxyz(2); y = qwxyz(3); z = qwxyz(4);
    sinr = 2*(w*x + y*z);
    cosr = 1 - 2*(x*x + y*y);
    roll = atan2(sinr, cosr);

    sinp = 2*(w*y - z*x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * pi/2;
    else
        pitch = asin(sinp);
    end

    siny = 2*(w*z + x*y);
    cosy = 1 - 2*(y*y + z*z);
    yaw = atan2(siny, cosy);
end


function s = shellEscapeDoubleQuotes(x)
    s = strrep(x, '"', '\"');
end


function s = shellEscapeSingleQuotes(x)
    s = strrep(x, '''', '''"''"''');
end


function out = timestampForFile()
    out = datestr(now, 'yyyymmdd_HHMMSS');
end


function v = nanmeanSafe(x)
    x = double(x(:));
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = mean(x);
    end
end


function v = nanstdSafe(x)
    x = double(x(:));
    x = x(isfinite(x));
    if numel(x) < 2
        v = nan;
    else
        v = std(x);
    end
end


function v = nanmaxSafe(x)
    x = double(x(:));
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = max(x);
    end
end


function v = nanlast(x)
    x = double(x(:));
    idx = find(isfinite(x), 1, 'last');
    if isempty(idx)
        v = nan;
    else
        v = x(idx);
    end
end


function [trendLast, oscStd, accelRms] = calcVzMotionMetrics(vzSeq, dt)
    trendLast = nan;
    oscStd = nan;
    accelRms = nan;

    v = double(vzSeq(:));
    valid = isfinite(v);
    if sum(valid) < 3
        return;
    end

    vFill = v;
    vFill(~valid) = 0.0;

    win = max(3, round(1.0 / max(dt, 1e-3)));
    trend = movmean(vFill, win, 'omitnan');

    validTrend = isfinite(trend) & valid;
    if ~any(validTrend)
        return;
    end

    trendLast = nanlast(trend(validTrend));

    resid = v(validTrend) - trend(validTrend);
    oscStd = nanstdSafe(resid);

    trendValid = trend(validTrend);
    if numel(trendValid) >= 2
        accel = diff(trendValid) ./ max(dt, 1e-3);
        accelRms = sqrt(mean(accel.^2));
    end
end


function t = tailVec(x, n)
    x = x(:);
    n = min(numel(x), n);
    if n <= 0
        t = [];
    else
        t = x(end-n+1:end);
    end
end


function clearNode(node)
    try
        clear node;
    catch
    end
end


function tf = isUserInterruptException(ME)
    tf = false;
    try
        id = lower(string(ME.identifier));
    catch
        id = "";
    end
    try
        msg = lower(string(ME.message));
    catch
        msg = "";
    end

    tf = contains(id, "operationterminatedbyuser") || ...
         contains(id, "interrupted") || ...
         contains(msg, "operation terminated by user") || ...
         contains(msg, "interrupt") || ...
         contains(msg, "ctrl+c");

    if tf
        return;
    end

    try
        causes = ME.cause;
        for i = 1:numel(causes)
            if isUserInterruptException(causes{i})
                tf = true;
                return;
            end
        end
    catch
    end
end


function s = emptyScenarioResult()
    s = struct();
    s.scenario_id = -1;
    s.label = "unstable";
    s.success = false;
    s.failure_reason = "not_run";
    s.exception_message = "";

    s.tag_lock_acquired = false;
    s.landing_cmd_time = nan;
    s.random_bias_x = nan;
    s.random_bias_y = nan;

    s.hover_height_cmd = nan;
    s.landing_height_m = nan;

    s.wind_speed_cmd = nan;
    s.wind_dir_cmd = nan;

    s.mean_wind_speed = nan;
    s.max_wind_speed = nan;
    s.mean_abs_roll_deg = nan;
    s.mean_abs_pitch_deg = nan;
    s.mean_abs_vz = nan;
    s.max_abs_vz = nan;
    s.mean_tag_error = nan;
    s.max_tag_error = nan;

    s.final_altitude = nan;
    s.final_abs_speed = nan;
    s.final_abs_roll_deg = nan;
    s.final_abs_pitch_deg = nan;
    s.final_tag_error = nan;

    s.stability_std_z = nan;
    s.stability_std_vz = nan;
    s.stability_std_vz_osc = nan;
    s.touchdown_accel_rms = nan;
    s.contact_count = nan;
    s.mean_imu_ang_vel = nan;
    s.max_imu_ang_vel = nan;
    s.mean_imu_lin_acc = nan;
    s.max_imu_lin_acc = nan;
    s.max_contact_force = nan;
    s.arm_force_fl_mean = nan;
    s.arm_force_fr_mean = nan;
    s.arm_force_rl_mean = nan;
    s.arm_force_rr_mean = nan;
    s.arm_force_imbalance = nan;
    s.final_state = nan;

    s.launch_pid = -1;
    s.launch_log = "";
end
