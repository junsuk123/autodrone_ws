% AutoSim.m
% Integrated autonomous simulation + incremental learning pipeline.
% It combines online landing decision, per-scenario validation, and model updates.
%
% Workflow:
% 1) Load latest model if present. If no model exists, start cold and bootstrap from collected labels.
% 2) For each scenario: launch -> collect sensor state -> build semantic states/relations -> infer landing feasibility -> decide land/abort.
% 3) Incrementally retrain model with accumulated dataset and save a new model snapshot.
% 4) Use updated model from the next scenario onward.
% 5) On user interrupt, save current plots/model/dataset/checkpoint and exit safely.
%
% Usage:
%   run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')

clear; clc; close all;

thisDir = fileparts(mfilename('fullpath'));
if ~isempty(thisDir)
    addpath(thisDir);
end

autosimClearStopRequest();

cfg = autosimDefaultConfig();
autosimEnsureDirectories(cfg);
lockCleanup = autosimAcquireLock(cfg); %#ok<NASGU>

fprintf('\n[AUTOSIM] Start at %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('[AUTOSIM] Scenario count: %d\n', cfg.scenario.count);

results = repmat(autosimEmptyScenarioResult(), 0, 1);
runStatus = "completed";
runError = [];

traceStore = table();
learningHistory = table();
plotState = autosimInitPlots();
model = autosimCreatePlaceholderModel(cfg, 'pre_init');

try
    [model, modelInfo] = autosimLoadOrInitModel(cfg);
    fprintf('[AUTOSIM] Initial model source: %s\n', modelInfo.source);

    for scenarioId = 1:cfg.scenario.count
        if autosimIsStopRequested()
            runStatus = "interrupted";
            fprintf('[AUTOSIM] Stop requested before scenario start: %s\n', autosimGetStopReason());
            break;
        end

        fprintf('\n[AUTOSIM] Scenario %d/%d\n', scenarioId, cfg.scenario.count);

        datasetState = autosimAnalyzeDatasetState(results, traceStore, learningHistory);
        scenarioPolicy = autosimChooseScenarioPolicy(cfg, datasetState, scenarioId);
        scenarioCfg = autosimBuildAdaptiveScenarioConfig(cfg, scenarioId, scenarioPolicy, datasetState);

        fprintf('[AUTOSIM] Adaptive policy=%s | stable=%.2f unstable=%.2f boundary=%.2f unsafeRate=%.2f\n', ...
            scenarioPolicy.mode, datasetState.stableRatio, 1.0 - datasetState.stableRatio, ...
            datasetState.boundarySampleRatio, datasetState.unsafeLandingRate);

        autosimCleanupProcesses(cfg);
        pause(cfg.process.kill_settle_sec);

        launchInfo = autosimStartLaunch(cfg, scenarioCfg, scenarioId);

        try
            pause(cfg.launch.warmup_sec);
            [scenarioResult, scenarioTrace] = autosimRunScenario(cfg, scenarioCfg, scenarioId, model);
            scenarioResult.launch_pid = launchInfo.pid;
            scenarioResult.launch_log = launchInfo.log_file;
        catch ME
            scenarioResult = autosimEmptyScenarioResult();
            scenarioResult.scenario_id = scenarioId;
            if exist('scenarioCfg', 'var') && isfield(scenarioCfg, 'policy_mode')
                scenarioResult.scenario_policy = string(scenarioCfg.policy_mode);
            end
            scenarioResult.label = "unstable";
            scenarioResult.success = false;
            if autosimIsUserInterrupt(ME)
                scenarioResult.failure_reason = "user_interrupt";
                runStatus = "interrupted";
            else
                scenarioResult.failure_reason = "runtime_exception";
            end
            scenarioResult.exception_message = string(ME.message);
            scenarioTrace = autosimEmptyTraceTable(scenarioId);
            if exist('scenarioCfg', 'var') && isfield(scenarioCfg, 'policy_mode')
                scenarioTrace.scenario_policy = string(scenarioCfg.policy_mode);
            end
            warning('[AUTOSIM] Scenario %d exception: %s', scenarioId, ME.message);
        end

        results(end+1,1) = scenarioResult; %#ok<SAGROW>
        traceStore = [traceStore; scenarioTrace]; %#ok<AGROW>

        [model, learnInfo] = autosimIncrementalTrainAndSave(cfg, results, model, scenarioId);
        learningHistory = [learningHistory; struct2table(learnInfo)]; %#ok<AGROW>

        plotState = autosimUpdatePlots(plotState, results, learningHistory);

        autosimSaveCheckpoint(cfg, results, traceStore, learningHistory, model, runStatus, "scenario_end");
        autosimPrintStats(results, scenarioId, cfg.scenario.count, learnInfo);

        if cfg.process.stop_after_each_scenario
            autosimCleanupProcesses(cfg, launchInfo.pid);
            pause(cfg.process.kill_settle_sec);
        end

        if autosimIsStopRequested()
            runStatus = "interrupted";
            fprintf('[AUTOSIM] Stop requested after scenario %d: %s\n', scenarioId, autosimGetStopReason());
        end

        if runStatus == "interrupted"
            fprintf('[AUTOSIM] User interrupt detected. Stop after %d scenario(s).\n', numel(results));
            break;
        end
    end
catch ME
    if autosimIsUserInterrupt(ME)
        runStatus = "interrupted";
        warning('[AUTOSIM] Interrupted by user: %s', ME.message);
    else
        runStatus = "failed";
        runError = ME;
        warning('[AUTOSIM] Fatal exception: %s', ME.message);
    end
end
%%
autosimCleanupProcesses(cfg);
pause(cfg.process.kill_settle_sec);

finalInfo = autosimFinalize(cfg, results, traceStore, learningHistory, model, plotState, runStatus);

if finalInfo.hasValidLabel
    fprintf('[AUTOSIM] Final stable ratio: %.1f%% (%d/%d)\n', ...
        100.0 * finalInfo.stableRatio, finalInfo.nStable, finalInfo.nValid);
end

fprintf('[AUTOSIM] Completed at %s (status=%s)\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), runStatus);

autosimClearStopRequest();

if ~isempty(runError)
    rethrow(runError);
end


function cfg = autosimDefaultConfig()
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
    cfg.paths.root = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab';
    cfg.paths.run_id = datestr(now, 'yyyymmdd_HHMMSS');
    cfg.paths.data_root = fullfile(cfg.paths.root, 'data');
    cfg.paths.log_root = fullfile(cfg.paths.root, 'logs');
    cfg.paths.plot_root = fullfile(cfg.paths.root, 'plots');
    cfg.paths.data_dir = fullfile(cfg.paths.data_root, cfg.paths.run_id);
    cfg.paths.model_dir = fullfile(cfg.paths.root, 'models');
    cfg.paths.plot_dir = fullfile(cfg.paths.plot_root, cfg.paths.run_id);
    cfg.paths.log_dir = fullfile(cfg.paths.log_root, cfg.paths.run_id);
    cfg.paths.lock_file = fullfile(cfg.paths.data_root, 'autosim.lock');
    cfg.paths.bringup_py_src = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/sjtu_drone_bringup';
    cfg.paths.bringup_py_install = '/home/j/INCSL/IICC26_ws/install/sjtu_drone_bringup/lib/python3.10/site-packages';
    cfg.paths.control_py_src = '/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/sjtu_drone_control';
    cfg.paths.control_py_install = '/home/j/INCSL/IICC26_ws/install/sjtu_drone_control/lib/python3.10/site-packages';

    pyPathChain = [ ...
        'export PYTHONPATH=' cfg.paths.control_py_install ':' cfg.paths.control_py_src ':' ...
        cfg.paths.bringup_py_install ':' cfg.paths.bringup_py_src ':$PYTHONPATH' ...
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
        'use_gui:=false use_rviz:=true use_teleop:=false controller:=joystick ' ...
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
    cfg.scenario.count = 200;
    cfg.scenario.duration_sec = inf;
    cfg.scenario.sample_period_sec = 0.2;
    cfg.scenario.post_land_observe_sec = 3.0;
    cfg.scenario.early_stop_after_landing_sec = 1.5;
    cfg.scenario.hover_height_min_m = 1.5;
    cfg.scenario.hover_height_max_m = 2.5;

    cfg.wind = struct();
    cfg.wind.enable = true;
    cfg.wind.update_period_sec = 0.1;
    cfg.wind.speed_min = 0.0;
    cfg.wind.speed_max = 3.0;
    cfg.wind.direction_min = -180.0;
    cfg.wind.direction_max = 180.0;
    cfg.wind.start_delay_after_hover_sec = 0.0;
    cfg.wind.start_require_tag_centered = true;
    cfg.wind.start_tag_center_hold_sec = 0.0;
    cfg.wind.start_force_after_hover_sec = 0.0;
    cfg.wind.model_ramp_sec = 0.0;
    cfg.wind.model_gust_amp_ratio = 0.25;
    cfg.wind.model_gust_freq_hz = 0.18;
    cfg.wind.model_noise_std_speed = 0.15;
    cfg.wind.model_dir_noise_std_deg = 2.5;
    cfg.wind.model_dir_osc_amp_deg = 10.0;
    cfg.wind.model_dir_osc_freq_hz = 0.09;
    cfg.wind.source = "kma_csv"; % "kma_csv" | "random"
    cfg.wind.kma_csv = fullfile(cfg.paths.data_root, 'OBS_ASOS_MI_20260309231031.csv');
    cfg.wind.kma_time_column = 'time';
    cfg.wind.kma_speed_column = 'wind_speed';
    cfg.wind.kma_direction_column = 'wind_dir';
    cfg.wind.kma_speed_scale = 1.0;
    cfg.wind.kma_direction_offset_deg = 0.0;
    cfg.wind.kma_interp_noise_std_speed = 0.10;
    cfg.wind.kma_interp_noise_std_dir_deg = 1.5;
    cfg.wind.kma_use_profile_direct = true;

    cfg.control = struct();
    cfg.control.takeoff_retry_sec = 1.0;
    cfg.control.hover_settle_sec = 3.0;
    cfg.control.flying_altitude_threshold = 2.5;
    cfg.control.xy_kp = 1.75;
    cfg.control.xy_ki = 0.0;
    cfg.control.xy_kd = 1.75;
    cfg.control.xy_i_limit = 1.0;
    cfg.control.xy_cmd_limit = 0.7;
    cfg.control.target_u = 0.0;
    cfg.control.target_v = 0.0;
    cfg.control.publish_cmd_always = true;
    cfg.control.xy_map_sign_x_from_v = 1.0;
    cfg.control.xy_map_sign_y_from_u = 1.0;
    cfg.control.tag_center_deadband = 0.005;
    cfg.control.tag_hold_timeout_sec = 0.6;
    cfg.control.tag_predict_horizon_sec = 0.15;
    cfg.control.tag_predict_timeout_sec = 0.6;
    cfg.control.tag_history_len = 20;
    cfg.control.tag_min_predict_samples = 2;
    cfg.control.pre_takeoff_require_tag_centered = false;
    cfg.control.pre_takeoff_tag_center_tolerance = 0.005;
    cfg.control.pre_takeoff_tag_center_hold_sec = 1.0;
    cfg.control.search_enable_spiral = true;
    cfg.control.search_spiral_cmd_max = 0.30;
    cfg.control.search_spiral_growth_per_sec = 0.08;
    cfg.control.search_spiral_omega_rad_sec = 1.2;
    cfg.control.search_spiral_start_radius = 0.04;
    cfg.control.pose_hold_enable = true;
    cfg.control.pose_hold_kp = 0.45;
    cfg.control.pose_hold_cmd_limit = 0.35;
    cfg.control.land_cmd_alt_m = 0.22;
    cfg.control.land_forced_timeout_sec = 30.0;

    cfg.agent = struct();
    cfg.agent.enable_model_decision = true;
    cfg.agent.semantic_only_mode = false;
    cfg.agent.semantic_land_threshold = 0.70;
    cfg.agent.semantic_abort_threshold = 0.40;
    cfg.agent.prob_land_threshold = 0.50;
    cfg.agent.min_samples_before_decision = 8;
    cfg.agent.min_hover_eval_sec = 3.0;
    cfg.agent.min_altitude_before_land = 0.10;
    cfg.agent.max_tag_error_before_land = 0.70;
    cfg.agent.decision_cooldown_sec = 0.5;
    cfg.agent.block_landing_if_unstable = true;
    cfg.agent.freeze_xy_if_unstable = false;
    cfg.agent.no_model_fallback_enable = true;
    cfg.agent.no_model_min_samples_before_land = 6;
    cfg.agent.no_model_max_tag_error = 0.90;
    cfg.agent.no_model_eval_window_sec = 1.8;
    cfg.agent.no_model_max_abs_vz = 0.90;
    cfg.agent.no_model_max_z_osc_std = 0.30;
    cfg.agent.no_model_max_z_flip_rate_hz = 4.0;
    cfg.agent.no_model_max_xy_std = 0.24;
    cfg.agent.no_model_max_xy_speed_rms = 0.55;
    cfg.agent.no_model_max_xy_radius = 0.55;
    cfg.agent.no_model_max_abs_roll_pitch_deg = 35.0;
    cfg.agent.no_model_max_wind_speed = 10.0;
    cfg.agent.no_model_require_semantic_safe = false;
    cfg.agent.model_min_total_samples_for_use = 10;
    cfg.agent.model_min_class_samples_for_use = 1;
    cfg.agent.model_minority_ratio_for_use = 0.05;

    cfg.learning = struct();
    cfg.learning.enable = true;
    cfg.learning.save_every_scenario = false;
    cfg.learning.update_every_n_scenarios = 2;
    cfg.learning.min_scenarios_before_first_update = 4;
    cfg.learning.bootstrap_min_samples = 1;
    cfg.learning.min_stable_samples_for_update = 2;
    cfg.learning.min_unstable_samples_for_update = 1;
    cfg.learning.minority_ratio_floor_for_update = 0.05;
    cfg.learning.force_update_after_stale_scenarios = 15;
    cfg.learning.tag_lock_error_max = 0.16;
    cfg.learning.tag_lock_hold_sec = 1.2;
    cfg.learning.random_landing_wait_min_sec = 1.0;
    cfg.learning.random_landing_wait_max_sec = 4.0;
    cfg.learning.random_cmd_duration_min_sec = 1.0;
    cfg.learning.random_cmd_duration_max_sec = 3.0;
    cfg.learning.random_xy_cmd_max = 0.35;

    cfg.adaptive = struct();
    cfg.adaptive.enable = true;
    cfg.adaptive.warmup_scenarios = 6;
    cfg.adaptive.target_unstable_ratio = 0.45;
    cfg.adaptive.target_boundary_ratio = 0.25;
    cfg.adaptive.boundary_margin_context = 0.10;
    cfg.adaptive.boundary_margin_prob = 0.10;
    cfg.adaptive.recent_window = 20;
    cfg.adaptive.base_exploit_prob = 0.60;
    cfg.adaptive.base_boundary_prob = 0.25;
    cfg.adaptive.base_hard_negative_prob = 0.15;
    cfg.adaptive.max_boundary_prob = 0.45;
    cfg.adaptive.max_hard_negative_prob = 0.40;
    cfg.adaptive.safe_probe_ratio_boost = 0.06;

    cfg.adaptive.exploit = struct();
    cfg.adaptive.exploit.wind_scale = 0.85;
    cfg.adaptive.exploit.gust_amp_scale = 0.85;
    cfg.adaptive.exploit.dir_osc_scale = 0.85;
    cfg.adaptive.exploit.hover_bias_m = 0.0;

    cfg.adaptive.boundary = struct();
    cfg.adaptive.boundary.wind_center_scale = 0.72;
    cfg.adaptive.boundary.wind_span_scale = 0.10;
    cfg.adaptive.boundary.gust_amp_scale = 1.00;
    cfg.adaptive.boundary.dir_osc_scale = 1.05;
    cfg.adaptive.boundary.hover_bias_m = 0.0;

    cfg.adaptive.hard_negative = struct();
    cfg.adaptive.hard_negative.wind_scale = 1.18;
    cfg.adaptive.hard_negative.wind_min_scale = 0.95;
    cfg.adaptive.hard_negative.gust_amp_scale = 1.35;
    cfg.adaptive.hard_negative.dir_osc_scale = 1.25;
    cfg.adaptive.hard_negative.hover_bias_m = -0.10;

    cfg.persistence = struct();
    cfg.persistence.checkpoint_mat = fullfile(cfg.paths.data_dir, 'autosim_checkpoint_latest.mat');
    cfg.persistence.checkpoint_csv = fullfile(cfg.paths.data_dir, 'autosim_dataset_latest.csv');
    cfg.persistence.trace_csv = fullfile(cfg.paths.data_dir, 'autosim_trace_latest.csv');

    cfg.process = struct();
    cfg.process.stop_after_each_scenario = true;
    cfg.process.kill_settle_sec = 2.0;
    cfg.process.cleanup_verify_timeout_sec = 8.0;

    cfg.thresholds = struct();
    cfg.thresholds.land_state_value = 0;
    cfg.thresholds.landed_altitude_max_m = 0.60;
    cfg.thresholds.final_speed_max_mps = 0.90;
    cfg.thresholds.final_attitude_max_deg = 35.0;
    cfg.thresholds.final_tag_error_max = 0.90;
    cfg.thresholds.final_stability_std_z_max = 0.45;
    cfg.thresholds.final_stability_std_vz_max = 0.55;
    cfg.thresholds.final_touchdown_vz_osc_max = 1.45;
    cfg.thresholds.final_touchdown_accel_rms_max = 3.30;
    cfg.thresholds.final_touchdown_abs_vz_max = 6.70;
    cfg.thresholds.final_imu_ang_vel_rms_max = 6.0;
    cfg.thresholds.final_imu_lin_acc_rms_max = 11.0;
    cfg.thresholds.final_contact_force_max_n = 50.0;
    cfg.thresholds.final_arm_force_imbalance_max_n = 40.0;

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

    cfg.ontology = struct();
    cfg.ontology.landing_area_size = [3.0, 3.0];
    cfg.ontology.obstacle_presence = false;
    cfg.ontology.tag_jitter_warn_px = 8.0;
    cfg.ontology.tag_jitter_unsafe_px = 20.0;
    cfg.ontology.tag_stability_score_warn = 0.65;
    cfg.ontology.tag_min_samples = 5;
    cfg.ontology.wind_condition_window_sec = 1.0;
    cfg.ontology.gust_base_window_sec = 6.0;
    cfg.ontology.gust_burst_window_sec = 0.8;
    cfg.ontology.wind_caution_speed = 0.55 * cfg.wind.speed_max;
    cfg.ontology.wind_unsafe_speed = 0.90 * cfg.wind.speed_max;
    cfg.ontology.gust_delta_min = 0.8;
    cfg.ontology.gust_delta_high = 2.0;
    cfg.ontology.gust_dvdt_min = 1.2;
    cfg.ontology.gust_dvdt_high = 3.0;
    cfg.ontology.semantic_feature_names = [ ...
        "wind_speed", "wind_dir_norm", "roll_abs", "pitch_abs", ...
        "tag_u", "tag_v", "jitter", "stability_score", ...
        "wind_risk_enc", "alignment_enc", "visual_enc", "context_enc" ...
    ];

    % Ontology-AI fusion: ontology concepts are preserved, but concept scores are
    % co-estimated by a lightweight AI branch and fused with rule-based scores.
    cfg.ontology_ai = struct();
    cfg.ontology_ai.enable = true;
    cfg.ontology_ai.rule_weight = 0.60; % fused = rule_weight*rule + (1-rule_weight)*ai

    % wind_risk AI branch weights (features defined in autosimOntologyReasoning)
    cfg.ontology_ai.wind_w = [1.20, 1.35, 0.45, 0.20, -0.15, 0.35, -0.30];
    cfg.ontology_ai.wind_b = -1.05;

    % alignment AI branch weights
    cfg.ontology_ai.align_w = [-1.45, -0.95, 1.00, 0.35, -0.30, -0.20];
    cfg.ontology_ai.align_b = 0.10;

    % visual AI branch weights
    cfg.ontology_ai.visual_w = [1.30, -1.10, 1.00, 0.90, -0.25];
    cfg.ontology_ai.visual_b = -0.15;

    % context AI branch weights
    cfg.ontology_ai.context_w = [-1.30, 1.05, 0.95, 0.70, -1.10];
    cfg.ontology_ai.context_b = 0.40;

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

    cfg.ros = struct();
    cfg.ros.enable_imu_subscription = true;
    cfg.ros.enable_bumper_subscription = true;

    cfg.shell = struct();
    cfg.shell.setup_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        pyPathChain ...
    ];
    cfg.shell.land_cli_cmd = [ ...
        ros2env ' && source ~/.bashrc && ' ...
        'source /opt/ros/humble/setup.bash && ' ...
        'source /home/j/INCSL/IICC26_ws/install/setup.bash && ' ...
        pyPathChain ' && ' ...
        'ros2 topic pub /drone/land std_msgs/msg/Empty {} --once' ...
    ];
end


function autosimEnsureDirectories(cfg)
    dirs = {cfg.paths.data_root, cfg.paths.log_root, cfg.paths.plot_root, cfg.paths.data_dir, cfg.paths.model_dir, cfg.paths.plot_dir, cfg.paths.log_dir};
    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir')
            mkdir(dirs{i});
        end
    end
end


function lockCleanup = autosimAcquireLock(cfg)
    lockPath = cfg.paths.lock_file;

    if isfile(lockPath)
        try
            oldPid = str2double(strtrim(fileread(lockPath)));
        catch
            oldPid = nan;
        end

        if isfinite(oldPid) && oldPid > 1
            [st, ~] = system(sprintf('bash -i -c "kill -0 %d >/dev/null 2>&1"', round(oldPid)));
            if st == 0
                error('Another AutoSim instance is running (pid=%d). Stop it first.', round(oldPid));
            end
        end
    end

    thisPid = feature('getpid');
    fid = fopen(lockPath, 'w');
    if fid < 0
        error('Failed to create lock file: %s', lockPath);
    end
    fprintf(fid, '%d\n', round(thisPid));
    fclose(fid);

    lockCleanup = onCleanup(@() autosimReleaseLock(lockPath));
end


function autosimReleaseLock(lockPath)
    try
        if isfile(lockPath)
            delete(lockPath);
        end
    catch
    end
end


function [model, info] = autosimLoadOrInitModel(cfg)
    dd = dir(fullfile(cfg.paths.model_dir, 'autosim_model_*.mat'));
    if isempty(dd)
        dd = dir(fullfile(cfg.paths.model_dir, 'landing_model_*.mat'));
    end

    if isempty(dd)
        model = autosimCreatePlaceholderModel(cfg, 'cold_start');
        info = struct('source', "cold_start_placeholder");
        return;
    end

    [~, idx] = max([dd.datenum]);
    modelPath = fullfile(dd(idx).folder, dd(idx).name);
    S = load(modelPath);
    if isfield(S, 'model')
        model = S.model;
    else
        error('Model file does not contain model variable: %s', modelPath);
    end

    if ~isfield(model, 'feature_names')
        model.feature_names = cfg.model.feature_names;
    end
    info = struct('source', string(modelPath));
end


function model = autosimCreatePlaceholderModel(cfg, reason)
    nFeat = numel(cfg.model.feature_names);
    model = struct();
    model.kind = "gaussian_nb";
    model.class_names = ["stable"; "unstable"];
    model.feature_names = cfg.model.feature_names;
    model.mu = zeros(2, nFeat);
    model.sigma2 = ones(2, nFeat);
    model.prior = [0.5; 0.5];
    model.placeholder = true;
    model.placeholder_reason = string(reason);
    model.created_at = string(datetime('now'));
end


function scenarioCfg = autosimBuildScenarioConfig(cfg, scenarioId)
    scenarioCfg = struct();
    scenarioCfg.id = scenarioId;
    scenarioCfg.policy_mode = "exploit";
    scenarioCfg.policy_reason = "base";
    scenarioCfg.policy_weight_exploit = nan;
    scenarioCfg.policy_weight_boundary = nan;
    scenarioCfg.policy_weight_hard_negative = nan;
    scenarioCfg.boundary_hint = false;
    scenarioCfg.hard_negative_hint = false;
    scenarioCfg.safe_probe_ratio_boost = 0.0;
    scenarioCfg.hover_height_m = autosimRandRange(cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
    scenarioCfg.wind_profile_offset_sec = 0;
    if cfg.wind.enable
        [scenarioCfg.wind_speed, scenarioCfg.wind_dir] = autosimPickScenarioWind(cfg, scenarioId);
        scenarioCfg.wind_profile_offset_sec = autosimPickScenarioWindOffsetSec(cfg, scenarioId);
    else
        scenarioCfg.wind_speed = 0.0;
        scenarioCfg.wind_dir = 0.0;
    end
end


function state = autosimAnalyzeDatasetState(results, traceStore, learningHistory)
    if nargin < 1 || isempty(results)
        summaryTbl = table();
    else
        summaryTbl = autosimSummaryTable(results);
    end

    dEval = autosimEvaluateDecisionMetrics(summaryTbl);

    state = struct();
    state.nStable = 0;
    state.nUnstable = 0;
    state.stableRatio = 0.5;
    state.decisionAccuracy = autosimClampNaN(dEval.accuracy, 0.0);
    state.unsafeLandingRate = autosimClampNaN(dEval.unsafe_landing_rate, 0.0);
    state.falseNegativeRate = autosimSafeDivide(dEval.fn, dEval.fn + dEval.tp);
    state.boundarySampleRatio = 0.0;
    state.recentUnstableRatio = 0.5;
    state.tp = dEval.tp;
    state.fp = dEval.fp;
    state.fn = dEval.fn;
    state.tn = dEval.tn;
    state.nValid = dEval.n_valid;

    if isempty(summaryTbl)
        return;
    end

    if ismember('label', summaryTbl.Properties.VariableNames)
        lbl = string(summaryTbl.label);
        state.nStable = sum(lbl == "stable");
        state.nUnstable = sum(lbl == "unstable");
        nAll = state.nStable + state.nUnstable;
        if nAll > 0
            state.stableRatio = state.nStable / nAll;
        end

        w = min(20, numel(lbl));
        if w > 0
            recent = lbl(max(1, end-w+1):end);
            state.recentUnstableRatio = sum(recent == "unstable") / numel(recent);
        end
    end

    boundaryByContext = false(height(summaryTbl), 1);
    boundaryByProb = false(height(summaryTbl), 1);
    if ~isempty(traceStore) && ismember('scenario_id', traceStore.Properties.VariableNames)
        if ismember('sem_context_enc', traceStore.Properties.VariableNames)
            ids = summaryTbl.scenario_id;
            contextMeans = nan(height(summaryTbl), 1);
            for i = 1:height(summaryTbl)
                sid = ids(i);
                v = traceStore.sem_context_enc(traceStore.scenario_id == sid);
                v = v(isfinite(v));
                if ~isempty(v)
                    contextMeans(i) = mean(v);
                end
            end
            boundaryByContext = isfinite(contextMeans) & (abs(contextMeans - 0.5) <= 0.10);
        end

        if ismember('pred_stable_prob', traceStore.Properties.VariableNames)
            ids = summaryTbl.scenario_id;
            probMeans = nan(height(summaryTbl), 1);
            for i = 1:height(summaryTbl)
                sid = ids(i);
                v = traceStore.pred_stable_prob(traceStore.scenario_id == sid);
                v = v(isfinite(v));
                if ~isempty(v)
                    probMeans(i) = mean(v);
                end
            end
            boundaryByProb = isfinite(probMeans) & (abs(probMeans - 0.5) <= 0.10);
        end
    end

    boundaryMask = boundaryByContext | boundaryByProb;
    if any(boundaryMask)
        state.boundarySampleRatio = sum(boundaryMask) / numel(boundaryMask);
    elseif state.nValid > 0
        % Fallback: no trace boundary signal, approximate by uncertain outcomes.
        state.boundarySampleRatio = min(1.0, (state.fp + state.fn) / state.nValid);
    end

    if nargin >= 3 && ~isempty(learningHistory) && ismember('stable_ratio', learningHistory.Properties.VariableNames)
        r = learningHistory.stable_ratio(end);
        if isfinite(r)
            state.stableRatio = r;
        end
    end
end


function policy = autosimChooseScenarioPolicy(cfg, datasetState, scenarioId)
    policy = struct();
    policy.mode = "exploit";
    policy.reason = "default";
    policy.p_exploit = autosimClampNaN(cfg.adaptive.base_exploit_prob, 0.60);
    policy.p_boundary = autosimClampNaN(cfg.adaptive.base_boundary_prob, 0.25);
    policy.p_hard_negative = autosimClampNaN(cfg.adaptive.base_hard_negative_prob, 0.15);

    if ~isfield(cfg, 'adaptive') || ~cfg.adaptive.enable || scenarioId <= cfg.adaptive.warmup_scenarios
        return;
    end

    targetUnstable = autosimClampNaN(cfg.adaptive.target_unstable_ratio, 0.45);
    targetBoundary = autosimClampNaN(cfg.adaptive.target_boundary_ratio, 0.25);

    unstableRatio = 1.0 - autosimClampNaN(datasetState.stableRatio, 0.5);
    boundaryRatio = autosimClampNaN(datasetState.boundarySampleRatio, 0.0);
    unsafeRate = autosimClampNaN(datasetState.unsafeLandingRate, 0.0);
    fnRate = autosimClampNaN(datasetState.falseNegativeRate, 0.0);

    if unstableRatio < targetUnstable
        d = min(0.25, targetUnstable - unstableRatio);
        policy.p_hard_negative = min(cfg.adaptive.max_hard_negative_prob, policy.p_hard_negative + d);
        policy.p_exploit = max(0.25, policy.p_exploit - 0.5 * d);
        policy.reason = "unstable_shortage";
    end

    if boundaryRatio < targetBoundary
        d = min(0.25, targetBoundary - boundaryRatio);
        policy.p_boundary = min(cfg.adaptive.max_boundary_prob, policy.p_boundary + d);
        policy.p_exploit = max(0.20, policy.p_exploit - 0.5 * d);
        if policy.reason == "default"
            policy.reason = "boundary_shortage";
        else
            policy.reason = policy.reason + "+boundary_shortage";
        end
    end

    if unsafeRate > 0.20
        policy.p_boundary = min(cfg.adaptive.max_boundary_prob, policy.p_boundary + 0.12);
        policy.p_exploit = max(0.20, policy.p_exploit - 0.08);
        if policy.reason == "default"
            policy.reason = "unsafe_fp_high";
        else
            policy.reason = policy.reason + "+unsafe_fp_high";
        end
    end

    if fnRate > 0.25
        policy.p_boundary = min(cfg.adaptive.max_boundary_prob, policy.p_boundary + 0.08);
        if policy.reason == "default"
            policy.reason = "fn_high";
        else
            policy.reason = policy.reason + "+fn_high";
        end
    end

    p = [policy.p_exploit, policy.p_boundary, policy.p_hard_negative];
    p = max(0.0, p);
    s = sum(p);
    if s <= 0
        p = [0.60, 0.25, 0.15];
    else
        p = p ./ s;
    end

    policy.p_exploit = p(1);
    policy.p_boundary = p(2);
    policy.p_hard_negative = p(3);

    r = rand();
    if r < p(3)
        policy.mode = "hard_negative";
    elseif r < (p(3) + p(2))
        policy.mode = "boundary_validation";
    else
        policy.mode = "exploit";
    end
end


function scenarioCfg = autosimBuildAdaptiveScenarioConfig(cfg, scenarioId, policyMode, datasetState)
    scenarioCfg = autosimBuildScenarioConfig(cfg, scenarioId);

    if isstruct(policyMode)
        policy = policyMode;
    else
        policy = struct('mode', string(policyMode), 'reason', "external", ...
            'p_exploit', nan, 'p_boundary', nan, 'p_hard_negative', nan);
    end

    scenarioCfg.policy_mode = string(policy.mode);
    scenarioCfg.policy_reason = string(policy.reason);
    scenarioCfg.policy_weight_exploit = autosimClampNaN(policy.p_exploit, nan);
    scenarioCfg.policy_weight_boundary = autosimClampNaN(policy.p_boundary, nan);
    scenarioCfg.policy_weight_hard_negative = autosimClampNaN(policy.p_hard_negative, nan);

    if ~isfield(cfg, 'adaptive') || ~cfg.adaptive.enable
        return;
    end

    switch char(policy.mode)
        case 'hard_negative'
            scenarioCfg.hard_negative_hint = true;
            scenarioCfg.boundary_hint = false;

            wsMin = cfg.adaptive.hard_negative.wind_min_scale * cfg.wind.speed_max;
            wsMax = cfg.adaptive.hard_negative.wind_scale * cfg.wind.speed_max;
            scenarioCfg.wind_speed = autosimClamp(autosimRandRange(wsMin, wsMax), cfg.wind.speed_min, cfg.wind.speed_max);
            scenarioCfg.hover_height_m = autosimClamp(scenarioCfg.hover_height_m + cfg.adaptive.hard_negative.hover_bias_m, ...
                cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
            scenarioCfg.gust_amp_scale = cfg.adaptive.hard_negative.gust_amp_scale;
            scenarioCfg.dir_osc_scale = cfg.adaptive.hard_negative.dir_osc_scale;

        case 'boundary_validation'
            scenarioCfg.hard_negative_hint = false;
            scenarioCfg.boundary_hint = true;

            c = cfg.adaptive.boundary.wind_center_scale * cfg.wind.speed_max;
            span = cfg.adaptive.boundary.wind_span_scale * cfg.wind.speed_max;
            scenarioCfg.wind_speed = autosimClamp(autosimRandRange(c - span, c + span), cfg.wind.speed_min, cfg.wind.speed_max);
            scenarioCfg.hover_height_m = autosimClamp(scenarioCfg.hover_height_m + cfg.adaptive.boundary.hover_bias_m, ...
                cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
            scenarioCfg.gust_amp_scale = cfg.adaptive.boundary.gust_amp_scale;
            scenarioCfg.dir_osc_scale = cfg.adaptive.boundary.dir_osc_scale;

            if autosimClampNaN(datasetState.falseNegativeRate, 0.0) > 0.25
                scenarioCfg.safe_probe_ratio_boost = cfg.adaptive.safe_probe_ratio_boost;
            end

        otherwise
            scenarioCfg.hard_negative_hint = false;
            scenarioCfg.boundary_hint = false;
            scenarioCfg.wind_speed = autosimClamp(cfg.adaptive.exploit.wind_scale * scenarioCfg.wind_speed, cfg.wind.speed_min, cfg.wind.speed_max);
            scenarioCfg.hover_height_m = autosimClamp(scenarioCfg.hover_height_m + cfg.adaptive.exploit.hover_bias_m, ...
                cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
            scenarioCfg.gust_amp_scale = cfg.adaptive.exploit.gust_amp_scale;
            scenarioCfg.dir_osc_scale = cfg.adaptive.exploit.dir_osc_scale;
    end
end


function info = autosimStartLaunch(cfg, scenarioCfg, scenarioId)
    autosimCleanupProcesses(cfg);

    % Never start a new launch if old graph processes are still alive.
    preSnap = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(preSnap)) > 0
        fprintf('[AUTOSIM] Stale process snapshot before launch:\n%s\n', preSnap);
        autosimCleanupProcesses(cfg);
        preSnap2 = autosimGetActiveProcessSnapshot();
        if strlength(strtrim(preSnap2)) > 0
            error('Cleanup did not converge before launch start. Remaining processes:\n%s', preSnap2);
        end
    end

    logFile = fullfile(cfg.paths.log_dir, sprintf('autosim_launch_s%03d_%s.log', scenarioId, autosimTimestamp()));
    launchCmd = sprintf(cfg.launch.command_template, scenarioCfg.hover_height_m);
    escCmd = autosimEscapeDq(launchCmd);
    escLog = autosimEscapeDq(logFile);

    bashCmd = sprintf('bash -i -c "%s > \\\"%s\\\" 2>&1 &"', escCmd, escLog);
    [st, out] = system(bashCmd);
    if st ~= 0
        error('Launch failed: %s', out);
    end

    pid = -1;
    [~, pOut] = system('bash -i -c "pgrep -n -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" || true"');
    tok = regexp(pOut, '(\d+)', 'tokens');
    if ~isempty(tok)
        pid = str2double(tok{end}{1});
    end

    info = struct('pid', pid, 'log_file', string(logFile));
    fprintf('[AUTOSIM] Launch started (pid=%d), hover=%.2f, wind=%.2f@%.1f\n', ...
        pid, scenarioCfg.hover_height_m, scenarioCfg.wind_speed, scenarioCfg.wind_dir);
end


function [res, traceTbl] = autosimRunScenario(cfg, scenarioCfg, scenarioId, model)
    node = ros2node(sprintf('/matlab_autosim_s%03d', scenarioId));
    rosCleanup = []; %#ok<NASGU>

    subState = ros2subscriber(node, cfg.topics.state, 'std_msgs/Int8');
    subPose = ros2subscriber(node, cfg.topics.pose, 'geometry_msgs/Pose');
    subVel = ros2subscriber(node, cfg.topics.vel, 'geometry_msgs/Twist');
    subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray');
    subWind = ros2subscriber(node, cfg.topics.wind_condition, 'std_msgs/Float32MultiArray');
    subImu = [];
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_imu_subscription') && cfg.ros.enable_imu_subscription
        try
            subImu = ros2subscriber(node, cfg.topics.imu, 'sensor_msgs/msg/Imu');
        catch
            subImu = [];
        end
    end
    subBumpers = [];
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'enable_bumper_subscription') && cfg.ros.enable_bumper_subscription
        try
            subBumpers = ros2subscriber(node, cfg.topics.bumpers, 'gazebo_msgs/msg/ContactsState');
        catch
            subBumpers = [];
        end
    end

    pubWind = ros2publisher(node, cfg.topics.wind_command, 'std_msgs/Float32MultiArray');
    pubTakeoff = ros2publisher(node, cfg.topics.takeoff_cmd, 'std_msgs/Empty');
    pubLand = ros2publisher(node, cfg.topics.land_cmd, 'std_msgs/Empty');
    pubCmd = ros2publisher(node, cfg.topics.cmd_vel, 'geometry_msgs/Twist');

    msgWind = ros2message(pubWind);
    msgTakeoff = ros2message(pubTakeoff);
    msgLand = ros2message(pubLand);
    msgCmd = ros2message(pubCmd);

    rosHandles = {msgCmd, msgLand, msgTakeoff, msgWind, ...
        pubCmd, pubLand, pubTakeoff, pubWind, ...
        subBumpers, subImu, subWind, subTag, subVel, subPose, subState, node};
    rosCleanup = onCleanup(@() autosimCleanupRosHandles(rosHandles)); %#ok<NASGU>

    sampleN = 200;

    t = zeros(sampleN,1);
    xPos = nan(sampleN,1);
    yPos = nan(sampleN,1);
    z = nan(sampleN,1);
    vz = nan(sampleN,1);
    speedAbs = nan(sampleN,1);
    rollDeg = nan(sampleN,1);
    pitchDeg = nan(sampleN,1);
    tagErr = nan(sampleN,1);
    windSpeed = nan(sampleN,1);
    windCmdSpeed = nan(sampleN,1);
    windCmdDir = nan(sampleN,1);
    stateVal = nan(sampleN,1);
    contact = zeros(sampleN,1);
    imuAngVel = nan(sampleN,1);
    imuLinAcc = nan(sampleN,1);
    contactForce = nan(sampleN,1);
    armForceFL = nan(sampleN,1);
    armForceFR = nan(sampleN,1);
    armForceRL = nan(sampleN,1);
    armForceRR = nan(sampleN,1);
    predStableProb = nan(sampleN,1);
    decisionTxt = strings(sampleN,1);
    phaseTxt = strings(sampleN,1);
    semanticWindRisk = strings(sampleN,1);
    semanticEnvironment = strings(sampleN,1);
    semanticDroneState = strings(sampleN,1);
    semanticAlign = strings(sampleN,1);
    semanticVisual = strings(sampleN,1);
    semanticContext = strings(sampleN,1);
    semanticRelation = strings(sampleN,1);
    semanticIntegration = strings(sampleN,1);
    semanticSafe = false(sampleN,1);
    landingFeasibility = nan(sampleN,1);
    semFeat = nan(sampleN, numel(cfg.ontology.semantic_feature_names));

    tagHist = nan(cfg.control.tag_history_len, 2);
    tagHistCount = 0;
    lastTagDetectT = -inf;
    lastTagU = nan;
    lastTagV = nan;
    haveLastTag = false;

    histN = max(20, round(8.0 / cfg.scenario.sample_period_sec));
    windSpeedHist = nan(histN, 1);
    windSpeedHistCount = 0;
    windDirHist = nan(histN, 1);
    windDirHistCount = 0;
    tagDetHist = nan(histN, 1);
    tagDetHistCount = 0;

    pidX = autosimPidInit();
    pidY = autosimPidInit();

    lastTakeoffT = -inf;
    lastWindT = -inf;
    lastDecisionT = -inf;
    lastCtrlT = 0.0;

    controlPhase = "pre_takeoff_stabilize";
    phaseEnterT = 0.0;
    hoverStartT = nan;
    decisionEvalStartT = nan;
    hoverCenterHoldStartT = nan;
    preTakeoffCenterHoldStartT = nan;
    windArmed = false;

    tagLockHoldStartT = nan;
    tagLockAcquired = false;
    randomLandingPlanned = false;
    randomLandingStartT = nan;
    randomLandingEndT = nan;
    randomBiasX = 0.0;
    randomBiasY = 0.0;
    tagLostSearchStartT = nan;

    landingSent = false;
    landingSentT = nan;
    landingDecisionMode = "abort";
    executedAction = "abort";
    landedHoldStartT = nan;
    kLast = 0;

    liveViz = autosimInitScenarioRealtimePlot(cfg, scenarioId, scenarioCfg);
    hasTrainedModel = autosimIsModelReliable(model, cfg);
    stopRequested = false;
    stopReason = "";

    t0 = tic;
    k = 0;
    while true
        if autosimIsStopRequested()
            stopRequested = true;
            stopReason = autosimGetStopReason();
            fprintf('[AUTOSIM] Scenario %03d stop requested: %s\n', scenarioId, stopReason);
            break;
        end

        k = k + 1;
        if k > sampleN
            growN = 200;
            t = [t; zeros(growN,1)]; %#ok<AGROW>
            xPos = [xPos; nan(growN,1)]; %#ok<AGROW>
            yPos = [yPos; nan(growN,1)]; %#ok<AGROW>
            z = [z; nan(growN,1)]; %#ok<AGROW>
            vz = [vz; nan(growN,1)]; %#ok<AGROW>
            speedAbs = [speedAbs; nan(growN,1)]; %#ok<AGROW>
            rollDeg = [rollDeg; nan(growN,1)]; %#ok<AGROW>
            pitchDeg = [pitchDeg; nan(growN,1)]; %#ok<AGROW>
            tagErr = [tagErr; nan(growN,1)]; %#ok<AGROW>
            windSpeed = [windSpeed; nan(growN,1)]; %#ok<AGROW>
            windCmdSpeed = [windCmdSpeed; nan(growN,1)]; %#ok<AGROW>
            windCmdDir = [windCmdDir; nan(growN,1)]; %#ok<AGROW>
            stateVal = [stateVal; nan(growN,1)]; %#ok<AGROW>
            contact = [contact; zeros(growN,1)]; %#ok<AGROW>
            imuAngVel = [imuAngVel; nan(growN,1)]; %#ok<AGROW>
            imuLinAcc = [imuLinAcc; nan(growN,1)]; %#ok<AGROW>
            contactForce = [contactForce; nan(growN,1)]; %#ok<AGROW>
            armForceFL = [armForceFL; nan(growN,1)]; %#ok<AGROW>
            armForceFR = [armForceFR; nan(growN,1)]; %#ok<AGROW>
            armForceRL = [armForceRL; nan(growN,1)]; %#ok<AGROW>
            armForceRR = [armForceRR; nan(growN,1)]; %#ok<AGROW>
            predStableProb = [predStableProb; nan(growN,1)]; %#ok<AGROW>
            decisionTxt = [decisionTxt; strings(growN,1)]; %#ok<AGROW>
            phaseTxt = [phaseTxt; strings(growN,1)]; %#ok<AGROW>
            semanticWindRisk = [semanticWindRisk; strings(growN,1)]; %#ok<AGROW>
            semanticEnvironment = [semanticEnvironment; strings(growN,1)]; %#ok<AGROW>
            semanticDroneState = [semanticDroneState; strings(growN,1)]; %#ok<AGROW>
            semanticAlign = [semanticAlign; strings(growN,1)]; %#ok<AGROW>
            semanticVisual = [semanticVisual; strings(growN,1)]; %#ok<AGROW>
            semanticContext = [semanticContext; strings(growN,1)]; %#ok<AGROW>
            semanticRelation = [semanticRelation; strings(growN,1)]; %#ok<AGROW>
            semanticIntegration = [semanticIntegration; strings(growN,1)]; %#ok<AGROW>
            semanticSafe = [semanticSafe; false(growN,1)]; %#ok<AGROW>
            landingFeasibility = [landingFeasibility; nan(growN,1)]; %#ok<AGROW>
            semFeat = [semFeat; nan(growN, numel(cfg.ontology.semantic_feature_names))]; %#ok<AGROW>
            sampleN = sampleN + growN;
        end

        tk = toc(t0);
        kLast = k;
        t(k) = tk;

        if cfg.wind.enable && (tk - lastWindT) >= cfg.wind.update_period_sec
            [wsCmd, wdCmd] = autosimComputeWindCommand(cfg, scenarioCfg, tk, windArmed);
            windCmdSpeed(k) = wsCmd;
            windCmdDir(k) = wdCmd;
            msgWind.data = single([wsCmd, wdCmd]);
            send(pubWind, msgWind);
            lastWindT = tk;
        end

        poseMsg = autosimTryReceive(subPose, 0.01);
        if ~isempty(poseMsg)
            xPos(k) = double(poseMsg.position.x);
            yPos(k) = double(poseMsg.position.y);
            z(k) = double(poseMsg.position.z);
            q = poseMsg.orientation;
            [r, p, ~] = autosimQuat2Eul([q.w, q.x, q.y, q.z]);
            rollDeg(k) = abs(rad2deg(r));
            pitchDeg(k) = abs(rad2deg(p));
        end

        velMsg = autosimTryReceive(subVel, 0.01);
        if ~isempty(velMsg)
            vx = double(velMsg.linear.x);
            vy = double(velMsg.linear.y);
            vz(k) = double(velMsg.linear.z);
            speedAbs(k) = sqrt(vx*vx + vy*vy + vz(k)*vz(k));
        end

        stateMsg = autosimTryReceive(subState, 0.01);
        if ~isempty(stateMsg)
            stateVal(k) = double(stateMsg.data);
        end

        tagMsg = autosimTryReceive(subTag, 0.01);
        tagDetected = false;
        uTag = nan;
        vTag = nan;
        if ~isempty(tagMsg)
            [tagDetected, uTag, vTag, te] = autosimParseTag(tagMsg);
            tagErr(k) = te;
        end

        if tagDetected && isfinite(uTag) && isfinite(vTag)
            [tagHist, tagHistCount] = autosimPushTag(tagHist, tagHistCount, uTag, vTag);
            lastTagU = uTag;
            lastTagV = vTag;
            lastTagDetectT = tk;
            haveLastTag = true;
        elseif haveLastTag && ((tk - lastTagDetectT) <= cfg.control.tag_hold_timeout_sec)
            tagDetected = true;
            uTag = lastTagU;
            vTag = lastTagV;
            tagErr(k) = sqrt(uTag*uTag + vTag*vTag);
        end

        [predOk, uPred, vPred] = autosimPredictTagCenter(tagHist, tagHistCount, uTag, vTag, tk, lastTagDetectT, ...
            cfg.control.tag_predict_horizon_sec, cfg.control.tag_predict_timeout_sec, cfg.scenario.sample_period_sec, cfg.control.tag_min_predict_samples);

        windMsg = autosimTryReceive(subWind, 0.01);
        windSpObs = nan;
        windDirObs = nan;
        if ~isempty(windMsg)
            [windSpObs, windDirObs] = autosimParseWindConditionMsg(windMsg);
            if isfinite(windSpObs)
                windSpeed(k) = windSpObs;
            end
        end

        if ~isempty(subImu)
            imuMsg = autosimTryReceive(subImu, 0.01);
            if ~isempty(imuMsg)
                [imuAngVel(k), imuLinAcc(k)] = autosimParseImuMetrics(imuMsg);
            end
        end

        if ~isempty(subBumpers)
            bumpMsg = autosimTryReceive(subBumpers, 0.01);
            if ~isempty(bumpMsg)
                [contact(k), contactForce(k), armForceFL(k), armForceFR(k), armForceRL(k), armForceRR(k)] = autosimParseContactForces(bumpMsg);
            end
        end

        windSpNow = windSpeed(k);
        if ~isfinite(windSpNow)
            windSpNow = windCmdSpeed(k);
        end
        if ~isfinite(windSpNow)
            windSpNow = scenarioCfg.wind_speed;
        end
        if ~isfinite(windSpNow)
            windSpNow = 0.0;
        end

        windDirNow = windDirObs;
        if ~isfinite(windDirNow)
            windDirNow = windCmdDir(k);
        end
        if ~isfinite(windDirNow)
            windDirNow = scenarioCfg.wind_dir;
        end
        if ~isfinite(windDirNow)
            windDirNow = 0.0;
        end

        rollNowRad = deg2rad(autosimNanLast(rollDeg(1:k)));
        pitchNowRad = deg2rad(autosimNanLast(pitchDeg(1:k)));
        xNow = autosimNanLast(xPos(1:k));
        yNow = autosimNanLast(yPos(1:k));
        vzNow = autosimNanLast(vz(1:k));
        zNow = autosimNanLast(z(1:k));

        tagJitterPx = autosimComputeTagJitterPx(tagHist, tagHistCount, cfg.ontology.tag_min_samples);
        tagStabilityScore = autosimComputeTagStabilityScore(tagJitterPx, cfg.ontology.tag_jitter_warn_px, cfg.ontology.tag_jitter_unsafe_px);
        tagCentered = tagDetected && isfinite(uTag) && isfinite(vTag) && ...
            (sqrt((uTag - cfg.control.target_u)^2 + (vTag - cfg.control.target_v)^2) <= cfg.agent.max_tag_error_before_land);

        [windSpeedHist, windSpeedHistCount] = autosimPushScalarHist(windSpeedHist, windSpeedHistCount, windSpNow);
        [windDirHist, windDirHistCount] = autosimPushScalarHist(windDirHist, windDirHistCount, windDirNow);
        [tagDetHist, tagDetHistCount] = autosimPushScalarHist(tagDetHist, tagDetHistCount, double(tagDetected));

        detWin = max(5, round(2.0 / cfg.scenario.sample_period_sec));
        detCont = autosimNanMean(autosimTail(tagDetHist(1:tagDetHistCount), detWin));

        windObs = struct( ...
            'wind_speed', windSpNow, ...
            'wind_direction', windDirNow, ...
            'wind_speed_hist', windSpeedHist(1:windSpeedHistCount), ...
            'wind_dir_hist', windDirHist(1:windDirHistCount), ...
            'dt', cfg.scenario.sample_period_sec);
        droneObs = struct( ...
            'position', [xNow; yNow; zNow], ...
            'roll', rollNowRad, ...
            'pitch', pitchNowRad, ...
            'velocity', [0.0; 0.0; vzNow]);
        tagObs = struct( ...
            'detected', tagDetected, ...
            'u_norm', uTag, ...
            'v_norm', vTag, ...
            'u_pred', uPred, ...
            'v_pred', vPred, ...
            'jitter_px', tagJitterPx, ...
            'stability_score', tagStabilityScore, ...
            'detection_continuity', detCont, ...
            'centered', tagCentered);

        ontoState = autosimBuildOntologyState(windObs, droneObs, tagObs, cfg);
        semantic = autosimOntologyReasoning(ontoState, cfg);
        semVec = autosimBuildSemanticFeatures(windObs, droneObs, tagObs, semantic, cfg);

        semanticWindRisk(k) = string(semantic.wind_risk);
        semanticEnvironment(k) = string(semantic.environment_state);
        semanticDroneState(k) = string(semantic.drone_state);
        semanticAlign(k) = string(semantic.alignment_state);
        semanticVisual(k) = string(semantic.visual_state);
        semanticContext(k) = string(semantic.landing_context);
        semanticRelation(k) = string(semantic.semantic_relation);
        semanticIntegration(k) = string(semantic.semantic_integration);
        semanticSafe(k) = logical(semantic.isSafeForLanding);
        landingFeasibility(k) = semantic.landing_feasibility;
        semFeat(k, :) = semVec;

        isFlying = false;
        if isfinite(stateVal(k))
            isFlying = (stateVal(k) == 1);
        elseif isfinite(z(k))
            isFlying = z(k) >= cfg.control.flying_altitude_threshold;
        end

        dtCtrl = max(1e-3, tk - lastCtrlT);
        lastCtrlT = tk;

        tagLockReadyNow = false;
        if predOk && isfinite(uPred) && isfinite(vPred)
            e = sqrt((uPred - cfg.control.target_u)^2 + (vPred - cfg.control.target_v)^2);
            tagLockReadyNow = e <= cfg.learning.tag_lock_error_max;
        elseif tagDetected && isfinite(uTag) && isfinite(vTag)
            e = sqrt((uTag - cfg.control.target_u)^2 + (vTag - cfg.control.target_v)^2);
            tagLockReadyNow = e <= cfg.learning.tag_lock_error_max;
        end

        cmdX = 0.0;
        cmdY = 0.0;

        if landingSent
            controlPhase = "landing_observe";
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
                    if ~isFlying && ((tk - lastTakeoffT) >= cfg.control.takeoff_retry_sec)
                        send(pubTakeoff, msgTakeoff);
                        lastTakeoffT = tk;
                    end
                    if isFlying
                        controlPhase = "hover_settle";
                        phaseEnterT = tk;
                        hoverStartT = tk;
                        decisionEvalStartT = nan;
                        hoverCenterHoldStartT = nan;
                        pidX = autosimPidInit();
                        pidY = autosimPidInit();
                    end

                case 'hover_settle'
                    if ~isFlying
                        controlPhase = "takeoff";
                        phaseEnterT = tk;
                        decisionEvalStartT = nan;
                    elseif (tk - phaseEnterT) >= cfg.control.hover_settle_sec
                        controlPhase = "xy_hold";
                        phaseEnterT = tk;
                        decisionEvalStartT = tk;
                    end

                case 'xy_hold'
                    if ~isFlying
                        controlPhase = "takeoff";
                        phaseEnterT = tk;
                        hoverStartT = nan;
                        decisionEvalStartT = nan;
                        hoverCenterHoldStartT = nan;
                        pidX = autosimPidInit();
                        pidY = autosimPidInit();
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

                            [ux, pidX] = autosimPidStep(errV, dtCtrl, pidX, cfg.control.xy_kp, cfg.control.xy_ki, cfg.control.xy_kd, cfg.control.xy_i_limit, cfg.control.xy_cmd_limit);
                            [uy, pidY] = autosimPidStep(errU, dtCtrl, pidY, cfg.control.xy_kp, cfg.control.xy_ki, cfg.control.xy_kd, cfg.control.xy_i_limit, cfg.control.xy_cmd_limit);

                            cmdX = cfg.control.xy_map_sign_x_from_v * ux;
                            cmdY = cfg.control.xy_map_sign_y_from_u * uy;

                            if sqrt(errU*errU + errV*errV) <= cfg.control.tag_center_deadband
                                cmdX = 0.0;
                                cmdY = 0.0;
                            end
                        else
                            pidX = autosimPidInit();
                            pidY = autosimPidInit();

                            if cfg.control.pose_hold_enable && isfinite(xNow) && isfinite(yNow)
                                errX = -xNow;
                                errY = -yNow;
                                cmdX = autosimClamp(cfg.control.pose_hold_kp * errX, -abs(cfg.control.pose_hold_cmd_limit), abs(cfg.control.pose_hold_cmd_limit));
                                cmdY = autosimClamp(cfg.control.pose_hold_kp * errY, -abs(cfg.control.pose_hold_cmd_limit), abs(cfg.control.pose_hold_cmd_limit));
                            elseif cfg.control.search_enable_spiral
                                if ~isfinite(tagLostSearchStartT)
                                    tagLostSearchStartT = tk;
                                end

                                tSearch = max(0.0, tk - tagLostSearchStartT);
                                rSearch = cfg.control.search_spiral_start_radius + cfg.control.search_spiral_growth_per_sec * tSearch;
                                rSearch = min(rSearch, cfg.control.search_spiral_cmd_max);
                                th = cfg.control.search_spiral_omega_rad_sec * tSearch;
                                cmdX = autosimClamp(rSearch * cos(th), -abs(cfg.control.search_spiral_cmd_max), abs(cfg.control.search_spiral_cmd_max));
                                cmdY = autosimClamp(rSearch * sin(th), -abs(cfg.control.search_spiral_cmd_max), abs(cfg.control.search_spiral_cmd_max));
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
                randomLandingPlanned = true;
                randomLandingStartT = tk + autosimRandRange(cfg.learning.random_landing_wait_min_sec, cfg.learning.random_landing_wait_max_sec);
                randomLandingEndT = randomLandingStartT + autosimRandRange(cfg.learning.random_cmd_duration_min_sec, cfg.learning.random_cmd_duration_max_sec);
                randomBiasX = autosimRandRange(-cfg.learning.random_xy_cmd_max, cfg.learning.random_xy_cmd_max);
                randomBiasY = autosimRandRange(-cfg.learning.random_xy_cmd_max, cfg.learning.random_xy_cmd_max);
            end

            if randomLandingPlanned && ~landingSent
                if tk >= randomLandingStartT && tk < randomLandingEndT
                    cmdX = autosimClamp(cmdX + randomBiasX, -abs(cfg.control.xy_cmd_limit), abs(cfg.control.xy_cmd_limit));
                    cmdY = autosimClamp(cmdY + randomBiasY, -abs(cfg.control.xy_cmd_limit), abs(cfg.control.xy_cmd_limit));
                elseif tk >= randomLandingEndT
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

        forceWindByTimeout = isfinite(hoverStartT) && ((tk - hoverStartT) >= cfg.wind.start_force_after_hover_sec);

        if ~windArmed && cfg.wind.enable && hoverDelayOk && (hoverCenterReady || forceWindByTimeout)
            windArmed = true;
            if forceWindByTimeout && ~hoverCenterReady
                fprintf('[AUTOSIM] s%03d wind armed by timeout at t=%.1fs (center-hold unmet)\n', scenarioId, tk);
            end
        end

        feat = autosimBuildOnlineFeatureVector(z(1:k), vz(1:k), speedAbs(1:k), rollDeg(1:k), pitchDeg(1:k), ...
            tagErr(1:k), windSpeed(1:k), contact(1:k), imuAngVel(1:k), imuLinAcc(1:k), ...
            contactForce(1:k), armForceFL(1:k), armForceFR(1:k), armForceRL(1:k), armForceRR(1:k), semVec, cfg);
        featureSchema = cfg.model.feature_names;
        if isfield(model, 'feature_names') && ~isempty(model.feature_names)
            featureSchema = model.feature_names;
        end
        requestedSemanticOnlyMode = isfield(cfg.agent, 'semantic_only_mode') && cfg.agent.semantic_only_mode;
        modelGateEnabled = cfg.agent.enable_model_decision && hasTrainedModel;
        semanticOnlyMode = requestedSemanticOnlyMode && ~modelGateEnabled;
        if modelGateEnabled
            [predLabel, predScore] = autosimPredictModel(model, feat, featureSchema);
            if predLabel == "stable"
                predStableProb(k) = predScore;
            else
                predStableProb(k) = 1.0 - predScore;
            end
        else
            predStableProb(k) = autosimClampNaN(semantic.landing_feasibility, 0.0);
            if predStableProb(k) >= autosimClampNaN(cfg.agent.semantic_land_threshold, 0.70)
                predLabel = "stable";
            else
                predLabel = "unstable";
            end
            predScore = predStableProb(k);
        end

        probeBoost = 0.0;
        if isfield(scenarioCfg, 'safe_probe_ratio_boost') && isfinite(scenarioCfg.safe_probe_ratio_boost)
            probeBoost = autosimClamp(scenarioCfg.safe_probe_ratio_boost, 0.0, 0.20);
        end
        adaptiveProbLandThreshold = autosimClamp(cfg.agent.prob_land_threshold - probeBoost, 0.05, 0.99);
        adaptiveSemanticLandThreshold = autosimClamp(cfg.agent.semantic_land_threshold - probeBoost, 0.05, 0.99);

        modelSaysStable = hasTrainedModel && modelGateEnabled && isfinite(predStableProb(k)) && (predLabel == "stable") && (predStableProb(k) >= adaptiveProbLandThreshold);
        modelSaysUnstable = hasTrainedModel && modelGateEnabled && (~modelSaysStable);

        if ~landingSent && cfg.agent.block_landing_if_unstable && modelSaysUnstable
            % Only block landing decision; keep XY correction active unless explicitly frozen.
            if cfg.agent.freeze_xy_if_unstable
                cmdX = 0.0;
                cmdY = 0.0;
            end
            randomLandingPlanned = false;
            if decisionTxt(k) == ""
                decisionTxt(k) = "wait_hover_unstable";
            end
        end

        hoverEvalReady = isFlying && (controlPhase == "xy_hold") && isfinite(decisionEvalStartT) && ...
            ((tk - decisionEvalStartT) >= cfg.agent.min_hover_eval_sec);

        canLandByModel = hoverEvalReady && hasTrainedModel && modelGateEnabled && ...
            (k >= cfg.agent.min_samples_before_decision) && ...
            modelSaysStable && ...
            isfinite(tagErr(k)) && (tagErr(k) <= cfg.agent.max_tag_error_before_land) && ...
            isfinite(z(k)) && (z(k) >= cfg.agent.min_altitude_before_land) && ...
            ((tk - lastDecisionT) >= cfg.agent.decision_cooldown_sec);

        canLandBySemantic = hoverEvalReady && semanticOnlyMode && ...
            isfinite(tagErr(k)) && (tagErr(k) <= cfg.agent.max_tag_error_before_land) && ...
            isfinite(z(k)) && (z(k) >= cfg.agent.min_altitude_before_land) && ...
            isfinite(semantic.landing_feasibility) && (semantic.landing_feasibility >= adaptiveSemanticLandThreshold) && ...
            logical(semanticSafe(k)) && ...
            ((tk - lastDecisionT) >= cfg.agent.decision_cooldown_sec);

        evalWinN = max(5, round(cfg.agent.no_model_eval_window_sec / cfg.scenario.sample_period_sec));
        zWin = autosimTail(z(1:k), evalWinN);
        xWin = autosimTail(xPos(1:k), evalWinN);
        yWin = autosimTail(yPos(1:k), evalWinN);
        [zOscStd, zFlipRateHz] = autosimCalcZOscillationMetrics(zWin, cfg.scenario.sample_period_sec);
        [xyStd, xySpeedRms] = autosimCalcXYMotionMetrics(xWin, yWin, cfg.scenario.sample_period_sec);
        xyRadiusNow = sqrt(xNow*xNow + yNow*yNow);

        canLandByNoModelThreshold = hoverEvalReady && (~modelGateEnabled) && (~semanticOnlyMode) && cfg.agent.no_model_fallback_enable && ...
            (k >= cfg.agent.no_model_min_samples_before_land) && ...
            isfinite(tagErr(k)) && (tagErr(k) <= cfg.agent.no_model_max_tag_error) && ...
            isfinite(z(k)) && (z(k) >= cfg.agent.min_altitude_before_land) && ...
            isfinite(vzNow) && (abs(vzNow) <= cfg.agent.no_model_max_abs_vz) && ...
            isfinite(zOscStd) && (zOscStd <= cfg.agent.no_model_max_z_osc_std) && ...
            isfinite(zFlipRateHz) && (zFlipRateHz <= cfg.agent.no_model_max_z_flip_rate_hz) && ...
            isfinite(xyStd) && (xyStd <= cfg.agent.no_model_max_xy_std) && ...
            isfinite(xySpeedRms) && (xySpeedRms <= cfg.agent.no_model_max_xy_speed_rms) && ...
            isfinite(xyRadiusNow) && (xyRadiusNow <= cfg.agent.no_model_max_xy_radius) && ...
            isfinite(autosimNanLast(rollDeg(1:k))) && (abs(autosimNanLast(rollDeg(1:k))) <= cfg.agent.no_model_max_abs_roll_pitch_deg) && ...
            isfinite(autosimNanLast(pitchDeg(1:k))) && (abs(autosimNanLast(pitchDeg(1:k))) <= cfg.agent.no_model_max_abs_roll_pitch_deg) && ...
            isfinite(windSpNow) && (windSpNow <= cfg.agent.no_model_max_wind_speed) && ...
            (~cfg.agent.no_model_require_semantic_safe || semanticSafe(k)) && ...
            ((tk - lastDecisionT) >= cfg.agent.decision_cooldown_sec);

        canLandByForcedTimeout = isFlying && (controlPhase == "xy_hold") && ~landingSent && isfinite(decisionEvalStartT) && ...
            isfield(cfg.control, 'land_forced_timeout_sec') && isfinite(cfg.control.land_forced_timeout_sec) && ...
            (cfg.control.land_forced_timeout_sec > 0) && ...
            ((tk - decisionEvalStartT) >= cfg.control.land_forced_timeout_sec) && ...
            ((tk - lastDecisionT) >= cfg.agent.decision_cooldown_sec);

        guardLandingAllowed = ~cfg.agent.block_landing_if_unstable || ~modelSaysUnstable;

        if ~landingSent && guardLandingAllowed && canLandBySemantic
            send(pubLand, msgLand);
            landingSent = true;
            landingSentT = tk;
            landingDecisionMode = "land";
            executedAction = "land";
            lastDecisionT = tk;
            decisionTxt(k) = "land_by_ontology_ai";
            controlPhase = "landing_observe";
        elseif ~landingSent && guardLandingAllowed && canLandByModel
            send(pubLand, msgLand);
            landingSent = true;
            landingSentT = tk;
            landingDecisionMode = "land";
            executedAction = "land";
            lastDecisionT = tk;
            decisionTxt(k) = "land_by_model";
            controlPhase = "landing_observe";
        elseif ~landingSent && guardLandingAllowed && canLandByNoModelThreshold
            send(pubLand, msgLand);
            landingSent = true;
            landingSentT = tk;
            landingDecisionMode = "land";
            executedAction = "land";
            lastDecisionT = tk;
            decisionTxt(k) = "land_by_threshold_no_model";
            controlPhase = "landing_observe";
        elseif ~landingSent && canLandByForcedTimeout
            send(pubLand, msgLand);
            landingSent = true;
            landingSentT = tk;
            landingDecisionMode = "abort";
            executedAction = "land";
            lastDecisionT = tk;
            decisionTxt(k) = "land_by_forced_timeout";
            controlPhase = "landing_observe";
        elseif decisionTxt(k) == "" && hasTrainedModel && cfg.agent.enable_model_decision && ~modelSaysStable
            decisionTxt(k) = "hold_by_model_unstable";
        elseif decisionTxt(k) == ""
            decisionTxt(k) = "track";
        end

        phaseTxt(k) = controlPhase;

        if landingSent || canLandBySemantic || canLandByModel || canLandByNoModelThreshold || canLandByForcedTimeout
            inferTxt = "LAND";
        else
            inferTxt = "NO-LAND";
        end

        autosimUpdateScenarioRealtimePlot(liveViz, xNow, yNow, windSpNow, windDirNow, tk, controlPhase, zNow, ...
            inferTxt, predStableProb(k), decisionTxt(k));

        if landingSent
            cmdX = 0.0;
            cmdY = 0.0;
        end

        % Keep publishing cmd_vel so the topic is always alive for debugging/monitoring.
        if isFlying || cfg.control.publish_cmd_always
            msgCmd.linear.x = cmdX;
            msgCmd.linear.y = cmdY;
            msgCmd.linear.z = 0.0;
            msgCmd.angular.x = 0.0;
            msgCmd.angular.y = 0.0;
            msgCmd.angular.z = 0.0;
            send(pubCmd, msgCmd);
        end

        if landingSent
            landedByState = isfinite(stateVal(k)) && (stateVal(k) == cfg.thresholds.land_state_value);
            landedByPose = isfinite(z(k)) && (z(k) <= (cfg.thresholds.landed_altitude_max_m + 0.05));
            if landedByState || landedByPose
                if ~isfinite(landedHoldStartT)
                    landedHoldStartT = tk;
                end
            else
                landedHoldStartT = nan;
            end

            if isfinite(landedHoldStartT) && ((tk - landedHoldStartT) >= cfg.scenario.early_stop_after_landing_sec)
                break;
            end
        end

        pause(cfg.scenario.sample_period_sec);
    end

    t = t(1:kLast);
    xPos = xPos(1:kLast);
    yPos = yPos(1:kLast);
    z = z(1:kLast);
    vz = vz(1:kLast);
    speedAbs = speedAbs(1:kLast);
    rollDeg = rollDeg(1:kLast);
    pitchDeg = pitchDeg(1:kLast);
    tagErr = tagErr(1:kLast);
    windSpeed = windSpeed(1:kLast);
    stateVal = stateVal(1:kLast);
    contact = contact(1:kLast);
    imuAngVel = imuAngVel(1:kLast);
    imuLinAcc = imuLinAcc(1:kLast);
    contactForce = contactForce(1:kLast);
    armForceFL = armForceFL(1:kLast);
    armForceFR = armForceFR(1:kLast);
    armForceRL = armForceRL(1:kLast);
    armForceRR = armForceRR(1:kLast);
    predStableProb = predStableProb(1:kLast);
    decisionTxt = decisionTxt(1:kLast);
    phaseTxt = phaseTxt(1:kLast);
    windCmdSpeed = windCmdSpeed(1:kLast);
    windCmdDir = windCmdDir(1:kLast);
    semanticWindRisk = semanticWindRisk(1:kLast);
    semanticEnvironment = semanticEnvironment(1:kLast);
    semanticDroneState = semanticDroneState(1:kLast);
    semanticAlign = semanticAlign(1:kLast);
    semanticVisual = semanticVisual(1:kLast);
    semanticContext = semanticContext(1:kLast);
    semanticRelation = semanticRelation(1:kLast);
    semanticIntegration = semanticIntegration(1:kLast);
    semanticSafe = semanticSafe(1:kLast);
    landingFeasibility = landingFeasibility(1:kLast);
    semFeat = semFeat(1:kLast, :);

    msgCmd.linear.x = 0.0;
    msgCmd.linear.y = 0.0;
    msgCmd.linear.z = 0.0;
    msgCmd.angular.x = 0.0;
    msgCmd.angular.y = 0.0;
    msgCmd.angular.z = 0.0;
    send(pubCmd, msgCmd);

    if ~landingSent
        fprintf('[AUTOSIM] s%03d ended without LAND inference, so no landing command was sent.\n', scenarioId);
    end

    postN = max(1, floor(cfg.scenario.post_land_observe_sec / cfg.scenario.sample_period_sec));
    if stopRequested
        postN = 0;
    end
    for m = 1:postN
        if autosimIsStopRequested()
            stopRequested = true;
            stopReason = autosimGetStopReason();
            fprintf('[AUTOSIM] Scenario %03d stop requested during post-observe: %s\n', scenarioId, stopReason);
            break;
        end

        poseMsg = autosimTryReceive(subPose, 0.01);
        if ~isempty(poseMsg)
            xPos(end+1,1) = double(poseMsg.position.x); %#ok<AGROW>
            yPos(end+1,1) = double(poseMsg.position.y); %#ok<AGROW>
            z(end+1,1) = double(poseMsg.position.z); %#ok<AGROW>
            q = poseMsg.orientation;
            [r, p, ~] = autosimQuat2Eul([q.w, q.x, q.y, q.z]);
            rollDeg(end+1,1) = abs(rad2deg(r)); %#ok<AGROW>
            pitchDeg(end+1,1) = abs(rad2deg(p)); %#ok<AGROW>
        else
            xPos(end+1,1) = nan; %#ok<AGROW>
            yPos(end+1,1) = nan; %#ok<AGROW>
            z(end+1,1) = nan; %#ok<AGROW>
            rollDeg(end+1,1) = nan; %#ok<AGROW>
            pitchDeg(end+1,1) = nan; %#ok<AGROW>
        end

        velMsg = autosimTryReceive(subVel, 0.01);
        if ~isempty(velMsg)
            vx = double(velMsg.linear.x);
            vy = double(velMsg.linear.y);
            vz(end+1,1) = double(velMsg.linear.z); %#ok<AGROW>
            speedAbs(end+1,1) = sqrt(vx*vx + vy*vy + vz(end)^2); %#ok<AGROW>
        else
            vz(end+1,1) = nan; %#ok<AGROW>
            speedAbs(end+1,1) = nan; %#ok<AGROW>
        end

        stateMsg = autosimTryReceive(subState, 0.01);
        if ~isempty(stateMsg)
            stateVal(end+1,1) = double(stateMsg.data); %#ok<AGROW>
        else
            stateVal(end+1,1) = nan; %#ok<AGROW>
        end

        tagMsg = autosimTryReceive(subTag, 0.01);
        if ~isempty(tagMsg)
            [~, ~, ~, te] = autosimParseTag(tagMsg);
            tagErr(end+1,1) = te; %#ok<AGROW>
        else
            tagErr(end+1,1) = nan; %#ok<AGROW>
        end

        windMsg = autosimTryReceive(subWind, 0.01);
        if ~isempty(windMsg)
            [wsPost, ~] = autosimParseWindConditionMsg(windMsg);
            windSpeed(end+1,1) = wsPost; %#ok<AGROW>
        else
            windSpeed(end+1,1) = nan; %#ok<AGROW>
        end

        if ~isempty(subImu)
            imuMsg = autosimTryReceive(subImu, 0.01);
            if ~isempty(imuMsg)
                [angNow, accNow] = autosimParseImuMetrics(imuMsg);
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

        if ~isempty(subBumpers)
            bumpMsg = autosimTryReceive(subBumpers, 0.01);
            if ~isempty(bumpMsg)
                [cNow, fNow, flNow, frNow, rlNow, rrNow] = autosimParseContactForces(bumpMsg);
                contact(end+1,1) = cNow; %#ok<AGROW>
                contactForce(end+1,1) = fNow; %#ok<AGROW>
                armForceFL(end+1,1) = flNow; %#ok<AGROW>
                armForceFR(end+1,1) = frNow; %#ok<AGROW>
                armForceRL(end+1,1) = rlNow; %#ok<AGROW>
                armForceRR(end+1,1) = rrNow; %#ok<AGROW>
            else
                contact(end+1,1) = 0; %#ok<AGROW>
                contactForce(end+1,1) = nan; %#ok<AGROW>
                armForceFL(end+1,1) = nan; %#ok<AGROW>
                armForceFR(end+1,1) = nan; %#ok<AGROW>
                armForceRL(end+1,1) = nan; %#ok<AGROW>
                armForceRR(end+1,1) = nan; %#ok<AGROW>
            end
        else
            contact(end+1,1) = 0; %#ok<AGROW>
            contactForce(end+1,1) = nan; %#ok<AGROW>
            armForceFL(end+1,1) = nan; %#ok<AGROW>
            armForceFR(end+1,1) = nan; %#ok<AGROW>
            armForceRL(end+1,1) = nan; %#ok<AGROW>
            armForceRR(end+1,1) = nan; %#ok<AGROW>
        end

        predStableProb(end+1,1) = nan; %#ok<AGROW>
        decisionTxt(end+1,1) = "post_observe"; %#ok<AGROW>
        t(end+1,1) = toc(t0); %#ok<AGROW>

        inferPost = "NO-LAND";
        if landingSent
            inferPost = "LAND";
        end
        predPost = autosimNanLast(predStableProb);
        if ~isfinite(predPost)
            predPost = nan;
        end

        autosimUpdateScenarioRealtimePlot(liveViz, autosimNanLast(xPos), autosimNanLast(yPos), ...
            autosimNanLast(windSpeed), autosimNanLast(windCmdDir), t(end), "post_observe", autosimNanLast(z), ...
            inferPost, predPost, "post_observe");

        pause(cfg.scenario.sample_period_sec);
    end

    autosimCleanupRosHandles(rosHandles);
    clear rosCleanup;

    res = autosimSummarizeAndLabel(cfg, scenarioId, scenarioCfg, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, contact, ...
        imuAngVel, imuLinAcc, contactForce, armForceFL, armForceFR, armForceRL, armForceRR);
    res.landing_cmd_time = landingSentT;
    res.pred_decision = string(landingDecisionMode);
    res.executed_action = string(executedAction);
    res.gt_safe_to_land = "unstable";
    if string(res.label) == "stable"
        res.gt_safe_to_land = "stable";
    end
    res.decision_outcome = autosimClassifyDecisionOutcome(res.gt_safe_to_land, res.pred_decision);
    res.semantic_environment = autosimLastNonEmptyString(semanticEnvironment, "unknown");
    res.semantic_drone_state = autosimLastNonEmptyString(semanticDroneState, "unknown");
    res.semantic_visual_state = autosimLastNonEmptyString(semanticVisual, "unknown");
    res.semantic_landing_context = autosimLastNonEmptyString(semanticContext, "unknown");
    res.semantic_relation = autosimLastNonEmptyString(semanticRelation, "unknown");
    res.semantic_integration = autosimLastNonEmptyString(semanticIntegration, "unknown");
    res.landing_feasibility = autosimLastFinite(landingFeasibility, nan);
    if isfield(scenarioCfg, 'policy_mode')
        res.scenario_policy = string(scenarioCfg.policy_mode);
    end
    if stopRequested
        res.exception_message = string(stopReason);
    end

    n = numel(t);
    traceTbl = table();
    traceTbl.scenario_id = repmat(scenarioId, n, 1);
    traceTbl.t_sec = t;
    traceTbl.x = autosimPadLen(xPos, n);
    traceTbl.y = autosimPadLen(yPos, n);
    traceTbl.z = autosimPadLen(z, n);
    traceTbl.vz = autosimPadLen(vz, n);
    traceTbl.speed_abs = autosimPadLen(speedAbs, n);
    traceTbl.roll_deg = autosimPadLen(rollDeg, n);
    traceTbl.pitch_deg = autosimPadLen(pitchDeg, n);
    traceTbl.tag_error = autosimPadLen(tagErr, n);
    traceTbl.wind_speed = autosimPadLen(windSpeed, n);
    traceTbl.wind_cmd_speed = autosimPadLen(windCmdSpeed, n);
    traceTbl.wind_cmd_dir = autosimPadLen(windCmdDir, n);
    traceTbl.state = autosimPadLen(stateVal, n);
    traceTbl.imu_ang_vel = autosimPadLen(imuAngVel, n);
    traceTbl.imu_lin_acc = autosimPadLen(imuLinAcc, n);
    traceTbl.contact_force = autosimPadLen(contactForce, n);
    traceTbl.arm_force_fl = autosimPadLen(armForceFL, n);
    traceTbl.arm_force_fr = autosimPadLen(armForceFR, n);
    traceTbl.arm_force_rl = autosimPadLen(armForceRL, n);
    traceTbl.arm_force_rr = autosimPadLen(armForceRR, n);
    traceTbl.pred_stable_prob = autosimPadLen(predStableProb, n);
    traceTbl.decision = autosimPadLenString(decisionTxt, n);
    traceTbl.control_phase = autosimPadLenString(phaseTxt, n);
    traceTbl.semantic_wind_risk = autosimPadLenString(semanticWindRisk, n);
    traceTbl.semantic_environment = autosimPadLenString(semanticEnvironment, n);
    traceTbl.semantic_drone_state = autosimPadLenString(semanticDroneState, n);
    traceTbl.semantic_alignment = autosimPadLenString(semanticAlign, n);
    traceTbl.semantic_visual = autosimPadLenString(semanticVisual, n);
    traceTbl.semantic_context = autosimPadLenString(semanticContext, n);
    traceTbl.semantic_relation = autosimPadLenString(semanticRelation, n);
    traceTbl.semantic_integration = autosimPadLenString(semanticIntegration, n);
    traceTbl.semantic_safe = autosimPadLen(double(semanticSafe), n);
    traceTbl.landing_feasibility = autosimPadLen(landingFeasibility, n);
    for i = 1:numel(cfg.ontology.semantic_feature_names)
        fn = char(cfg.ontology.semantic_feature_names(i));
        traceTbl.(['sem_' fn]) = autosimPadLen(semFeat(:, i), n);
    end
    traceTbl.scenario_policy = repmat(string(res.scenario_policy), n, 1);
    traceTbl.pred_decision = repmat(string(res.pred_decision), n, 1);
    traceTbl.executed_action = repmat(string(res.executed_action), n, 1);
    traceTbl.gt_safe_to_land = repmat(string(res.gt_safe_to_land), n, 1);
    traceTbl.decision_outcome = repmat(string(res.decision_outcome), n, 1);
    traceTbl.final_label = repmat(string(res.label), n, 1);
end


function out = autosimPadLen(x, n)
    x = x(:);
    if numel(x) < n
        out = [x; nan(n-numel(x),1)];
    else
        out = x(1:n);
    end
end


function out = autosimPadLenString(x, n)
    x = string(x(:));
    if numel(x) < n
        out = [x; repmat("", n-numel(x), 1)];
    else
        out = x(1:n);
    end
end


function out = autosimLastNonEmptyString(x, fallback)
    out = string(fallback);
    if isempty(x)
        return;
    end

    x = string(x(:));
    mask = strlength(x) > 0;
    if any(mask)
        out = x(find(mask, 1, 'last'));
    end
end


function out = autosimLastFinite(x, fallback)
    out = fallback;
    if isempty(x)
        return;
    end

    x = double(x(:));
    idx = find(isfinite(x), 1, 'last');
    if ~isempty(idx)
        out = x(idx);
    end
end


function feat = autosimBuildOnlineFeatureVector(z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, contact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR, semVec, cfg)
    if nargin < 16
        semVec = [];
    end
    if nargin < 17
        cfg = [];
    end

    feat = struct();
    feat.mean_wind_speed = autosimNanMean(windSpeed);
    feat.max_wind_speed = autosimNanMax(windSpeed);
    feat.mean_abs_roll_deg = autosimNanMean(abs(rollDeg));
    feat.mean_abs_pitch_deg = autosimNanMean(abs(pitchDeg));
    feat.mean_abs_vz = autosimNanMean(abs(vz));
    feat.max_abs_vz = autosimNanMax(abs(vz));
    feat.mean_tag_error = autosimNanMean(tagErr);
    feat.max_tag_error = autosimNanMax(tagErr);

    feat.final_altitude = autosimNanLast(z);
    feat.final_abs_speed = autosimNanLast(speedAbs);
    feat.final_abs_roll_deg = autosimNanLast(abs(rollDeg));
    feat.final_abs_pitch_deg = autosimNanLast(abs(pitchDeg));
    feat.final_tag_error = autosimNanLast(tagErr);

    finalWindow = min(numel(z), 25);
    zTail = autosimTail(z, finalWindow);
    vzTail = autosimTail(vz, finalWindow);
    feat.stability_std_z = autosimNanStd(zTail);
    feat.stability_std_vz = autosimNanStd(vzTail);

    [~, feat.stability_std_vz_osc, feat.touchdown_accel_rms] = autosimCalcVzMetrics(vzTail, 0.2);
    feat.contact_count = sum(contact > 0);
    feat.mean_imu_ang_vel = autosimNanMean(imuAngVel);
    feat.max_imu_ang_vel = autosimNanMax(imuAngVel);
    feat.mean_imu_lin_acc = autosimNanMean(imuLinAcc);
    feat.max_imu_lin_acc = autosimNanMax(imuLinAcc);
    feat.max_contact_force = autosimNanMax(contactForce);
    feat.arm_force_imbalance = autosimNanMax([abs(armFL-armFR), abs(armRL-armRR)]);

    if ~isempty(semVec) && ~isempty(cfg) && isfield(cfg, 'ontology') && isfield(cfg.ontology, 'semantic_feature_names')
        semNames = string(cfg.ontology.semantic_feature_names);
        feat.wind_risk_enc = autosimSemGet(semVec, semNames, "wind_risk_enc", 0.0);
        feat.alignment_enc = autosimSemGet(semVec, semNames, "alignment_enc", 0.0);
        feat.visual_enc = autosimSemGet(semVec, semNames, "visual_enc", 0.0);
        feat.context_enc = autosimSemGet(semVec, semNames, "context_enc", 0.0);
    end
end


function out = autosimSummarizeAndLabel(cfg, scenarioId, scenarioCfg, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, bumperContact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR)
    out = autosimEmptyScenarioResult();
    out.scenario_id = scenarioId;
    if isfield(scenarioCfg, 'policy_mode')
        out.scenario_policy = string(scenarioCfg.policy_mode);
    end
    out.hover_height_cmd = scenarioCfg.hover_height_m;
    out.wind_speed_cmd = scenarioCfg.wind_speed;
    out.wind_dir_cmd = scenarioCfg.wind_dir;

    out.mean_wind_speed = autosimNanMean(windSpeed);
    out.max_wind_speed = autosimNanMax(windSpeed);
    out.mean_abs_roll_deg = autosimNanMean(abs(rollDeg));
    out.mean_abs_pitch_deg = autosimNanMean(abs(pitchDeg));
    out.mean_abs_vz = autosimNanMean(abs(vz));
    out.max_abs_vz = autosimNanMax(abs(vz));
    out.mean_tag_error = autosimNanMean(tagErr);
    out.max_tag_error = autosimNanMax(tagErr);

    out.final_altitude = autosimNanLast(z);
    out.landing_height_m = out.final_altitude;
    out.final_abs_speed = autosimNanLast(speedAbs);
    out.final_abs_roll_deg = autosimNanLast(abs(rollDeg));
    out.final_abs_pitch_deg = autosimNanLast(abs(pitchDeg));
    out.final_tag_error = autosimNanLast(tagErr);

    [zStable, vzStable] = autosimSelectLandingStabilityWindow(z, vz, stateVal, cfg);

    out.stability_std_z = autosimNanStd(zStable);
    out.stability_std_vz = autosimNanStd(vzStable);

    vzTouch = autosimSelectTouchdownDynamicsWindow(vz, stateVal, cfg);
    [~, out.stability_std_vz_osc, out.touchdown_accel_rms] = autosimCalcVzMetrics(vzTouch, cfg.scenario.sample_period_sec);
    out.max_abs_vz = max(out.max_abs_vz, autosimNanMax(abs(vzTouch)));
    out.contact_count = sum(bumperContact > 0);
    out.mean_imu_ang_vel = autosimNanMean(imuAngVel);
    out.max_imu_ang_vel = autosimNanMax(imuAngVel);
    out.mean_imu_lin_acc = autosimNanMean(imuLinAcc);
    out.max_imu_lin_acc = autosimNanMax(imuLinAcc);
    out.max_contact_force = autosimNanMax(contactForce);
    out.arm_force_fl_mean = autosimNanMean(armFL);
    out.arm_force_fr_mean = autosimNanMean(armFR);
    out.arm_force_rl_mean = autosimNanMean(armRL);
    out.arm_force_rr_mean = autosimNanMean(armRR);
    out.arm_force_imbalance = autosimNanMax([abs(armFL-armFR), abs(armRL-armRR)]);
    out.final_state = autosimNanLast(stateVal);

    c = cfg.thresholds;
    condState = isfinite(out.final_state) && out.final_state == c.land_state_value;
    condAlt = isfinite(out.final_altitude) && out.final_altitude <= c.landed_altitude_max_m;
    condSpeed = isfinite(out.final_abs_speed) && out.final_abs_speed <= c.final_speed_max_mps;
    condRoll = isfinite(out.final_abs_roll_deg) && out.final_abs_roll_deg <= c.final_attitude_max_deg;
    condPitch = isfinite(out.final_abs_pitch_deg) && out.final_abs_pitch_deg <= c.final_attitude_max_deg;
    condTag = (~isfinite(out.final_tag_error)) || (out.final_tag_error <= c.final_tag_error_max);
    condStdZ = isfinite(out.stability_std_z) && out.stability_std_z <= c.final_stability_std_z_max;
    condStdVz = isfinite(out.stability_std_vz) && out.stability_std_vz <= c.final_stability_std_vz_max;
    condVzOsc = (~isfinite(out.stability_std_vz_osc)) || (out.stability_std_vz_osc <= c.final_touchdown_vz_osc_max);
    condTdAcc = (~isfinite(out.touchdown_accel_rms)) || (out.touchdown_accel_rms <= c.final_touchdown_accel_rms_max);
    condTdVz = isfinite(out.max_abs_vz) && (out.max_abs_vz <= c.final_touchdown_abs_vz_max);
    condImuAng = (~isfinite(out.max_imu_ang_vel)) || (out.max_imu_ang_vel <= c.final_imu_ang_vel_rms_max);
    condImuAcc = (~isfinite(out.max_imu_lin_acc)) || (out.max_imu_lin_acc <= c.final_imu_lin_acc_rms_max);
    condContactForce = (~isfinite(out.max_contact_force)) || (out.max_contact_force <= c.final_contact_force_max_n);
    condArmBalance = (~isfinite(out.arm_force_imbalance)) || (out.arm_force_imbalance <= c.final_arm_force_imbalance_max_n);

    passAll = condState && condAlt && condSpeed && condRoll && condPitch && condTag && condStdZ && condStdVz && condVzOsc && condTdAcc && condTdVz && ...
        condImuAng && condImuAcc && condContactForce && condArmBalance;

    if passAll
        out.label = "stable";
        out.success = true;
        out.failure_reason = "";
    else
        out.label = "unstable";
        out.success = false;
        out.failure_reason = autosimBuildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz, condVzOsc, condTdAcc, condTdVz, ...
            condImuAng, condImuAcc, condContactForce, condArmBalance);
    end
end


function reason = autosimBuildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz, condVzOsc, condTdAcc, condTdVz, condImuAng, condImuAcc, condContactForce, condArmBalance)
    parts = strings(0,1);
    if ~condState, parts(end+1,1) = "state_not_landed"; end %#ok<AGROW>
    if ~condAlt, parts(end+1,1) = "altitude_high"; end %#ok<AGROW>
    if ~condSpeed, parts(end+1,1) = "speed_high"; end %#ok<AGROW>
    if ~condRoll, parts(end+1,1) = "roll_high"; end %#ok<AGROW>
    if ~condPitch, parts(end+1,1) = "pitch_high"; end %#ok<AGROW>
    if ~condTag, parts(end+1,1) = "tag_error_high"; end %#ok<AGROW>
    if ~condStdZ, parts(end+1,1) = "z_unstable"; end %#ok<AGROW>
    if ~condStdVz, parts(end+1,1) = "vz_unstable"; end %#ok<AGROW>
    if ~condVzOsc, parts(end+1,1) = "touchdown_vz_osc_high"; end %#ok<AGROW>
    if ~condTdAcc, parts(end+1,1) = "touchdown_accel_high"; end %#ok<AGROW>
    if ~condTdVz, parts(end+1,1) = "touchdown_vz_peak_high"; end %#ok<AGROW>
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


function [zStable, vzStable] = autosimSelectLandingStabilityWindow(z, vz, stateVal, cfg)
    dt = max(cfg.scenario.sample_period_sec, 1e-3);
    tailN = max(3, round(2.4 / dt));

    zStable = autosimTail(z, tailN);
    vzStable = autosimTail(vz, tailN);

    landedIdx = find(isfinite(stateVal) & (stateVal == cfg.thresholds.land_state_value));
    if numel(landedIdx) < 3
        return;
    end

    % Use the final contiguous landed segment to avoid including descent dynamics.
    segStart = 1;
    d = diff(landedIdx);
    jumpIdx = find(d > 1, 1, 'last');
    if ~isempty(jumpIdx)
        segStart = jumpIdx + 1;
    end
    landedTail = landedIdx(segStart:end);
    if numel(landedTail) < 3
        return;
    end

    settleSkipN = round(0.8 / dt);
    if numel(landedTail) > (settleSkipN + 2)
        landedTail = landedTail((settleSkipN + 1):end);
    end

    useN = min(numel(landedTail), tailN);
    idxUse = landedTail(end-useN+1:end);

    zSel = z(idxUse);
    vzSel = vz(idxUse);
    zSel = zSel(isfinite(zSel));
    vzSel = vzSel(isfinite(vzSel));
    if numel(zSel) >= 3
        zStable = zSel;
    end
    if numel(vzSel) >= 3
        vzStable = vzSel;
    end
end


function vzTouch = autosimSelectTouchdownDynamicsWindow(vz, stateVal, cfg)
    dt = max(cfg.scenario.sample_period_sec, 1e-3);
    preN = max(3, round(1.2 / dt));
    postN = max(4, round(1.6 / dt));
    n = numel(vz);

    vzTouch = autosimTail(vz, preN + postN);
    if n < 3 || numel(stateVal) ~= n
        return;
    end

    landedIdx = find(isfinite(stateVal) & (stateVal == cfg.thresholds.land_state_value));
    if isempty(landedIdx)
        return;
    end

    segStart = 1;
    d = diff(landedIdx);
    jumpIdx = find(d > 1, 1, 'last');
    if ~isempty(jumpIdx)
        segStart = jumpIdx + 1;
    end
    landedTail = landedIdx(segStart:end);
    if isempty(landedTail)
        return;
    end

    tdIdx = landedTail(1);
    i0 = max(1, tdIdx - preN);
    i1 = min(n, tdIdx + postN);
    vzSel = vz(i0:i1);
    vzSel = vzSel(isfinite(vzSel));
    if numel(vzSel) >= 4
        vzTouch = vzSel;
    end
end


function [model, info] = autosimIncrementalTrainAndSave(cfg, results, modelPrev, scenarioId)
    tbl = autosimSummaryTable(results);

    valid = false(height(tbl), 1);
    if ismember('label', tbl.Properties.VariableNames)
        valid = (tbl.label == "stable") | (tbl.label == "unstable");
    end

    info = struct();
    info.scenario_id = scenarioId;
    info.model_updated = false;
    info.n_train = sum(valid);
    info.stable_ratio = 0.0;
    info.n_stable = 0;
    info.n_unstable = 0;
    info.skip_reason = "";
    info.model_path = "";

    if sum(valid) < cfg.learning.bootstrap_min_samples
        model = modelPrev;
        info.skip_reason = "insufficient_total_samples";
        return;
    end

    if isfield(cfg.learning, 'min_scenarios_before_first_update') && isfinite(cfg.learning.min_scenarios_before_first_update)
        if scenarioId < cfg.learning.min_scenarios_before_first_update
            model = modelPrev;
            info.skip_reason = "warmup_before_first_update";
            return;
        end
    end

    if isfield(cfg.learning, 'save_every_scenario') && ~cfg.learning.save_every_scenario
        updateEvery = 1;
        if isfield(cfg.learning, 'update_every_n_scenarios') && isfinite(cfg.learning.update_every_n_scenarios)
            updateEvery = max(1, round(cfg.learning.update_every_n_scenarios));
        end
        if mod(max(1, scenarioId), updateEvery) ~= 0
            model = modelPrev;
            info.skip_reason = "cadence_skip";
            return;
        end
    end

    trainTbl = tbl(valid, :);
    y = string(trainTbl.label);
    y(y ~= "stable") = "unstable";

    nStable = sum(y == "stable");
    nUnstable = sum(y == "unstable");
    nTrain = numel(y);
    minorityRatio = min(nStable, nUnstable) / max(1, nTrain);
    info.n_stable = nStable;
    info.n_unstable = nUnstable;
    info.stable_ratio = mean(y == "stable");

    minStable = 1;
    minUnstable = 1;
    minMinorityRatio = 0.0;
    forceAfterStale = inf;
    if isfield(cfg.learning, 'min_stable_samples_for_update') && isfinite(cfg.learning.min_stable_samples_for_update)
        minStable = max(1, round(cfg.learning.min_stable_samples_for_update));
    end
    if isfield(cfg.learning, 'min_unstable_samples_for_update') && isfinite(cfg.learning.min_unstable_samples_for_update)
        minUnstable = max(1, round(cfg.learning.min_unstable_samples_for_update));
    end
    if isfield(cfg.learning, 'minority_ratio_floor_for_update') && isfinite(cfg.learning.minority_ratio_floor_for_update)
        minMinorityRatio = max(0.0, min(0.5, cfg.learning.minority_ratio_floor_for_update));
    end
    if isfield(cfg.learning, 'force_update_after_stale_scenarios') && isfinite(cfg.learning.force_update_after_stale_scenarios)
        forceAfterStale = max(1, round(cfg.learning.force_update_after_stale_scenarios));
    end

    lastUpdateScenario = nan;
    if isstruct(modelPrev) && isfield(modelPrev, 'last_update_scenario') && isfinite(modelPrev.last_update_scenario)
        lastUpdateScenario = double(modelPrev.last_update_scenario);
    end
    staleScenarioCount = inf;
    if isfinite(lastUpdateScenario)
        staleScenarioCount = max(0, scenarioId - lastUpdateScenario);
    end
    allowForcedRefresh = (nUnstable >= 1) && (staleScenarioCount >= forceAfterStale);

    if ~allowForcedRefresh && (nStable < minStable || nUnstable < minUnstable || minorityRatio < minMinorityRatio)
        model = modelPrev;
        info.skip_reason = "class_imbalance_guard";
        return;
    end

    featNames = cellstr(cfg.model.feature_names);
    X = zeros(height(trainTbl), numel(featNames));
    for i = 1:numel(featNames)
        col = featNames{i};
        if ismember(col, trainTbl.Properties.VariableNames)
            X(:,i) = autosimToNumeric(trainTbl.(col));
        end
    end

    model = autosimTrainGaussianNB(X, y, cfg.model.feature_names);
    model.n_train = nTrain;
    model.n_stable = nStable;
    model.n_unstable = nUnstable;
    model.stable_ratio = info.stable_ratio;
    model.minority_ratio = minorityRatio;
    model.last_update_scenario = scenarioId;

    ts = autosimTimestamp();
    modelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_%s_s%03d.mat', ts, scenarioId));
    save(modelPath, 'model');

    info.model_updated = true;
    info.model_path = string(modelPath);
    info.skip_reason = "";
end


function tf = autosimIsModelReliable(model, cfg)
    tf = false;
    if isempty(model)
        return;
    end

    if isfield(model, 'placeholder') && logical(model.placeholder)
        return;
    end

    if ~isfield(model, 'n_train') || ~isfield(model, 'n_stable') || ~isfield(model, 'n_unstable')
        % Legacy model files may not carry training-count metadata.
        tf = autosimHasUsableModelParameters(model);
        return;
    end

    nTrain = double(model.n_train);
    nStable = double(model.n_stable);
    nUnstable = double(model.n_unstable);
    if ~isfinite(nTrain) || ~isfinite(nStable) || ~isfinite(nUnstable)
        return;
    end

    minTrain = 1;
    minClass = 1;
    minMinorityRatio = 0.0;
    if isfield(cfg.agent, 'model_min_total_samples_for_use') && isfinite(cfg.agent.model_min_total_samples_for_use)
        minTrain = max(1, round(cfg.agent.model_min_total_samples_for_use));
    end
    if isfield(cfg.agent, 'model_min_class_samples_for_use') && isfinite(cfg.agent.model_min_class_samples_for_use)
        minClass = max(1, round(cfg.agent.model_min_class_samples_for_use));
    end
    if isfield(cfg.agent, 'model_minority_ratio_for_use') && isfinite(cfg.agent.model_minority_ratio_for_use)
        minMinorityRatio = max(0.0, min(0.5, cfg.agent.model_minority_ratio_for_use));
    end

    minorityRatio = min(nStable, nUnstable) / max(1, nTrain);
    tf = (nTrain >= minTrain) && (nStable >= minClass) && (nUnstable >= minClass) && (minorityRatio >= minMinorityRatio);
end


function tf = autosimHasUsableModelParameters(model)
    tf = false;
    req = {'class_names', 'mu', 'sigma2', 'prior'};
    for i = 1:numel(req)
        if ~isfield(model, req{i})
            return;
        end
    end

    cls = string(model.class_names(:));
    if numel(cls) < 2 || ~all(ismember(["stable"; "unstable"], cls))
        return;
    end

    mu = double(model.mu);
    s2 = double(model.sigma2);
    pr = double(model.prior(:));

    if isempty(mu) || isempty(s2) || numel(pr) < 2
        return;
    end
    if any(size(mu) ~= size(s2))
        return;
    end
    if size(mu, 1) ~= numel(cls)
        return;
    end
    if any(~isfinite(mu), 'all') || any(~isfinite(s2), 'all') || any(~isfinite(pr))
        return;
    end
    if any(s2(:) <= 0)
        return;
    end

    tf = true;
end


function tbl = autosimSummaryTable(results)
    if isempty(results)
        tbl = table();
        return;
    end

    tbl = struct2table(results);
    wanted = {
        'scenario_id','label','success','failure_reason', ...
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
        'landing_cmd_time','pred_decision','executed_action','gt_safe_to_land','decision_outcome', ...
        'semantic_environment','semantic_drone_state','semantic_visual_state','semantic_landing_context', ...
        'semantic_relation','semantic_integration','landing_feasibility','scenario_policy', ...
        'launch_pid','launch_log','exception_message'
    };

    cols = intersect(wanted, tbl.Properties.VariableNames, 'stable');
    tbl = tbl(:, cols);
end


function [predLabel, predScore] = autosimPredictModel(model, featStruct, featureNames)
    X = zeros(1, numel(featureNames));
    for i = 1:numel(featureNames)
        fn = char(featureNames(i));
        if isfield(featStruct, fn)
            X(i) = double(featStruct.(fn));
        end
    end

    [lbl, score] = autosimPredictGaussianNB(model, X);
    predLabel = lbl(1);
    predScore = score(1);
end


function model = autosimTrainGaussianNB(X, y, featureNames)
    X = autosimSanitize(X);
    cls = unique(y);
    nClass = numel(cls);
    nFeat = size(X, 2);

    mu = zeros(nClass, nFeat);
    sigma2 = ones(nClass, nFeat);
    prior = zeros(nClass, 1);

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
    model.placeholder = false;
end


function [predLabel, predScore] = autosimPredictGaussianNB(model, X)
    X = autosimSanitize(X);
    if isrow(X)
        X = reshape(X, 1, []);
    end

    n = size(X, 1);
    c = numel(model.class_names);
    logp = zeros(n, c);

    for i = 1:c
        mu = model.mu(i,:);
        s2 = model.sigma2(i,:);
        lp = -0.5 * sum(log(2*pi*s2) + ((X - mu).^2) ./ s2, 2);
        lp = lp + log(max(model.prior(i), eps));
        logp(:,i) = lp;
    end

    [mx, idx] = max(logp, [], 2);
    predLabel = strings(n,1);
    for k = 1:n
        predLabel(k) = string(model.class_names(idx(k)));
    end

    lse = zeros(n,1);
    for k = 1:n
        a = logp(k,:) - mx(k);
        lse(k) = mx(k) + log(sum(exp(a)));
    end
    predScore = exp(mx - lse);
end


function x = autosimSanitize(x)
    x(~isfinite(x)) = 0.0;
end


function v = autosimToNumeric(x)
    if isnumeric(x)
        v = double(x);
    elseif islogical(x)
        v = double(x);
    elseif isstring(x)
        v = double(str2double(x));
        v(~isfinite(v)) = 0.0;
    else
        v = zeros(numel(x),1);
    end

    if isrow(v)
        v = v.';
    end
    v(~isfinite(v)) = 0.0;
end


function autosimSaveCheckpoint(cfg, results, traceStore, learningHistory, model, runStatus, reason)
    try
        summaryTbl = autosimSummaryTable(results);
        checkpoint = struct();
        checkpoint.reason = string(reason);
        checkpoint.timestamp = string(datetime('now'));
        checkpoint.run_status = string(runStatus);
        checkpoint.n_rows = height(summaryTbl);

        save(cfg.persistence.checkpoint_mat, 'results', 'summaryTbl', 'traceStore', 'learningHistory', 'model', 'checkpoint');
        writetable(summaryTbl, cfg.persistence.checkpoint_csv);
        if ~isempty(traceStore)
            writetable(traceStore, cfg.persistence.trace_csv);
        end
    catch ME
        warning('[AUTOSIM] Checkpoint save failed: %s', ME.message);
    end
end


function finalInfo = autosimFinalize(cfg, results, traceStore, learningHistory, model, plotState, runStatus)
    ts = autosimTimestamp();
    tag = lower(char(runStatus));

    summaryTbl = autosimSummaryTable(results);

    datasetMat = fullfile(cfg.paths.data_dir, sprintf('autosim_dataset_%s_%s.mat', ts, tag));
    datasetCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_dataset_%s_%s.csv', ts, tag));
    traceCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_trace_%s_%s.csv', ts, tag));
    historyCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_learning_%s_%s.csv', ts, tag));
    modelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_final_%s_%s.mat', ts, tag));
    plotPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_result_%s_%s.png', ts, tag));
    perfCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_performance_%s_%s.csv', ts, tag));
    decisionCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_decision_metrics_%s_%s.csv', ts, tag));
    perfPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_performance_%s_%s.png', ts, tag));
    gtPredPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_gt_vs_pred_%s_%s.png', ts, tag));

    save(datasetMat, 'results', 'summaryTbl', 'traceStore', 'learningHistory', 'model', 'runStatus');
    writetable(summaryTbl, datasetCsv);
    if ~isempty(traceStore)
        writetable(traceStore, traceCsv);
    end
    if ~isempty(learningHistory)
        writetable(learningHistory, historyCsv);
    end
    save(modelPath, 'model');

    try
        if ~isempty(plotState) && isfield(plotState, 'fig') && isgraphics(plotState.fig)
            exportgraphics(plotState.fig, plotPng, 'Resolution', 150);
        end
    catch ME
        warning('[AUTOSIM] Plot save failed: %s', ME.message);
    end

    try
        autosimSaveScenarioPerformanceReport(summaryTbl, traceStore, perfCsv, decisionCsv, perfPng);
    catch ME
        warning('[AUTOSIM] Performance report save failed: %s', ME.message);
    end

    try
        autosimPlotGtVsPrediction(summaryTbl, model, cfg, gtPredPng);
    catch ME
        warning('[AUTOSIM] GT-vs-pred plot failed: %s', ME.message);
    end

    valid = false(height(summaryTbl), 1);
    if ismember('label', summaryTbl.Properties.VariableNames)
        valid = (summaryTbl.label == "stable") | (summaryTbl.label == "unstable");
    end
    nValid = sum(valid);
    nStable = 0;
    stableRatio = 0.0;
    if nValid > 0
        nStable = sum(summaryTbl.label(valid) == "stable");
        stableRatio = nStable / nValid;
    end

    fprintf('[AUTOSIM] Saved dataset: %s\n', datasetCsv);
    fprintf('[AUTOSIM] Saved trace:   %s\n', traceCsv);
    fprintf('[AUTOSIM] Saved model:   %s\n', modelPath);
    fprintf('[AUTOSIM] Saved plot:    %s\n', plotPng);
    fprintf('[AUTOSIM] Saved perf:    %s\n', perfCsv);
    fprintf('[AUTOSIM] Saved dmetric: %s\n', decisionCsv);
    fprintf('[AUTOSIM] Saved perfpng: %s\n', perfPng);
    fprintf('[AUTOSIM] Saved gtpred:  %s\n', gtPredPng);

    dEval = autosimEvaluateDecisionMetrics(summaryTbl);
    if dEval.n_valid > 0
        fprintf('[AUTOSIM] Decision metrics | Acc=%.3f Prec=%.3f Rec=%.3f Unsafe=%.3f F1=%.3f\n', ...
            dEval.accuracy, dEval.precision, dEval.recall, dEval.unsafe_landing_rate, dEval.f1);
    end

    finalInfo = struct();
    finalInfo.hasValidLabel = (nValid > 0);
    finalInfo.nValid = nValid;
    finalInfo.nStable = nStable;
    finalInfo.stableRatio = stableRatio;
end


function autosimPrintStats(results, idx, totalCount, learnInfo)
    labels = strings(numel(results), 1);
    for i = 1:numel(results)
        labels(i) = string(results(i).label);
    end

    nStable = sum(labels == "stable");
    nUnstable = sum(labels == "unstable");
    nValid = nStable + nUnstable;

    if nValid > 0
        fprintf('[AUTOSIM] Stats %d/%d: stable=%d (%.1f%%), unstable=%d (%.1f%%)\n', ...
            idx, totalCount, nStable, 100*nStable/nValid, nUnstable, 100*nUnstable/nValid);
    else
        fprintf('[AUTOSIM] Stats %d/%d: no valid labels\n', idx, totalCount);
    end

    if learnInfo.model_updated
        fprintf('[AUTOSIM] Model updated with n=%d, stableRatio=%.2f, path=%s\n', ...
            learnInfo.n_train, learnInfo.stable_ratio, learnInfo.model_path);
    else
        if isfield(learnInfo, 'skip_reason') && strlength(string(learnInfo.skip_reason)) > 0
            fprintf('[AUTOSIM] Model update skipped (%s): n=%d, stable=%d, unstable=%d\n', ...
                string(learnInfo.skip_reason), learnInfo.n_train, learnInfo.n_stable, learnInfo.n_unstable);
        else
            fprintf('[AUTOSIM] Model not updated yet (waiting for labeled scenario data).\n');
        end
    end
end


function plotState = autosimInitPlots()
    plotState = struct();
    plotState.fig = figure('Name', 'AutoSim Decision Progress', 'NumberTitle', 'off');
    set(plotState.fig, 'CloseRequestFcn', @(src, evt) autosimHandleStopFigureClose(src, "progress_plot_closed"));
    autosimPlaceFigureRight(plotState.fig, [0.36, 0.46], [0.52, 0.51]);
    tl = tiledlayout(plotState.fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    plotState.ax1 = ax1;
    set(ax1, 'FontSize', 9);
    plotState.accLine = animatedline(ax1, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5);
    plotState.precLine = animatedline(ax1, 'Color', [0.20 0.60 0.20], 'LineWidth', 1.5);
    plotState.recLine = animatedline(ax1, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
    plotState.unsafeLine = animatedline(ax1, 'Color', [0.75 0.20 0.20], 'LineWidth', 1.6);
    title(ax1, 'Decision Metrics Trend', 'FontSize', 10);
    xlabel(ax1, 'scenario', 'FontSize', 9);
    ylabel(ax1, 'score', 'FontSize', 9);
    ylim(ax1, [0 1]);
    grid(ax1, 'on');
    legend(ax1, {'accuracy', 'precision', 'recall', 'unsafe rate'}, 'Location', 'best', 'FontSize', 8);

    ax2 = nexttile(tl, 2);
    plotState.ax2 = ax2;
    set(ax2, 'FontSize', 9);
    plotState.tpLine = animatedline(ax2, 'Color', [0.20 0.60 0.20], 'LineWidth', 1.4);
    plotState.fpLine = animatedline(ax2, 'Color', [0.75 0.20 0.20], 'LineWidth', 1.4);
    plotState.fnLine = animatedline(ax2, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.4);
    plotState.tnLine = animatedline(ax2, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
    title(ax2, 'Decision Confusion Counts', 'FontSize', 10);
    xlabel(ax2, 'scenario', 'FontSize', 9);
    ylabel(ax2, 'count', 'FontSize', 9);
    grid(ax2, 'on');
    legend(ax2, {'TP', 'FP', 'FN', 'TN'}, 'Location', 'best', 'FontSize', 8);
end


function plotState = autosimUpdatePlots(plotState, results, learningHistory)
    if isempty(plotState) || ~isfield(plotState, 'fig') || ~isgraphics(plotState.fig)
        return;
    end

    sIdx = numel(results);
    summaryTbl = autosimSummaryTable(results);
    dEval = autosimEvaluateDecisionMetrics(summaryTbl);

    if dEval.n_valid > 0
        addpoints(plotState.accLine, sIdx, dEval.accuracy);
        addpoints(plotState.precLine, sIdx, dEval.precision);
        addpoints(plotState.recLine, sIdx, dEval.recall);
        addpoints(plotState.unsafeLine, sIdx, dEval.unsafe_landing_rate);

        addpoints(plotState.tpLine, sIdx, dEval.tp);
        addpoints(plotState.fpLine, sIdx, dEval.fp);
        addpoints(plotState.fnLine, sIdx, dEval.fn);
        addpoints(plotState.tnLine, sIdx, dEval.tn);
    end

    drawnow limitrate nocallbacks;
end


function viz = autosimInitScenarioRealtimePlot(cfg, scenarioId, scenarioCfg)
    figName = 'AutoSim Live XY + Wind';
    fig = findobj('Type', 'figure', 'Name', figName);
    if isempty(fig) || ~isgraphics(fig)
        fig = figure('Name', figName, 'NumberTitle', 'off');
    else
        figure(fig);
        clf(fig);
    end
    set(fig, 'CloseRequestFcn', @(src, evt) autosimHandleStopFigureClose(src, sprintf("scenario_plot_closed_s%03d", scenarioId)));
    autosimPlaceFigureRight(fig, [0.42, 0.56], [0.55, 0.44]);

    ax = axes(fig);
    set(ax, 'FontSize', 9);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');

    halfW = max(0.5, cfg.ontology.landing_area_size(1) / 2.0);
    halfH = max(0.5, cfg.ontology.landing_area_size(2) / 2.0);
    margin = 2.0;
    xlim(ax, [-halfW - margin, halfW + margin]);
    ylim(ax, [-halfH - margin, halfH + margin]);

    rectangle(ax, 'Position', [-halfW, -halfH, 2*halfW, 2*halfH], ...
        'EdgeColor', [0.10 0.55 0.20], 'LineWidth', 1.6, 'LineStyle', '--');
    plot(ax, 0.0, 0.0, 'p', 'Color', [0.10 0.55 0.20], 'MarkerSize', 10, 'LineWidth', 1.1);

    vizTrail = animatedline(ax, 'Color', [0.10 0.35 0.90], 'LineWidth', 1.1);
    vizDrone = plot(ax, nan, nan, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.95 0.25 0.20], ...
        'MarkerEdgeColor', [0.40 0.05 0.05]);
    vizWind = quiver(ax, nan, nan, 0.0, 0.0, 0.0, 'Color', [0.95 0.60 0.10], ...
        'LineWidth', 1.8, 'MaxHeadSize', 1.8);

    title(ax, sprintf('Scenario %03d Live View (hover=%.2fm, cmd wind=%.2f@%.1fdeg)', ...
        scenarioId, scenarioCfg.hover_height_m, scenarioCfg.wind_speed, scenarioCfg.wind_dir), 'FontSize', 10);
    xlabel(ax, 'x [m]', 'FontSize', 9);
    ylabel(ax, 'y [m]', 'FontSize', 9);
    legend(ax, {'Landing pad center', 'Drone trail', 'Drone', 'Wind vector'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal', 'FontSize', 8);

    vizInfo = text(ax, 0.02, 0.98, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontName', 'Courier New', 'FontSize', 8, 'Color', [0.15 0.15 0.15]);

    viz = struct();
    viz.fig = fig;
    viz.ax = ax;
    viz.trail = vizTrail;
    viz.drone = vizDrone;
    viz.wind = vizWind;
    viz.info = vizInfo;
end


function autosimUpdateScenarioRealtimePlot(viz, x, y, windSpeed, windDirDeg, tSec, phase, z, inferTxt, predStableProb, decisionTxt)
    if isempty(viz) || ~isfield(viz, 'fig') || ~isgraphics(viz.fig)
        return;
    end

    if ~isfinite(x) || ~isfinite(y)
        return;
    end

    if nargin < 9 || strlength(string(inferTxt)) == 0
        inferTxt = "NO-LAND";
    end
    if nargin < 10
        predStableProb = nan;
    end
    if nargin < 11 || strlength(string(decisionTxt)) == 0
        decisionTxt = "";
    end

    addpoints(viz.trail, x, y);
    set(viz.drone, 'XData', x, 'YData', y);

    ws = windSpeed;
    wd = windDirDeg;
    if ~isfinite(ws)
        ws = 0.0;
    end
    if ~isfinite(wd)
        wd = 0.0;
    end

    arrowLen = max(0.15, min(2.0, 0.25 * abs(ws)));
    u = arrowLen * cosd(wd);
    v = arrowLen * sind(wd);
    set(viz.wind, 'XData', x, 'YData', y, 'UData', u, 'VData', v);

    if isfinite(predStableProb)
        predTxt = sprintf('%.2f', predStableProb);
    else
        predTxt = 'nan';
    end

    set(viz.info, 'String', sprintf('t=%.1fs | phase=%s | z=%.2fm | infer=%s | p_stable=%s | decision=%s | wind=%.2f m/s @ %.1fdeg', ...
        tSec, string(phase), z, string(inferTxt), predTxt, string(decisionTxt), ws, wd));

    drawnow limitrate nocallbacks;
end


function tbl = autosimEmptyTraceTable(scenarioId)
    tbl = table();
    tbl.scenario_id = scenarioId;
    tbl.t_sec = nan;
    tbl.x = nan;
    tbl.y = nan;
    tbl.z = nan;
    tbl.vz = nan;
    tbl.speed_abs = nan;
    tbl.roll_deg = nan;
    tbl.pitch_deg = nan;
    tbl.tag_error = nan;
    tbl.wind_speed = nan;
    tbl.wind_cmd_speed = nan;
    tbl.wind_cmd_dir = nan;
    tbl.state = nan;
    tbl.imu_ang_vel = nan;
    tbl.imu_lin_acc = nan;
    tbl.contact_force = nan;
    tbl.arm_force_fl = nan;
    tbl.arm_force_fr = nan;
    tbl.arm_force_rl = nan;
    tbl.arm_force_rr = nan;
    tbl.pred_stable_prob = nan;
    tbl.decision = "";
    tbl.control_phase = "";
    tbl.semantic_wind_risk = "";
    tbl.semantic_environment = "";
    tbl.semantic_drone_state = "";
    tbl.semantic_alignment = "";
    tbl.semantic_visual = "";
    tbl.semantic_context = "";
    tbl.semantic_relation = "";
    tbl.semantic_integration = "";
    tbl.semantic_safe = nan;
    tbl.landing_feasibility = nan;
    tbl.sem_wind_speed = nan;
    tbl.sem_wind_dir_norm = nan;
    tbl.sem_roll_abs = nan;
    tbl.sem_pitch_abs = nan;
    tbl.sem_tag_u = nan;
    tbl.sem_tag_v = nan;
    tbl.sem_jitter = nan;
    tbl.sem_stability_score = nan;
    tbl.sem_wind_risk_enc = nan;
    tbl.sem_alignment_enc = nan;
    tbl.sem_visual_enc = nan;
    tbl.sem_context_enc = nan;
    tbl.scenario_policy = "exploit";
    tbl.pred_decision = "abort";
    tbl.executed_action = "abort";
    tbl.gt_safe_to_land = "unstable";
    tbl.decision_outcome = "TN";
    tbl.final_label = "unstable";
end


function autosimCleanupProcesses(cfg, launchPid)
    if nargin < 2
        launchPid = -1;
    end

    if isfinite(launchPid) && launchPid > 1
        autosimKillTree(launchPid);
    end

    preSnap = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(preSnap)) > 0
        fprintf('[AUTOSIM] Cleanup pre-snapshot:\n%s\n', preSnap);
    end

    for pass = 1:3
        system(['bash -i -c "set +m; ' ...
            'pgrep -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" | xargs -r kill -9 || true; ' ...
            'pkill -9 -f \"[r]os2 launch sjtu_drone_bringup\" || true; ' ...
            'pkill -9 -f \"[s]jtu_drone_bringup.launch.py\" || true; ' ...
            'pkill -9 -f \"[c]omponent_container\" || true; ' ...
            'pkill -9 -f \"[a]priltag\" || true; ' ...
            'pkill -9 -f \"[s]tatic_transform_publisher\" || true; ' ...
            'pkill -9 -f \"[r]obot_state_publisher\" || true; ' ...
            'pkill -9 -f \"[j]oint_state_publisher\" || true; ' ...
            'pkill -9 -f \"[s]pawn_drone\" || true; ' ...
            'pkill -9 -f \"[t]eleop_joystick\" || true; ' ...
            'pkill -9 -f \"[t]eleop_node\" || true; ' ...
            'pkill -9 -f \"[s]jtu_drone_control.*teleop\" || true; ' ...
            'pkill -9 -f \"[j]oy_node\" || true; ' ...
            'pkill -9 -f \"[g]azebo_wind_plugin_node\" || true; ' ...
            'pkill -9 gzserver || true; ' ...
            'pkill -9 gzclient || true; ' ...
            'pkill -9 -x joy_node || true; ' ...
            'pkill -9 -x teleop_node || true; ' ...
            'pkill -9 -x static_transform_publisher || true; ' ...
            'pkill -9 -x robot_state_publisher || true; ' ...
            'pkill -9 -x joint_state_publisher || true; ' ...
            'pkill -9 -x rviz2 || true; ' ...
            'pkill -9 -f \"[r]viz2\" || true" 2>/dev/null']);
        pause(0.25 * pass);
    end

    autosimKillActiveProcessTrees();
    autosimRefreshRos2Daemon();

    verifyTimeout = 8.0;
    if isfield(cfg, 'process') && isfield(cfg.process, 'cleanup_verify_timeout_sec') && isfinite(cfg.process.cleanup_verify_timeout_sec)
        verifyTimeout = max(1.0, cfg.process.cleanup_verify_timeout_sec);
    end
    autosimWaitForProcessCleanup(verifyTimeout);

    % Final hard pass for frequent duplicate-node offenders.
    system(['bash -i -c "set +m; ' ...
        'pkill -9 -x joy_node || true; ' ...
        'pkill -9 -x teleop_node || true; ' ...
        'pkill -9 -x static_transform_publisher || true; ' ...
        'pkill -9 -x robot_state_publisher || true; ' ...
        'pkill -9 -x joint_state_publisher || true; ' ...
        'pkill -9 -x rviz2 || true" 2>/dev/null']);
    autosimWaitForProcessCleanup(2.0);

    postSnap = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(postSnap)) > 0
        fprintf('[AUTOSIM] Cleanup post-snapshot (still alive):\n%s\n', postSnap);
    end

    pause(max(0.2, cfg.process.kill_settle_sec));
end


function autosimKillTree(pid)
    if ~isfinite(pid) || pid <= 1
        return;
    end
    cmd = sprintf('bash -i -c "pkill -9 -P %d >/dev/null 2>&1 || true; kill -9 %d >/dev/null 2>&1 || true"', round(pid), round(pid));
    system(cmd);
end


function autosimWaitForProcessCleanup(timeoutSec)
    t0 = tic;
    while toc(t0) <= timeoutSec
        snap = autosimGetActiveProcessSnapshot();
        if strlength(strtrim(snap)) == 0
            return;
        end
        autosimKillActiveProcessTrees();
        pause(0.25);
    end
end


function autosimKillActiveProcessTrees()
    pids = autosimGetActiveProcessPids();
    if isempty(pids)
        return;
    end
    for i = 1:numel(pids)
        autosimKillTree(pids(i));
    end
end


function pids = autosimGetActiveProcessPids()
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


function out = autosimGetActiveProcessSnapshot()
    cmd = ['bash -i -c "' ...
        'pgrep -af \"[r]os2 launch sjtu_drone_bringup|[c]omponent_container|[a]priltag|[j]oint_state_publisher|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]viz2|[j]oy_node|[g]azebo|[g]zserver|[g]zclient|[s]pawn_drone|[g]azebo_wind_plugin_node\" ' ...
        '| sed -n \"1,120p\" || true"'];
    [~, txt] = system(cmd);
    out = string(txt);
end


function autosimRefreshRos2Daemon()
    system('bash -i -c "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; ros2 daemon stop >/dev/null 2>&1 || true; ros2 daemon start >/dev/null 2>&1 || true"');
end


function x = autosimTryReceive(sub, timeout)
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end


function [ws, wd] = autosimParseWindConditionMsg(msg)
    ws = nan;
    wd = nan;
    try
        d = double(msg.data);
        if numel(d) >= 1
            ws = d(1);
        end
        if numel(d) >= 2
            wd = autosimWrapTo180(d(2));
        end
    catch
    end
end


function [hist, count] = autosimPushScalarHist(hist, count, x)
    if ~isfinite(x)
        return;
    end

    hist(1:end-1) = hist(2:end);
    hist(end) = x;
    count = min(numel(hist), count + 1);
end


function m = autosimCircularMeanDeg(thetaDeg)
    th = thetaDeg(isfinite(thetaDeg));
    if isempty(th)
        m = nan;
        return;
    end

    c = mean(cosd(th));
    s = mean(sind(th));
    m = autosimWrapTo180(rad2deg(atan2(s, c)));
end


function [windSpeed, windDir] = autosimPickScenarioWind(cfg, scenarioId)
    windSpeed = autosimRandRange(cfg.wind.speed_min, cfg.wind.speed_max);
    windDir = autosimRandRange(cfg.wind.direction_min, cfg.wind.direction_max);

    src = "random";
    if isfield(cfg.wind, 'source')
        src = string(cfg.wind.source);
    end

    if src ~= "kma_csv"
        return;
    end

    profile = autosimGetKmaWindProfile(cfg);
    if isempty(profile)
        return;
    end

    profileN = numel(profile.speed_sec);
    if profileN < 1
        return;
    end

    idx = mod(max(1, scenarioId) - 1, profileN) + 1;
    windSpeed = profile.speed_sec(idx);
    windDir = profile.dir_sec(idx);
end


function offsetSec = autosimPickScenarioWindOffsetSec(cfg, scenarioId)
    offsetSec = 0;

    src = "random";
    if isfield(cfg.wind, 'source')
        src = string(cfg.wind.source);
    end
    if src ~= "kma_csv"
        return;
    end

    profile = autosimGetKmaWindProfile(cfg);
    if isempty(profile) || ~isfield(profile, 'speed_sec') || isempty(profile.speed_sec)
        return;
    end

    n = numel(profile.speed_sec);
    step = max(1, floor(n / max(1, cfg.scenario.count)));
    offsetSec = mod(max(1, scenarioId) - 1, n) * step;
    offsetSec = mod(offsetSec, n);
end


function profile = autosimGetKmaWindProfile(cfg)
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
        T = readtable(csvPath, 'PreserveVariableNames', true, 'TextType', 'string');
    catch
        return;
    end

    if isempty(T) || width(T) < 2
        return;
    end

    speedIdx = autosimFindColumnIndex(T.Properties.VariableNames, cfg.wind.kma_speed_column, {'wind_speed','speed','ws','windspd'});
    dirIdx = autosimFindColumnIndex(T.Properties.VariableNames, cfg.wind.kma_direction_column, {'wind_dir','direction','wd','winddir'});

    if speedIdx < 1
        speedIdx = autosimFindColumnIndexContains(T.Properties.VariableNames, {'(m/s)','m/s','speed','windspeed','wind_spd'});
    end
    if dirIdx < 1
        dirIdx = autosimFindColumnIndexContains(T.Properties.VariableNames, {'(deg)','deg','direction','winddir','wind_dir'});
    end

    if speedIdx < 1 || dirIdx < 1
        return;
    end

    timeIdx = autosimFindColumnIndex(T.Properties.VariableNames, cfg.wind.kma_time_column, {'time','timestamp','datetime','date'});
    if timeIdx < 1
        timeIdx = autosimFindColumnIndexContains(T.Properties.VariableNames, {'time','timestamp','date','일시'});
    end

    speed = autosimToNumeric(T{:, speedIdx});
    dir = autosimToNumeric(T{:, dirIdx});

    if timeIdx > 0
        tSec = autosimParseTimeColumnSec(T{:, timeIdx});
    else
        tSec = (0:numel(speed)-1).' * 60.0;
    end

    if numel(tSec) ~= numel(speed)
        tSec = (0:numel(speed)-1).' * 60.0;
    end

    mask = isfinite(speed) & isfinite(dir) & isfinite(tSec);
    tSec = tSec(mask);
    speed = speed(mask);
    dir = dir(mask);
    if isempty(speed) || numel(speed) < 2
        return;
    end

    [tSec, keepIdx] = unique(tSec, 'stable');
    speed = speed(keepIdx);
    dir = dir(keepIdx);
    if numel(speed) < 2
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

    secGrid = (0:1:floor(tSec(end))).';
    if numel(secGrid) < 2
        secGrid = (0:1:60).';
    end

    speedSec = interp1(tSec, speed, secGrid, 'pchip', 'extrap');

    dirCos = cosd(dir);
    dirSin = sind(dir);
    dirCosSec = interp1(tSec, dirCos, secGrid, 'pchip', 'extrap');
    dirSinSec = interp1(tSec, dirSin, secGrid, 'pchip', 'extrap');
    dirSec = atan2d(dirSinSec, dirCosSec);

    noiseStdSpeed = 0.0;
    noiseStdDir = 0.0;
    if isfield(cfg.wind, 'kma_interp_noise_std_speed') && isfinite(cfg.wind.kma_interp_noise_std_speed)
        noiseStdSpeed = max(0.0, cfg.wind.kma_interp_noise_std_speed);
    end
    if isfield(cfg.wind, 'kma_interp_noise_std_dir_deg') && isfinite(cfg.wind.kma_interp_noise_std_dir_deg)
        noiseStdDir = max(0.0, cfg.wind.kma_interp_noise_std_dir_deg);
    end

    if noiseStdSpeed > 0
        speedSec = speedSec + noiseStdSpeed * randn(size(speedSec));
    end
    if noiseStdDir > 0
        dirSec = dirSec + noiseStdDir * randn(size(dirSec));
    end

    speedSec = max(0.0, speedSec);
    dirSec = mod(dirSec + 180.0, 360.0) - 180.0;

    profile = struct( ...
        'speed', speed(:), ...
        'dir', dir(:), ...
        't_sec', tSec(:), ...
        'sec_grid', secGrid(:), ...
        'speed_sec', speedSec(:), ...
        'dir_sec', dirSec(:));
    cachedPath = csvPath;
    cachedProfile = profile;
end


function tSec = autosimParseTimeColumnSec(col)
    n = numel(col);
    tSec = nan(n,1);
    if n < 1
        return;
    end

    try
        if isdatetime(col)
            dt = col(:);
            if ~isempty(dt.TimeZone)
                dt.TimeZone = '';
            end
            tSec = seconds(dt - dt(1));
            return;
        end
    catch
    end

    s = string(col(:));
    s = strtrim(s);

    dt = NaT(n,1);
    try
        dt = datetime(s, 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
    catch
    end
    if any(isnat(dt))
        try
            dt2 = datetime(s, 'InputFormat', 'yyyy-MM-dd HH:mm');
            fillMask = isnat(dt) & ~isnat(dt2);
            dt(fillMask) = dt2(fillMask);
        catch
        end
    end
    if any(isnat(dt))
        try
            dt2 = datetime(s);
            fillMask = isnat(dt) & ~isnat(dt2);
            dt(fillMask) = dt2(fillMask);
        catch
        end
    end

    valid = ~isnat(dt);
    if any(valid)
        baseT = dt(find(valid, 1, 'first'));
        tSec(valid) = seconds(dt(valid) - baseT);
    end
end


function idx = autosimFindColumnIndex(varNames, preferredName, fallbackNames)
    idx = -1;
    if nargin < 3
        fallbackNames = {};
    end

    names = string(varNames);
    if ~isempty(preferredName)
        hit = find(lower(names) == lower(string(preferredName)), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end

    for i = 1:numel(fallbackNames)
        hit = find(lower(names) == lower(string(fallbackNames{i})), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end
end


function idx = autosimFindColumnIndexContains(varNames, tokens)
    idx = -1;
    if nargin < 2 || isempty(tokens)
        return;
    end

    names = lower(string(varNames));
    for i = 1:numel(tokens)
        tok = lower(string(tokens{i}));
        hit = find(contains(names, tok), 1, 'first');
        if ~isempty(hit)
            idx = hit;
            return;
        end
    end
end


function [angVelNorm, linAccNorm] = autosimParseImuMetrics(msg)
    angVelNorm = nan;
    linAccNorm = nan;

    try
        if isprop(msg, 'angular_velocity')
            av = msg.angular_velocity;
        else
            av = msg.angularvelocity;
        end
        ax = double(av.x);
        ay = double(av.y);
        az = double(av.z);
        angVelNorm = sqrt(ax*ax + ay*ay + az*az);
    catch
    end

    try
        if isprop(msg, 'linear_acceleration')
            la = msg.linear_acceleration;
        else
            la = msg.linearacceleration;
        end
        lx = double(la.x);
        ly = double(la.y);
        lz = double(la.z);
        linAccNorm = sqrt(lx*lx + ly*ly + lz*lz);
    catch
    end
end


function [hasContact, totalForce, fFL, fFR, fRL, fRR] = autosimParseContactForces(msg)
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

    nState = numel(states);
    if nState <= 0
        return;
    end

    hasContact = 1;
    totalForce = 0.0;
    fFL = 0.0;
    fFR = 0.0;
    fRL = 0.0;
    fRR = 0.0;

    for i = 1:nState
        st = states(i);

        c1 = "";
        c2 = "";
        try, c1 = string(st.collision1_name); catch, end
        try, c2 = string(st.collision2_name); catch, end
        armKey = autosimContactArmKey(c1 + " " + c2);

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
        switch armKey
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


function key = autosimContactArmKey(nameText)
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


function [speedCmd, dirCmd] = autosimComputeWindCommand(cfg, scenarioCfg, tNow, windArmed)
    if ~windArmed || ~cfg.wind.enable
        speedCmd = 0.0;
        dirCmd = 0.0;
        return;
    end

    baseSpeed = max(0.0, scenarioCfg.wind_speed);
    baseDir = scenarioCfg.wind_dir;

    useProfileDirect = false;
    if isfield(cfg.wind, 'source') && cfg.wind.source == "kma_csv"
        profile = autosimGetKmaWindProfile(cfg);
        if ~isempty(profile) && isfield(profile, 'speed_sec') && ~isempty(profile.speed_sec)
            offsetSec = 0;
            if isfield(scenarioCfg, 'wind_profile_offset_sec') && isfinite(scenarioCfg.wind_profile_offset_sec)
                offsetSec = round(scenarioCfg.wind_profile_offset_sec);
            end
            nSec = numel(profile.speed_sec);
            idx = mod(max(0, round(tNow)) + offsetSec, nSec) + 1;
            baseSpeed = profile.speed_sec(idx);
            baseDir = profile.dir_sec(idx);
            if isfield(cfg.wind, 'kma_use_profile_direct') && cfg.wind.kma_use_profile_direct
                useProfileDirect = true;
            end
        end
    end

    ramp = 1.0;
    if isfield(cfg.wind, 'model_ramp_sec') && isfinite(cfg.wind.model_ramp_sec) && cfg.wind.model_ramp_sec > 0
        ramp = autosimClamp(tNow / cfg.wind.model_ramp_sec, 0.0, 1.0);
    end

    if useProfileDirect
        speedCmd = max(0.0, ramp * baseSpeed);
    else
        gustScale = 1.0;
        if isfield(scenarioCfg, 'gust_amp_scale') && isfinite(scenarioCfg.gust_amp_scale)
            gustScale = max(0.0, scenarioCfg.gust_amp_scale);
        end
        gustAmp = baseSpeed * cfg.wind.model_gust_amp_ratio * gustScale;
        gust = gustAmp * sin(2.0 * pi * cfg.wind.model_gust_freq_hz * tNow);
        noise = cfg.wind.model_noise_std_speed * randn();
        speedCmd = max(0.0, ramp * (baseSpeed + gust + noise));
    end

    if useProfileDirect
        dirCmd = baseDir;
    else
        dirOscAmp = cfg.wind.model_dir_osc_amp_deg;
        dirScale = 1.0;
        if isfield(scenarioCfg, 'dir_osc_scale') && isfinite(scenarioCfg.dir_osc_scale)
            dirScale = max(0.0, scenarioCfg.dir_osc_scale);
        end
        dirOscAmp = dirOscAmp * dirScale;
        if isfield(cfg.wind, 'source') && cfg.wind.source == "kma_csv"
            dirOscAmp = 0.5 * dirOscAmp;
        end
        dirOsc = dirOscAmp * sin(2.0 * pi * cfg.wind.model_dir_osc_freq_hz * tNow + pi/4.0);
        dirNoise = cfg.wind.model_dir_noise_std_deg * randn();
        dirCmd = baseDir + dirOsc + dirNoise;
    end
    dirCmd = mod(dirCmd + 180.0, 360.0) - 180.0;
end


function jitterPx = autosimComputeTagJitterPx(histUV, count, minSamples)
    jitterPx = nan;
    if count < max(minSamples, 2)
        return;
    end

    rows = histUV(end-count+1:end, :);
    rows = rows(all(isfinite(rows), 2), :);
    if size(rows, 1) < max(minSamples, 2)
        return;
    end

    xPx = rows(:,1) * 320.0;
    yPx = rows(:,2) * 240.0;
    dxy = diff([xPx, yPx], 1, 1);
    d = sqrt(sum(dxy.^2, 2));
    if isempty(d)
        return;
    end
    jitterPx = sqrt(mean(d.^2));
end


function s = autosimComputeTagStabilityScore(jitterPx, warnPx, unsafePx)
    if ~isfinite(jitterPx)
        s = 0.0;
        return;
    end
    if jitterPx <= warnPx
        s = 1.0;
        return;
    end
    s = 1.0 - autosimClamp((jitterPx - warnPx) / max(unsafePx - warnPx, 1e-6), 0.0, 1.0);
end


function onto = autosimBuildOntologyState(windObs, droneObs, tagObs, cfg)
    dt = max(1e-3, autosimClampNaN(windObs.dt, 0.2));

    wsHist = windObs.wind_speed_hist(:);
    wdHist = windObs.wind_dir_hist(:);

    repN = max(3, round(cfg.ontology.wind_condition_window_sec / dt));
    wsRep = autosimTail(wsHist, repN);
    wdRep = autosimTail(wdHist, repN);

    windSpeedRep = autosimNanMean(wsRep);
    if ~isfinite(windSpeedRep)
        windSpeedRep = autosimClampNaN(windObs.wind_speed, 0.0);
    end
    windDirRep = autosimCircularMeanDeg(wdRep);
    if ~isfinite(windDirRep)
        windDirRep = autosimClampNaN(windObs.wind_direction, 0.0);
    end

    baseN = max(8, round(cfg.ontology.gust_base_window_sec / dt));
    burstN = max(3, round(cfg.ontology.gust_burst_window_sec / dt));
    wsBase = autosimTail(wsHist, baseN);
    wsBurst = autosimTail(wsHist, burstN);

    vBase = autosimNanMean(wsBase);
    vPeak = autosimNanMax(wsBurst);
    deltaPeak = max(0.0, vPeak - vBase);
    burstDvdt = diff(wsBurst) / dt;
    dvdtPeak = autosimNanMax(abs(burstDvdt));

    gustActive = (deltaPeak >= cfg.ontology.gust_delta_min) || (dvdtPeak >= cfg.ontology.gust_dvdt_min);
    gustIntensity = autosimClamp( ...
        0.60 * autosimNormalize01(deltaPeak, 0.0, cfg.ontology.gust_delta_high) + ...
        0.40 * autosimNormalize01(dvdtPeak, 0.0, cfg.ontology.gust_dvdt_high), 0.0, 1.0);
    if ~gustActive
        gustLevel = 'none';
    elseif gustIntensity < 0.5
        gustLevel = 'weak';
    else
        gustLevel = 'strong';
    end

    onto = struct();
    onto.entities = struct();
    onto.entities.WindCondition = struct( ...
        'wind_speed', windSpeedRep, ...
        'wind_direction', windDirRep);

    onto.entities.Gust = struct( ...
        'active', gustActive, ...
        'intensity', gustIntensity, ...
        'delta_peak', deltaPeak, ...
        'dvdt_peak', dvdtPeak, ...
        'level', gustLevel);

    onto.entities.DroneState = struct( ...
        'position', droneObs.position, ...
        'roll', droneObs.roll, ...
        'pitch', droneObs.pitch, ...
        'abs_attitude', max(abs(droneObs.roll), abs(droneObs.pitch)), ...
        'vz', droneObs.velocity(3));

    onto.entities.TagObservation = struct( ...
        'detected', tagObs.detected, ...
        'u_norm', tagObs.u_norm, ...
        'v_norm', tagObs.v_norm, ...
        'u_pred', tagObs.u_pred, ...
        'v_pred', tagObs.v_pred, ...
        'jitter_px', tagObs.jitter_px, ...
        'stability_score', tagObs.stability_score, ...
        'detection_continuity', tagObs.detection_continuity, ...
        'centered', tagObs.centered);

    onto.entities.LandingContext = struct( ...
        'landing_area_size', cfg.ontology.landing_area_size, ...
        'obstacle_presence', cfg.ontology.obstacle_presence, ...
        'wind_speed_caution', cfg.ontology.wind_caution_speed, ...
        'wind_speed_unsafe', cfg.ontology.wind_unsafe_speed);

    onto.semantic_state = struct();
    onto.semantic_state.Environment = struct( ...
        'wind_speed', windSpeedRep, ...
        'wind_direction', windDirRep, ...
        'gust_active', gustActive, ...
        'gust_intensity', gustIntensity, ...
        'gust_level', string(gustLevel));
    onto.semantic_state.DroneState = struct( ...
        'position', droneObs.position, ...
        'roll', droneObs.roll, ...
        'pitch', droneObs.pitch, ...
        'abs_attitude', max(abs(droneObs.roll), abs(droneObs.pitch)), ...
        'vz', droneObs.velocity(3));
    onto.semantic_state.VisualState = struct( ...
        'detected', tagObs.detected, ...
        'u_norm', tagObs.u_norm, ...
        'v_norm', tagObs.v_norm, ...
        'jitter_px', tagObs.jitter_px, ...
        'stability_score', tagObs.stability_score, ...
        'centered', tagObs.centered);
    onto.semantic_state.LandingContext = struct( ...
        'landing_area_size', cfg.ontology.landing_area_size, ...
        'obstacle_presence', cfg.ontology.obstacle_presence);
end


function semantic = autosimOntologyReasoning(onto, cfg)
    w = onto.entities.WindCondition;
    g = onto.entities.Gust;
    d = onto.entities.DroneState;
    t = onto.entities.TagObservation;
    c = onto.entities.LandingContext;

    condScore = autosimNormalize01(w.wind_speed, c.wind_speed_caution, c.wind_speed_unsafe);
    windRiskRuleEnc = autosimClamp( ...
        0.65 * condScore + 0.35 * g.intensity, 0.0, 1.0);

    if t.detected && isfinite(t.u_norm) && isfinite(t.v_norm)
        errNow = sqrt((t.u_norm - cfg.control.target_u)^2 + (t.v_norm - cfg.control.target_v)^2);
        if isfinite(t.u_pred) && isfinite(t.v_pred)
            errPred = sqrt((t.u_pred - cfg.control.target_u)^2 + (t.v_pred - cfg.control.target_v)^2);
        else
            errPred = errNow;
        end
        detCont = autosimClampNaN(t.detection_continuity, 0.0);
        alignRuleEnc = autosimClamp(1.0 - ( ...
            0.55 * autosimNormalize01(errNow, 0.0, cfg.agent.max_tag_error_before_land) + ...
            0.25 * autosimNormalize01(errPred, 0.0, cfg.agent.max_tag_error_before_land) + ...
            0.20 * (1.0 - detCont)), 0.0, 1.0);
    else
        errNow = nan;
        errPred = nan;
        alignRuleEnc = 0.0;
    end

    jitterN = autosimNormalize01(t.jitter_px, 0.0, cfg.ontology.tag_jitter_unsafe_px);
    stabScore = autosimClampNaN(t.stability_score, 0.0);
    detCont = autosimClampNaN(t.detection_continuity, 0.0);
    visualRuleEnc = autosimClamp( ...
        0.35 * double(t.detected) + 0.25 * (1.0 - jitterN) + 0.25 * stabScore + 0.15 * detCont, 0.0, 1.0);
    if ~t.detected
        visualRuleEnc = 0.6 * visualRuleEnc;
    end

    attStab = 1.0 - autosimNormalize01(d.abs_attitude, 0.0, deg2rad(cfg.thresholds.final_attitude_max_deg));
    riskCtx = ...
        0.40 * windRiskRuleEnc + ...
        0.20 * (1.0 - alignRuleEnc) + ...
        0.20 * (1.0 - visualRuleEnc) + ...
        0.20 * (1.0 - attStab) + ...
        0.40 * double(c.obstacle_presence);
    contextRuleEnc = autosimClamp(1.0 - riskCtx, 0.0, 1.0);

    windRiskEnc = windRiskRuleEnc;
    alignEnc = alignRuleEnc;
    visualEnc = visualRuleEnc;
    contextEnc = contextRuleEnc;

    if isfield(cfg, 'ontology_ai') && isfield(cfg.ontology_ai, 'enable') && cfg.ontology_ai.enable
        aiFeatWind = [ ...
            condScore, ...
            autosimClampNaN(g.intensity, 0.0), ...
            autosimNormalize01(d.abs_attitude, 0.0, deg2rad(cfg.thresholds.final_attitude_max_deg)), ...
            double(t.detected), ...
            detCont, ...
            jitterN, ...
            1.0 - stabScore ...
        ];
        windRiskAiEnc = autosimLinearSigmoid(aiFeatWind, cfg.ontology_ai.wind_w, cfg.ontology_ai.wind_b, 0.5);

        aiFeatAlign = [ ...
            autosimNormalize01(errNow, 0.0, cfg.agent.max_tag_error_before_land), ...
            autosimNormalize01(errPred, 0.0, cfg.agent.max_tag_error_before_land), ...
            detCont, ...
            double(t.detected), ...
            1.0 - jitterN, ...
            stabScore ...
        ];
        alignAiEnc = autosimLinearSigmoid(aiFeatAlign, cfg.ontology_ai.align_w, cfg.ontology_ai.align_b, 0.0);

        aiFeatVisual = [ ...
            double(t.detected), ...
            jitterN, ...
            stabScore, ...
            detCont, ...
            autosimClampNaN(g.intensity, 0.0) ...
        ];
        visualAiEnc = autosimLinearSigmoid(aiFeatVisual, cfg.ontology_ai.visual_w, cfg.ontology_ai.visual_b, 0.0);

        aiFeatContext = [ ...
            windRiskAiEnc, ...
            alignAiEnc, ...
            visualAiEnc, ...
            attStab, ...
            1.0 - double(c.obstacle_presence) ...
        ];
        contextAiEnc = autosimLinearSigmoid(aiFeatContext, cfg.ontology_ai.context_w, cfg.ontology_ai.context_b, 0.5);

        rw = autosimClampNaN(cfg.ontology_ai.rule_weight, 0.60);
        rw = autosimClamp(rw, 0.0, 1.0);
        aw = 1.0 - rw;

        windRiskEnc = autosimClamp(rw * windRiskRuleEnc + aw * windRiskAiEnc, 0.0, 1.0);
        alignEnc = autosimClamp(rw * alignRuleEnc + aw * alignAiEnc, 0.0, 1.0);
        visualEnc = autosimClamp(rw * visualRuleEnc + aw * visualAiEnc, 0.0, 1.0);
        contextEnc = autosimClamp(rw * contextRuleEnc + aw * contextAiEnc, 0.0, 1.0);
    end

    windRisk = autosimRiskLevel3(windRiskEnc);

    if alignEnc >= 0.70
        alignState = 'aligned';
    else
        alignState = 'misaligned';
    end
    if visualEnc >= cfg.ontology.tag_stability_score_warn
        visualState = 'stable';
    else
        visualState = 'unstable';
    end
    if contextEnc >= 0.70
        contextState = 'safe';
    elseif contextEnc >= 0.40
        contextState = 'caution';
    else
        contextState = 'unsafe';
    end

    if windRiskEnc < 0.33
        environmentState = 'favorable';
    elseif windRiskEnc < 0.66
        environmentState = 'moderate';
    else
        environmentState = 'adverse';
    end

    droneVzNorm = autosimNormalize01(abs(d.vz), 0.0, cfg.agent.no_model_max_abs_vz);
    droneStability = autosimClamp(0.65 * attStab + 0.35 * (1.0 - droneVzNorm), 0.0, 1.0);
    if droneStability >= 0.70
        droneState = 'stable';
    elseif droneStability >= 0.40
        droneState = 'recovering';
    else
        droneState = 'unstable';
    end

    if strcmp(environmentState, 'favorable') && strcmp(droneState, 'stable') && strcmp(visualState, 'stable') && strcmp(contextState, 'safe')
        semanticRelation = 'supportive';
    elseif strcmp(contextState, 'unsafe') || strcmp(environmentState, 'adverse') || strcmp(droneState, 'unstable')
        semanticRelation = 'conflicting';
    else
        semanticRelation = 'conditional';
    end

    landingFeasibility = autosimClamp( ...
        0.30 * (1.0 - windRiskEnc) + ...
        0.20 * alignEnc + ...
        0.20 * visualEnc + ...
        0.15 * contextEnc + ...
        0.15 * droneStability, 0.0, 1.0);

    if landingFeasibility >= cfg.agent.semantic_land_threshold && strcmp(contextState, 'safe')
        semanticIntegration = 'clear_to_land';
        finalDecision = 'land';
    elseif landingFeasibility <= cfg.agent.semantic_abort_threshold || strcmp(contextState, 'unsafe')
        semanticIntegration = 'abort_recommended';
        finalDecision = 'abort';
    else
        semanticIntegration = 'monitor_and_reassess';
        finalDecision = 'abort';
    end

    semantic = struct();
    semantic.environment_state = environmentState;
    semantic.drone_state = droneState;
    semantic.wind_risk = windRisk;
    semantic.alignment_state = alignState;
    semantic.visual_state = visualState;
    semantic.landing_context = contextState;
    semantic.semantic_relation = semanticRelation;
    semantic.semantic_integration = semanticIntegration;
    semantic.landing_feasibility = landingFeasibility;
    semantic.final_decision = finalDecision;
    semantic.isSafeForLanding = strcmp(finalDecision, 'land');
    semantic.wind_risk_enc = windRiskEnc;
    semantic.alignment_enc = alignEnc;
    semantic.visual_enc = visualEnc;
    semantic.context_enc = contextEnc;

end


function vec = autosimBuildSemanticFeatures(windObs, droneObs, tagObs, semantic, cfg)
    vec = [ ...
        autosimNormalize01(windObs.wind_speed, 0.0, cfg.wind.speed_max), ...
        autosimWrapTo180(windObs.wind_direction) / 180.0, ...
        autosimNormalize01(abs(droneObs.roll), 0.0, deg2rad(cfg.thresholds.final_attitude_max_deg)*1.5), ...
        autosimNormalize01(abs(droneObs.pitch), 0.0, deg2rad(cfg.thresholds.final_attitude_max_deg)*1.5), ...
        autosimClampNaN(tagObs.u_norm, 0.0), ...
        autosimClampNaN(tagObs.v_norm, 0.0), ...
        autosimNormalize01(tagObs.jitter_px, 0.0, cfg.ontology.tag_jitter_unsafe_px), ...
        autosimClampNaN(tagObs.stability_score, 0.0), ...
        autosimClampNaN(semantic.wind_risk_enc, autosimEncodeCategory(semantic.wind_risk, {'low','medium','high'}, [0.0, 0.5, 1.0], 1.0)), ...
        autosimClampNaN(semantic.alignment_enc, autosimEncodeCategory(semantic.alignment_state, {'aligned','misaligned'}, [1.0, 0.0], 0.0)), ...
        autosimClampNaN(semantic.visual_enc, autosimEncodeCategory(semantic.visual_state, {'stable','unstable'}, [1.0, 0.0], 0.0)), ...
        autosimClampNaN(semantic.context_enc, autosimEncodeCategory(semantic.landing_context, {'safe','caution','unsafe'}, [1.0, 0.5, 0.0], 0.0)) ...
    ];
end


function autosimSaveScenarioPerformanceReport(summaryTbl, traceStore, perfCsvPath, decisionCsvPath, perfPngPath)
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    dTbl = autosimBuildDecisionTable(summaryTbl);
    n = height(dTbl);
    if n == 0
        return;
    end

    perfTbl = dTbl;
    perfTbl.cum_accuracy = nan(n, 1);
    perfTbl.cum_precision = nan(n, 1);
    perfTbl.cum_recall = nan(n, 1);
    perfTbl.cum_unsafe_landing_rate = nan(n, 1);
    perfTbl.cum_f1 = nan(n, 1);

    for i = 1:n
        de = autosimEvaluateDecisionMetrics(dTbl(1:i, :));
        perfTbl.cum_accuracy(i) = de.accuracy;
        perfTbl.cum_precision(i) = de.precision;
        perfTbl.cum_recall(i) = de.recall;
        perfTbl.cum_unsafe_landing_rate(i) = de.unsafe_landing_rate;
        perfTbl.cum_f1(i) = de.f1;
    end

    % Keep per-scenario confidence summary when trace data is available.
    perfTbl.mean_confidence = nan(n,1);
    if ~isempty(traceStore) && ismember('scenario_id', traceStore.Properties.VariableNames) && ismember('pred_stable_prob', traceStore.Properties.VariableNames)
        for i = 1:n
            sid = perfTbl.scenario_id(i);
            v = traceStore.pred_stable_prob(traceStore.scenario_id == sid);
            v = v(isfinite(v));
            if ~isempty(v)
                perfTbl.mean_confidence(i) = mean(v);
            end
        end
    end

    writetable(perfTbl, perfCsvPath);

    dOverall = autosimEvaluateDecisionMetrics(dTbl);
    dOverallTbl = struct2table(dOverall);
    writetable(dOverallTbl, decisionCsvPath);

    fig = figure('Name', 'AutoSim Decision Performance', 'NumberTitle', 'off');
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_accuracy, '-', 'LineWidth', 1.6, 'Color', [0.00 0.45 0.74]);
    hold(ax1, 'on');
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_precision, '-', 'LineWidth', 1.4, 'Color', [0.20 0.60 0.20]);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_recall, '-', 'LineWidth', 1.4, 'Color', [0.85 0.33 0.10]);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_unsafe_landing_rate, '-', 'LineWidth', 1.6, 'Color', [0.75 0.20 0.20]);
    ylim(ax1, [0 1]);
    xlabel(ax1, 'scenario');
    ylabel(ax1, 'score');
    title(ax1, 'Cumulative Decision Metrics');
    legend(ax1, {'accuracy', 'precision', 'recall', 'unsafe rate'}, 'Location', 'best');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    metricVals = [dOverall.accuracy, dOverall.precision, dOverall.recall, dOverall.unsafe_landing_rate];
    b = bar(ax2, metricVals, 0.70);
    b.FaceColor = 'flat';
    b.CData = [ ...
        0.00 0.45 0.74; ...
        0.20 0.60 0.20; ...
        0.85 0.33 0.10; ...
        0.75 0.20 0.20 ...
    ];
    ylim(ax2, [0 1]);
    xticks(ax2, 1:4);
    xticklabels(ax2, {'Accuracy','Precision','Recall','UnsafeRate'});
    ylabel(ax2, 'score');
    title(ax2, sprintf('Overall Decision Metrics (n=%d)', dOverall.n_valid));
    grid(ax2, 'on');

    exportgraphics(fig, perfPngPath, 'Resolution', 150);
end


function autosimPlotGtVsPrediction(summaryTbl, model, cfg, outPngPath)
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    dTbl = autosimBuildDecisionTable(summaryTbl);
    dEval = autosimEvaluateDecisionMetrics(dTbl);
    if dEval.n_valid == 0
        return;
    end

    sid = dTbl.scenario_id;
    gtSafe = dTbl.gt_safe;
    predLand = dTbl.pred_land;

    fig = figure('Name', 'AutoSim GT vs Decision', 'NumberTitle', 'off');
    tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    stem(ax1, sid, double(gtSafe), 'Color', [0.10 0.60 0.10], 'Marker', 'o', 'LineWidth', 1.0);
    hold(ax1, 'on');
    stem(ax1, sid, double(predLand), 'Color', [0.85 0.33 0.10], 'Marker', 'x', 'LineWidth', 1.0);
    ylim(ax1, [-0.1 1.1]);
    yticks(ax1, [0 1]);
    yticklabels(ax1, {'unsafe/hover', 'safe/land'});
    xlabel(ax1, 'scenario');
    ylabel(ax1, 'class');
    title(ax1, 'GT Safety vs Algorithm Decision (Land/Abort)');
    legend(ax1, {'GT safe(1)/unsafe(0)', 'Decision land(1)/abort(0)'}, 'Location', 'southoutside', 'Orientation', 'horizontal');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    cm = [dEval.tp dEval.fp; dEval.fn dEval.tn]; % rows: pred land/abort, cols: actual safe/unsafe
    imagesc(ax2, cm);
    axis(ax2, 'equal');
    axis(ax2, 'tight');
    colormap(ax2, parula(128));
    colorbar(ax2);
    xticks(ax2, [1 2]);
    yticks(ax2, [1 2]);
    xticklabels(ax2, {'Actual Safe', 'Actual Unsafe'});
    yticklabels(ax2, {'Pred Land', 'Pred Abort'});
    title(ax2, sprintf('Confusion Matrix | Acc=%.3f Prec=%.3f Rec=%.3f Unsafe=%.3f', ...
        dEval.accuracy, dEval.precision, dEval.recall, dEval.unsafe_landing_rate));
    for rr = 1:2
        for cc = 1:2
            txtColor = 'w';
            if cm(rr, cc) < max(cm(:)) * 0.45
                txtColor = 'k';
            end
            text(ax2, cc, rr, sprintf('%d', cm(rr, cc)), 'HorizontalAlignment', 'center', 'Color', txtColor, 'FontWeight', 'bold');
        end
    end

    ax3 = nexttile(tl, 3);
    metricVals = [dEval.accuracy, dEval.precision, dEval.recall, dEval.unsafe_landing_rate];
    b = bar(ax3, metricVals, 0.70);
    b.FaceColor = 'flat';
    b.CData = [ ...
        0.00 0.45 0.74; ...
        0.20 0.60 0.20; ...
        0.85 0.33 0.10; ...
        0.75 0.20 0.20 ...
    ];
    ylim(ax3, [0 1]);
    xticks(ax3, 1:4);
    xticklabels(ax3, {'Accuracy','Precision','Recall','UnsafeRate'});
    ylabel(ax3, 'score');
    title(ax3, 'Main Decision Metrics');
    grid(ax3, 'on');

    subtitle(tl, sprintf('Decision-focused evaluation | valid scenarios=%d | TP=%d FP=%d FN=%d TN=%d', ...
        dEval.n_valid, dEval.tp, dEval.fp, dEval.fn, dEval.tn));

    exportgraphics(fig, outPngPath, 'Resolution', 150);
end


function dTbl = autosimBuildDecisionTable(summaryTbl)
    dTbl = table();
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    n = height(summaryTbl);
    sid = summaryTbl.scenario_id;

    gtSafe = false(n, 1);
    gtValid = false(n, 1);
    if ismember('gt_safe_to_land', summaryTbl.Properties.VariableNames)
        gtLbl = string(summaryTbl.gt_safe_to_land);
        gtSafe = (gtLbl == "stable") | (gtLbl == "safe");
        gtValid = (gtLbl == "stable") | (gtLbl == "safe") | (gtLbl == "unstable") | (gtLbl == "unsafe");
    elseif ismember('label', summaryTbl.Properties.VariableNames)
        lbl = string(summaryTbl.label);
        gtSafe = (lbl == "stable");
        gtValid = (lbl == "stable") | (lbl == "unstable");
    elseif ismember('success', summaryTbl.Properties.VariableNames)
        gtSafe = logical(summaryTbl.success);
        gtValid = true(n, 1);
    end

    predLand = false(n, 1);
    predValid = false(n, 1);
    if ismember('pred_decision', summaryTbl.Properties.VariableNames)
        p = string(summaryTbl.pred_decision);
        predLand = (p == "land");
        predValid = (p == "land") | (p == "abort") | (p == "hover");
    elseif ismember('landing_cmd_time', summaryTbl.Properties.VariableNames)
        lct = summaryTbl.landing_cmd_time;
        predLand = isfinite(lct);
        predValid = true(n, 1);
    end

    dTbl.scenario_id = sid;
    dTbl.gt_safe = gtSafe;
    dTbl.pred_land = predLand;
    dTbl.valid = gtValid & predValid;
end


function de = autosimEvaluateDecisionMetrics(inTbl)
    de = struct();
    de.tp = 0;
    de.fp = 0;
    de.fn = 0;
    de.tn = 0;
    de.n_valid = 0;
    de.accuracy = nan;
    de.precision = nan;
    de.recall = nan;
    de.f1 = nan;
    de.unsafe_landing_rate = nan;
    de.risk_score = nan;

    if isempty(inTbl)
        return;
    end

    if ismember('gt_safe', inTbl.Properties.VariableNames) && ismember('pred_land', inTbl.Properties.VariableNames)
        gtSafe = logical(inTbl.gt_safe);
        predLand = logical(inTbl.pred_land);
        if ismember('valid', inTbl.Properties.VariableNames)
            valid = logical(inTbl.valid);
        else
            valid = true(height(inTbl), 1);
        end
    elseif ismember('label', inTbl.Properties.VariableNames)
        dt = autosimBuildDecisionTable(inTbl);
        gtSafe = dt.gt_safe;
        predLand = dt.pred_land;
        valid = dt.valid;
    else
        return;
    end

    de.tp = sum(predLand(valid) & gtSafe(valid));
    de.fp = sum(predLand(valid) & ~gtSafe(valid));
    de.fn = sum(~predLand(valid) & gtSafe(valid));
    de.tn = sum(~predLand(valid) & ~gtSafe(valid));
    de.n_valid = de.tp + de.fp + de.fn + de.tn;

    de.accuracy = autosimSafeDivide(de.tp + de.tn, de.n_valid);
    de.precision = autosimSafeDivide(de.tp, de.tp + de.fp);
    de.recall = autosimSafeDivide(de.tp, de.tp + de.fn);
    if isfinite(de.precision) && isfinite(de.recall) && (de.precision + de.recall) > 0
        de.f1 = 2.0 * de.precision * de.recall / (de.precision + de.recall);
    end
    de.unsafe_landing_rate = autosimSafeDivide(de.fp, de.fp + de.tn);
    de.risk_score = autosimSafeDivide(de.tp + de.tn, de.tp + de.tn + 2*de.fp + de.fn);
end


function v = autosimSafeDivide(num, den)
    if den <= 0
        v = nan;
    else
        v = num / den;
    end
end


function out = autosimClassifyDecisionOutcome(gtSafeLabel, predDecision)
    gtSafe = (string(gtSafeLabel) == "stable") || (string(gtSafeLabel) == "safe");
    predLand = (string(predDecision) == "land");

    if gtSafe && predLand
        out = "TP";
    elseif (~gtSafe) && predLand
        out = "FP";
    elseif gtSafe && (~predLand)
        out = "FN";
    else
        out = "TN";
    end
end


function p = autosimPercentile(x, q)
    x = sort(double(x(:)));
    x = x(isfinite(x));
    if isempty(x)
        p = nan;
        return;
    end
    if q <= 0
        p = x(1);
        return;
    end
    if q >= 100
        p = x(end);
        return;
    end

    idx = 1 + (numel(x)-1) * (q/100.0);
    lo = floor(idx);
    hi = ceil(idx);
    if lo == hi
        p = x(lo);
    else
        a = idx - lo;
        p = (1-a) * x(lo) + a * x(hi);
    end
end


function v = autosimNormalize01(x, xmin, xmax)
    if ~isfinite(x)
        v = 0.0;
        return;
    end
    den = max(xmax - xmin, 1e-6);
    v = autosimClamp((x - xmin) / den, 0.0, 1.0);
end


function y = autosimClampNaN(x, fallback)
    if ~isfinite(x)
        y = fallback;
    else
        y = x;
    end
end


function enc = autosimEncodeCategory(value, categories, encodings, defaultVal)
    enc = defaultVal;
    for i = 1:numel(categories)
        if strcmp(value, categories{i})
            enc = encodings(i);
            return;
        end
    end
end


function level = autosimRiskLevel3(score01)
    s = autosimClampNaN(score01, 1.0);
    if s < 0.33
        level = 'low';
    elseif s < 0.66
        level = 'medium';
    else
        level = 'high';
    end
end


function v = autosimSemGet(semVec, semNames, key, fallback)
    idx = find(semNames == key, 1);
    if isempty(idx) || idx > numel(semVec) || ~isfinite(semVec(idx))
        v = fallback;
    else
        v = double(semVec(idx));
    end
end


function y = autosimLinearSigmoid(x, w, b, fallback)
    xv = double(x(:));
    wv = double(w(:));
    if nargin < 4
        fallback = 0.5;
    end

    if isempty(xv) || isempty(wv) || numel(xv) ~= numel(wv)
        y = autosimClampNaN(fallback, 0.5);
        return;
    end

    if any(~isfinite(xv)) || any(~isfinite(wv)) || ~isfinite(b)
        y = autosimClampNaN(fallback, 0.5);
        return;
    end

    z = sum(xv .* wv) + double(b);
    y = 1.0 / (1.0 + exp(-z));
    y = autosimClamp(y, 0.0, 1.0);
end


function d = autosimWrapTo180(deg)
    d = mod(deg + 180.0, 360.0) - 180.0;
end


function [detected, u, v, err] = autosimParseTag(msg)
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


function [hist, count] = autosimPushTag(hist, count, u, v)
    if ~isfinite(u) || ~isfinite(v)
        return;
    end

    hist(1:end-1,:) = hist(2:end,:);
    hist(end,:) = [u, v];
    count = min(size(hist,1), count + 1);
end


function [ok, uPred, vPred] = autosimPredictTagCenter(hist, count, uNow, vNow, tNow, lastDetectT, horizonSec, timeoutSec, samplePeriodSec, minSamples)
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
    if size(rows,1) < max(2, minSamples)
        return;
    end

    p2 = rows(end, :);
    p1 = rows(end-1, :);
    dt = max(samplePeriodSec, 1e-3);
    vel = (p2 - p1) ./ dt;
    pPred = p2 + vel .* horizonSec;

    uPred = pPred(1);
    vPred = pPred(2);
    ok = isfinite(uPred) && isfinite(vPred);
end


function st = autosimPidInit()
    st = struct('integral', 0.0, 'prev_error', 0.0, 'initialized', false);
end


function [u, st] = autosimPidStep(err, dt, st, kp, ki, kd, iLimit, outLimit)
    if ~st.initialized
        st.prev_error = err;
        st.initialized = true;
    end

    st.integral = autosimClamp(st.integral + err * dt, -abs(iLimit), abs(iLimit));
    derr = (err - st.prev_error) / max(dt, 1e-6);
    st.prev_error = err;

    u = kp * err + ki * st.integral + kd * derr;
    u = autosimClamp(u, -abs(outLimit), abs(outLimit));
end


function y = autosimClamp(x, lo, hi)
    y = min(max(x, lo), hi);
end


function autosimPlaceFigureRight(fig, sizeFracWH, centerFracXY)
    if nargin < 2 || numel(sizeFracWH) ~= 2
        sizeFracWH = [0.40, 0.55];
    end
    if nargin < 3 || numel(centerFracXY) ~= 2
        centerFracXY = [0.75, 0.50];
    end

    try
        scr = get(0, 'ScreenSize');
        w = max(640, round(scr(3) * sizeFracWH(1)));
        h = max(420, round(scr(4) * sizeFracWH(2)));

        cx = scr(1) + round(scr(3) * centerFracXY(1));
        cy = scr(2) + round(scr(4) * centerFracXY(2));
        x = min(max(scr(1), cx - round(w/2)), scr(1) + scr(3) - w);
        y = min(max(scr(2), cy - round(h/2)), scr(2) + scr(4) - h);

        set(fig, 'Units', 'pixels', 'Position', [x, y, w, h]);
    catch
    end
end


function [r,p,y] = autosimQuat2Eul(qwxyz)
    w = qwxyz(1);
    x = qwxyz(2);
    yy = qwxyz(3);
    z = qwxyz(4);

    sinr = 2 * (w*x + yy*z);
    cosr = 1 - 2 * (x*x + yy*yy);
    r = atan2(sinr, cosr);

    sinp = 2 * (w*yy - z*x);
    if abs(sinp) >= 1
        p = sign(sinp) * pi/2;
    else
        p = asin(sinp);
    end

    siny = 2 * (w*z + x*yy);
    cosy = 1 - 2 * (yy*yy + z*z);
    y = atan2(siny, cosy);
end


function out = autosimTail(x, n)
    x = x(:);
    n = min(numel(x), n);
    if n <= 0
        out = [];
    else
        out = x(end-n+1:end);
    end
end


function [trendLast, oscStd, accelRms] = autosimCalcVzMetrics(vzSeq, dt)
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

    trendLast = autosimNanLast(trend(validTrend));
    resid = v(validTrend) - trend(validTrend);
    oscStd = autosimNanStd(resid);

    tVec = trend(validTrend);
    if numel(tVec) >= 2
        acc = diff(tVec) ./ max(dt, 1e-3);
        accelRms = sqrt(mean(acc.^2));
    end
end


function [zOscStd, flipRateHz] = autosimCalcZOscillationMetrics(zSeq, dt)
    zOscStd = nan;
    flipRateHz = nan;

    z = double(zSeq(:));
    z = z(isfinite(z));
    if numel(z) < 4
        return;
    end

    win = max(3, round(0.8 / max(dt, 1e-3)));
    zTrend = movmean(z, win, 'omitnan');
    zResid = z - zTrend;
    zOscStd = std(zResid);

    dz = diff(z);
    sgn = sign(dz);
    sgn = sgn(sgn ~= 0);
    if numel(sgn) < 2
        flipRateHz = 0.0;
        return;
    end

    flips = sum(sgn(2:end) ~= sgn(1:end-1));
    dur = max((numel(z)-1) * dt, 1e-3);
    flipRateHz = flips / dur;
end


function [xyStd, xySpeedRms] = autosimCalcXYMotionMetrics(xSeq, ySeq, dt)
    xyStd = nan;
    xySpeedRms = nan;

    x = double(xSeq(:));
    y = double(ySeq(:));
    valid = isfinite(x) & isfinite(y);
    x = x(valid);
    y = y(valid);
    if numel(x) < 3
        return;
    end

    cx = mean(x);
    cy = mean(y);
    r = sqrt((x - cx).^2 + (y - cy).^2);
    xyStd = std(r);

    vx = diff(x) ./ max(dt, 1e-3);
    vy = diff(y) ./ max(dt, 1e-3);
    vxy = sqrt(vx.^2 + vy.^2);
    xySpeedRms = sqrt(mean(vxy.^2));
end


function s = autosimEscapeDq(x)
    s = strrep(char(string(x)), '"', '\\"');
end


function t = autosimTimestamp()
    t = datestr(now, 'yyyymmdd_HHMMSS');
end


function v = autosimNanMean(x)
    x = double(x(:));
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = mean(x);
    end
end


function v = autosimNanStd(x)
    x = double(x(:));
    x = x(isfinite(x));
    if numel(x) < 2
        v = nan;
    else
        v = std(x);
    end
end


function v = autosimNanMax(x)
    x = double(x(:));
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = max(x);
    end
end


function v = autosimNanLast(x)
    x = double(x(:));
    idx = find(isfinite(x), 1, 'last');
    if isempty(idx)
        v = nan;
    else
        v = x(idx);
    end
end


function x = autosimRandRange(a, b)
    lo = min(a, b);
    hi = max(a, b);
    x = lo + rand() * (hi - lo);
end


function autosimCleanupRosHandles(handles)
    if nargin < 1 || isempty(handles)
        return;
    end

    if ~iscell(handles)
        handles = {handles};
    end

    for i = 1:numel(handles)
        obj = handles{i};
        if isempty(obj)
            continue;
        end

        try
            if isobject(obj)
                delete(obj);
            end
        catch
        end

        handles{i} = [];
        pause(0.01);
    end
end


function autosimClearNode(node)
    try
        autosimCleanupRosHandles({node});
    catch
    end
end


function autosimClearStopRequest()
    try
        setappdata(0, 'AutoSimStopRequested', false);
        setappdata(0, 'AutoSimStopReason', "");
    catch
    end
end


function autosimRequestStop(reason)
    if nargin < 1 || strlength(string(reason)) == 0
        reason = "user_stop_requested";
    end

    try
        setappdata(0, 'AutoSimStopRequested', true);
        setappdata(0, 'AutoSimStopReason', string(reason));
    catch
    end
end


function tf = autosimIsStopRequested()
    tf = false;
    try
        if isappdata(0, 'AutoSimStopRequested')
            tf = logical(getappdata(0, 'AutoSimStopRequested'));
        end
    catch
        tf = false;
    end
end


function reason = autosimGetStopReason()
    reason = "";
    try
        if isappdata(0, 'AutoSimStopReason')
            reason = string(getappdata(0, 'AutoSimStopReason'));
        end
    catch
        reason = "";
    end
end


function autosimHandleStopFigureClose(src, reason)
    autosimRequestStop(reason);
    try
        delete(src);
    catch
    end
end


function tf = autosimIsUserInterrupt(ME)
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
            if autosimIsUserInterrupt(causes{i})
                tf = true;
                return;
            end
        end
    catch
    end
end


function s = autosimEmptyScenarioResult()
    s = struct();
    s.scenario_id = -1;
    s.label = "unstable";
    s.success = false;
    s.failure_reason = "not_run";
    s.exception_message = "";

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

    s.landing_cmd_time = nan;
    s.pred_decision = "abort";
    s.executed_action = "abort";
    s.gt_safe_to_land = "unstable";
    s.decision_outcome = "TN";
    s.semantic_environment = "unknown";
    s.semantic_drone_state = "unknown";
    s.semantic_visual_state = "unknown";
    s.semantic_landing_context = "unknown";
    s.semantic_relation = "unknown";
    s.semantic_integration = "unknown";
    s.landing_feasibility = nan;
    s.scenario_policy = "exploit";
    s.launch_pid = -1;
    s.launch_log = "";
end
