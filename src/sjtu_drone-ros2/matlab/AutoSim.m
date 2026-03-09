% AutoSim.m
% Integrated autonomous simulation + incremental learning pipeline.
% It combines online landing decision, per-scenario validation, and model updates.
%
% Workflow:
% 1) Load latest model if present. If no model exists, start cold and bootstrap from collected labels.
% 2) For each scenario: launch -> control/decision -> collect state/environment -> label success/failure.
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
        fprintf('\n[AUTOSIM] Scenario %d/%d\n', scenarioId, cfg.scenario.count);

        scenarioCfg = autosimBuildScenarioConfig(cfg, scenarioId);
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

autosimCleanupProcesses(cfg);
pause(cfg.process.kill_settle_sec);

finalInfo = autosimFinalize(cfg, results, traceStore, learningHistory, model, plotState, runStatus);

if finalInfo.hasValidLabel
    fprintf('[AUTOSIM] Final stable ratio: %.1f%% (%d/%d)\n', ...
        100.0 * finalInfo.stableRatio, finalInfo.nStable, finalInfo.nValid);
end

fprintf('[AUTOSIM] Completed at %s (status=%s)\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), runStatus);

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
    cfg.paths.data_dir = fullfile(cfg.paths.root, 'data');
    cfg.paths.model_dir = fullfile(cfg.paths.root, 'models');
    cfg.paths.plot_dir = fullfile(cfg.paths.root, 'plots');
    cfg.paths.log_dir = fullfile(cfg.paths.root, 'logs');
    cfg.paths.lock_file = fullfile(cfg.paths.data_dir, 'autosim.lock');
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

    cfg.scenario = struct();
    cfg.scenario.count = 30;
    cfg.scenario.duration_sec = 25.0;
    cfg.scenario.sample_period_sec = 0.2;
    cfg.scenario.post_land_observe_sec = 3.0;
    cfg.scenario.early_stop_after_landing_sec = 1.5;
    cfg.scenario.hover_height_min_m = 1.5;
    cfg.scenario.hover_height_max_m = 2.5;

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
    cfg.wind.start_force_after_hover_sec = 8.0;
    cfg.wind.model_ramp_sec = 2.0;
    cfg.wind.model_gust_amp_ratio = 0.25;
    cfg.wind.model_gust_freq_hz = 0.18;
    cfg.wind.model_noise_std_speed = 0.15;
    cfg.wind.model_dir_noise_std_deg = 2.5;
    cfg.wind.model_dir_osc_amp_deg = 10.0;
    cfg.wind.model_dir_osc_freq_hz = 0.09;

    cfg.control = struct();
    cfg.control.takeoff_retry_sec = 1.0;
    cfg.control.hover_settle_sec = 2.0;
    cfg.control.flying_altitude_threshold = 0.25;
    cfg.control.xy_kp = 1.15;
    cfg.control.xy_ki = 0.001;
    cfg.control.xy_kd = 0.08;
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
    cfg.control.land_forced_timeout_sec = 18.0;

    cfg.agent = struct();
    cfg.agent.enable_model_decision = true;
    cfg.agent.prob_land_threshold = 0.67;
    cfg.agent.min_samples_before_decision = 15;
    cfg.agent.min_altitude_before_land = 0.18;
    cfg.agent.max_tag_error_before_land = 0.35;
    cfg.agent.decision_cooldown_sec = 0.8;
    cfg.agent.block_landing_if_unstable = true;
    cfg.agent.freeze_xy_if_unstable = false;
    cfg.agent.force_guard_landing_on_timeout = false;
    cfg.agent.enforce_inference_only = true;

    cfg.learning = struct();
    cfg.learning.enable = true;
    cfg.learning.save_every_scenario = true;
    cfg.learning.bootstrap_min_samples = 1;
    cfg.learning.tag_lock_error_max = 0.12;
    cfg.learning.tag_lock_hold_sec = 1.2;
    cfg.learning.random_landing_wait_min_sec = 1.0;
    cfg.learning.random_landing_wait_max_sec = 4.0;
    cfg.learning.random_cmd_duration_min_sec = 1.0;
    cfg.learning.random_cmd_duration_max_sec = 3.0;
    cfg.learning.random_xy_cmd_max = 0.35;

    cfg.persistence = struct();
    cfg.persistence.checkpoint_mat = fullfile(cfg.paths.data_dir, 'autosim_checkpoint_latest.mat');
    cfg.persistence.checkpoint_csv = fullfile(cfg.paths.data_dir, 'autosim_dataset_latest.csv');
    cfg.persistence.trace_csv = fullfile(cfg.paths.data_dir, 'autosim_trace_latest.csv');

    cfg.process = struct();
    cfg.process.stop_after_each_scenario = true;
    cfg.process.kill_settle_sec = 2.0;

    cfg.thresholds = struct();
    cfg.thresholds.land_state_value = 0;
    cfg.thresholds.landed_altitude_max_m = 0.30;
    cfg.thresholds.final_speed_max_mps = 0.40;
    cfg.thresholds.final_attitude_max_deg = 15.0;
    cfg.thresholds.final_tag_error_max = 0.35;
    cfg.thresholds.final_stability_std_z_max = 0.18;
    cfg.thresholds.final_stability_std_vz_max = 0.22;

    cfg.model = struct();
    cfg.model.feature_names = [ ...
        "mean_wind_speed", "max_wind_speed", "mean_abs_roll_deg", "mean_abs_pitch_deg", ...
        "mean_abs_vz", "max_abs_vz", "mean_tag_error", "max_tag_error", ...
        "final_altitude", "final_abs_speed", "final_abs_roll_deg", "final_abs_pitch_deg", ...
        "final_tag_error", "stability_std_z", "stability_std_vz", "stability_std_vz_osc", ...
        "touchdown_accel_rms", "contact_count" ...
    ];

    cfg.ontology = struct();
    cfg.ontology.landing_area_size = [3.0, 3.0];
    cfg.ontology.obstacle_presence = false;
    cfg.ontology.tag_jitter_warn_px = 8.0;
    cfg.ontology.tag_jitter_unsafe_px = 20.0;
    cfg.ontology.tag_stability_score_warn = 0.65;
    cfg.ontology.tag_min_samples = 5;
    cfg.ontology.semantic_feature_names = [ ...
        "wind_speed", "wind_dir_norm", "roll_abs", "pitch_abs", ...
        "tag_u", "tag_v", "jitter", "stability_score", ...
        "wind_risk_enc", "alignment_enc", "visual_enc", "context_enc" ...
    ];

    cfg.topics = struct();
    cfg.topics.state = '/drone/state';
    cfg.topics.pose = '/drone/gt_pose';
    cfg.topics.vel = '/drone/gt_vel';
    cfg.topics.tag_state = '/landing_tag_state';
    cfg.topics.wind_condition = '/wind_condition';
    cfg.topics.wind_command = '/wind_command';
    cfg.topics.land_cmd = '/drone/land';
    cfg.topics.takeoff_cmd = '/drone/takeoff';
    cfg.topics.cmd_vel = '/drone/cmd_vel';

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
    dirs = {cfg.paths.data_dir, cfg.paths.model_dir, cfg.paths.plot_dir, cfg.paths.log_dir};
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
    scenarioCfg.hover_height_m = autosimRandRange(cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
    if cfg.wind.enable
        scenarioCfg.wind_speed = autosimRandRange(cfg.wind.speed_min, cfg.wind.speed_max);
        scenarioCfg.wind_dir = autosimRandRange(cfg.wind.direction_min, cfg.wind.direction_max);
    else
        scenarioCfg.wind_speed = 0.0;
        scenarioCfg.wind_dir = 0.0;
    end
end


function info = autosimStartLaunch(cfg, scenarioCfg, scenarioId)
    autosimCleanupProcesses(cfg);

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
    nodeCleanup = onCleanup(@() autosimClearNode(node)); %#ok<NASGU>

    subState = ros2subscriber(node, cfg.topics.state, 'std_msgs/Int8');
    subPose = ros2subscriber(node, cfg.topics.pose, 'geometry_msgs/Pose');
    subVel = ros2subscriber(node, cfg.topics.vel, 'geometry_msgs/Twist');
    subTag = ros2subscriber(node, cfg.topics.tag_state, 'std_msgs/Float32MultiArray');
    subWind = ros2subscriber(node, cfg.topics.wind_condition, 'std_msgs/Float32MultiArray');

    pubWind = ros2publisher(node, cfg.topics.wind_command, 'std_msgs/Float32MultiArray');
    pubTakeoff = ros2publisher(node, cfg.topics.takeoff_cmd, 'std_msgs/Empty');
    pubLand = ros2publisher(node, cfg.topics.land_cmd, 'std_msgs/Empty');
    pubCmd = ros2publisher(node, cfg.topics.cmd_vel, 'geometry_msgs/Twist');

    msgWind = ros2message(pubWind);
    msgTakeoff = ros2message(pubTakeoff);
    msgLand = ros2message(pubLand);
    msgCmd = ros2message(pubCmd);

    sampleN = max(1, floor(cfg.scenario.duration_sec / cfg.scenario.sample_period_sec));

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
    predStableProb = nan(sampleN,1);
    decisionTxt = strings(sampleN,1);
    phaseTxt = strings(sampleN,1);
    semanticWindRisk = strings(sampleN,1);
    semanticAlign = strings(sampleN,1);
    semanticVisual = strings(sampleN,1);
    semanticContext = strings(sampleN,1);
    semanticSafe = false(sampleN,1);
    semFeat = nan(sampleN, numel(cfg.ontology.semantic_feature_names));

    tagHist = nan(cfg.control.tag_history_len, 2);
    tagHistCount = 0;
    lastTagDetectT = -inf;
    lastTagU = nan;
    lastTagV = nan;
    haveLastTag = false;

    pidX = autosimPidInit();
    pidY = autosimPidInit();

    lastTakeoffT = -inf;
    lastWindT = -inf;
    lastDecisionT = -inf;
    lastCtrlT = 0.0;

    controlPhase = "pre_takeoff_stabilize";
    phaseEnterT = 0.0;
    hoverStartT = nan;
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
    landingDescentActive = false;
    tagLostSearchStartT = nan;

    landingSent = false;
    landingSentT = nan;
    landedHoldStartT = nan;
    kLast = 0;

    liveViz = autosimInitScenarioRealtimePlot(cfg, scenarioId, scenarioCfg);

    t0 = tic;
    for k = 1:sampleN
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
        if ~isempty(windMsg)
            windSpeed(k) = autosimParseWind(windMsg);
        end

        windSpNow = windSpeed(k);
        if ~isfinite(windSpNow)
            windSpNow = windCmdSpeed(k);
        end
        if ~isfinite(windSpNow)
            windSpNow = 0.0;
        end

        windDirNow = windCmdDir(k);
        if ~isfinite(windDirNow)
            windDirNow = scenarioCfg.wind_dir;
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

        windObs = struct('wind_speed', windSpNow, 'wind_direction', windDirNow);
        droneObs = struct('position', [xNow; yNow; zNow], 'roll', rollNowRad, 'pitch', pitchNowRad, 'velocity', [0.0; 0.0; vzNow]);
        tagObs = struct('detected', tagDetected, 'u_norm', uTag, 'v_norm', vTag, 'jitter_px', tagJitterPx, ...
            'stability_score', tagStabilityScore, 'centered', tagCentered);

        ontoState = autosimBuildOntologyState(windObs, droneObs, tagObs, cfg);
        semantic = autosimOntologyReasoning(ontoState, cfg);
        semVec = autosimBuildSemanticFeatures(windObs, droneObs, tagObs, semantic, cfg);

        semanticWindRisk(k) = string(semantic.wind_risk);
        semanticAlign(k) = string(semantic.alignment_state);
        semanticVisual(k) = string(semantic.visual_state);
        semanticContext(k) = string(semantic.landing_context);
        semanticSafe(k) = logical(semantic.isSafeForLanding);
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
                        hoverCenterHoldStartT = nan;
                        pidX = autosimPidInit();
                        pidY = autosimPidInit();
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
                    landingDescentActive = true;
                elseif tk >= randomLandingEndT
                    landingDescentActive = true;
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
            tagErr(1:k), windSpeed(1:k), contact(1:k));
        [predLabel, predScore] = autosimPredictModel(model, feat, cfg.model.feature_names);
        if predLabel == "stable"
            predStableProb(k) = predScore;
        else
            predStableProb(k) = 1.0 - predScore;
        end

        modelSaysStable = isfinite(predStableProb(k)) && (predLabel == "stable") && (predStableProb(k) >= cfg.agent.prob_land_threshold);
        modelSaysUnstable = cfg.agent.enable_model_decision && (~modelSaysStable);

        if ~landingSent && cfg.agent.block_landing_if_unstable && modelSaysUnstable
            % Only block landing decision; keep XY correction active unless explicitly frozen.
            if cfg.agent.freeze_xy_if_unstable
                cmdX = 0.0;
                cmdY = 0.0;
            end
            landingDescentActive = false;
            randomLandingPlanned = false;
            if decisionTxt(k) == ""
                decisionTxt(k) = "wait_hover_unstable";
            end
        end

        canLandByModel = cfg.agent.enable_model_decision && ...
            (k >= cfg.agent.min_samples_before_decision) && ...
            modelSaysStable && ...
            isfinite(tagErr(k)) && (tagErr(k) <= cfg.agent.max_tag_error_before_land) && ...
            isfinite(z(k)) && (z(k) >= cfg.agent.min_altitude_before_land) && ...
            ((tk - lastDecisionT) >= cfg.agent.decision_cooldown_sec);

        guardLandingAllowed = ~cfg.agent.block_landing_if_unstable || ~modelSaysUnstable;

        if ~landingSent && canLandByModel
            send(pubLand, msgLand);
            landingSent = true;
            landingSentT = tk;
            lastDecisionT = tk;
            decisionTxt(k) = "land_by_model";
            controlPhase = "landing_observe";
        elseif ~landingSent && ~cfg.agent.enforce_inference_only && guardLandingAllowed && landingDescentActive && isfinite(z(k)) && z(k) <= cfg.control.land_cmd_alt_m
            send(pubLand, msgLand);
            landingSent = true;
            landingSentT = tk;
            decisionTxt(k) = "land_by_altitude_guard";
            controlPhase = "landing_observe";
        elseif ~landingSent && ~cfg.agent.enforce_inference_only && tk >= cfg.control.land_forced_timeout_sec && (guardLandingAllowed || cfg.agent.force_guard_landing_on_timeout)
            landCmd = sprintf('bash -i -c "%s"', autosimEscapeDq(cfg.shell.land_cli_cmd));
            system(landCmd);
            landingSent = true;
            landingSentT = tk;
            decisionTxt(k) = "land_by_timeout_guard";
            controlPhase = "landing_observe";
        elseif ~landingSent && cfg.agent.enforce_inference_only && tk >= cfg.control.land_forced_timeout_sec
            decisionTxt(k) = "wait_inference_no_land";
        elseif ~landingSent && tk >= cfg.control.land_forced_timeout_sec && cfg.agent.block_landing_if_unstable && modelSaysUnstable
            if cfg.agent.freeze_xy_if_unstable
                cmdX = 0.0;
                cmdY = 0.0;
            end
            decisionTxt(k) = "wait_hover_unstable";
        elseif decisionTxt(k) == ""
            decisionTxt(k) = "track";
        end

        phaseTxt(k) = controlPhase;

        if canLandByModel
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

    if kLast <= 0
        kLast = sampleN;
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
    predStableProb = predStableProb(1:kLast);
    decisionTxt = decisionTxt(1:kLast);
    phaseTxt = phaseTxt(1:kLast);
    windCmdSpeed = windCmdSpeed(1:kLast);
    windCmdDir = windCmdDir(1:kLast);
    semanticWindRisk = semanticWindRisk(1:kLast);
    semanticAlign = semanticAlign(1:kLast);
    semanticVisual = semanticVisual(1:kLast);
    semanticContext = semanticContext(1:kLast);
    semanticSafe = semanticSafe(1:kLast);
    semFeat = semFeat(1:kLast, :);

    msgCmd.linear.x = 0.0;
    msgCmd.linear.y = 0.0;
    msgCmd.linear.z = 0.0;
    msgCmd.angular.x = 0.0;
    msgCmd.angular.y = 0.0;
    msgCmd.angular.z = 0.0;
    send(pubCmd, msgCmd);

    if ~landingSent && ~cfg.agent.enforce_inference_only
        send(pubLand, msgLand);
        landingSentT = toc(t0);
    end

    postN = max(1, floor(cfg.scenario.post_land_observe_sec / cfg.scenario.sample_period_sec));
    for m = 1:postN
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
            windSpeed(end+1,1) = autosimParseWind(windMsg); %#ok<AGROW>
        else
            windSpeed(end+1,1) = nan; %#ok<AGROW>
        end

        contact(end+1,1) = 0; %#ok<AGROW>
        predStableProb(end+1,1) = nan; %#ok<AGROW>
        decisionTxt(end+1,1) = "post_observe"; %#ok<AGROW>
        t(end+1,1) = toc(t0); %#ok<AGROW>

        autosimUpdateScenarioRealtimePlot(liveViz, autosimNanLast(xPos), autosimNanLast(yPos), ...
            autosimNanLast(windSpeed), autosimNanLast(windCmdDir), t(end), "post_observe", autosimNanLast(z), ...
            "NO-LAND", nan, "post_observe");

        pause(cfg.scenario.sample_period_sec);
    end

    res = autosimSummarizeAndLabel(cfg, scenarioId, scenarioCfg, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, contact);
    res.landing_cmd_time = landingSentT;

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
    traceTbl.pred_stable_prob = autosimPadLen(predStableProb, n);
    traceTbl.decision = autosimPadLenString(decisionTxt, n);
    traceTbl.control_phase = autosimPadLenString(phaseTxt, n);
    traceTbl.semantic_wind_risk = autosimPadLenString(semanticWindRisk, n);
    traceTbl.semantic_alignment = autosimPadLenString(semanticAlign, n);
    traceTbl.semantic_visual = autosimPadLenString(semanticVisual, n);
    traceTbl.semantic_context = autosimPadLenString(semanticContext, n);
    traceTbl.semantic_safe = autosimPadLen(double(semanticSafe), n);
    for i = 1:numel(cfg.ontology.semantic_feature_names)
        fn = char(cfg.ontology.semantic_feature_names(i));
        traceTbl.(['sem_' fn]) = autosimPadLen(semFeat(:, i), n);
    end
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


function feat = autosimBuildOnlineFeatureVector(z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, contact)
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
end


function out = autosimSummarizeAndLabel(cfg, scenarioId, scenarioCfg, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, bumperContact)
    out = autosimEmptyScenarioResult();
    out.scenario_id = scenarioId;
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

    finalWindow = max(3, round(5.0 / cfg.scenario.sample_period_sec));
    zStable = autosimTail(z, finalWindow);
    vzStable = autosimTail(vz, finalWindow);

    out.stability_std_z = autosimNanStd(zStable);
    out.stability_std_vz = autosimNanStd(vzStable);
    [~, out.stability_std_vz_osc, out.touchdown_accel_rms] = autosimCalcVzMetrics(vzStable, cfg.scenario.sample_period_sec);
    out.contact_count = sum(bumperContact > 0);
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

    passAll = condState && condAlt && condSpeed && condRoll && condPitch && condTag && condStdZ && condStdVz;

    if passAll
        out.label = "stable";
        out.success = true;
        out.failure_reason = "";
    else
        out.label = "unstable";
        out.success = false;
        out.failure_reason = autosimBuildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz);
    end
end


function reason = autosimBuildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz)
    parts = strings(0,1);
    if ~condState, parts(end+1,1) = "state_not_landed"; end %#ok<AGROW>
    if ~condAlt, parts(end+1,1) = "altitude_high"; end %#ok<AGROW>
    if ~condSpeed, parts(end+1,1) = "speed_high"; end %#ok<AGROW>
    if ~condRoll, parts(end+1,1) = "roll_high"; end %#ok<AGROW>
    if ~condPitch, parts(end+1,1) = "pitch_high"; end %#ok<AGROW>
    if ~condTag, parts(end+1,1) = "tag_error_high"; end %#ok<AGROW>
    if ~condStdZ, parts(end+1,1) = "z_unstable"; end %#ok<AGROW>
    if ~condStdVz, parts(end+1,1) = "vz_unstable"; end %#ok<AGROW>

    if isempty(parts)
        reason = "unknown";
    else
        reason = strjoin(parts, ';');
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
    info.model_path = "";

    if sum(valid) < cfg.learning.bootstrap_min_samples
        model = modelPrev;
        return;
    end

    trainTbl = tbl(valid, :);
    y = string(trainTbl.label);
    y(y ~= "stable") = "unstable";

    featNames = cellstr(cfg.model.feature_names);
    X = zeros(height(trainTbl), numel(featNames));
    for i = 1:numel(featNames)
        col = featNames{i};
        if ismember(col, trainTbl.Properties.VariableNames)
            X(:,i) = autosimToNumeric(trainTbl.(col));
        end
    end

    model = autosimTrainGaussianNB(X, y, cfg.model.feature_names);

    ts = autosimTimestamp();
    modelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_%s_s%03d.mat', ts, scenarioId));
    save(modelPath, 'model');

    info.model_updated = true;
    info.model_path = string(modelPath);
    info.stable_ratio = mean(y == "stable");
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
        'stability_std_z','stability_std_vz','stability_std_vz_osc','touchdown_accel_rms','contact_count','final_state', ...
        'landing_cmd_time','launch_pid','launch_log','exception_message'
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
    confHistCsv = fullfile(cfg.paths.data_dir, sprintf('autosim_confidence_hist_%s_%s.csv', ts, tag));
    perfPng = fullfile(cfg.paths.plot_dir, sprintf('autosim_performance_%s_%s.png', ts, tag));

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
        autosimSaveScenarioPerformanceReport(summaryTbl, traceStore, perfCsv, confHistCsv, perfPng);
    catch ME
        warning('[AUTOSIM] Performance report save failed: %s', ME.message);
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
    fprintf('[AUTOSIM] Saved perfpng: %s\n', perfPng);

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
        fprintf('[AUTOSIM] Model not updated yet (waiting for labeled scenario data).\n');
    end
end


function plotState = autosimInitPlots()
    plotState = struct();
    plotState.fig = figure('Name', 'AutoSim Learning Progress', 'NumberTitle', 'off');
    tl = tiledlayout(plotState.fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    plotState.ax1 = ax1;
    plotState.stableLine = animatedline(ax1, 'Color', [0.20 0.60 0.20], 'LineWidth', 1.5);
    plotState.unstableLine = animatedline(ax1, 'Color', [0.75 0.20 0.20], 'LineWidth', 1.5);
    title(ax1, 'Scenario Outcome Trend');
    xlabel(ax1, 'scenario');
    ylabel(ax1, 'count');
    grid(ax1, 'on');
    legend(ax1, {'stable', 'unstable'}, 'Location', 'best');

    ax2 = nexttile(tl, 2);
    plotState.ax2 = ax2;
    plotState.learnNLine = animatedline(ax2, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
    plotState.stableRatioLine = animatedline(ax2, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.4);
    title(ax2, 'Incremental Learning Status');
    xlabel(ax2, 'scenario');
    ylabel(ax2, 'value');
    grid(ax2, 'on');
    legend(ax2, {'train_samples', 'stable_ratio'}, 'Location', 'best');
end


function plotState = autosimUpdatePlots(plotState, results, learningHistory)
    if isempty(plotState) || ~isfield(plotState, 'fig') || ~isgraphics(plotState.fig)
        return;
    end

    sIdx = numel(results);
    labels = strings(numel(results),1);
    for i = 1:numel(results)
        labels(i) = results(i).label;
    end

    nStable = sum(labels == "stable");
    nUnstable = sum(labels == "unstable");

    addpoints(plotState.stableLine, sIdx, nStable);
    addpoints(plotState.unstableLine, sIdx, nUnstable);

    if ~isempty(learningHistory)
        nTrain = learningHistory.n_train(end);
        stableRatio = learningHistory.stable_ratio(end);
        addpoints(plotState.learnNLine, sIdx, nTrain);
        addpoints(plotState.stableRatioLine, sIdx, stableRatio);
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

    ax = axes(fig);
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
    plot(ax, 0.0, 0.0, 'p', 'Color', [0.10 0.55 0.20], 'MarkerSize', 12, 'LineWidth', 1.2);

    vizTrail = animatedline(ax, 'Color', [0.10 0.35 0.90], 'LineWidth', 1.1);
    vizDrone = plot(ax, nan, nan, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.95 0.25 0.20], ...
        'MarkerEdgeColor', [0.40 0.05 0.05]);
    vizWind = quiver(ax, nan, nan, 0.0, 0.0, 0.0, 'Color', [0.95 0.60 0.10], ...
        'LineWidth', 1.8, 'MaxHeadSize', 1.8);

    title(ax, sprintf('Scenario %03d Live View (hover=%.2fm, cmd wind=%.2f@%.1fdeg)', ...
        scenarioId, scenarioCfg.hover_height_m, scenarioCfg.wind_speed, scenarioCfg.wind_dir));
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');
    legend(ax, {'Landing pad center', 'Drone trail', 'Drone', 'Wind vector'}, ...
        'Location', 'northoutside', 'Orientation', 'horizontal');

    vizInfo = text(ax, 0.02, 0.98, '', 'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontName', 'Courier New', 'FontSize', 10, 'Color', [0.15 0.15 0.15]);

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
    tbl.pred_stable_prob = nan;
    tbl.decision = "";
    tbl.control_phase = "";
    tbl.semantic_wind_risk = "";
    tbl.semantic_alignment = "";
    tbl.semantic_visual = "";
    tbl.semantic_context = "";
    tbl.semantic_safe = nan;
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
    tbl.final_label = "unstable";
end


function autosimCleanupProcesses(cfg, launchPid)
    if nargin < 2
        launchPid = -1;
    end

    if isfinite(launchPid) && launchPid > 1
        autosimKillTree(launchPid);
    end

    system(['bash -i -c "set +m; ' ...
        'pkill -9 -f \"[r]os2 launch sjtu_drone_bringup\" || true; ' ...
        'pkill -9 -f \"[s]jtu_drone_bringup.launch.py\" || true; ' ...
        'pkill -9 -f \"[c]omponent_container\" || true; ' ...
        'pkill -9 -f \"[a]priltag\" || true; ' ...
        'pkill -9 -f \"[r]obot_state_publisher\" || true; ' ...
        'pkill -9 -f \"[j]oint_state_publisher\" || true; ' ...
        'pkill -9 gzserver || true; ' ...
        'pkill -9 gzclient || true; ' ...
        'pkill -9 -x rviz2 || true" 2>/dev/null']);

    system('bash -i -c "source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true; ros2 daemon stop >/dev/null 2>&1 || true; ros2 daemon start >/dev/null 2>&1 || true"');

    pause(max(0.2, cfg.process.kill_settle_sec));
end


function autosimKillTree(pid)
    if ~isfinite(pid) || pid <= 1
        return;
    end
    cmd = sprintf('bash -i -c "pkill -9 -P %d >/dev/null 2>&1 || true; kill -9 %d >/dev/null 2>&1 || true"', round(pid), round(pid));
    system(cmd);
end


function x = autosimTryReceive(sub, timeout)
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end


function ws = autosimParseWind(msg)
    ws = nan;
    try
        d = double(msg.data);
        if ~isempty(d)
            ws = d(1);
        end
    catch
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

    ramp = 1.0;
    if isfield(cfg.wind, 'model_ramp_sec') && isfinite(cfg.wind.model_ramp_sec) && cfg.wind.model_ramp_sec > 0
        ramp = autosimClamp(tNow / cfg.wind.model_ramp_sec, 0.0, 1.0);
    end

    gustAmp = baseSpeed * cfg.wind.model_gust_amp_ratio;
    gust = gustAmp * sin(2.0 * pi * cfg.wind.model_gust_freq_hz * tNow);
    noise = cfg.wind.model_noise_std_speed * randn();
    speedCmd = max(0.0, ramp * (baseSpeed + gust + noise));

    dirOsc = cfg.wind.model_dir_osc_amp_deg * sin(2.0 * pi * cfg.wind.model_dir_osc_freq_hz * tNow + pi/4.0);
    dirNoise = cfg.wind.model_dir_noise_std_deg * randn();
    dirCmd = baseDir + dirOsc + dirNoise;
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
    onto = struct();
    onto.entities = struct();
    onto.entities.WindCondition = struct( ...
        'wind_speed', windObs.wind_speed, ...
        'wind_direction', windObs.wind_direction);

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
        'jitter_px', tagObs.jitter_px, ...
        'stability_score', tagObs.stability_score, ...
        'centered', tagObs.centered);

    onto.entities.LandingContext = struct( ...
        'landing_area_size', cfg.ontology.landing_area_size, ...
        'obstacle_presence', cfg.ontology.obstacle_presence, ...
        'wind_speed_caution', cfg.wind.speed_max * 0.5, ...
        'wind_speed_unsafe', cfg.wind.speed_max);
end


function semantic = autosimOntologyReasoning(onto, cfg)
    w = onto.entities.WindCondition;
    d = onto.entities.DroneState;
    t = onto.entities.TagObservation;
    c = onto.entities.LandingContext;

    if w.wind_speed >= c.wind_speed_unsafe || d.abs_attitude > deg2rad(cfg.thresholds.final_attitude_max_deg) * 1.2
        windRisk = 'high';
    elseif w.wind_speed >= c.wind_speed_caution || d.abs_attitude > deg2rad(cfg.thresholds.final_attitude_max_deg)
        windRisk = 'medium';
    else
        windRisk = 'low';
    end

    if t.detected && isfinite(t.u_norm) && isfinite(t.v_norm)
        alignErr = sqrt((t.u_norm - cfg.control.target_u)^2 + (t.v_norm - cfg.control.target_v)^2);
        if alignErr <= cfg.agent.max_tag_error_before_land
            alignState = 'aligned';
        else
            alignState = 'misaligned';
        end
    else
        alignState = 'misaligned';
    end

    if ~t.detected
        visualState = 'unstable';
    elseif isfinite(t.stability_score) && t.stability_score >= cfg.ontology.tag_stability_score_warn
        visualState = 'stable';
    else
        visualState = 'unstable';
    end

    if c.obstacle_presence || strcmp(windRisk, 'high')
        contextState = 'unsafe';
    elseif strcmp(windRisk, 'medium') || strcmp(alignState, 'misaligned') || strcmp(visualState, 'unstable')
        contextState = 'caution';
    else
        contextState = 'safe';
    end

    semantic = struct();
    semantic.wind_risk = windRisk;
    semantic.alignment_state = alignState;
    semantic.visual_state = visualState;
    semantic.landing_context = contextState;
    semantic.isSafeForLanding = strcmp(contextState, 'safe');
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
        autosimEncodeCategory(semantic.wind_risk, {'low','medium','high'}, [0.0, 0.5, 1.0], 1.0), ...
        autosimEncodeCategory(semantic.alignment_state, {'aligned','misaligned'}, [1.0, 0.0], 0.0), ...
        autosimEncodeCategory(semantic.visual_state, {'stable','unstable'}, [1.0, 0.0], 0.0), ...
        autosimEncodeCategory(semantic.landing_context, {'safe','caution','unsafe'}, [1.0, 0.5, 0.0], 0.0) ...
    ];
end


function autosimSaveScenarioPerformanceReport(summaryTbl, traceStore, perfCsvPath, histCsvPath, perfPngPath)
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    n = height(summaryTbl);
    perfTbl = table();
    perfTbl.scenario_id = summaryTbl.scenario_id;
    if ismember('label', summaryTbl.Properties.VariableNames)
        perfTbl.label = summaryTbl.label;
    else
        perfTbl.label = repmat("unknown", n, 1);
    end
    if ismember('success', summaryTbl.Properties.VariableNames)
        perfTbl.success = double(summaryTbl.success);
    else
        perfTbl.success = zeros(n, 1);
    end

    perfTbl.cum_success_rate = nan(n,1);
    for i = 1:n
        perfTbl.cum_success_rate(i) = mean(perfTbl.success(1:i) > 0.5);
    end

    perfTbl.mean_confidence = nan(n,1);
    perfTbl.p10_confidence = nan(n,1);
    perfTbl.p50_confidence = nan(n,1);
    perfTbl.p90_confidence = nan(n,1);

    if ~isempty(traceStore) && ismember('scenario_id', traceStore.Properties.VariableNames) && ismember('pred_stable_prob', traceStore.Properties.VariableNames)
        for i = 1:n
            sid = perfTbl.scenario_id(i);
            v = traceStore.pred_stable_prob(traceStore.scenario_id == sid);
            v = v(isfinite(v));
            if ~isempty(v)
                perfTbl.mean_confidence(i) = mean(v);
                perfTbl.p10_confidence(i) = autosimPercentile(v, 10);
                perfTbl.p50_confidence(i) = autosimPercentile(v, 50);
                perfTbl.p90_confidence(i) = autosimPercentile(v, 90);
            end
        end
    end

    writetable(perfTbl, perfCsvPath);

    confVals = perfTbl.mean_confidence(isfinite(perfTbl.mean_confidence));
    edges = 0:0.05:1;
    centers = edges(1:end-1) + diff(edges)/2;
    counts = histcounts(confVals, edges);
    histTbl = table(centers(:), counts(:), 'VariableNames', {'confidence_bin_center', 'count'});
    writetable(histTbl, histCsvPath);

    fig = figure('Name', 'AutoSim Scenario Performance', 'NumberTitle', 'off');
    tl = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);
    plot(ax1, perfTbl.scenario_id, perfTbl.cum_success_rate, '-o', 'LineWidth', 1.5, 'Color', [0.20 0.60 0.20]);
    ylim(ax1, [0 1]);
    xlabel(ax1, 'scenario');
    ylabel(ax1, 'cumulative success rate');
    title(ax1, 'Success Rate Trend');
    grid(ax1, 'on');

    ax2 = nexttile(tl, 2);
    bar(ax2, centers, counts, 1.0, 'FaceColor', [0.00 0.45 0.74]);
    xlim(ax2, [0 1]);
    xlabel(ax2, 'mean model confidence');
    ylabel(ax2, 'scenario count');
    title(ax2, 'Confidence Distribution');
    grid(ax2, 'on');

    exportgraphics(fig, perfPngPath, 'Resolution', 150);
    close(fig);
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


function autosimClearNode(node)
    try
        clear node;
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
    s.final_state = nan;

    s.landing_cmd_time = nan;
    s.launch_pid = -1;
    s.launch_log = "";
end
