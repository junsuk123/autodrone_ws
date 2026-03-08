% landing_decision_matlab.m
% MATLAB script to run an ontology-based landing decision node for ROS2/Gazebo.
% Features:
% - Optional: start ROS2/Gazebo bringup launch (uses system call set in cfg.launchCMD)
% - Subscribes to /wind_condition (std_msgs/Float32MultiArray) and to drone gt_pose
% - Builds a minimal ontology (WindCondition, DroneState, LandingZone) as structs
% - Implements two simple decision methods: Decision Tree and Bayesian score
% - Publishes decision on /landing_decision (std_msgs/String) with values: "land","wait","caution"
%
% Usage:
% 1) Source ROS2 and workspace in shell or let this script launch it (see cfg.launchCMD)
% 2) From MATLAB: run this script. It will optionally start the launch and then run a loop.
%
% Notes: requires MATLAB ROS2 support (ros2node, ros2subscriber, ros2publisher, ros2message, receive/send)

clear; clc; close all;

% Ensure helper functions in this folder are resolvable regardless of current folder.
thisDir = fileparts(mfilename('fullpath'));
if ~isempty(thisDir)
    addpath(thisDir);
end

%% Configuration
cfg.use_launch = false; % if true the script will attempt to start the configured launch command
cfg.launchCMD = '';
% Example if you want MATLAB to start the ros2 launch (adjust paths):
% cfg.launchCMD = 'bash -lc "source /opt/ros/humble/setup.bash; source /home/user/INCSL/IICC26_ws/install/setup.bash; ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py use_gui:=false &"';

% Topics (tune if your namespace differs)
ns = '/drone'; % model namespace used by the bringup launch
topic_wind = '/wind_condition';                     % Float32MultiArray [speed, direction]
topic_pose = [ns '/gt_pose'];                       % geometry_msgs/Pose
topic_state = [ns '/state'];                        % std_msgs/Int8 (0:landed, 1:flying, 2:takingoff, 3:landing)
topic_takeoff = [ns '/takeoff'];                    % std_msgs/Empty
topic_cmd_vel = [ns '/cmd_vel'];                    % geometry_msgs/Twist
topic_decision = '/landing_decision';               % std_msgs/String
topic_tags = [ns '/bottom/tags'];                   % apriltag_msgs/AprilTagDetectionArray
topic_tag_state = '/landing_tag_state';             % Float32MultiArray from apriltag_state_bridge

% Decision params
params.wind_speed_unsafe = 7.0;   % m/s above -> unsafe
params.wind_speed_caution = 4.0;  % m/s between caution and unsafe
params.max_attitude = deg2rad(10);% roll/pitch limit
params.max_vz_land = 0.5;         % vertical speed limit for safe landing
params.decision_rate = 2.0;       % Hz
params.tag_require_detection = true;
params.tag_target_id_enabled = true;
params.tag_target_id = 0;
params.tag_center_tolerance = 0.35; % normalized frame distance (0=center, 1=edge)
params.tag_jitter_warn_px = 8.0;    % caution threshold
params.tag_jitter_unsafe_px = 20.0; % wait threshold
params.tag_area_jitter_warn_ratio = 0.03; % std(area)/mean(area)
params.tag_area_jitter_unsafe_ratio = 0.08;
params.tag_min_area_px2 = 2000.0;   % too small means landing marker is far/uncertain
params.tag_margin_warn = 30.0;
params.tag_margin_unsafe = 15.0;
params.tag_stability_score_warn = 0.65;
params.tag_stability_score_land = 0.85;
params.tag_min_samples = 5;         % minimum samples before trusting jitter

% Startup + control params (MATLAB node drives takeoff/hover/xy-hold)
params.startup_ready_timeout_sec = 15.0;
params.takeoff_retry_sec = 1.0;
params.hover_settle_sec = 2.0;
params.tag_predict_horizon_sec = 0.15;
params.tag_predict_timeout_sec = 0.6;
params.tag_hold_last_state = true;      % keep last valid tag state during short detection gaps
params.tag_hold_timeout_sec = 0.6;      % max hold duration (sec)
params.pre_takeoff_zero_wind_enabled = true;      % force zero wind before first takeoff command
params.pre_takeoff_zero_wind_settle_sec = 2.0;    % wait after commanding zero wind
params.pre_takeoff_require_tag_centered = false;  % keep takeoff unblocked; enforce centering during hover/xy_hold
params.pre_takeoff_tag_center_tolerance = 0.03;   % normalized distance to image center
params.pre_takeoff_tag_center_hold_sec = 1.0;     % continuous centered time before takeoff
params.xy_hold_enabled = true;
params.xy_pid_kp = 1.2;
params.xy_pid_ki = 0.05;
params.xy_pid_kd = 0.12;
params.xy_pid_integral_limit = 1.0;
params.xy_cmd_limit = 0.7;
params.xy_control_center_deadband = 0.01; % tighter than landing tolerance; used only to zero XY control near center
params.flying_altitude_threshold = 0.20; % fallback flight inference when /state is stale
params.state_stale_timeout_sec = 1.0;    % if no /state update within this window, trust pose fallback
params.wind_start_require_tag_centered = true; % start wind only after hover center lock
params.wind_start_tag_center_hold_sec = 1.0;
% Default signs set for this workspace so positive image error commands move toward the tag.
% If one axis is still reversed on your setup, flip only that axis sign.
params.xy_map_sign_x_from_v = 1.0;  % x command from image v error
params.xy_map_sign_y_from_u = 1.0;  % y command from image u error

% Camera image size for normalized tag center calculations
tag_cfg.image_width = 640;
tag_cfg.image_height = 480;
tag_cfg.history_len = 20;

% Wind publisher integration: if true, MATLAB will generate and publish wind to /wind_command
cfg.start_wind_publisher = true; % enable built-in wind generator
cfg.wind_start_delay_after_hover_sec = 5.0; % start wind N seconds after entering hover_settle
cfg.wind_pub_params = struct();   % see wind_publisher_matlab.startWindPublisher options (rate, steady_speed, etc.)
cfg.wind_pub_params.use_set_wind_service = false;   % publish /wind_command continuously
cfg.wind_pub_params.topic_publish_mode = 'matlab';  % 'matlab' publisher per tick, or 'cli'
cfg.enable_live_plot = true;          % show real-time trends in MATLAB figure
cfg.plot_window_sec = 45;             % sliding window length for live plot
cfg.log_decision_changes_only = true; % reduce console noise
cfg.log_summary_interval_sec = 5.0;   % periodic compact status log
% Optional topic fallback mode inside startWindPublisher:
% cfg.wind_pub_params.topic_publish_mode = 'cli';
% cfg.wind_pub_params.cli_setup_cmd = 'source /home/j/INCSL/IICC26_ws/install/setup.bash';
% When enabled and /set_wind service fails, MATLAB runs:
% ros2 topic pub /wind_command std_msgs/msg/Float32MultiArray "data: [speed, direction]" -1

% Bayesian model params (simple Gaussian assumptions)
bayes.mu_safe = [0.0, 0.0];        % mean [wind_speed, attitude_score]
bayes.sigma_safe = [1.5, 0.5];     % std deviations
bayes.threshold = 0.5;             % probability threshold to consider safe

%% Launch if requested
if cfg.use_launch && ~isempty(cfg.launchCMD)
    fprintf('[MATLAB] Starting configured launch (background)...\n');
    [st,out] = system(cfg.launchCMD);
    if st ~= 0
        warning('Launch command returned non-zero: %s', out);
    else
        pause(2.0); % give launch a moment
    end
end

%% ROS2 node and pubs/subs
try
    node = ros2node('/matlab_landing_decision');
catch ME
    error('Failed to create ros2 node. Make sure ROS2 MATLAB support is installed and ROS2 env is sourced.\nError: %s', ME.message);
end

sub_wind = ros2subscriber(node, topic_wind, 'std_msgs/Float32MultiArray');
sub_pose = ros2subscriber(node, topic_pose, 'geometry_msgs/Pose');
sub_state = ros2subscriber(node, topic_state, 'std_msgs/Int8');

pub_takeoff = ros2publisher(node, topic_takeoff, 'std_msgs/Empty');
pub_cmd = ros2publisher(node, topic_cmd_vel, 'geometry_msgs/Twist');
pub_dec = ros2publisher(node, topic_decision, 'std_msgs/String');
pub_wind_cmd = ros2publisher(node, '/wind_command', 'std_msgs/Float32MultiArray');

% Optional AprilTag detection subscriber for landing zone reasoning.
sub_tags = [];
sub_tag_state = [];
try
    % Prefer canonical ROS2 type string first.
    sub_tags = ros2subscriber(node, topic_tags, 'apriltag_msgs/msg/AprilTagDetectionArray');
    fprintf('[MATLAB] AprilTag subscriber enabled on %s (type=apriltag_msgs/msg/AprilTagDetectionArray)\n', topic_tags);
catch ME
    try
        % Fallback for environments that accept legacy shorthand.
        sub_tags = ros2subscriber(node, topic_tags, 'apriltag_msgs/AprilTagDetectionArray');
        fprintf('[MATLAB] AprilTag subscriber enabled on %s (type=apriltag_msgs/AprilTagDetectionArray)\n', topic_tags);
    catch ME2
        fprintf('[MATLAB] AprilTag custom subscriber unavailable (%s | %s). Trying bridge topic %s.\n', ...
                ME.message, ME2.message, topic_tag_state);
        try
            sub_tag_state = ros2subscriber(node, topic_tag_state, 'std_msgs/Float32MultiArray');
            fprintf('[MATLAB] AprilTag bridge subscriber enabled on %s\n', topic_tag_state);
        catch ME3
            warning(['AprilTag bridge subscriber also disabled (%s). ', ...
                     'Tag-based landing stability will remain unavailable.'], ME3.message);
        end
    end
end

% Keep recent tag centers to estimate frame jitter.
tag_history = nan(tag_cfg.history_len, 2);
tag_hist_count = 0;
tag_area_history = nan(tag_cfg.history_len, 1);
tag_margin_history = nan(tag_cfg.history_len, 1);

% Wind publisher is started later in the loop (delay after hover phase begins).
windTimer = [];
cleanupWind = [];

% Template message
msg_takeoff = ros2message(pub_takeoff);
msg_cmd = ros2message(pub_cmd);
msg_dec = ros2message(pub_dec);
msg_wind_zero = ros2message(pub_wind_cmd);
msg_wind_zero.data = single([0.0, 0.0]);

fprintf('[MATLAB] Node ready. Subscribed to %s, %s, %s. Publishing to %s, %s, %s\n', ...
    topic_wind, topic_pose, topic_state, topic_takeoff, topic_cmd_vel, topic_decision);

%% Minimal ontology constructors
makeWind = @(speed,dir) struct('wind_speed',double(speed),'wind_direction',double(dir));
makeDrone = @(pos,quat,vel,ang) struct('position',pos,'orientation',quat,'velocity',vel,'angular',ang);
makeLandingZone = @(area_size,obstacles) struct('landing_area_size',area_size,'obstacle_presence',obstacles);
makeTagObs = @(det,n,tid,u,v,jit,arj,area,margin,qok,stb,score,ctr) struct( ...
    'detected',det,'num_tags',n,'tag_id',tid,'u_norm',u,'v_norm',v, ...
    'jitter_px',jit,'area_jitter_ratio',arj,'area_px2',area, ...
    'margin',margin,'quality_ok',qok,'stable',stb,'stability_score',score,'centered',ctr);

% A simple landing zone (user may override)
landingZone = makeLandingZone([3.0,3.0], false);

%% Decision functions
function out = decision_tree(wind, drone, lz, tagObs, params)
    % returns 'land','wait' or 'caution'
    ws = wind.wind_speed;
    % small attitude proxy: estimate from orientation quaternion to roll/pitch
    q = drone.orientation; % quaternion w,x,y,z
    [roll,pitch,yaw] = quat2eul_local([q.w, q.x, q.y, q.z]);
    att = max(abs(roll), abs(pitch));
    vz = 0; if isfield(drone,'velocity') && ~isempty(drone.velocity), vz = drone.velocity(3); end
    if ws >= params.wind_speed_unsafe || att > params.max_attitude*1.5
        out = 'wait';
        return;
    end
    if ws >= params.wind_speed_caution || att > params.max_attitude
        out = 'caution';
        return;
    end
    if abs(vz) > params.max_vz_land
        out = 'caution';
        return;
    end

    % Tag-based landing-zone observability checks
    if params.tag_require_detection && ~tagObs.detected
        out = 'wait';
        return;
    end

    if tagObs.detected
        if ~tagObs.quality_ok
            if tagObs.margin < params.tag_margin_unsafe
                out = 'wait';
            else
                out = 'caution';
            end
            return;
        end

        if tagObs.area_px2 < params.tag_min_area_px2
            out = 'caution';
            return;
        end

        if ~tagObs.centered
            out = 'caution';
            return;
        end
        if tagObs.jitter_px >= params.tag_jitter_unsafe_px
            out = 'wait';
            return;
        end
        if ~tagObs.stable || tagObs.jitter_px >= params.tag_jitter_warn_px
            out = 'caution';
            return;
        end
        if tagObs.stability_score < params.tag_stability_score_warn
            out = 'caution';
            return;
        end
    end

    % landing area check (very simple): if obstacles present -> caution
    if lz.obstacle_presence
        out = 'caution';
        return;
    end
    out = 'land';
end

function p = bayesian_score(wind, drone, bayes)
    % Compute a toy probability that it's safe using wind_speed and an attitude score
    ws = wind.wind_speed;
    q = drone.orientation; [roll,pitch,~] = quat2eul_local([q.w, q.x, q.y, q.z]);
    att_score = max(abs(roll), abs(pitch));
    % Gaussian likelihoods (independent approx)
    lw = exp(-0.5*((ws - bayes.mu_safe(1))/bayes.sigma_safe(1))^2);
    la = exp(-0.5*((att_score - bayes.mu_safe(2))/bayes.sigma_safe(2))^2);
    % normalized-ish score
    p = (lw * la) / (1 + lw * la);
end

%% Helper: quaternion->euler
function [roll,pitch,yaw] = quat2eul_local(qwxyz)
    w = qwxyz(1); x = qwxyz(2); y = qwxyz(3); z = qwxyz(4);
    % roll
    sinr = 2*(w*x + y*z); cosr = 1 - 2*(x*x + y*y);
    roll = atan2(sinr, cosr);
    % pitch
    sinp = 2*(w*y - z*x);
    if abs(sinp) >= 1, pitch = sign(sinp)*pi/2; else pitch = asin(sinp); end
    % yaw
    siny = 2*(w*z + x*y); cosy = 1 - 2*(y*y + z*z);
    yaw = atan2(siny, cosy);
end

%% Main loop: poll topics and decide
dec_method = 'decision_tree'; % or 'bayesian'
rate = rateControl(params.decision_rate);

loop_t0 = tic;
last_decision = '';
last_log_t = -inf;
viz = [];
control_phase = 'wait_ready';
control_phase_enter_t = 0.0;
last_takeoff_cmd_t = -inf;
have_pose = false;
have_state = false;
drone_state = -1; % 0:landed, 1:flying, 2:takingoff, 3:landing
last_tag_detect_t = -inf;
last_valid_tag = struct('n_tags',0,'tag_id',-1,'cx_px',nan,'cy_px',nan,'area_px2',nan,'margin',nan);
have_valid_tag = false;
last_state_rx_t = -inf;
last_ctrl_t = 0.0;
hover_start_t = nan;
wind_started = false;
pre_takeoff_tag_center_hold_start_t = nan;
hover_center_hold_start_t = nan;
pid_x = initPidState();
pid_y = initPidState();
if cfg.enable_live_plot
    viz = initLivePlots(cfg.plot_window_sec, params);
end

fprintf('[MATLAB] Entering decision loop (method=%s). Ctrl-C to stop.\n', dec_method);
while true
    % get latest wind
    wind_msg = tryReceive(sub_wind, 0.1);
    if isempty(wind_msg)
        wind = makeWind(0.0, 0.0);
    else
        % expect at least two values [speed, direction]
        dat = double(wind_msg.data);
        if numel(dat) < 2, dat = [dat(:); zeros(2-numel(dat),1)]; end
        wind = makeWind(dat(1), dat(2));
    end

    % get latest pose
    pose_msg = tryReceive(sub_pose, 0.01);
    if isempty(pose_msg)
        drone = makeDrone(struct('x',0,'y',0,'z',0), struct('w',1,'x',0,'y',0,'z',0), [0;0;0], [0;0;0]);
    else
        have_pose = true;
        pos = [pose_msg.position.x; pose_msg.position.y; pose_msg.position.z];
        q = pose_msg.orientation; quat = struct('w',q.w,'x',q.x,'y',q.y,'z',q.z);
        % velocity not available from Pose message; set zero or extend to subscribe to twist
        vel = [0;0;0]; ang = [0;0;0];
        drone = makeDrone(pos, quat, vel, ang);
    end

    state_msg = tryReceive(sub_state, 0.01);
    if ~isempty(state_msg)
        have_state = true;
        drone_state = double(state_msg.data);
        last_state_rx_t = toc(loop_t0);
    end

    % get latest apriltag detections and derive frame-center + jitter features
    detected = false; n_tags = 0; tag_id = -1; cx_px = nan; cy_px = nan; area_px2 = nan; margin = nan;
    raw_detected = false;
    tag_msg = [];
    if ~isempty(sub_tags)
        tag_msg = tryReceive(sub_tags, 0.01);
    end

    if ~isempty(tag_msg)
        [raw_detected, n_tags, tag_id, cx_px, cy_px, area_px2, margin] = extractTagFeatures(tag_msg, params.tag_target_id_enabled, params.tag_target_id);
    elseif ~isempty(sub_tag_state)
        bridge_msg = tryReceive(sub_tag_state, 0.01);
        [raw_detected, n_tags, tag_id, cx_px, cy_px, area_px2, margin] = extractTagFeaturesFromBridge(bridge_msg);
    end

    if raw_detected
        detected = true;
        last_valid_tag = struct('n_tags', n_tags, 'tag_id', tag_id, 'cx_px', cx_px, 'cy_px', cy_px, 'area_px2', area_px2, 'margin', margin);
        have_valid_tag = true;

        [tag_history, tag_hist_count] = pushTagCenter(tag_history, tag_hist_count, cx_px, cy_px);
        [tag_area_history, ~] = pushTagScalar(tag_area_history, area_px2);
        [tag_margin_history, ~] = pushTagScalar(tag_margin_history, margin);
        last_tag_detect_t = toc(loop_t0);
    elseif params.tag_hold_last_state && have_valid_tag
        hold_age = toc(loop_t0) - last_tag_detect_t;
        if hold_age <= params.tag_hold_timeout_sec
            detected = true;
            n_tags = last_valid_tag.n_tags;
            tag_id = last_valid_tag.tag_id;
            cx_px = last_valid_tag.cx_px;
            cy_px = last_valid_tag.cy_px;
            area_px2 = last_valid_tag.area_px2;
            margin = last_valid_tag.margin;
        end
    end

    [pred_ok, pred_cx, pred_cy] = predictTagCenterPx(tag_history, tag_hist_count, cx_px, cy_px, ...
        toc(loop_t0), last_tag_detect_t, params.tag_predict_horizon_sec, params.tag_predict_timeout_sec, params.decision_rate);

    [u_norm, v_norm, centered] = computeFrameCenterMetrics(cx_px, cy_px, tag_cfg.image_width, tag_cfg.image_height, params.tag_center_tolerance);
    [u_pred, v_pred, centered_pred] = computeFrameCenterMetrics(pred_cx, pred_cy, tag_cfg.image_width, tag_cfg.image_height, params.tag_center_tolerance);
    jitter_px = computeTagJitter(tag_history, tag_hist_count, params.tag_min_samples);
    area_jitter_ratio = computeScalarJitterRatio(tag_area_history, tag_hist_count, params.tag_min_samples);
    margin_mean = computeScalarMean(tag_margin_history, tag_hist_count, params.tag_min_samples);
    quality_ok = detected && isfinite(margin_mean) && margin_mean >= params.tag_margin_warn;
    stability_score = computeTagStabilityScore(jitter_px, area_jitter_ratio, margin_mean, params);
    stable = detected && isfinite(jitter_px) && isfinite(area_jitter_ratio) && ...
        (jitter_px < params.tag_jitter_warn_px) && ...
        (area_jitter_ratio < params.tag_area_jitter_warn_ratio) && ...
        (stability_score >= params.tag_stability_score_warn);
    tagObs = makeTagObs(detected, n_tags, tag_id, u_norm, v_norm, jitter_px, area_jitter_ratio, area_px2, margin_mean, quality_ok, stable, stability_score, centered);

    % run decision method
    if strcmp(dec_method,'decision_tree')
        decision = decision_tree(wind, drone, landingZone, tagObs, params);
    else
        p_safe = bayesian_score(wind, drone, bayes);
        if p_safe >= bayes.threshold, decision = 'land'; else decision = 'wait'; end
    end

    % publish decision
    msg_dec.data = char(decision);
    send(pub_dec, msg_dec);

    t_now = toc(loop_t0);
    dt_ctrl = max(1e-3, t_now - last_ctrl_t);
    last_ctrl_t = t_now;

    % Determine flight status for control: prefer fresh /state, fallback to altitude if state is stale.
    state_is_fresh = have_state && ((t_now - last_state_rx_t) <= params.state_stale_timeout_sec);
    if state_is_fresh
        is_flying_for_control = (drone_state == 1);
    else
        is_flying_for_control = have_pose && (drone.position(3) >= params.flying_altitude_threshold);
    end

    % Startup sequence: wait for readiness -> pre-takeoff stabilize -> takeoff -> settle hover -> xy hold.
    cmd_x = 0.0;
    cmd_y = 0.0;
    centered_ctrl_dbg = false;
    tag_center_for_takeoff = detected && isfinite(u_norm) && isfinite(v_norm) && ...
        (sqrt(u_norm^2 + v_norm^2) <= params.pre_takeoff_tag_center_tolerance);
    if params.xy_hold_enabled
        switch control_phase
            case 'wait_ready'
                if (have_pose && have_state) || (t_now >= params.startup_ready_timeout_sec && have_pose)
                    control_phase = 'pre_takeoff_stabilize';
                    control_phase_enter_t = t_now;
                    pre_takeoff_tag_center_hold_start_t = nan;
                end

            case 'pre_takeoff_stabilize'
                if params.pre_takeoff_zero_wind_enabled
                    send(pub_wind_cmd, msg_wind_zero);
                end

                if params.pre_takeoff_require_tag_centered
                    if tag_center_for_takeoff
                        if ~isfinite(pre_takeoff_tag_center_hold_start_t)
                            pre_takeoff_tag_center_hold_start_t = t_now;
                        end
                    else
                        pre_takeoff_tag_center_hold_start_t = nan;
                    end
                    tag_center_ready = isfinite(pre_takeoff_tag_center_hold_start_t) && ...
                        ((t_now - pre_takeoff_tag_center_hold_start_t) >= params.pre_takeoff_tag_center_hold_sec);
                else
                    tag_center_ready = true;
                end

                wind_settled = ~params.pre_takeoff_zero_wind_enabled || ...
                    ((t_now - control_phase_enter_t) >= params.pre_takeoff_zero_wind_settle_sec);

                if wind_settled && tag_center_ready
                    control_phase = 'takeoff';
                    control_phase_enter_t = t_now;
                end

            case 'takeoff'
                if params.pre_takeoff_zero_wind_enabled
                    send(pub_wind_cmd, msg_wind_zero);
                end
                if (t_now - last_takeoff_cmd_t) >= params.takeoff_retry_sec
                    send(pub_takeoff, msg_takeoff);
                    last_takeoff_cmd_t = t_now;
                end
                if is_flying_for_control
                    control_phase = 'hover_settle';
                    control_phase_enter_t = t_now;
                    hover_start_t = t_now;
                    pid_x = initPidState();
                    pid_y = initPidState();
                end

            case 'hover_settle'
                if (t_now - control_phase_enter_t) >= params.hover_settle_sec
                    control_phase = 'xy_hold';
                    control_phase_enter_t = t_now;
                end

            case 'xy_hold'
                if ~is_flying_for_control
                    control_phase = 'takeoff';
                    control_phase_enter_t = t_now;
                    hover_start_t = nan;
                    hover_center_hold_start_t = nan;
                    pid_x = initPidState();
                    pid_y = initPidState();
                else
                    use_pred = pred_ok && isfinite(u_pred) && isfinite(v_pred);
                    use_now = detected && isfinite(u_norm) && isfinite(v_norm);

                    if use_pred
                        u_ctrl = u_pred;
                        v_ctrl = v_pred;
                    elseif use_now
                        u_ctrl = u_norm;
                        v_ctrl = v_norm;
                    else
                        u_ctrl = nan;
                        v_ctrl = nan;
                    end

                    if isfinite(u_ctrl) && isfinite(v_ctrl)
                        centered_ctrl = sqrt(u_ctrl^2 + v_ctrl^2) <= params.xy_control_center_deadband;
                        centered_ctrl_dbg = centered_ctrl;
                        err_u = -u_ctrl;
                        err_v = -v_ctrl;

                        [ux, pid_x] = pidStep(err_v, dt_ctrl, pid_x, params.xy_pid_kp, params.xy_pid_ki, params.xy_pid_kd, params.xy_pid_integral_limit, params.xy_cmd_limit);
                        [uy, pid_y] = pidStep(err_u, dt_ctrl, pid_y, params.xy_pid_kp, params.xy_pid_ki, params.xy_pid_kd, params.xy_pid_integral_limit, params.xy_cmd_limit);

                        cmd_x = params.xy_map_sign_x_from_v * ux;
                        cmd_y = params.xy_map_sign_y_from_u * uy;

                        if centered_ctrl
                            cmd_x = 0.0;
                            cmd_y = 0.0;
                        end
                    else
                        pid_x = initPidState();
                        pid_y = initPidState();
                    end
                end
        end
    end

    % Start wind only after drone has entered hover phase and the configured delay has elapsed.
    if cfg.start_wind_publisher && ~wind_started && isfinite(hover_start_t)
        hover_delay_ok = (t_now - hover_start_t) >= cfg.wind_start_delay_after_hover_sec;

        if params.wind_start_require_tag_centered
            hover_center_now = strcmp(control_phase, 'xy_hold') && detected && centered_ctrl_dbg;
            if hover_center_now
                if ~isfinite(hover_center_hold_start_t)
                    hover_center_hold_start_t = t_now;
                end
            else
                hover_center_hold_start_t = nan;
            end

            hover_center_ready = isfinite(hover_center_hold_start_t) && ...
                ((t_now - hover_center_hold_start_t) >= params.wind_start_tag_center_hold_sec);
        else
            hover_center_ready = true;
        end

        if hover_delay_ok && hover_center_ready
            try
                windTimer = startWindPublisher(cfg.wind_pub_params);
                cleanupWind = onCleanup(@() safeStopWind(windTimer));
                wind_started = true;
                fprintf('[MATLAB] Wind start condition met (hover+%.1fs). Wind publisher started.\n', cfg.wind_start_delay_after_hover_sec);
            catch ME
                warning('Failed to start delayed MATLAB wind publisher: %s', ME.message);
            end
        end
    end

    if is_flying_for_control
        msg_cmd.linear.x = cmd_x;
        msg_cmd.linear.y = cmd_y;
        msg_cmd.linear.z = 0.0;
        msg_cmd.angular.x = 0.0;
        msg_cmd.angular.y = 0.0;
        msg_cmd.angular.z = 0.0;
        send(pub_cmd, msg_cmd);
    end

    if cfg.enable_live_plot
        ctrlViz = struct();
        ctrlViz.u_now = u_norm;
        ctrlViz.v_now = v_norm;
        ctrlViz.u_pred = u_pred;
        ctrlViz.v_pred = v_pred;
        ctrlViz.pred_ok = pred_ok;
        ctrlViz.centered_pred = centered_pred;
        ctrlViz.centered_ctrl = centered_ctrl_dbg;
        ctrlViz.cmd_x = cmd_x;
        ctrlViz.cmd_y = cmd_y;
        ctrlViz.control_phase = control_phase;
        ctrlViz.drone_state = drone_state;
        ctrlViz.is_flying_for_control = is_flying_for_control;
        ctrlViz.state_is_fresh = state_is_fresh;
        ctrlViz.tag_detected = detected;
        viz = updateLivePlots(viz, t_now, wind, tagObs, decision, ctrlViz, params);
    end

    if cfg.log_decision_changes_only
        if ~strcmp(decision, last_decision)
            fprintf('[%s] phase=%s state=%d fly=%d stateFresh=%d decision=%s wind=%.2f tag_detect=%d pred_ok=%d u=%.3f v=%.3f cmd=(%.2f,%.2f) stability=%.2f\n', ...
                datestr(now,'HH:MM:SS'), control_phase, drone_state, is_flying_for_control, state_is_fresh, decision, wind.wind_speed, tagObs.detected, pred_ok, u_pred, v_pred, cmd_x, cmd_y, tagObs.stability_score);
            last_decision = decision;
            last_log_t = t_now;
        elseif (t_now - last_log_t) >= cfg.log_summary_interval_sec
            fprintf('[%s] phase=%s keep=%s state=%d fly=%d stateFresh=%d pred_ok=%d cmd=(%.2f,%.2f) wind=%.2f stability=%.2f\n', ...
                datestr(now,'HH:MM:SS'), control_phase, decision, drone_state, is_flying_for_control, state_is_fresh, pred_ok, cmd_x, cmd_y, wind.wind_speed, tagObs.stability_score);
            last_log_t = t_now;
        end
    else
        fprintf('[%s] decision=%s wind=%.2f stability=%.2f\n', ...
            datestr(now,'HH:MM:SS'), decision, wind.wind_speed, tagObs.stability_score);
    end

    waitfor(rate);
end

%% small helper functions
function x = tryReceive(sub, timeout)
    % wrapper to avoid exceptions
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end

function r = rateControl(freq)
    % return a simple rate controller object with waitfor method
    r.period = 1.0/freq;
    r.tlast = tic;
    r.waitfor = @() local_wait(r);
    function local_wait(self)
        elapsed = toc(r.tlast);
        towait = r.period - elapsed;
        if towait > 0, pause(towait); end
        r.tlast = tic;
    end
end

function safeStopWind(timerHandle)
    % Stop and delete a MATLAB timer safely
    try
        if ~isempty(timerHandle) && isvalid(timerHandle)
            stop(timerHandle);
            delete(timerHandle);
            fprintf('[MATLAB] Wind publisher stopped and timer deleted.\n');
        end
    catch
        % ignore cleanup errors
    end
end

function [detected, n_tags, tag_id, cx, cy, area_px2, margin] = extractTagFeatures(tagMsg, useTargetId, targetId)
    detected = false;
    n_tags = 0;
    tag_id = -1;
    cx = nan;
    cy = nan;
    area_px2 = nan;
    margin = nan;
    if isempty(tagMsg)
        return;
    end

    detections = [];
    try
        detections = tagMsg.detections;
    catch
        return;
    end
    if isempty(detections)
        return;
    end

    n_tags = numel(detections);

    % Select detection: target ID preferred, otherwise highest decision margin.
    selectedIdx = -1;
    bestMargin = -inf;
    for i = 1:n_tags
        det = detections(i);
        thisId = -1;
        try
            thisId = double(det.id);
        catch
        end
        thisMargin = 0.0;
        try
            thisMargin = double(det.decision_margin);
        catch
        end

        if useTargetId && thisId == targetId
            selectedIdx = i;
            break;
        end
        if ~useTargetId && thisMargin > bestMargin
            bestMargin = thisMargin;
            selectedIdx = i;
        end
    end

    if selectedIdx < 0
        return;
    end

    det = detections(selectedIdx);
    try
        tag_id = double(det.id);
    catch
        tag_id = -1;
    end
    try
        margin = double(det.decision_margin);
    catch
        margin = nan;
    end

    % Try common apriltag center naming variants.
    try
        c = det.center;
        cx = double(c.x); cy = double(c.y);
    catch
        try
            c = det.centre;
            cx = double(c.x); cy = double(c.y);
        catch
        end
    end

    % Fallback: average corners if center field is unavailable.
    corners = [];
    try
        corners = det.corners;
    catch
    end
    if (~isfinite(cx) || ~isfinite(cy)) && ~isempty(corners)
        [cx, cy] = cornersCenter(corners);
    end

    if ~isempty(corners)
        area_px2 = cornersArea(corners);
    end

    detected = isfinite(cx) && isfinite(cy);
end

function [hist, count] = pushTagCenter(hist, count, cx, cy)
    if any(~isfinite([cx cy]))
        return;
    end
    hist(1:end-1,:) = hist(2:end,:);
    hist(end,:) = [cx, cy];
    count = min(size(hist,1), count + 1);
end

function [hist, count] = pushTagScalar(hist, value)
    if ~isfinite(value)
        return;
    end
    hist(1:end-1,:) = hist(2:end,:);
    hist(end,:) = value;
    validCount = sum(isfinite(hist));
    count = min(size(hist,1), validCount);
end

function [u_norm, v_norm, centered] = computeFrameCenterMetrics(cx, cy, w, h, tol)
    if ~isfinite(cx) || ~isfinite(cy) || w <= 0 || h <= 0
        u_norm = nan;
        v_norm = nan;
        centered = false;
        return;
    end
    % normalize to [-1,1], where (0,0) is image center
    u_norm = (cx - (w/2.0)) / (w/2.0);
    v_norm = (cy - (h/2.0)) / (h/2.0);
    centered = sqrt(u_norm^2 + v_norm^2) <= tol;
end

function jitter = computeTagJitter(hist, count, minSamples)
    jitter = inf;
    if count < max(minSamples, 2)
        return;
    end

    rows = hist(end-count+1:end, :);
    rows = rows(all(isfinite(rows),2), :);
    if size(rows,1) < max(minSamples, 2)
        return;
    end

    % Use RMS of frame-to-frame center displacement as jitter metric.
    dxy = diff(rows, 1, 1);
    d = sqrt(sum(dxy.^2, 2));
    jitter = sqrt(mean(d.^2));
end

function ratio = computeScalarJitterRatio(hist, count, minSamples)
    ratio = inf;
    if count < max(minSamples, 2)
        return;
    end
    rows = hist(end-count+1:end, :);
    rows = rows(isfinite(rows));
    if numel(rows) < max(minSamples, 2)
        return;
    end
    mu = mean(rows);
    if mu <= 1e-6
        return;
    end
    ratio = std(rows) / mu;
end

function mu = computeScalarMean(hist, count, minSamples)
    mu = nan;
    if count < max(minSamples, 1)
        return;
    end
    rows = hist(end-count+1:end, :);
    rows = rows(isfinite(rows));
    if numel(rows) < max(minSamples, 1)
        return;
    end
    mu = mean(rows);
end

function score = computeTagStabilityScore(centerJitter, areaJitter, margin, params)
    if ~isfinite(centerJitter) || ~isfinite(areaJitter) || ~isfinite(margin)
        score = 0.0;
        return;
    end

    s1 = 1.0 - min(1.0, centerJitter / max(params.tag_jitter_unsafe_px, 1e-6));
    s2 = 1.0 - min(1.0, areaJitter / max(params.tag_area_jitter_unsafe_ratio, 1e-6));
    s3 = min(1.0, margin / max(params.tag_margin_warn, 1e-6));
    score = max(0.0, min(1.0, 0.5*s1 + 0.3*s2 + 0.2*s3));
end

function [cx, cy] = cornersCenter(corners)
    xs = zeros(numel(corners),1);
    ys = zeros(numel(corners),1);
    for k = 1:numel(corners)
        xs(k) = double(corners(k).x);
        ys(k) = double(corners(k).y);
    end
    cx = mean(xs);
    cy = mean(ys);
end

function area = cornersArea(corners)
    area = nan;
    if isempty(corners)
        return;
    end
    n = numel(corners);
    if n < 3
        return;
    end
    xs = zeros(n,1);
    ys = zeros(n,1);
    for k = 1:n
        xs(k) = double(corners(k).x);
        ys(k) = double(corners(k).y);
    end
    xs2 = [xs; xs(1)];
    ys2 = [ys; ys(1)];
    area = 0.5 * abs(sum(xs2(1:end-1).*ys2(2:end) - xs2(2:end).*ys2(1:end-1)));
end

function [detected, n_tags, tag_id, cx, cy, area_px2, margin] = extractTagFeaturesFromBridge(msg)
    detected = false;
    n_tags = 0;
    tag_id = -1;
    cx = nan;
    cy = nan;
    area_px2 = nan;
    margin = nan;

    if isempty(msg)
        return;
    end

    try
        d = double(msg.data);
    catch
        return;
    end

    if numel(d) < 7
        return;
    end

    detected = d(1) > 0.5;
    tag_id = d(2);
    cx = d(3);
    cy = d(4);
    area_px2 = d(5);
    margin = d(6);
    n_tags = d(7);
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

function [ok, pred_cx, pred_cy] = predictTagCenterPx(hist, count, cx, cy, tNow, lastDetectT, horizonSec, timeoutSec, nominalRate)
    ok = false;
    pred_cx = nan;
    pred_cy = nan;

    if isfinite(cx) && isfinite(cy)
        pred_cx = cx;
        pred_cy = cy;
        ok = true;
    end

    if count < 2
        return;
    end

    if (tNow - lastDetectT) > timeoutSec
        return;
    end

    rows = hist(end-count+1:end, :);
    rows = rows(all(isfinite(rows), 2), :);
    if size(rows, 1) < 2
        return;
    end

    p2 = rows(end, :);
    p1 = rows(end-1, :);
    dt = 1.0 / max(nominalRate, 1e-3);
    v = (p2 - p1) ./ dt;
    p_pred = p2 + v .* horizonSec;

    pred_cx = p_pred(1);
    pred_cy = p_pred(2);
    ok = isfinite(pred_cx) && isfinite(pred_cy);
end

function y = clampValue(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end

function viz = initLivePlots(windowSec, params)
    viz = struct();
    viz.window_sec = max(5.0, windowSec);
    viz.fig = figure('Name','Landing Decision Live Monitor','NumberTitle','off');
    tlo = tiledlayout(viz.fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tlo, 1);
    viz.wind_line = animatedline(ax1, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5);
    yline(ax1, params.wind_speed_caution, '--', 'Caution', 'Color', [0.85 0.33 0.10]);
    yline(ax1, params.wind_speed_unsafe, '--', 'Unsafe', 'Color', [0.64 0.08 0.18]);
    title(ax1, 'Wind Speed (m/s)');
    ylabel(ax1, 'm/s');
    grid(ax1, 'on');

    ax2 = nexttile(tlo, 2);
    viz.jitter_line = animatedline(ax2, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.2);
    viz.score_line = animatedline(ax2, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.2);
    yline(ax2, params.tag_jitter_warn_px, '--', 'Jitter Warn', 'Color', [0.85 0.33 0.10]);
    yline(ax2, params.tag_jitter_unsafe_px, '--', 'Jitter Unsafe', 'Color', [0.64 0.08 0.18]);
    title(ax2, 'Tag Jitter (px) and Stability Score');
    ylabel(ax2, 'value');
    grid(ax2, 'on');
    legend(ax2, {'jitter','stability score'}, 'Location', 'best');

    ax3 = nexttile(tlo, 3);
    viz.decision_line = animatedline(ax3, 'Color', [0.00 0.00 0.00], 'LineWidth', 1.6);
    title(ax3, 'Decision State');
    xlabel(ax3, 'time (s)');
    ylabel(ax3, 'decision');
    yticks(ax3, [0 1 2]);
    yticklabels(ax3, {'wait','caution','land'});
    ylim(ax3, [-0.2 2.2]);
    grid(ax3, 'on');

    ax4 = nexttile(tlo, 4);
    hold(ax4, 'on');
    axis(ax4, [-1.05 1.05 -1.05 1.05]);
    axis(ax4, 'square');
    grid(ax4, 'on');
    xlabel(ax4, 'u (image horizontal, normalized)');
    ylabel(ax4, 'v (image vertical, normalized)');
    title(ax4, 'Tag-Drone Alignment and PID Intent');

    th = linspace(0, 2*pi, 100);
    plot(ax4, params.tag_center_tolerance*cos(th), params.tag_center_tolerance*sin(th), '--', 'Color', [0.5 0.5 0.5]);
    viz.center_marker = plot(ax4, 0, 0, '+', 'Color', [0 0 0], 'MarkerSize', 9, 'LineWidth', 1.4);
    viz.tag_now_marker = plot(ax4, nan, nan, 'o', 'Color', [0.85 0.33 0.10], 'MarkerSize', 6, 'LineWidth', 1.3);
    viz.tag_pred_marker = plot(ax4, nan, nan, 's', 'Color', [0.00 0.45 0.74], 'MarkerSize', 6, 'LineWidth', 1.3);
    viz.align_quiver = quiver(ax4, 0, 0, 0, 0, 0, 'Color', [0.10 0.10 0.10], 'LineWidth', 1.5, 'MaxHeadSize', 2.0);
    viz.cmd_quiver = quiver(ax4, 0, 0, 0, 0, 0, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.5, 'MaxHeadSize', 2.0);
    viz.ctrl_text = text(ax4, -1.02, 1.00, '', 'VerticalAlignment', 'top', 'FontSize', 9, 'Color', [0.1 0.1 0.1]);

    legend(ax4, {'center tolerance', 'image center', 'tag now', 'tag predicted', 'align vector', 'cmd vector'}, 'Location', 'eastoutside');

    viz.ax = [ax1 ax2 ax3];
    viz.ax_align = ax4;
end

function viz = updateLivePlots(viz, tNow, wind, tagObs, decision, ctrlViz, params)
    if isempty(viz) || ~isfield(viz, 'fig') || ~isgraphics(viz.fig)
        return;
    end

    addpoints(viz.wind_line, tNow, wind.wind_speed);
    if isfinite(tagObs.jitter_px)
        addpoints(viz.jitter_line, tNow, tagObs.jitter_px);
    end
    if isfinite(tagObs.stability_score)
        addpoints(viz.score_line, tNow, tagObs.stability_score);
    end
    addpoints(viz.decision_line, tNow, decisionToNumeric(decision));

    xmin = max(0, tNow - viz.window_sec);
    xmax = max(viz.window_sec, tNow);
    for k = 1:numel(viz.ax)
        xlim(viz.ax(k), [xmin xmax]);
    end

    if isfield(viz, 'ax_align') && isgraphics(viz.ax_align)
        if isfinite(ctrlViz.u_now) && isfinite(ctrlViz.v_now)
            set(viz.tag_now_marker, 'XData', ctrlViz.u_now, 'YData', ctrlViz.v_now);
        else
            set(viz.tag_now_marker, 'XData', nan, 'YData', nan);
        end

        if ctrlViz.pred_ok && isfinite(ctrlViz.u_pred) && isfinite(ctrlViz.v_pred)
            set(viz.tag_pred_marker, 'XData', ctrlViz.u_pred, 'YData', ctrlViz.v_pred);

            align_u = -ctrlViz.u_pred;
            align_v = -ctrlViz.v_pred;
            set(viz.align_quiver, 'XData', ctrlViz.u_pred, 'YData', ctrlViz.v_pred, 'UData', align_u, 'VData', align_v);

            cmd_norm = max(params.xy_cmd_limit, 1e-6);
            cmd_u = clampValue(ctrlViz.cmd_y / cmd_norm, -1.0, 1.0);
            cmd_v = clampValue(ctrlViz.cmd_x / cmd_norm, -1.0, 1.0);
            set(viz.cmd_quiver, 'XData', ctrlViz.u_pred, 'YData', ctrlViz.v_pred, 'UData', cmd_u*0.6, 'VData', cmd_v*0.6);
        else
            set(viz.tag_pred_marker, 'XData', nan, 'YData', nan);
            set(viz.align_quiver, 'XData', 0, 'YData', 0, 'UData', 0, 'VData', 0);
            set(viz.cmd_quiver, 'XData', 0, 'YData', 0, 'UData', 0, 'VData', 0);
        end

        align_err = nan;
        if isfinite(ctrlViz.u_pred) && isfinite(ctrlViz.v_pred)
            align_err = sqrt(ctrlViz.u_pred^2 + ctrlViz.v_pred^2);
        end

        centered_ctrl = 0;
        if isfield(ctrlViz, 'centered_ctrl')
            centered_ctrl = ctrlViz.centered_ctrl;
        end

        txt = sprintf(['phase=%s state=%d fly=%d fresh=%d det=%d pred=%d centeredPred=%d centeredCtrl=%d\n', ...
                       'u_pred=%.3f v_pred=%.3f align_err=%.3f\n', ...
                       'cmd_x=%.3f cmd_y=%.3f'], ...
                       char(ctrlViz.control_phase), ctrlViz.drone_state, ctrlViz.is_flying_for_control, ctrlViz.state_is_fresh, ...
                       ctrlViz.tag_detected, ctrlViz.pred_ok, ctrlViz.centered_pred, centered_ctrl, ...
                       ctrlViz.u_pred, ctrlViz.v_pred, align_err, ctrlViz.cmd_x, ctrlViz.cmd_y);
        set(viz.ctrl_text, 'String', txt);
    end

    drawnow limitrate nocallbacks;
end

function y = decisionToNumeric(decision)
    switch char(decision)
        case 'land'
            y = 2;
        case 'caution'
            y = 1;
        otherwise
            y = 0;
    end
end

% EOF
