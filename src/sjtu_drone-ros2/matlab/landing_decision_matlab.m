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
topic_decision = '/landing_decision';               % std_msgs/String

% Decision params
params.wind_speed_unsafe = 7.0;   % m/s above -> unsafe
params.wind_speed_caution = 4.0;  % m/s between caution and unsafe
params.max_attitude = deg2rad(10);% roll/pitch limit
params.max_vz_land = 0.5;         % vertical speed limit for safe landing
params.decision_rate = 2.0;       % Hz

% Wind publisher integration: if true, MATLAB will generate and publish wind to /wind_command
cfg.start_wind_publisher = true; % enable built-in wind generator
cfg.wind_pub_params = struct();   % see wind_publisher_matlab.startWindPublisher options (rate, steady_speed, etc.)
cfg.wind_pub_params.use_set_wind_service = false;   % publish /wind_command continuously
cfg.wind_pub_params.topic_publish_mode = 'matlab';  % 'matlab' publisher per tick, or 'cli'
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
pub_dec = ros2publisher(node, topic_decision, 'std_msgs/String');

% Optionally start the MATLAB wind publisher which will publish to /wind_command
windTimer = [];
if cfg.start_wind_publisher
    try
        % ensure wind_publisher_matlab is on path
        windTimer = startWindPublisher(cfg.wind_pub_params);
        % ensure cleanup on exit
        cleanupWind = onCleanup(@() safeStopWind(windTimer));
    catch ME
        warning('Failed to start MATLAB wind publisher: %s', ME.message);
    end
end

% Template message
msg_dec = ros2message(pub_dec);

fprintf('[MATLAB] Node ready. Subscribed to %s and %s. Publishing decisions to %s\n', topic_wind, topic_pose, topic_decision);

%% Minimal ontology constructors
makeWind = @(speed,dir) struct('wind_speed',double(speed),'wind_direction',double(dir));
makeDrone = @(pos,quat,vel,ang) struct('position',pos,'orientation',quat,'velocity',vel,'angular',ang);
makeLandingZone = @(area_size,obstacles) struct('landing_area_size',area_size,'obstacle_presence',obstacles);

% A simple landing zone (user may override)
landingZone = makeLandingZone([3.0,3.0], false);

%% Decision functions
function out = decision_tree(wind, drone, lz, params)
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
        pos = [pose_msg.position.x; pose_msg.position.y; pose_msg.position.z];
        q = pose_msg.orientation; quat = struct('w',q.w,'x',q.x,'y',q.y,'z',q.z);
        % velocity not available from Pose message; set zero or extend to subscribe to twist
        vel = [0;0;0]; ang = [0;0;0];
        drone = makeDrone(pos, quat, vel, ang);
    end

    % run decision method
    if strcmp(dec_method,'decision_tree')
        decision = decision_tree(wind, drone, landingZone, params);
    else
        p_safe = bayesian_score(wind, drone, bayes);
        if p_safe >= bayes.threshold, decision = 'land'; else decision = 'wait'; end
    end

    % publish decision
    msg_dec.data = char(decision);
    send(pub_dec, msg_dec);

    % print small status
    fprintf('[%s] wind=%.2fm/s dir=%.1fdeg -> decision=%s\n', datestr(now,'HH:MM:SS'), wind.wind_speed, wind.wind_direction, decision);

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

% EOF
