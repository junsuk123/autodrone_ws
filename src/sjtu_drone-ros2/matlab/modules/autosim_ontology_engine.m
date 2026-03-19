function varargout = autosim_ontology_engine(action, varargin)
% autosim_ontology_engine
% Ontology module for state construction, reasoning, and semantic features.

switch lower(string(action))
    case "build_state"
        varargout{1} = build_state(varargin{1}, varargin{2}, varargin{3}, varargin{4});
    case "reason"
        varargout{1} = reason(varargin{1}, varargin{2});
    case "build_features"
        varargout{1} = build_features(varargin{1}, varargin{2}, varargin{3}, varargin{4}, varargin{5});
    otherwise
        error("autosim_ontology_engine:unknownAction", "Unknown action: %s", string(action));
end
end

function onto = build_state(windObs, droneObs, tagObs, cfg)
onto = struct();
onto.wind_speed = asv(windObs, 'wind_speed', 0.0);
onto.wind_dir = asv(windObs, 'wind_direction', 0.0);
onto.roll_abs = abs(asv(droneObs, 'roll', 0.0));
onto.pitch_abs = abs(asv(droneObs, 'pitch', 0.0));
onto.altitude = asv(droneObs, 'position', [0;0;0]);
if numel(onto.altitude) >= 3
    onto.altitude = onto.altitude(3);
else
    onto.altitude = 0.0;
end
onto.vz = asv(droneObs, 'velocity', [0;0;0]);
if numel(onto.vz) >= 3
    onto.vz = onto.vz(3);
else
    onto.vz = 0.0;
end
onto.tag_detected = logical(asv(tagObs, 'detected', false));
onto.tag_u = asv(tagObs, 'u_norm', nan);
onto.tag_v = asv(tagObs, 'v_norm', nan);
onto.tag_jitter_px = asv(tagObs, 'jitter_px', nan);
onto.tag_stability_score = asv(tagObs, 'stability_score', 0.0);
onto.detection_continuity = asv(tagObs, 'detection_continuity', 0.0);
onto.tag_err_hist = asv(tagObs, 'err_hist', nan);
onto.tag_detected_hist = asv(tagObs, 'detected_hist', nan);
onto.wind_speed_hist = asv(windObs, 'wind_speed_hist', nan);
onto.wind_dir_hist = asv(windObs, 'wind_dir_hist', nan);
onto.dt = max(asv(windObs, 'dt', 0.1), 1e-3);

% MathematicalObject layer
onto.wind_velocity = onto.wind_speed;
onto.wind_acceleration = compute_wind_acceleration(onto.wind_speed_hist, onto.dt);
onto.wind_risk = compute_wind_risk(onto.wind_velocity, onto.wind_acceleration, cfg.ontology.wind_caution_speed, cfg.ontology.wind_unsafe_speed);

onto.gust_intensity = nanstd_safe(diff_with_zero(onto.wind_speed_hist));
onto.wind_variability = nanstd_safe(onto.wind_speed_hist);
onto.wind_dir_shift = abs(last_safe(onto.wind_dir_hist) - first_safe(onto.wind_dir_hist));
onto.wind_dir_spread = nanstd_safe(onto.wind_dir_hist);
onto.tag_error_volatility = nanstd_safe(onto.tag_err_hist);
onto.tag_error_drift = temporal_slope(onto.tag_err_hist, onto.dt);

% Minimal ontology node taxonomy for runtime reasoning.
onto.Event = struct( ...
    'WindObservedEvent', true, ...
    'MarkerObservedEvent', logical(onto.tag_detected), ...
    'LandingRiskUpdatedEvent', true);
onto.SpatialThing = struct( ...
    'Drone', true, ...
    'LandingPad', true, ...
    'VisualMarker', logical(onto.tag_detected), ...
    'WindRegion', true);
onto.MathObject = struct( ...
    'WindVelocity', onto.wind_velocity, ...
    'WindAcceleration', onto.wind_acceleration, ...
    'WindRisk', onto.wind_risk, ...
    'MarkerCenterError', sqrt(asv(tagObs, 'u_norm', 0.0)^2 + asv(tagObs, 'v_norm', 0.0)^2), ...
    'FeatureVector', [], ...
    'DecisionAlgorithm', "OntologyAIFusion");
end

function semantic = reason(onto, cfg)
semantic = struct();

% WindRisk is explicitly derived from WindVelocity and WindAcceleration.
windVelocity = asv(onto, 'wind_velocity', onto.wind_speed);
windAcc = asv(onto, 'wind_acceleration', 0.0);
windCaution = cfg.ontology.wind_caution_speed;
windUnsafe = cfg.ontology.wind_unsafe_speed;
windRisk = compute_wind_risk(windVelocity, windAcc, windCaution, windUnsafe);

% Assign wind risk display name and encoding
if windRisk >= windUnsafe
    semantic.wind_risk = "high";
    semantic.wind_risk_enc = 1.0;
elseif windRisk >= windCaution
    semantic.wind_risk = "caution";
    semantic.wind_risk_enc = 0.6;
else
    semantic.wind_risk = "low";
    semantic.wind_risk_enc = 0.1;
end

% Store raw physical measurements for downstream use
semantic.wind_velocity = windVelocity;
semantic.wind_acceleration = windAcc;

attWarn = deg2rad(cfg.ontology.control_attitude_warn_deg);
attHigh = deg2rad(cfg.ontology.control_attitude_high_deg);
att = max(onto.roll_abs, onto.pitch_abs);
if att >= attHigh
    semantic.drone_state = "aggressive";
elseif att >= attWarn
    semantic.drone_state = "adjusting";
else
    semantic.drone_state = "stable";
end

if ~onto.tag_detected
    semantic.visual_state = "lost";
    semantic.visual_enc = 0.0;
elseif onto.tag_stability_score < cfg.ontology.tag_stability_score_warn
    semantic.visual_state = "noisy";
    semantic.visual_enc = 0.4;
else
    semantic.visual_state = "good";
    semantic.visual_enc = 0.9;
end

if onto.tag_detected && isfinite(onto.tag_u) && isfinite(onto.tag_v)
    err = sqrt(onto.tag_u^2 + onto.tag_v^2);
else
    err = inf;
end
if err < 0.08
    semantic.alignment_state = "centered";
    semantic.alignment_enc = 0.9;
elseif err < 0.2
    semantic.alignment_state = "offset";
    semantic.alignment_enc = 0.5;
else
    semantic.alignment_state = "misaligned";
    semantic.alignment_enc = 0.1;
end

contextScore = 0.35 * (1.0 - semantic.wind_risk_enc) + 0.30 * semantic.alignment_enc + 0.35 * semantic.visual_enc;
semantic.context_enc = clamp(contextScore, 0.0, 1.0);
if semantic.context_enc >= 0.7
    semantic.landing_context = "clear";
elseif semantic.context_enc >= 0.45
    semantic.landing_context = "caution";
else
    semantic.landing_context = "unsafe";
end

semantic.semantic_relation = "consistent";
if semantic.wind_risk == "high" && semantic.alignment_enc > 0.7
    semantic.semantic_relation = "conflicting";
end

semantic.landing_feasibility = clamp(0.55 * semantic.context_enc + 0.25 * semantic.alignment_enc + 0.20 * semantic.visual_enc, 0.0, 1.0);
semantic.isSafeForLanding = semantic.landing_feasibility >= 0.55;
if semantic.isSafeForLanding
    semantic.semantic_integration = "AttemptLanding";
    semantic.final_decision = "AttemptLanding";
else
    semantic.semantic_integration = "HoldLanding";
    semantic.final_decision = "HoldLanding";
end
semantic.action = semantic.final_decision;
semantic.environment_state = semantic.wind_risk;
end

function vec = build_features(windObs, droneObs, tagObs, semantic, cfg)
names = string(cfg.ontology.semantic_feature_names);
vec = nan(1, numel(names));
for i = 1:numel(names)
    key = names(i);
    switch key
        case "wind_speed"
            vec(i) = asv(windObs, 'wind_speed', 0.0);
        case "wind_velocity"
            vec(i) = asv(windObs, 'wind_velocity', asv(windObs, 'wind_speed', 0.0));
        case "wind_acceleration"
            vec(i) = asv(windObs, 'wind_acceleration', 0.0);
        case "wind_dir_norm"
            vec(i) = abs(asv(windObs, 'wind_direction', 0.0)) / 180.0;
        case "roll_abs"
            vec(i) = abs(asv(droneObs, 'roll', 0.0));
        case "pitch_abs"
            vec(i) = abs(asv(droneObs, 'pitch', 0.0));
        case "tag_u"
            vec(i) = asv(tagObs, 'u_norm', 0.0);
        case "tag_v"
            vec(i) = asv(tagObs, 'v_norm', 0.0);
        case "jitter"
            vec(i) = asv(tagObs, 'jitter_px', 0.0);
        case "stability_score"
            vec(i) = asv(tagObs, 'stability_score', 0.0);
        case "wind_risk_enc"
            vec(i) = asv(semantic, 'wind_risk_enc', 0.0);
        case "alignment_enc"
            vec(i) = asv(semantic, 'alignment_enc', 0.0);
        case "visual_enc"
            vec(i) = asv(semantic, 'visual_enc', 0.0);
        case "context_enc"
            vec(i) = asv(semantic, 'context_enc', 0.0);
        otherwise
            vec(i) = 0.0;
    end
end
vec(~isfinite(vec)) = 0.0;
end

function v = asv(s, name, fallback)
if isstruct(s) && isfield(s, name)
    v = s.(name);
else
    v = fallback;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end

function v = nanstd_safe(x)
x = double(x(:));
x = x(isfinite(x));
if numel(x) <= 1
    v = 0.0;
else
    v = std(x);
end
end

function y = diff_with_zero(x)
x = double(x(:));
if numel(x) <= 1
    y = 0.0;
else
    y = [0.0; diff(x)];
end
end

function v = first_safe(x)
x = double(x(:));
idx = find(isfinite(x), 1, 'first');
if isempty(idx)
    v = 0.0;
else
    v = x(idx);
end
end

function v = last_safe(x)
x = double(x(:));
idx = find(isfinite(x), 1, 'last');
if isempty(idx)
    v = 0.0;
else
    v = x(idx);
end
end

function slope = temporal_slope(x, dt)
x = double(x(:));
x = x(isfinite(x));
if numel(x) < 2 || ~isfinite(dt) || dt <= 0
    slope = 0.0;
    return;
end
t = (0:numel(x)-1)' * dt;
p = polyfit(t, x, 1);
slope = p(1);
end

function windAcc = compute_wind_acceleration(windSpeedHist, dt)
% Compute WindAcceleration from wind speed history
% WindAcceleration = rate of change of wind speed over time = dv/dt (m/s^2)
% 
% Approach: Use recent samples to estimate the trend, with smoothing to handle noise
% Returns: acceleration value in m/s^2
    
windSpeedHist = double(windSpeedHist(:));
windSpeedHist = windSpeedHist(isfinite(windSpeedHist));

if numel(windSpeedHist) < 2 || ~isfinite(dt) || dt <= 0
    windAcc = 0.0;
    return;
end

% Use most recent window (last 60% of samples) to capture current trend
windowSize = max(2, round(0.6 * numel(windSpeedHist)));
recentSamples = windSpeedHist(max(1, end-windowSize+1):end);

if numel(recentSamples) < 2
    windAcc = 0.0;
    return;
end

% Linear fit to estimate acceleration (slope of v vs t)
t = (0:numel(recentSamples)-1)' * dt;
p = polyfit(t, recentSamples, 1);
windAcc = p(1);  % Slope in m/s per second = m/s^2
end

function windRisk = compute_wind_risk(windVelocity, windAcceleration, windCaution, windUnsafe)
% Compute WindRisk as a function of current wind velocity and wind acceleration
% 
% WindRisk = max(velocity-based component, acceleration-adjusted component)
% This ensures high risk when:
%  1) Current speed is already high, OR
%  2) Speed is rapidly increasing even if current value is moderate
%
% windVelocity: current wind speed (m/s)
% windAcceleration: rate of change of wind speed (m/s^2)
% windCaution: velocity threshold for caution (m/s)
% windUnsafe: velocity threshold for unsafe (m/s)

    if ~isfinite(windVelocity)
        windVelocity = 0.0;
    end
    if ~isfinite(windAcceleration)
        windAcceleration = 0.0;
    end
    
    % Component 1: Direct velocity-based risk
    velocityRisk = windVelocity;
    
    % Component 2: Acceleration-adjusted risk
    % If wind is accelerating quickly, treat it as more risky even if current value is lower
    % Use acceleration to project near-future wind conditions
    accelAdjustment = 0.0;
    if windAcceleration > 0
        % Positive acceleration (increasing wind) adds risk
        % Scale: each 0.5 m/s^2 of acceleration is worth ~0.1 m/s of risk margin
        accelAdjustment = min(0.3 * windUnsafe, 0.2 * windAcceleration);
    end
    
    % Combine: risk is the maximum of velocity and accelerated estimate
    windRisk = max(velocityRisk, velocityRisk + accelAdjustment);
end
