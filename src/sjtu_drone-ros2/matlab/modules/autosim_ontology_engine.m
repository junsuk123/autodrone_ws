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
onto.wind_velocity_vec = asv(windObs, 'wind_velocity', [onto.wind_speed; 0.0]);
onto.wind_velocity = vector_mag(onto.wind_velocity_vec);
onto.wind_acceleration_vec = asv(windObs, 'wind_acceleration', [0.0; 0.0]);
onto.wind_acceleration = vector_mag(onto.wind_acceleration_vec);
onto.wind_risk = compute_wind_risk(onto.wind_velocity_vec, onto.wind_acceleration_vec, cfg, onto.roll_abs, onto.pitch_abs);
windVelComp = ensure_vec2(onto.wind_velocity_vec, [onto.wind_velocity; 0.0]);
windAccComp = ensure_vec2(onto.wind_acceleration_vec, [onto.wind_acceleration; 0.0]);
onto.wind_velocity_x = windVelComp(1);
onto.wind_velocity_y = windVelComp(2);
onto.wind_acceleration_x = windAccComp(1);
onto.wind_acceleration_y = windAccComp(2);

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
windVelocityVec = ensure_vec2(asv(onto, 'wind_velocity_vec', [windVelocity; 0.0]), [windVelocity; 0.0]);
windAccVec = ensure_vec2(asv(onto, 'wind_acceleration_vec', [windAcc; 0.0]), [windAcc; 0.0]);
windVelocity = hypot(windVelocityVec(1), windVelocityVec(2));
windAcc = hypot(windAccVec(1), windAccVec(2));
windCaution = cfg.ontology.wind_caution_speed;
windUnsafe = cfg.ontology.wind_unsafe_speed;
rollAbs = abs(asv(onto, 'roll_abs', 0.0));
pitchAbs = abs(asv(onto, 'pitch_abs', 0.0));
windRisk = compute_wind_risk(windVelocityVec, windAccVec, cfg, rollAbs, pitchAbs);

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
semantic.wind_velocity_x = windVelocityVec(1);
semantic.wind_velocity_y = windVelocityVec(2);
semantic.wind_acceleration_x = windAccVec(1);
semantic.wind_acceleration_y = windAccVec(2);

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

semantic.landing_feasibility = clamp(0.45 * (1.0 - semantic.wind_risk_enc) + 0.30 * semantic.alignment_enc + 0.25 * semantic.visual_enc, 0.0, 1.0);
semantic.isSafeForLanding = semantic.landing_feasibility >= 0.55;
if semantic.isSafeForLanding
    semantic.final_decision = "AttemptLanding";
else
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
            vec(i) = asv(windObs, 'wind_velocity_mag', vector_mag(asv(windObs, 'wind_velocity', asv(windObs, 'wind_speed', 0.0))));
        case "wind_acceleration"
            vec(i) = asv(windObs, 'wind_acceleration_mag', vector_mag(asv(windObs, 'wind_acceleration', 0.0)));
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

function windRisk = compute_wind_risk(windVelocity, windAcceleration, cfg, rollAbs, pitchAbs)
    if nargin < 4 || ~isfinite(double(rollAbs))
        rollAbs = 0.0;
    end
    if nargin < 5 || ~isfinite(double(pitchAbs))
        pitchAbs = 0.0;
    end

    [velMag, velComp] = vector_mag_and_component_max(windVelocity);
    [accMag, accComp] = vector_mag_and_component_max(windAcceleration);
    windUnsafe = max(1e-3, asv(cfg.ontology, 'wind_unsafe_speed', 1.0));

    model = get_wind_load_model(cfg, rollAbs, pitchAbs);
    dynPressure = 0.5 * model.rho * model.cd * model.area;

    dragMag = dynPressure * (velMag ^ 2);
    dragComp = dynPressure * (velComp ^ 2);

    accelPeak = max(accMag, accComp);
    gustGain = 1.0 + min(0.5, 0.08 * max(0.0, accelPeak));

    dragEff = max(dragMag, dragComp) * gustGain;
    dragRatio = dragEff / max(1e-6, model.drag_capacity_n);
    dragEquivalentSpeed = windUnsafe * sqrt(max(0.0, dragRatio));

    windRisk = max([velMag, velComp, dragEquivalentSpeed]);
end

function model = get_wind_load_model(cfg, rollAbs, pitchAbs)
    if nargin < 2 || ~isfinite(double(rollAbs))
        rollAbs = 0.0;
    end
    if nargin < 3 || ~isfinite(double(pitchAbs))
        pitchAbs = 0.0;
    end

    model = struct();
    model.rho = 1.225;
    model.cd = 1.10;
    model.area = 0.075;
    model.drag_capacity_n = 6.0;

    if ~isstruct(cfg) || ~isfield(cfg, 'wind') || ~isstruct(cfg.wind) || ~isfield(cfg.wind, 'physics') || ~isstruct(cfg.wind.physics)
        return;
    end

    p = cfg.wind.physics;
    model.rho = max(1e-6, windPhysicsField(p, 'air_density_kgpm3', model.rho));
    model.cd = max(1e-6, windPhysicsField(p, 'drag_coefficient', model.cd));
    model.area = max(1e-6, windPhysicsField(p, 'frontal_area_m2', model.area));

    massKg = windPhysicsField(p, 'mass_kg', 1.4);
    grav = windPhysicsField(p, 'gravity_mps2', 9.81);
    maxThrust = windPhysicsField(p, 'max_total_thrust_n', 24.0);
    minMargin = max(0.0, windPhysicsField(p, 'min_thrust_margin_n', 0.5));

    cosTilt = cos(abs(double(rollAbs))) * cos(abs(double(pitchAbs)));
    cosTilt = min(1.0, max(0.15, cosTilt));
    requiredHoverThrust = (massKg * grav) / cosTilt;

    margin = maxThrust - requiredHoverThrust;
    if ~(isfinite(margin) && (margin > 0))
        marginFlat = windPhysicsField(p, 'thrust_margin_n', maxThrust - massKg * grav);
        if isfinite(marginFlat)
            margin = marginFlat;
        else
            margin = maxThrust - massKg * grav;
        end
    end
    margin = max(1e-6, max(margin, minMargin));

    model.drag_capacity_n = margin;
end

function v = windPhysicsField(p, name, fallback)
    if isstruct(p) && isfield(p, name)
        vv = double(p.(name));
        if isfinite(vv)
            v = vv;
            return;
        end
    end
    v = fallback;
end

function m = vector_mag(v)
vv = double(v);
if isempty(vv)
    m = 0.0;
    return;
end
if isvector(vv)
    vv = vv(:);
    if numel(vv) >= 2
        m = hypot(vv(1), vv(2));
    else
        m = abs(vv(1));
    end
else
    if size(vv, 2) >= 2
        m = hypot(vv(:,1), vv(:,2));
    else
        m = abs(vv(:,1));
    end
end
m(~isfinite(m)) = 0.0;
if numel(m) > 1
    m = mean(m);
end
end

function v2 = ensure_vec2(v, fallback)
vv = double(v(:));
if isempty(vv)
    vv = double(fallback(:));
end
if isempty(vv)
    v2 = [0.0; 0.0];
elseif numel(vv) == 1
    v2 = [vv(1); 0.0];
else
    v2 = vv(1:2);
end
v2(~isfinite(v2)) = 0.0;
end

function [magVal, compMax] = vector_mag_and_component_max(v)
v2 = ensure_vec2(v, [0.0; 0.0]);
magVal = hypot(v2(1), v2(2));
compMax = max(abs(v2(1)), abs(v2(2)));
end
