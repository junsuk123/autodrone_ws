function varargout = autosim_ai_engine(action, varargin)
% autosim_ai_engine
% AI/model module: online feature extraction and GaussianNB prediction/training.

switch lower(string(action))
    case "build_online_features"
        varargout{1} = build_online_features(varargin{:});
    case "predict_model"
        [varargout{1}, varargout{2}] = predict_model(varargin{1}, varargin{2}, varargin{3});
    case "train_gnb"
        varargout{1} = train_gnb(varargin{1}, varargin{2}, varargin{3}, varargin{4});
    case "predict_gnb"
        [varargout{1}, varargout{2}] = predict_gnb(varargin{1}, varargin{2});
    otherwise
        error("autosim_ai_engine:unknownAction", "Unknown action: %s", string(action));
end
end

function feat = build_online_features(z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, contact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR, semVec, cfg, windVelocity, windAcceleration)
feat = struct();
feat.mean_wind_speed = nanmean_safe(windSpeed);
feat.max_wind_speed = nanmax_safe(windSpeed);
if nargin < 18 || isempty(windVelocity)
    windVelocity = windSpeed;
end
if nargin < 19 || isempty(windAcceleration)
    windAcceleration = 0.0;
end
[feat.wind_velocity_x, feat.wind_velocity_y, feat.wind_velocity] = resolve_wind_vector_features(windVelocity, windSpeed);
[feat.wind_acceleration_x, feat.wind_acceleration_y, feat.wind_acceleration] = resolve_wind_vector_features(windAcceleration, 0.0);
feat.mean_abs_roll_deg = nanmean_safe(abs(rollDeg));
feat.mean_abs_pitch_deg = nanmean_safe(abs(pitchDeg));
feat.mean_abs_vz = nanmean_safe(abs(vz));
feat.max_abs_vz = nanmax_safe(abs(vz));
feat.mean_tag_error = nanmean_safe(tagErr);
feat.max_tag_error = nanmax_safe(tagErr);

feat.final_altitude = nanlast_safe(z);
feat.final_abs_speed = nanlast_safe(speedAbs);
feat.final_abs_roll_deg = nanlast_safe(abs(rollDeg));
feat.final_abs_pitch_deg = nanlast_safe(abs(pitchDeg));
feat.final_tag_error = nanlast_safe(tagErr);

finalWindow = min(numel(z), 25);
zTail = tail_safe(z, finalWindow);
vzTail = tail_safe(vz, finalWindow);
feat.stability_std_z = nanstd_safe(zTail);
feat.stability_std_vz = nanstd_safe(vzTail);

feat.contact_count = sum(contact > 0);
feat.mean_imu_ang_vel = nanmean_safe(imuAngVel);
feat.max_imu_ang_vel = nanmax_safe(imuAngVel);
feat.mean_imu_lin_acc = nanmean_safe(imuLinAcc);
feat.max_imu_lin_acc = nanmax_safe(imuLinAcc);
feat.max_contact_force = nanmax_safe(contactForce);
feat.arm_force_imbalance = nanmax_safe([abs(armFL-armFR), abs(armRL-armRR)]);

if nargin >= 17 && ~isempty(semVec) && nargin >= 18 && isfield(cfg, 'ontology') && isfield(cfg.ontology, 'semantic_feature_names')
    semNames = string(cfg.ontology.semantic_feature_names);
    feat.wind_risk_enc = sem_get(semVec, semNames, "wind_risk_enc", 0.0);
    feat.alignment_enc = sem_get(semVec, semNames, "alignment_enc", 0.0);
    feat.visual_enc = sem_get(semVec, semNames, "visual_enc", 0.0);
    feat.wind_body_risk_enc = sem_get(semVec, semNames, "wind_body_risk_enc", feat.wind_risk_enc);
    feat.wind_gust_risk_enc = sem_get(semVec, semNames, "wind_gust_risk_enc", 0.0);
    feat.wind_dir_change_risk_enc = sem_get(semVec, semNames, "wind_dir_change_risk_enc", 0.0);
end

if nargin >= 17 && ~isempty(cfg)
    ontoFeat = autosimBuildOntologyInputFromFeatureStruct(feat, cfg);
    feat.onto_wind_condition = ontoFeat.onto_wind_condition;
    feat.onto_gust = ontoFeat.onto_gust;
    feat.onto_temporal_pattern = ontoFeat.onto_temporal_pattern;
    feat.onto_drone_state = ontoFeat.onto_drone_state;
    feat.onto_tag_observation = ontoFeat.onto_tag_observation;
end
end

function [predLabel, predScore] = predict_model(model, featStruct, featureNames)
X = zeros(1, numel(featureNames));
for i = 1:numel(featureNames)
    fn = char(featureNames(i));
    if isfield(featStruct, fn)
        X(i) = double(featStruct.(fn));
    end
end
[predLabel, predScore] = predict_gnb(model, X);
end

function model = train_gnb(X, y, featureNames, priorUniformBlend)
X = sanitize(X);
cls = unique(y);
nClass = numel(cls);
nFeat = size(X, 2);
if nargin < 4
    priorUniformBlend = 0.55;
end

mu = zeros(nClass, nFeat);
sigma2 = ones(nClass, nFeat);
prior = zeros(nClass, 1);
for i = 1:nClass
    mask = (y == cls(i));
    Xi = X(mask, :);
    prior(i) = max(mean(mask), eps);
    mu(i,:) = mean(Xi, 1);
    sigma2(i,:) = var(Xi, 0, 1);
end

uniformPrior = ones(size(prior)) / max(numel(prior), 1);
prior = (1.0 - priorUniformBlend) * prior + priorUniformBlend * uniformPrior;
prior = prior / sum(prior);

model = struct();
model.class_names = cls(:);
model.mu = mu;
model.sigma2 = max(sigma2, 1e-6);
model.prior = prior(:);
model.feature_names = string(featureNames(:)).';
end

function [predLabel, predScore] = predict_gnb(model, X)
X = sanitize(X);
if isvector(X)
    X = reshape(X, 1, []);
end
n = size(X, 1);
k = numel(model.class_names);
logPost = zeros(n, k);
for i = 1:k
    mu = model.mu(i,:);
    sg = model.sigma2(i,:);
    lp = -0.5 * sum(log(2.0*pi*sg) + ((X - mu).^2) ./ sg, 2);
    logPost(:,i) = log(max(model.prior(i), eps)) + lp;
end
[~, idx] = max(logPost, [], 2);
predLabel = model.class_names(idx);

if k >= 2
    ex = exp(logPost - max(logPost, [], 2));
    post = ex ./ max(sum(ex, 2), eps);
    cls = string(model.class_names);
    attemptIdx = find(cls == "AttemptLanding", 1, 'first');
    if isempty(attemptIdx)
        % Backward compatibility with legacy model files.
        attemptIdx = find(cls == "stable", 1, 'first');
    end
    if isempty(attemptIdx)
        predScore = post(:,1);
    else
        predScore = post(:, attemptIdx);
    end
else
    predScore = ones(n,1);
end

if n == 1
    predLabel = predLabel(1);
    predScore = predScore(1);
end
end

function x = sanitize(x)
x(~isfinite(x)) = 0;
end

function out = tail_safe(x, n)
x = x(:);
if nargin < 2
    n = numel(x);
end
n = max(0, floor(n));
if n == 0
    out = x([]);
elseif numel(x) <= n
    out = x;
else
    out = x(end-n+1:end);
end
end

function v = nanmean_safe(x)
x = double(x(:)); x = x(isfinite(x));
if isempty(x), v = nan; else, v = mean(x); end
end

function v = nanstd_safe(x)
x = double(x(:)); x = x(isfinite(x));
if numel(x) <= 1, v = 0.0; else, v = std(x); end
end

function v = nanmax_safe(x)
x = double(x(:)); x = x(isfinite(x));
if isempty(x), v = nan; else, v = max(x); end
end

function v = nanlast_safe(x)
x = double(x(:)); idx = find(isfinite(x), 1, 'last');
if isempty(idx), v = nan; else, v = x(idx); end
end

function v = sem_get(semVec, semNames, key, fallback)
v = fallback;
idx = find(string(semNames) == string(key), 1, 'first');
if ~isempty(idx) && numel(semVec) >= idx && isfinite(semVec(idx))
    v = double(semVec(idx));
end

function [vx, vy, vmag] = resolve_wind_vector_features(v, fallbackMag)
vv = double(v);
if isempty(vv)
    vv = fallbackMag;
end

if isvector(vv)
    vv = vv(:);
    if numel(vv) >= 2
        vx = clamp_nan(vv(1), 0.0);
        vy = clamp_nan(vv(2), 0.0);
        vmag = hypot(vx, vy);
        return;
    end
    vmag = clamp_nan(vv(1), clamp_nan(fallbackMag, 0.0));
    vx = vmag;
    vy = 0.0;
    return;
end

if size(vv, 2) >= 2
    vx = nanmean_safe(vv(:,1));
    vy = nanmean_safe(vv(:,2));
    vmag = hypot(vx, vy);
    return;
end

vmag = nanmean_safe(vv(:,1));
if ~isfinite(vmag)
    vmag = clamp_nan(fallbackMag, 0.0);
end
vx = vmag;
vy = 0.0;
end

function y = clamp_nan(x, fallback)
if isfinite(x)
    y = x;
else
    y = fallback;
end
end
end
