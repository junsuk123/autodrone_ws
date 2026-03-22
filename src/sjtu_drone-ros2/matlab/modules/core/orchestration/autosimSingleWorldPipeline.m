function autosimSingleWorldPipeline(matlabDir)
% autosimSingleWorldPipeline
% One Gazebo world -> merged dataset -> one-shot training -> holdout validation.

if nargin < 1 || strlength(string(matlabDir)) == 0
    matlabDir = fileparts(mfilename('fullpath'));
end
matlabDir = char(string(matlabDir));

scenarioCount = autosimSingleEnvNumber('AUTOSIM_SINGLE_SCENARIO_COUNT', nan);
multiDroneCount = autosimSingleEnvNumber('AUTOSIM_SINGLE_MULTI_DRONE_COUNT', 4);
mergeLastRuns = autosimSingleEnvNumber('AUTOSIM_SINGLE_MERGE_LAST_RUNS', 5);
independentMode = autosimSingleEnvBool('AUTOSIM_SINGLE_INDEPENDENT_PER_DRONE', true);
if ~isfinite(multiDroneCount) || multiDroneCount < 1
    multiDroneCount = 4;
end
if ~isfinite(mergeLastRuns) || mergeLastRuns < 1
    mergeLastRuns = 5;
end

if independentMode && round(multiDroneCount) > 1
    workerCount = round(multiDroneCount);
    fprintf('[AUTOSIM MAIN] Independent per-drone collection enabled | workers=%d, drones_per_worker=1\n', workerCount);

    setenv('AUTOSIM_MAIN_WORKERS', num2str(workerCount));
    setenv('AUTOSIM_MAIN_MULTI_DRONE_COUNT', '1');
    setenv('AUTOSIM_MAIN_PRIMARY_DRONE_INDEX', '1');
    setenv('AUTOSIM_MAIN_MULTI_DRONE_SPACING_M', '3.0');
    setenv('AUTOSIM_MAIN_ROS_LOCALHOST_ONLY', '0');
    if isfinite(scenarioCount) && scenarioCount >= 1
        setenv('AUTOSIM_MAIN_SCENARIO_COUNT', num2str(round(scenarioCount)));
    end

    autosimMainOrchestrate(matlabDir, 1);
    return;
end

setenv('AUTOSIM_DISABLE_INCREMENTAL_TRAIN', 'true');
setenv('AUTOSIM_MULTI_DRONE_COUNT', num2str(round(multiDroneCount)));
setenv('AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX', 'drone_w');
setenv('AUTOSIM_PRIMARY_DRONE_INDEX', '1');
setenv('AUTOSIM_MAIN_WORKERS', '1');
if isfinite(scenarioCount) && scenarioCount >= 1
    setenv('AUTOSIM_SCENARIO_COUNT', num2str(round(scenarioCount)));
end

fprintf('[AUTOSIM MAIN] Single-world pipeline start | multi_drone_count=%d\n', round(multiDroneCount));
run(fullfile(matlabDir, 'AutoSim.m'));

cfg = autosimDefaultConfig();
[cfg, ~] = autosimApplyExternalOverride(cfg, matlabDir);
[cfg, ~] = autosimApplyRuntimeOverrides(cfg);
autosimEnsureDirectories(cfg);

[mergedTbl, sourceFiles] = autosimSingleMergeRecentDatasets(cfg.paths.data_root, round(mergeLastRuns)); %#ok<ASGLU>
if isempty(mergedTbl)
    warning('[AUTOSIM MAIN] No completed dataset found for post-train/validation.');
    return;
end

ts = autosimTimestamp();
mergedCsv = fullfile(cfg.paths.data_root, sprintf('autosim_dataset_singleworld_merged_%s.csv', ts));
writetable(mergedTbl, mergedCsv);
fprintf('[AUTOSIM MAIN] Merged dataset saved: %s (rows=%d)\n', mergedCsv, height(mergedTbl));

modelPrev = autosimCreatePlaceholderModel(cfg, 'single_world_post_collect');
scenarioId = max(1, height(mergedTbl));
[model, learnInfo] = autosim_learning_engine('incremental_train_and_save', cfg, mergedTbl, modelPrev, scenarioId);

if isfield(learnInfo, 'model_updated') && learnInfo.model_updated
    finalModelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_singleworld_final_%s.mat', ts));
    save(finalModelPath, 'model');
    fprintf('[AUTOSIM MAIN] Final single-world model saved: %s\n', finalModelPath);
else
    warning('[AUTOSIM MAIN] Post-train skipped: %s', char(string(learnInfo.skip_reason)));
end

reportPath = fullfile(cfg.paths.data_root, sprintf('autosim_singleworld_holdout_validation_%s.csv', ts));
autosimSingleWriteHoldoutReport(cfg, mergedTbl, reportPath);
fprintf('[AUTOSIM MAIN] Holdout validation report saved: %s\n', reportPath);
end

function [mergedTbl, sourceFiles] = autosimSingleMergeRecentDatasets(dataRoot, maxRuns)
mergedTbl = table();
sourceFiles = strings(0, 1);

if ~isfolder(dataRoot)
    return;
end

runDirs = dir(dataRoot);
runDirs = runDirs([runDirs.isdir]);
runDirs = runDirs(~ismember({runDirs.name}, {'.', '..'}));
if isempty(runDirs)
    return;
end

[~, ord] = sort([runDirs.datenum], 'descend');
runDirs = runDirs(ord);
takeN = min(numel(runDirs), maxRuns);

for i = 1:takeN
    rdir = fullfile(runDirs(i).folder, runDirs(i).name);
    d = dir(fullfile(rdir, 'autosim_dataset_*_completed.csv'));
    if isempty(d)
        d = dir(fullfile(rdir, 'autosim_dataset_*_interrupted.csv'));
    end
    if isempty(d)
        continue;
    end
    [~, idx] = max([d.datenum]);
    csvPath = fullfile(d(idx).folder, d(idx).name);

    try
        T = readtable(csvPath);
    catch
        continue;
    end

    T.source_run_id = repmat(string(runDirs(i).name), height(T), 1);
    T.source_file = repmat(string(csvPath), height(T), 1);

    if isempty(mergedTbl)
        mergedTbl = T;
    else
        shared = intersect(mergedTbl.Properties.VariableNames, T.Properties.VariableNames, 'stable');
        mergedTbl = [mergedTbl(:, shared); T(:, shared)]; %#ok<AGROW>
    end
    sourceFiles(end+1, 1) = string(csvPath); %#ok<AGROW>
end
end

function autosimSingleWriteHoldoutReport(cfg, T, reportPath)
[X, y] = autosimSingleBuildXY(cfg, T);
if isempty(X) || numel(y) < 20
    warning('[AUTOSIM MAIN] Holdout validation skipped: insufficient labeled samples.');
    return;
end

rng(20260322, 'twister');
n = numel(y);
ord = randperm(n);
nTrain = max(1, floor(0.8 * n));
trIdx = ord(1:nTrain);
teIdx = ord(nTrain+1:end);
if isempty(teIdx)
    teIdx = trIdx;
end

modelCv = autosimTrainGaussianNB(X(trIdx, :), y(trIdx), cfg.model.feature_names, cfg.model.prior_uniform_blend, cfg);
[predLabel, predScore] = autosimPredictGaussianNB(modelCv, X(teIdx, :), cfg); %#ok<ASGLU>

gtSafe = (y(teIdx) == "AttemptLanding");
predLand = (predLabel == "AttemptLanding");

evalTbl = table();
evalTbl.gt_safe = gtSafe;
evalTbl.pred_land = predLand;
evalTbl.valid = true(numel(gtSafe), 1);
de = autosimEvaluateDecisionMetrics(evalTbl);

R = table();
R.n_total = n;
R.n_train = numel(trIdx);
R.n_test = numel(teIdx);
R.accuracy = de.accuracy;
R.precision = de.precision;
R.recall = de.recall;
R.specificity = de.specificity;
R.balanced_accuracy = de.balanced_accuracy;
R.f1 = de.f1;
R.unsafe_landing_rate = de.unsafe_landing_rate;
R.tp = de.tp;
R.fp = de.fp;
R.fn = de.fn;
R.tn = de.tn;
R.created_at = string(datetime('now'));
R.report_type = "single_world_holdout";

writetable(R, reportPath);
end

function [X, y] = autosimSingleBuildXY(cfg, T)
X = [];
y = strings(0, 1);

if isempty(T)
    return;
end

T = autosimEnsureOntologyFeatureColumns(T, cfg);

if ismember('gt_safe_to_land', T.Properties.VariableNames)
    yAll = autosimNormalizeActionLabel(T.gt_safe_to_land);
elseif ismember('label', T.Properties.VariableNames)
    yAll = autosimNormalizeActionLabel(T.label);
else
    return;
end

valid = (yAll == "AttemptLanding") | (yAll == "HoldLanding");
if ~any(valid)
    return;
end

featNames = cellstr(cfg.model.feature_names);
X = zeros(sum(valid), numel(featNames));
for i = 1:numel(featNames)
    fn = featNames{i};
    if ismember(fn, T.Properties.VariableNames)
        col = T.(fn);
        if isnumeric(col) || islogical(col)
            v = double(col);
        else
            v = str2double(string(col));
        end
        v(~isfinite(v)) = 0.0;
        X(:, i) = v(valid);
    else
        X(:, i) = 0.0;
    end
end
y = yAll(valid);
end

function n = autosimSingleEnvNumber(name, defaultVal)
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

function tf = autosimSingleEnvBool(name, defaultVal)
txt = strtrim(lower(getenv(name)));
if isempty(txt)
    tf = logical(defaultVal);
    return;
end
tf = any(strcmp(txt, {'1', 'true', 'yes', 'y', 'on'}));
end