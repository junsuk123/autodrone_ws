% AutoSimValidation.m
% Standalone holdout validation using all FinalDataset records.
% - Loads all FinalDataset CSV files.
% - Uses deterministic 70/30 split.
% - Validates latest compatible model on 30% holdout split.
%
% Optional variables before run:
%   modelPath (string/char): explicit model file path.
%   splitSeed (numeric): deterministic split seed.
%   autoPlot (logical): run AutoSimPaperPlots after validation.

if ~exist('autosim_keep_workspace', 'var') || ~logical(autosim_keep_workspace)
    clear;
end
clc; close all force;

rootDir = fileparts(mfilename('fullpath'));
modDir = fullfile(rootDir, 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

cfg = autosimDefaultConfig();
[cfg, ~] = autosimApplyExternalOverride(cfg, rootDir);
[cfg, ~] = autosimApplyRuntimeOverrides(cfg);
autosimEnsureDirectories(cfg);

if ~exist('splitSeed', 'var') || ~isfinite(splitSeed)
    splitSeed = 20260323;
end
if ~exist('autoPlot', 'var')
    autoPlot = false;
end
if ~exist('modelPath', 'var')
    modelPath = "";
end

[allTbl, sourceFiles, finalRoot] = autosimLoadAllFinalDataset(rootDir); %#ok<ASGLU>
if isempty(allTbl)
    error('AutoSimValidation:NoFinalDataset', 'No FinalDataset CSV found under %s', finalRoot);
end
allTbl = autosimEnsureOntologyFeatureColumns(allTbl, cfg);
droneMeta = autosimValidationResolveDroneMeta(cfg, allTbl);

[trainTbl, valTbl, splitInfo] = autosimValidationSplit70_30(allTbl, 0.7, splitSeed); %#ok<ASGLU>
if isempty(valTbl)
    error('AutoSimValidation:EmptyValidationSplit', 'Validation split is empty.');
end

modelPath = autosimValidationResolveModelPath(modelPath, rootDir, cfg.paths.model_dir);
S = load(char(modelPath), 'model');
if ~isfield(S, 'model')
    error('AutoSimValidation:InvalidModelFile', 'Model file does not contain variable "model": %s', char(modelPath));
end
model = S.model;

featureNames = string(cfg.model.feature_names(:));
if isfield(model, 'feature_names') && ~isempty(model.feature_names)
    featureNames = string(model.feature_names(:));
end

X = zeros(height(valTbl), numel(featureNames));
for i = 1:numel(featureNames)
    fn = char(featureNames(i));
    if ismember(fn, valTbl.Properties.VariableNames)
        col = valTbl.(fn);
        if isnumeric(col) || islogical(col)
            X(:, i) = double(col);
        else
            X(:, i) = str2double(string(col));
        end
    end
end
X(~isfinite(X)) = 0.0;

[predLabel, predScore] = autosimPredictGaussianNB(model, X, cfg);

valTbl.pred_decision = autosimNormalizeActionLabel(predLabel);
valTbl.pred_score = double(predScore(:));

if ~ismember('gt_safe_to_land', valTbl.Properties.VariableNames)
    if ismember('label', valTbl.Properties.VariableNames)
        valTbl.gt_safe_to_land = autosimNormalizeActionLabel(valTbl.label);
    else
        error('AutoSimValidation:MissingGroundTruth', 'Validation table has no gt_safe_to_land or label field.');
    end
end

metrics = autosimEvaluateDecisionMetrics(valTbl);

ts = autosimTimestamp();
runDir = fullfile(cfg.paths.data_root, ['validation_holdout_' char(ts)]);
if ~exist(runDir, 'dir')
    mkdir(runDir);
end

datasetCsv = fullfile(runDir, 'autosim_dataset_latest.csv');
summaryCsv = fullfile(runDir, 'autosim_validation_summary.csv');
splitCsv = fullfile(runDir, 'autosim_validation_split.csv');

writetable(valTbl, datasetCsv);

sumTbl = table();
sumTbl.created_at = string(datetime('now'));
sumTbl.model_path = string(modelPath);
sumTbl.n_all = height(allTbl);
sumTbl.n_train = height(trainTbl);
sumTbl.n_val = height(valTbl);
sumTbl.seed = splitSeed;
sumTbl.accuracy = metrics.accuracy;
sumTbl.precision = metrics.precision;
sumTbl.safe_recall = metrics.recall;
sumTbl.unsafe_reject = metrics.specificity;
sumTbl.balanced_accuracy = metrics.balanced_accuracy;
sumTbl.f1 = metrics.f1;
sumTbl.unsafe_landing_rate = metrics.unsafe_landing_rate;
sumTbl.n_excluded_intervention = metrics.n_excluded_intervention;
sumTbl.n_excluded_hover = metrics.n_excluded_hover;
if droneMeta.is_multi
    sumTbl.collection_multi_drone_count = droneMeta.count;
    sumTbl.collection_mode = "multi_drone";
end
writetable(sumTbl, summaryCsv);

splitTbl = table();
splitTbl.created_at = string(datetime('now'));
splitTbl.n_all = height(allTbl);
splitTbl.n_train = height(trainTbl);
splitTbl.n_val = height(valTbl);
splitTbl.train_ratio = splitInfo.train_ratio;
splitTbl.val_ratio = splitInfo.val_ratio;
splitTbl.seed = splitInfo.seed;
splitTbl.source_files = strjoin(sourceFiles, ';');
if droneMeta.is_multi
    splitTbl.collection_multi_drone_count = droneMeta.count;
    splitTbl.collection_mode = "multi_drone";
end
writetable(splitTbl, splitCsv);

fprintf('[AutoSimValidation] model: %s\n', char(modelPath));
fprintf('[AutoSimValidation] all=%d train=%d val=%d (seed=%d)\n', height(allTbl), height(trainTbl), height(valTbl), round(splitSeed));
fprintf('[AutoSimValidation] summary: %s\n', summaryCsv);
fprintf('[AutoSimValidation] accuracy=%.4f precision=%.4f recall=%.4f specificity=%.4f balanced=%.4f unsafe_landing=%.4f\n', ...
    metrics.accuracy, metrics.precision, metrics.recall, metrics.specificity, metrics.balanced_accuracy, metrics.unsafe_landing_rate);

validationResult = struct();
validationResult.runDir = string(runDir);
validationResult.datasetCsv = string(datasetCsv);
validationResult.summaryCsv = string(summaryCsv);
validationResult.splitCsv = string(splitCsv);
validationResult.modelPath = string(modelPath);
assignin('base', 'validationResult', validationResult);

if logical(autoPlot)
    runDir = char(validationResult.runDir); %#ok<NASGU>
    outputDir = ""; %#ok<NASGU>
    run(fullfile(rootDir, 'AutoSimPaperPlots.m'));
end


function [trainTbl, valTbl, info] = autosimValidationSplit70_30(T, trainRatio, seed)
if nargin < 2 || ~isfinite(trainRatio)
    trainRatio = 0.7;
end
if nargin < 3 || ~isfinite(seed)
    seed = 20260323;
end

trainRatio = max(0.1, min(0.9, trainRatio));
rng(round(seed), 'twister');

n = height(T);
idxAll = (1:n)';

y = strings(n, 1);
if ismember('gt_safe_to_land', T.Properties.VariableNames)
    y = autosimNormalizeActionLabel(T.gt_safe_to_land);
elseif ismember('label', T.Properties.VariableNames)
    y = autosimNormalizeActionLabel(T.label);
end

validLbl = (y == "AttemptLanding") | (y == "HoldLanding");
idxTrain = [];
idxVal = [];

if any(validLbl)
    cls = unique(y(validLbl));
    for i = 1:numel(cls)
        idc = idxAll(validLbl & y == cls(i));
        idc = idc(randperm(numel(idc)));
        nTr = max(1, floor(trainRatio * numel(idc)));
        nTr = min(nTr, numel(idc));
        idxTrain = [idxTrain; idc(1:nTr)]; %#ok<AGROW>
        if nTr < numel(idc)
            idxVal = [idxVal; idc(nTr+1:end)]; %#ok<AGROW>
        end
    end

    idxOther = idxAll(~validLbl);
    if ~isempty(idxOther)
        idxOther = idxOther(randperm(numel(idxOther)));
        nTrO = floor(trainRatio * numel(idxOther));
        idxTrain = [idxTrain; idxOther(1:nTrO)]; %#ok<AGROW>
        idxVal = [idxVal; idxOther(nTrO+1:end)]; %#ok<AGROW>
    end
else
    idxShuf = idxAll(randperm(n));
    nTr = max(1, floor(trainRatio * n));
    idxTrain = idxShuf(1:nTr);
    idxVal = idxShuf(nTr+1:end);
end

if isempty(idxVal)
    idxShuf = idxAll(randperm(n));
    nTr = max(1, min(n-1, floor(trainRatio * n)));
    idxTrain = idxShuf(1:nTr);
    idxVal = idxShuf(nTr+1:end);
end

idxTrain = idxTrain(randperm(numel(idxTrain)));
idxVal = idxVal(randperm(numel(idxVal)));
trainTbl = T(idxTrain, :);
valTbl = T(idxVal, :);

info = struct();
info.train_ratio = height(trainTbl) / max(1, n);
info.val_ratio = height(valTbl) / max(1, n);
info.seed = seed;
end


function modelPathOut = autosimValidationResolveModelPath(modelPathIn, rootDir, defaultModelRoot)
if strlength(string(modelPathIn)) > 0
    modelPathOut = string(modelPathIn);
    if ~isfile(char(modelPathOut))
        error('AutoSimValidation:ModelPathNotFound', 'Specified model path does not exist: %s', char(modelPathOut));
    end
    return;
end

roots = string(defaultModelRoot);
parallelRoot = fullfile(rootDir, 'parallel_runs');
if isfolder(parallelRoot)
    runDirs = dir(parallelRoot);
    runDirs = runDirs([runDirs.isdir]);
    runDirs = runDirs(~ismember({runDirs.name}, {'.', '..'}));
    if ~isempty(runDirs)
        [~, ord] = sort([runDirs.datenum], 'descend');
        for i = 1:numel(ord)
            cand = fullfile(runDirs(ord(i)).folder, runDirs(ord(i)).name, 'output', 'models');
            if isfolder(cand)
                roots(end+1, 1) = string(cand); %#ok<AGROW>
                break;
            end
        end
    end
end
roots = unique(roots, 'stable');

files = struct('folder', {}, 'name', {}, 'datenum', {});
for i = 1:numel(roots)
    root = char(roots(i));
    if ~isfolder(root)
        continue;
    end
    d1 = dir(fullfile(root, '**', 'autosim_model_final_*.mat'));
    d2 = dir(fullfile(root, '**', 'autosim_model_*.mat'));
    d = [d1; d2]; %#ok<AGROW>
    if isempty(d)
        continue;
    end
    if isempty(files)
        files = d;
    else
        files = [files; d]; %#ok<AGROW>
    end
end

if isempty(files)
    error('AutoSimValidation:NoModel', 'No model file found under discovered model roots.');
end

[~, idx] = max([files.datenum]);
modelPathOut = string(fullfile(files(idx).folder, files(idx).name));
end

function meta = autosimValidationResolveDroneMeta(cfg, T)
meta = struct();
meta.count = 1;
meta.is_multi = false;
meta.source = "runtime";

if isfield(cfg, 'runtime') && isstruct(cfg.runtime) && isfield(cfg.runtime, 'multi_drone_count')
    n = double(cfg.runtime.multi_drone_count);
    if isfinite(n) && n >= 1
        meta.count = max(1, round(n));
    end
end

if ~isempty(T)
    if ismember('drone_namespace', T.Properties.VariableNames)
        ns = string(T.drone_namespace);
        ns = strtrim(ns);
        ns = ns(strlength(ns) > 0);
        if ~isempty(ns)
            meta.count = max(meta.count, numel(unique(ns, 'stable')));
            meta.source = "dataset_namespace";
        end
    elseif ismember('drone_id', T.Properties.VariableNames)
        ids = T.drone_id;
        if isnumeric(ids) || islogical(ids)
            ids = double(ids);
            ids = ids(isfinite(ids));
            if ~isempty(ids)
                meta.count = max(meta.count, numel(unique(round(ids))));
                meta.source = "dataset_drone_id";
            end
        else
            ids = string(ids);
            ids = strtrim(ids);
            ids = ids(strlength(ids) > 0);
            if ~isempty(ids)
                meta.count = max(meta.count, numel(unique(ids, 'stable')));
                meta.source = "dataset_drone_id";
            end
        end
    end
end

meta.is_multi = meta.count >= 2;
end
