function AutoSimTrain()
% AutoSimTrain
% Data-only training entrypoint using FinalDataset.
% - Loads all FinalDataset records.
% - Splits 70/30 (train/validation).
% - Trains model from train split and saves reports.

% ================= USER SETTINGS (edit here) =================
trainCfg = struct();

% Train/validation split ratio.
trainCfg.train_ratio = 0.7;

% Deterministic split seed.
trainCfg.split_seed = 20260323;

% Output tag appended to saved filenames.
trainCfg.output_tag = "train_only";
% =============================================================

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    error('AutoSimTrain:PathResolveFailed', 'Failed to resolve AutoSimTrain path.');
end

modDir = fullfile(thisDir, 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

% Force training mode even if previous shell set disable flag.
setenv('AUTOSIM_DISABLE_INCREMENTAL_TRAIN', 'false');

cfg = autosimDefaultConfig();
[cfg, ~] = autosimApplyExternalOverride(cfg, thisDir);
[cfg, ~] = autosimApplyRuntimeOverrides(cfg);
if isfield(cfg, 'learning') && isstruct(cfg.learning)
    cfg.learning.enable = true;
end
autosimEnsureDirectories(cfg);

[allTbl, sourceFiles, sourceMode] = autosimTrainLoadDataset(thisDir); %#ok<ASGLU>
if isempty(allTbl)
    error('AutoSimTrain:NoDataset', 'No usable FinalDataset found for training.');
end
allTbl = autosimEnsureOntologyFeatureColumns(allTbl, cfg);

[trainTbl, valTbl, splitInfo] = autosimTrainSplit70_30(allTbl, trainCfg.train_ratio, trainCfg.split_seed);
if isempty(trainTbl)
    error('AutoSimTrain:NoTrainSplit', 'Train split is empty after 70/30 partition.');
end
droneMeta = autosimTrainResolveDroneMeta(cfg, allTbl);

ts = autosimTimestamp();
tag = char(string(trainCfg.output_tag));
if strlength(string(tag)) == 0
    tag = 'train_only';
end

mergedCsv = fullfile(cfg.paths.data_root, sprintf('autosim_dataset_%s_all_%s.csv', tag, ts));
trainCsv = fullfile(cfg.paths.data_root, sprintf('autosim_dataset_%s_train_%s.csv', tag, ts));
valCsv = fullfile(cfg.paths.data_root, sprintf('autosim_dataset_%s_val_%s.csv', tag, ts));
writetable(allTbl, mergedCsv);
writetable(trainTbl, trainCsv);
if ~isempty(valTbl)
    writetable(valTbl, valCsv);
end

[modelPrev, modelInfo] = autosimLoadOrInitModel(cfg); %#ok<NASGU>
scenarioId = max(1, height(trainTbl));
[model, learnInfo] = autosim_learning_engine('incremental_train_and_save', cfg, trainTbl, modelPrev, scenarioId);

summary = autosimTrainBuildSummary(trainTbl, sourceFiles, sourceMode, mergedCsv, learnInfo, cfg, droneMeta);
summaryCsv = fullfile(cfg.paths.data_root, sprintf('autosim_train_summary_%s_%s.csv', tag, ts));
writetable(summary, summaryCsv);

splitSummary = table();
splitSummary.created_at = string(datetime('now'));
splitSummary.n_all = height(allTbl);
splitSummary.n_train = height(trainTbl);
splitSummary.n_val = height(valTbl);
splitSummary.train_ratio = splitInfo.train_ratio;
splitSummary.val_ratio = splitInfo.val_ratio;
splitSummary.seed = splitInfo.seed;
splitSummary.source_files = strjoin(sourceFiles, ';');
if droneMeta.is_multi
    splitSummary.collection_multi_drone_count = droneMeta.count;
    splitSummary.collection_mode = "multi_drone";
end
splitSummaryCsv = fullfile(cfg.paths.data_root, sprintf('autosim_train_split_%s_%s.csv', tag, ts));
writetable(splitSummary, splitSummaryCsv);

if isfield(learnInfo, 'model_updated') && learnInfo.model_updated
    finalModelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_final_%s_%s.mat', tag, ts));
    save(finalModelPath, 'model');
    fprintf('[AutoSimTrain] Model updated and saved: %s\n', finalModelPath);
else
    fprintf('[AutoSimTrain] Model update skipped: %s\n', char(string(learnInfo.skip_reason)));
end

fprintf('[AutoSimTrain] Data source mode: %s\n', sourceMode);
fprintf('[AutoSimTrain] All dataset:   %s (rows=%d)\n', mergedCsv, height(allTbl));
fprintf('[AutoSimTrain] Train dataset: %s (rows=%d)\n', trainCsv, height(trainTbl));
fprintf('[AutoSimTrain] Val dataset:   %s (rows=%d)\n', valCsv, height(valTbl));
fprintf('[AutoSimTrain] Training summary: %s\n', summaryCsv);
fprintf('[AutoSimTrain] Split summary:    %s\n', splitSummaryCsv);
end

function [tbl, sourceFiles, sourceMode] = autosimTrainLoadDataset(thisDir)
tbl = table();
sourceFiles = strings(0, 1);
sourceMode = "none";

[tbl, sourceFiles] = autosimLoadAllFinalDataset(thisDir);
if ~isempty(tbl)
    sourceMode = "final_dataset_all";
end
end

function [trainTbl, valTbl, info] = autosimTrainSplit70_30(T, trainRatio, seed)
if nargin < 2 || ~isfinite(trainRatio)
    trainRatio = 0.7;
end
if nargin < 3 || ~isfinite(seed)
    seed = 20260323;
end

trainRatio = max(0.1, min(0.9, trainRatio));
rng(round(seed), 'twister');

n = height(T);
if n == 0
    trainTbl = T;
    valTbl = T;
    info = struct('train_ratio', trainRatio, 'val_ratio', 1.0 - trainRatio, 'seed', seed);
    return;
end

y = strings(n, 1);
if ismember('gt_safe_to_land', T.Properties.VariableNames)
    y = autosimNormalizeActionLabel(T.gt_safe_to_land);
elseif ismember('label', T.Properties.VariableNames)
    y = autosimNormalizeActionLabel(T.label);
end

validLbl = (y == "AttemptLanding") | (y == "HoldLanding");
idxAll = (1:n)';
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

function S = autosimTrainBuildSummary(T, sourceFiles, sourceMode, mergedCsv, learnInfo, cfg, droneMeta)
S = table();
S.created_at = string(datetime('now'));
S.source_mode = string(sourceMode);
S.source_count = numel(sourceFiles);
S.total_rows = height(T);
S.merged_dataset_path = string(mergedCsv);

if isempty(sourceFiles)
    S.source_files = "";
else
    S.source_files = strjoin(sourceFiles, ';');
end

S.model_updated = false;
S.skip_reason = "";
S.scenario_id = nan;
S.n_train = nan;
S.n_stable = nan;
S.n_unstable = nan;
S.stable_ratio = nan;
S.model_dir = string(cfg.paths.model_dir);
if nargin >= 7 && isstruct(droneMeta) && isfield(droneMeta, 'is_multi') && droneMeta.is_multi
    S.collection_multi_drone_count = droneMeta.count;
    S.collection_mode = "multi_drone";
end

if isstruct(learnInfo)
    if isfield(learnInfo, 'model_updated')
        S.model_updated = logical(learnInfo.model_updated);
    end
    if isfield(learnInfo, 'skip_reason')
        S.skip_reason = string(learnInfo.skip_reason);
    end
    if isfield(learnInfo, 'scenario_id')
        S.scenario_id = double(learnInfo.scenario_id);
    end
    if isfield(learnInfo, 'n_train')
        S.n_train = double(learnInfo.n_train);
    end
    if isfield(learnInfo, 'n_stable')
        S.n_stable = double(learnInfo.n_stable);
    end
    if isfield(learnInfo, 'n_unstable')
        S.n_unstable = double(learnInfo.n_unstable);
    end
    if isfield(learnInfo, 'stable_ratio')
        S.stable_ratio = double(learnInfo.stable_ratio);
    end
end
end

function meta = autosimTrainResolveDroneMeta(cfg, T)
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