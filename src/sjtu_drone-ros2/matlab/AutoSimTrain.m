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
    % Train-only workflow should always produce a fresh model artifact.
    cfg.learning.save_every_scenario = true;
    cfg.learning.update_every_n_scenarios = 1;
end
autosimEnsureDirectories(cfg);

[allTbl, sourceFiles, sourceMode] = autosimTrainLoadDataset(thisDir); %#ok<ASGLU>
if isempty(allTbl)
    error('AutoSimTrain:NoDataset', 'No usable FinalDataset found for training.');
end
allTblRawCount = height(allTbl);
[allTbl, recentNUsed, trainWindowInfo] = autosimTrainApplyRecentWindow(allTbl);
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

summary = autosimTrainBuildSummary(trainTbl, sourceFiles, sourceMode, mergedCsv, learnInfo, cfg, droneMeta, allTblRawCount, recentNUsed);
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
splitSummary.validation_reserved_n = trainWindowInfo.validation_reserved_n;
splitSummary.train_pool_n = trainWindowInfo.train_pool_n;
splitSummary.train_window_requested_n = trainWindowInfo.requested_recent_n;
splitSummary.train_window_used_n = trainWindowInfo.used_recent_n;
splitSummary.train_window_shrunk = logical(trainWindowInfo.was_shrunk);
splitSummary.source_files = strjoin(sourceFiles, ';');
if droneMeta.is_multi
    splitSummary.collection_multi_drone_count = droneMeta.count;
    splitSummary.collection_mode = "multi_drone";
end
splitSummaryCsv = fullfile(cfg.paths.data_root, sprintf('autosim_train_split_%s_%s.csv', tag, ts));
writetable(splitSummary, splitSummaryCsv);

trainPlotPng = fullfile(cfg.paths.plot_root, sprintf('autosim_train_overview_%s_%s.png', tag, ts));
autosimTrainSaveOverviewPlot(trainPlotPng, trainTbl, valTbl, allTblRawCount, trainWindowInfo);

if isfield(learnInfo, 'model_updated') && learnInfo.model_updated
    finalModelPath = fullfile(cfg.paths.model_dir, sprintf('autosim_model_final_%s_%s.mat', tag, ts));
    save(finalModelPath, 'model');
    fprintf('[AutoSimTrain] Model updated and saved: %s\n', finalModelPath);
else
    fprintf('[AutoSimTrain] Model update skipped: %s\n', char(string(learnInfo.skip_reason)));
end

fprintf('[AutoSimTrain] Data source mode: %s\n', sourceMode);
if isfinite(recentNUsed) && recentNUsed > 0
    fprintf('[AutoSimTrain] Recent window: last %d rows (raw=%d, used=%d)\n', round(recentNUsed), allTblRawCount, height(allTbl));
end
fprintf('[AutoSimTrain] Validation reserved rows: %d\n', trainWindowInfo.validation_reserved_n);
if trainWindowInfo.was_shrunk
    fprintf('[AutoSimTrain] Training window shrunk: requested=%d used=%d (pool=%d)\n', ...
        trainWindowInfo.requested_recent_n, trainWindowInfo.used_recent_n, trainWindowInfo.train_pool_n);
end
fprintf('[AutoSimTrain] All dataset:   %s (rows=%d)\n', mergedCsv, height(allTbl));
fprintf('[AutoSimTrain] Train dataset: %s (rows=%d)\n', trainCsv, height(trainTbl));
fprintf('[AutoSimTrain] Val dataset:   %s (rows=%d)\n', valCsv, height(valTbl));
fprintf('[AutoSimTrain] Training summary: %s\n', summaryCsv);
fprintf('[AutoSimTrain] Split summary:    %s\n', splitSummaryCsv);
fprintf('[AutoSimTrain] Training plot:    %s\n', trainPlotPng);
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

function S = autosimTrainBuildSummary(T, sourceFiles, sourceMode, mergedCsv, learnInfo, cfg, droneMeta, rawCount, recentNUsed)
S = table();
S.created_at = string(datetime('now'));
S.source_mode = string(sourceMode);
S.source_count = numel(sourceFiles);
S.total_rows = height(T);
S.total_rows_raw = rawCount;
if isfinite(recentNUsed) && recentNUsed > 0
    S.recent_dataset_n = round(recentNUsed);
else
    S.recent_dataset_n = nan;
end
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

function [T, recentN, info] = autosimTrainApplyRecentWindow(T)
info = struct();
info.validation_reserved_n = 0;
info.train_pool_n = height(T);
info.requested_recent_n = nan;
info.used_recent_n = height(T);
info.was_shrunk = false;

n = height(T);
if n <= 0
    recentN = inf;
    info.used_recent_n = 0;
    return;
end

reserveN = autosimTrainResolveValidationReservedN();
if isfinite(reserveN) && reserveN > 0
    reserveN = min(n, round(reserveN));
    info.validation_reserved_n = reserveN;
    if n <= reserveN
        error('AutoSimTrain:InsufficientTrainRows', ...
            'Not enough data for training after reserving %d validation rows (total=%d).', reserveN, n);
    end
    T = T(1:n-reserveN, :);
end

poolN = height(T);
info.train_pool_n = poolN;
recentN = autosimTrainResolveRecentDatasetN();
if ~(isfinite(recentN) && recentN > 0)
    info.used_recent_n = poolN;
    return;
end

reqN = round(recentN);
info.requested_recent_n = reqN;
if poolN <= 0
    info.used_recent_n = 0;
    return;
end

k = min(poolN, reqN);
T = T(poolN - k + 1:poolN, :);
info.used_recent_n = k;
info.was_shrunk = poolN < reqN;
end

function recentN = autosimTrainResolveRecentDatasetN()
recentN = inf;
raw = string(getenv('AUTOSIM_RECENT_DATASET_N'));
if strlength(raw) == 0
    return;
end
v = str2double(raw);
if isfinite(v) && v > 0
    recentN = round(v);
end
end

function reserveN = autosimTrainResolveValidationReservedN()
reserveN = 0;
raw = string(getenv('AUTOSIM_VALIDATION_FIXED_N'));
if strlength(raw) == 0
    return;
end
v = str2double(raw);
if isfinite(v) && v > 0
    reserveN = round(v);
end
end

function autosimTrainSaveOverviewPlot(outPng, trainTbl, valTbl, rawCount, info)
try
    outDir = fileparts(outPng);
    if ~exist(outDir, 'dir')
        mkdir(outDir);
    end

    fig = figure('Color', 'w', 'Name', 'AutoSim Train Overview', ...
        'Position', [120, 120, 1080, 440], 'Visible', 'on');

    ax1 = subplot(1, 2, 1, 'Parent', fig);
    counts = [height(trainTbl), height(valTbl)];
    bh = bar(ax1, counts, 0.58); %#ok<NASGU>
    ax1.XTick = [1, 2];
    ax1.XTickLabel = {'Train used', 'Validation holdout'};
    ylabel(ax1, 'Scenario Rows');
    title(ax1, 'Train/Validation Scenario Count');
    grid(ax1, 'on');
    ylim(ax1, [0, max(1, 1.15 * max(counts))]);
    text(ax1, 1, counts(1), sprintf('  %d', counts(1)), 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
    text(ax1, 2, counts(2), sprintf('  %d', counts(2)), 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

    ax2 = subplot(1, 2, 2, 'Parent', fig);
    labels = categorical({'Raw all', 'Train pool', 'Train used'});
    values = [rawCount, info.train_pool_n, height(trainTbl)];
    bar(ax2, labels, values, 0.55);
    ylabel(ax2, 'Rows');
    title(ax2, sprintf('Training Window (reserved val=%d)', info.validation_reserved_n));
    grid(ax2, 'on');
    ylim(ax2, [0, max(1, 1.15 * max(values))]);

    if info.was_shrunk
        shrinkText = sprintf('Train window shrunk: requested=%d, used=%d', ...
            info.requested_recent_n, info.used_recent_n);
    else
        shrinkText = sprintf('Train window used=%d', info.used_recent_n);
    end
    annotation(fig, 'textbox', [0.06 0.01 0.88 0.08], ...
        'String', shrinkText, 'EdgeColor', 'none', 'HorizontalAlignment', 'center');

    exportgraphics(fig, outPng, 'Resolution', 180);
catch ME
    warning('[AutoSimTrain] Failed to save training overview plot: %s', ME.message);
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