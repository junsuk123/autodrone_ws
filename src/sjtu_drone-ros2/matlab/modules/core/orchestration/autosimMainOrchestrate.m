function autosimMainOrchestrate(matlabDir, multiDroneCount)
% autosimMainOrchestrate(matlabDir, multiDroneCount)
% Parallel worker launch + monitor + final merged training orchestration.
%
% Args:
%   matlabDir (string, optional): Path to matlab root directory
%   multiDroneCount (int, optional): Number of drones to spawn. Default: 4

if nargin < 1 || strlength(string(matlabDir)) == 0
    matlabDir = autosimMainResolveMatlabDir('');
else
    matlabDir = autosimMainResolveMatlabDir(matlabDir);
end

if nargin < 2 || isempty(multiDroneCount)
    multiDroneCount = 4;  % Default: 4 drones
end

if strlength(string(matlabDir)) == 0
    error('Could not resolve matlab root directory. Call autosimMainOrchestrate(''/path/to/.../matlab'') or run AutoSimMain.');
end
matlabDir = char(string(matlabDir));

% FIRST: Clean up any lingering Gazebo processes to prevent port conflicts
autosimCleanupGazebo();

cfg = autosimMainBuildConfig();

% Override multiDroneCount from parameter
if nargin >= 2 && ~isempty(multiDroneCount)
    cfg.multiDroneCount = multiDroneCount;
    cfg.runtime.multi_drone_count = multiDroneCount;
    fprintf('[AUTOSIM CONFIG] Override: multiDroneCount = %d (drones + landing pads)\n', multiDroneCount);
end
runScript = fullfile(matlabDir, 'scripts', 'run_autosim_parallel.sh');
stopScript = fullfile(matlabDir, 'scripts', 'stop_autosim_parallel.sh');

if ~isfile(runScript) || ~isfile(stopScript)
    error('Parallel scripts not found under matlab/scripts.');
end

% Ensure previous AutoSim parallel session does not interfere with a new one.
autosimMainStopParallel(stopScript, '');

envPrefix = sprintf(['SCENARIO_COUNT=%d DOMAIN_BASE=%d GAZEBO_PORT_BASE=%d ' ...
    'AUTOSIM_ENABLE_PROGRESS_PLOT=%s AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ=%s ' ...
    'AUTOSIM_DISABLE_INCREMENTAL_TRAIN=%s AUTOSIM_MULTI_DRONE_COUNT=%d ' ...
    'AUTOSIM_MULTI_DRONE_SPACING_M=%.2f AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX=%s ' ...
    'AUTOSIM_MULTI_DRONE_SPAWN_TAGS=%s AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST=%s ' ...
    'AUTOSIM_PRIMARY_DRONE_INDEX=%d AUTOSIM_ROS_LOCALHOST_ONLY=%s'], ...
    cfg.scenarioCount, cfg.domainBase, cfg.gazeboPortBase, ...
    autosimMainBoolText(cfg.enableProgressPlot), autosimMainBoolText(cfg.enableScenarioLiveViz), ...
    autosimMainBoolText(cfg.disableWorkerIncrementalTrain), cfg.multiDroneCount, cfg.multiDroneSpacingM, ...
    cfg.multiDroneNamespacePrefix, autosimMainBoolText(cfg.multiDroneSpawnTags), ...
    autosimMainBoolText(cfg.multiDroneUseWorldTagAsFirst), cfg.primaryDroneIndex, ...
    autosimMainBoolText(cfg.rosLocalhostOnly));

launchCmd = sprintf('cd "%s" && %s "%s" "%s"', matlabDir, envPrefix, runScript, cfg.workersArg);
fprintf('[AUTOSIM MAIN] Launch command: %s\n', launchCmd);

[st, out] = system(launchCmd);
if st ~= 0
    error('Failed to start parallel AutoSim workers:\n%s', out);
end

sessionRoot = autosimMainExtractSessionRoot(out);
if strlength(sessionRoot) == 0
    sessionRoot = autosimMainFindLatestSessionRoot(matlabDir);
end
if strlength(sessionRoot) == 0
    error('Parallel session root not found.\n%s', out);
end

fprintf('[AUTOSIM MAIN] Session root: %s\n', char(sessionRoot));
tailCmd = sprintf('%s %s', fullfile(matlabDir, 'scripts', 'tail_autosim_parallel_logs.sh'), char(sessionRoot));
fprintf('[AUTOSIM MAIN] Unified worker logs command: %s\n', tailCmd);

% Domain bridge is optional. In single-worker local-domain mode it is disabled by default.
if cfg.enableDomainBridge
    bridgeScript = fullfile(matlabDir, 'scripts', 'run_autosim_domain_bridge.sh');
    if isfile(bridgeScript)
        bridgeCmd = sprintf('(source /opt/ros/humble/setup.bash 2>/dev/null && OBSERVE_DOMAIN=90 "%s" "%s") >/dev/null 2>&1 &', bridgeScript, char(sessionRoot));
        fprintf('[AUTOSIM MAIN] Starting domain bridge: %s\n', char(sessionRoot));
        try
            system(bridgeCmd);
            pause(2); % Wait for bridge to initialize
            fprintf('[AUTOSIM MAIN] Domain bridge started (pid monitoring in background)\n');
        catch ME
            warning('[AUTOSIM MAIN] Domain bridge startup warning: %s', ME.message);
        end
    end
end

cleanupObj = onCleanup(@() autosimMainStopParallel(stopScript, char(sessionRoot))); %#ok<NASGU>

addpath(fullfile(matlabDir, 'scripts'));
if cfg.enableParallelMonitor
    try
        monitor_autosim_parallel(char(sessionRoot), cfg.monitorPollSec);
    catch ME
        warning('[AUTOSIM MAIN] Monitor ended: %s', ME.message);
    end
end

if cfg.waitForWorkersToExit
    autosimMainWaitForWorkers(char(sessionRoot), cfg.workerPollSec);
end

if cfg.trainMergedAtEnd
    mergeInfo = autosimMainTrainMergedFromWorkers(matlabDir, char(sessionRoot), cfg);
    if mergeInfo.modelUpdated
        fprintf('[AUTOSIM MAIN] Final merged model saved: %s\n', mergeInfo.modelPath);
    else
        warning('[AUTOSIM MAIN] Final merged training skipped: %s', mergeInfo.skipReason);
    end
end

fprintf('[AUTOSIM MAIN] Exiting. Parallel workers will be stopped now.\n');
end

function matlabDir = autosimMainResolveMatlabDir(seedDir)
matlabDir = "";
requiredRel = {fullfile('scripts', 'run_autosim_parallel.sh'), fullfile('scripts', 'stop_autosim_parallel.sh')};

candidates = strings(0, 1);
if nargin >= 1 && strlength(string(seedDir)) > 0
    candidates(end+1, 1) = string(seedDir); %#ok<AGROW>
end
candidates(end+1, 1) = string(pwd); %#ok<AGROW>
candidates(end+1, 1) = string(fileparts(mfilename('fullpath'))); %#ok<AGROW>

for i = 1:numel(candidates)
    c = autosimMainSanitizePathCandidate(candidates(i));
    if strlength(c) == 0
        continue;
    end

    if isfile(char(c))
        c = string(fileparts(char(c)));
    end

    d = c;
    for depth = 1:10
        if strlength(d) == 0 || strcmp(char(d), filesep)
            break;
        end
        ok = true;
        for k = 1:numel(requiredRel)
            if ~isfile(fullfile(char(d), requiredRel{k}))
                ok = false;
                break;
            end
        end
        if ok
            matlabDir = d;
            return;
        end
        parent = string(fileparts(char(d)));
        if parent == d
            break;
        end
        d = parent;
    end
end
end

function cfg = autosimMainBuildConfig()
cfg = struct();
cfg.workersArg = '1';
cfg.scenarioCount = 300;
cfg.domainBase = 0;
cfg.gazeboPortBase = 13045;
cfg.multiDroneCount = 4;
cfg.multiDroneSpacingM = 3.0;
cfg.multiDroneNamespacePrefix = 'drone_w';
cfg.multiDroneSpawnTags = true;
cfg.multiDroneUseWorldTagAsFirst = false;
cfg.primaryDroneIndex = 1;
cfg.enableProgressPlot = false;
cfg.enableScenarioLiveViz = false;
cfg.monitorPollSec = 2.0;
cfg.workerPollSec = 2.0;
cfg.waitForWorkersToExit = true;
cfg.enableParallelMonitor = true;
cfg.disableWorkerIncrementalTrain = true;
cfg.trainMergedAtEnd = true;
cfg.mergedTrainMinSamples = 20;
cfg.rosLocalhostOnly = false;
cfg.enableDomainBridge = false;

workersEnv = strtrim(getenv('AUTOSIM_MAIN_WORKERS'));
if ~isempty(workersEnv)
    cfg.workersArg = workersEnv;
end

scenarioEnv = strtrim(getenv('AUTOSIM_MAIN_SCENARIO_COUNT'));
if ~isempty(scenarioEnv)
    scenarioN = str2double(scenarioEnv);
    if isfinite(scenarioN) && scenarioN >= 1
        cfg.scenarioCount = round(scenarioN);
    end
end

multiCountEnv = strtrim(getenv('AUTOSIM_MAIN_MULTI_DRONE_COUNT'));
if ~isempty(multiCountEnv)
    n = str2double(multiCountEnv);
    if isfinite(n) && n >= 1
        cfg.multiDroneCount = round(n);
    end
end

multiSpacingEnv = strtrim(getenv('AUTOSIM_MAIN_MULTI_DRONE_SPACING_M'));
if ~isempty(multiSpacingEnv)
    s = str2double(multiSpacingEnv);
    if isfinite(s) && s > 0
        cfg.multiDroneSpacingM = s;
    end
end

primaryIdxEnv = strtrim(getenv('AUTOSIM_MAIN_PRIMARY_DRONE_INDEX'));
if ~isempty(primaryIdxEnv)
    p = str2double(primaryIdxEnv);
    if isfinite(p) && p >= 1
        cfg.primaryDroneIndex = round(p);
    end
end

localhostEnv = strtrim(lower(getenv('AUTOSIM_MAIN_ROS_LOCALHOST_ONLY')));
if ~isempty(localhostEnv)
    cfg.rosLocalhostOnly = any(strcmp(localhostEnv, {'1', 'true', 'yes', 'y', 'on'}));
end

bridgeEnv = strtrim(lower(getenv('AUTOSIM_MAIN_ENABLE_DOMAIN_BRIDGE')));
if ~isempty(bridgeEnv)
    cfg.enableDomainBridge = any(strcmp(bridgeEnv, {'1', 'true', 'yes', 'y', 'on'}));
end

trainMergedEnv = strtrim(lower(getenv('AUTOSIM_MAIN_TRAIN_MERGED_AT_END')));
if ~isempty(trainMergedEnv)
    cfg.trainMergedAtEnd = any(strcmp(trainMergedEnv, {'1', 'true', 'yes', 'y', 'on'}));
end
end

function txt = autosimMainBoolText(tf)
if tf
    txt = 'true';
else
    txt = 'false';
end
end

function sessionRoot = autosimMainExtractSessionRoot(outputText)
sessionRoot = "";
tok = regexp(outputText, '(?m)^\[AUTOSIM\]\s+Session root:\s*([^\r\n]+)', 'tokens', 'once');
if isempty(tok)
    tok = regexp(outputText, '(?m)^Session root:\s*([^\r\n]+)', 'tokens', 'once');
end
if ~isempty(tok)
    sessionRoot = autosimMainSanitizePathCandidate(string(tok{1}));
end
end

function sessionRoot = autosimMainFindLatestSessionRoot(matlabDir)
sessionRoot = "";
rootDir = fullfile(matlabDir, 'parallel_runs');
if ~isfolder(rootDir)
    return;
end

d = dir(rootDir);
d = d([d.isdir]);
d = d(~ismember({d.name}, {'.', '..'}));
if isempty(d)
    return;
end

[~, idx] = max([d.datenum]);
sessionRoot = string(fullfile(d(idx).folder, d(idx).name));
end

function autosimMainStopParallel(stopScript, sessionRoot)
if ~isfile(stopScript)
    return;
end

if nargin < 2
    sessionRoot = '';
end

cleanSessionRoot = autosimMainSanitizePathCandidate(string(sessionRoot));
if endsWith(cleanSessionRoot, "/workers.tsv")
    cleanSessionRoot = string(fileparts(char(cleanSessionRoot)));
end

if strlength(cleanSessionRoot) == 0
    cmd = sprintf('"%s"', stopScript);
else
    cmd = sprintf('"%s" "%s"', stopScript, char(cleanSessionRoot));
end
fprintf('[AUTOSIM MAIN] Stop command: %s\n', cmd);
try
    system(cmd);
catch
end
end

function p = autosimMainSanitizePathCandidate(raw)
p = string(raw);
p = regexprep(p, '[\r\n]+', '');
p = strtrim(p);
p = strip(p, 'both', '"');
end

function autosimMainWaitForWorkers(sessionRoot, pollSec)
pidTable = fullfile(sessionRoot, 'workers.tsv');
if ~isfile(pidTable)
    warning('[AUTOSIM MAIN] workers.tsv not found: %s', pidTable);
    return;
end

fprintf('[AUTOSIM MAIN] Waiting for workers to finish...\n');
lastAlive = -1;
while true
    [allExited, nAlive, totalWorkers] = autosimMainCheckWorkerExitStatus(pidTable);
    if nAlive ~= lastAlive
        fprintf('[AUTOSIM MAIN] Worker status: alive=%d/%d\n', nAlive, totalWorkers);
        lastAlive = nAlive;
    end
    if allExited
        break;
    end
    pause(max(0.2, pollSec));
end
fprintf('[AUTOSIM MAIN] All workers finished.\n');
end

function [allExited, nAlive, totalWorkers] = autosimMainCheckWorkerExitStatus(pidTable)
allExited = false;
nAlive = 0;
totalWorkers = 0;

try
    T = readtable(pidTable, 'FileType', 'text', 'Delimiter', '\t');
catch
    return;
end

if ~ismember('pid', T.Properties.VariableNames)
    return;
end

pids = double(T.pid);
pids = pids(isfinite(pids) & pids > 0);
totalWorkers = numel(pids);
if totalWorkers == 0
    allExited = true;
    return;
end

for i = 1:totalWorkers
    pid = round(pids(i));
    [st, ~] = system(sprintf('kill -0 %d >/dev/null 2>&1', pid));
    if st == 0
        nAlive = nAlive + 1;
    end
end

allExited = (nAlive == 0);
end

function info = autosimMainTrainMergedFromWorkers(matlabDir, sessionRoot, cfgMain)
info = struct();
info.modelUpdated = false;
info.modelPath = "";
info.skipReason = "unknown";

[workerTables, sourceCsvList] = autosimMainCollectWorkerTables(sessionRoot); %#ok<ASGLU>
if isempty(workerTables)
    info.skipReason = "no_worker_dataset";
    return;
end

mergedTbl = workerTables{1};
for i = 2:numel(workerTables)
    mergedTbl = [mergedTbl; workerTables{i}]; %#ok<AGROW>
end

validTrainMask = false(height(mergedTbl), 1);
targetLabels = strings(height(mergedTbl), 1);
if ismember('gt_safe_to_land', mergedTbl.Properties.VariableNames)
    targetLabels = autosimNormalizeActionLabel(mergedTbl.gt_safe_to_land);
    validTrainMask = (targetLabels == "AttemptLanding") | (targetLabels == "HoldLanding");
elseif ismember('label', mergedTbl.Properties.VariableNames)
    targetLabels = autosimNormalizeActionLabel(mergedTbl.label);
    validTrainMask = (targetLabels == "AttemptLanding") | (targetLabels == "HoldLanding");
end

nTrain = sum(validTrainMask);
if nTrain < cfgMain.mergedTrainMinSamples
    info.skipReason = sprintf('insufficient_train_samples_%d', nTrain);
    return;
end

y = targetLabels(validTrainMask);
nAttempt = sum(y == "AttemptLanding");
nHold = sum(y == "HoldLanding");
if (nAttempt < 1) || (nHold < 1)
    info.skipReason = sprintf('class_imbalance_attempt_%d_hold_%d', nAttempt, nHold);
    return;
end

cfg = autosimDefaultConfig();
featNames = cellstr(cfg.model.feature_names);
trainTbl = mergedTbl(validTrainMask, :);
X = zeros(height(trainTbl), numel(featNames));
for i = 1:numel(featNames)
    col = featNames{i};
    if ismember(col, trainTbl.Properties.VariableNames)
        X(:, i) = autosimMainToNumeric(trainTbl.(col));
    else
        X(:, i) = 0;
    end
end

model = autosimTrainGaussianNB(X, y, cfg.model.feature_names, cfg.model.prior_uniform_blend, cfg);
model.schema_version = string(cfg.model.schema_version);
model.n_train = nTrain;
model.n_stable = nAttempt;
model.n_unstable = nHold;
model.n_attempt_landing = nAttempt;
model.n_hold_landing = nHold;
model.stable_ratio = nAttempt / max(1, nTrain);
model.attempt_landing_ratio = model.stable_ratio;
model.minority_ratio = min(nAttempt, nHold) / max(1, nTrain);
model.last_update_scenario = nTrain;

ts = autosimMainTimestamp();
modelDir = fullfile(matlabDir, 'models');
if ~isfolder(modelDir)
    mkdir(modelDir);
end
modelPath = fullfile(modelDir, sprintf('autosim_model_parallel_final_%s.mat', ts));
save(modelPath, 'model');

mergedOutDir = fullfile(sessionRoot, 'output', 'data', 'merged');
if ~isfolder(mergedOutDir)
    mkdir(mergedOutDir);
end
mergedCsv = fullfile(mergedOutDir, sprintf('autosim_dataset_merged_%s.csv', ts));
writetable(mergedTbl, mergedCsv);

info.modelUpdated = true;
info.modelPath = string(modelPath);
info.skipReason = "";
fprintf('[AUTOSIM MAIN] Merged dataset rows=%d train_rows=%d attempt=%d hold=%d\n', height(mergedTbl), nTrain, nAttempt, nHold);
fprintf('[AUTOSIM MAIN] Merged dataset saved: %s\n', mergedCsv);
end

function [tablesOut, sourceCsvList] = autosimMainCollectWorkerTables(sessionRoot)
tablesOut = {};
sourceCsvList = strings(0, 1);

workerDirs = dir(fullfile(sessionRoot, 'output', 'data', 'worker_*'));
if isempty(workerDirs)
    return;
end

for i = 1:numel(workerDirs)
    wdir = fullfile(workerDirs(i).folder, workerDirs(i).name);
    runDirs = dir(fullfile(wdir, '*'));
    runDirs = runDirs([runDirs.isdir]);
    runDirs = runDirs(~ismember({runDirs.name}, {'.', '..'}));
    if isempty(runDirs)
        continue;
    end

    [~, idxLatest] = max([runDirs.datenum]);
    latestRunDir = fullfile(runDirs(idxLatest).folder, runDirs(idxLatest).name);
    csvPath = autosimMainPickWorkerDatasetCsv(latestRunDir);
    if strlength(csvPath) == 0
        continue;
    end

    try
        T = readtable(char(csvPath));
    catch
        warning('[AUTOSIM MAIN] Failed to read worker dataset: %s', char(csvPath));
        continue;
    end

    workerId = i;
    tok = regexp(workerDirs(i).name, 'worker_(\d+)', 'tokens', 'once');
    if ~isempty(tok)
        wid = str2double(tok{1});
        if isfinite(wid)
            workerId = round(wid);
        end
    end

    T.worker_id = repmat(workerId, height(T), 1);
    tablesOut{end+1, 1} = T; %#ok<AGROW>
    sourceCsvList(end+1, 1) = csvPath; %#ok<AGROW>
end
end

function csvPath = autosimMainPickWorkerDatasetCsv(runDir)
csvPath = "";

patterns = {
    'autosim_dataset_*_completed.csv', ...
    'autosim_dataset_latest.csv', ...
    'autosim_dataset_*.csv'
};

for i = 1:numel(patterns)
    dd = dir(fullfile(runDir, patterns{i}));
    if isempty(dd)
        continue;
    end
    [~, idx] = max([dd.datenum]);
    csvPath = string(fullfile(dd(idx).folder, dd(idx).name));
    return;
end
end

function ts = autosimMainTimestamp()
ts = char(string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
end

function v = autosimMainToNumeric(x)
if isnumeric(x)
    v = double(x);
elseif islogical(x)
    v = double(x);
elseif isstring(x)
    v = double(str2double(x));
elseif ischar(x)
    v = double(str2double(string(x)));
elseif iscellstr(x)
    v = double(str2double(string(x)));
else
    try
        v = double(x);
    catch
        v = nan(size(x));
    end
end
v(~isfinite(v)) = 0.0;
end
