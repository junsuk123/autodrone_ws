function AutoSimCollect(collectionCfg)
% AutoSimCollect
% Standalone data collection entrypoint used by AutoSimMain and direct runs.
%
% Usage:
%   AutoSimCollect();
%   AutoSimCollect(struct('scenario_count', 200, 'drone_count', 2));

if nargin < 1 || isempty(collectionCfg)
    collectionCfg = struct();
end

collectionCfg = autosimCollectWithDefaults(collectionCfg);
autosimCollectApplySettings(collectionCfg);
autosimCollectVerifyRuntimeConfig(collectionCfg);

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    error('Failed to resolve AutoSimCollect path.');
end

if isfield(collectionCfg, 'safe_cleanup_on_start') && collectionCfg.safe_cleanup_on_start
    autosimCollectForceCleanup(thisDir, 'startup');
end
cleanupGuard = onCleanup(@() autosimCollectCleanupOnExit(collectionCfg, thisDir)); %#ok<NASGU>

fprintf('[AutoSimCollect] Start collection (scenarios=%d, drones=%d)\n', ...
    round(collectionCfg.scenario_count), round(collectionCfg.drone_count));
setenv('AUTOSIM_DISABLE_INCREMENTAL_TRAIN', 'true');
if logical(collectionCfg.independent_per_drone) && round(collectionCfg.drone_count) > 1
    % Independent mode: scale by workers, one drone per Gazebo worker.
    setenv('AUTOSIM_MAIN_TRAIN_MERGED_AT_END', 'false');
    try
        autosimMainOrchestrate(thisDir, 1);
    catch ME
        if autosimCollectIsUserTermination(ME)
            fprintf('[AutoSimCollect] Stopped by user during parallel monitor.\n');
            return;
        end
        rethrow(ME);
    end
else
    run(fullfile(thisDir, 'AutoSim.m'));
end
fprintf('[AutoSimCollect] Collection complete.\n');
end

function tf = autosimCollectIsUserTermination(ME)
tf = false;
if nargin < 1 || isempty(ME)
    return;
end

try
    id = lower(string(ME.identifier));
catch
    id = "";
end

try
    msg = lower(string(ME.message));
catch
    msg = "";
end

tf = contains(id, "operationterminatedbyuser") || ...
     contains(id, "interrupted") || ...
     contains(msg, "operation terminated by user") || ...
     contains(msg, "operation terminated") || ...
     contains(msg, "terminated by user") || ...
     contains(msg, "interrupt") || ...
     contains(msg, "ctrl+c");

if tf
    return;
end

try
    causes = ME.cause;
    for i = 1:numel(causes)
        if autosimCollectIsUserTermination(causes{i})
            tf = true;
            return;
        end
    end
catch
end
end

function cfg = autosimCollectWithDefaults(cfg)
if ~isfield(cfg, 'scenario_count')
    cfg.scenario_count = 1000;
end
if ~isfield(cfg, 'drone_count')
    cfg.drone_count = 1;
end
if ~isfield(cfg, 'independent_per_drone')
    cfg.independent_per_drone = true;
end
if ~isfield(cfg, 'merge_last_runs')
    cfg.merge_last_runs = 5;
end
if ~isfield(cfg, 'launch_use_gui')
    cfg.launch_use_gui = false;
end
if ~isfield(cfg, 'launch_use_rviz')
    cfg.launch_use_rviz = false;
end
if ~isfield(cfg, 'launch_use_teleop')
    cfg.launch_use_teleop = true;
end
if ~isfield(cfg, 'multi_drone_spacing_m')
    cfg.multi_drone_spacing_m = 10.0;
end
if ~isfield(cfg, 'multi_drone_namespace_prefix')
    cfg.multi_drone_namespace_prefix = 'drone_w';
end
if ~isfield(cfg, 'multi_drone_spawn_tags')
    cfg.multi_drone_spawn_tags = true;
end
if ~isfield(cfg, 'multi_drone_use_world_tag_as_first')
    cfg.multi_drone_use_world_tag_as_first = false;
end
if ~isfield(cfg, 'primary_drone_index')
    cfg.primary_drone_index = 1;
end
if ~isfield(cfg, 'safe_cleanup_on_start')
    cfg.safe_cleanup_on_start = true;
end
if ~isfield(cfg, 'safe_cleanup_on_exit')
    cfg.safe_cleanup_on_exit = true;
end
end

function autosimCollectCleanupOnExit(cfg, thisDir)
if isfield(cfg, 'safe_cleanup_on_exit') && cfg.safe_cleanup_on_exit
    autosimCollectForceCleanup(thisDir, 'exit');
end
end

function autosimCollectForceCleanup(thisDir, reason)
fprintf('[AutoSimCollect] Safe cleanup (%s): checking stale Gazebo/RViz/ROS processes...\n', char(string(reason)));
try
    modDir = fullfile(thisDir, 'modules');
    if exist(modDir, 'dir')
        addpath(modDir);
    end
    coreDir = fullfile(modDir, 'core');
    if exist(coreDir, 'dir')
        addpath(genpath(coreDir));
    end

    cfg = autosimDefaultConfig();
    [cfg, ~] = autosimApplyRuntimeOverrides(cfg);
    autosimCleanupProcesses(cfg);
catch ME
    warning('[AutoSimCollect] Structured cleanup failed (%s). Fallback kill will run.', ME.message);
    system(['bash -i -c "set +m; ' ...
        'pkill -9 gzserver >/dev/null 2>&1 || true; ' ...
        'pkill -9 gzclient >/dev/null 2>&1 || true; ' ...
        'pkill -9 -x rviz2 >/dev/null 2>&1 || true; ' ...
        'pkill -9 -f \"[r]os2 launch sjtu_drone_bringup\" >/dev/null 2>&1 || true; ' ...
        'pkill -9 -f \"[s]jtu_drone_bringup.launch.py\" >/dev/null 2>&1 || true"']);
end
end

function autosimCollectApplySettings(cfg)
if isfield(cfg, 'scenario_count') && isfinite(cfg.scenario_count) && cfg.scenario_count >= 1
    scenarioCountTxt = num2str(round(cfg.scenario_count));
    setenv('AUTOSIM_SINGLE_SCENARIO_COUNT', scenarioCountTxt);
    setenv('AUTOSIM_MAIN_SCENARIO_COUNT', scenarioCountTxt);
    setenv('AUTOSIM_SCENARIO_COUNT', scenarioCountTxt);
end

if isfield(cfg, 'drone_count') && isfinite(cfg.drone_count) && cfg.drone_count >= 1
    droneCount = max(1, round(cfg.drone_count));
    droneCountTxt = num2str(droneCount);
    setenv('AUTOSIM_SINGLE_MULTI_DRONE_COUNT', droneCountTxt);

    independentMode = false;
    if isfield(cfg, 'independent_per_drone')
        independentMode = logical(cfg.independent_per_drone);
    end

    if independentMode && droneCount > 1 && isfield(cfg, 'launch_use_rviz') && logical(cfg.launch_use_rviz)
        warning('[AutoSimCollect] launch_use_rviz=true requested, but forcing false in independent multi-worker mode for stability.');
        cfg.launch_use_rviz = false;
    end

    if independentMode && droneCount > 1
        % One Gazebo per worker, one drone per Gazebo.
        setenv('AUTOSIM_MAIN_WORKERS', droneCountTxt);
        setenv('AUTOSIM_MAIN_MULTI_DRONE_COUNT', '1');
        setenv('AUTOSIM_MULTI_DRONE_COUNT', '1');
        setenv('AUTOSIM_MAIN_PRIMARY_DRONE_INDEX', '1');
        setenv('AUTOSIM_PRIMARY_DRONE_INDEX', '1');
    else
        setenv('AUTOSIM_MAIN_WORKERS', '1');
        setenv('AUTOSIM_MAIN_MULTI_DRONE_COUNT', droneCountTxt);
        setenv('AUTOSIM_MULTI_DRONE_COUNT', droneCountTxt);
    end
end

if isfield(cfg, 'merge_last_runs') && isfinite(cfg.merge_last_runs) && cfg.merge_last_runs >= 1
    setenv('AUTOSIM_SINGLE_MERGE_LAST_RUNS', num2str(round(cfg.merge_last_runs)));
end

if isfield(cfg, 'independent_per_drone')
    setenv('AUTOSIM_SINGLE_INDEPENDENT_PER_DRONE', autosimCollectBoolText(logical(cfg.independent_per_drone)));
end

if isfield(cfg, 'launch_use_gui')
    setenv('AUTOSIM_USE_GUI', autosimCollectBoolText(logical(cfg.launch_use_gui)));
end
if isfield(cfg, 'launch_use_rviz')
    setenv('AUTOSIM_USE_RVIZ', autosimCollectBoolText(logical(cfg.launch_use_rviz)));
end
if isfield(cfg, 'launch_use_teleop')
    setenv('AUTOSIM_USE_TELEOP', autosimCollectBoolText(logical(cfg.launch_use_teleop)));
end

if isfield(cfg, 'dynamic_worker_scale')
    setenv('AUTOSIM_DYNAMIC_WORKER_SCALE', autosimCollectBoolText(logical(cfg.dynamic_worker_scale)));
end
if isfield(cfg, 'memory_probe_wait_sec') && isfinite(cfg.memory_probe_wait_sec) && cfg.memory_probe_wait_sec >= 0
    setenv('AUTOSIM_MEMORY_PROBE_WAIT_SEC', num2str(round(double(cfg.memory_probe_wait_sec))));
end

if isfield(cfg, 'multi_drone_spacing_m') && isfinite(cfg.multi_drone_spacing_m) && cfg.multi_drone_spacing_m > 0
    spacingTxt = num2str(double(cfg.multi_drone_spacing_m));
    setenv('AUTOSIM_MAIN_MULTI_DRONE_SPACING_M', spacingTxt);
    setenv('AUTOSIM_MULTI_DRONE_SPACING_M', spacingTxt);
end

if isfield(cfg, 'multi_drone_namespace_prefix') && strlength(string(cfg.multi_drone_namespace_prefix)) > 0
    prefixTxt = char(string(cfg.multi_drone_namespace_prefix));
    setenv('AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX', prefixTxt);
end

if isfield(cfg, 'multi_drone_spawn_tags')
    spawnTagsTxt = autosimCollectBoolText(logical(cfg.multi_drone_spawn_tags));
    setenv('AUTOSIM_MULTI_DRONE_SPAWN_TAGS', spawnTagsTxt);
end

if isfield(cfg, 'multi_drone_use_world_tag_as_first')
    worldTagTxt = autosimCollectBoolText(logical(cfg.multi_drone_use_world_tag_as_first));
    setenv('AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST', worldTagTxt);
end

if isfield(cfg, 'primary_drone_index') && isfinite(cfg.primary_drone_index) && cfg.primary_drone_index >= 1
    idxTxt = num2str(round(cfg.primary_drone_index));
    setenv('AUTOSIM_MAIN_PRIMARY_DRONE_INDEX', idxTxt);
    setenv('AUTOSIM_PRIMARY_DRONE_INDEX', idxTxt);
end
end

function txt = autosimCollectBoolText(tf)
if tf
    txt = 'true';
else
    txt = 'false';
end
end

function autosimCollectVerifyRuntimeConfig(collectionCfg)
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    return;
end

modDir = fullfile(thisDir, 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

try
    cfg = autosimDefaultConfig();
    [cfg, ~] = autosimApplyRuntimeOverrides(cfg);
    requestedCount = max(1, round(double(collectionCfg.drone_count)));
    independentMode = isfield(collectionCfg, 'independent_per_drone') && logical(collectionCfg.independent_per_drone) && requestedCount > 1;
    expectedRuntimeCount = requestedCount;
    if independentMode
        expectedRuntimeCount = 1;
    end
    effectiveCount = max(1, round(double(cfg.runtime.multi_drone_count)));
    fprintf('[AutoSimCollect] Effective runtime drones=%d (requested=%d, expected_per_worker=%d), ns=%s\n', ...
        effectiveCount, requestedCount, expectedRuntimeCount, char(string(cfg.runtime.drone_namespace)));
    if effectiveCount ~= expectedRuntimeCount
        warning('[AutoSimCollect] Runtime override mismatch: expected_per_worker=%d, effective=%d (requested_total=%d)', ...
            expectedRuntimeCount, effectiveCount, requestedCount);
    end
catch ME
    warning('[AutoSimCollect] Runtime verification skipped: %s', ME.message);
end
end
