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

fprintf('[AutoSimCollect] Start collection (scenarios=%d, drones=%d)\n', ...
    round(collectionCfg.scenario_count), round(collectionCfg.drone_count));
setenv('AUTOSIM_DISABLE_INCREMENTAL_TRAIN', 'true');
run(fullfile(thisDir, 'AutoSim.m'));
fprintf('[AutoSimCollect] Collection complete.\n');
end

function cfg = autosimCollectWithDefaults(cfg)
if ~isfield(cfg, 'scenario_count')
    cfg.scenario_count = 1000;
end
if ~isfield(cfg, 'drone_count')
    cfg.drone_count = 1;
end
if ~isfield(cfg, 'independent_per_drone')
    cfg.independent_per_drone = false;
end
if ~isfield(cfg, 'merge_last_runs')
    cfg.merge_last_runs = 5;
end
if ~isfield(cfg, 'launch_use_gui')
    cfg.launch_use_gui = false;
end
if ~isfield(cfg, 'launch_use_rviz')
    cfg.launch_use_rviz = true;
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
    cfg.multi_drone_use_world_tag_as_first = true;
end
if ~isfield(cfg, 'primary_drone_index')
    cfg.primary_drone_index = 1;
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
    droneCountTxt = num2str(round(cfg.drone_count));
    setenv('AUTOSIM_SINGLE_MULTI_DRONE_COUNT', droneCountTxt);
    setenv('AUTOSIM_MAIN_MULTI_DRONE_COUNT', droneCountTxt);
    setenv('AUTOSIM_MULTI_DRONE_COUNT', droneCountTxt);
    setenv('AUTOSIM_MAIN_WORKERS', '1');
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
    effectiveCount = max(1, round(double(cfg.runtime.multi_drone_count)));
    fprintf('[AutoSimCollect] Effective runtime drones=%d (requested=%d), ns=%s\n', ...
        effectiveCount, requestedCount, char(string(cfg.runtime.drone_namespace)));
    if effectiveCount ~= requestedCount
        warning('[AutoSimCollect] Runtime override mismatch: requested drones=%d, effective drones=%d', ...
            requestedCount, effectiveCount);
    end
catch ME
    warning('[AutoSimCollect] Runtime verification skipped: %s', ME.message);
end
end
