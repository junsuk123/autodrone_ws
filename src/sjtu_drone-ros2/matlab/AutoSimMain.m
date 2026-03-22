function AutoSimMain()
% AutoSimMain
% Single-world stable pipeline entrypoint.
% Collects data in one Gazebo world, then merges data and runs train/validation.

% ================= USER SETTINGS (edit here) =================
mainCfg = struct();

% Per-drone scenario count (independent mode: scenarios per worker/drone).
mainCfg.scenarios_per_drone = 300;

% Number of drones to run.
mainCfg.drone_count = 4;

% true: one worker per drone (independent collection), false: legacy single-world coupling path.
mainCfg.independent_per_drone = true;

% Post-collection merge window.
mainCfg.merge_last_runs = 5;

% Main launch parameters.
mainCfg.launch_use_gui = false;
mainCfg.launch_use_rviz = true;
mainCfg.launch_use_teleop = false;
mainCfg.multi_drone_spacing_m = 3.0;
mainCfg.multi_drone_namespace_prefix = 'drone_w';
mainCfg.multi_drone_spawn_tags = true;
mainCfg.multi_drone_use_world_tag_as_first = false;
mainCfg.primary_drone_index = 1;
% ==============================================================

thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
    error('Failed to resolve AutoSimMain path.');
end

modDir = fullfile(thisDir, 'modules');
if exist(modDir, 'dir')
    addpath(modDir);
end
coreDir = fullfile(modDir, 'core');
if exist(coreDir, 'dir')
    addpath(genpath(coreDir));
end

autosimMainApplyUserSettings(mainCfg);

autosimSingleWorldPipeline(thisDir);
end

function autosimMainApplyUserSettings(mainCfg)
if isfield(mainCfg, 'scenarios_per_drone') && isfinite(mainCfg.scenarios_per_drone) && mainCfg.scenarios_per_drone >= 1
    scenarioCountTxt = num2str(round(mainCfg.scenarios_per_drone));
    setenv('AUTOSIM_SINGLE_SCENARIO_COUNT', scenarioCountTxt);
    setenv('AUTOSIM_MAIN_SCENARIO_COUNT', scenarioCountTxt);
end

if isfield(mainCfg, 'drone_count') && isfinite(mainCfg.drone_count) && mainCfg.drone_count >= 1
    droneCountTxt = num2str(round(mainCfg.drone_count));
    setenv('AUTOSIM_SINGLE_MULTI_DRONE_COUNT', droneCountTxt);
    setenv('AUTOSIM_MAIN_MULTI_DRONE_COUNT', droneCountTxt);
    setenv('AUTOSIM_MULTI_DRONE_COUNT', droneCountTxt);
end

if isfield(mainCfg, 'merge_last_runs') && isfinite(mainCfg.merge_last_runs) && mainCfg.merge_last_runs >= 1
    setenv('AUTOSIM_SINGLE_MERGE_LAST_RUNS', num2str(round(mainCfg.merge_last_runs)));
end

if isfield(mainCfg, 'independent_per_drone')
    setenv('AUTOSIM_SINGLE_INDEPENDENT_PER_DRONE', autosimMainBoolText(logical(mainCfg.independent_per_drone)));
end

if isfield(mainCfg, 'launch_use_gui')
    setenv('AUTOSIM_USE_GUI', autosimMainBoolText(logical(mainCfg.launch_use_gui)));
end
if isfield(mainCfg, 'launch_use_rviz')
    setenv('AUTOSIM_USE_RVIZ', autosimMainBoolText(logical(mainCfg.launch_use_rviz)));
end
if isfield(mainCfg, 'launch_use_teleop')
    setenv('AUTOSIM_USE_TELEOP', autosimMainBoolText(logical(mainCfg.launch_use_teleop)));
end

if isfield(mainCfg, 'multi_drone_spacing_m') && isfinite(mainCfg.multi_drone_spacing_m) && mainCfg.multi_drone_spacing_m > 0
    spacingTxt = num2str(double(mainCfg.multi_drone_spacing_m));
    setenv('AUTOSIM_MAIN_MULTI_DRONE_SPACING_M', spacingTxt);
    setenv('AUTOSIM_MULTI_DRONE_SPACING_M', spacingTxt);
end

if isfield(mainCfg, 'multi_drone_namespace_prefix') && strlength(string(mainCfg.multi_drone_namespace_prefix)) > 0
    prefixTxt = char(string(mainCfg.multi_drone_namespace_prefix));
    setenv('AUTOSIM_MULTI_DRONE_NAMESPACE_PREFIX', prefixTxt);
end

if isfield(mainCfg, 'multi_drone_spawn_tags')
    spawnTagsTxt = autosimMainBoolText(logical(mainCfg.multi_drone_spawn_tags));
    setenv('AUTOSIM_MULTI_DRONE_SPAWN_TAGS', spawnTagsTxt);
end

if isfield(mainCfg, 'multi_drone_use_world_tag_as_first')
    worldTagTxt = autosimMainBoolText(logical(mainCfg.multi_drone_use_world_tag_as_first));
    setenv('AUTOSIM_MULTI_DRONE_USE_WORLD_TAG_AS_FIRST', worldTagTxt);
end

if isfield(mainCfg, 'primary_drone_index') && isfinite(mainCfg.primary_drone_index) && mainCfg.primary_drone_index >= 1
    idxTxt = num2str(round(mainCfg.primary_drone_index));
    setenv('AUTOSIM_MAIN_PRIMARY_DRONE_INDEX', idxTxt);
    setenv('AUTOSIM_PRIMARY_DRONE_INDEX', idxTxt);
end
end

function txt = autosimMainBoolText(tf)
if tf
    txt = 'true';
else
    txt = 'false';
end
end
