function AutoSimMain()
% AutoSimMain
% Main integrated pipeline entrypoint.
% Always runs: data collection -> training -> validation -> plotting.

% ================= USER SETTINGS (edit here) =================
mainCfg = struct();

% Data collection settings (editable in main).
mainCfg.collection = struct();
mainCfg.collection.scenario_count = 10;
mainCfg.collection.drone_count = 4;
mainCfg.collection.independent_per_drone = true;
mainCfg.collection.merge_last_runs = 5;
mainCfg.collection.launch_use_gui = false;
mainCfg.collection.launch_use_rviz = true;
mainCfg.collection.launch_use_teleop = false;
mainCfg.collection.multi_drone_spacing_m = 10.0;
mainCfg.collection.multi_drone_namespace_prefix = 'drone_w';
mainCfg.collection.multi_drone_spawn_tags = true;
mainCfg.collection.multi_drone_use_world_tag_as_first = true;
mainCfg.collection.primary_drone_index = 1;

% Pipeline stages (main always executes all stages).
mainCfg.run_collection = true;
mainCfg.run_training = true;
mainCfg.run_validation = true;
mainCfg.run_plots = true;
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

if mainCfg.run_collection
    c = mainCfg.collection;
    fprintf('[AutoSimMain] Stage 1/4: data collection start (scenarios=%d, drones=%d)\n', ...
        round(c.scenario_count), round(c.drone_count));
    AutoSimCollect(c);
end

if mainCfg.run_training
    fprintf('[AutoSimMain] Stage 2/4: training start (FinalDataset all, split=7:3)\n');
    run(fullfile(thisDir, 'AutoSimTrain.m'));
end

if mainCfg.run_validation
    fprintf('[AutoSimMain] Stage 3/4: validation start (FinalDataset all, split=7:3)\n');
    autosim_keep_workspace = true; %#ok<NASGU>
    autoPlot = false; %#ok<NASGU>
    run(fullfile(thisDir, 'AutoSimValidation.m'));
end

if mainCfg.run_plots
    fprintf('[AutoSimMain] Stage 4/4: plotting start\n');
    runDir = ""; %#ok<NASGU>
    outputDir = ""; %#ok<NASGU>
    if evalin('base', 'exist(''validationResult'',''var'')')
        vr = evalin('base', 'validationResult');
        if isstruct(vr) && isfield(vr, 'runDir')
            runDir = char(string(vr.runDir)); %#ok<NASGU>
        end
    end
    run(fullfile(thisDir, 'AutoSimPaperPlots.m'));
end

fprintf('[AutoSimMain] Pipeline complete.\n');
end
