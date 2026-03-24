function AutoSimMain()
% AutoSimMain
% Main integrated pipeline entrypoint.
% Always runs: data collection -> training -> validation -> plotting.

% ================= USER SETTINGS (edit here) =================
mainCfg = struct();

% Use only recent N rows from FinalDataset for train/validation/plots.
% Set to Inf (or <=0) to use full dataset.
mainCfg.dataset_recent_n = 1000;
% Data collection settings (editable in main).
mainCfg.collection = struct();
mainCfg.collection.scenario_count = 10;
mainCfg.collection.drone_count = 3;
mainCfg.collection.independent_per_drone = true;
mainCfg.collection.merge_last_runs = 5;
mainCfg.collection.launch_use_gui = false;
mainCfg.collection.launch_use_rviz = true;
mainCfg.collection.launch_use_teleop = false;
mainCfg.collection.multi_drone_spacing_m = 10.0;
mainCfg.collection.multi_drone_namespace_prefix = 'drone_w';
mainCfg.collection.multi_drone_spawn_tags = true;
mainCfg.collection.multi_drone_use_world_tag_as_first = false;
mainCfg.collection.primary_drone_index = 1;
% Parallel autoscaling controls (applied by run_autosim_parallel.sh).
mainCfg.collection.dynamic_worker_scale = true;
mainCfg.collection.memory_probe_wait_sec = 8;

% Pipeline stages (main always executes all stages).
mainCfg.run_collection = true;
mainCfg.run_training = true;
mainCfg.run_validation = true;
mainCfg.run_plots = true;

% Safety cleanup guards: kill stale sim/ROS viewers before start and on abnormal end.
mainCfg.safe_cleanup_on_start = true;
mainCfg.safe_cleanup_on_error = true;
mainCfg.safe_cleanup_on_exit = true;

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

if mainCfg.safe_cleanup_on_start
    autosimMainForceCleanup('startup', thisDir);
end
cleanupGuard = onCleanup(@() autosimMainCleanupOnExit(mainCfg, thisDir)); %#ok<NASGU>

try
    if mainCfg.run_collection
        c = mainCfg.collection;
        fprintf('[AutoSimMain] Stage 1/4: data collection start (scenarios=%d, drones=%d)\n', ...
            round(c.scenario_count), round(c.drone_count));
        AutoSimCollect(c);
    end

    recentN = double(mainCfg.dataset_recent_n);
    if isfinite(recentN) && recentN > 0
        setenv('AUTOSIM_RECENT_DATASET_N', sprintf('%d', round(recentN)));
        fprintf('[AutoSimMain] Recent dataset window enabled: last %d rows\n', round(recentN));
    else
        setenv('AUTOSIM_RECENT_DATASET_N', '');
        fprintf('[AutoSimMain] Recent dataset window disabled: using full dataset\n');
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
catch ME
    if isfield(mainCfg, 'safe_cleanup_on_error') && mainCfg.safe_cleanup_on_error
        autosimMainForceCleanup('error', thisDir);
    end
    rethrow(ME);
end
end

function autosimMainCleanupOnExit(mainCfg, thisDir)
if isfield(mainCfg, 'safe_cleanup_on_exit') && mainCfg.safe_cleanup_on_exit
    autosimMainForceCleanup('exit', thisDir);
end
end

function autosimMainForceCleanup(reason, thisDir)
fprintf('[AutoSimMain] Safe cleanup (%s): checking stale Gazebo/RViz/ROS processes...\n', char(string(reason)));
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
    warning('[AutoSimMain] Structured cleanup failed (%s). Fallback kill will run.', ME.message);
    system(['bash -i -c "set +m; ' ...
        'pkill -9 gzserver >/dev/null 2>&1 || true; ' ...
        'pkill -9 gzclient >/dev/null 2>&1 || true; ' ...
        'pkill -9 -x rviz2 >/dev/null 2>&1 || true; ' ...
        'pkill -9 -f \"[r]os2 launch sjtu_drone_bringup\" >/dev/null 2>&1 || true; ' ...
        'pkill -9 -f \"[s]jtu_drone_bringup.launch.py\" >/dev/null 2>&1 || true"']);
end
end
