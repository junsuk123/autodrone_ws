    % AutoSim.m
% Integrated autonomous simulation + incremental learning pipeline.
% It combines online landing decision, per-scenario validation, and model updates.
%
% Workflow:
% 1) Load latest model if present. If no model exists, start cold and bootstrap from collected labels.
% 2) For each scenario: launch -> collect sensor state -> build semantic states/relations -> infer landing feasibility -> decide land/abort.
% 3) Incrementally retrain model with accumulated dataset and save a new model snapshot.
% 4) Use updated model from the next scenario onward.
% 5) On user interrupt, save current plots/model/dataset/checkpoint and exit safely.
%
% Usage:
%   run('/home/j/INCSL/IICC26_ws/src/sjtu_drone-ros2/matlab/AutoSim.m')

clear; clc; close all;

% Close stale figures without invoking outdated CloseRequestFcn callbacks.
try
    figs = findall(0, 'Type', 'figure');
    for i = 1:numel(figs)
        try
            set(figs(i), 'CloseRequestFcn', 'closereq');
        catch
        end
    end
    if ~isempty(figs)
        close(figs);
    end
catch
    try
        delete(findall(0, 'Type', 'figure'));
    catch
    end
end

thisDir = fileparts(mfilename('fullpath'));
if ~isempty(thisDir)
    addpath(thisDir);
    modDir = fullfile(thisDir, 'modules');
    if exist(modDir, 'dir')
        addpath(modDir);
    end
    coreDir = fullfile(modDir, 'core');
    if exist(coreDir, 'dir')
        addpath(genpath(coreDir));
    end
    autosimAddGeneratedMsgPath(thisDir);
end

autosimClearStopRequest();

% Ensure latest gate logic is loaded even when MATLAB keeps function cache.
try
    clear('autosimSendToFleet');
catch
end

cfg = autosimDefaultConfig();
[cfg, overrideInfo] = autosimApplyExternalOverride(cfg, thisDir);
[cfg, runtimeInfo] = autosimApplyRuntimeOverrides(cfg);
[cfg, windLimitInfo] = autosimApplyWindPhysicsLimits(cfg);
autosimEnsureDirectories(cfg);
lockCleanup = autosimAcquireLock(cfg); %#ok<NASGU>

if overrideInfo.loaded
    fprintf('[AUTOSIM] External override applied: %s\n', overrideInfo.path);
end
fprintf('[AUTOSIM] Worker %d/%d | cleanup_scope=%s | domain=%s | gazebo_port=%s\n', ...
    cfg.runtime.worker_id, cfg.runtime.worker_count, char(string(cfg.process.cleanup_scope)), ...
    autosimNumToText(runtimeInfo.domain_id), autosimNumToText(runtimeInfo.gazebo_port));
if windLimitInfo.applied
    fprintf('[AUTOSIM] Wind physics src[mass=%s,thrust=%s] m=%.3fkg Tmax=%.2fN margin=%.2fN hover=%.2fm/s landing=%.2fm/s\n', ...
        char(string(windLimitInfo.mass_source)), char(string(windLimitInfo.thrust_source)), ...
        windLimitInfo.mass_kg, windLimitInfo.max_total_thrust_n, windLimitInfo.thrust_margin_n, ...
        windLimitInfo.hover_limit_mps, windLimitInfo.landing_limit_mps);
end

% Worker start cleanup: kill stale per-worker Gazebo/ROS remnants before creating ROS context.
autosimCleanupProcesses(cfg);
pause(cfg.process.kill_settle_sec);

fprintf('\n[AUTOSIM] Start at %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('[AUTOSIM] Scenario count: %d\n', cfg.scenario.count);

results = {};  % Use cell array to avoid struct array concatenation issues
runStatus = "completed";
runError = [];
rosCtx = [];

traceStore = table();
learningHistory = table();
plotState = autosimInitPlots(cfg);
model = autosimCreatePlaceholderModel(cfg, 'pre_init');

try
    [model, modelInfo] = autosimLoadOrInitModel(cfg);
    fprintf('[AUTOSIM] Initial model source: %s\n', modelInfo.source);

    launchInfo = struct('pid', -1, 'log_file', "");
    launchActive = false;
    rosCtx = autosimCreateRosContext(cfg);

    for scenarioId = 1:cfg.scenario.count
        if autosimIsStopRequested()
            runStatus = "interrupted";
            fprintf('[AUTOSIM] Stop requested before scenario start: %s\n', autosimGetStopReason());
            break;
        end

        fprintf('\n[AUTOSIM] Scenario %d/%d\n', scenarioId, cfg.scenario.count);

        resultsStruct = autosimCellToStructArray(results);
        datasetState = autosimAnalyzeDatasetState(cfg, resultsStruct, traceStore, learningHistory);
        scenarioPolicy = autosimChooseScenarioPolicy(cfg, datasetState, scenarioId);
        scenarioCfg = autosimBuildAdaptiveScenarioConfig(cfg, scenarioId, scenarioPolicy, datasetState);

        fprintf('[AUTOSIM] Adaptive policy=%s | stable=%.2f unstable=%.2f boundary=%.2f unsafeRate=%.2f\n', ...
            scenarioPolicy.mode, datasetState.stableRatio, 1.0 - datasetState.stableRatio, ...
            datasetState.boundarySampleRatio, datasetState.unsafeLandingRate);
        if isfield(scenarioCfg, 'target_case')
            fprintf('[AUTOSIM] Curriculum target_case=%s | wind=%.2f@%.1f\n', ...
                char(string(scenarioCfg.target_case)), scenarioCfg.wind_speed, scenarioCfg.wind_dir);
        end

        if (~launchActive) || (~isfield(cfg.process, 'reuse_simulation_with_reset')) || (~cfg.process.reuse_simulation_with_reset)
            launchInfo = autosimStartLaunch(cfg, scenarioCfg, scenarioId);
            launchActive = true;
            preRunPauseSec = cfg.launch.warmup_sec;
        else
            resetOk = autosimResetSimulationForScenario(cfg, rosCtx, scenarioId, scenarioCfg);
            if ~resetOk
                warning('[AUTOSIM] Reset path failed, relaunching simulation for scenario %d.', scenarioId);
                launchInfo = autosimStartLaunch(cfg, scenarioCfg, scenarioId);
                launchActive = true;
                preRunPauseSec = cfg.launch.warmup_sec;
            else
                preRunPauseSec = 0.0;
            end
        end

        try
            if preRunPauseSec > 0
                pause(preRunPauseSec);
            end
            [scenarioResult, scenarioTrace] = autosimRunScenario(cfg, scenarioCfg, scenarioId, model, rosCtx);
            scenarioResult.launch_pid = launchInfo.pid;
            scenarioResult.launch_log = launchInfo.log_file;
        catch ME
            scenarioResult = autosimEmptyScenarioResult();
            scenarioResult.scenario_id = scenarioId;
            if exist('scenarioCfg', 'var') && isfield(scenarioCfg, 'policy_mode')
                scenarioResult.scenario_policy = string(scenarioCfg.policy_mode);
            end
            if exist('scenarioCfg', 'var') && isfield(scenarioCfg, 'target_case')
                scenarioResult.target_case = string(scenarioCfg.target_case);
            end
            scenarioResult.label = "unstable";
            scenarioResult.success = false;
            if autosimIsUserInterrupt(ME)
                scenarioResult.failure_reason = "user_interrupt";
                runStatus = "interrupted";
            else
                scenarioResult.failure_reason = "runtime_exception";
            end
            scenarioResult.exception_message = string(ME.message);
            scenarioTrace = autosimEmptyTraceTable(scenarioId);
            if exist('scenarioCfg', 'var') && isfield(scenarioCfg, 'policy_mode')
                scenarioTrace.scenario_policy = string(scenarioCfg.policy_mode);
            end
            if exist('scenarioCfg', 'var') && isfield(scenarioCfg, 'target_case')
                scenarioTrace.target_case = string(scenarioCfg.target_case);
            end
            warning('[AUTOSIM] Scenario %d exception: %s', scenarioId, ME.message);
            fprintf(2, '[AUTOSIM] Scenario %d stack:\n%s\n', scenarioId, getReport(ME, 'extended', 'hyperlinks', 'off'));
        end

        results{end+1, 1} = scenarioResult;  % Use cell array for compatibility
        traceStore = [traceStore; scenarioTrace]; %#ok<AGROW>

        if autosimPipelineTrainEnabled(cfg)
            resultsStruct = autosimCellToStructArray(results);
            [model, learnInfo] = autosimIncrementalTrainAndSave(cfg, resultsStruct, model, scenarioId);
        else
            learnInfo = autosimLearningDisabledInfo(scenarioId);
        end
        learningHistory = [learningHistory; struct2table(learnInfo)]; %#ok<AGROW>

        resultsStruct = autosimCellToStructArray(results);
        plotState = autosimUpdatePlots(plotState, resultsStruct, learningHistory);

        autosimSaveCheckpoint(cfg, resultsStruct, traceStore, learningHistory, model, runStatus, "scenario_end");
        autosimPrintStats(resultsStruct, scenarioId, cfg.scenario.count, learnInfo);

        if cfg.process.stop_after_each_scenario && (~isfield(cfg.process, 'reuse_simulation_with_reset') || ~cfg.process.reuse_simulation_with_reset)
            autosimCleanupProcesses(cfg, launchInfo.pid);
            pause(cfg.process.kill_settle_sec);
            launchActive = false;
        end

        if autosimIsStopRequested()
            runStatus = "interrupted";
            fprintf('[AUTOSIM] Stop requested after scenario %d: %s\n', scenarioId, autosimGetStopReason());
        end

        if runStatus == "interrupted"
            fprintf('[AUTOSIM] User interrupt detected. Stop after %d scenario(s).\n', numel(results));
            break;
        end
    end
catch ME
    if autosimIsUserInterrupt(ME)
        runStatus = "interrupted";
        warning('[AUTOSIM] Interrupted by user: %s', ME.message);
    else
        runStatus = "failed";
        runError = ME;
        warning('[AUTOSIM] Fatal exception: %s', ME.message);
    end
end
%%
autosimCleanupProcesses(cfg);
pause(cfg.process.kill_settle_sec);
autosimReleaseRosContext(rosCtx);

resultsStruct = autosimCellToStructArray(results);
finalInfo = autosimFinalize(cfg, resultsStruct, traceStore, learningHistory, model, plotState, runStatus);

if finalInfo.hasValidLabel
    fprintf('[AUTOSIM] Final stable ratio: %.1f%% (%d/%d)\n', ...
        100.0 * finalInfo.stableRatio, finalInfo.nStable, finalInfo.nValid);
end

fprintf('[AUTOSIM] Completed at %s (status=%s)\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'), runStatus);

autosimClearStopRequest();

if ~isempty(runError)
    rethrow(runError);
end

function out = autosimNumToText(v)
if isfinite(v)
    out = sprintf('%d', round(v));
else
    out = 'n/a';
end
end
