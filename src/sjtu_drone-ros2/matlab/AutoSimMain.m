function AutoSimMain()
% AutoSimMain
% One-click parallel AutoSim runner for MATLAB Run button.
%
% Behavior:
% 1) Starts parallel AutoSim workers using matlab/scripts/run_autosim_parallel.sh
% 2) Opens lightweight aggregate monitor
% 3) Stops all workers automatically when this function exits/interrupted
clear; clc; close all
thisDir = fileparts(mfilename('fullpath'));
if isempty(thisDir)
	error('Failed to resolve AutoSimMain path.');
end

cfg = autosimMainConfig();
runScript = fullfile(thisDir, 'scripts', 'run_autosim_parallel.sh');
stopScript = fullfile(thisDir, 'scripts', 'stop_autosim_parallel.sh');

if ~isfile(runScript) || ~isfile(stopScript)
	error('Parallel scripts not found under matlab/scripts.');
end

% Ensure previous AutoSim parallel session does not interfere with a new one.
autosimStopParallel(stopScript, '');

envPrefix = sprintf(['SCENARIO_COUNT=%d DOMAIN_BASE=%d GAZEBO_PORT_BASE=%d ' ...
	'AUTOSIM_ENABLE_PROGRESS_PLOT=%s AUTOSIM_ENABLE_SCENARIO_LIVE_VIZ=%s'], ...
	cfg.scenarioCount, cfg.domainBase, cfg.gazeboPortBase, ...
	autosimBoolText(cfg.enableProgressPlot), autosimBoolText(cfg.enableScenarioLiveViz));

launchCmd = sprintf('cd "%s" && %s "%s" "%s"', thisDir, envPrefix, runScript, cfg.workersArg);
fprintf('[AUTOSIM MAIN] Launch command: %s\n', launchCmd);

[st, out] = system(launchCmd);
if st ~= 0
	error('Failed to start parallel AutoSim workers:\n%s', out);
end

sessionRoot = autosimExtractSessionRoot(out);
if strlength(sessionRoot) == 0
	sessionRoot = autosimFindLatestSessionRoot(thisDir);
end
if strlength(sessionRoot) == 0
	error('Parallel session root not found.\n%s', out);
end

fprintf('[AUTOSIM MAIN] Session root: %s\n', char(sessionRoot));
cleanupObj = onCleanup(@() autosimStopParallel(stopScript, char(sessionRoot))); %#ok<NASGU>

addpath(fullfile(thisDir, 'scripts'));
try
	monitor_autosim_parallel(char(sessionRoot), cfg.monitorPollSec);
catch ME
	warning('[AUTOSIM MAIN] Monitor ended: %s', ME.message);
end

fprintf('[AUTOSIM MAIN] Exiting. Parallel workers will be stopped now.\n');
end

function cfg = autosimMainConfig()
cfg = struct();
cfg.workersArg = '4';
cfg.scenarioCount = 300;
cfg.domainBase = 60;
cfg.gazeboPortBase = 13045;
cfg.enableProgressPlot = false;
cfg.enableScenarioLiveViz = false;
cfg.monitorPollSec = 2.0;

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
end

function txt = autosimBoolText(tf)
if tf
	txt = 'true';
else
	txt = 'false';
end
end

function sessionRoot = autosimExtractSessionRoot(outputText)
sessionRoot = "";
tok = regexp(outputText, '(?m)^\[AUTOSIM\]\s+Session root:\s*([^\r\n]+)', 'tokens', 'once');
if isempty(tok)
	tok = regexp(outputText, '(?m)^Session root:\s*([^\r\n]+)', 'tokens', 'once');
end
if ~isempty(tok)
	sessionRoot = autosimSanitizePathCandidate(string(tok{1}));
end
end

function sessionRoot = autosimFindLatestSessionRoot(matlabDir)
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

function autosimStopParallel(stopScript, sessionRoot)
if ~isfile(stopScript)
	return;
end

if nargin < 2
	sessionRoot = '';
end

cleanSessionRoot = autosimSanitizePathCandidate(string(sessionRoot));
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

function p = autosimSanitizePathCandidate(raw)
p = string(raw);
p = regexprep(p, '[\r\n]+', '');
p = strtrim(p);
p = strip(p, 'both', '"');
end
