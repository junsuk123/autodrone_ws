function info = autosimStartLaunch(cfg, scenarioCfg, scenarioId)
    autosimCleanupProcesses(cfg);

    cleanupScope = "global";
    if isfield(cfg, 'process') && isfield(cfg.process, 'cleanup_scope')
        cleanupScope = lower(string(cfg.process.cleanup_scope));
    end

    % Never start a new launch if old graph processes are still alive.
    if cleanupScope == "global"
        preSnap = autosimGetActiveProcessSnapshot();
        if strlength(strtrim(preSnap)) > 0
            fprintf('[AUTOSIM] Stale process snapshot before launch:\n%s\n', preSnap);
            autosimCleanupProcesses(cfg);
            preSnap2 = autosimGetActiveProcessSnapshot();
            if strlength(strtrim(preSnap2)) > 0
                error('Cleanup did not converge before launch start. Remaining processes:\n%s', preSnap2);
            end
        end
    end

    logFile = fullfile(cfg.paths.log_dir, sprintf('autosim_launch_s%03d_%s.log', scenarioId, autosimTimestamp()));
    launchCmd = sprintf( ...
        cfg.launch.command_template, ...
        char(cfg.runtime.drone_namespace), ...
        scenarioCfg.hover_height_m, ...
        char(cfg.runtime.drone_namespace), ...
        char(cfg.topics.tag_state));
    if isfield(cfg, 'runtime') && isfield(cfg.runtime, 'launch_env_prefix') && strlength(string(cfg.runtime.launch_env_prefix)) > 0
        launchCmd = sprintf('%s && %s', char(cfg.runtime.launch_env_prefix), launchCmd);
    end
    escCmd = autosimEscapeDq(launchCmd);
    escLog = autosimEscapeDq(logFile);

    bashCmd = sprintf('bash -i -c "%s > \\\"%s\\\" 2>&1 & echo $!"', escCmd, escLog);
    [st, out] = system(bashCmd);
    if st ~= 0
        error('Launch failed: %s', out);
    end

    pid = nan;
    tok = regexp(out, '(\d+)', 'tokens');
    if ~isempty(tok)
        pid = str2double(tok{end}{1});
    end
    if ~isfinite(pid)
        pid = -1;
    end

    info = struct('pid', pid, 'log_file', string(logFile));
    fprintf('[AUTOSIM] Launch started (pid=%d), hover=%.2f, wind=%.2f@%.1f\n', ...
        pid, scenarioCfg.hover_height_m, scenarioCfg.wind_speed, scenarioCfg.wind_dir);
end


