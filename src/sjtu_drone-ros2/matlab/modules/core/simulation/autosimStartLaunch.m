function info = autosimStartLaunch(cfg, scenarioCfg, scenarioId)
    autosimCleanupProcesses(cfg);

    % Never start a new launch if old graph processes are still alive.
    preSnap = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(preSnap)) > 0
        fprintf('[AUTOSIM] Stale process snapshot before launch:\n%s\n', preSnap);
        autosimCleanupProcesses(cfg);
        preSnap2 = autosimGetActiveProcessSnapshot();
        if strlength(strtrim(preSnap2)) > 0
            error('Cleanup did not converge before launch start. Remaining processes:\n%s', preSnap2);
        end
    end

    logFile = fullfile(cfg.paths.log_dir, sprintf('autosim_launch_s%03d_%s.log', scenarioId, autosimTimestamp()));
    launchCmd = sprintf(cfg.launch.command_template, scenarioCfg.hover_height_m);
    escCmd = autosimEscapeDq(launchCmd);
    escLog = autosimEscapeDq(logFile);

    bashCmd = sprintf('bash -i -c "%s > \\\"%s\\\" 2>&1 &"', escCmd, escLog);
    [st, out] = system(bashCmd);
    if st ~= 0
        error('Launch failed: %s', out);
    end

    pid = -1;
    [~, pOut] = system('bash -i -c "pgrep -n -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" || true"');
    tok = regexp(pOut, '(\d+)', 'tokens');
    if ~isempty(tok)
        pid = str2double(tok{end}{1});
    end

    info = struct('pid', pid, 'log_file', string(logFile));
    fprintf('[AUTOSIM] Launch started (pid=%d), hover=%.2f, wind=%.2f@%.1f\n', ...
        pid, scenarioCfg.hover_height_m, scenarioCfg.wind_speed, scenarioCfg.wind_dir);
end


