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
    spawnTagsTxt = 'false';
    if isfield(cfg.runtime, 'multi_drone_spawn_tags') && cfg.runtime.multi_drone_spawn_tags
        spawnTagsTxt = 'true';
    end
    useWorldTagTxt = 'true';
    if isfield(cfg.runtime, 'multi_drone_use_world_tag_as_first') && ~cfg.runtime.multi_drone_use_world_tag_as_first
        useWorldTagTxt = 'false';
    end
    useGuiTxt = 'false';
    if isfield(cfg, 'launch') && isfield(cfg.launch, 'use_gui') && cfg.launch.use_gui
        useGuiTxt = 'true';
    end
    useRvizTxt = 'false';
    if isfield(cfg, 'launch') && isfield(cfg.launch, 'use_rviz') && cfg.launch.use_rviz
        useRvizTxt = 'true';
    end
    useTeleopTxt = 'false';
    if isfield(cfg, 'launch') && isfield(cfg.launch, 'use_teleop') && cfg.launch.use_teleop
        useTeleopTxt = 'true';
    end
    fprintf('[AUTOSIM] Launch config: ns=%s multi=%d spacing=%.2f gui=%s rviz=%s teleop=%s prefix=%s spawn_tags=%s world_tag_first=%s\n', ...
        char(cfg.runtime.drone_namespace), ...
        round(cfg.runtime.multi_drone_count), ...
        cfg.runtime.multi_drone_spacing_m, ...
        useGuiTxt, useRvizTxt, useTeleopTxt, ...
        char(string(cfg.runtime.multi_drone_namespace_prefix)), ...
        spawnTagsTxt, useWorldTagTxt);

    launchCmd = sprintf( ...
        cfg.launch.command_template, ...
        char(cfg.runtime.drone_namespace), ...
        round(cfg.runtime.multi_drone_count), ...
        cfg.runtime.multi_drone_spacing_m, ...
        char(string(cfg.runtime.multi_drone_namespace_prefix)), ...
        spawnTagsTxt, ...
        useWorldTagTxt, ...
        scenarioCfg.hover_height_m, ...
        useGuiTxt, ...
        useRvizTxt, ...
        useTeleopTxt, ...
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


