function autosimCleanupProcesses(cfg, launchPid)
    if nargin < 2
        launchPid = -1;
    end

    if isfinite(launchPid) && launchPid > 1
        autosimKillTree(launchPid);
    end

    cleanupScope = "global";
    if isfield(cfg, 'process') && isfield(cfg.process, 'cleanup_scope')
        cleanupScope = lower(string(cfg.process.cleanup_scope));
    end

    if cleanupScope == "instance"
        autosimCleanupInstanceProcesses(cfg);
        autosimRefreshRos2Daemon();
        pause(max(0.2, cfg.process.kill_settle_sec));
        return;
    end

    preSnap = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(preSnap)) > 0
        fprintf('[AUTOSIM] Cleanup pre-snapshot:\n%s\n', preSnap);
    end

    for pass = 1:3
        system(['bash -i -c "set +m; ' ...
            'pgrep -f \"[r]os2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py\" | xargs -r kill -9 || true; ' ...
            'pkill -9 -f \"[r]os2 launch sjtu_drone_bringup\" || true; ' ...
            'pkill -9 -f \"[s]jtu_drone_bringup.launch.py\" || true; ' ...
            'pkill -9 -f \"[c]omponent_container\" || true; ' ...
            'pkill -9 -f \"[a]priltag\" || true; ' ...
            'pkill -9 -f \"[s]tatic_transform_publisher\" || true; ' ...
            'pkill -9 -f \"[r]obot_state_publisher\" || true; ' ...
            'pkill -9 -f \"[j]oint_state_publisher\" || true; ' ...
            'pkill -9 -f \"[s]pawn_drone\" || true; ' ...
            'pkill -9 -f \"[t]eleop_joystick\" || true; ' ...
            'pkill -9 -f \"[t]eleop_node\" || true; ' ...
            'pkill -9 -f \"[s]jtu_drone_control.*teleop\" || true; ' ...
            'pkill -9 -f \"[j]oy_node\" || true; ' ...
            'pkill -9 -f \"[g]azebo_wind_plugin_node\" || true; ' ...
            'pkill -9 gzserver || true; ' ...
            'pkill -9 gzclient || true; ' ...
            'pkill -9 -x joy_node || true; ' ...
            'pkill -9 -x teleop_node || true; ' ...
            'pkill -9 -x static_transform_publisher || true; ' ...
            'pkill -9 -x robot_state_publisher || true; ' ...
            'pkill -9 -x joint_state_publisher || true; ' ...
            'pkill -9 -x rviz2 || true; ' ...
            'pkill -9 -f \"[r]viz2\" || true" 2>/dev/null']);
        pause(0.25 * pass);
    end

    autosimKillActiveProcessTrees();
    autosimRefreshRos2Daemon();

    verifyTimeout = 8.0;
    if isfield(cfg, 'process') && isfield(cfg.process, 'cleanup_verify_timeout_sec') && isfinite(cfg.process.cleanup_verify_timeout_sec)
        verifyTimeout = max(1.0, cfg.process.cleanup_verify_timeout_sec);
    end
    autosimWaitForProcessCleanup(verifyTimeout);

    % Final hard pass for frequent duplicate-node offenders.
    system(['bash -i -c "set +m; ' ...
        'pkill -9 -x joy_node || true; ' ...
        'pkill -9 -x teleop_node || true; ' ...
        'pkill -9 -x static_transform_publisher || true; ' ...
        'pkill -9 -x robot_state_publisher || true; ' ...
        'pkill -9 -x joint_state_publisher || true; ' ...
        'pkill -9 -x rviz2 || true" 2>/dev/null']);
    autosimWaitForProcessCleanup(2.0);

    postSnap = autosimGetActiveProcessSnapshot();
    if strlength(strtrim(postSnap)) > 0
        fprintf('[AUTOSIM] Cleanup post-snapshot (still alive):\n%s\n', postSnap);
    end

    pause(max(0.2, cfg.process.kill_settle_sec));
end


function autosimCleanupInstanceProcesses(cfg)
    ns = "";
    domainId = nan;
    gazeboPort = nan;

    if isfield(cfg, 'runtime') && isstruct(cfg.runtime)
        if isfield(cfg.runtime, 'drone_namespace')
            ns = string(cfg.runtime.drone_namespace);
        end
        if isfield(cfg.runtime, 'domain_id')
            domainId = double(cfg.runtime.domain_id);
        end
        if isfield(cfg.runtime, 'gazebo_port')
            gazeboPort = double(cfg.runtime.gazebo_port);
        end
    end

    ns = strtrim(ns);
    nsPattern = regexprep(char(ns), '([\\.^$|()\[\]{}*+?])', '\\$1');
    scopeMsg = sprintf('ns=%s domain=%s port=%s', char(defaultText(ns, "n/a")), numText(domainId), numText(gazeboPort));
    fprintf('[AUTOSIM] Instance cleanup (%s)\n', scopeMsg);

    cmdParts = {'set +m'};
    if strlength(ns) > 0
        cmdParts{end+1} = sprintf('pkill -9 -f ''[r]os2 launch sjtu_drone_bringup.*%s'' || true', nsPattern); %#ok<AGROW>
        cmdParts{end+1} = sprintf('pkill -9 -f ''[a]priltag.*%s'' || true', nsPattern); %#ok<AGROW>
        cmdParts{end+1} = sprintf('pkill -9 -f ''[s]pawn_drone.*%s'' || true', nsPattern); %#ok<AGROW>
        cmdParts{end+1} = sprintf('pkill -9 -f ''[s]pawn_apriltag.*%s'' || true', nsPattern); %#ok<AGROW>
        cmdParts{end+1} = sprintf('pkill -9 -f ''[r]obot_state_publisher.*%s'' || true', nsPattern); %#ok<AGROW>
        cmdParts{end+1} = sprintf('pkill -9 -f ''[j]oint_state_publisher.*%s'' || true', nsPattern); %#ok<AGROW>
        cmdParts{end+1} = sprintf('pkill -9 -f ''[s]tatic_transform_publisher.*%s'' || true', nsPattern); %#ok<AGROW>
    end
    if isfinite(gazeboPort)
        pnum = round(gazeboPort);
        cmdParts{end+1} = sprintf(['for p in $(ss -ltnp ''( sport = :%d )'' 2>/dev/null | ' ...
            'awk -F''pid='' ''NF>1 {split($2,a,","); print a[1]}'' | ' ...
            'tr -d ''[:space:]'' | grep -E ''^[0-9]+$'' | sort -u); do ' ...
            'kill -9 $p >/dev/null 2>&1 || true; done'], pnum); %#ok<AGROW>
    end
    cmd = sprintf('bash -i -c "%s" 2>/dev/null', strjoin(cmdParts, '; '));
    system(cmd);

    % Retry once after short settle to catch delayed children.
    pause(0.3);
    system(cmd);
end


function t = numText(v)
    if isfinite(v)
        t = sprintf('%d', round(v));
    else
        t = 'n/a';
    end
end


function out = defaultText(v, fallback)
    if strlength(v) > 0
        out = v;
    else
        out = fallback;
    end
end


