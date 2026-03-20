function pids = autosimGetActiveProcessPids()
    cmd = ['bash -i -c "' ...
        'pgrep -f \"[r]os2 launch sjtu_drone_bringup|[c]omponent_container|[a]priltag|[j]oint_state_publisher|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]viz2|[j]oy_node|[g]azebo|[g]zserver|[g]zclient|[s]pawn_drone|[g]azebo_wind_plugin_node\" || true"'];
    [~, txt] = system(cmd);
    toks = regexp(txt, '(\d+)', 'tokens');
    if isempty(toks)
        pids = [];
        return;
    end

    pids = zeros(numel(toks), 1);
    for i = 1:numel(toks)
        pids(i) = str2double(toks{i}{1});
    end
    pids = unique(pids(isfinite(pids) & pids > 1));
end


