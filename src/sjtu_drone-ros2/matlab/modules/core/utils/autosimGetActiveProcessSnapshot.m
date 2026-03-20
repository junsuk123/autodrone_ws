function out = autosimGetActiveProcessSnapshot()
    cmd = ['bash -i -c "' ...
        'pgrep -af \"[r]os2 launch sjtu_drone_bringup|[c]omponent_container|[a]priltag|[j]oint_state_publisher|[r]obot_state_publisher|[s]tatic_transform_publisher|[r]viz2|[j]oy_node|[g]azebo|[g]zserver|[g]zclient|[s]pawn_drone|[g]azebo_wind_plugin_node\" ' ...
        '| sed -n \"1,120p\" || true"'];
    [~, txt] = system(cmd);
    out = string(txt);
end


