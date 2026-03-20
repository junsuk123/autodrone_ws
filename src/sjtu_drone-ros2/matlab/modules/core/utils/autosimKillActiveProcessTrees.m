function autosimKillActiveProcessTrees()
    pids = autosimGetActiveProcessPids();
    if isempty(pids)
        return;
    end
    for i = 1:numel(pids)
        autosimKillTree(pids(i));
    end
end


