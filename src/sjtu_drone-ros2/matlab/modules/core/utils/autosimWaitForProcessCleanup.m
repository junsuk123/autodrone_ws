function autosimWaitForProcessCleanup(timeoutSec)
    t0 = tic;
    while toc(t0) <= timeoutSec
        snap = autosimGetActiveProcessSnapshot();
        if strlength(strtrim(snap)) == 0
            return;
        end
        autosimKillActiveProcessTrees();
        pause(0.25);
    end
end


