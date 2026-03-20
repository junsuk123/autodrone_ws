function tf = autosimIsLikelyAutoSimProcess(cmdline)
    cmd = lower(char(cmdline));
    tf = contains(cmd, 'matlab') || contains(cmd, 'autosim');
end


