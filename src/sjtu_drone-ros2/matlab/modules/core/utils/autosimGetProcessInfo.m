function proc = autosimGetProcessInfo(pid)
    proc = struct('exists', false, 'start_ticks', nan, 'cmdline', "");
    pid = round(pid);
    if ~(isfinite(pid) && pid > 1)
        return;
    end

    procDir = fullfile('/proc', sprintf('%d', pid));
    if ~exist(procDir, 'dir')
        return;
    end

    proc.exists = true;
    try
        statRaw = strtrim(fileread(fullfile(procDir, 'stat')));
        closeParen = find(statRaw == ')', 1, 'last');
        if ~isempty(closeParen)
            tail = strtrim(statRaw(closeParen + 1:end));
            fields = regexp(tail, '\s+', 'split');
            if numel(fields) >= 20
                proc.start_ticks = str2double(fields{20});
            end
        end
    catch
    end

    try
        cmdRaw = fileread(fullfile(procDir, 'cmdline'));
        cmdRaw(cmdRaw == char(0)) = ' ';
        proc.cmdline = strtrim(string(cmdRaw));
    catch
    end
end


