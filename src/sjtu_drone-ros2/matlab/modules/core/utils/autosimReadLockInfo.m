function info = autosimReadLockInfo(lockPath)
    info = struct('pid', nan, 'start_ticks', nan, 'cmdline', "");

    try
        txt = fileread(lockPath);
    catch
        return;
    end

    lines = regexp(txt, '\r?\n', 'split');
    lines = lines(~cellfun(@isempty, lines));
    if isempty(lines)
        return;
    end

    first = strtrim(lines{1});
    if startsWith(first, 'pid=')
        info.pid = str2double(strtrim(extractAfter(string(first), "pid=")));
    else
        info.pid = str2double(first);
    end

    for i = 2:numel(lines)
        line = strtrim(lines{i});
        if startsWith(line, 'start_ticks=')
            info.start_ticks = str2double(strtrim(extractAfter(string(line), "start_ticks=")));
        elseif startsWith(line, 'cmdline=')
            info.cmdline = strtrim(extractAfter(string(line), "cmdline="));
        end
    end
end


