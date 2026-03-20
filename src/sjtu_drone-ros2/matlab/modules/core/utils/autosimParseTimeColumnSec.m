function tSec = autosimParseTimeColumnSec(col)
    n = numel(col);
    tSec = nan(n,1);
    if n < 1
        return;
    end

    try
        if isdatetime(col)
            dt = col(:);
            if ~isempty(dt.TimeZone)
                dt.TimeZone = '';
            end
            tSec = seconds(dt - dt(1));
            return;
        end
    catch
    end

    s = string(col(:));
    s = strtrim(s);

    dt = NaT(n,1);
    try
        dt = datetime(s, 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
    catch
    end
    if any(isnat(dt))
        try
            dt2 = datetime(s, 'InputFormat', 'yyyy-MM-dd HH:mm');
            fillMask = isnat(dt) & ~isnat(dt2);
            dt(fillMask) = dt2(fillMask);
        catch
        end
    end
    if any(isnat(dt))
        try
            dt2 = datetime(s);
            fillMask = isnat(dt) & ~isnat(dt2);
            dt(fillMask) = dt2(fillMask);
        catch
        end
    end

    valid = ~isnat(dt);
    if any(valid)
        baseT = dt(find(valid, 1, 'first'));
        tSec(valid) = seconds(dt(valid) - baseT);
    end
end


