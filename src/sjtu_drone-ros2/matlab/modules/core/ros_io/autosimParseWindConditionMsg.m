function [ws, wd] = autosimParseWindConditionMsg(msg)
    ws = nan;
    wd = nan;
    try
        d = double(msg.data);
        if numel(d) >= 1
            ws = d(1);
        end
        if numel(d) >= 2
            wd = autosimWrapTo180(d(2));
        end
    catch
    end
end


