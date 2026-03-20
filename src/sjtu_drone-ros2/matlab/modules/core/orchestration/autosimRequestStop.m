function autosimRequestStop(reason)
    if nargin < 1 || strlength(string(reason)) == 0
        reason = "user_stop_requested";
    end

    try
        setappdata(0, 'AutoSimStopRequested', true);
        setappdata(0, 'AutoSimStopReason', string(reason));
    catch
    end
end


