function reason = autosimGetStopReason()
    reason = "";
    try
        if isappdata(0, 'AutoSimStopReason')
            reason = string(getappdata(0, 'AutoSimStopReason'));
        end
    catch
        reason = "";
    end
end


