function autosimClearStopRequest()
    try
        setappdata(0, 'AutoSimStopRequested', false);
        setappdata(0, 'AutoSimStopReason', "");
    catch
    end
end


