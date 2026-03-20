function tf = autosimIsStopRequested()
    tf = false;
    try
        if isappdata(0, 'AutoSimStopRequested')
            tf = logical(getappdata(0, 'AutoSimStopRequested'));
        end
    catch
        tf = false;
    end
end


