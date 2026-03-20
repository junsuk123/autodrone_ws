function tf = autosimLooksLikeRosDataMessage(msg)
    tf = false;
    try
        if isstruct(msg) && isfield(msg, 'data')
            tf = true;
            return;
        end
        tmp = msg.data; %#ok<NASGU>
        tf = true;
    catch
        tf = false;
    end
end


