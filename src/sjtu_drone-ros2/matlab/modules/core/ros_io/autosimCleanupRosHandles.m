function autosimCleanupRosHandles(handles)
    if nargin < 1 || isempty(handles)
        return;
    end

    if ~iscell(handles)
        handles = {handles};
    end

    for i = 1:numel(handles)
        obj = handles{i};
        if isempty(obj)
            continue;
        end

        try
            if isobject(obj)
                delete(obj);
            end
        catch
        end

        handles{i} = [];
        pause(0.01);
    end
end


