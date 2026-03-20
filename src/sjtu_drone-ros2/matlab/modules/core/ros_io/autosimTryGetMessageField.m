function [ok, value] = autosimTryGetMessageField(obj, names)
    ok = false;
    value = [];
    if nargin < 2 || isempty(names) || isempty(obj)
        return;
    end

    names = string(names(:));

    for i = 1:numel(names)
        nm = char(names(i));
        try
            if isstruct(obj) && isfield(obj, nm)
                value = obj.(nm);
                ok = true;
                return;
            end
        catch
        end

        try
            if isobject(obj) && isprop(obj, nm)
                value = obj.(nm);
                ok = true;
                return;
            end
        catch
        end
    end

    objFields = strings(0,1);
    try
        if isstruct(obj)
            objFields = string(fieldnames(obj));
        elseif isobject(obj)
            objFields = string(properties(obj));
        end
    catch
    end

    if isempty(objFields)
        return;
    end

    lowerFields = lower(objFields);
    for i = 1:numel(names)
        target = lower(strrep(char(names(i)), '-', '_'));
        target = regexprep(target, '[^a-z0-9_]', '');
        for j = 1:numel(objFields)
            cand = lower(strrep(char(objFields(j)), '-', '_'));
            cand = regexprep(cand, '[^a-z0-9_]', '');
            if strcmp(cand, target)
                try
                    value = obj.(char(objFields(j)));
                    ok = true;
                    return;
                catch
                end
            end
        end
    end
end


