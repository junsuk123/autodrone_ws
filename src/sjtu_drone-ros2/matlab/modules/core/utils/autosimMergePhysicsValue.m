function v = autosimMergePhysicsValue(p, name, fileValue, preferFile, fallback)
    current = fallback;
    if isfield(p, name)
        tmp = double(p.(name));
        if isfinite(tmp)
            current = tmp;
        end
    end

    if preferFile && isfinite(fileValue)
        v = fileValue;
    elseif ~preferFile && isfinite(current)
        v = current;
    elseif isfinite(fileValue)
        v = fileValue;
    else
        v = current;
    end
end


