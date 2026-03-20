function label = autosimPhysicsSourceLabel(val, pathA, pathB, kind)
    if ~isfinite(val)
        label = "default_" + string(kind);
        return;
    end

    pa = string(pathA);
    pb = string(pathB);
    if strlength(pa) > 0 && isfile(char(pa))
        label = "file:" + pa;
    elseif strlength(pb) > 0 && isfile(char(pb))
        label = "file:" + pb;
    else
        label = "resolved_" + string(kind);
    end
end

