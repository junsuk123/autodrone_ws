function v = autosimWindField(s, name, fallback)
    v = fallback;
    if isfield(s, name)
        tmp = double(s.(name));
        if isfinite(tmp)
            v = tmp;
        end
    end
end


