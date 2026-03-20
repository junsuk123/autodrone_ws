function v = autosimCastFiniteOrDefault(x, defaultVal)
    v = defaultVal;
    try
        x = double(x);
        if ~isempty(x)
            x = x(1);
        end
        if isfinite(x)
            v = x;
        end
    catch
    end
end


