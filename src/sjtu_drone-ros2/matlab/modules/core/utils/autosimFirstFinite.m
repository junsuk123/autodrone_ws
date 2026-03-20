function v = autosimFirstFinite(values)
    v = nan;
    values = double(values(:));
    for i = 1:numel(values)
        if isfinite(values(i))
            v = values(i);
            return;
        end
    end
end


