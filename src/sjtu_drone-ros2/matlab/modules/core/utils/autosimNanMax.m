function v = autosimNanMax(x)
    x = double(x(:));
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = max(x);
    end
end


