function v = autosimNanMean(x)
    x = double(x(:));
    x = x(isfinite(x));
    if isempty(x)
        v = nan;
    else
        v = mean(x);
    end
end


