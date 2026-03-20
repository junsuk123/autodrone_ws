function v = autosimNanStd(x)
    x = double(x(:));
    x = x(isfinite(x));
    if numel(x) < 2
        v = nan;
    else
        v = std(x);
    end
end


