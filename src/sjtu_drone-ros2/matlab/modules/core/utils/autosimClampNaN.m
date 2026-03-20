function y = autosimClampNaN(x, fallback)
    if ~isfinite(x)
        y = fallback;
    else
        y = x;
    end
end


