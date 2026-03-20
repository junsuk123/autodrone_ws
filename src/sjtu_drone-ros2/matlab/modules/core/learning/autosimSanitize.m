function x = autosimSanitize(x)
    x(~isfinite(x)) = 0.0;
end


