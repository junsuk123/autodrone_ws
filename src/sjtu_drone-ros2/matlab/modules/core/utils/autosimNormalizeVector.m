function y = autosimNormalizeVector(x)
    x = double(x(:));
    y = zeros(size(x));
    valid = isfinite(x);
    if ~any(valid)
        return;
    end

    xv = x(valid);
    lo = min(xv);
    hi = max(xv);
    if hi <= lo
        y(valid) = 0.0;
        return;
    end

    y(valid) = (xv - lo) ./ (hi - lo);
    y = autosimClamp(y, 0.0, 1.0);
end


