function slope = autosimTemporalSlope(x, dt)
    xv = double(x(:));
    valid = isfinite(xv);
    if sum(valid) < 2
        slope = 0.0;
        return;
    end

    xv = xv(valid);
    t = (0:numel(xv)-1)' * max(dt, 1e-3);
    t = t - mean(t);
    xv = xv - mean(xv);
    den = sum(t.^2);
    if den <= 1e-9
        slope = 0.0;
    else
        slope = sum(t .* xv) / den;
    end
end


