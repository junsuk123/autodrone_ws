function v = autosimNormalize01(x, xmin, xmax)
    if ~isfinite(x)
        v = 0.0;
        return;
    end
    den = max(xmax - xmin, 1e-6);
    v = autosimClamp((x - xmin) / den, 0.0, 1.0);
end


