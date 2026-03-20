function s = autosimComputeTagStabilityScore(jitterPx, warnPx, unsafePx)
    if ~isfinite(jitterPx)
        s = 0.0;
        return;
    end
    if jitterPx <= warnPx
        s = 1.0;
        return;
    end
    s = 1.0 - autosimClamp((jitterPx - warnPx) / max(unsafePx - warnPx, 1e-6), 0.0, 1.0);
end


