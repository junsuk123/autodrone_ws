function [zOscStd, flipRateHz] = autosimCalcZOscillationMetrics(zSeq, dt)
    zOscStd = nan;
    flipRateHz = nan;

    z = double(zSeq(:));
    z = z(isfinite(z));
    if numel(z) < 4
        return;
    end

    win = max(3, round(0.8 / max(dt, 1e-3)));
    zTrend = movmean(z, win, 'omitnan');
    zResid = z - zTrend;
    zOscStd = std(zResid);

    dz = diff(z);
    sgn = sign(dz);
    sgn = sgn(sgn ~= 0);
    if numel(sgn) < 2
        flipRateHz = 0.0;
        return;
    end

    flips = sum(sgn(2:end) ~= sgn(1:end-1));
    dur = max((numel(z)-1) * dt, 1e-3);
    flipRateHz = flips / dur;
end


