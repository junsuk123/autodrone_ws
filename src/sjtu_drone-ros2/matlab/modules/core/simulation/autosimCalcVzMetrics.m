function [trendLast, oscStd, accelRms] = autosimCalcVzMetrics(vzSeq, dt)
    trendLast = nan;
    oscStd = nan;
    accelRms = nan;

    v = double(vzSeq(:));
    valid = isfinite(v);
    if sum(valid) < 3
        return;
    end

    vFill = v;
    vFill(~valid) = 0.0;
    win = max(3, round(1.0 / max(dt, 1e-3)));
    trend = movmean(vFill, win, 'omitnan');

    validTrend = isfinite(trend) & valid;
    if ~any(validTrend)
        return;
    end

    trendLast = autosimNanLast(trend(validTrend));
    resid = v(validTrend) - trend(validTrend);
    oscStd = autosimNanStd(resid);

    tVec = trend(validTrend);
    if numel(tVec) >= 2
        acc = diff(tVec) ./ max(dt, 1e-3);
        accelRms = sqrt(mean(acc.^2));
    end
end


