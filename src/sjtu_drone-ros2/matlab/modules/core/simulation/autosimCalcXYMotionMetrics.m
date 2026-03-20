function [xyStd, xySpeedRms] = autosimCalcXYMotionMetrics(xSeq, ySeq, dt)
    xyStd = nan;
    xySpeedRms = nan;

    x = double(xSeq(:));
    y = double(ySeq(:));
    valid = isfinite(x) & isfinite(y);
    x = x(valid);
    y = y(valid);
    if numel(x) < 3
        return;
    end

    cx = mean(x);
    cy = mean(y);
    r = sqrt((x - cx).^2 + (y - cy).^2);
    xyStd = std(r);

    vx = diff(x) ./ max(dt, 1e-3);
    vy = diff(y) ./ max(dt, 1e-3);
    vxy = sqrt(vx.^2 + vy.^2);
    xySpeedRms = sqrt(mean(vxy.^2));
end


