function [ok, uPred, vPred] = autosimPredictTagCenter(hist, count, uNow, vNow, tNow, lastDetectT, horizonSec, timeoutSec, samplePeriodSec, minSamples)
    ok = false;
    uPred = nan;
    vPred = nan;

    if isfinite(uNow) && isfinite(vNow)
        uPred = uNow;
        vPred = vNow;
        ok = true;
    end

    if count < max(2, minSamples)
        return;
    end
    if (tNow - lastDetectT) > timeoutSec
        return;
    end

    rows = hist(end-count+1:end, :);
    rows = rows(all(isfinite(rows), 2), :);
    if size(rows,1) < max(2, minSamples)
        return;
    end

    p2 = rows(end, :);
    p1 = rows(end-1, :);
    dt = max(samplePeriodSec, 1e-3);
    vel = (p2 - p1) ./ dt;
    pPred = p2 + vel .* horizonSec;

    uPred = pPred(1);
    vPred = pPred(2);
    ok = isfinite(uPred) && isfinite(vPred);
end


