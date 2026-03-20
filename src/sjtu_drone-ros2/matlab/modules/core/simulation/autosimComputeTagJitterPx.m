function jitterPx = autosimComputeTagJitterPx(histUV, count, minSamples)
    jitterPx = nan;
    if count < max(minSamples, 2)
        return;
    end

    rows = histUV(end-count+1:end, :);
    rows = rows(all(isfinite(rows), 2), :);
    if size(rows, 1) < max(minSamples, 2)
        return;
    end

    xPx = rows(:,1) * 320.0;
    yPx = rows(:,2) * 240.0;
    dxy = diff([xPx, yPx], 1, 1);
    d = sqrt(sum(dxy.^2, 2));
    if isempty(d)
        return;
    end
    jitterPx = sqrt(mean(d.^2));
end


