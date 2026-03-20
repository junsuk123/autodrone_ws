function [hist, count] = autosimPushScalarHist(hist, count, x)
    if ~isfinite(x)
        return;
    end

    hist(1:end-1) = hist(2:end);
    hist(end) = x;
    count = min(numel(hist), count + 1);
end


