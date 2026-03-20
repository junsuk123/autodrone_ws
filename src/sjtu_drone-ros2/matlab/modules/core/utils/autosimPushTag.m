function [hist, count] = autosimPushTag(hist, count, u, v)
    if ~isfinite(u) || ~isfinite(v)
        return;
    end

    hist(1:end-1,:) = hist(2:end,:);
    hist(end,:) = [u, v];
    count = min(size(hist,1), count + 1);
end


