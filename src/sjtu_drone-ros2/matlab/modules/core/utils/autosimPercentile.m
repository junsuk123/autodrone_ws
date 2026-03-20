function p = autosimPercentile(x, q)
    x = sort(double(x(:)));
    x = x(isfinite(x));
    if isempty(x)
        p = nan;
        return;
    end
    if q <= 0
        p = x(1);
        return;
    end
    if q >= 100
        p = x(end);
        return;
    end

    idx = 1 + (numel(x)-1) * (q/100.0);
    lo = floor(idx);
    hi = ceil(idx);
    if lo == hi
        p = x(lo);
    else
        a = idx - lo;
        p = (1-a) * x(lo) + a * x(hi);
    end
end


