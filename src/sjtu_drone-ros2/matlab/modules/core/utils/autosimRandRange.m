function x = autosimRandRange(a, b)
    lo = min(a, b);
    hi = max(a, b);
    x = lo + rand() * (hi - lo);
end


