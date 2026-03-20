function y = autosimClamp(x, lo, hi)
    y = min(max(x, lo), hi);
end


