function v = autosimSafeDivide(num, den)
    if den <= 0
        v = nan;
    else
        v = num / den;
    end
end


