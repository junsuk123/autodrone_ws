function label = autosimVizLevelText(score, levels)
    s = autosimClampNaN(score, 0.0);
    s = autosimClamp(s, 0.0, 1.0);
    if s < 0.33
        idx = 1;
    elseif s < 0.66
        idx = min(2, numel(levels));
    else
        idx = min(3, numel(levels));
    end
    label = string(levels{idx});
end


