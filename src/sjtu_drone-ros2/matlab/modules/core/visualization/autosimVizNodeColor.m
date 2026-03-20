function c = autosimVizNodeColor(score, invertScale)
    s = autosimClampNaN(score, 0.0);
    s = autosimClamp(s, 0.0, 1.0);
    if invertScale
        s = 1.0 - s;
    end
    c0 = [0.93 0.95 0.98];
    c1 = [0.42 0.78 0.48];
    c = (1.0 - s) * c0 + s * c1;
end


