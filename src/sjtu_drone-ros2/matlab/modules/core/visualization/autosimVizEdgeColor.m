function c = autosimVizEdgeColor(score)
    s = autosimClampNaN(score, 0.0);
    s = autosimClamp(s, 0.0, 1.0);
    c0 = [0.78 0.78 0.78];
    c1 = [0.18 0.52 0.86];
    c = (1.0 - s) * c0 + s * c1;
end


