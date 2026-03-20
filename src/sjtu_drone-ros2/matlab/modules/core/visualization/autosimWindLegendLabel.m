function label = autosimWindLegendLabel(token)
    token = string(token);
    if strlength(token) == 0 || token == "unknown"
        label = "unknown";
        return;
    end
    label = autosimVizCompactToken(token);
end


