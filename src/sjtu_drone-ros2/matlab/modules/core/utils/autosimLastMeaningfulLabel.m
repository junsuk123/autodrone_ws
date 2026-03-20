function label = autosimLastMeaningfulLabel(values)
    label = "unknown";
    if isempty(values)
        return;
    end
    values = string(values(:));
    values = strtrim(values);
    validMask = strlength(values) > 0 & values ~= "unknown" & values ~= "<missing>";
    if any(validMask)
        label = values(find(validMask, 1, 'last'));
    end
end


