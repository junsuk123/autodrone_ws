function enc = autosimEncodeCategory(value, categories, encodings, defaultVal)
    enc = defaultVal;
    for i = 1:numel(categories)
        if strcmp(value, categories{i})
            enc = encodings(i);
            return;
        end
    end
end


