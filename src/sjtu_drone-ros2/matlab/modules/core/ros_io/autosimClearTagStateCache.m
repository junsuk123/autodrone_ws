function autosimClearTagStateCache(cacheKey)
    key = char(string(cacheKey));
    if isappdata(0, key)
        rmappdata(0, key);
    end
end


