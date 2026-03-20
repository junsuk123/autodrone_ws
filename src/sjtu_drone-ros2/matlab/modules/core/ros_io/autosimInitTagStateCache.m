function autosimInitTagStateCache(cacheKey)
    cache = struct('has_message', false, 'rx_count', 0, 'detected', false, 'u', nan, 'v', nan, 'tag_error', nan);
    setappdata(0, char(cacheKey), cache);
end


