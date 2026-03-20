function cache = autosimGetTagStateCache(cacheKey)
    key = char(string(cacheKey));
    if isappdata(0, key)
        cache = getappdata(0, key);
    else
        cache = struct('has_message', false, 'rx_count', 0, 'detected', false, 'u', nan, 'v', nan, 'tag_error', nan);
    end
end


