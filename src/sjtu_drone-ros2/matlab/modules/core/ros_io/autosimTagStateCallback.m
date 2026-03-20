function autosimTagStateCallback(cacheKey, varargin)
    msg = [];
    for i = 1:numel(varargin)
        cand = varargin{i};
        if autosimLooksLikeRosDataMessage(cand)
            msg = cand;
            break;
        end
    end
    if isempty(msg)
        return;
    end

    [detected, uTag, vTag, tagErr] = autosimParseTag(msg);
    cache = autosimGetTagStateCache(cacheKey);
    cache.has_message = true;
    cache.rx_count = autosimClampNaN(cache.rx_count, 0) + 1;
    cache.detected = logical(detected);
    cache.u = uTag;
    cache.v = vTag;
    cache.tag_error = tagErr;
    setappdata(0, char(cacheKey), cache);
end


