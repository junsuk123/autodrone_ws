function [hasFreshTag, tagDetected, uTag, vTag, tagErr, rxCountOut] = autosimReadTagInput(subTag, recvTimeoutSec, tagCallbackEnabled, tagCacheKey, lastSeenRxCount)
    hasFreshTag = false;
    tagDetected = false;
    uTag = nan;
    vTag = nan;
    tagErr = nan;
    rxCountOut = lastSeenRxCount;

    if tagCallbackEnabled && strlength(string(tagCacheKey)) > 0
        cache = autosimGetTagStateCache(tagCacheKey);
        rxCountOut = max(lastSeenRxCount, autosimClampNaN(cache.rx_count, 0));
        if cache.has_message && (rxCountOut > lastSeenRxCount)
            hasFreshTag = true;
            tagDetected = logical(cache.detected);
            uTag = cache.u;
            vTag = cache.v;
            tagErr = cache.tag_error;
        end
        return;
    end

    tagMsg = autosimTryReceive(subTag, recvTimeoutSec);
    if ~isempty(tagMsg)
        hasFreshTag = true;
        rxCountOut = lastSeenRxCount + 1;
        [tagDetected, uTag, vTag, tagErr] = autosimParseTag(tagMsg);
    end
end


