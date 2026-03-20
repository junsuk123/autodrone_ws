function autosimReleaseRosContext(rosCtx)
    if nargin < 1 || isempty(rosCtx)
        return;
    end

    if isstruct(rosCtx) && isfield(rosCtx, 'tag_cache_key') && strlength(string(rosCtx.tag_cache_key)) > 0
        autosimClearTagStateCache(char(string(rosCtx.tag_cache_key)));
    end

    if isstruct(rosCtx) && isfield(rosCtx, 'cleanupHandles')
        autosimCleanupRosHandles(rosCtx.cleanupHandles);
    elseif isstruct(rosCtx) && isfield(rosCtx, 'node')
        autosimCleanupRosHandles({rosCtx.node});
    end
end


