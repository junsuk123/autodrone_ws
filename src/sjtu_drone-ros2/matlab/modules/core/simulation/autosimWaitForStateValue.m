function ok = autosimWaitForStateValue(rosCtx, expectedState, timeoutSec)
    ok = false;
    if timeoutSec <= 0
        return;
    end
    if ~isfield(rosCtx, 'subState') || isempty(rosCtx.subState)
        return;
    end

    t0 = tic;
    while toc(t0) < timeoutSec
        stMsg = autosimTryReceive(rosCtx.subState, 0.10);
        if ~isempty(stMsg)
            try
                if double(stMsg.data) == expectedState
                    ok = true;
                    return;
                end
            catch
            end
        end
        pause(0.02);
    end
end


