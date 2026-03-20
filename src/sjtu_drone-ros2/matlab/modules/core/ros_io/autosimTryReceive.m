function x = autosimTryReceive(sub, timeout)
    if nargin < 2 || ~isfinite(timeout)
        timeout = 0.01;
    end
    timeout = max(0.0, double(timeout));
    if isempty(sub)
        x = [];
        return;
    end
    try
        x = receive(sub, timeout);
    catch
        x = [];
    end
end


