function out = autosimTail(x, n)
    x = x(:);
    n = min(numel(x), n);
    if n <= 0
        out = [];
    else
        out = x(end-n+1:end);
    end
end


