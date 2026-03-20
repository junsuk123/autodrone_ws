function out = autosimPadLen(x, n)
    x = x(:);
    if numel(x) < n
        out = [x; nan(n-numel(x),1)];
    else
        out = x(1:n);
    end
end


