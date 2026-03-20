function out = autosimPadLenString(x, n)
    x = string(x(:));
    if numel(x) < n
        out = [x; repmat("", n-numel(x), 1)];
    else
        out = x(1:n);
    end
end


