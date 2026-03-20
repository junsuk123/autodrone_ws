function out = autosimLastNonEmptyString(x, fallback)
    out = string(fallback);
    if isempty(x)
        return;
    end

    x = string(x(:));
    mask = strlength(x) > 0;
    if any(mask)
        out = x(find(mask, 1, 'last'));
    end
end


