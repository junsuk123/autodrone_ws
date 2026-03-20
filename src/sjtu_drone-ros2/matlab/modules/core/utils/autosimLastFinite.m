function out = autosimLastFinite(x, fallback)
    out = fallback;
    if isempty(x)
        return;
    end

    x = double(x(:));
    idx = find(isfinite(x), 1, 'last');
    if ~isempty(idx)
        out = x(idx);
    end
end


