function v = autosimNanLast(x)
    x = double(x(:));
    idx = find(isfinite(x), 1, 'last');
    if isempty(idx)
        v = nan;
    else
        v = x(idx);
    end
end


