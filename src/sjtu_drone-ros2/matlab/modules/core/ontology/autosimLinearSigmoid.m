function y = autosimLinearSigmoid(x, w, b, fallback)
    xv = double(x(:));
    wv = double(w(:));
    if nargin < 4
        fallback = 0.5;
    end

    if isempty(xv) || isempty(wv) || numel(xv) ~= numel(wv)
        y = autosimClampNaN(fallback, 0.5);
        return;
    end

    if any(~isfinite(xv)) || any(~isfinite(wv)) || ~isfinite(b)
        y = autosimClampNaN(fallback, 0.5);
        return;
    end

    z = sum(xv .* wv) + double(b);
    y = 1.0 / (1.0 + exp(-z));
    y = autosimClamp(y, 0.0, 1.0);
end


