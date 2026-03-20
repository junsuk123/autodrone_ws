function vec = autosimToNumericVector(x)
    vec = [];
    try
        vec = double(x(:));
        vec = vec(isfinite(vec));
    catch
        vec = [];
    end
end


