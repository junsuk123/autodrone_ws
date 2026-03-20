function v = autosimToNumeric(x)
    if isnumeric(x)
        v = double(x);
    elseif islogical(x)
        v = double(x);
    elseif isstring(x)
        v = double(str2double(x));
        v(~isfinite(v)) = 0.0;
    else
        v = zeros(numel(x),1);
    end

    if isrow(v)
        v = v.';
    end
    v(~isfinite(v)) = 0.0;
end


