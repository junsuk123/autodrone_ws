function mag = autosimWindVectorMag(v)
    vv = double(v);
    if isempty(vv)
        mag = 0.0;
        return;
    end
    if isvector(vv)
        vv = vv(:);
        if numel(vv) >= 2
            mag = hypot(vv(1), vv(2));
        else
            mag = abs(vv(1));
        end
    else
        if size(vv, 2) >= 2
            mag = hypot(vv(:,1), vv(:,2));
        else
            mag = abs(vv(:,1));
        end
    end
    mag(~isfinite(mag)) = 0.0;
end


