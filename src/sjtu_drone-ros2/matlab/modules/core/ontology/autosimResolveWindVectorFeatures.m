function [vx, vy, vmag] = autosimResolveWindVectorFeatures(v, fallbackMag)
    vv = double(v);
    if isempty(vv)
        vv = fallbackMag;
    end

    if isvector(vv)
        vv = vv(:);
        if numel(vv) >= 2
            vx = autosimClampNaN(vv(1), 0.0);
            vy = autosimClampNaN(vv(2), 0.0);
            vmag = hypot(vx, vy);
            return;
        end
        vmag = autosimClampNaN(vv(1), autosimClampNaN(fallbackMag, 0.0));
        vx = vmag;
        vy = 0.0;
        return;
    end

    if size(vv, 2) >= 2
        vx = autosimNanMean(vv(:,1));
        vy = autosimNanMean(vv(:,2));
        vmag = hypot(vx, vy);
        return;
    end

    vmag = autosimNanMean(vv(:,1));
    if ~isfinite(vmag)
        vmag = autosimClampNaN(fallbackMag, 0.0);
    end
    vx = vmag;
    vy = 0.0;
end


