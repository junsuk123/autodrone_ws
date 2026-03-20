function forceScalar = autosimParseForceScalarFromCustomWrench(obj)
    forceScalar = 0.0;
    [okMag, magVal] = autosimTryGetMessageField(obj, ["magnitude", "force", "norm", "value", "total_force"]);
    if okMag && isscalar(magVal)
        v = double(magVal);
        if isfinite(v)
            forceScalar = abs(v);
            return;
        end
    end

    [okFx, fxVal] = autosimTryGetMessageField(obj, ["fx", "force_x", "x"]);
    [okFy, fyVal] = autosimTryGetMessageField(obj, ["fy", "force_y", "y"]);
    [okFz, fzVal] = autosimTryGetMessageField(obj, ["fz", "force_z", "z"]);
    if okFx && okFy && okFz
        fx = double(fxVal);
        fy = double(fyVal);
        fz = double(fzVal);
        if isfinite(fx) && isfinite(fy) && isfinite(fz)
            forceScalar = sqrt(fx*fx + fy*fy + fz*fz);
        end
    end
end


