function m = autosimCircularMeanDeg(thetaDeg)
    th = thetaDeg(isfinite(thetaDeg));
    if isempty(th)
        m = nan;
        return;
    end

    c = mean(cosd(th));
    s = mean(sind(th));
    m = autosimWrapTo180(rad2deg(atan2(s, c)));
end


