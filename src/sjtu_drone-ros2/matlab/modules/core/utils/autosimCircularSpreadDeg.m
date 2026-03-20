function spreadDeg = autosimCircularSpreadDeg(thetaDeg)
    th = double(thetaDeg(:));
    th = th(isfinite(th));
    if numel(th) < 2
        spreadDeg = 0.0;
        return;
    end

    c = mean(cosd(th));
    s = mean(sind(th));
    R = sqrt(c.^2 + s.^2);
    R = autosimClamp(R, 1e-6, 1.0);
    spreadDeg = rad2deg(sqrt(max(0.0, -2.0 * log(R))));
end


