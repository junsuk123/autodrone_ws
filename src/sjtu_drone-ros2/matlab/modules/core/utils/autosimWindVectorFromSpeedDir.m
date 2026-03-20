function [vx, vy] = autosimWindVectorFromSpeedDir(speed, dirDeg)
    s = double(speed);
    d = double(dirDeg);
    vx = s .* cosd(d);
    vy = s .* sind(d);
    vx(~isfinite(vx)) = 0.0;
    vy(~isfinite(vy)) = 0.0;
end


