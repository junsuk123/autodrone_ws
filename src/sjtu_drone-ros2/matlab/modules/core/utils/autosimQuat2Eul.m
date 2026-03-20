function [r,p,y] = autosimQuat2Eul(qwxyz)
    w = qwxyz(1);
    x = qwxyz(2);
    yy = qwxyz(3);
    z = qwxyz(4);

    sinr = 2 * (w*x + yy*z);
    cosr = 1 - 2 * (x*x + yy*yy);
    r = atan2(sinr, cosr);

    sinp = 2 * (w*yy - z*x);
    if abs(sinp) >= 1
        p = sign(sinp) * pi/2;
    else
        p = asin(sinp);
    end

    siny = 2 * (w*z + x*yy);
    cosy = 1 - 2 * (yy*yy + z*z);
    y = atan2(siny, cosy);
end


