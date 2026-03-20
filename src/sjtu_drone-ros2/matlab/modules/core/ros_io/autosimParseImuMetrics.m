function [angVelNorm, linAccNorm] = autosimParseImuMetrics(msg)
    angVelNorm = nan;
    linAccNorm = nan;

    try
        if isprop(msg, 'angular_velocity')
            av = msg.angular_velocity;
        else
            av = msg.angularvelocity;
        end
        ax = double(av.x);
        ay = double(av.y);
        az = double(av.z);
        angVelNorm = sqrt(ax*ax + ay*ay + az*az);
    catch
    end

    try
        if isprop(msg, 'linear_acceleration')
            la = msg.linear_acceleration;
        else
            la = msg.linearacceleration;
        end
        lx = double(la.x);
        ly = double(la.y);
        lz = double(la.z);
        linAccNorm = sqrt(lx*lx + ly*ly + lz*lz);
    catch
    end
end


