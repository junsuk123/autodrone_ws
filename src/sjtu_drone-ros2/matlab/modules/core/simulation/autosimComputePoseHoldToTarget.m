function [cmdX, cmdY] = autosimComputePoseHoldToTarget(cfg, xNow, yNow, xRef, yRef)
    cmdX = 0.0;
    cmdY = 0.0;
    if ~isfinite(xNow) || ~isfinite(yNow) || ~isfinite(xRef) || ~isfinite(yRef)
        return;
    end

    kp = autosimClampNaN(cfg.control.pose_hold_kp, 0.45);
    lim = abs(autosimClampNaN(cfg.control.pose_hold_cmd_limit, 0.35));
    errX = xRef - xNow;
    errY = yRef - yNow;

    cmdX = autosimClamp(kp * errX, -lim, lim);
    cmdY = autosimClamp(kp * errY, -lim, lim);
end


