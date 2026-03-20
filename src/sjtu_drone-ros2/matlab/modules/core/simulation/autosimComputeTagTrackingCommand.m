function [cmdX, cmdY, pidX, pidY, tagLostSearchStartT] = autosimComputeTagTrackingCommand(cfg, tk, dtCtrl, xNow, yNow, predOk, uPred, vPred, tagDetected, uTag, vTag, pidX, pidY, tagLostSearchStartT)
    cmdX = 0.0;
    cmdY = 0.0;

    usePred = predOk && isfinite(uPred) && isfinite(vPred);
    useNow = tagDetected && isfinite(uTag) && isfinite(vTag);

    if usePred
        uCtrl = uPred;
        vCtrl = vPred;
    elseif useNow
        uCtrl = uTag;
        vCtrl = vTag;
    else
        uCtrl = nan;
        vCtrl = nan;
    end

    if isfinite(uCtrl) && isfinite(vCtrl)
        tagLostSearchStartT = nan;
        errU = cfg.control.target_u - uCtrl;
        errV = cfg.control.target_v - vCtrl;

        [ux, pidX] = autosimPidStep(errV, dtCtrl, pidX, cfg.control.xy_kp, cfg.control.xy_ki, cfg.control.xy_kd, cfg.control.xy_i_limit, cfg.control.xy_cmd_limit);
        [uy, pidY] = autosimPidStep(errU, dtCtrl, pidY, cfg.control.xy_kp, cfg.control.xy_ki, cfg.control.xy_kd, cfg.control.xy_i_limit, cfg.control.xy_cmd_limit);

        cmdX = cfg.control.xy_map_sign_x_from_v * ux;
        cmdY = cfg.control.xy_map_sign_y_from_u * uy;

        if sqrt(errU * errU + errV * errV) <= cfg.control.tag_center_deadband
            cmdX = 0.0;
            cmdY = 0.0;
        end
    else
        pidX = autosimPidInit();
        pidY = autosimPidInit();

        if cfg.control.pose_hold_enable && isfinite(xNow) && isfinite(yNow)
            errX = -xNow;
            errY = -yNow;
            cmdX = autosimClamp(cfg.control.pose_hold_kp * errX, -abs(cfg.control.pose_hold_cmd_limit), abs(cfg.control.pose_hold_cmd_limit));
            cmdY = autosimClamp(cfg.control.pose_hold_kp * errY, -abs(cfg.control.pose_hold_cmd_limit), abs(cfg.control.pose_hold_cmd_limit));
        elseif cfg.control.search_enable_spiral
            if ~isfinite(tagLostSearchStartT)
                tagLostSearchStartT = tk;
            end

            tSearch = max(0.0, tk - tagLostSearchStartT);
            rSearch = cfg.control.search_spiral_start_radius + cfg.control.search_spiral_growth_per_sec * tSearch;
            rSearch = min(rSearch, cfg.control.search_spiral_cmd_max);
            th = cfg.control.search_spiral_omega_rad_sec * tSearch;
            cmdX = autosimClamp(rSearch * cos(th), -abs(cfg.control.search_spiral_cmd_max), abs(cfg.control.search_spiral_cmd_max));
            cmdY = autosimClamp(rSearch * sin(th), -abs(cfg.control.search_spiral_cmd_max), abs(cfg.control.search_spiral_cmd_max));
        end
    end
end


