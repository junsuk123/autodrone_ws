function [pidX, pidY, tagLostSearchStartT, lastTagU, lastTagV, lastTagDetectT, haveLastTag, lastTagRxT, tagRxCount] = autosimRunFastTagControlBurst(cfg, rosCtx, pubCmd, msgCmd, t0, burstStartT, burstDurationSec, xNow, yNow, pidX, pidY, tagLostSearchStartT, lastTagU, lastTagV, lastTagDetectT, haveLastTag, lastTagRxT, tagRxCount, randomLandingPlanned, randomLandingStartT, randomLandingEndT, randomBiasX, randomBiasY)
    if ~(isfield(cfg.control, 'tag_fast_loop_enable') && cfg.control.tag_fast_loop_enable)
        return;
    end
    if burstDurationSec <= 0
        return;
    end

    fastDt = 0.02;
    if isfield(cfg.control, 'tag_fast_loop_period_sec') && isfinite(cfg.control.tag_fast_loop_period_sec)
        fastDt = max(0.005, cfg.control.tag_fast_loop_period_sec);
    end

    recvTimeoutSec = 0.0;
    if isfield(cfg, 'ros') && isfield(cfg.ros, 'receive_timeout_sec') && isfinite(cfg.ros.receive_timeout_sec)
        recvTimeoutSec = max(0.0, min(cfg.ros.receive_timeout_sec, 0.002));
    end

    tagCallbackEnabled = isfield(rosCtx, 'tag_callback_enabled') && rosCtx.tag_callback_enabled;
    tagCacheKey = "";
    if isfield(rosCtx, 'tag_cache_key')
        tagCacheKey = string(rosCtx.tag_cache_key);
    end

    burstEndT = burstStartT + burstDurationSec;
    nextTickT = burstStartT + fastDt;
    while toc(t0) < burstEndT
        tkFast = toc(t0);
        if tkFast < nextTickT
            pause(max(0.0, min(nextTickT - tkFast, fastDt * 0.5)));
            continue;
        end

        [hasFreshTag, tagDetected, uTag, vTag, ~, tagRxCountNow] = autosimReadTagInput(rosCtx.subTag, recvTimeoutSec, tagCallbackEnabled, tagCacheKey, tagRxCount);
        tagRxCount = tagRxCountNow;
        if hasFreshTag
            lastTagRxT = tkFast;
        end

        if hasFreshTag && tagDetected && isfinite(uTag) && isfinite(vTag)
            lastTagU = uTag;
            lastTagV = vTag;
            lastTagDetectT = tkFast;
            haveLastTag = true;
        elseif haveLastTag && ((tkFast - lastTagDetectT) <= cfg.control.tag_hold_timeout_sec)
            tagDetected = true;
            uTag = lastTagU;
            vTag = lastTagV;
        else
            tagDetected = false;
            uTag = nan;
            vTag = nan;
        end

        [cmdX, cmdY, pidX, pidY, tagLostSearchStartT] = autosimComputeTagTrackingCommand( ...
            cfg, tkFast, fastDt, xNow, yNow, false, nan, nan, tagDetected, uTag, vTag, pidX, pidY, tagLostSearchStartT);

        if randomLandingPlanned && tkFast >= randomLandingStartT && tkFast < randomLandingEndT
            cmdX = autosimClamp(cmdX + randomBiasX, -abs(cfg.control.xy_cmd_limit), abs(cfg.control.xy_cmd_limit));
            cmdY = autosimClamp(cmdY + randomBiasY, -abs(cfg.control.xy_cmd_limit), abs(cfg.control.xy_cmd_limit));
        end

        msgCmd.linear.x = cmdX;
        msgCmd.linear.y = cmdY;
        msgCmd.linear.z = 0.0;
        msgCmd.angular.x = 0.0;
        msgCmd.angular.y = 0.0;
        msgCmd.angular.z = 0.0;
        send(pubCmd, msgCmd);

        nextTickT = nextTickT + fastDt;
    end
end


