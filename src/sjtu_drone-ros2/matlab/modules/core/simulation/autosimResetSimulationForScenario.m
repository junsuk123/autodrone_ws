function ok = autosimResetSimulationForScenario(cfg, rosCtx, scenarioId, scenarioCfg)
    ok = false;
    if nargin < 2 || isempty(rosCtx) || ~isstruct(rosCtx)
        return;
    end
    hasTopicResetPub = isfield(rosCtx, 'pubReset') && ~isempty(rosCtx.pubReset) && isfield(rosCtx, 'msgReset') && ~isempty(rosCtx.msgReset);
    
    hoverHeightForReset = nan;
    if nargin >= 4 && isstruct(scenarioCfg) && isfield(scenarioCfg, 'hover_height_m') && isfinite(scenarioCfg.hover_height_m)
        hoverHeightForReset = scenarioCfg.hover_height_m;
    end

    % Stop residual command to avoid carry-over motion after reset.
    if isfield(rosCtx, 'pubCmd') && ~isempty(rosCtx.pubCmd) && isfield(rosCtx, 'msgCmd') && ~isempty(rosCtx.msgCmd)
        try
            rosCtx.msgCmd.linear.x = 0.0;
            rosCtx.msgCmd.linear.y = 0.0;
            rosCtx.msgCmd.linear.z = 0.0;
            rosCtx.msgCmd.angular.x = 0.0;
            rosCtx.msgCmd.angular.y = 0.0;
            rosCtx.msgCmd.angular.z = 0.0;
            send(rosCtx.pubCmd, rosCtx.msgCmd);
        catch
        end
    end

    nPub = 3;
    dtPub = 0.12;
    settleSec = 2.0;
    landedTimeoutSec = 4.0;
    forceLandBeforeReset = true;
    nLandPub = 3;
    dtLandPub = 0.20;
    forceLandTimeoutSec = 8.0;
    takeoffAfterReset = true;
    nTakeoffPub = 2;
    dtTakeoffPub = 0.20;
    flyingTimeoutSec = 8.0;
    takeoffSettleSec = 1.0;
    softResetEnable = true;
    softResetFallbackToTopic = true;
    landStateValue = 0;
    flyingStateValue = 1;
    if isfield(cfg, 'process')
        if isfield(cfg.process, 'force_land_before_reset')
            forceLandBeforeReset = logical(cfg.process.force_land_before_reset);
        end
        if isfield(cfg.process, 'force_land_publish_count') && isfinite(cfg.process.force_land_publish_count)
            nLandPub = max(1, round(cfg.process.force_land_publish_count));
        end
        if isfield(cfg.process, 'force_land_publish_interval_sec') && isfinite(cfg.process.force_land_publish_interval_sec)
            dtLandPub = max(0.01, cfg.process.force_land_publish_interval_sec);
        end
        if isfield(cfg.process, 'force_land_wait_timeout_sec') && isfinite(cfg.process.force_land_wait_timeout_sec)
            forceLandTimeoutSec = max(0.0, cfg.process.force_land_wait_timeout_sec);
        end
        if isfield(cfg.process, 'reset_publish_count') && isfinite(cfg.process.reset_publish_count)
            nPub = max(1, round(cfg.process.reset_publish_count));
        end
        if isfield(cfg.process, 'reset_publish_interval_sec') && isfinite(cfg.process.reset_publish_interval_sec)
            dtPub = max(0.01, cfg.process.reset_publish_interval_sec);
        end
        if isfield(cfg.process, 'reset_settle_sec') && isfinite(cfg.process.reset_settle_sec)
            settleSec = max(0.0, cfg.process.reset_settle_sec);
        end
        if isfield(cfg.process, 'reset_wait_landed_timeout_sec') && isfinite(cfg.process.reset_wait_landed_timeout_sec)
            landedTimeoutSec = max(0.0, cfg.process.reset_wait_landed_timeout_sec);
        end
        if isfield(cfg.process, 'takeoff_after_reset')
            takeoffAfterReset = logical(cfg.process.takeoff_after_reset);
        end
        if isfield(cfg.process, 'takeoff_publish_count') && isfinite(cfg.process.takeoff_publish_count)
            nTakeoffPub = max(1, round(cfg.process.takeoff_publish_count));
        end
        if isfield(cfg.process, 'takeoff_publish_interval_sec') && isfinite(cfg.process.takeoff_publish_interval_sec)
            dtTakeoffPub = max(0.01, cfg.process.takeoff_publish_interval_sec);
        end
        if isfield(cfg.process, 'takeoff_wait_flying_timeout_sec') && isfinite(cfg.process.takeoff_wait_flying_timeout_sec)
            flyingTimeoutSec = max(0.0, cfg.process.takeoff_wait_flying_timeout_sec);
        end
        if isfield(cfg.process, 'takeoff_settle_sec') && isfinite(cfg.process.takeoff_settle_sec)
            takeoffSettleSec = max(0.0, cfg.process.takeoff_settle_sec);
        end
        if isfield(cfg.process, 'soft_reset_enable')
            softResetEnable = logical(cfg.process.soft_reset_enable);
        end
        if isfield(cfg.process, 'soft_reset_fallback_to_topic')
            softResetFallbackToTopic = logical(cfg.process.soft_reset_fallback_to_topic);
        end
    end
    if isfield(cfg, 'thresholds') && isfield(cfg.thresholds, 'land_state_value') && isfinite(cfg.thresholds.land_state_value)
        landStateValue = round(cfg.thresholds.land_state_value);
    end
    if landStateValue == 0
        flyingStateValue = 1;
    end

    if forceLandBeforeReset
        if isfield(rosCtx, 'pubLand') && ~isempty(rosCtx.pubLand) && isfield(rosCtx, 'msgLand') && ~isempty(rosCtx.msgLand)
            try
                for i = 1:nLandPub
                    send(rosCtx.pubLand, rosCtx.msgLand);
                    pause(dtLandPub);
                end
            catch ME
                warning('[AUTOSIM] Pre-reset land publish failed for scenario %d: %s', scenarioId, ME.message);
            end
        end

        if forceLandTimeoutSec > 0
            landedNow = autosimWaitForStateValue(rosCtx, landStateValue, forceLandTimeoutSec);
            if ~landedNow
                warning('[AUTOSIM] Scenario %d did not confirm landed before reset; continuing with reset.', scenarioId);
            else
                fprintf('[AUTOSIM] Scenario %d pre-reset landing confirmed.\n', scenarioId);
            end
        end
    end

    softResetOK = false;
    if softResetEnable
        softResetOK = autosimSoftReset(cfg, scenarioId);
    end

    doTopicReset = (~softResetOK) || softResetFallbackToTopic;
    if doTopicReset
        if ~hasTopicResetPub
            warning('[AUTOSIM] Reset publisher unavailable and soft reset failed/disabled for scenario %d.', scenarioId);
            return;
        end
        try
            for i = 1:nPub
                send(rosCtx.pubReset, rosCtx.msgReset);
                pause(dtPub);
            end
        catch ME
            warning('[AUTOSIM] Reset publish failed for scenario %d: %s', scenarioId, ME.message);
            return;
        end
    end

    if settleSec > 0
        pause(settleSec);
    end

    if landedTimeoutSec > 0
        landedAfterReset = autosimWaitForStateValue(rosCtx, landStateValue, landedTimeoutSec);
        if landedAfterReset
            fprintf('[AUTOSIM] Scenario %d reset complete (state=landed).\n', scenarioId);
        else
            warning('[AUTOSIM] Scenario %d reset landed-state confirmation timed out; continuing.', scenarioId);
        end
    end

    if isfinite(hoverHeightForReset)
        autosimSetDronePositionToOrigin(rosCtx, hoverHeightForReset, scenarioId);
    end

    if takeoffAfterReset
        if ~(isfield(rosCtx, 'pubTakeoff') && ~isempty(rosCtx.pubTakeoff) && isfield(rosCtx, 'msgTakeoff') && ~isempty(rosCtx.msgTakeoff))
            warning('[AUTOSIM] Takeoff publisher unavailable after reset for scenario %d.', scenarioId);
            return;
        end

        try
            for i = 1:nTakeoffPub
                send(rosCtx.pubTakeoff, rosCtx.msgTakeoff);
                pause(dtTakeoffPub);
            end
        catch ME
            warning('[AUTOSIM] Post-reset takeoff publish failed for scenario %d: %s', scenarioId, ME.message);
            return;
        end

        if flyingTimeoutSec > 0
            flyingNow = autosimWaitForStateValue(rosCtx, flyingStateValue, flyingTimeoutSec);
            if ~flyingNow
                warning('[AUTOSIM] Scenario %d failed to reach flying state after reset/takeoff.', scenarioId);
                return;
            end
        end

        if takeoffSettleSec > 0
            pause(takeoffSettleSec);
        end
        fprintf('[AUTOSIM] Scenario %d reset + takeoff complete.\n', scenarioId);
    end

    ok = true;
    if ~takeoffAfterReset
        fprintf('[AUTOSIM] Scenario %d reset command sent; proceeding after settle delay.\n', scenarioId);
    end
end


