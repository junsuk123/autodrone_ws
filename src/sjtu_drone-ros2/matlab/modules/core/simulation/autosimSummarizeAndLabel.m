function out = autosimSummarizeAndLabel(cfg, scenarioId, scenarioCfg, requireLandingOutcomeEvaluation, z, vz, speedAbs, rollDeg, pitchDeg, tagErr, windSpeed, stateVal, bumperContact, imuAngVel, imuLinAcc, contactForce, armFL, armFR, armRL, armRR, windDir)
    out = autosimEmptyScenarioResult();
    out.scenario_id = scenarioId;
    if isfield(scenarioCfg, 'policy_mode')
        out.scenario_policy = string(scenarioCfg.policy_mode);
    end
    if isfield(scenarioCfg, 'target_case')
        out.target_case = string(scenarioCfg.target_case);
    end
    out.hover_height_cmd = scenarioCfg.hover_height_m;
    out.wind_speed_cmd = scenarioCfg.wind_speed;
    out.wind_dir_cmd = scenarioCfg.wind_dir;

    out.mean_wind_speed = autosimNanMean(windSpeed);
    out.max_wind_speed = autosimNanMax(windSpeed);
    [windVelX, windVelY] = autosimWindVectorFromSpeedDir(windSpeed, windDir);
    out.wind_velocity_x = autosimNanMean(windVelX);
    out.wind_velocity_y = autosimNanMean(windVelY);
    out.wind_velocity = autosimNanMean(hypot(windVelX, windVelY));
    windAccX = autosimComputeWindAcceleration(windVelX, cfg.scenario.sample_period_sec);
    windAccY = autosimComputeWindAcceleration(windVelY, cfg.scenario.sample_period_sec);
    out.wind_acceleration_x = windAccX;
    out.wind_acceleration_y = windAccY;
    out.wind_acceleration = hypot(windAccX, windAccY);
    out.mean_abs_roll_deg = autosimNanMean(abs(rollDeg));
    out.mean_abs_pitch_deg = autosimNanMean(abs(pitchDeg));
    out.mean_abs_vz = autosimNanMean(abs(vz));
    out.max_abs_vz = autosimNanMax(abs(vz));
    out.mean_tag_error = autosimNanMean(tagErr);
    out.max_tag_error = autosimNanMax(tagErr);

    out.final_altitude = autosimNanLast(z);
    out.landing_height_m = out.final_altitude;
    out.final_abs_speed = autosimNanLast(speedAbs);
    out.final_abs_roll_deg = autosimNanLast(abs(rollDeg));
    out.final_abs_pitch_deg = autosimNanLast(abs(pitchDeg));
    out.final_tag_error = autosimNanLast(tagErr);

    [zStable, vzStable] = autosimSelectLandingStabilityWindow(z, vz, stateVal, cfg);

    out.stability_std_z = autosimNanStd(zStable);
    out.stability_std_vz = autosimNanStd(vzStable);

    vzTouch = autosimSelectTouchdownDynamicsWindow(vz, stateVal, cfg);
    [~, out.stability_std_vz_osc, out.touchdown_accel_rms] = autosimCalcVzMetrics(vzTouch, cfg.scenario.sample_period_sec);
    out.max_abs_vz = max(out.max_abs_vz, autosimNanMax(abs(vzTouch)));
    out.contact_count = sum(bumperContact > 0);
    out.mean_imu_ang_vel = autosimNanMean(imuAngVel);
    out.max_imu_ang_vel = autosimNanMax(imuAngVel);
    out.mean_imu_lin_acc = autosimNanMean(imuLinAcc);
    out.max_imu_lin_acc = autosimNanMax(imuLinAcc);
    out.max_contact_force = autosimNanMax(contactForce);
    out.arm_force_fl_mean = autosimNanMean(armFL);
    out.arm_force_fr_mean = autosimNanMean(armFR);
    out.arm_force_rl_mean = autosimNanMean(armRL);
    out.arm_force_rr_mean = autosimNanMean(armRR);
    out.arm_force_fl_peak = autosimNanMax(armFL);
    out.arm_force_fr_peak = autosimNanMax(armFR);
    out.arm_force_rl_peak = autosimNanMax(armRL);
    out.arm_force_rr_peak = autosimNanMax(armRR);
    out.arm_force_peak_max = autosimNanMax([out.arm_force_fl_peak, out.arm_force_fr_peak, out.arm_force_rl_peak, out.arm_force_rr_peak]);
    out.arm_force_imbalance = autosimNanMax([abs(armFL-armFR), abs(armRL-armRR)]);
    [impactWeightN, impactTotalLimitN, impactImbalanceLimitN, impactArmPeakLimitN] = autosimComputeImpactThresholds(cfg);
    out.impact_weight_n = impactWeightN;
    out.impact_total_force_limit_n = impactTotalLimitN;
    out.impact_arm_imbalance_limit_n = impactImbalanceLimitN;
    out.impact_arm_peak_limit_n = impactArmPeakLimitN;
    out.contact_metrics_available = isfinite(out.max_contact_force) && isfinite(out.arm_force_imbalance) && isfinite(out.arm_force_peak_max);
    out.final_state = autosimNanLast(stateVal);

    if isfield(cfg, 'scenario') && isfield(cfg.scenario, 'analysis_stop_at_landing') && cfg.scenario.analysis_stop_at_landing && ~requireLandingOutcomeEvaluation
        [passHoverWindow, hoverFailureReason] = autosimEvaluateHoverWindowSafety(out, cfg);
        if passHoverWindow
            out.label = "stable";
            out.success = true;
            out.failure_reason = "";
        else
            out.label = "unstable";
            out.success = false;
            out.failure_reason = hoverFailureReason;
        end
        return;
    end

    c = cfg.thresholds;
    condState = isfinite(out.final_state) && out.final_state == c.land_state_value;
    condAlt = isfinite(out.final_altitude) && out.final_altitude <= c.landed_altitude_max_m;
    condSpeed = isfinite(out.final_abs_speed) && out.final_abs_speed <= c.final_speed_max_mps;
    condRoll = isfinite(out.final_abs_roll_deg) && out.final_abs_roll_deg <= c.final_attitude_max_deg;
    condPitch = isfinite(out.final_abs_pitch_deg) && out.final_abs_pitch_deg <= c.final_attitude_max_deg;
    condTag = (~isfinite(out.final_tag_error)) || (out.final_tag_error <= c.final_tag_error_max);
    condStdZ = isfinite(out.stability_std_z) && out.stability_std_z <= c.final_stability_std_z_max;
    condStdVz = isfinite(out.stability_std_vz) && out.stability_std_vz <= c.final_stability_std_vz_max;
    condVzOsc = (~isfinite(out.stability_std_vz_osc)) || (out.stability_std_vz_osc <= c.final_touchdown_vz_osc_max);
    condTdAcc = (~isfinite(out.touchdown_accel_rms)) || (out.touchdown_accel_rms <= c.final_touchdown_accel_rms_max);
    condTdVz = isfinite(out.max_abs_vz) && (out.max_abs_vz <= c.final_touchdown_abs_vz_max);
    condImuAng = (~isfinite(out.max_imu_ang_vel)) || (out.max_imu_ang_vel <= c.final_imu_ang_vel_rms_max);
    condImuAcc = (~isfinite(out.max_imu_lin_acc)) || (out.max_imu_lin_acc <= c.final_imu_lin_acc_rms_max);
    if isfield(c, 'require_contact_metrics') && c.require_contact_metrics
        condContactMetricsAvailable = out.contact_metrics_available;
    else
        condContactMetricsAvailable = true;
    end
    condContactForce = (~isfinite(out.max_contact_force)) || (out.max_contact_force <= out.impact_total_force_limit_n);
    condArmBalance = (~isfinite(out.arm_force_imbalance)) || (out.arm_force_imbalance <= out.impact_arm_imbalance_limit_n);
    condArmPeak = (~isfinite(out.arm_force_peak_max)) || (out.arm_force_peak_max <= out.impact_arm_peak_limit_n);

    passAll = condState && condAlt && condSpeed && condRoll && condPitch && condTag && condStdZ && condStdVz && condVzOsc && condTdAcc && condTdVz && ...
        condImuAng && condImuAcc && condContactMetricsAvailable && condContactForce && condArmBalance && condArmPeak;

    if passAll
        out.label = "stable";
        out.success = true;
        out.failure_reason = "";
    else
        out.label = "unstable";
        out.success = false;
        out.failure_reason = autosimBuildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz, condVzOsc, condTdAcc, condTdVz, ...
            condImuAng, condImuAcc, condContactMetricsAvailable, condContactForce, condArmBalance, condArmPeak);
    end
end


