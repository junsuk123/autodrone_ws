function [passAll, reason] = autosimEvaluateHoverWindowSafety(out, cfg)
    condAlt = isfinite(out.final_altitude) && (out.final_altitude >= cfg.agent.min_altitude_before_land);
    condSpeed = isfinite(out.final_abs_speed) && (out.final_abs_speed <= cfg.agent.no_model_max_xy_speed_rms);
    condRoll = isfinite(out.final_abs_roll_deg) && (out.final_abs_roll_deg <= cfg.agent.no_model_max_abs_roll_pitch_deg);
    condPitch = isfinite(out.final_abs_pitch_deg) && (out.final_abs_pitch_deg <= cfg.agent.no_model_max_abs_roll_pitch_deg);
    condTag = (~isfinite(out.final_tag_error)) || (out.final_tag_error <= cfg.agent.max_tag_error_before_land);
    condStdZ = isfinite(out.stability_std_z) && (out.stability_std_z <= cfg.agent.no_model_max_z_osc_std);
    condStdVz = isfinite(out.stability_std_vz) && (out.stability_std_vz <= cfg.agent.no_model_max_abs_vz);
    condWind = (~isfinite(out.max_wind_speed)) || (out.max_wind_speed <= cfg.agent.no_model_max_wind_speed);
    condImuAng = (~isfinite(out.max_imu_ang_vel)) || (out.max_imu_ang_vel <= cfg.thresholds.final_imu_ang_vel_rms_max);
    condImuAcc = (~isfinite(out.max_imu_lin_acc)) || (out.max_imu_lin_acc <= cfg.thresholds.final_imu_lin_acc_rms_max);
    if isfield(cfg.thresholds, 'require_contact_metrics') && cfg.thresholds.require_contact_metrics
        condContactMetricsAvailable = out.contact_metrics_available;
    else
        condContactMetricsAvailable = true;
    end
    condContactForce = (~isfinite(out.max_contact_force)) || (out.max_contact_force <= out.impact_total_force_limit_n);
    condArmBalance = (~isfinite(out.arm_force_imbalance)) || (out.arm_force_imbalance <= out.impact_arm_imbalance_limit_n);
    condArmPeak = (~isfinite(out.arm_force_peak_max)) || (out.arm_force_peak_max <= out.impact_arm_peak_limit_n);

    passAll = condAlt && condSpeed && condRoll && condPitch && condTag && condStdZ && condStdVz && condWind && condImuAng && condImuAcc && ...
        condContactMetricsAvailable && condContactForce && condArmBalance && condArmPeak;

    parts = strings(0,1);
    if ~condAlt, parts(end+1,1) = "hover_altitude_low"; end %#ok<AGROW>
    if ~condSpeed, parts(end+1,1) = "hover_speed_high"; end %#ok<AGROW>
    if ~condRoll, parts(end+1,1) = "roll_high"; end %#ok<AGROW>
    if ~condPitch, parts(end+1,1) = "pitch_high"; end %#ok<AGROW>
    if ~condTag, parts(end+1,1) = "tag_error_high"; end %#ok<AGROW>
    if ~condStdZ, parts(end+1,1) = "z_unstable"; end %#ok<AGROW>
    if ~condStdVz, parts(end+1,1) = "vz_unstable"; end %#ok<AGROW>
    if ~condWind, parts(end+1,1) = "wind_high"; end %#ok<AGROW>
    if ~condImuAng, parts(end+1,1) = "imu_angular_rate_high"; end %#ok<AGROW>
    if ~condImuAcc, parts(end+1,1) = "imu_linear_accel_high"; end %#ok<AGROW>
    if ~condContactMetricsAvailable, parts(end+1,1) = "contact_metrics_unavailable"; end %#ok<AGROW>
    if ~condContactForce, parts(end+1,1) = "contact_force_high"; end %#ok<AGROW>
    if ~condArmBalance, parts(end+1,1) = "arm_force_imbalance_high"; end %#ok<AGROW>
    if ~condArmPeak, parts(end+1,1) = "arm_peak_impact_high"; end %#ok<AGROW>

    if isempty(parts)
        reason = "";
    else
        reason = strjoin(parts, ",");
    end
end


