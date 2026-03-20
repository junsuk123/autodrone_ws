function reason = autosimBuildFailureReason(condState, condAlt, condSpeed, condRoll, condPitch, condTag, condStdZ, condStdVz, condVzOsc, condTdAcc, condTdVz, condImuAng, condImuAcc, condContactMetricsAvailable, condContactForce, condArmBalance, condArmPeak)
    parts = strings(0,1);
    if ~condState, parts(end+1,1) = "state_not_landed"; end %#ok<AGROW>
    if ~condAlt, parts(end+1,1) = "altitude_high"; end %#ok<AGROW>
    if ~condSpeed, parts(end+1,1) = "speed_high"; end %#ok<AGROW>
    if ~condRoll, parts(end+1,1) = "roll_high"; end %#ok<AGROW>
    if ~condPitch, parts(end+1,1) = "pitch_high"; end %#ok<AGROW>
    if ~condTag, parts(end+1,1) = "tag_error_high"; end %#ok<AGROW>
    if ~condStdZ, parts(end+1,1) = "z_unstable"; end %#ok<AGROW>
    if ~condStdVz, parts(end+1,1) = "vz_unstable"; end %#ok<AGROW>
    if ~condVzOsc, parts(end+1,1) = "touchdown_vz_osc_high"; end %#ok<AGROW>
    if ~condTdAcc, parts(end+1,1) = "touchdown_accel_high"; end %#ok<AGROW>
    if ~condTdVz, parts(end+1,1) = "touchdown_vz_peak_high"; end %#ok<AGROW>
    if ~condImuAng, parts(end+1,1) = "imu_angular_rate_high"; end %#ok<AGROW>
    if ~condImuAcc, parts(end+1,1) = "imu_linear_accel_high"; end %#ok<AGROW>
    if ~condContactMetricsAvailable, parts(end+1,1) = "contact_metrics_unavailable"; end %#ok<AGROW>
    if ~condContactForce, parts(end+1,1) = "contact_force_high"; end %#ok<AGROW>
    if ~condArmBalance, parts(end+1,1) = "arm_force_imbalance"; end %#ok<AGROW>
    if ~condArmPeak, parts(end+1,1) = "arm_peak_impact_high"; end %#ok<AGROW>

    if isempty(parts)
        reason = "unknown";
    else
        reason = strjoin(parts, ';');
    end
end


