function [weightN, totalLimitN, imbalanceLimitN, armPeakLimitN] = autosimComputeImpactThresholds(cfg)
    massKg = 1.4;
    grav = 9.81;
    if isfield(cfg, 'wind') && isfield(cfg.wind, 'physics')
        if isfield(cfg.wind.physics, 'mass_kg') && isfinite(cfg.wind.physics.mass_kg) && (cfg.wind.physics.mass_kg > 0)
            massKg = cfg.wind.physics.mass_kg;
        end
        if isfield(cfg.wind.physics, 'gravity_mps2') && isfinite(cfg.wind.physics.gravity_mps2) && (cfg.wind.physics.gravity_mps2 > 0)
            grav = cfg.wind.physics.gravity_mps2;
        end
    end

    weightN = massKg * grav;
    totalLimitN = cfg.thresholds.final_contact_force_max_n;
    imbalanceLimitN = cfg.thresholds.final_arm_force_imbalance_max_n;
    armPeakLimitN = cfg.thresholds.final_arm_force_peak_max_n;

    useMassScaled = isfield(cfg.thresholds, 'use_mass_scaled_impact_thresholds') && cfg.thresholds.use_mass_scaled_impact_thresholds;
    if useMassScaled
        totalFactor = autosimClampNaN(cfg.thresholds.mass_scaled_contact_force_factor, 3.8);
        imbalanceFactor = autosimClampNaN(cfg.thresholds.mass_scaled_arm_imbalance_factor, 1.4);
        armPeakFactor = autosimClampNaN(cfg.thresholds.mass_scaled_arm_peak_factor, 1.5);

        totalLimitN = totalFactor * weightN;
        imbalanceLimitN = imbalanceFactor * weightN;
        armPeakLimitN = armPeakFactor * weightN;
    end
end


