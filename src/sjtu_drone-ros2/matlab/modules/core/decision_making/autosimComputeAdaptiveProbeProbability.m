function [probeProb, reason] = autosimComputeAdaptiveProbeProbability(cfg, datasetState, policyMode)
    policyMode = string(policyMode);
    probeProb = 0.0;
    reasonParts = strings(0, 1);
    bootstrapActive = false;

    if ~isfield(cfg, 'probe') || ~isfield(cfg.probe, 'enable') || ~cfg.probe.enable
        reason = "probe_disabled";
        return;
    end

    switch policyMode
        case "hard_negative"
            probeProb = autosimClampNaN(cfg.probe.hard_negative_prob, 0.0);
            maxProb = autosimClampNaN(cfg.probe.max_hard_negative_prob, 1.0);
            targetPolicyRatio = autosimClampNaN(cfg.probe.target_recent_hard_negative_ratio, 0.75);
            recentPolicyRatio = autosimClampNaN(datasetState.recentHardNegativeProbeRatio, 0.0);
            bootstrapMinProb = autosimClampNaN(cfg.probe.bootstrap_hard_negative_prob, probeProb);
        case "boundary_validation"
            probeProb = autosimClampNaN(cfg.probe.boundary_prob, 0.0);
            maxProb = autosimClampNaN(cfg.probe.max_boundary_prob, 1.0);
            targetPolicyRatio = autosimClampNaN(cfg.probe.target_recent_boundary_ratio, 0.40);
            recentPolicyRatio = autosimClampNaN(datasetState.recentBoundaryProbeRatio, 0.0);
            bootstrapMinProb = autosimClampNaN(cfg.probe.bootstrap_boundary_prob, probeProb);
        otherwise
            probeProb = autosimClampNaN(cfg.probe.exploit_prob, 0.0);
            maxProb = autosimClampNaN(cfg.probe.max_exploit_prob, 1.0);
            targetPolicyRatio = autosimClampNaN(cfg.probe.target_recent_exploit_ratio, 0.05);
            recentPolicyRatio = autosimClampNaN(datasetState.recentExploitProbeRatio, 0.0);
            bootstrapMinProb = autosimClampNaN(cfg.probe.bootstrap_exploit_prob, probeProb);
    end

    probeProb = autosimClamp(probeProb, 0.0, 1.0);
    maxProb = autosimClamp(maxProb, probeProb, 1.0);
    reasonParts(end+1,1) = policyMode + "_base";

    totalShortage = max(0.0, autosimClampNaN(cfg.probe.target_recent_total_ratio, 0.18) - autosimClampNaN(datasetState.recentProbeRatio, 0.0));
    if totalShortage > 0
        probeProb = probeProb + autosimClampNaN(cfg.probe.total_shortage_gain, 0.0) * totalShortage;
        reasonParts(end+1,1) = "total_shortage";
    end

    policyShortage = max(0.0, targetPolicyRatio - recentPolicyRatio);
    if policyShortage > 0
        probeProb = probeProb + autosimClampNaN(cfg.probe.policy_shortage_gain, 0.0) * policyShortage;
        reasonParts(end+1,1) = "policy_shortage";
    end

    if policyMode ~= "exploit"
        unsafeShortage = max(0.0, autosimClampNaN(cfg.probe.target_recent_unsafe_probe_ratio, 0.10) - autosimClampNaN(datasetState.recentUnsafeProbeRatio, 0.0));
        if unsafeShortage > 0
            probeProb = probeProb + autosimClampNaN(cfg.probe.unsafe_shortage_gain, 0.0) * unsafeShortage;
            reasonParts(end+1,1) = "unsafe_probe_shortage";
        end
    end

    if autosimClampNaN(datasetState.unsafeLandingRate, 0.0) > 0.20 && policyMode ~= "exploit"
        probeProb = probeProb + autosimClampNaN(cfg.probe.high_fp_boost, 0.0);
        reasonParts(end+1,1) = "fp_high";
    end

    if autosimClampNaN(datasetState.nScenarios, 0.0) < autosimClampNaN(cfg.probe.bootstrap_episode_count, 10)
        bootstrapActive = true;
        probeProb = max(probeProb, autosimClamp(bootstrapMinProb, 0.0, maxProb));
        bootstrapShortage = max(0.0, autosimClampNaN(cfg.probe.bootstrap_target_total_ratio, 0.40) - autosimClampNaN(datasetState.recentProbeRatio, 0.0));
        if bootstrapShortage > 0
            probeProb = probeProb + autosimClampNaN(cfg.probe.bootstrap_gain, 0.0) * bootstrapShortage;
            reasonParts(end+1,1) = "bootstrap_shortage";
        end
    end

    probeProb = autosimClamp(probeProb, 0.0, maxProb);
    if bootstrapActive
        reasonParts(end+1,1) = "bootstrap_active";
    end
    reason = strjoin(cellstr(reasonParts), "+");
end


