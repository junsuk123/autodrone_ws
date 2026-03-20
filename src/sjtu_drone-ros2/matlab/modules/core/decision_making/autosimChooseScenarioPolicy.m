function policy = autosimChooseScenarioPolicy(cfg, datasetState, scenarioId)
    policy = struct();
    policy.mode = "exploit";
    policy.reason = "default";
    policy.p_exploit = autosimClampNaN(cfg.adaptive.base_exploit_prob, 0.60);
    policy.p_boundary = autosimClampNaN(cfg.adaptive.base_boundary_prob, 0.25);
    policy.p_hard_negative = autosimClampNaN(cfg.adaptive.base_hard_negative_prob, 0.15);
    
    if autosimPipelineValidationEnabled(cfg) && isfield(cfg, 'validation') && isfield(cfg.validation, 'enable') && cfg.validation.enable
        policy.mode = autosimValidationMode(cfg, scenarioId);
        policy.reason = "validation_schedule";
        policy.p_exploit = nan;
        policy.p_boundary = nan;
        policy.p_hard_negative = nan;
        return;
    end

    if ~isfield(cfg, 'adaptive') || ~cfg.adaptive.enable || scenarioId <= cfg.adaptive.warmup_scenarios
        return;
    end

    targetUnstable = autosimClampNaN(cfg.adaptive.target_unstable_ratio, 0.45);
    targetBoundary = autosimClampNaN(cfg.adaptive.target_boundary_ratio, 0.25);

    unstableRatio = 1.0 - autosimClampNaN(datasetState.stableRatio, 0.5);
    boundaryRatio = autosimClampNaN(datasetState.boundarySampleRatio, 0.0);
    unsafeRate = autosimClampNaN(datasetState.unsafeLandingRate, 0.0);
    fnRate = autosimClampNaN(datasetState.falseNegativeRate, 0.0);

    if unstableRatio < targetUnstable
        d = min(0.25, targetUnstable - unstableRatio);
        policy.p_hard_negative = min(cfg.adaptive.max_hard_negative_prob, policy.p_hard_negative + d);
        policy.p_exploit = max(0.25, policy.p_exploit - 0.5 * d);
        policy.reason = "unstable_shortage";
    end

    if boundaryRatio < targetBoundary
        d = min(0.25, targetBoundary - boundaryRatio);
        policy.p_boundary = min(cfg.adaptive.max_boundary_prob, policy.p_boundary + d);
        policy.p_exploit = max(0.20, policy.p_exploit - 0.5 * d);
        if policy.reason == "default"
            policy.reason = "boundary_shortage";
        else
            policy.reason = policy.reason + "+boundary_shortage";
        end
    end

    if unsafeRate > 0.20
        policy.p_boundary = min(cfg.adaptive.max_boundary_prob, policy.p_boundary + 0.12);
        policy.p_exploit = max(0.20, policy.p_exploit - 0.08);
        if policy.reason == "default"
            policy.reason = "unsafe_fp_high";
        else
            policy.reason = policy.reason + "+unsafe_fp_high";
        end
    end

    if fnRate > 0.25
        policy.p_boundary = min(cfg.adaptive.max_boundary_prob, policy.p_boundary + 0.08);
        if policy.reason == "default"
            policy.reason = "fn_high";
        else
            policy.reason = policy.reason + "+fn_high";
        end
    end

    p = [policy.p_exploit, policy.p_boundary, policy.p_hard_negative];
    p = max(0.0, p);
    s = sum(p);
    if s <= 0
        p = [0.60, 0.25, 0.15];
    else
        p = p ./ s;
    end

    policy.p_exploit = p(1);
    policy.p_boundary = p(2);
    policy.p_hard_negative = p(3);

    r = rand();
    if r < p(3)
        policy.mode = "hard_negative";
    elseif r < (p(3) + p(2))
        policy.mode = "boundary_validation";
    else
        policy.mode = "exploit";
    end
end


