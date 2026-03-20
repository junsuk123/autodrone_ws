function scenarioCfg = autosimBuildAdaptiveScenarioConfig(cfg, scenarioId, policyMode, datasetState)
    scenarioCfg = autosimBuildScenarioConfig(cfg, scenarioId);
    probeProb = 0.0;
    probeReason = "none";

    if isstruct(policyMode)
        policy = policyMode;
    else
        policy = struct('mode', string(policyMode), 'reason', "external", ...
            'p_exploit', nan, 'p_boundary', nan, 'p_hard_negative', nan);
    end

    scenarioCfg.policy_mode = string(policy.mode);
    scenarioCfg.policy_reason = string(policy.reason);
    scenarioCfg.policy_weight_exploit = autosimClampNaN(policy.p_exploit, nan);
    scenarioCfg.policy_weight_boundary = autosimClampNaN(policy.p_boundary, nan);
    scenarioCfg.policy_weight_hard_negative = autosimClampNaN(policy.p_hard_negative, nan);

    if ~isfield(cfg, 'adaptive') || ~cfg.adaptive.enable
        return;
    end

    switch char(policy.mode)
        case 'hard_negative'
            scenarioCfg.hard_negative_hint = true;
            scenarioCfg.boundary_hint = false;

            wsMin = cfg.adaptive.hard_negative.wind_min_scale * cfg.wind.speed_max;
            wsMax = cfg.adaptive.hard_negative.wind_scale * cfg.wind.speed_max;
            scenarioCfg.wind_speed = autosimClamp(autosimRandRange(wsMin, wsMax), cfg.wind.speed_min, cfg.wind.speed_max);
            scenarioCfg.hover_height_m = autosimClamp(scenarioCfg.hover_height_m + cfg.adaptive.hard_negative.hover_bias_m, ...
                cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
            scenarioCfg.gust_amp_scale = cfg.adaptive.hard_negative.gust_amp_scale;
            scenarioCfg.dir_osc_scale = cfg.adaptive.hard_negative.dir_osc_scale;

        case 'boundary_validation'
            scenarioCfg.hard_negative_hint = false;
            scenarioCfg.boundary_hint = true;

            c = cfg.adaptive.boundary.wind_center_scale * cfg.wind.speed_max;
            span = cfg.adaptive.boundary.wind_span_scale * cfg.wind.speed_max;
            scenarioCfg.wind_speed = autosimClamp(autosimRandRange(c - span, c + span), cfg.wind.speed_min, cfg.wind.speed_max);
            scenarioCfg.hover_height_m = autosimClamp(scenarioCfg.hover_height_m + cfg.adaptive.boundary.hover_bias_m, ...
                cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
            scenarioCfg.gust_amp_scale = cfg.adaptive.boundary.gust_amp_scale;
            scenarioCfg.dir_osc_scale = cfg.adaptive.boundary.dir_osc_scale;

            if autosimClampNaN(datasetState.falseNegativeRate, 0.0) > 0.25
                scenarioCfg.safe_probe_ratio_boost = cfg.adaptive.safe_probe_ratio_boost;
            end

        otherwise
            scenarioCfg.hard_negative_hint = false;
            scenarioCfg.boundary_hint = false;
            scenarioCfg.wind_speed = autosimClamp(cfg.adaptive.exploit.wind_scale * scenarioCfg.wind_speed, cfg.wind.speed_min, cfg.wind.speed_max);
            scenarioCfg.hover_height_m = autosimClamp(scenarioCfg.hover_height_m + cfg.adaptive.exploit.hover_bias_m, ...
                cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
            scenarioCfg.gust_amp_scale = cfg.adaptive.exploit.gust_amp_scale;
            scenarioCfg.dir_osc_scale = cfg.adaptive.exploit.dir_osc_scale;
    end

    if isfield(cfg, 'probe') && isfield(cfg.probe, 'enable') && cfg.probe.enable
        [probeProb, probeReason] = autosimComputeAdaptiveProbeProbability(cfg, datasetState, string(policy.mode));
        probeProb = autosimClamp(probeProb, 0.0, 1.0);
        scenarioCfg.probe_landing_probability = probeProb;
        if rand() < probeProb
            scenarioCfg.probe_landing_selected = true;
            scenarioCfg.probe_landing_reason = probeReason;
        else
            scenarioCfg.probe_landing_reason = probeReason;
        end
    end

    scenarioCfg = autosimApplyCurriculumTargetCase(cfg, scenarioCfg, datasetState, scenarioId);
end


