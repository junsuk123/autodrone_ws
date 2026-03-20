function scenarioCfg = autosimBuildScenarioConfig(cfg, scenarioId)
    scenarioCfg = struct();
    scenarioCfg.id = scenarioId;
    scenarioCfg.policy_mode = "exploit";
    scenarioCfg.policy_reason = "base";
    scenarioCfg.policy_weight_exploit = nan;
    scenarioCfg.policy_weight_boundary = nan;
    scenarioCfg.policy_weight_hard_negative = nan;
    scenarioCfg.boundary_hint = false;
    scenarioCfg.hard_negative_hint = false;
    scenarioCfg.safe_probe_ratio_boost = 0.0;
    scenarioCfg.probe_landing_selected = false;
    scenarioCfg.probe_landing_probability = 0.0;
    scenarioCfg.probe_landing_reason = "none";
    scenarioCfg.target_case = "none";
    scenarioCfg.force_hover_abort_timeout = false;
    scenarioCfg.force_land_at_timeout = false;
    scenarioCfg.hover_timeout_sec = nan;
    scenarioCfg.hover_height_m = autosimRandRange(cfg.scenario.hover_height_min_m, cfg.scenario.hover_height_max_m);
    scenarioCfg.wind_profile_offset_sec = 0;
    if cfg.wind.enable
        [scenarioCfg.wind_speed, scenarioCfg.wind_dir] = autosimPickScenarioWind(cfg, scenarioId);
        scenarioCfg.wind_profile_offset_sec = autosimPickScenarioWindOffsetSec(cfg, scenarioId);
    else
        scenarioCfg.wind_speed = 0.0;
        scenarioCfg.wind_dir = 0.0;
    end
    scenarioCfg = autosimApplyValidationScenarioConfig(cfg, scenarioCfg, scenarioId);
end


