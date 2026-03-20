function scenarioCfg = autosimApplyCurriculumTargetCase(cfg, scenarioCfg, datasetState, scenarioId)
    if ~isfield(cfg, 'curriculum') || ~isfield(cfg.curriculum, 'enable') || ~cfg.curriculum.enable
        return;
    end

    targetCase = autosimChooseCurriculumCase(cfg, datasetState, scenarioId);
    scenarioCfg.target_case = targetCase;

    timeoutSec = autosimClampNaN(cfg.curriculum.hover_timeout_sec, cfg.control.land_forced_timeout_sec);
    if ~isfinite(timeoutSec) || timeoutSec <= 0
        timeoutSec = 30.0;
    end
    scenarioCfg.hover_timeout_sec = timeoutSec;

    scenarioCfg.force_hover_abort_timeout = false;
    scenarioCfg.force_land_at_timeout = false;

    switch targetCase
        case "safe_land"
            scenarioCfg = autosimApplyCurriculumWindTarget(cfg, scenarioCfg, "safe", scenarioId);
            scenarioCfg.probe_landing_selected = false;

        case "safe_hover_timeout"
            scenarioCfg = autosimApplyCurriculumWindTarget(cfg, scenarioCfg, "safe", scenarioId);
            scenarioCfg.force_hover_abort_timeout = true;
            scenarioCfg.probe_landing_selected = false;

        case "unsafe_hover_timeout"
            scenarioCfg = autosimApplyCurriculumWindTarget(cfg, scenarioCfg, "unsafe", scenarioId);
            scenarioCfg.force_hover_abort_timeout = true;
            scenarioCfg.probe_landing_selected = false;
            scenarioCfg.hard_negative_hint = true;
            scenarioCfg.boundary_hint = true;
            scenarioCfg.gust_amp_scale = max(autosimClampNaN(scenarioCfg.gust_amp_scale, 1.0), 1.35);
            scenarioCfg.dir_osc_scale = max(autosimClampNaN(scenarioCfg.dir_osc_scale, 1.0), 1.20);

        case "unsafe_forced_land"
            scenarioCfg = autosimApplyCurriculumWindTarget(cfg, scenarioCfg, "unsafe", scenarioId);
            scenarioCfg.force_land_at_timeout = true;
            scenarioCfg.probe_landing_selected = false;
            scenarioCfg.hard_negative_hint = true;
            scenarioCfg.boundary_hint = true;
            scenarioCfg.gust_amp_scale = max(autosimClampNaN(scenarioCfg.gust_amp_scale, 1.0), 1.35);
            scenarioCfg.dir_osc_scale = max(autosimClampNaN(scenarioCfg.dir_osc_scale, 1.0), 1.20);

        otherwise
    end
end


