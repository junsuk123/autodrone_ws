function scenarioCfg = autosimApplyValidationScenarioConfig(cfg, scenarioCfg, scenarioId)
    if ~autosimPipelineValidationEnabled(cfg)
        return;
    end
    if ~isfield(cfg, 'validation') || ~isfield(cfg.validation, 'enable') || ~cfg.validation.enable
        return;
    end

    profile = autosimGetKmaWindProfile(cfg);
    if autosimIsModuleEnabled(cfg, 'learning_engine')
        try
            scenarioCfg = autosim_learning_engine('apply_validation_scenario_config', cfg, scenarioCfg, scenarioId, profile);
            return;
        catch
        end
    end
end


