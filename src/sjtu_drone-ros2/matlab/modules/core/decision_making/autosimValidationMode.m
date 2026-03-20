function mode = autosimValidationMode(cfg, scenarioId)
    mode = "boundary";
    if autosimIsModuleEnabled(cfg, 'learning_engine')
        try
            mode = autosim_learning_engine('validation_mode', cfg, scenarioId);
            return;
        catch
        end
    end
end


