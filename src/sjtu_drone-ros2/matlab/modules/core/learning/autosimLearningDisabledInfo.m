function info = autosimLearningDisabledInfo(scenarioId)
    info = struct();
    info.scenario_id = scenarioId;
    info.model_updated = false;
    info.n_train = 0;
    info.stable_ratio = 0.0;
    info.n_stable = 0;
    info.n_unstable = 0;
    info.skip_reason = "pipeline_train_disabled";
    info.model_path = "";
end


