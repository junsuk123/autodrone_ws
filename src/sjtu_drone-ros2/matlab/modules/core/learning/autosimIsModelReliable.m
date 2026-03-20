function tf = autosimIsModelReliable(model, cfg)
    tf = false;
    if isempty(model)
        return;
    end

    if isfield(model, 'placeholder') && logical(model.placeholder)
        return;
    end

    if ~autosimModelFeatureSchemaMatches(model, cfg)
        return;
    end

    hasLegacyCounts = isfield(model, 'n_stable') && isfield(model, 'n_unstable');
    hasActionCounts = isfield(model, 'n_attempt_landing') && isfield(model, 'n_hold_landing');
    if ~isfield(model, 'n_train') || (~hasLegacyCounts && ~hasActionCounts)
        % Legacy model files may not carry training-count metadata.
        tf = autosimHasUsableModelParameters(model);
        return;
    end

    nTrain = double(model.n_train);
    if hasActionCounts
        nStable = double(model.n_attempt_landing);
        nUnstable = double(model.n_hold_landing);
    else
        nStable = double(model.n_stable);
        nUnstable = double(model.n_unstable);
    end
    if ~isfinite(nTrain) || ~isfinite(nStable) || ~isfinite(nUnstable)
        return;
    end

    minTrain = 1;
    minClass = 1;
    minMinorityRatio = 0.0;
    if isfield(cfg.agent, 'model_min_total_samples_for_use') && isfinite(cfg.agent.model_min_total_samples_for_use)
        minTrain = max(1, round(cfg.agent.model_min_total_samples_for_use));
    end
    if isfield(cfg.agent, 'model_min_class_samples_for_use') && isfinite(cfg.agent.model_min_class_samples_for_use)
        minClass = max(1, round(cfg.agent.model_min_class_samples_for_use));
    end
    if isfield(cfg.agent, 'model_minority_ratio_for_use') && isfinite(cfg.agent.model_minority_ratio_for_use)
        minMinorityRatio = max(0.0, min(0.5, cfg.agent.model_minority_ratio_for_use));
    end

    minorityRatio = min(nStable, nUnstable) / max(1, nTrain);
    tf = (nTrain >= minTrain) && (nStable >= minClass) && (nUnstable >= minClass) && (minorityRatio >= minMinorityRatio);
end


