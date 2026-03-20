function [model, info] = autosimIncrementalTrainAndSave(cfg, results, modelPrev, scenarioId)
    tbl = autosimSummaryTable(results);
    if autosimIsModuleEnabled(cfg, 'learning_engine')
        try
            [model, info] = autosim_learning_engine('incremental_train_and_save', cfg, tbl, modelPrev, scenarioId);
            return;
        catch
        end
    end

    model = modelPrev;
    info = autosimLearningDisabledInfo(scenarioId);
    info.skip_reason = "learning_engine_unavailable";
    if ismember('gt_safe_to_land', tbl.Properties.VariableNames)
        yAll = autosimNormalizeActionLabel(tbl.gt_safe_to_land);
        valid = (yAll == "AttemptLanding") | (yAll == "HoldLanding");
        info.n_train = sum(valid);
        if any(valid)
            y = yAll(valid);
            info.n_stable = sum(y == "AttemptLanding");
            info.n_unstable = sum(y == "HoldLanding");
            info.n_attempt_landing = info.n_stable;
            info.n_hold_landing = info.n_unstable;
            info.stable_ratio = info.n_stable / max(1, info.n_train);
            info.attempt_landing_ratio = info.stable_ratio;
        end
    elseif ismember('label', tbl.Properties.VariableNames)
        yAll = autosimNormalizeActionLabel(tbl.label);
        valid = (yAll == "AttemptLanding") | (yAll == "HoldLanding");
        info.n_train = sum(valid);
        if any(valid)
            y = yAll(valid);
            info.n_stable = sum(y == "AttemptLanding");
            info.n_unstable = sum(y == "HoldLanding");
            info.n_attempt_landing = info.n_stable;
            info.n_hold_landing = info.n_unstable;
            info.stable_ratio = info.n_stable / max(1, info.n_train);
            info.attempt_landing_ratio = info.stable_ratio;
        end
    end
end


