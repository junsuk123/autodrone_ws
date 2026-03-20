function de = autosimEvaluateDecisionMetrics(inTbl)
    de = struct();
    de.tp = 0;
    de.fp = 0;
    de.fn = 0;
    de.tn = 0;
    de.n_valid = 0;
    de.n_safe = 0;
    de.n_unsafe = 0;
    de.accuracy = nan;
    de.precision = nan;
    de.recall = nan;
    de.specificity = nan;
    de.balanced_accuracy = nan;
    de.f1 = nan;
    de.unsafe_landing_rate = nan;
    de.risk_score = nan;
    de.n_excluded_intervention = 0;
    de.n_excluded_hover = 0;

    if isempty(inTbl)
        return;
    end

    if ismember('gt_safe', inTbl.Properties.VariableNames) && ismember('pred_land', inTbl.Properties.VariableNames)
        gtSafe = logical(inTbl.gt_safe);
        predLand = logical(inTbl.pred_land);
        if ismember('pred_hover', inTbl.Properties.VariableNames)
            de.n_excluded_hover = sum(logical(inTbl.pred_hover));
        end
        if ismember('intervention_case', inTbl.Properties.VariableNames)
            de.n_excluded_intervention = sum(logical(inTbl.intervention_case));
        end
        if ismember('valid', inTbl.Properties.VariableNames)
            valid = logical(inTbl.valid);
        else
            valid = true(height(inTbl), 1);
        end
    elseif ismember('label', inTbl.Properties.VariableNames)
        dt = autosimBuildDecisionTable(inTbl);
        gtSafe = dt.gt_safe;
        predLand = dt.pred_land;
        valid = dt.valid;
    else
        return;
    end

    de.tp = sum(predLand(valid) & gtSafe(valid));
    de.fp = sum(predLand(valid) & ~gtSafe(valid));
    de.fn = sum(~predLand(valid) & gtSafe(valid));
    de.tn = sum(~predLand(valid) & ~gtSafe(valid));
    de.n_valid = de.tp + de.fp + de.fn + de.tn;
    de.n_safe = de.tp + de.fn;
    de.n_unsafe = de.fp + de.tn;

    de.accuracy = autosimSafeDivide(de.tp + de.tn, de.n_valid);
    de.precision = autosimSafeDivide(de.tp, de.tp + de.fp);
    de.recall = autosimSafeDivide(de.tp, de.tp + de.fn);
    de.specificity = autosimSafeDivide(de.tn, de.tn + de.fp);
    validBalanced = [de.recall, de.specificity];
    validBalanced = validBalanced(isfinite(validBalanced));
    if ~isempty(validBalanced)
        de.balanced_accuracy = mean(validBalanced);
    end
    if isfinite(de.precision) && isfinite(de.recall) && (de.precision + de.recall) > 0
        de.f1 = 2.0 * de.precision * de.recall / (de.precision + de.recall);
    end
    de.unsafe_landing_rate = autosimSafeDivide(de.fp, de.fp + de.tn);
    de.risk_score = autosimSafeDivide(de.tp + de.tn, de.tp + de.tn + 2*de.fp + de.fn);
end


