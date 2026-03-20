function dTbl = autosimBuildDecisionTable(summaryTbl, decisionField)
    dTbl = table();
    if isempty(summaryTbl) || ~ismember('scenario_id', summaryTbl.Properties.VariableNames)
        return;
    end

    if nargin < 2 || strlength(string(decisionField)) == 0
        decisionField = "pred_decision";
    end
    decisionField = string(decisionField);

    n = height(summaryTbl);
    sid = summaryTbl.scenario_id;

    gtSafe = false(n, 1);
    gtValid = false(n, 1);
    if ismember('gt_safe_to_land', summaryTbl.Properties.VariableNames)
        gtLbl = autosimNormalizeActionLabel(summaryTbl.gt_safe_to_land);
        gtSafe = (gtLbl == "AttemptLanding");
        gtValid = (gtLbl == "AttemptLanding") | (gtLbl == "HoldLanding");
    elseif ismember('label', summaryTbl.Properties.VariableNames)
        lbl = autosimNormalizeActionLabel(summaryTbl.label);
        gtSafe = (lbl == "AttemptLanding");
        gtValid = (lbl == "AttemptLanding") | (lbl == "HoldLanding");
    elseif ismember('success', summaryTbl.Properties.VariableNames)
        gtSafe = logical(summaryTbl.success);
        gtValid = true(n, 1);
    end

    predLand = false(n, 1);
    predValid = false(n, 1);
    predHover = false(n, 1);
    if ismember(decisionField, string(summaryTbl.Properties.VariableNames))
        p = autosimNormalizeActionLabel(summaryTbl.(char(decisionField)));
        predLand = (p == "AttemptLanding");
        predHover = false(n, 1);
        predValid = (p == "AttemptLanding") | (p == "HoldLanding");
    elseif ismember('landing_cmd_time', summaryTbl.Properties.VariableNames)
        lct = summaryTbl.landing_cmd_time;
        predLand = isfinite(lct);
        predValid = true(n, 1);
    end

    interventionCase = false(n, 1);
    if ismember('target_case', summaryTbl.Properties.VariableNames)
        tc = string(summaryTbl.target_case);
        interventionCase = interventionCase | ...
            (tc == "safe_hover_timeout") | (tc == "unsafe_hover_timeout") | (tc == "unsafe_forced_land");
    end
    if ismember('action_source', summaryTbl.Properties.VariableNames)
        as = string(summaryTbl.action_source);
        interventionCase = interventionCase | (as == "timeout_hover_abort") | (as == "timeout_hover_hold") | (as == "timeout_forced_land");
    end

    dTbl.scenario_id = sid;
    dTbl.gt_safe = gtSafe;
    dTbl.pred_land = predLand;
    dTbl.pred_hover = predHover;
    dTbl.intervention_case = interventionCase;
    dTbl.valid = gtValid & predValid & ~interventionCase;
    dTbl.valid_raw = gtValid & predValid;
end


