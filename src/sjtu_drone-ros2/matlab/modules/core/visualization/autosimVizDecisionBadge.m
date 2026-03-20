function [color, textOut] = autosimVizDecisionBadge(integration, inferTxt, feasibilityScore, predTxt)
    integration = autosimNormalizeActionLabel(integration);
    inferTxt = string(inferTxt);
    switch integration
        case "AttemptLanding"
            color = [0.72 0.89 0.74];
            titleTxt = "ATTEMPT_LANDING";
        case "HoldLanding"
            color = [0.94 0.76 0.76];
            titleTxt = "HOLD_LANDING";
        otherwise
            color = [0.95 0.90 0.70];
            titleTxt = "REASSESS";
    end
    textOut = sprintf('%s | %s | %.2f', titleTxt, inferTxt, feasibilityScore);
end


