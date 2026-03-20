function out = autosimClassifyDecisionOutcome(gtSafeLabel, predDecision)
    gtSafe = autosimNormalizeActionLabel(gtSafeLabel) == "AttemptLanding";
    predLand = autosimNormalizeActionLabel(predDecision) == "AttemptLanding";

    if gtSafe && predLand
        out = "TP";
    elseif (~gtSafe) && predLand
        out = "FP";
    elseif gtSafe && (~predLand)
        out = "FN";
    else
        out = "TN";
    end
end


