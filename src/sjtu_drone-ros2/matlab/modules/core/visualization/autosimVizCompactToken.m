function out = autosimVizCompactToken(token)
    token = string(token);
    tokenNorm = autosimNormalizeActionLabel(token);
    if tokenNorm == "AttemptLanding"
        out = "attempt landing";
        return;
    elseif tokenNorm == "HoldLanding"
        out = "hold landing";
        return;
    end
    switch token
        case "conditional"
            out = "conditional";
        case "conflicting"
            out = "conflicting";
        case "gust_front"
            out = "gust front";
        case "persistent_strong"
            out = "persistent wind";
        case "turbulent_shift"
            out = "turbulent shift";
        case "steady_calm"
            out = "steady calm";
        case "steady_flow"
            out = "steady flow";
        case "converging"
            out = "converging";
        case "steady_offset"
            out = "steady offset";
        case "diverging"
            out = "diverging";
        case "locked_on"
            out = "locked on";
        case "drifting_lock"
            out = "drifting lock";
        case "intermittent_lock"
            out = "intermittent";
        case "easy_to_hold"
            out = "easy hold";
        case "corrective_hold"
            out = "corrective";
        case "hard_to_hold"
            out = "hard hold";
        case "monitoring"
            out = "monitoring";
        otherwise
            out = replace(token, "_", " ");
    end
end


